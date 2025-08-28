#include "FreeRTOS.h"
#include "task.h"
#include <app_sm.h>
#include "jtag.h"
#include <adiv5.h>
#include "cmsis_edi.h"
#include <target.h>
#include <cmsis_gcc.h>
#include <string.h>
#include "cmsis_os.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "common.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define RX_FIFO_SIZE 256
#define TX_FIFO_SIZE 256

#define SYMBOL_BACKSPACE '\b'
#define SYMBOL_ESCAPE 0x1b
#define SYMBOL_ARROW 0x5b
#define SYMBOL_ARROW_UP    0x41 // 'A'
#define SYMBOL_ARROW_DOWN  0x42 // 'B'
#define SYMBOL_ARROW_RIGHT 0x43 // 'C'
#define SYMBOL_ARROW_LEFT  0x44 // 'D'

/*
 * STMCube c lib does not support uint64_t printing, so we have to split the
 * number to 2 by 32
 */
#define VALUE64_FMT "0x%08x%08x"
#define VALUE64_ARG(__v64) \
    (int)(((__v64) >> 32) & 0xffffffff), (int)((__v64) & 0xffffffff)


struct target t;

struct usb_stats {
  int processed_rx;
  int processed_tx;
  int popped_from_rx_fifo;
  int pushed_to_tx_fifo;
  int popped_from_rx_fifo_newline;
  int deinit_counter;
  int suspend_counter;
  int init_counter;
  bool is_usb_attached;
};

struct usb_stats ust = { 0 };

typedef enum {
  CMD_NONE                = 0,
  CMD_TARGET_STATUS       = 1,
  CMD_TARGET_INIT         = 2,
  CMD_TARGET_HALT         = 3,
  CMD_TARGET_RESUME       = 4,
  CMD_TARGET_SOFT_RESET   = 5,
  CMD_TARGET_MEM_READ_32  = 6,
  CMD_TARGET_MEM_WRITE_32 = 7,
  CMD_TARGET_MEM_READ     = 8,
  CMD_TARGET_REG_WRITE_64 = 9,
  CMD_TARGET_REG_READ_64  = 10,
  CMD_TARGET_DUMP_REGS    = 11,
  CMD_UNKNOWN
} cmd_t;

struct cmd {
  cmd_t cmd;
  uint32_t arg0;
  uint32_t arg1;
  uint32_t arg2;
  uint32_t arg3;
};

struct fifo {
  uint8_t *buf;
  int head;
  int tail;
  size_t sz;
};

static osMessageQueueId_t cmd_msg_queue;

static osMutexId_t msg_mutex;
static osMutexId_t tx_mutex;

static osThreadId_t thread_id_uart_rx;
static osThreadId_t thread_id_uart_tx;

static uint32_t task_uart_rx_buf[256];
static uint32_t task_uart_tx_buf[256];

static void task_fn_uart_rx(void *argument);
static void task_fn_uart_tx(void *argument);

static StaticTask_t task_uart_rx_cb;

static const osThreadAttr_t task_attrs_uart_rx = {
  .name = "task_uart_rx",
  .cb_mem = &task_uart_rx_cb,
  .cb_size = sizeof(task_uart_rx_cb),
  .stack_mem = &task_uart_rx_buf[0],
  .stack_size = sizeof(task_uart_rx_buf),
  .priority = (osPriority_t) osPriorityHigh,
};

static StaticTask_t task_uart_tx_cb;

static const osThreadAttr_t task_attrs_uart_tx = {
  .name = "task_uart_tx",
  .cb_mem = &task_uart_tx_cb,
  .cb_size = sizeof(task_uart_tx_cb),
  .stack_mem = &task_uart_tx_buf[0],
  .stack_size = sizeof(task_uart_tx_buf),
  .priority = (osPriority_t) osPriorityHigh,
};

static struct cmd queue_cmd_buf[2 * sizeof(uint16_t)];

static StaticQueue_t queue_cmd_cb;

static const osMessageQueueAttr_t queue_cmd_attrs = {
  .name = "queue_cmd",
  .cb_mem = &queue_cmd_cb,
  .cb_size = sizeof(queue_cmd_cb),
  .mq_mem = &queue_cmd_buf,
  .mq_size = sizeof(queue_cmd_buf)
};

#define SEMA_STATIC(__name) \
static StaticSemaphore_t __name ## _cb; \
static const osSemaphoreAttr_t __name ## _attr = { \
  .name = #__name, \
  .attr_bits = 0, \
  .cb_mem = &__name ## _cb, \
  .cb_size = sizeof(__name ## _cb) \
}; \
static osSemaphoreId_t __name;

#define SEMA_INIT(__name, __max, __init) \
  __name = osSemaphoreNew(__max, __init, &__name ## _attr)

SEMA_STATIC(sema_tx_fifo_non_empty);
SEMA_STATIC(sema_tx_fifo_not_full);
SEMA_STATIC(sema_rx_avail);

static uint8_t usb_rx_buf[RX_FIFO_SIZE];
static uint8_t usb_tx_buf[TX_FIFO_SIZE];

static uint8_t msg_buf[128];

static struct fifo uart_fifo_rx = {
  .buf = usb_rx_buf,
  .head = 0,
  .tail = 0,
  .sz = sizeof(usb_rx_buf)
};

static struct fifo uart_fifo_tx = {
  .buf = usb_tx_buf,
  .head = 0,
  .tail = 0,
  .sz = sizeof(usb_tx_buf)
};

static bool rx_fifo_ready = false;
static bool tx_fifo_ready = false;
static bool special_symbol_active = false;
static bool arrow_symbol_active = false;

static void sema_refill(osSemaphoreId_t s, uint32_t max_count)
{
  while (osSemaphoreGetCount(s) < max_count)
    osSemaphoreRelease(s);
}

static void sema_exhaust(osSemaphoreId_t s)
{
  while (osSemaphoreGetCount(s))
    osSemaphoreAcquire(s, osWaitForever);
}

static void fifo_reset(struct fifo *f, uint8_t *buf, size_t buf_sz)
{
  memset(buf, 0, buf_sz);
  f->buf = buf;
  f->sz = buf_sz;
  f->head = 0;
  f->tail = 0;
}

static bool fifo_push(struct fifo *f, uint8_t ch)
{
  int next_head = f->head + 1;
  if (next_head == f->sz)
    next_head = 0;

  if (next_head == f->tail)
    return false;

  f->buf[f->head++] = ch;
  if (f->head == f->sz)
    f->head = 0;

  return true;
}

extern void Error_Handler(void);

static inline void tx_fifo_push(uint8_t ch)
{
  if (!tx_fifo_ready)
    return;

  if (osSemaphoreAcquire(sema_tx_fifo_not_full, osWaitForever) != osOK)
    Error_Handler();

  portENTER_CRITICAL();
  if (!tx_fifo_ready) {
    portEXIT_CRITICAL();
    return;
  }

  if (!fifo_push(&uart_fifo_tx, ch))
    Error_Handler();

  ust.pushed_to_tx_fifo++;
  osSemaphoreRelease(sema_tx_fifo_non_empty);
  portEXIT_CRITICAL();
  osThreadYield();
}

static int uart_tx_fifo_push(const uint8_t *msg, size_t len)
{
  size_t i;
  osMutexAcquire(tx_mutex, osWaitForever);

  for (i = 0; i < len; ++i)
    tx_fifo_push(msg[i]);

  osMutexRelease(tx_mutex);
  return i;
}

#define msg_push(__msg, __l) uart_tx_fifo_push((const uint8_t *)__msg, __l)

static bool fifo_pop(struct fifo *f, uint8_t *ch)
{
  if (f->tail == f->head)
    return false;

  if (f->head != f->tail) {
    *ch = f->buf[f->tail++];
    if (f->tail == f->sz)
      f->tail = 0;
  }
  return true;
}

#if 0
static void on_new_symbols(const uint8_t *src, size_t len)
{
  char c;
  char buf[4];

  for (size_t i = 0; i < len && buf_idx < sizeof(buf); ++i) {
    c = src[i];
    snprintf(buf, sizeof(buf), "%02x ", (int)c);
    while (CDC_Transmit_FS((const uint8_t *)buf, 4) == USBD_BUSY);
    if (c == '\n') {
      while (CDC_Transmit_FS("hello", 5) == USBD_BUSY);
    }
    else
      buf[buf_idx++] = c;
  }
}
#endif

typedef enum {
    SM_IDLING = 0,
    SM_USB_TRANSMIT_DONE,
    SM_CMD_DONE,
    SM_USB_NEW_CMD_PENDING,
} app_sm_state_t;

app_sm_state_t appstate;

static void app_prompt(void)
{
  msg_push("\r\n>", 3);
}

void app_sm_init(void)
{
  struct cmd c;

  appstate = SM_IDLING;

  /* creation of queue_cmd */
  cmd_msg_queue = osMessageQueueNew(2, sizeof(struct cmd),
    &queue_cmd_attrs);

  SEMA_INIT(sema_rx_avail, sizeof(usb_rx_buf) - 1, 0);

  SEMA_INIT(sema_tx_fifo_non_empty,
    sizeof(usb_tx_buf) - 1,
    0);

  SEMA_INIT(sema_tx_fifo_not_full,
    sizeof(usb_tx_buf) - 1,
    sizeof(usb_tx_buf) - 1);

  fifo_reset(&uart_fifo_rx, usb_rx_buf, sizeof(usb_rx_buf));
  fifo_reset(&uart_fifo_tx, usb_tx_buf, sizeof(usb_tx_buf));

  msg_mutex = osMutexNew(NULL);
  tx_mutex = osMutexNew(NULL);

  thread_id_uart_rx = osThreadNew(task_fn_uart_rx, NULL, &task_attrs_uart_rx);
  thread_id_uart_tx = osThreadNew(task_fn_uart_tx, NULL, &task_attrs_uart_tx);

  c.cmd = CMD_NONE;
  // osMessageQueuePut(cmd_msg_queue, &c, 0, 0);
}

/* happens on reset */
void on_cdc_deinit_isr(void)
{
  ust.deinit_counter++;
}

void on_cdc_suspend_isr(void)
{
  ust.suspend_counter++;
  if (!ust.is_usb_attached)
    return;

  ust.is_usb_attached = false;
  rx_fifo_ready = false;
  tx_fifo_ready = false;
  fifo_reset(&uart_fifo_rx, usb_rx_buf, sizeof(usb_rx_buf));
  fifo_reset(&uart_fifo_tx, usb_tx_buf, sizeof(usb_tx_buf));
  osSemaphoreRelease(sema_rx_avail);
  osSemaphoreRelease(sema_tx_fifo_non_empty);
  osSemaphoreRelease(sema_tx_fifo_not_full);
}

void on_cdc_init_isr(void)
{
  ust.is_usb_attached = true;

  xTaskNotifyFromISR(thread_id_uart_tx, 1, eSetBits, NULL);
  xTaskNotifyFromISR(thread_id_uart_rx, 1, eSetBits, NULL);

  ust.init_counter++;
  osSemaphoreRelease(sema_rx_avail);
  osSemaphoreRelease(sema_tx_fifo_non_empty);
  osSemaphoreRelease(sema_tx_fifo_not_full);
}

void on_cdc_receive(const uint8_t *buf, size_t len)
{
  size_t i;
  if (!rx_fifo_ready)
    return;

  for (i = 0; i < len; ++i) {
    ust.processed_rx++;
    fifo_push(&uart_fifo_rx, buf[i]);
    osSemaphoreRelease(sema_rx_avail);
  }
}

static uint8_t cmdbuf[256];
int cmdbuf_cursor = 0;
static int cmdbuf_len = 0;

extern void uart_send_byte(uint8_t b);

extern bool uart_tx_is_ready(void);

static inline bool cmdbuf_parse_mrw(struct cmd *c, const char *p)
{
  uint64_t addr;
  c->cmd = CMD_TARGET_MEM_READ_32;
  addr = strtol(p, NULL, 0);
  c->arg0 = addr & 0xffffffff;
  c->arg1 = (addr >> 32) & 0xffffffff;
  return true;
}

static inline bool cmdbuf_parse_mr(struct cmd *c, const char *p)
{
  uint64_t addr;
  uint32_t size;
  c->cmd = CMD_TARGET_MEM_READ;
  addr = strtol(p, (char **)&p, 0);
  size = strtol(p, NULL, 0);
  c->arg0 = addr & 0xffffffff;
  c->arg1 = (addr >> 32) & 0xffffffff;
  c->arg2 = size;
  return true;
}

static inline bool cmdbuf_parse_mww(struct cmd *c, const char *p)
{
  uint64_t addr;

  c->cmd = CMD_TARGET_MEM_WRITE_32;
  addr = strtol(p, (char **)&p, 0);
  c->arg0 = addr & 0xffffffff;
  c->arg1 = (addr >> 32) & 0xffffffff;
  c->arg2 = strtol(p, NULL, 0);
  return true;
}

#define is_digit10(__c) (__c >= '0' && __c <= '9')
static inline bool is_valid_regname_symbol(char c)
{
  return is_digit10(c)
    || (c >= 'a' && c <= 'z')
    || (c >= 'A' && c <= 'Z')
    || c == '_';
}

static inline int parse_reg_name(const char **ptr)
{
  int n;
  int i0 = 0;
  int i = 0;
  const char *p = *ptr;
  char c;

  int reg_id = AARCH64_CORE_REG_UNKNOWN;

  while (1) {
    if (i0 > 24)
      goto out_err;

    c = p[i0];
    if (!c)
      goto out_err;

    if (c != ' ')
      break;

    i0++;
  }

  i = i0;
  /* parsing register name, staring from numeric is invalid */
  if (is_digit10(c))
    goto out_err;

  while(1) {
    if (!c)
      break;
    /* We want to leave space in reg for last '\0' */
    if (i - i0 > 24)
      goto out_err;

    if (c == ' ')
      break;

    if (!is_valid_regname_symbol(c))
      goto out_err;

    i++;
    c = p[i];
  }

  if (p[i0] == 'x') {
    int x_reg_index;
    for (n = i0 + 1; n < i; ++n) {
      if (!is_digit10(p[n]))
        goto out_err;
    }
    int num_digits = n - i0 - 1;
    if (!num_digits || num_digits > 2)
      goto out_err;

    x_reg_index = (p[i0 + 1] - '0');
    if (num_digits == 2)
      x_reg_index = x_reg_index * 10 + (p[i0 + 1] - '0');

    reg_id = AARCH64_CORE_REG_X0 + x_reg_index;
    goto out;
  }

  if (p[i0] == 'p' && p[i0 + 1] == 'c') {
    reg_id = AARCH64_CORE_REG_PC;
    goto out;
  }

  if (p[i0] == 's' && p[i0 + 1] == 'p') {
    reg_id = AARCH64_CORE_REG_SP;
    goto out;
  }

out:
  *ptr += i;

out_err:
  return reg_id;
}

static inline bool cmdbuf_parse_rw(struct cmd *cmd, const char *p)
{
  uint64_t v;
  int reg_id = parse_reg_name(&p);
  if (reg_id == AARCH64_CORE_REG_UNKNOWN)
    return false;

  v = strtoll(p, (char **)&p, 0);
  cmd->cmd = CMD_TARGET_REG_WRITE_64;
  cmd->arg0 = reg_id;
  cmd->arg1 = v & 0xffffffff;
  cmd->arg2 = (v >> 32) & 0xffffffff;
  return true;
}

static inline bool cmdbuf_parse_rr(struct cmd *cmd, const char *p)
{
  int reg_id = parse_reg_name(&p);
  if (reg_id == AARCH64_CORE_REG_UNKNOWN)
    return false;

  cmd->cmd = CMD_TARGET_REG_READ_64;
  cmd->arg0 = reg_id;
  return true;
}

static bool cmdbuf_parse(struct cmd *c)
{
  size_t n;
  const char *p = (const char *)cmdbuf;

  c->arg0 = 0;
  c->arg1 = 0;
  c->arg2 = 0;
  c->arg3 = 0;

  n = strnlen(p, sizeof(cmdbuf));
  if (!n) {
    c->cmd = CMD_NONE;
    return true;
  }

  if (!strncmp(p, "status", 6)) {
    c->cmd = CMD_TARGET_STATUS;
    return true;
  }
  if (!strncmp(p, "dumpregs", 8)) {
    c->cmd = CMD_TARGET_DUMP_REGS;
    return true;
  }
  if (!strncmp(p, "init", 4)) {
    c->cmd = CMD_TARGET_INIT;
    return true;
  }
  else if (!strncmp(p, "halt", 4)) {
    c->cmd = CMD_TARGET_HALT;
    return true;
  }
  else if (!strncmp(p, "resume", 6)) {
    c->cmd = CMD_TARGET_RESUME;
    return true;
  }
  else if (!strncmp(p, "srst", 4)) {
    c->cmd = CMD_TARGET_SOFT_RESET;
    return true;
  }
  else if (!strncmp(p, "mrw ", 4)) {
    return cmdbuf_parse_mrw(c, p + 4);
  }
  else if (!strncmp(p, "mrd ", 4)) {
    return cmdbuf_parse_mr(c, p + 4);
  }
  else if (!strncmp(p, "mww ", 4)) {
    return cmdbuf_parse_mww(c, p + 4);
  }
  else if (!strncmp(p, "rw ", 3)) {
    return cmdbuf_parse_rw(c, p + 3);
  }
  else if (!strncmp(p, "rr ", 3)) {
    return cmdbuf_parse_rr(c, p + 3);
  }

  return false;
}

static void msg(const char *fmt, ...)
{
  int n;
  va_list vl;
  va_start(vl, fmt);

  osMutexAcquire(msg_mutex, osWaitForever);
  n = vsnprintf((char *)msg_buf, sizeof(msg_buf), fmt, vl);
  uart_tx_fifo_push((const uint8_t *)msg_buf, n);
  osMutexRelease(msg_mutex);
  va_end(vl);
}

static void rx_on_new_char(char c)
{
  ust.popped_from_rx_fifo++;

  if (special_symbol_active) {
    if (!arrow_symbol_active) {
      if (c == SYMBOL_ARROW) {
        arrow_symbol_active = true;
        return;
      }
    } else {
      if (c == SYMBOL_ARROW_LEFT) {
        if (cmdbuf_cursor) {
          cmdbuf_cursor--;

          tx_fifo_push(SYMBOL_ESCAPE);
          tx_fifo_push(SYMBOL_ARROW);
          tx_fifo_push(SYMBOL_ARROW_LEFT);
        }
      }
      else if (c == SYMBOL_ARROW_RIGHT) {
        if (cmdbuf_cursor < cmdbuf_len) {
          cmdbuf_cursor++;

          tx_fifo_push(SYMBOL_ESCAPE);
          tx_fifo_push(SYMBOL_ARROW);
          tx_fifo_push(SYMBOL_ARROW_RIGHT);
        }
      }
      else if (c == SYMBOL_ARROW_UP) {
      }
      else if (c == SYMBOL_ARROW_DOWN) {
      }
      arrow_symbol_active = false;
      special_symbol_active = false;
      return;
    }
  }

  if (c == SYMBOL_BACKSPACE) {
    if (cmdbuf_cursor) {
      cmdbuf_cursor--;
      msg_push("\b \b", 3);
    }
  }
  else if (c == SYMBOL_ESCAPE) {
    special_symbol_active = true;
    return;
  }

  else if (c == '\r') {
  }
  else if (c == '\n') {
    ust.popped_from_rx_fifo_newline++;
    struct cmd cmd;
    bool parse_success;

    if (!cmdbuf_cursor) {
      cmd.cmd = CMD_NONE;
      osMessageQueuePut(cmd_msg_queue, &cmd, 0, 0);
      return;
    }

    msg_push("\r\n", 2);
    cmdbuf[cmdbuf_cursor] = 0;
    parse_success = cmdbuf_parse(&cmd);

    cmdbuf_cursor = 0;
    cmdbuf_len = 0;

    if (!parse_success) {
      msg("unknown command ");
      msg((const char *)cmdbuf);
      msg(">\r\n");
      return;
    }

    osMessageQueuePut(cmd_msg_queue, &cmd, 0, 0);
  }
  else {
    cmdbuf[cmdbuf_cursor++] = c;
    tx_fifo_push(c);
    if (cmdbuf_cursor > cmdbuf_len)
      cmdbuf_len = cmdbuf_cursor;
  }
}

static void app_uart_rx_init(void)
{
  cmdbuf_len = 0;
  cmdbuf_cursor = 0;
  arrow_symbol_active = false;
  special_symbol_active = false;
  special_symbol_active = false;
  sema_exhaust(sema_rx_avail);
  rx_fifo_ready = true;
}

static inline bool uart_is_up(void)
{
  uint32_t flags = 0;
  if (!ust.is_usb_attached) {
    xTaskNotifyWait(0, 0xffffffffUL, &flags, portMAX_DELAY);
    if (flags != 1) {
      while(1);
    }
    xTaskNotify(thread_id_uart_tx, 2, eSetBits);
    xTaskNotifyWait(0, 0xffffffffUL, &flags, portMAX_DELAY);
    if (flags != 2) {
      while(1);
    }
  }

  if (rx_fifo_ready)
    return true;

  app_uart_rx_init();
  return true;
}

static void app_handle_uart_rx(void)
{
  uint8_t c;

  if (!uart_is_up())
    return;

  osSemaphoreAcquire(sema_rx_avail, osWaitForever);

  portENTER_CRITICAL();
  if (!rx_fifo_ready) {
    portEXIT_CRITICAL();
    return;
  }

  if (!fifo_pop(&uart_fifo_rx, &c))
    Error_Handler();

  portEXIT_CRITICAL();
  rx_on_new_char(c);
}

void task_fn_uart_rx(void *argument)
{
  while(1) {
    app_handle_uart_rx();
    osThreadYield();
  }
}

void on_cdc_transmit_done(void)
{
}

static void app_handle_uart_tx(void)
{
  uint8_t c;
  uint32_t total_flags = 0;
  uint32_t flags;

  if (!ust.is_usb_attached) {
    while (total_flags != 3) {
      xTaskNotifyWait(0, 0xffffffffUL, &flags, portMAX_DELAY);
      total_flags |= flags;
    }

    sema_refill(sema_tx_fifo_not_full, sizeof(usb_tx_buf) - 1);
    sema_exhaust(sema_tx_fifo_non_empty);
    tx_fifo_ready = true;
    xTaskNotify(thread_id_uart_rx, 2, eSetBits);
    osThreadYield();
  }

  if (!uart_tx_is_ready())
    return;

  osSemaphoreAcquire(sema_tx_fifo_non_empty, osWaitForever);
  portENTER_CRITICAL();
  if (!tx_fifo_ready) {
    portEXIT_CRITICAL();
    return;
  }

  if (!fifo_pop(&uart_fifo_tx, &c))
    Error_Handler();

  ust.processed_tx++;
  osSemaphoreRelease(sema_tx_fifo_not_full);
  portEXIT_CRITICAL();
  uart_send_byte(c);
}

void task_fn_uart_tx(void *argument)
{
  while(1)
    app_handle_uart_tx();
}

void Error_Handler(void);

#define CHECK_ATTACHED() \
  if (!t->attached) { \
    msg("error: not attached\r\n"); \
    return; \
  }

static void app_process_mem_read_32(struct target *t, struct cmd *c)
{
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;
  uint32_t value = 0;

  CHECK_ATTACHED();

  if (target_mem_read_32(t, addr, &value))
    msg("0x%08x\r\n", value);
  else
    msg("mem read 32 failed\r\n");
}

static void app_process_mem_read(struct target *t, struct cmd *c)
{
  uint32_t value = 0;
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;
  int num_reads = (c->arg2 + 3) / 4;
  CHECK_ATTACHED();

  if (!target_mem_read_fast_start(t, addr)) {
    msg("error\r\n");
    return;
  }

  for (int i = 0; i < num_reads; ++i) {
    if (!target_mem_read_fast_next(t, &value)) {
      msg("error\r\n");
      break;
    }
    msg("0x%08x\r\n", value);
  }

  if (!target_mem_read_fast_stop(t)) {
    msg("error\r\n");
    return;
  }

  msg("done\r\n");
}


static void app_process_mem_write_32(struct target *t, struct cmd *c)
{
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;

  CHECK_ATTACHED();

  if (target_mem_write_32(t, addr, c->arg2))
    msg("done\r\n");
  else
    msg("mem write 32 failed\r\n");
}

static void app_process_reg_write_64(struct target *t, struct cmd *c)
{
  uint64_t regvalue = (((uint64_t)(c->arg2)) << 32) | c->arg1;

  CHECK_ATTACHED();

  if (target_reg_write_64(t, c->arg0, regvalue))
    msg("done\r\n");
  else
    msg("failed\r\n");
}

static void app_process_reg_read_64(struct target *t, struct cmd *c)
{
  uint64_t value = 0;
  CHECK_ATTACHED();

  if (target_reg_read_64(t, c->arg0, &value))
    msg("done:" VALUE64_FMT "\r\n", VALUE64_ARG(value));
  else
    msg("failed\r\n");
}

void app_process_dump_regs(struct target *t)
{
  int i, j;
  struct target_core *c;
  struct aarch64_context *core_ctx;

  CHECK_ATTACHED();

  for (i = 0; i < ARRAY_SIZE(t->core); ++i) {
    c = &t->core[i];
    if (!c->halted)
      continue;
    core_ctx = &c->a64.ctx;
    for (j = 0; j < ARRAY_SIZE(core_ctx->x0_30); ++j) {
      msg("%d,x%d," VALUE64_FMT "\r\n", i, j, VALUE64_ARG(core_ctx->x0_30[j]));
    }

    msg("%d,sp," VALUE64_FMT "\r\n", i, VALUE64_ARG(core_ctx->sp));
    msg("%d,pc," VALUE64_FMT "\r\n", i, VALUE64_ARG(core_ctx->pc));
  }
  msg("done\r\n");
}

#define msgbuf_push(__ptr, __s) \
  do { \
    strcpy(__ptr, __s); \
    __ptr += sizeof(__s) - 1; \
  } while(0)

static void app_process_status(const struct target *t)
{
  char *p = (char *)msg_buf;
  osMutexAcquire(msg_mutex, osWaitForever);
  msgbuf_push(p, "status: ");

  if (!t->attached) {
    msgbuf_push(p, "not attached\r\n");
    goto out;
  }

  msgbuf_push(p, "attached, ");
  if (target_is_halted(t))
    msgbuf_push(p, "halted\r\n");
  else
    msgbuf_push(p, "running\r\n");

out:
  *p = 0;
  uart_tx_fifo_push(msg_buf, p - (char *)msg_buf);
  osMutexRelease(msg_mutex);
}

void app_sm_process_next_cmd(void)
{
  osStatus_t status;
  struct cmd cmd;
  uint8_t prio;
  bool success;

  status = osMessageQueueGet(cmd_msg_queue, &cmd, &prio, osWaitForever);

  if (status != osOK)
    Error_Handler();

  if (!ust.is_usb_attached) {
    osMessageQueueReset(cmd_msg_queue);
    return;
  }

  switch(cmd.cmd) {
    case CMD_NONE:
      break;
    case CMD_TARGET_STATUS:
      app_process_status(&t);
      break;
    case CMD_TARGET_INIT:
      target_init(&t);
      msg("target initialized: %08x\r\n", t.idcode);
      break;
    case CMD_TARGET_HALT:
      if (!t.attached) {
        msg("target not attached, can not halt\n");
      }
      else {
        success = target_halt(&t);
        msg("target halt status: %d\r\n", success ? 1 : 0);
      }
      break;
    case CMD_TARGET_RESUME:
      if (target_is_halted(&t)) {
        success = target_resume(&t);
        msg("target resume status: %d\r\n", success ? 1 : 0);
      } else {
        msg("target not halted.\r\n");
      }
      break;
    case CMD_TARGET_SOFT_RESET:
      success = target_soft_reset(&t);
      msg("target soft reset status: %d\r\n", success ? 1 : 0);
      break;
    case CMD_TARGET_MEM_READ_32:
      app_process_mem_read_32(&t, &cmd);
      break;
    case CMD_TARGET_MEM_WRITE_32:
      app_process_mem_write_32(&t, &cmd);
      break;
    case CMD_TARGET_REG_WRITE_64:
      app_process_reg_write_64(&t, &cmd);
      break;
    case CMD_TARGET_REG_READ_64:
      app_process_reg_read_64(&t, &cmd);
      break;
    case CMD_TARGET_MEM_READ:
      app_process_mem_read(&t, &cmd);
      break;
    case CMD_TARGET_DUMP_REGS:
      app_process_dump_regs(&t);
      break;
    default:
      break;
  }
  app_prompt();
}
