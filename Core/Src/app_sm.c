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

#define CHRCMP2(__str, __c0, __c1) \
  ((__str)[0] == (__c0) && (__str)[1] == (__c1))

#define CHRCMP3(__str, __c0, __c1, __c2) \
  ((__str)[0] == (__c0) && (__str)[1] == (__c1) && (__str)[2] == (__c2))


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
  CMD_TARGET_MEM_ACCESS   = 6,
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
  uint32_t count;
  mem_access_size_t access_size;
  bool is_write;
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

static struct cmd queue_cmd_buf[12];

static StaticQueue_t queue_cmd_cb;

static const osMessageQueueAttr_t queue_cmd_attrs = {
  .name = "queue_cmd",
  .cb_mem = &queue_cmd_cb,
  .cb_size = sizeof(queue_cmd_cb),
  .mq_mem = queue_cmd_buf,
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

static char cmdbuf[256];
int cmdbuf_cursor = 0;
static int cmdbuf_len = 0;

extern void uart_send_byte(uint8_t b);

extern bool uart_tx_is_ready(void);

#define is_digit10(__c) (__c >= '0' && __c <= '9')

#define SKIP_SPACES(__ptr, __end) \
  for (; __ptr != __end && *__ptr && *__ptr == ' '; ++__ptr);

static inline int parse_reg_name(bool is_write, const char **ptr,
  const char *end)
{
  const char *p = *ptr;
  int reg_idx;
  int reg_id;

  SKIP_SPACES(p, end);

  if (p == end || !*p)
    return AARCH64_CORE_REG_UNKNOWN;

  /* parsing register name, staring from numeric is invalid */
  if (is_digit10(*p))
    return AARCH64_CORE_REG_UNKNOWN;

  if (p == end)
    return AARCH64_CORE_REG_UNKNOWN;

  if ((!is_write && end - p == 2) || (is_write && end - p > 3 && p[2] == ' ')) {
    if (CHRCMP2(p, 'p', 'c')) {
      reg_id = AARCH64_CORE_REG_PC;
      p += 2;
      goto out;
    }
    if (CHRCMP2(p, 's', 'p')) {
      reg_id = AARCH64_CORE_REG_SP;
      p += 2;
      goto out;
    }
    return AARCH64_CORE_REG_UNKNOWN;
  }

  if (*p != 'x')
    return AARCH64_CORE_REG_UNKNOWN;

  p++;
  if (!is_digit10(*p))
    return AARCH64_CORE_REG_UNKNOWN;

  reg_idx = *p - '0';
  p++;

  if (p != end && is_digit10(*p)) {
    reg_idx = reg_idx * 10 + *p - '0';
    p++;
  }

  /* I don't expect 3 digit register names */
  if (p != end && is_digit10(*p))
    return AARCH64_CORE_REG_UNKNOWN;

  reg_id = AARCH64_CORE_REG_X0 + reg_idx;

out:
  *ptr = p;
  return reg_id;
}

static inline bool cmdline_parse_read_write_cmd(const char *l,
  const char *end, struct cmd *c)
{
  /*
   * Memory read / write:
   * access size: r8, r16, r32, r64
   * number of elements: r32/3
   * + address argument: r32 0x00000000, r64/8 0x00000000
   * + write arg: w32 0x00000000 0xffffffff, w64 0x00000000 0xffff111100002222
   * Register read / write:
   * rr register_name
   * rw register_name 0xffff11
   */
  int reg_id;
  const char *old_l;
  bool is_write;
  uint64_t addr;
  uint64_t value;
  int count;
  mem_access_size_t access_size;

  /* smallest line is 5 chars rr X */
  if (end - l < 4)
    return false;

  if (CHRCMP3(l, 'r', 'r', ' ') || CHRCMP3(l, 'r', 'w', ' ')) {
    /* Register read / write */

    is_write = l[1] == 'r';
    l += 3;
    reg_id = parse_reg_name(is_write, &l, end);
    if (reg_id == AARCH64_CORE_REG_UNKNOWN)
      return false;

    SKIP_SPACES(l, end);

    if (is_write) {
      old_l = l;
      value = strtol(l, (char **)&l, 0);
      if (l == old_l)
        return false;
      SKIP_SPACES(l, end);
    }

    if (l != end)
      return false;

    c->cmd = is_write ? CMD_TARGET_REG_WRITE_64 : CMD_TARGET_REG_READ_64;
    c->arg0 = reg_id;
    if (is_write) {
      c->arg1 = value & 0xffffffff;
      c->arg2 = (value >> 32) & 0xffffffff;
    }
    return true;
  }

  /* Should be mem read or write */
  if (l[0] != 'r' && l[0] != 'w')
    return false;

  is_write = l[0] == 'w';
  l++;

  if (end - l < 2)
    return false;

  if (l[0] == '8') {
    access_size = MEM_ACCESS_SIZE_8;
    l++;
    goto parse_count;
  }

  if (end - l < 3)
    return false;

  if (CHRCMP2(l, '1', '6'))
    access_size = MEM_ACCESS_SIZE_16;
  else if (CHRCMP2(l, '3', '2'))
    access_size = MEM_ACCESS_SIZE_32;
  else if (CHRCMP2(l, '6', '4'))
    access_size = MEM_ACCESS_SIZE_64;
  else
    return false;
  l += 2;

parse_count:
  count = 1;
  if (l[0] == '/') {
    l++;
    old_l = l;
    count = strtol(l, (char **)&l, 0);
    if (l == old_l)
      return false;
  }

  if (l[0] != ' ')
    return false;

  SKIP_SPACES(l, end);
  old_l = l;
  addr = strtol(l, (char **)&l, 0);
  if (l == old_l)
    return false;

  if (is_write) {
    if (l[0] != ' ')
      return false;
    SKIP_SPACES(l, end);
    old_l = l;
    value = strtol(l, (char **)&l, 0);
    if (l == old_l)
      return false;
  }

  c->arg0 = addr & 0xffffffff;
  c->arg1 = (addr >> 32) & 0xffffffff;
  c->cmd = CMD_TARGET_MEM_ACCESS;
  c->is_write = is_write;
  c->access_size = access_size;
  c->count = count;
  if (is_write) {
    c->arg2 = value & 0xffffffff;
    c->arg3 = (value >> 32) & 0xffffffff;
  }
  return true;
}

static bool cmdbuf_parse(struct cmd *c, const char *buf, const char *end)
{
  size_t n;
  const char *p = buf;

  c->arg0 = 0;
  c->arg1 = 0;
  c->arg2 = 0;
  c->arg3 = 0;

  n = strnlen(p, MIN(sizeof(cmdbuf), end - buf));
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
  else if (cmdline_parse_read_write_cmd(buf, end, c))
    return true;

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
    parse_success = cmdbuf_parse(&cmd, cmdbuf, cmdbuf + cmdbuf_cursor);

    cmdbuf_cursor = 0;
    cmdbuf_len = 0;

    if (!parse_success) {
      msg("unknown command ");
      msg(cmdbuf);
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

static void print_value(uint64_t value, mem_access_size_t access_size)
{
  if (access_size == MEM_ACCESS_SIZE_32)
    msg("0x%08x\r\n", (uint32_t)value);
  else
    msg("0x%016x\r\n", value);
}

static void app_process_mem_access(struct target *t, struct cmd *c)
{
  int ret;

  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;

  CHECK_ATTACHED();

  if (c->access_size != MEM_ACCESS_SIZE_32
    && c->access_size != MEM_ACCESS_SIZE_64) {
    msg("error: access size not supported\r\n");
    return;
  }

  if (c->is_write) {
    uint64_t regvalue;
    if (c->access_size == MEM_ACCESS_SIZE_64)
      regvalue = (((uint64_t)(c->arg2)) << 32) | c->arg1;
    else
      regvalue = c->arg2;

    ret = target_mem_write(t, c->access_size, addr, regvalue);
  }
  else
    ret = target_mem_read(t, c->access_size, addr, c->count, print_value);

  if (ret)
    msg("error: %d", ret);
  else
    msg("done");
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
    case CMD_TARGET_MEM_ACCESS:
      app_process_mem_access(&t, &cmd);
      break;
    case CMD_TARGET_DUMP_REGS:
      app_process_dump_regs(&t);
      break;
    default:
      break;
  }
  app_prompt();
}
