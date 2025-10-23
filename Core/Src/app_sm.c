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
#include <errno.h>
#include "common.h"
#include "cmd.h"

#define RX_FIFO_SIZE 256
#define TX_FIFO_SIZE 256

#define SYMBOL_BACKSPACE '\b'
#define SYMBOL_BACKSPACE2 0x7f
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

static void cmd_done(int ret)
{
  msg("done %d\r\n>", ret);
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

  if (c == SYMBOL_BACKSPACE || c == SYMBOL_BACKSPACE2) {
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
      msg("\r\n");
      cmd_done(-EINVAL);
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
    msg("error: not attached"); \
    return -EPIPE; \
  }

static void print_value(uint64_t value, mem_access_size_t access_size)
{
  if (access_size == MEM_ACCESS_SIZE_32)
    msg("0x%08x\r\n", (uint32_t)value);
  else
    msg(VALUE64_FMT "\r\n", VALUE64_ARG(value));
}

static int app_process_mem_access(struct target *t, struct cmd *c)
{
  uint64_t value;
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;

  CHECK_ATTACHED();

  if (c->access_size != MEM_ACCESS_SIZE_32
    && c->access_size != MEM_ACCESS_SIZE_64) {
    msg("error: access size not supported\r\n");
    return -ENOTSUP;
  }

  if (c->is_write) {
    if (c->access_size == MEM_ACCESS_SIZE_64)
      value = (((uint64_t)(c->arg3)) << 32) | c->arg2;
    else
      value = c->arg2;

    return target_mem_write(t, c->access_size, addr, value);
  }
  return target_mem_read(t, c->access_size, addr, c->count, print_value);
}

static int app_process_reg_access(struct target *t, struct cmd *c)
{
  int ret;
  bool direct;
  bool sync;

  uint64_t value;

  CHECK_ATTACHED();

  if (c->is_write) {
    sync = (bool)c->arg3;
    value = (((uint64_t)(c->arg2)) << 32) | c->arg1;
    ret = target_reg_write_64(t, c->arg0, value, sync);
  } else {
    direct = (bool)c->arg3;
    ret = target_reg_read_64(t, c->arg0, &value, direct);
    if (!ret)
      print_value(value, MEM_ACCESS_SIZE_64);
  }
  return ret;
}

static int app_process_exec(struct target *t, struct cmd *c)
{
  CHECK_ATTACHED();
  return target_exec(t, &c->arg0, 1);
}

static int app_process_breakpoint(struct target *t, struct cmd *c)
{
  const uint64_t value = (((uint64_t)(c->arg1)) << 32) | c->arg0;
  const bool hardware = c->arg2;
  const bool remove = c->arg3;

  CHECK_ATTACHED();
  return target_breakpoint(t, remove, hardware, value);
}

struct dump_regs_iter {
  int core_idx;
};

static void dump_regs_iter_cb(const char *name, uint64_t value, void *arg)
{
  struct dump_regs_iter *iter = arg;
  msg("%d,%s," VALUE64_FMT "\r\n", iter->core_idx, name, VALUE64_ARG(value));
}

static int app_process_dump_regs(struct target *t)
{
  int ret;
  int core_idx;
  struct target_core *c;
  struct dump_regs_iter iter;

  CHECK_ATTACHED();

  for (core_idx = 0; core_idx < ARRAY_SIZE(t->core); ++core_idx) {
    c = &t->core[core_idx];
    if (!c->halted)
      continue;

    iter.core_idx = core_idx;
    ret = target_iter_regs(t, core_idx, dump_regs_iter_cb, &iter);
    if (ret)
      return ret;
  }

  return 0;
}

#define msgbuf_push(__ptr, __s) \
  do { \
    strcpy(__ptr, __s); \
    __ptr += sizeof(__s) - 1; \
  } while(0)

static int app_process_status(const struct target *t)
{
  int ret;
  char *p = (char *)msg_buf;
  osMutexAcquire(msg_mutex, osWaitForever);
  msgbuf_push(p, "status: ");

  if (!t->attached) {
    msgbuf_push(p, "not attached\r\n");
    ret = -1;
    goto out;
  }

  msgbuf_push(p, "attached, ");
  if (target_is_halted(t))
    msgbuf_push(p, "halted\r\n");
  else
    msgbuf_push(p, "running\r\n");

  ret = 0;
out:
  *p = 0;
  uart_tx_fifo_push(msg_buf, p - (char *)msg_buf);
  osMutexRelease(msg_mutex);
  return ret;
}

void app_sm_process_next_cmd(void)
{
  int ret;
  osStatus_t status;
  struct cmd cmd;
  uint8_t prio;

  status = osMessageQueueGet(cmd_msg_queue, &cmd, &prio, osWaitForever);

  if (status != osOK)
    Error_Handler();

  if (!ust.is_usb_attached) {
    osMessageQueueReset(cmd_msg_queue);
    return;
  }

  switch(cmd.cmd) {
    case CMD_NONE:
      ret = 0;
      break;
    case CMD_TARGET_STATUS:
      ret = app_process_status(&t);
      break;
    case CMD_TARGET_INIT:
      ret = target_init(&t);
      msg("target initialized: %08x\r\n", t.idcode);
      break;
    case CMD_TARGET_HALT:
      if (!t.attached) {
        msg("target not attached, can not halt\r\n");
        ret = -EINVAL;
      }
      else
        ret = target_halt(&t);
      break;
    case CMD_TARGET_RUNNING_CHECK_HALTED:
      if (!t.attached) {
        msg("not attached\r\n");
        ret = -EINVAL;
      }
      else if (target_is_halted(&t)) {
        const char *halt_reason = NULL;
        target_get_halt_reason(&t, &halt_reason);
        msg("halt_reason: %s\r\n", halt_reason);
        ret = 0;
      } else {
        ret = target_check_halted(&t);
      }
      break;
    case CMD_TARGET_RESUME:
      if (target_is_halted(&t))
        ret = target_resume(&t);
      else {
        msg("target not halted\r\n");
        ret = -EINVAL;
      }
      break;
    case CMD_TARGET_STEP:
      if (target_is_halted(&t))
        ret = target_step(&t);
      else {
        msg("target not halted\r\n");
        ret = -EINVAL;
      }
      break;
    case CMD_TARGET_SOFT_RESET:
      ret = target_soft_reset(&t);
      break;
    case CMD_TARGET_MEM_ACCESS:
      ret = app_process_mem_access(&t, &cmd);
      break;
    case CMD_TARGET_REG_ACCESS:
      ret = app_process_reg_access(&t, &cmd);
      break;
    case CMD_TARGET_BREAKPOINT:
      ret = app_process_breakpoint(&t, &cmd);
      break;
    case CMD_TARGET_DUMP_REGS:
      ret = app_process_dump_regs(&t);
      break;
    case CMD_TARGET_EXEC:
      ret = app_process_exec(&t, &cmd);
      break;
    default:
      ret = -EINVAL;
      break;
  }

  cmd_done(ret);
}
