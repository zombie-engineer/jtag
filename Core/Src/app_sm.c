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

struct target t;

typedef enum {
  CMD_NONE                = 0,
  CMD_TARGET_STATUS       = 1,
  CMD_TARGET_INIT         = 2,
  CMD_TARGET_HALT         = 3,
  CMD_TARGET_RESUME       = 4,
  CMD_TARGET_SOFT_RESET   = 5,
  CMD_TARGET_MEM_READ_32  = 6,
  CMD_TARGET_MEM_WRITE_32 = 7,
  CMD_TARGET_DUMP_REGS    = 8,
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

/* Definitions for queue_cmd */
static osMessageQueueId_t cmd_queue_handle;

static osMutexId_t msg_mutex;
static osMutexId_t tx_mutex;

/* Definitions for task_uart_rx */
osThreadId_t task_uart_rxHandle;

uint32_t rx_buf_stopper;
uint32_t task_uart_rx_buf[ 128 ];
uint32_t rx_buf_stopper1;

typedef StaticTask_t osStaticThreadDef_t;
void task_fn_uart_rx(void *argument);
void task_fn_uart_tx(void *argument);
osStaticThreadDef_t task_uart_rx_cb;
const osThreadAttr_t task_uart_rx_attributes = {
  .name = "task_uart_rx",
  .cb_mem = &task_uart_rx_cb,
  .cb_size = sizeof(task_uart_rx_cb),
  .stack_mem = &task_uart_rx_buf[0],
  .stack_size = sizeof(task_uart_rx_buf),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for task_uart_tx */
osThreadId_t task_uart_txHandle;
uint32_t task_uart_tx_buf[ 128 ];
osStaticThreadDef_t task_uart_tx_cb;
const osThreadAttr_t task_uart_tx_attributes = {
  .name = "task_uart_tx",
  .cb_mem = &task_uart_tx_cb,
  .cb_size = sizeof(task_uart_tx_cb),
  .stack_mem = &task_uart_tx_buf[0],
  .stack_size = sizeof(task_uart_tx_buf),
  .priority = (osPriority_t) osPriorityLow,
};

static struct cmd queue_cmd_buf[2 * sizeof(uint16_t)];

static StaticQueue_t queue_cmd_cb;
static const osMessageQueueAttr_t queue_cmd_attributes = {
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

SEMA_STATIC(tx_done_sema);
SEMA_STATIC(tx_avail_sema);
SEMA_STATIC(tx_free_sema);
SEMA_STATIC(rx_sema);

static uint8_t usb_rx_buf[RX_FIFO_SIZE];
static uint8_t usb_tx_buf[TX_FIFO_SIZE];

static char msg_buf[128];

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

static inline bool tx_fifo_push(uint8_t ch)
{
  bool success;

  if (osSemaphoreAcquire(tx_free_sema, osWaitForever) != osOK)
    Error_Handler();

  portENTER_CRITICAL();
  success = fifo_push(&uart_fifo_tx, ch);
  portEXIT_CRITICAL();
  if (success) {
    if (osSemaphoreRelease(tx_avail_sema) != osOK)
      Error_Handler();
  }
  return success;
}

static int uart_tx_fifo_push(const uint8_t *msg, size_t len)
{
  size_t i;
  osMutexAcquire(tx_mutex, osWaitForever);

  for (i = 0; i < len; ++i) {
    while(1) {
      if (tx_fifo_push(msg[i]))
        break;
    }
  }
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
  cmd_queue_handle = osMessageQueueNew (2, sizeof(struct cmd),
    &queue_cmd_attributes);

  SEMA_INIT(rx_sema      , sizeof(usb_rx_buf), 0);
  SEMA_INIT(tx_avail_sema, sizeof(usb_tx_buf), 0);
  SEMA_INIT(tx_free_sema , sizeof(usb_tx_buf), sizeof(usb_tx_buf));
  SEMA_INIT(tx_done_sema , 1, 0);

  msg_mutex = osMutexNew(NULL);
  tx_mutex = osMutexNew(NULL);

  /* creation of task_uart_rx */
  task_uart_rxHandle = osThreadNew(task_fn_uart_rx, NULL, &task_uart_rx_attributes);

  /* creation of task_uart_tx */
  task_uart_txHandle = osThreadNew(task_fn_uart_tx, NULL, &task_uart_tx_attributes);

  c.cmd = CMD_NONE;
  osMessageQueuePut(cmd_queue_handle, &c, 0, 0);
}

void app_on_new_chars(const uint8_t *buf, size_t len)
{
  size_t i;

  for (i = 0; i < len; ++i) {
    fifo_push(&uart_fifo_rx, buf[i]);
    osSemaphoreRelease(rx_sema);
  }
}

uint8_t cmdbuf[256];
int cmdbuf_cursor = 0;
int cmdbuf_len = 0;

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
  else if (!strncmp(p, "mww ", 4)) {
    return cmdbuf_parse_mww(c, p + 4);
  }

  return false;
}

static void msg(const char *fmt, ...)
{
  int n;
  va_list vl;
  va_start(vl, fmt);

  osMutexAcquire(msg_mutex, osWaitForever);
  n = vsnprintf(msg_buf, sizeof(msg_buf), fmt, vl);
  uart_tx_fifo_push((const uint8_t *)msg_buf, n);
  osMutexRelease(msg_mutex);
  va_end(vl);
}

static bool special_symbol_active = false;
static bool arrow_symbol_active = false;

#define SYMBOL_BACKSPACE '\b'
#define SYMBOL_ESCAPE 0x1b
#define SYMBOL_ARROW 0x5b
#define SYMBOL_ARROW_UP    0x41 // 'A'
#define SYMBOL_ARROW_DOWN  0x42 // 'B'
#define SYMBOL_ARROW_RIGHT 0x43 // 'C'
#define SYMBOL_ARROW_LEFT  0x44 // 'D'

static void app_handle_uart_rx(void)
{
  uint8_t c;
  bool success;

  osSemaphoreAcquire(rx_sema, osWaitForever);
  portENTER_CRITICAL();
  success = fifo_pop(&uart_fifo_rx, &c);
  portEXIT_CRITICAL();

  if (!success)
    return;

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
    struct cmd cmd;

    if (!cmdbuf_cursor) {
      cmd.cmd = CMD_NONE;
      osMessageQueuePut(cmd_queue_handle, &cmd, 0, 0);
      return;
    }

    msg_push("\r\n", 2);
    cmdbuf[cmdbuf_cursor] = 0;
    success = cmdbuf_parse(&cmd);

    cmdbuf_cursor = 0;
    cmdbuf_len = 0;

    if (!success) {
      msg("unknown command\r\n>");
      return;
    }

    osMessageQueuePut(cmd_queue_handle, &cmd, 0, 0);
  }
  else {
    cmdbuf[cmdbuf_cursor++] = c;
    tx_fifo_push(c);
    if (cmdbuf_cursor > cmdbuf_len)
      cmdbuf_len = cmdbuf_cursor;
  }
}

void transmit_completed_callback(void)
{
  // osSemaphoreRelease(tx_done_sema);
}

static void app_handle_uart_tx(void)
{
  uint8_t c;
  bool success;

  osThreadYield();
//  osSemaphoreAcquire(tx_done_sema, osWaitForever);
  if (!uart_tx_is_ready())
    return;

  osSemaphoreAcquire(tx_avail_sema, osWaitForever);
  portENTER_CRITICAL();
  success = fifo_pop(&uart_fifo_tx, &c);
  portEXIT_CRITICAL();
  osSemaphoreRelease(tx_free_sema);
  if (!success)
    return;

  uart_send_byte(c);
}

void task_fn_uart_rx(void *argument)
{
  rx_buf_stopper = 0x11223344;
  rx_buf_stopper1 = 0xaabbccdd;

  while(1)
    app_handle_uart_rx();
}

void task_fn_uart_tx(void *argument)
{
  while(1)
    app_handle_uart_tx();
}

void Error_Handler(void);

static void app_process_mem_read_32(struct target *t, struct cmd *c)
{
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;
  uint32_t value = 0;

  if (target_mem_read_32(t, addr, &value))
    msg("0x%08x\r\n", value);
  else
    msg("mem read 32 failed\r\n");
}

static void app_process_mem_write_32(struct target *t, struct cmd *c)
{
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;
  if (target_mem_write_32(t, addr, c->arg2))
    msg("done\r\n");
  else
    msg("mem write 32 failed\r\n");
}

void app_process_dump_regs(struct target *t)
{
  int i, j;
  struct target_core *c;
  struct aarch64_context *core_ctx;
  if (!t->attached) {
    msg("error: not attached\r\n");
    return;
  }

/*
 * STMCube c lib does not support uint64_t printing, so we have to split the
 * number to 2 by 32
 */
#define REG64_FMT "0x%08x%08x"
#define REG64_ARG(__v64) \
    (int)(((__v64) >> 32) & 0xffffffff), (int)((__v64) & 0xffffffff)

#define PRINT_REG64(__core, __idx, __v64) \
    msg("%d,x%d," REG64_FMT "\r\n", __core, __idx, REG64_ARG(__v64))

  for (i = 0; i < ARRAY_SIZE(t->core); ++i) {
    c = &t->core[i];
    if (!c->halted)
      continue;
    core_ctx = &c->a64.ctx;
    for (j = 0; j < ARRAY_SIZE(core_ctx->x0_30); ++j) {
      msg("%d,x%d," REG64_FMT "\r\n", i, j, REG64_ARG(core_ctx->x0_30[j]));
    }

    msg("%d,sp," REG64_FMT "\r\n", i, REG64_ARG(core_ctx->sp));
    msg("%d,pc," REG64_FMT "\r\n", i, REG64_ARG(core_ctx->pc));
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
  char *p = msg_buf;
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
  uart_tx_fifo_push((const uint8_t *)msg_buf, p - msg_buf);
  osMutexRelease(msg_mutex);
}

void app_sm_process_next_cmd(void)
{
  osStatus_t status;
  struct cmd cmd;
  uint8_t prio;
  uint32_t idcode;
  bool success;

  status = osMessageQueueGet(cmd_queue_handle, &cmd, &prio, osWaitForever);
  if (status != osOK)
    Error_Handler();

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
    case CMD_TARGET_DUMP_REGS:
      app_process_dump_regs(&t);
      break;
    default:
      break;
  }
  app_prompt();
}
