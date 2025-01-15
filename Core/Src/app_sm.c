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

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define RX_FIFO_SIZE 256
#define TX_FIFO_SIZE 256

struct target t;

typedef enum {
  CMD_TARGET_NONE   = 0,
  CMD_TARGET_INIT   = 1,
  CMD_TARGET_HALT   = 2,
  CMD_TARGET_RESUME = 3,
  CMD_TARGET_SOFT_RESET   = 4,
  CMD_TARGET_MEM_READ_32  = 5,
  CMD_TARGET_MEM_WRITE_32 = 6,
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
static osSemaphoreId_t tx_done_sema;
static osSemaphoreId_t rx_sema;
static osSemaphoreId_t tx_sema;

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

static StaticSemaphore_t rx_sema_cb;
static const osSemaphoreAttr_t rx_sema_attr = {
  .name = "rx_sema",
  .attr_bits = 0,
  .cb_mem = &rx_sema_cb,
  .cb_size = sizeof(rx_sema_cb)
};

static StaticSemaphore_t tx_sema_cb;
static const osSemaphoreAttr_t tx_sema_attr = {
  .name = "tx_sema",
  .attr_bits = 0,
  .cb_mem = &tx_sema_cb,
  .cb_size = sizeof(tx_sema_cb)
};

static StaticSemaphore_t tx_done_sema_cb;
static const osSemaphoreAttr_t tx_done_sema_attr = {
  .name = "tx_done_sema",
  .attr_bits = 0,
  .cb_mem = &tx_done_sema_cb,
  .cb_size = sizeof(tx_done_sema_cb)
};

static uint8_t usb_rx_buf[RX_FIFO_SIZE];
static uint8_t usb_tx_buf[TX_FIFO_SIZE];

static char logfun_buf[64];

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

static inline bool tx_fifo_push(uint8_t ch)
{
  bool success;
  portENTER_CRITICAL();
  success = fifo_push(&uart_fifo_tx, ch);
  portEXIT_CRITICAL();
  if (success)
    osSemaphoreRelease(tx_sema);
  return success;
}

static int uart_tx_fifo_push(const uint8_t *msg, size_t len)
{
  size_t i;

  for (i = 0; i < len; ++i) {
    if (!tx_fifo_push(msg[i]))
      break;
  }
  return i;
}

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
  tx_fifo_push('\r');
  tx_fifo_push('\n');
  tx_fifo_push('>');
}

void app_sm_init(void)
{
  appstate = SM_IDLING;

  /* creation of queue_cmd */
  cmd_queue_handle = osMessageQueueNew (2, sizeof(struct cmd),
    &queue_cmd_attributes);

  rx_sema = osSemaphoreNew(sizeof(usb_rx_buf), 0, &rx_sema_attr);
  tx_sema = osSemaphoreNew(sizeof(usb_tx_buf), 0, &tx_sema_attr);
  tx_done_sema = osSemaphoreNew(1, 0, &tx_done_sema_attr);

  /* creation of task_uart_rx */
  task_uart_rxHandle = osThreadNew(task_fn_uart_rx, NULL, &task_uart_rx_attributes);

  /* creation of task_uart_tx */
  task_uart_txHandle = osThreadNew(task_fn_uart_tx, NULL, &task_uart_tx_attributes);
  app_prompt();
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
    c->cmd = CMD_TARGET_NONE;
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

static void logfun(const char *fmt, ...)
{
  int n;
  va_list vl;
  va_start(vl, fmt);

  n = vsnprintf(logfun_buf, sizeof(logfun_buf), fmt, vl);
  uart_tx_fifo_push((const uint8_t *)logfun_buf, n);
  va_end(vl);
}

static bool special_symbol_active = false;
static bool arrow_symbol_active = false;

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

  if (c == SYMBOL_ESCAPE) {
    special_symbol_active = true;
    return;
  }

  else if (c == '\n') {
    tx_fifo_push('\n');
  }

  else if (c == '\r') {
    struct cmd cmd;

    if (!cmdbuf_cursor) {
      cmd.cmd = CMD_TARGET_NONE;
      osMessageQueuePut(cmd_queue_handle, &cmd, 0, 0);
      return;
    }

    tx_fifo_push('\r');
    tx_fifo_push('\n');
    cmdbuf[cmdbuf_cursor] = 0;
    // logfun("executing '%s'\r\n", cmdbuf);
    success = cmdbuf_parse(&cmd);

    cmdbuf_cursor = 0;
    cmdbuf_len = 0;

    if (!success) {
      logfun("unknown command\r\n");
      return;
    }

    osMessageQueuePut(cmd_queue_handle, &cmd, 0, 0);
  } else {
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

  osSemaphoreAcquire(tx_sema, osWaitForever);
  portENTER_CRITICAL();
  success = fifo_pop(&uart_fifo_tx, &c);
  portEXIT_CRITICAL();
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

void app_process_mem_read_32(struct target *t, struct cmd *c)
{
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;
  uint32_t value = 0;

  if (target_mem_read_32(t, addr, &value))
    logfun("%08x\r\n", value);
  else
    logfun("mem read 32 failed\r\n");
}

void app_process_mem_write_32(struct target *t, struct cmd *c)
{
  uint64_t addr = (((uint64_t)(c->arg1)) << 32) | c->arg0;
  if (target_mem_write_32(t, addr, c->arg2))
    logfun("done\r\n");
  else
    logfun("mem write 32 failed\r\n");
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
    case CMD_TARGET_NONE:
      break;
    case CMD_TARGET_INIT:
      target_init(&t, &idcode);
      logfun("target initialized: %08x\r\n", idcode);
      break;
    case CMD_TARGET_HALT:
      success = target_halt(&t);
      logfun("target halt status: %d\r\n", success ? 1 : 0);
      break;
    case CMD_TARGET_RESUME:
      success = target_resume(&t);
      logfun("target resume status: %d\r\n", success ? 1 : 0);
      break;
    case CMD_TARGET_SOFT_RESET:
      success = target_soft_reset(&t);
      logfun("target soft reset status: %d\r\n", success ? 1 : 0);
      break;
    case CMD_TARGET_MEM_READ_32:
      app_process_mem_read_32(&t, &cmd);
      break;
    case CMD_TARGET_MEM_WRITE_32:
      app_process_mem_write_32(&t, &cmd);
      break;
    default:
      break;
  }
  app_prompt();
}
