#include <app_sm.h>
#include "jtag.h"
#include <adiv5.h>
#include "cmsis_edi.h"
#include <target.h>
#include <cmsis_gcc.h>

struct target t;

#define RX_FIFO_SIZE 256
#define TX_FIFO_SIZE 256

static uint8_t usb_rx_buf[RX_FIFO_SIZE];
static uint8_t usb_tx_buf[TX_FIFO_SIZE];

struct fifo {
  uint8_t *buf;
  int head;
  int tail;
  size_t sz;
};

struct fifo uart_fifo_rx = {
  .buf = usb_rx_buf,
  .head = 0,
  .tail = 0,
  .sz = sizeof(usb_rx_buf)
};

struct fifo uart_fifo_tx = {
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

void app_sm_init(void)
{
  target_init(&t);
  appstate = SM_IDLING;
}

static void jtag_push_cmd(void)
{
}

extern void usb_push_message(const uint8_t *buf, size_t len);

void app_on_new_chars(const uint8_t *buf, size_t len)
{
  size_t i;

  for (i = 0; i < len; ++i)
    fifo_push(&uart_fifo_rx, buf[i]);
}

uint8_t cmdbuf[256];
int cmdbuf_idx = 0;

extern void uart_send_byte(uint8_t b);
extern bool uart_tx_is_ready(void);

typedef enum {
  CMD_TARGET_NONE   = 0,
  CMD_TARGET_INIT   = 1,
  CMD_TARGET_HALT   = 2,
  CMD_TARGET_RESUME = 3,
} cmd_t;

typedef enum {
  CMD_STATE_NONE = 0,
  CMD_STATE_PENDING,
  CMD_STATE_BUSY,
  CMD_STATE_DONE,
} cmd_state_t;

struct cmd {
  cmd_state_t state;
  cmd_t cmd;
  uint32_t arg0;
  uint32_t arg1;
  uint32_t arg2;
  uint32_t arg3;
};

struct cmd cmd = {
  .state = CMD_STATE_NONE
};

static void app_handle_uart_rx(void)
{
  uint8_t c;
  bool success;

  __disable_irq();
  success = fifo_pop(&uart_fifo_rx, &c);
  __enable_irq();

  if (!success)
    return;

  __disable_irq();
  fifo_push(&uart_fifo_tx, c);
  __enable_irq();

  if (c == '\r') {
    fifo_push(&uart_fifo_tx, '0');
    fifo_push(&uart_fifo_tx, 'x');
    fifo_push(&uart_fifo_tx, '0');
  }
  else
    cmdbuf[cmdbuf_idx++] = c;

}

static void app_handle_uart_tx(void)
{
  uint8_t c;
  bool success;

  if (!uart_tx_is_ready())
    return;

  __disable_irq();
  success = fifo_pop(&uart_fifo_tx, &c);
  __enable_irq();
  if (!success)
    return;

  uart_send_byte(c);
}

static void app_exec_cmd(struct cmd *c)
{
  switch(c->cmd)
  {
    case CMD_TARGET_NONE:
      break;
    case CMD_TARGET_INIT:
      target_init(&t);
      break;
    case CMD_TARGET_HALT:
      target_halt(&t);
      break;
    case CMD_TARGET_RESUME:
      target_resume(&t);
      break;
    default:
      break;
  }
}

static void app_process_cmd(void)
{
  switch (cmd.state) {
    case CMD_STATE_NONE:
      break;
    case CMD_STATE_PENDING:
      cmd.state = CMD_STATE_BUSY;
      app_exec_cmd(&cmd);
      break;
    case CMD_STATE_BUSY:
      break;
    case CMD_STATE_DONE:
      break;
    default:
      break;
  }
}

void app_sm(void)
{
  app_handle_uart_rx();
  app_handle_uart_tx();
  app_process_cmd();


#if 0
  switch (appstate) {
    case SM_IDLING:
      break;
    case SM_USB_TRANSMIT_DONE:
      break;
    case SM_CMD_DONE:
      usb_push_message("done", 4);
      break;
    case SM_USB_NEW_CMD_PENDING:
      jtag_push_cmd();
      break;
  }
#endif
  __WFI();
}
