#pragma once
#include <stdint.h>
#include <stddef.h>

typedef enum {
  JTAG_STATE_UNKNOWN = 0,
  JTAG_STATE_TEST_LOGIC_RESET,
  JTAG_STATE_RUN_TEST_IDLE,
  JTAG_STATE_SELECT_DR_SCAN,
  JTAG_STATE_CAPTURE_DR,
  JTAG_STATE_SHIFT_DR,
  JTAG_STATE_EXIT1_DR,
  JTAG_STATE_PAUSE_DR,
  JTAG_STATE_EXIT2_DR,
  JTAG_STATE_UPDATE_DR,
  JTAG_STATE_SELECT_IR_SCAN,
  JTAG_STATE_CAPTURE_IR,
  JTAG_STATE_SHIFT_IR,
  JTAG_STATE_EXIT1_IR,
  JTAG_STATE_PAUSE_IR,
  JTAG_STATE_EXIT2_IR,
  JTAG_STATE_UPDATE_IR,
  JTAG_STATE_COUNT = JTAG_STATE_UPDATE_IR
} jtag_state_t;

struct jtag_state_path {
  uint8_t path;
  uint8_t n;
};

const struct jtag_state_path *jtag_get_state_path(jtag_state_t from,
  jtag_state_t to);
