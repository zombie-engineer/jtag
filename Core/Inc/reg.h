#pragma once

typedef enum {
  REG_STATE_INVALID = 0,
  REG_STATE_CLEAN,
  REG_STATE_MODIFIED,
} reg_state_t;

struct reg {
  reg_state_t state;
  uint64_t value;
};

typedef void (*reg_iter_cb_t)();
