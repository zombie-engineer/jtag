#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <io_api.h>

typedef enum {
  CMD_NONE               = 0,
  CMD_TARGET_STATUS      = 1,
  CMD_TARGET_INIT        = 2,
  CMD_TARGET_HALT        = 3,
  CMD_TARGET_RESUME      = 4,
  CMD_TARGET_STEP        = 5,
  CMD_TARGET_SOFT_RESET  = 6,
  CMD_TARGET_MEM_ACCESS  = 7,
  CMD_TARGET_REG_ACCESS  = 8,
  CMD_TARGET_BREAKPOINT  = 9,
  CMD_TARGET_RUNNING_CHECK_HALTED = 10,
  CMD_TARGET_EXEC        = 11,
  CMD_TARGET_DUMP_REGS   = 13,
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

bool cmdbuf_parse(struct cmd *c, const char *buf, const char *end);
