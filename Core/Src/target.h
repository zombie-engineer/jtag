#pragma once
#include "adiv5.h"
#include "aarch64.h"
#include <stdint.h>
#include <stdbool.h>

struct target_core {
  struct aarch64 a64;

  uint32_t debug;
  uint32_t cti;
  uint32_t pmu;
  uint32_t etm;
  bool debug_exists;
  bool cti_exists;
  bool pmu_exists;
  bool etm_exists;

  bool halted;
};

struct target {
  struct adiv5_dap dap;
  struct target_core core[4];
};

bool target_halt(struct target *d);
bool target_resume(struct target *d);
bool target_init(struct target *t);
bool target_exec_instr(struct target *t, uint32_t instr);
