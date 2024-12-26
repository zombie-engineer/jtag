#pragma once
#include "adiv5.h"
#include "target_arm.h"
#include <stdint.h>
#include <stdbool.h>

struct target_core {
  struct ext_dbg_aarch64 edi;

  uint32_t debug;
  uint32_t cti;
  uint32_t pmu;
  uint32_t etm;
  bool debug_exists;
  bool cti_exists;
  bool pmu_exists;
  bool etm_exists;
};

struct target {
  struct adiv5_dap dap;
  struct target_core core[4];
};

void target_halt(struct target *d);
void target_resume(struct target *d);
void target_read(struct target *d);
bool target_init(struct target *t);