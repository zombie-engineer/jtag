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
  bool attached;
  uint32_t idcode;
};

int target_halt(struct target *d);
bool target_is_halted(const struct target *t);
int target_resume(struct target *d);
int target_step(struct target *d);
int target_breakpoint(struct target *d, bool remove, bool hardware,
  uint64_t arg);
int target_init(struct target *t);
int target_soft_reset(struct target *t);

int target_mem_read(struct target *t, mem_access_size_t access_size,
  uint64_t addr, size_t num, void (*cb)(uint64_t, mem_access_size_t));

int target_mem_write(struct target *t, mem_access_size_t access_size,
  uint64_t addr, uint64_t value);

int target_reg_write_64(struct target *t, uint32_t reg_id, uint64_t value);
int target_reg_read_64(struct target *t, uint32_t reg_id, uint64_t *out);
int target_exec_instr(struct target *t, uint32_t instr);
int target_mem_read_fast_start(struct target *t, uint64_t addr);
int target_mem_read_fast_next(struct target *t, uint32_t *value);
int target_mem_read_fast_stop(struct target *t);
