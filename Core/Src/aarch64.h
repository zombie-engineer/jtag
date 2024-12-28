#pragma once
#include <stdint.h>
#include <stdbool.h>

struct adiv5_dap;

struct aarch64_context {
  uint64_t pc;
  uint64_t sp;
  uint64_t x0_30[31];
  int el;
  int pstate;
};

struct aarch64_dbg_regs_cache {
  /* Event status register cache */
  uint32_t edesr;
  /* Execution control register cache */
  uint32_t edecr;
  /* Status and control register cache */
  uint32_t edscr;
  /* Debug reserve control register cache */
  uint32_t edrcr;
  /* Aux control register cache */
  uint32_t edacr;
  /* Processor status register cache */
  uint32_t edprsr;
  /* Debug Lock Status register cache */
  uint32_t edlsr;

  /* Last sampled PC  */
  uint64_t sampled_pc;

  uint32_t edcidsr;
  uint32_t edvidsr;
  uint32_t edprcr;
  uint32_t midr_el1;
  uint64_t edpfr;
  uint64_t eddfr;
  uint32_t memfeature0;
  uint32_t memfeature1;
};

struct aarch64 {
  struct adiv5_dap *dap;
  struct aarch64_dbg_regs_cache regs;
  bool is_mem_mode;
  struct aarch64_context ctx;
};

void aarch64_init(struct aarch64 *a, struct adiv5_dap *d, uint32_t baseaddr,
  uint32_t cti_baseaddr);

bool aarch64_set_memory_mode(struct aarch64 *a, uint32_t baseaddr);

bool aarch64_set_normal_mode(struct aarch64 *a, uint32_t baseaddr);

bool aarch64_halt(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr);

bool aarch64_resume(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr);

void aarch64_mess(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr);

bool aarch64_exec(struct aarch64 *a, uint32_t baseaddr, uint32_t instr);
bool aarch64_fetch_context(struct aarch64 *a, uint32_t baseaddr);
