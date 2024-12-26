#pragma once
#include <stdint.h>

struct adiv5_dap;

struct ext_dbg_aarch64 {
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

void target_arm_init(struct adiv5_dap *d, uint32_t baseaddr,
  struct ext_dbg_aarch64 *ed);
