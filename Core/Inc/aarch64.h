#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include <io_api.h>
#include <breakpoint.h>
#include <reg.h>

#define AARCH64_CORE_REG_X0   0
#define AARCH64_CORE_REG_X1   1
#define AARCH64_CORE_REG_X2   2
#define AARCH64_CORE_REG_X3   3
#define AARCH64_CORE_REG_X4   4
#define AARCH64_CORE_REG_X5   5
#define AARCH64_CORE_REG_X6   6
#define AARCH64_CORE_REG_X7   7
#define AARCH64_CORE_REG_X8   8
#define AARCH64_CORE_REG_X9   9
#define AARCH64_CORE_REG_X10  10
#define AARCH64_CORE_REG_X11  11
#define AARCH64_CORE_REG_X12  12
#define AARCH64_CORE_REG_X13  13
#define AARCH64_CORE_REG_X14  14
#define AARCH64_CORE_REG_X15  15
#define AARCH64_CORE_REG_X16  16
#define AARCH64_CORE_REG_X17  17
#define AARCH64_CORE_REG_X18  18
#define AARCH64_CORE_REG_X19  19
#define AARCH64_CORE_REG_X20  20
#define AARCH64_CORE_REG_X21  21
#define AARCH64_CORE_REG_X22  22
#define AARCH64_CORE_REG_X23  23
#define AARCH64_CORE_REG_X24  24
#define AARCH64_CORE_REG_X25  25
#define AARCH64_CORE_REG_X26  26
#define AARCH64_CORE_REG_X27  27
#define AARCH64_CORE_REG_X28  28
#define AARCH64_CORE_REG_X29  29
#define AARCH64_CORE_REG_X30  30
#define AARCH64_CORE_REG_PC   31
#define AARCH64_CORE_REG_SP   32
#define AARCH64_CORE_REG_CPSR 33
#define AARCH64_CORE_REG_FPSR 34
#define AARCH64_CORE_REG_FPCR 35
#define AARCH64_STATE_REGS (AARCH64_CORE_REG_SP + 1)

#define AARCH64_CORE_REG_SCTLR_EL1  36
#define AARCH64_CORE_REG_ESR_EL2    37
#define AARCH64_CORE_REG_FAR_EL2    38
#define AARCH64_CORE_REG_DISR_EL1   39
#define AARCH64_CORE_REG_DSPSR_EL0  40
#define AARCH64_CORE_REG_CTR_EL0    41
#define AARCH64_CORE_REG_CLIDR_EL1  42
#define AARCH64_CORE_REG_CSSELR_EL1 43
#define AARCH64_CORE_REG_VBAR_EL1   44
#define AARCH64_CORE_REG_UNKNOWN 0xffff

typedef enum {
  /* Failed to establish halt reason  */
  AARCH64_HALT_REASON_UNKNOWN = 0,
  /* Halted due to request from extenal debugger */
  AARCH64_HALT_REASON_EXTERNAL_SIGNAL = 1,
  /* Halted due to step */
  AARCH64_HALT_REASON_HALT_STEP = 2,
  /* Halted due to cpu executed 'HLT' instruction */
  AARCH64_HALT_REASON_SW_HLT_INSTR = 3,
  /* Halted due to step */
  AARCH64_HALT_REASON_HALT_STEP_NO_SYNDROME = 4,
} aarch64_halt_reason_t;


struct adiv5_dap;

struct aarch64_context {
  struct reg regcache[AARCH64_STATE_REGS];
  struct reg sctlr_el1;
  struct reg vbar_el1;
  int el;
  uint64_t pstate;
  uint64_t fpsr;
  uint64_t fpcr;
  bool mmu_on;
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
  struct aarch64_context ctx;
  bool cache_line_fetched;
  int dcache_line_sz;
  int icache_line_sz;
  aarch64_halt_reason_t halt_reason;
};

int aarch64_init(struct aarch64 *a, struct adiv5_dap *d, uint32_t baseaddr,
  uint32_t cti_baseaddr);

bool aarch64_set_memory_mode(struct aarch64 *a, uint32_t baseaddr);

bool aarch64_set_normal_mode(struct aarch64 *a, uint32_t baseaddr);

int aarch64_halt(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr);

int aarch64_check_halted(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr);

int aarch64_resume(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr);

int aarch64_restore_reg(struct aarch64 *a, uint32_t baseaddr, uint32_t reg_id);
int aarch64_restore_before_resume(struct aarch64 *a, uint32_t baseaddr);

int aarch64_step(struct aarch64 *a, uint32_t baseaddr, uint32_t cti_baseaddr);

int aarch64_breakpoint(struct aarch64 *a, uint32_t baseaddr, bool remove,
  bool hardware, struct breakpoint *b);

void aarch64_mess(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr);

int aarch64_exec(struct aarch64 *a, uint32_t baseaddr,
  const uint32_t *const instr, int num);

int aarch64_iter_state_regs(struct aarch64 *a, uint32_t baseaddr,
  reg_iter_cb_t cb, void *arg);

int aarch64_write_mem32(struct aarch64 *a, uint32_t baseaddr, uint64_t addr,
  const uint32_t *src, size_t num_words);

int aarch64_write_mem32_once(struct aarch64 *a, uint32_t baseaddr,
  uint64_t addr, uint32_t value);

int aarch64_read_cached_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t *value);
int aarch64_write_cached_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t value);

int aarch64_reg_read_64(struct aarch64 *a, uint32_t baseaddr, uint32_t reg_id,
  uint64_t *out);

int aarch64_read_mem_once(struct aarch64 *a, uint32_t baseaddr,
  mem_access_size_t access_size, uint64_t addr, void *out_value);

int aarch64_mem_read_fast(struct aarch64 *a, uint32_t baseaddr,
  mem_access_size_t access_size, uint64_t addr, size_t count,
  void (*cb)(uint64_t, mem_access_size_t));

void aarch64_get_halt_reason(struct aarch64 *a, const char **str);
