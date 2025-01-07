#include "aarch64.h"
#include "adiv5.h"
#include "cmsis_edi.h"
#include "arm_cti.h"
#include "aarch64_edprsr.h"
#include "aarch64_edscr.h"
#include "common.h"

static inline bool aarch64_set_mod_reg_bits(struct adiv5_dap *d,
  uint32_t regaddr, uint32_t *reg_cache, uint32_t mask, uint32_t value)
{
  uint32_t reg;
  uint32_t *preg = reg_cache ? reg_cache : &reg;
  uint32_t good_value;

  adiv5_mem_ap_read_word(d, regaddr, preg);
  good_value = ((*preg) & ~mask) | value;
  adiv5_mem_ap_write_word(d, regaddr, good_value);
  adiv5_mem_ap_read_word(d, regaddr, preg);

  return good_value == *preg;
}

static void aarch64_read_status_regs(struct adiv5_dap *d,
  struct aarch64_dbg_regs_cache *ed, uint32_t baseaddr)
{
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &ed->edesr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &ed->edecr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
}

bool aarch64_set_memory_mode(struct aarch64 *a, uint32_t baseaddr)
{
  bool success;

  if (aarch64_edscr_is_mem_mode(a->regs.edscr))
    return true;

  success = aarch64_set_mod_reg_bits(a->dap,
    baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr, 1<<20, 1<<20);

  if (success)
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);

  return success;
}

bool aarch64_set_normal_mode(struct aarch64 *a, uint32_t baseaddr)
{
  bool success;

  if (!aarch64_edscr_is_mem_mode(a->regs.edscr))
    return true;

  success = aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 1<<20, 0);

  if (success) {
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);
  }

  return success;
}

bool aarch64_halt(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr)
{
  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 1<<14, 1<<14))
    return false;

#if 0
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDECR,
    &a->regs.edecr);
#endif

  /* HALT */
  cti_pulse_event(a->dap, cti_baseaddr, CTI_EVENT_HALT);
  while(1) {
    /* Processor status register has info if core is halted or not */
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);
    if (aarch64_edprsr_is_halted(a->regs.edprsr))
      break;
  }

  aarch64_read_status_regs(a->dap, &a->regs, baseaddr);

  if (aarch64_edscr_is_error(a->regs.edscr)) {
    adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, (1<<2)|(1<<3));
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
    aarch64_read_status_regs(a->dap, &a->regs, baseaddr);
  }

  return true;
}

bool aarch64_resume(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr)
{
  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 1<<14, 1<<14))
    return false;

  /* RESUME */
  cti_ungate_channel(a->dap, cti_baseaddr, 1);
  cti_gate_channel(a->dap, cti_baseaddr, 0);
  cti_ack(a->dap, cti_baseaddr);

  while(1) {
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);
    if (((a->regs.edprsr >> 4) & 1) == 0)
      break;
  }

  aarch64_read_status_regs(a->dap, &a->regs, baseaddr);
  return true;
}

void aarch64_mess(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr)
{
  uint32_t reg = 0;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return;

  /* mrs x0, dlr_el0 */
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, 0xd53b4520);

  /* msr dbgdtrtx_el0, x0 */
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, 0xd5130500);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);

  /* add x0, x0, #4 */
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, 0x91001000);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);
  /* msr dlr_el0, x0 */
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, 0xd51b4520);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);

  return;
  // adiv5_mem_ap_write_word(baseaddr + DBG_REG_ADDR_EDITR, 0xd5130500);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);
  aarch64_set_memory_mode(a, baseaddr);

  while(1) {
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
    if (reg == 0xd518c000) {
    }
  }


  /*
   * reg = *DTRTX
   * LDR W1, [X0], #4
   * MSR W1, DBGDTRTX_EL0
   */

  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, 0x80000);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, a->regs.edscr & ~(1 << 20));
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);

  // adiv5_mem_ap_write_word(baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, 0x3fffffff);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDLSR, &a->regs.edlsr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, &a->regs.edrcr);
  a->regs.edrcr |= 1<<3;
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, a->regs.edrcr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  while(1) {
    adiv5_mem_ap_read_word_drw(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
    adiv5_mem_ap_read_word_drw(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDESR, &a->regs.edesr);
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);
  }
}

#define AARCH64_CORE_REG_X0 0
#define AARCH64_CORE_REG_X1 1
#define AARCH64_CORE_REG_X2 2
#define AARCH64_CORE_REG_X3 3
#define AARCH64_CORE_REG_X4 4
#define AARCH64_CORE_REG_X5 5
#define AARCH64_CORE_REG_X6 6
#define AARCH64_CORE_REG_X7 7
#define AARCH64_CORE_REG_X8 8
#define AARCH64_CORE_REG_X9 9
#define AARCH64_CORE_REG_X10 10
#define AARCH64_CORE_REG_X11 11
#define AARCH64_CORE_REG_X12 12
#define AARCH64_CORE_REG_X13 13
#define AARCH64_CORE_REG_X14 14
#define AARCH64_CORE_REG_X15 15
#define AARCH64_CORE_REG_X16 16
#define AARCH64_CORE_REG_X17 17
#define AARCH64_CORE_REG_X18 18
#define AARCH64_CORE_REG_X19 19
#define AARCH64_CORE_REG_X20 20
#define AARCH64_CORE_REG_X21 21
#define AARCH64_CORE_REG_X22 22
#define AARCH64_CORE_REG_X23 23
#define AARCH64_CORE_REG_X24 24
#define AARCH64_CORE_REG_X25 25
#define AARCH64_CORE_REG_X26 26
#define AARCH64_CORE_REG_X27 27
#define AARCH64_CORE_REG_X28 28
#define AARCH64_CORE_REG_X29 29
#define AARCH64_CORE_REG_X30 30
#define AARCH64_CORE_REG_PC  31
#define AARCH64_CORE_REG_SP  32
#define AARCH64_CORE_REG_SCTLR_EL1 33
#define AARCH64_CORE_REG_ESR_EL2   34
#define AARCH64_CORE_REG_FAR_EL2   35
#define AARCH64_CORE_REG_DISR_EL1  36

/*
 * 4-byte Aarch64 instruction that MOVes contents of register Xn to
 * DBGDTR_EL0, with 'n' - being register index from 0 to 30,
 * x0, x1, x2, ..., x30
 * DBGDTR_EL0 is a system register, accessible from software that is
 * mapped to external debug registers DBGDTRTX_EL0 and DBGDTRRX_EL0 like that:
 * DBGDTRTX_EL0[31:0] <- DBGDTR_EL0[31:0]
 * DBGDTRRX_EL0[31:0] <- DBGDTR_EL0[63:32]
 */
#define AARCH64_INSTR_MSR_DBGDTR_EL0(__r) (0xd5130400 | (__r & 0x1f))
#define AARCH64_INSTR_MRS_DBGDTR_EL0(__r) (0xd5330400 | (__r & 0x1f))

static bool aarch64_read_core_reg(struct aarch64 *a, uint32_t baseaddr,
  int reg, uint64_t *out_reg)
{
  uint32_t tmp_lo = 0, tmp_hi = 0;

  uint32_t instructions[2];
  int num_i;

  if (reg >= AARCH64_CORE_REG_X0 && reg <= AARCH64_CORE_REG_X30) {
    /* msr dbgdtr_el0, xN, where N==reg*/
    instructions[0] = AARCH64_INSTR_MSR_DBGDTR_EL0(reg);
    num_i = 1;
  }
  else if (reg == AARCH64_CORE_REG_PC) {
    /* mrs x0, dlr_el0 */
    instructions[0] = 0xd53b4520;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
  }
  else if (reg == AARCH64_CORE_REG_SP) {
    /* mov x0, sp */
    instructions[0] = 0x910003e0;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
  }
  else if (reg == AARCH64_CORE_REG_SCTLR_EL1) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd5381000;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
  }
  else if (reg == AARCH64_CORE_REG_ESR_EL2) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd53c5200;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
  }
  else if (reg == AARCH64_CORE_REG_FAR_EL2) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd53c6000;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
  }
  else if (reg == AARCH64_CORE_REG_DISR_EL1) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd538c120;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
  }
  else
    return false;

  for (int i = 0; i < num_i; ++i) {
    adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
      instructions[i]);
  }

  while(1) {
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
      &a->regs.edscr);
    if (aarch64_edscr_is_error(a->regs.edscr))
      return false;
    if (aarch64_edscr_is_tx_full(a->regs.edscr))
      break;
  }

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &tmp_hi);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &tmp_lo);
  *out_reg = (uint64_t)tmp_lo | ((uint64_t)tmp_hi << 32);
  return true;
}

static bool aarch64_write_core_reg(struct aarch64 *a, uint32_t baseaddr,
  int reg, uint64_t value)
{
  uint32_t instructions[2];
  int num_i;

  if (reg >= AARCH64_CORE_REG_X0 && reg <= AARCH64_CORE_REG_X30) {
    /* msr dbgdtr_el0, xN, where N==reg*/
    instructions[0] = AARCH64_INSTR_MRS_DBGDTR_EL0(reg);
    num_i = 1;
  }
#if 0
  else if (reg == AARCH64_CORE_REG_PC) {
    /* mrs x0, dlr_el0 */
    instructions[0] = 0xd53b4520;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
  }
  else if (reg == AARCH64_CORE_REG_SP) {
    /* mov x0, sp */
    instructions[0] = 0x910003e0;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
  }
  else if (reg == AARCH64_CORE_REG_SCTLR_EL1) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd5381000;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
  }
#endif
  else
    return false;

  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0,
    value & 0xffffffff);

  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
    (value >> 32) & 0xffffffff);

  for (int i = 0; i < num_i; ++i) {
    adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
      instructions[i]);
  }

  while(1) {
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
      &a->regs.edscr);
    if (aarch64_edscr_is_error(a->regs.edscr))
      return false;
    // if (aarch64_edscr_is_tx_full(a->regs.edscr))
    break;
  }

  return true;
}

bool aarch64_fetch_context(struct aarch64 *a, uint32_t baseaddr)
{
  uint64_t reg;

  struct aarch64_context *c = &a->ctx;

  for (int i = AARCH64_CORE_REG_X0; i <= AARCH64_CORE_REG_X30; ++i) {
    if (!aarch64_read_core_reg(a, baseaddr, i, &reg))
      return false;
    c->x0_30[i] = reg;
  }

  if (!aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_PC, &reg))
    return false;
  c->pc = reg;

  if (!aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_SP, &reg))
    return false;
  c->sp = reg;
  c->el = aarch64_edscr_get_el(a->regs.edscr);

  if (!aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_SCTLR_EL1, &reg))
    return false;
  c->sctlr_el1 = reg;
  c->mmu_on = reg & 1;

  return true;
}

bool aarch64_read_mem32(struct aarch64 *a, uint32_t baseaddr, uint64_t addr,
    uint32_t *dst, size_t num_words)
{
  uint32_t tmp;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return false;

  if (!aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr))
    return false;

#if 0
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    AARCH64_INSTR_MSR_DBGDTR_EL0(0));

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);
#endif

  if (!aarch64_set_memory_mode(a, baseaddr))
    return false;

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &tmp);
  for (size_t i = 0; i < num_words; ++i)
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
      &dst[i]);
  return true;
}

bool aarch64_write_mem32(struct aarch64 *a, uint32_t baseaddr, uint64_t addr,
    const uint32_t *src, size_t num_words)
{
  if (!aarch64_set_normal_mode(a, baseaddr))
    return false;

  if (!aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr))
    return false;

  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    AARCH64_INSTR_MSR_DBGDTR_EL0(0));

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  if (!aarch64_set_memory_mode(a, baseaddr))
    return false;

  for (size_t i = 0; i < num_words; ++i) {
    adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0,
      src[i]);
  }

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  return !aarch64_edscr_is_error(a->regs.edscr);
}

bool aarch64_read_mem32_once(struct aarch64 *a, uint32_t baseaddr,
  uint64_t addr, uint32_t *out_value)
{
  uint64_t reg = 0;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return false;

  if (!aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr))
    return false;

  /*
   * There is an issue with some of the STR/LDR instructions that can be put
   * into EDITR to read/write memory. The once that are compiled by GCC as
   * binary code for ldr w1, [x0] and str w1, [x0] lead to EDSCR.ERR bit set
   * to 1.
   * In Armv8 architectures manual there is a bytecode, used for
   * writing/reading values while accessing DBGDTRTX_EL0 and DBGDTRRX_EL0
   * regs. Also Openocd has precompiled bytecode for 'slow' reads.
   * These 2 work fine, so same bytecode I use here.
   */
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    0xb8404401);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  if (aarch64_edscr_is_error(a->regs.edscr)) {
    adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, (1<<2)|(1<<3));
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
    aarch64_read_status_regs(a->dap, &a->regs, baseaddr);
    return false;
  }

  if (!aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_X1, &reg))
    return false;

  *out_value = reg & 0xffffffff;
  return true;
}

bool aarch64_write_mem32_once(struct aarch64 *a, uint32_t baseaddr,
  uint64_t addr, uint32_t value)
{
  if (!aarch64_set_normal_mode(a, baseaddr))
    return false;

  if (!aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr))
    return false;

  if (!aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X1, value))
    return false;

  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    0xb8004401);
#if 0
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    0xf8408400 | (5 << 5) | (1 << 0));
#endif
    // 0xb8404401);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  if (aarch64_edscr_is_error(a->regs.edscr)) {
    adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, (1<<2)|(1<<3));
    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
    aarch64_read_status_regs(a->dap, &a->regs, baseaddr);
  }

  return !aarch64_edscr_is_error(a->regs.edscr);
}

bool aarch64_exec(struct aarch64 *a, uint32_t baseaddr, const uint32_t *const instr, int num)
{
  if (!aarch64_set_normal_mode(a, baseaddr))
    return false;

  for (int i = 0; i < num; ++i) {
    adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDITR, instr[i]);

    adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
        &a->regs.edscr);

    if (aarch64_edscr_is_error(a->regs.edscr)) {
      adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, (1<<2)|(1<<3));
      adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
      aarch64_read_status_regs(a->dap, &a->regs, baseaddr);
      return false;
    }
  }

  return true;
}


void aarch64_init(struct aarch64 *a, struct adiv5_dap *d, uint32_t baseaddr,
  uint32_t cti_baseaddr)
{
  uint32_t reg;

  a->dap = d;
  /*
   * Unlock Debug lock, to verify - readback the value in EDPRSR.OSLK, with 0
   * as UNLOCKED. Only after OSLK unlock the rest of the debug registers can
   * be accessed in predictable expected way.
   */
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_OSLAR_EL1, 0);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_OSLAR_EL1, &reg);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);

  /*
   * Contents of EDLSR show if software lock is implemented and se
   */
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDLSR, &a->regs.edlsr);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDESR, &a->regs.edesr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDECR, &a->regs.edecr);
  adiv5_mem_ap_write_word(a->dap, baseaddr + DBG_REG_ADDR_EDECR, 3);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDESR, &a->regs.edesr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDECR, &a->regs.edecr);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, &a->regs.edrcr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDACR, &a->regs.edacr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDECCR, &a->regs.edacr);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPCSR_LO, &reg);
  a->regs.sampled_pc = ((uint64_t)reg) & 0xffffffff;
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPCSR_HI, &reg);
  a->regs.sampled_pc |= ((uint64_t)reg) << 32;

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDCIDSR, &a->regs.edcidsr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDVIDSR, &a->regs.edvidsr);

  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR, &a->regs.edprsr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_EDPRCR, &a->regs.edprcr);
  adiv5_mem_ap_read_word(a->dap, baseaddr + DBG_REG_ADDR_MIDR_EL1, &a->regs.midr_el1);
  adiv5_mem_ap_read_word(a->dap, baseaddr + 0xd38, &a->regs.memfeature0);
  adiv5_mem_ap_read_word(a->dap, baseaddr + 0xd3c, &a->regs.memfeature1);
  cti_init(a->dap, cti_baseaddr);
}
