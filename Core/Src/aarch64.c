#include "aarch64.h"
#include "adiv5.h"
#include "cmsis_edi.h"
#include "arm_cti.h"
#include "aarch64_edprsr.h"
#include "aarch64_edscr.h"
#include "common.h"
#include <errno.h>

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
#define op0 19
#define op1 16
#define CRn 12
#define CRm 8
#define op2 5

#define MRSMSR_SYSREG(_op0, _op1, _CRn, _CRm, _op2) \
      (_op0 << op0) \
    | (_op1 << op1) \
    | (_CRn << CRn) \
    | (_CRm << CRm) \
    | (_op2 << op2) \

#define DBGDTR_EL0 MRSMSR_SYSREG(0b10, 0b011,      0, 0b0100,     0)
#define DLR_EL0    MRSMSR_SYSREG(0b11, 0b011, 0b0100, 0b0101, 0b001)
#define X0         0

#define AARCH64_I_MSR(__dstreg, __srcreg) \
  (0xd5100000 | __dstreg | (__srcreg & 0x1f))

#define AARCH64_I_MRS(__dstreg, __srcreg) \
  (0xd5300000 | __srcreg | (__dstreg & 0x1f))

/* 0xb8404401 */
#define AARCH64_LDR_POSTINC_32(__addrreg, __dstreg) \
  (0xb8400400 | ((4 & 0x1ff) << 12) | (__addrreg << 5)| __dstreg)

#define AARCH64_LDR_POSTINC_64(__addrreg, __dstreg) \
  (0xf8400400 | ((8 & 0x1ff) << 12) | (__addrreg << 5)| __dstreg)

static inline bool aarch64_set_mod_reg_bits(struct adiv5_dap *d,
  uint32_t regaddr, uint32_t *reg_cache, uint32_t mask, uint32_t value)
{
  uint32_t reg;
  uint32_t *preg = reg_cache ? reg_cache : &reg;
  uint32_t good_value;

  adiv5_mem_ap_read_word_e(d, regaddr, preg);
  good_value = ((*preg) & ~mask) | value;
  adiv5_mem_ap_write_word_e(d, regaddr, good_value);
  adiv5_mem_ap_read_word_e(d, regaddr, preg);

  return good_value == *preg;
}

static int aarch64_read_status_regs(struct adiv5_dap *d,
  struct aarch64_dbg_regs_cache *ed, uint32_t baseaddr, bool full)
{
  adiv5_mem_ap_read_word_e(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  if (!full)
    return 0;

  /* EDECR is not a status register */
  adiv5_mem_ap_read_word_e(d, baseaddr + DBG_REG_ADDR_EDECR, &ed->edecr);
  adiv5_mem_ap_read_word_e(d, baseaddr + DBG_REG_ADDR_EDESR, &ed->edesr);
  return 0;
}

bool aarch64_set_memory_mode(struct aarch64 *a, uint32_t baseaddr)
{
  bool success;

  if (aarch64_edscr_is_mem_mode(a->regs.edscr))
    return true;

  success = aarch64_set_mod_reg_bits(a->dap,
    baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr, 1<<20, 1<<20);

  if (success)
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
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
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);
  }

  return success;
}

static inline int aarc64_clear_stick_error_bits(struct aarch64 *a,
  uint32_t baseaddr)
{
  const uint32_t value =
      (1<<AARCH64_EDRCR_BIT_CSE)
    | (1<<AARCH64_EDRCR_BIT_CSPA);

  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDRCR, value);
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);
  return 0;
}

static inline int aarch64_check_handle_sticky_error(struct aarch64 *a,
  uint32_t baseaddr)
{
  int ret;

  ret = aarch64_read_status_regs(a->dap, &a->regs, baseaddr, false);
  if (ret)
    return ret;

  if (aarch64_edscr_is_error(a->regs.edscr)) {
    ret = aarc64_clear_stick_error_bits(a, baseaddr);
    if (ret)
      return ret;

    ret = aarch64_read_status_regs(a->dap, &a->regs, baseaddr, false);
    if (ret)
      return ret;
    return -EIO;
  }

  return 0;
}

int aarch64_check_halted(struct aarch64 *a, uint32_t baseaddr)
{
  int ret = -EAGAIN;
  int ret2;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
    &a->regs.edprsr);

  if (aarch64_edprsr_is_halted(a->regs.edprsr))
    ret = 0;

  ret2 = aarch64_check_handle_sticky_error(a, baseaddr);
  return ret2 ? ret2 : ret;
}

int aarch64_halt(struct aarch64 *a, uint32_t baseaddr, uint32_t cti_baseaddr)
{
  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 1<<EDSCR_HDE, 1<<EDSCR_HDE))
    return -EIO;

  /* HALT */
  cti_pulse_event(a->dap, cti_baseaddr, CTI_EVENT_HALT);

  do {
    /* Processor status register has info if core is halted or not */
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);
  } while (!aarch64_edprsr_is_halted(a->regs.edprsr));

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

static inline bool aarch64_ctx_get_reg(struct aarch64_context *ctx,
  uint32_t reg_id, uint64_t *out)
{
  if (reg_id <= AARCH64_CORE_REG_X30) {
    *out = ctx->x0_30[reg_id];
    return true;
  }
  if (reg_id == AARCH64_CORE_REG_PC) {
    *out = ctx->pc;
    return true;
  }
  if (reg_id == AARCH64_CORE_REG_SP) {
    *out = ctx->sp;
    return true;
  }

  return false;
}

static inline bool aarch64_ctx_set_reg(struct aarch64_context *ctx,
  uint32_t reg_id, uint64_t value)
{
  if (reg_id <= AARCH64_CORE_REG_X30) {
    ctx->x0_30[reg_id] = value;
    goto dirty;
  }
  if (reg_id == AARCH64_CORE_REG_PC) {
    ctx->pc = value;
    goto dirty;
  }
  if (reg_id == AARCH64_CORE_REG_SP) {
    ctx->sp = value;
    goto dirty;
  }

  return false;

dirty:
  ctx->dirty_mask |= 1ull << reg_id;
  return true;
}

int aarch64_restore_reg(struct aarch64 *a, uint32_t baseaddr, uint32_t reg_id)
{
  int ret;
  uint64_t value;
  uint64_t tmp_reg;

  if (!(a->ctx.dirty_mask & (1<<reg_id)))
    return 0;

  if (!aarch64_ctx_get_reg(&a->ctx, reg_id, &value))
    return -EIO;

  ret = aarch64_write_core_reg(a, baseaddr, reg_id, value);
  if (!ret)
    a->ctx.dirty_mask &= ~(1<<reg_id);

  /* Ignore request to restore PC, because it is restored from DLR_EL0 */
  if (reg_id == AARCH64_CORE_REG_PC) {
    ret = aarch64_read_core_reg(a, baseaddr, reg_id, &tmp_reg);
    if (!ret)
      return ret;
  }
  return ret;
}

int aarch64_restore_before_resume(struct aarch64 *a, uint32_t baseaddr)
{
  int ret;
  uint32_t i;

  if (!a->ctx.dirty_mask)
    return 0;

  /*
   * Because we are going to restore something, we are going to dirty X0, so
   * exclude it now to restore it as the last step
   */
  for (i = 1; i < AARCH64_CORE_REGS_COUNT; ++i) {
    ret = aarch64_restore_reg(a, baseaddr, i);
    if (ret)
      return ret;
  }
  ret = aarch64_restore_reg(a, baseaddr, AARCH64_CORE_REG_X0);
  if (ret)
    return ret;

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

int aarch64_resume(struct aarch64 *a, uint32_t baseaddr, uint32_t cti_baseaddr)
{
  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 1<<EDSCR_HDE, 1<<EDSCR_HDE))
    return -EIO;

  /* RESUME */
  cti_ungate_channel(a->dap, cti_baseaddr, 1);
  cti_gate_channel(a->dap, cti_baseaddr, 0);
  cti_ack(a->dap, cti_baseaddr);

#if 0
  do {
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);
  } while (aarch64_edprsr_is_halted(a->regs.edprsr));
#endif

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

int aarch64_step(struct aarch64 *a, uint32_t baseaddr, uint32_t cti_baseaddr)
{
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDESR,
    &a->regs.edesr);

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDECR,
    &a->regs.edecr, 1<<EDECR_BIT_SS, 1<<EDECR_BIT_SS))
    return -EIO;

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 3<<EDSCR_INTdis, 3<<EDSCR_INTdis))
    return -EIO;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDESR,
    &a->regs.edesr);
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
    &a->regs.edprsr);

  /* RESUME */
  cti_ack2(a->dap, cti_baseaddr, CTI_EVENT_HALT);
  cti_ungate_channel(a->dap, cti_baseaddr, 1);
  cti_gate_channel(a->dap, cti_baseaddr, 0);
  cti_pulse_event(a->dap, cti_baseaddr, CTI_EVENT_RESUME);

  do {
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);

    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDESR,
      &a->regs.edesr);

    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
      &a->regs.edscr);
  } while (!aarch64_edprsr_is_halted(a->regs.edprsr));

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 3<<EDSCR_INTdis, 0))
    return -EIO;

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDECR,
    &a->regs.edecr, 1<<EDECR_BIT_SS, 0))
    return -EIO;

  return 0;
}

int aarch64_breakpoint(struct aarch64 *a, uint32_t baseaddr, bool remove,
  bool hardware, uint64_t arg)
{
  if (hardware) {
    uint32_t feature_lo;
    uint32_t feature_hi;
    uint32_t ctrl;
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDDFR_LO,
      &feature_lo);
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDDFR_HI,
      &feature_hi);
    if (remove) {
    }
    else {
      ctrl = 1 | (3<<1) | (1<<13) | (0xf << 5);
      uint32_t addr_lo = arg & 0xffffffff;
      uint32_t addr_hi = (arg >> 32) & 0xffffffff;

      adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGBVR0_EL1,
        addr_lo);
      adiv5_mem_ap_write_word_e(a->dap,
        baseaddr + DBG_REG_ADDR_DBGBVR0_EL1 + 4, addr_hi);

      adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGBCR0_EL1,
        ctrl);
    }
  }
  return 0;
}

int aarch64_read_core_reg(struct aarch64 *a, uint32_t baseaddr, uint32_t reg,
  uint64_t *out_reg)
{
  uint32_t tmp_lo = 0, tmp_hi = 0;

  uint32_t instructions[2];
  int num_i;

  if (reg >= AARCH64_CORE_REG_X0 && reg <= AARCH64_CORE_REG_X30) {
    /* msr dbgdtr_el0, xN, where N==reg*/
    instructions[0] = AARCH64_I_MSR(DBGDTR_EL0, reg);
    num_i = 1;
    if (reg != AARCH64_CORE_REG_X0)
      a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else if (reg == AARCH64_CORE_REG_PC) {
    /* mrs x0, dlr_el0 */
    instructions[0] = AARCH64_I_MRS(X0, DLR_EL0);
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_I_MSR(DBGDTR_EL0, X0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else if (reg == AARCH64_CORE_REG_SP) {
    /* mov x0, sp */
    instructions[0] = 0x910003e0;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else if (reg == AARCH64_CORE_REG_SCTLR_EL1) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd5381000;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else if (reg == AARCH64_CORE_REG_ESR_EL2) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd53c5200;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else if (reg == AARCH64_CORE_REG_FAR_EL2) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd53c6000;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else if (reg == AARCH64_CORE_REG_DISR_EL1) {
    /* mrs x0, sctlr_el1 */
    instructions[0] = 0xd538c120;
    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else if (reg == AARCH64_CORE_REG_DSPSR_EL0) {
    instructions[0] = 0xd53b4500;
    instructions[1] = AARCH64_INSTR_MSR_DBGDTR_EL0(0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else
    return -EINVAL;

  for (int i = 0; i < num_i; ++i) {
    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
      instructions[i]);
  }

  while(1) {
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
      &a->regs.edscr);
    if (aarch64_edscr_is_error(a->regs.edscr))
      return -EIO;
    if (aarch64_edscr_is_tx_full(a->regs.edscr))
      break;
  }

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0,
    &tmp_hi);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
    &tmp_lo);

  *out_reg = (uint64_t)tmp_lo | ((uint64_t)tmp_hi << 32);
  return 0;
}

int aarch64_write_core_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t value)
{
  uint32_t instructions[2];
  int num_i;

  if (reg_id <= AARCH64_CORE_REG_X30) {
    /* msr dbgdtr_el0, xN, where N==reg*/
    instructions[0] = AARCH64_INSTR_MRS_DBGDTR_EL0(reg_id);
    num_i = 1;
  }
  else if (reg_id == AARCH64_CORE_REG_PC) {
    /* mrs x0, dbgdtr_el0, same as x0 = dbgdtr_el0 */
    instructions[0] = AARCH64_I_MRS(X0, DBGDTR_EL0);
    /* msr dlr_el0, x0 */
    instructions[1] = AARCH64_I_MSR(DLR_EL0, X0);
    num_i = 2;
    a->ctx.dirty_mask |= 1 << AARCH64_CORE_REG_X0;
  }
  else
    return -EINVAL;

  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0,
    value & 0xffffffff);

  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
    (value >> 32) & 0xffffffff);

  for (int i = 0; i < num_i; ++i) {
    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
      instructions[i]);
  }

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

int aarch64_write_cached_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t value)
{
  return aarch64_ctx_set_reg(&a->ctx, reg_id, value) ? 0 : -EIO;
}

int aarch64_read_cached_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t *value)
{
  return aarch64_ctx_get_reg(&a->ctx, reg_id, value) ? 0 : -EIO;
}

int aarch64_fetch_context(struct aarch64 *a, uint32_t baseaddr)
{
  int ret;
  uint64_t reg;

  struct aarch64_context *c = &a->ctx;

  c->dirty_mask = 0;

  for (int i = AARCH64_CORE_REG_X0; i <= AARCH64_CORE_REG_X30; ++i) {
    ret = aarch64_read_core_reg(a, baseaddr, i, &reg);
    if (ret)
      return ret;
    c->x0_30[i] = reg;
  }

  ret = aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_PC, &reg);
  if (ret)
    return ret;

  c->pc = reg;

  ret = aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_SP, &reg);
  if (ret)
    return ret;

  c->sp = reg;
  c->el = aarch64_edscr_get_el(a->regs.edscr);

  ret = aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_SCTLR_EL1, &reg);
  if (ret)
    return ret;

  c->sctlr_el1 = reg;
  c->mmu_on = reg & 1;

  ret = aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_DSPSR_EL0,
    &c->pstate);
  if (ret)
    return ret;

  return 0;
}

int aarch64_read_mem32(struct aarch64 *a, uint32_t baseaddr, uint64_t addr,
    uint32_t *dst, size_t num_words)
{
  int ret;

  uint32_t tmp;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr);
  if (ret)
    return ret;

  if (!aarch64_set_memory_mode(a, baseaddr))
    return -EIO;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &tmp);
  for (size_t i = 0; i < num_words; ++i)
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
      &dst[i]);
  return 0;
}

int aarch64_write_mem32(struct aarch64 *a, uint32_t baseaddr, uint64_t addr,
    const uint32_t *src, size_t num_words)
{
  int ret;
  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr);
  if (ret)
    return ret;

  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    AARCH64_INSTR_MSR_DBGDTR_EL0(0));

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  if (!aarch64_set_memory_mode(a, baseaddr))
    return -EIO;

  for (size_t i = 0; i < num_words; ++i) {
    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0,
      src[i]);
  }

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  return aarch64_edscr_is_error(a->regs.edscr) ? -EIO : 0;
}

int aarch64_read_mem_once(struct aarch64 *a, uint32_t baseaddr,
  mem_access_size_t access_size, uint64_t addr, void *out_value)
{
  int ret;
  uint64_t reg = 0;
  uint32_t instruction;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr);
  if (ret)
    return ret;

  if (access_size == MEM_ACCESS_SIZE_32)
    instruction = AARCH64_LDR_POSTINC_32(AARCH64_CORE_REG_X0,
      AARCH64_CORE_REG_X0);
  else if (access_size == MEM_ACCESS_SIZE_64)
    instruction = AARCH64_LDR_POSTINC_64(AARCH64_CORE_REG_X0,
      AARCH64_CORE_REG_X0);
  else 
    return -ENOTSUP;

  /*
   * There is an issue with some of the STR/LDR instructions that can be put
   * into EDITR to read/write memory. The ones that are compiled by GCC as
   * binary code for ldr w1, [x0] and str w1, [x0] lead to EDSCR.ERR bit set
   * to 1.
   * In Armv8 architectures manual there is a bytecode, used for
   * writing/reading values while accessing DBGDTRTX_EL0 and DBGDTRRX_EL0
   * regs. Also Openocd has precompiled bytecode for 'slow' reads.
   * These 2 work fine, so same bytecode I use here.
   * LDR(immediate) 0xb8404401
   * 10|111|0|00|01|0| 000000100 |01|00000|00001
   * size=10   opc=01  imm9=4   Rn=0   Rt=1
   * size = 10 - 32bit variant
   * LDR <Wt> [<Xn,SP>], #4
   */
  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    instruction);

  ret = aarch64_check_handle_sticky_error(a, baseaddr);
  if (ret)
    return ret;

  ret = aarch64_read_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, &reg);
  if (ret)
    return ret;

  if (access_size == MEM_ACCESS_SIZE_32)
    *(uint32_t *)out_value = (uint32_t)(reg & 0xffffffff);
  else
    *(uint64_t *)out_value = reg;

  return 0;
}

int aarch64_read_mem32_fast_start(struct aarch64 *a, uint32_t baseaddr,
  uint64_t addr)
{
  int ret;
  uint32_t tmp;
  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr);
  if (ret)
    return ret;

  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    AARCH64_I_MSR(DBGDTR_EL0, X0));

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  if (!aarch64_set_memory_mode(a, baseaddr))
    return -1;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &tmp);
  return 0;
}

int aarch64_read_mem32_fast_next(struct aarch64 *a, uint32_t baseaddr,
  uint32_t *value)
{
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
      value);
  return 0;
}

int aarch64_read_mem32_fast_stop(struct aarch64 *a, uint32_t baseaddr)
{
  return aarch64_set_normal_mode(a, baseaddr) ? 0 : -1;
}

int aarch64_write_mem32_once(struct aarch64 *a, uint32_t baseaddr,
  uint64_t addr, uint32_t value)
{
  int ret;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr);
  if (ret)
    return ret;

  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X1, value);
  if (ret)
    return ret;

  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    0xb8004401);
#if 0
  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    0xf8408400 | (5 << 5) | (1 << 0));
#endif
    // 0xb8404401);

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

int aarch64_exec(struct aarch64 *a, uint32_t baseaddr,
  const uint32_t *const instr, int num)
{
  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  for (int i = 0; i < num; ++i)
    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR, instr[i]);

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

int aarch64_init(struct aarch64 *a, struct adiv5_dap *d, uint32_t baseaddr,
  uint32_t cti_baseaddr)
{
  uint32_t reg;

  a->dap = d;
  /*
   * Unlock Debug lock, to verify - readback the value in EDPRSR.OSLK, with 0
   * as UNLOCKED. Only after OSLK unlock the rest of the debug registers can
   * be accessed in predictable expected way.
   */
  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_OSLAR_EL1, 0);
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_OSLAR_EL1, &reg);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
    &a->regs.edprsr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDLSR,
    &a->regs.edlsr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDESR,
    &a->regs.edesr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDECR,
    &a->regs.edecr);
  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDECR, 3);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDESR,
    &a->regs.edesr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDECR,
    &a->regs.edecr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDRCR,
    &a->regs.edrcr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDACR,
    &a->regs.edacr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDECCR,
    &a->regs.edacr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPCSR_LO, &reg);
  a->regs.sampled_pc = ((uint64_t)reg) & 0xffffffff;
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPCSR_HI, &reg);
  a->regs.sampled_pc |= ((uint64_t)reg) << 32;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDCIDSR,
    &a->regs.edcidsr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDVIDSR,
    &a->regs.edvidsr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
    &a->regs.edprsr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRCR,
    &a->regs.edprcr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_MIDR_EL1,
    &a->regs.midr_el1);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + 0xd38, &a->regs.memfeature0);
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + 0xd3c, &a->regs.memfeature1);
  cti_init(a->dap, cti_baseaddr);

  return 0;
}
