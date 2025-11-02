#include "aarch64.h"
#include "adiv5.h"
#include "cmsis_edi.h"
#include "arm_cti.h"
#include "aarch64_opcode.h"
#include "aarch64_edprsr.h"
#include "aarch64_edscr.h"
#include "common.h"
#include <errno.h>

static bool aarch64_reg_get_name(uint32_t reg_id, char *buf, size_t bufsz)
{
#define SET_AND_CHECK(__v) \
  *ptr++ = (__v); if (ptr == end) return false;

  char *ptr = buf;
  char *end = buf + bufsz;

  if (ptr == end)
    return false;

  if (reg_id >= AARCH64_CORE_REG_X0 && reg_id <= AARCH64_CORE_REG_X30) {
    SET_AND_CHECK('x');

    if (reg_id == AARCH64_CORE_REG_X30) {
      SET_AND_CHECK('3');
      SET_AND_CHECK('0');
    }
    else if (reg_id >= AARCH64_CORE_REG_X20) {
      SET_AND_CHECK('2');
      SET_AND_CHECK('0' + reg_id - AARCH64_CORE_REG_X20);
    } else if (reg_id >= AARCH64_CORE_REG_X10) {
      SET_AND_CHECK('1');
      SET_AND_CHECK('0' + reg_id - AARCH64_CORE_REG_X10);
    } else {
      SET_AND_CHECK('0' + reg_id);
    }
  }
  else if (reg_id == AARCH64_CORE_REG_PC) {
    SET_AND_CHECK('p');
    SET_AND_CHECK('c');
  }
  else if (reg_id == AARCH64_CORE_REG_SP) {
    SET_AND_CHECK('s');
    SET_AND_CHECK('p');
  }
  else
    return false;

  *ptr = 0;
#undef SET_AND_CHECK
  return true;
}

static void aarch64_regcache_reset_on_halt(struct aarch64 *a)
{
  int i;

  for (i = 0; i < ARRAY_SIZE(a->ctx.regcache); ++i)
    a->ctx.regcache[i].state = REG_STATE_INVALID;
}

static inline void aarch64_update_halt_reason(struct aarch64 *a)
{
  int status = aarch64_edscr_get_status(a->regs.edscr);

  if (status == EDSCR_STATUS_EXT_DEBUG_REQ)
    a->halt_reason = AARCH64_HALT_REASON_EXTERNAL_SIGNAL;
  else if (status == EDSCR_STATUS_HLT_INSTR)
    a->halt_reason = AARCH64_HALT_REASON_SW_HLT_INSTR;
  else if (status == EDSCR_STATUS_HLT_STEP_NORM)
    a->halt_reason = AARCH64_HALT_REASON_HALT_STEP;
  else if (status == EDSCR_STATUS_HLT_STEP_NO_SYNDROME)
    a->halt_reason = AARCH64_HALT_REASON_HALT_STEP_NO_SYNDROME;
  else
    a->halt_reason = AARCH64_HALT_REASON_UNKNOWN;
}

static inline int
__attribute__((unused))
aarch64_read_sampled_pc(struct aarch64 *a, uint32_t baseaddr)
{
  uint32_t r32;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPCSR_LO, &r32);
  a->regs.sampled_pc = ((uint64_t)r32) & 0xffffffff;
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPCSR_HI, &r32);
  a->regs.sampled_pc |= ((uint64_t)r32) << 32;
  return 0;
}

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

static int aarch64_status_update(struct aarch64 *a, uint32_t baseaddr)
{
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDESR,
    &a->regs.edesr);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
    &a->regs.edprsr);

  return 0;
}

static inline bool aarch64_status_halted(const struct aarch64 *a)
{
  return aarch64_edprsr_is_halted(a->regs.edprsr);
}

static bool aarch64_set_memory_mode(struct aarch64 *a, uint32_t baseaddr)
{
  bool success;

  if (aarch64_edscr_is_ma_set(a->regs.edscr))
    return true;

  success = aarch64_set_mod_reg_bits(a->dap,
    baseaddr + DBG_REG_ADDR_EDSCR, &a->regs.edscr, 1<<20, 1<<20);

  if (success)
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
      &a->regs.edprsr);

  return success;
}

static bool aarch64_set_normal_mode(struct aarch64 *a, uint32_t baseaddr)
{
  bool success;

  if (!aarch64_edscr_is_ma_set(a->regs.edscr))
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

static inline bool aarch64_status_is_exception_catch_pending(struct aarch64 *a)
{
  return aarch64_edesr_is_ec_set(a->regs.edesr);
}

static inline bool aarch64_status_is_sticky_err_set(struct aarch64 *a)
{
  return aarch64_edscr_is_err_set(a->regs.edscr);
}

static inline int aarch64_check_handle_sticky_error(struct aarch64 *a,
  uint32_t baseaddr)
{
  int ret;

  ret = aarch64_status_update(a, baseaddr);
  if (ret)
    return ret;

  if (aarch64_status_is_exception_catch_pending(a))
    return -EINTR;

  if (aarch64_status_is_sticky_err_set(a)) {
    ret = aarc64_clear_stick_error_bits(a, baseaddr);
    if (ret)
      return ret;

    ret = aarch64_status_update(a, baseaddr);
    if (ret)
      return ret;
    return -EIO;
  }

  return 0;
}

static int aarch64_regcache_mark_modified(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id)
{
  int ret;
  struct reg *r;

  if (reg_id > AARCH64_STATE_REGS)
    return -EINVAL;

  r = &a->ctx.regcache[reg_id];

  if (r->state == REG_STATE_INVALID) {
      ret = aarch64_reg_read_64(a, baseaddr, reg_id, &r->value);
      if (ret)
        return ret;
  }

  r->state = REG_STATE_MODIFIED;
  return 0;
}

static int aarch64_write_core_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t value)
{
  int ret;
  uint32_t instructions[2];
  int num_i;

  if (reg_id <= AARCH64_STATE_REGS) {
    ret = aarch64_regcache_mark_modified(a, baseaddr, reg_id);
    if (ret)
      return ret;
  }

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
    ret = aarch64_regcache_mark_modified(a, baseaddr, AARCH64_CORE_REG_X0);
    if (ret)
      return ret;
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


static int aarch64_on_debug_entry(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr)
{
  int ret;

  ret = aarc64_clear_stick_error_bits(a, baseaddr);
  if (ret)
    return ret;

  cti_ack2(a->dap, cti_baseaddr, CTI_EVENT_HALT);
  aarch64_regcache_reset_on_halt(a);
  aarch64_update_halt_reason(a);
  return 0;
}

void aarch64_get_halt_reason(struct aarch64 *a, const char **out)
{
  if (a->halt_reason == AARCH64_HALT_REASON_EXTERNAL_SIGNAL)
    *out = "Debug request";
  else if (a->halt_reason == AARCH64_HALT_REASON_SW_HLT_INSTR)
    *out = "HLT instruction";
  else if (a->halt_reason == AARCH64_HALT_REASON_HALT_STEP)
    *out = "HALT step";
  else if (a->halt_reason == AARCH64_HALT_REASON_HALT_STEP_NO_SYNDROME)
    *out = "HALT step (no syndrome)";
  else
    *out = "UNKNOWN";
}

int aarch64_check_halted(struct aarch64 *a, uint32_t baseaddr,
  uint32_t cti_baseaddr)
{
  int ret = -EAGAIN;
  int status;

  ret = aarch64_status_update(a, baseaddr);
  if (ret)
    return ret;

  if (aarch64_status_is_sticky_err_set(a)) {
    ret = aarc64_clear_stick_error_bits(a, baseaddr);
    if (ret)
      return ret;

    ret = aarch64_status_update(a, baseaddr);
    if (ret)
      return ret;

    return -EIO;
  }

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDPRSR,
    &a->regs.edprsr);

  ret = aarch64_edprsr_is_halted(a->regs.edprsr) ? 0 : -EAGAIN;

  status = aarch64_edscr_get_status(a->regs.edscr);
  if (status == EDSCR_STATUS_EXCEPT_CATCH)
    return -EINTR;

  if (!ret)
    ret = aarch64_on_debug_entry(a, baseaddr, cti_baseaddr);

  return ret;
}

int aarch64_halt(struct aarch64 *a, uint32_t baseaddr, uint32_t cti_baseaddr)
{
  int ret;

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

  aarch64_regcache_reset_on_halt(a);
  ret = aarch64_check_handle_sticky_error(a, baseaddr);
  aarch64_update_halt_reason(a);
  return ret;
}

static inline int aarch64_regcache_read(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t *out)
{
  int ret;
  struct reg *r;

  if (reg_id <= AARCH64_STATE_REGS) {
    r = &a->ctx.regcache[reg_id];

    if (r->state == REG_STATE_INVALID) {
      ret = aarch64_reg_read_64(a, baseaddr, reg_id, &r->value);
      if (ret)
        return ret;
      r->state = REG_STATE_CLEAN;
    }

    *out = r->value;
    return 0;
  }

  if (reg_id == AARCH64_CORE_REG_CPSR) {
    *out = a->ctx.pstate;
    return 0;
  }

  if (reg_id == AARCH64_CORE_REG_FPSR) {
    *out = a->ctx.fpsr;
    return 0;
  }

  if (reg_id == AARCH64_CORE_REG_FPCR) {
    *out = a->ctx.fpcr;
    return 0;
  }

  return -EINVAL;
}

int aarch64_restore_reg(struct aarch64 *a, uint32_t baseaddr, uint32_t reg_id)
{
  int ret;
  struct reg *r;

  if (reg_id > AARCH64_STATE_REGS)
    return -EINVAL;

  r = &a->ctx.regcache[reg_id];

  if (r->state != REG_STATE_MODIFIED)
    return 0;

  ret = aarch64_write_core_reg(a, baseaddr, reg_id, r->value);
  if (ret)
    return ret;

  r->state = REG_STATE_CLEAN;
  return ret;
}

int aarch64_restore_before_resume(struct aarch64 *a, uint32_t baseaddr)
{
  int ret;
  uint32_t i;

  /*
   * Because we are going to restore something, we are going to dirty X0, so
   * exclude it now to restore it as the last step
   */
  for (i = AARCH64_CORE_REG_X1; i < AARCH64_STATE_REGS; ++i) {
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

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

int aarch64_step(struct aarch64 *a, uint32_t baseaddr, uint32_t cti_baseaddr)
{
  int ret;

  ret = aarch64_status_update(a, baseaddr);
  if (ret)
    return ret;

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDECR,
    &a->regs.edecr, 1<<EDECR_BIT_SS, 1<<EDECR_BIT_SS))
    return -EIO;

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 3<<EDSCR_INTdis, 3<<EDSCR_INTdis))
    return -EIO;

  /* RESUME */
  cti_ack2(a->dap, cti_baseaddr, CTI_EVENT_HALT);
  cti_ungate_channel(a->dap, cti_baseaddr, 1);
  cti_gate_channel(a->dap, cti_baseaddr, 0);
  cti_pulse_event(a->dap, cti_baseaddr, CTI_EVENT_RESUME);

  while (1) {
    ret = aarch64_status_update(a, baseaddr);
    if (ret)
      return ret;

    if (aarch64_status_halted(a))
      break;
  };

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr, 3<<EDSCR_INTdis, 0))
    return -EIO;

  if (!aarch64_set_mod_reg_bits(a->dap, baseaddr + DBG_REG_ADDR_EDECR,
    &a->regs.edecr, 1<<EDECR_BIT_SS, 0))
    return -EIO;

  ret = aarch64_status_update(a, baseaddr);
  if (ret)
    return ret;

  aarch64_regcache_reset_on_halt(a);
  aarch64_update_halt_reason(a);

  return aarch64_check_handle_sticky_error(a, baseaddr);
}

static int aarch64_get_cache_line_info(struct aarch64 *a, uint32_t baseaddr,
    int *dcache_line_size, int *icache_line_size)
{
  int ret;
  uint64_t ctr_el0;

  if (!a->cache_line_fetched) {
    ret = aarch64_reg_read_64(a, baseaddr, AARCH64_CORE_REG_CTR_EL0, &ctr_el0);
    if (ret)
      return ret;

    a->icache_line_sz = 4ul << (ctr_el0 & 0xf);
    a->dcache_line_sz = 4ul << ((ctr_el0 >> 16) & 0xf);
  }

  *dcache_line_size = a->dcache_line_sz;
  *icache_line_size = a->icache_line_sz;

  return 0;
}

static int aarch64_breakpoint_sw_cache_sync(struct aarch64 *a,
  uint32_t baseaddr, uint64_t addr)
{
  int ret;

  int dcache_line_sz;
  int icache_line_sz;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  uint32_t dc_civac_x0 = OPCODE_DC_CIVAC_X0;

  const uint32_t icache_sync[] = {
    /* Enforces order (not sure if this is required in debug */
    OPCODE_DSB_ISH,
    /* Reset instruction cache */
    OPCODE_IC_IVAU_X0,
    /* Enforces order (not sure if this is required in debug */
    OPCODE_DSB_ISH,
    /* Resets instruction pipeline */
    OPCODE_ISB
  };

  if (!a->ctx.mmu_on)
    return 0;

  ret = aarch64_get_cache_line_info(a, baseaddr, &dcache_line_sz,
    &icache_line_sz);

  if (ret)
    return ret;

  /* Only required if MMU is enabled, so caches are enabled as well */
  /* NOTE: Need to check, maybe ISB ISH are needed if MMU is not enable */
  const uint64_t dcache_addr_mask = ~(uint64_t)(a->dcache_line_sz - 1);
  const uint64_t icache_addr_mask = ~(uint64_t)(a->icache_line_sz - 1);
  const uint64_t dcache_line_addr = addr & dcache_addr_mask;
  const uint64_t icache_line_addr = addr & icache_addr_mask;

  /*
   * instruction bytes were written to cached memory, so they are in DATA cache
   * now cache, not in INSTRUCTION cache, so we first have to flush them to
   * system memory, then we need to invalidate instruction cache
   */

  /* X0 = D-cache line address for instruction address */
  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0,
    dcache_line_addr);
  if (ret)
    return ret;

  /* Write instruction from DATA cache to physical memory */
  ret = aarch64_exec(a, baseaddr, &dc_civac_x0, 1);
  if (ret)
    return ret;

  /* X0 = I-cache line address for instruction address */
  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0,
    icache_line_addr);
  if (ret)
    return ret;

  /* Synchronize order, reset I-cache and reset instruction pipeline */
  return aarch64_exec(a, baseaddr, icache_sync, ARRAY_SIZE(icache_sync));
}

static int aarch64_breakpoint_sw(struct aarch64 *a, uint32_t baseaddr,
  struct breakpoint *b, bool restore)
{
  int ret;
  uint32_t test_instr;
  uint32_t instr;

  if (b->addr & 3ul)
    return -EINVAL;

  if (!restore) {
    /*
     * We are inserting breakpoint, load previous instruction, which we are
     * overwriting to restore it next time.
     */
    ret = aarch64_read_mem_once(a, baseaddr, MEM_ACCESS_SIZE_32, b->addr,
      &b->instr);
    if (ret)
      return ret;

    instr = AARCH64_HLT(11);
  }
  else
    instr = b->instr;

  ret = aarch64_write_mem32_once(a, baseaddr, b->addr, instr);
  if (ret)
    return ret;

  ret = aarch64_breakpoint_sw_cache_sync(a, baseaddr, b->addr);
  if (ret)
    return ret;

  /* test */
  ret = aarch64_read_mem_once(a, baseaddr, MEM_ACCESS_SIZE_32, b->addr,
    &test_instr);
  if (ret)
    return ret;

  if (test_instr != instr)
    return -16;

  return ret;
}

static int aarch64_breakpoint_hw(struct aarch64 *a, uint32_t baseaddr,
  struct breakpoint *b, bool remove)
{
  uint32_t feature_lo;
  uint32_t feature_hi;
  uint32_t ctrl;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDDFR_LO,
    &feature_lo);
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDDFR_HI,
    &feature_hi);
  if (remove) {
    ctrl = 0;
    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGBCR0_EL1,
      ctrl);
  }
  else {
    ctrl = 1 | (3<<1) | (1<<13) | (0xf << 5);
    uint32_t addr_lo = b->addr & 0xffffffff;
    uint32_t addr_hi = (b->addr >> 32) & 0xffffffff;

    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGBVR0_EL1,
      addr_lo);
    adiv5_mem_ap_write_word_e(a->dap,
      baseaddr + DBG_REG_ADDR_DBGBVR0_EL1 + 4, addr_hi);

    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGBCR0_EL1,
      ctrl);
  }
  return 0;
}

int aarch64_breakpoint(struct aarch64 *a, uint32_t baseaddr, bool remove,
  bool hardware, struct breakpoint *b)
{
  if (hardware)
    return aarch64_breakpoint_hw(a, baseaddr, b, remove);
  return aarch64_breakpoint_sw(a, baseaddr, b, remove);
}

int aarch64_reg_read_64(struct aarch64 *a, uint32_t baseaddr, uint32_t reg,
  uint64_t *out_reg)
{
  int ret;
  uint32_t tmp_lo = 0, tmp_hi = 0;

  uint32_t instructions[2];
  int num_i;

  if (reg >= AARCH64_CORE_REG_X0 && reg <= AARCH64_CORE_REG_X30) {
    /* msr dbgdtr_el0, xN, where N==reg*/
    instructions[0] = AARCH64_I_MSR(DBGDTR_EL0, reg);
    num_i = 1;
  }
  else {
    if (!aarch64_get_mrs_opcode(&instructions[0], reg))
      return -EINVAL;

    /* msr dbgdtr_el0, x0 */
    instructions[1] = AARCH64_I_MSR(DBGDTR_EL0, X0);
    num_i = 2;
    ret = aarch64_regcache_mark_modified(a, baseaddr, AARCH64_CORE_REG_X0);
    if (ret)
      return ret;
  }

  for (int i = 0; i < num_i; ++i) {
    adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
      instructions[i]);
  }

  while(1) {
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
      &a->regs.edscr);
    if (aarch64_edscr_is_err_set(a->regs.edscr))
      return -EIO;
    if (aarch64_edscr_is_tx_full_set(a->regs.edscr))
      break;
  }

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0,
    &tmp_hi);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
    &tmp_lo);

  *out_reg = (uint64_t)tmp_lo | ((uint64_t)tmp_hi << 32);
  return 0;
}

int aarch64_write_cached_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t value)
{
  struct reg *r;

  if (reg_id > AARCH64_STATE_REGS)
    return -EIO;

  r = &a->ctx.regcache[reg_id];

  if (r->value != value || r->state != REG_STATE_CLEAN) {
    r->value = value;
    r->state = REG_STATE_MODIFIED;
  }

  return 0;
}

int aarch64_read_cached_reg(struct aarch64 *a, uint32_t baseaddr,
  uint32_t reg_id, uint64_t *value)
{
  return aarch64_regcache_read(a, baseaddr, reg_id, value);
}

int aarch64_iter_state_regs(struct aarch64 *a, uint32_t baseaddr,
  reg_iter_cb_t cb, void *cb_arg)
{
  int ret;
  uint32_t reg_id;
  char regname[4];
  uint64_t value;

  for (reg_id = 0; reg_id < AARCH64_STATE_REGS; ++reg_id) {
    ret = aarch64_regcache_read(a, baseaddr, reg_id, &value);
    if (ret)
      return ret;

    if (!aarch64_reg_get_name(reg_id, regname, sizeof(regname)))
      return -EFAULT;
    cb(regname, value, cb_arg);
  }
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

  return aarch64_edscr_is_err_set(a->regs.edscr) ? -EIO : 0;
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

  ret = aarch64_reg_read_64(a, baseaddr, AARCH64_CORE_REG_X0, &reg);
  if (ret)
    return ret;

  if (access_size == MEM_ACCESS_SIZE_32)
    *(uint32_t *)out_value = (uint32_t)(reg & 0xffffffff);
  else
    *(uint64_t *)out_value = reg;

  return 0;
}

int aarch64_mem_read_fast(struct aarch64 *a, uint32_t baseaddr,
  mem_access_size_t access_size, uint64_t addr, size_t count,
  void (*cb)(uint64_t, mem_access_size_t))
{
  int ret;
  uint64_t value;
  uint32_t *ptr;

  if (!aarch64_set_normal_mode(a, baseaddr))
    return -EIO;

  /* Set starting address to X0 */
  ret = aarch64_write_core_reg(a, baseaddr, AARCH64_CORE_REG_X0, addr);
  if (ret)
    return ret;

  /*
   * Prepare for external read from DBGDTRTX_EL0.
   * We need to set TXfull to 1 before going into memory mode and trying to
   * read, otherwise on read we will receive value 0xffff0000, and EDSCR.TXU
   * (TX underrun bit) set to 1.
   * Arm Architecture Reference Manual for A-profile architecture
   * The Debug Communication Channel and Instruction Transfer Register
   * H4.3 DCC and ITR access modes
   * Read carefully:
   * - H4.4.3 Overrun and underrun flags
   * - Table H4-1 DCC and ITR ready flags and the associated overrun/underrun
   *   flags
   */
  adiv5_mem_ap_write_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDITR,
    AARCH64_I_MSR(DBGDTR_EL0, X0));

  if (!aarch64_set_memory_mode(a, baseaddr))
    return -1;

  /*
   * Arm Architecture Reference Manual for A-profile architecture
   * The Debug Communication Channel and Instruction Transfer Register
   * H4.3 DCC and ITR access modes
   * External reads from DBGDTRTX_EL0 cause:
   * 1. Existing value in DTRTX to be returned. This clears EDSCR.TXfull to 0
   * 2. Equivalent to LDR W1, [X0], #4 in AArch64, or LDR R1, [R0], #4 AArch32
   * 3. Equivalent to MSR DBGDTRTX_EL0, X1
   * 4. EDSCR.TXfull,ITE to be set to {1,1} and X1, R1 to be set to UNKNOWN
   */

  /*
   * First read from DBGDTRTX_EL0 returns older value but will also trigger:
   * LDR W1, [X0], #4
   * MSR DBGDTRTX_EL0, X1
   * Which will set DBGDTRTX_EL0 with first 4 bytes of addr
   */
  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
    (uint32_t *)&value);


  ret = aarch64_regcache_mark_modified(a, baseaddr, AARCH64_CORE_REG_X0);
  if (ret)
    return ret;
  ret = aarch64_regcache_mark_modified(a, baseaddr, AARCH64_CORE_REG_X1);
  if (ret)
    return ret;

  /*
   * Next read from DBGDTRTX_EL0 will trigger first 4 bytes and then next 4
   * bytes, etc.
   * 2 Details:
   * For 64 bit reads we will have to go in pairs of 2x 32bit reads
   * For the last 4 byte read we don't want to be in memory mode because we
   * don't want to fill DBGDTRTX_EL0 with next memory value. First this would
   * generate 1 extra memory access that the user has not requested
   * Second, this will fill DBGDTRTX_EL0 with new value that will be received
   * by next reader of this register, this causes some nasty bugs
   */
  for (int i = 0; i < count - 1; ++i) {
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
      (uint32_t *)&value);
    if (access_size == MEM_ACCESS_SIZE_64) {
      adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
        ((uint32_t *)&value) + 1);
    }
    cb(value, access_size);
  }

  ptr = (uint32_t *)&value;

  if (access_size == MEM_ACCESS_SIZE_64)
    adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0,
      ptr++);

  if (!aarch64_set_normal_mode(a, baseaddr))
    return -1;

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, ptr);

  cb(value, access_size);

  adiv5_mem_ap_read_word_e(a->dap, baseaddr + DBG_REG_ADDR_EDSCR,
    &a->regs.edscr);

  return 0;
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

  a->cache_line_fetched = false;
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
