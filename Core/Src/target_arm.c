#include "target_arm.h"
#include "adiv5.h"
#include "cmsis_edi.h"
#include "arm_cti.h"

static inline bool target_arm_set_mod_reg_bits(struct adiv5_dap *d,
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

static void target_arm_read_status_regs(struct adiv5_dap *d,
  struct ext_dbg_aarch64 *ed, uint32_t baseaddr)
{
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &ed->edesr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &ed->edecr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
}

bool target_arm_set_memory_mode(struct adiv5_dap *d,
  struct ext_dbg_aarch64 *ed, uint32_t baseaddr)
{
  bool success = target_arm_set_mod_reg_bits(d, baseaddr + DBG_REG_ADDR_EDSCR,
    &ed->edscr, 1<<20, 1<<20);

  if (success)
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);

  return success;
}

bool target_arm_set_normal_mode(struct adiv5_dap *d,
  struct ext_dbg_aarch64 *ed, uint32_t baseaddr)
{
  bool success = target_arm_set_mod_reg_bits(d, baseaddr + DBG_REG_ADDR_EDSCR,
    &ed->edscr, 1<<20, 0);

  if (success) {
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
  }

  return success;
}

bool target_arm_halt(struct adiv5_dap *d, struct ext_dbg_aarch64 *ed,
  uint32_t baseaddr, uint32_t cti_baseaddr)
{
  if (!target_arm_set_mod_reg_bits(d, baseaddr + DBG_REG_ADDR_EDSCR,
    &ed->edscr, 1<<14, 1<<14))
    return false;

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &ed->edecr);

  /* HALT */
  cti_pulse_event(d, cti_baseaddr, CTI_EVENT_HALT);
  while(1) {
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
    if ((ed->edprsr >> 4) & 1)
      break;
  }

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &ed->edesr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &ed->edecr);
  if (ed->edscr & (1<<6)) {
    adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDRCR, (1<<2)|(1<<3));
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  }

  target_arm_read_status_regs(d, ed, baseaddr);
  return true;
}

bool target_arm_resume(struct adiv5_dap *d, struct ext_dbg_aarch64 *ed,
  uint32_t baseaddr, uint32_t cti_baseaddr)
{
  if (!target_arm_set_mod_reg_bits(d, baseaddr + DBG_REG_ADDR_EDSCR,
    &ed->edscr, 1<<14, 1<<14))
    return false;

  /* RESUME */
  cti_ungate_channel(d, cti_baseaddr, 1);
  cti_gate_channel(d, cti_baseaddr, 0);
  cti_ack(d, cti_baseaddr);

  while(1) {
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
    if (((ed->edprsr >> 4) & 1) == 0)
      break;
  }

  target_arm_read_status_regs(d, ed, baseaddr);
  return true;
}

void target_arm_mess(struct adiv5_dap *d, struct ext_dbg_aarch64 *ed,
  uint32_t baseaddr, uint32_t cti_baseaddr)
{
  uint32_t reg = 0;

  if (!target_arm_set_normal_mode(d, ed, baseaddr))
    return;

  /* mrs x0, dlr_el0 */
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, 0xd53b4520);

  /* msr dbgdtrtx_el0, x0 */
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, 0xd5130500);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);

  /* add x0, x0, #4 */
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, 0x91001000);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
  /* msr dlr_el0, x0 */
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, 0xd51b4520);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);

  return;
  // adiv5_mem_ap_write_word(baseaddr + DBG_REG_ADDR_EDITR, 0xd5130500);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
  target_arm_set_memory_mode(d, ed, baseaddr);

  while(1) {
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
    if (reg == 0xd518c000) {
    }
  }


  /*
   * reg = *DTRTX
   * LDR W1, [X0], #4
   * MSR W1, DBGDTRTX_EL0
   */

  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, 0x80000);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDSCR, ed->edscr & ~(1 << 20));
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);

  // adiv5_mem_ap_write_word(baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, 0x3fffffff);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDLSR, &ed->edlsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDRCR, &ed->edrcr);
  ed->edrcr |= 1<<3;
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDRCR, ed->edrcr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  while(1) {
    adiv5_mem_ap_read_word_drw(d, baseaddr + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
    adiv5_mem_ap_read_word_drw(d, baseaddr + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &ed->edesr);
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
    adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
  }
}

void target_arm_exec_instr(struct adiv5_dap *d, struct ext_dbg_aarch64 *ed,
  uint32_t baseaddr, uint32_t instr)
{
  if (!target_arm_set_normal_mode(d, ed, baseaddr))
    return;

  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDITR, instr);
}

void target_arm_init(struct adiv5_dap *d, struct ext_dbg_aarch64 *ed,
    uint32_t baseaddr, uint32_t cti_baseaddr)
{
  uint32_t reg;
  /*
   * Unlock Debug lock, to verify - readback the value in EDPRSR.OSLK, with 0
   * as UNLOCKED. Only after OSLK unlock the rest of the debug registers can
   * be accessed in predictable expected way.
   */
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_OSLAR_EL1, 0);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_OSLAR_EL1, &reg);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);

  /*
   * Contents of EDLSR show if software lock is implemented and se
   */
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDLSR, &ed->edlsr);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &ed->edesr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &ed->edecr);
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDECR, 3);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &ed->edesr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &ed->edecr);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &ed->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDRCR, &ed->edrcr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDACR, &ed->edacr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECCR, &ed->edacr);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPCSR_LO, &reg);
  ed->sampled_pc = ((uint64_t)reg) & 0xffffffff;
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPCSR_HI, &reg);
  ed->sampled_pc |= ((uint64_t)reg) << 32;

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDCIDSR, &ed->edcidsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDVIDSR, &ed->edvidsr);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &ed->edprsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRCR, &ed->edprcr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_MIDR_EL1, &ed->midr_el1);
  adiv5_mem_ap_read_word(d, baseaddr + 0xd38, &ed->memfeature0);
  adiv5_mem_ap_read_word(d, baseaddr + 0xd3c, &ed->memfeature1);
  cti_init(d, cti_baseaddr);
}


