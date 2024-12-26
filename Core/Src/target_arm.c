#include "target_arm.h"
#include "adiv5.h"
#include "cmsis_edi.h"

void target_arm_init(struct adiv5_dap *d, uint32_t baseaddr,
  struct ext_dbg_aarch64 *edi)
{
  uint32_t reg;
  /*
   * Unlock Debug lock, to verify - readback the value in EDPRSR.OSLK, with 0
   * as UNLOCKED. Only after OSLK unlock the rest of the debug registers can
   * be accessed in predictable expected way.
   */
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_OSLAR_EL1, 0);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_OSLAR_EL1, &reg);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &edi->edscr);

  /*
   * Contents of EDLSR show if software lock is implemented and se
   */
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDLSR, &edi->edlsr);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &edi->edesr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &edi->edecr);
  adiv5_mem_ap_write_word(d, baseaddr + DBG_REG_ADDR_EDECR, 3);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDESR, &edi->edesr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECR, &edi->edecr);

#if 0
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDWAR0, &edi->edwar);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDWAR1, &edi->edwar_hi);
#endif

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDRCR, &edi->edrcr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDACR, &edi->edacr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDECCR, &edi->edacr);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPCSR_LO, &reg);
  edi->sampled_pc = ((uint64_t)reg) & 0xffffffff;
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPCSR_HI, &reg);
  edi->sampled_pc |= ((uint64_t)reg) << 32;

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDCIDSR, &edi->edcidsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDVIDSR, &edi->edvidsr);

  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPRCR, &edi->edprcr);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_MIDR_EL1, &edi->midr_el1);
#if 0
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPFR_LO, &edi->edpfr_lo);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDPFR_HI, &edi->edpfr_hi);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDDFR_LO, &edi->eddfr_lo);
  adiv5_mem_ap_read_word(d, baseaddr + DBG_REG_ADDR_EDDFR_HI, &edi->eddfr_hi);
#endif
  adiv5_mem_ap_read_word(d, baseaddr + 0xd38, &edi->memfeature0);
  adiv5_mem_ap_read_word(d, baseaddr + 0xd3c, &edi->memfeature1);
}


