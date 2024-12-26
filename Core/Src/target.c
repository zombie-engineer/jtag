#include "target.h"
#include "adiv5.h"
#include "jtag.h"
#include "cmsis_edi.h"
#include "arm_cmsis.h"
#include "target_arm.h"
#include "arm_cti.h"
#include "cmsis_arch_id.h"

void target_set_memory_mode(struct target *t)
{
  uint32_t base = t->core[0].debug;
  struct ext_dbg_aarch64 *edi = &t->core[0].edi;
  
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDSCR, edi->edscr | (1 << 20));
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
}

void target_set_normal_mode(struct target *t)
{
  uint32_t base = t->core[0].debug;
  struct ext_dbg_aarch64 *edi = &t->core[0].edi;

  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDSCR, edi->edscr & ~(1 << 20));
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
}

void target_halt(struct target *t)
{
  uint32_t base = t->core[0].debug;
  struct ext_dbg_aarch64 *edi = &t->core[0].edi;

  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDESR, &edi->edesr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDECR, &edi->edecr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDSCR, edi->edscr | (1<<14));
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);

  /* HALT */
  cti_pulse_event(&t->dap, t->core[0].cti, CTI_EVENT_HALT);
  while(1) {
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
    if ((edi->edprsr >> 4) & 1)
      break;
  }

  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDESR, &edi->edesr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDECR, &edi->edecr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  if (edi->edscr & (1<<6)) {
    adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDRCR, (1<<2)|(1<<3));
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  }
}

void target_resume(struct target *t)
{
  struct ext_dbg_aarch64 *edi = &t->core[0].edi;
  uint32_t base = t->core[0].debug;

  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDESR, &edi->edesr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDECR, &edi->edecr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDSCR, edi->edscr | (1<<14));
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);

  /* RESUME */
  cti_ungate_channel(&t->dap, t->core[0].cti, 1);
  cti_gate_channel(&t->dap, t->core[0].cti, 0);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
  cti_ack(&t->dap, t->core[0].cti);

  while(1) {
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDESR, &edi->edesr);
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDECR, &edi->edecr);
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
    if (((edi->edprsr >> 4) & 1) == 0)
      break;
  }

  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDESR, &edi->edesr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDECR, &edi->edecr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
}

void target_read(struct target *t)
{
  uint32_t reg = 0;
  struct ext_dbg_aarch64 *edi = &t->core[0].edi;
  uint32_t base = t->core[0].debug;

#if 0
  adiv5_transfer(AP, OP_READ, AP_REG_ADDR_CSW, 0, &dap.csw);
  dap.csw &= ~(3<<4);
  dap.csw &= ~7;
  dap.csw |= 3;
  adiv5_transfer(AP, OP_WRITE, AP_REG_ADDR_CSW, dap.csw, &dap.csw);
  adiv5_transfer(AP, OP_READ, AP_REG_ADDR_CSW, 0, &dap.csw);

  adiv5_mem_ap_write_word(base + DBG_REG_ADDR_EDLAR, 0xc5acce55);
#endif
  target_set_normal_mode(t);
  // adiv5_mem_ap_write_word(base + DBG_REG_ADDR_EDITR, 0xd2a00100);

  /* mrs x0, dlr_el0 */
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDITR, 0xd53b4520);

  /* msr dbgdtrtx_el0, x0 */
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDITR, 0xd5130500);

  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);

  /* add x0, x0, #4 */
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDITR, 0x91001000);

  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
  /* msr dlr_el0, x0 */
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDITR, 0xd51b4520);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);

  return;
  // adiv5_mem_ap_write_word(base + DBG_REG_ADDR_EDITR, 0xd5130500);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
  target_set_memory_mode(t);

  while(1) {
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
    if (reg == 0xd518c000) {
    }
  }


  /*
   * reg = *DTRTX
   * LDR W1, [X0], #4
   * MSR W1, DBGDTRTX_EL0
   */

  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_DBGDTRRX_EL0, 0x80000);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDSCR, edi->edscr & ~(1 << 20));
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);

  // adiv5_mem_ap_write_word(base + DBG_REG_ADDR_DBGDTRTX_EL0, 0x3fffffff);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDLSR, &edi->edlsr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDRCR, &edi->edrcr);
  edi->edrcr |= 1<<3;
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDRCR, edi->edrcr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  adiv5_mem_ap_write_word(&t->dap, base + DBG_REG_ADDR_EDITR, 0x52800020);
  adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
  while(1) {
    adiv5_mem_ap_read_word_drw(&t->dap, base + DBG_REG_ADDR_DBGDTRTX_EL0, &reg);
    adiv5_mem_ap_read_word_drw(&t->dap, base + DBG_REG_ADDR_DBGDTRRX_EL0, &reg);
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDESR, &edi->edesr);
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDSCR, &edi->edscr);
    adiv5_mem_ap_read_word(&t->dap, base + DBG_REG_ADDR_EDPRSR, &edi->edprsr);
  }
}


#define ADIV5_MAX_ROM_ENTRIES 32

static void arm_cmsis_mem_ap_examine(struct target *t, uint32_t baseaddr)
{
  int core_idx;
  int arch_id;

  struct arm_cmsis_regs cmsis = { 0 };

  cmsis_read_regs(&t->dap, baseaddr, &cmsis);

  core_idx = cmsis.devaff0 & 0xf;
  arch_id = cmsis.devarch & 0xffff;

  if (arch_id == CMSIS_ARCH_ID_PROCESSOR_DEBUG_V8_0_A) {
    t->core[core_idx].debug = baseaddr;
    t->core[core_idx].debug_exists = true;
  } else if (arch_id == CMSIS_ARCH_ID_CTI) {
    t->core[core_idx].cti = baseaddr;
    t->core[core_idx].cti_exists = true;
  } else if (arch_id == CMSIS_ARCH_ID_PMU) {
    t->core[core_idx].pmu = baseaddr;
    t->core[core_idx].pmu_exists = true;
  } else if (arch_id == CMSIS_ARCH_ID_ETM) {
    t->core[core_idx].etm = baseaddr;
    t->core[core_idx].etm_exists = true;
  }
  // adiv5_mem_ap_read_word(baseaddr + 0x314, &reg0);
  //adiv5_mem_ap_read_word(baseaddr + 0x088, &reg1);
}

static void target_parse_rom(struct target *t)
{
  int i;
  uint32_t mem_addr;
  uint32_t rom_entry;
  uint32_t rom_entries[ADIV5_MAX_ROM_ENTRIES] = { 0 };

  for (i = 0; i < ADIV5_MAX_ROM_ENTRIES; ++i) {
    adiv5_mem_ap_read_word(&t->dap, t->dap.base + i * 4, &rom_entries[i]);
    if (!rom_entries[i])
      break;
  }

  for (i = 0; i < ADIV5_MAX_ROM_ENTRIES; ++i) {
    rom_entry = rom_entries[i];
    if (!rom_entry)
      break;

    mem_addr = t->dap.base + (rom_entry & 0xfffff000);
    arm_cmsis_mem_ap_examine(t, mem_addr);
  }
}

bool target_init(struct target *t)
{
  int i;
  int num_devs;

  jtag_init();
  jtag_reset();

  num_devs = jtag_scan_num_devs();
  jtag_read_idcode();
  adiv5_dap_init(&t->dap);

  target_parse_rom(t);

  t->core[0].cti = 0x80018000;
  t->core[1].cti = 0x80019000;
  t->core[2].cti = 0x8001a000;
  t->core[3].cti = 0x8001b000;

  for (i = 0; i < 4; ++i)
    target_arm_init(&t->dap, t->core[i].debug, &t->core[i].edi);

  cti_init(&t->dap, t->core[0].cti);
  while(1) {
    target_halt(t);
    target_read(t);
    target_resume(t);
  }

  return true;
}

