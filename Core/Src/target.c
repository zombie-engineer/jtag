#include "target.h"
#include "adiv5.h"
#include "jtag.h"
#include "cmsis_edi.h"
#include "arm_cmsis.h"
#include "target_arm.h"
#include "cmsis_arch_id.h"

#define ADIV5_MAX_ROM_ENTRIES 32
#define ARRAY_SIZE(_a) (sizeof(_a)/sizeof(_a[0]))

bool target_halt(struct target *t)
{
  return target_arm_halt(&t->dap, &t->core[0].edi, t->core[0].debug,
    t->core[0].cti);
}

bool target_resume(struct target *t)
{
  return target_arm_resume(&t->dap, &t->core[0].edi, t->core[0].debug,
    t->core[0].cti);
}


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

bool target_exec_instr(struct target *t, uint32_t instr)
{
  target_arm_exec_instr(&t->dap, &t->core[0].edi, t->core[0].debug, instr);
}

void raspberrypi_soft_reset(struct target *t)
{
  size_t i;
    /* Raspberry PI reset sequence */
    /* 0x3f100024, (0x5a << 24) | 1 */
    /* 0x3f10001c, (0x5a << 24) | 0x20 */
/*
   0:   52800021        mov     w1, #0x1                        // #1
   4:   72ab4001        movk    w1, #0x5a00, lsl #16
   8:   d2800480        mov     x0, #0x24                       // #36
   c:   f2a7e200        movk    x0, #0x3f10, lsl #16
  10:   b9000001        str     w1, [x0]
  14:   52800401        mov     w1, #0x20                       // #32
  18:   72ab4001        movk    w1, #0x5a00, lsl #16
  1c:   d2800380        mov     x0, #0x1c                       // #28
  20:   f2a7e200        movk    x0, #0x3f10, lsl #16
  24:   b9000001        str     w1, [x0]
*/
  const uint32_t instructions[] = {
    0x52800021,
    0x72ab4001,
    0xd2800480,
    0xf2a7e200,
    0xb9000001,
    0x52800401,
    0x72ab4001,
    0xd2800380,
    0xf2a7e200,
    0xb9000001
  };

  for (i = 0; i < ARRAY_SIZE(instructions); ++i)
    target_exec_instr(t, instructions[i]);
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
    target_arm_init(&t->dap, &t->core[i].edi, t->core[i].debug,
      t->core[i].cti);

  int f = 0;
  while(1) {
    target_halt(t);
    if (f)
      raspberrypi_soft_reset(t);
    else
      target_arm_mess(&t->dap, &t->core[0].edi, t->core[0].debug, t->core[0].cti);
    target_resume(t);
    f = 1;
  }

  return true;
}

