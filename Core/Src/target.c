#include "target.h"
#include "adiv5.h"
#include "jtag.h"
#include "cmsis_edi.h"
#include "arm_cmsis.h"
#include "cmsis_arch_id.h"
#include "common.h"

#define ADIV5_MAX_ROM_ENTRIES 32

bool target_core_halt(struct target_core *c)
{
  if (!aarch64_halt(&c->a64, c->debug, c->cti))
    return false;

  c->halted = true;
  return aarch64_fetch_context(&c->a64, c->debug);
}

bool target_core_resume(struct target_core *c)
{
  bool success;

  if (!c->halted)
    return false;

  success = aarch64_resume(&c->a64, c->debug, c->cti);
  if (success)
    c->halted = false;

  return success;
}

bool target_halt(struct target *t)
{
  return target_core_halt(&t->core[0]);
}

bool target_resume(struct target *t)
{
  return target_core_resume(&t->core[0]);
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

bool target_core_exec(struct target_core *c, const uint32_t * const instr, int num)
{
  if (!c->halted)
    return false;

  return aarch64_exec(&c->a64, c->debug, instr, num);
}

bool target_core_write_mem32(struct target_core *c, uint64_t dstaddr,
    const uint32_t *src, size_t num_words)
{
  if (!c->halted)
    return false;

  return aarch64_write_mem32(&c->a64, c->debug, dstaddr, src, num_words);
}

bool target_core_read_mem32(struct target_core *c, uint64_t srcaddr,
  uint32_t *dst, size_t num_words)
{
  if (!c->halted)
    return false;

  return aarch64_read_mem32(&c->a64, c->debug, srcaddr, dst, num_words);
}

bool target_core_write_mem32_once(struct target_core *c, uint64_t dstaddr,
    uint32_t value)
{
  if (!c->halted)
    return false;

  return aarch64_write_mem32_once(&c->a64, c->debug, dstaddr, value);
}

bool raspberrypi_soft_reset(struct target *t)
{
    /* Raspberry PI reset sequence */
    /* 0x3f100024, (0x5a << 24) | 1 */
    /* 0x3f10001c, (0x5a << 24) | 0x20 */

  if (!target_core_write_mem32_once(&t->core[0], 0x3f100024, (0x5a << 24) | 1))
    return false;

  if (!target_core_write_mem32_once(&t->core[0], 0x3f10001c, (0x5a << 24) | 0x20))
    return false;

  return true;
}

bool target_init(struct target *t)
{
  int i;
  int num_devs;
  uint32_t words[2] = {
    0xaabbccdd,
    0x11223344
  };
  uint32_t test_words[2] = { 0, 0 };

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
    aarch64_init(&t->core[i].a64, &t->dap, t->core[i].debug, t->core[i].cti);

#if 0
  int f = 1;
  while(1) {
    target_halt(t);
    raspberrypi_soft_reset(t);
    break;
    if (!target_core_write_mem32_once(&t->core[0], 0x80000, 0xd1e2b374))
      while(1);

    if (!target_core_write_mem32(&t->core[0], 0x80000, words, 2))
      while(1);

    if (!target_core_read_mem32(&t->core[0], 0x80000, test_words, 2))
      while(1);
    if (f)
      raspberrypi_soft_reset(t);
    else
      aarch64_mess(&t->core[0].a64, t->core[0].debug, t->core[0].cti);
    target_resume(t);
    f = 1;
  }

#endif
  return true;
}

