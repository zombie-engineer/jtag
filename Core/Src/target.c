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

  success = aarch64_restore_before_resume(&c->a64, c->debug);
  if (!success)
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

bool target_is_halted(const struct target *t)
{
  return t->core[0].halted;
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
  // adiv5_mem_ap_read_word_e(baseaddr + 0x314, &reg0);
  //adiv5_mem_ap_read_word_e(baseaddr + 0x088, &reg1);
}

static bool target_parse_rom(struct target *t)
{
  int i;
  uint32_t mem_addr;
  uint32_t rom_entry;
  uint32_t rom_entries[ADIV5_MAX_ROM_ENTRIES] = { 0 };

  for (i = 0; i < ADIV5_MAX_ROM_ENTRIES; ++i) {
    adiv5_mem_ap_read_word_e(&t->dap, t->dap.base + i * 4, &rom_entries[i]);
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
  return true;
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

  return aarch64_read_mem32_once(&c->a64, c->debug, srcaddr, dst);
}

bool target_core_mem_read_fast_start(struct target_core *c, uint64_t addr)
{
  if (!c->halted)
    return false;

  return aarch64_read_mem32_fast_start(&c->a64, c->debug, addr);
}

bool target_core_mem_read_fast_next(struct target_core *c, uint32_t *value)
{
  if (!c->halted)
    return false;

  return aarch64_read_mem32_fast_next(&c->a64, c->debug, value);
}

bool target_core_mem_read_fast_stop(struct target_core *c)
{
  if (!c->halted)
    return false;

  return aarch64_read_mem32_fast_stop(&c->a64, c->debug);
}

bool target_core_write_mem32_once(struct target_core *c, uint64_t dstaddr,
    uint32_t value)
{
  if (!c->halted)
    return false;

  return aarch64_write_mem32_once(&c->a64, c->debug, dstaddr, value);
}

bool target_core_reg_write64(struct target_core *c, uint32_t reg_id,
  uint64_t value)
{
  if (!c->halted)
    return false;

  return aarch64_write_core_reg(&c->a64, c->debug, reg_id, value);
}

bool target_core_reg_read64(struct target_core *c, uint32_t reg_id,
  uint64_t *out)
{
  if (!c->halted)
    return false;

  return aarch64_read_core_reg(&c->a64, c->debug, reg_id, out);
}

bool raspberrypi_soft_reset(struct target *t)
{
    /* Raspberry PI reset sequence */
    /* 0x3f100024, (0x5a << 24) | 1 */
    /* 0x3f10001c, (0x5a << 24) | 0x20 */

  if (!target_core_write_mem32_once(&t->core[0], 0x3f100024, (0x5a << 24) | 1))
    return false;

  /* Write basicly should not complete */
  if (target_core_write_mem32_once(&t->core[0], 0x3f10001c, (0x5a << 24) | 0x20))
    return false;

  return true;
}

bool target_soft_reset(struct target *t)
{
  if (raspberrypi_soft_reset(t)) {
    t->attached = false;
    t->core[0].halted = false;
    return true;
  }

  return false;
}

bool target_init(struct target *t)
{
  int i;
  int num_devs;
  bool success;

  jtag_init();
  jtag_reset();

  num_devs = jtag_scan_num_devs();
  if (!num_devs)
    return false;

  t->idcode = jtag_read_idcode();
  if (!t->idcode || t->idcode == 0xffffffff)
    return false;

  if (!adiv5_dap_init(&t->dap))
    return false;

  if (!target_parse_rom(t))
    return false;

  t->core[0].cti = 0x80018000;
  t->core[1].cti = 0x80019000;
  t->core[2].cti = 0x8001a000;
  t->core[3].cti = 0x8001b000;

  success = true;
  for (i = 0; i < 4 && success; ++i)
    success = aarch64_init(&t->core[i].a64, &t->dap, t->core[i].debug,
      t->core[i].cti);

  if (success) {
    t->attached = true;
    t->core[0].halted = false;
  }
  return success;
}

bool target_mem_read_32(struct target *t, uint64_t addr, uint32_t *out)
{
  return target_core_read_mem32(&t->core[0], addr, out, 1);
}

bool target_mem_write_32(struct target *t, uint64_t addr, uint32_t value)
{
  return target_core_write_mem32_once(&t->core[0], addr, value);
}

bool target_reg_write_64(struct target *t, uint32_t reg_id, uint64_t value)
{
  return target_core_reg_write64(&t->core[0], reg_id, value);
}

bool target_reg_read_64(struct target *t, uint32_t reg_id, uint64_t *out)
{
  return target_core_reg_read64(&t->core[0], reg_id, out);
}

bool target_mem_read_fast_start(struct target *t, uint64_t addr)
{
  return target_core_mem_read_fast_start(&t->core[0], addr);
}

bool target_mem_read_fast_next(struct target *t, uint32_t *value)
{
  return target_core_mem_read_fast_next(&t->core[0], value);
}

bool target_mem_read_fast_stop(struct target *t)
{
  return target_core_mem_read_fast_stop(&t->core[0]);
}
