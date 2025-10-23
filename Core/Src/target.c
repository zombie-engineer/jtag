#include "target.h"
#include "adiv5.h"
#include "jtag.h"
#include "cmsis_edi.h"
#include "arm_cmsis.h"
#include "cmsis_arch_id.h"
#include "common.h"
#include <io_api.h>
#include <errno.h>
#include <string.h>

#define ADIV5_MAX_ROM_ENTRIES 32

static int target_core_set_sw_breakpoints(struct target_core *c,
  struct breakpoint *sw_breakpoints, int num_breakpoints, bool remove)
{
  int ret;
  int i;
  struct breakpoint *b;
  for (i = 0; i < num_breakpoints; ++i) {
    b = &sw_breakpoints[i];
    if (!b->busy)
      continue;

    ret = aarch64_breakpoint(&c->a64, c->debug, remove, false, b);
    if (ret)
      return ret;
  }
  return 0;
}

int target_core_halt(struct target_core *c, struct breakpoint *sw_breakpoints,
  int num_breakpoints)
{
  int ret = aarch64_halt(&c->a64, c->debug, c->cti);
  if (ret)
    return ret;

  c->halted = true;

  return target_core_set_sw_breakpoints(c, sw_breakpoints, num_breakpoints,
    true);
}

static int target_core_iter_regs(struct target_core *c,
  reg_iter_cb_t cb, void *arg)
{
  return aarch64_iter_state_regs(&c->a64, c->debug, cb, arg);
}

static int target_core_exec(struct target_core *c, const uint32_t *const instr,
  int count)
{
  if (!c->halted)
    return -EPIPE;

  return aarch64_exec(&c->a64, c->debug, instr, count);
}

static void target_core_get_halt_reason(struct target_core *c,
  const char **str)
{
  aarch64_get_halt_reason(&c->a64, str);
}

static int target_core_check_halted(struct target_core *c,
  struct breakpoint *sw_breakpoints, int num_breakpoints)
{
  int ret = aarch64_check_halted(&c->a64, c->debug, c->cti);
  if (ret)
    return ret;

  c->halted = true;

  return target_core_set_sw_breakpoints(c, sw_breakpoints, num_breakpoints,
    true);
}

static int target_core_step(struct target_core *c,
  struct breakpoint *sw_breakpoints, int num_breakpoints);

int target_core_resume(struct target_core *c,
  struct breakpoint *sw_breakpoints, int num_breakpoints)
{
  int ret;
  struct breakpoint *b;
  int i;
  uint64_t pc;

  if (!c->halted)
    return -EINVAL;

  for (i = 0; i < num_breakpoints; ++i) {
    b = &sw_breakpoints[i];
    if (!b->busy)
      continue;

    ret = aarch64_read_cached_reg(&c->a64, c->debug, AARCH64_CORE_REG_PC, &pc);
    if (ret)
      return ret;

    if (b->addr == pc) {
      ret = target_core_step(c, sw_breakpoints, num_breakpoints);
      if (ret)
        return ret;
    }
  }

  ret = target_core_set_sw_breakpoints(c, sw_breakpoints, num_breakpoints,
    false);
  if (ret)
    return ret;

  ret = aarch64_restore_before_resume(&c->a64, c->debug);
  if (ret)
    return ret;

  ret = aarch64_resume(&c->a64, c->debug, c->cti);
  if (!ret)
    c->halted = false;

  return ret;
}

static int target_core_step(struct target_core *c,
  struct breakpoint *sw_breakpoints, int num_breakpoints)
{
  int ret;

  if (!c->halted)
    return -EINVAL;

  ret = aarch64_restore_before_resume(&c->a64, c->debug);
  if (ret)
    return ret;

  ret = aarch64_step(&c->a64, c->debug, c->cti);
  if (ret)
    return ret;

  return 0;
}

int target_core_breakpoint(struct target_core *c, bool remove, bool hardware,
  struct breakpoint *b)
{
  if (!c->halted)
    return -EINVAL;

  return aarch64_breakpoint(&c->a64, c->debug, remove, hardware, b);
}

int target_halt(struct target *t)
{
  return target_core_halt(&t->core[0], t->breakpoints_sw,
    ARRAY_SIZE(t->breakpoints_sw));
}

int target_iter_regs(struct target *t, int core_idx, reg_iter_cb_t cb,
  void *arg)
{
  return target_core_iter_regs(&t->core[core_idx], cb, arg);
}

int target_exec(struct target *t, const uint32_t *instr, int count)
{
  return target_core_exec(&t->core[0], instr, count);
}

int target_check_halted(struct target *t)
{
  return target_core_check_halted(&t->core[0], t->breakpoints_sw,
    ARRAY_SIZE(t->breakpoints_sw));
}

bool target_is_halted(const struct target *t)
{
  return t->core[0].halted;
}

int target_resume(struct target *t)
{
  return target_core_resume(&t->core[0], t->breakpoints_sw,
    ARRAY_SIZE(t->breakpoints_sw));
}

int target_step(struct target *t)
{
  return target_core_step(&t->core[0], t->breakpoints_sw,
    ARRAY_SIZE(t->breakpoints_sw));
}

static struct breakpoint *target_get_breakpoint(struct target *t,
  bool hardware, uint64_t addr, bool busy)
{
  int i;
  struct breakpoint *bps, *b;
  int num_bps;

  if (hardware) {
    bps = t->breakpoints_hw;
    num_bps = ARRAY_SIZE(t->breakpoints_hw);
  }
  else {
    bps = t->breakpoints_sw;
    num_bps = ARRAY_SIZE(t->breakpoints_sw);
  }

  for (i = 0; i < num_bps; ++i) {
    b = &bps[i];
    if (!busy) {
      if (!b->busy) {
        b->busy = true;
        b->enabled = true;
        b->addr = addr;
        return b;
      }
    } else {
      /* busy, we are looking for existing breakpoint with matching addr  */
      if (b->addr == addr)
        return b;
    }
  }
  return NULL;
}

int target_breakpoint(struct target *t, bool remove, bool hardware,
  uint64_t addr)
{
  struct breakpoint *b;

  b = target_get_breakpoint(t, hardware, addr, remove);
  if (!b)
    return remove ? -EINVAL : -ENOMEM;

  if (!remove) {
    /* HW breakpoint can be set now (immediately) */
    if (hardware)
      return target_core_breakpoint(&t->core[0], remove, hardware, b);

    /* Software breakpoints are set during resume */
    return 0;
  }

  /* remove */
  if (hardware)
    return target_core_breakpoint(&t->core[0], remove, hardware, b);

  /* sw breakpoint is removed now */
  b->busy = false;
  return 0;
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

static int target_core_mem_read(struct target_core *c,
  mem_access_size_t access_size, uint64_t addr, size_t count,
  void (*cb)(uint64_t, mem_access_size_t))
{
  int ret;
  uint64_t value;

  if (!c->halted)
    return -EPIPE;

  if (count == 0)
    return -EINVAL;

  if (count == 1) {
    ret = aarch64_read_mem_once(&c->a64, c->debug, access_size, addr, &value);
    if (!ret)
      cb(value, access_size);
    return ret;
  }

  if (access_size != MEM_ACCESS_SIZE_32 && access_size != MEM_ACCESS_SIZE_64)
    return -ENOTSUP;

  return aarch64_mem_read_fast(&c->a64, c->debug, access_size, addr, count,
    cb);
}

static int target_core_mem_write(struct target_core *c,
  mem_access_size_t access_size, uint64_t dstaddr, uint32_t value)
{
  if (!c->halted)
    return -EPIPE;

  return aarch64_write_mem32_once(&c->a64, c->debug, dstaddr, value);
}

int target_core_reg_write64(struct target_core *c, uint32_t reg_id,
  uint64_t value, bool sync)
{
  int ret;

  if (!c->halted)
    return -EPIPE;

  ret = aarch64_write_cached_reg(&c->a64, c->debug, reg_id, value);
  if (ret)
    return ret;

  if (sync)
    ret = aarch64_restore_reg(&c->a64, c->debug, reg_id);
  return ret;
}

int target_core_reg_read64(struct target_core *c, uint32_t reg_id,
  uint64_t *out, bool direct)
{
  if (!c->halted)
    return -EPIPE;

  if (direct)
    return aarch64_reg_read_64(&c->a64, c->debug, reg_id, out);

  return aarch64_read_cached_reg(&c->a64, c->debug, reg_id, out);
}

int raspberrypi_soft_reset(struct target *t)
{
  int ret;
    /* Raspberry PI reset sequence */
    /* 0x3f100024, (0x5a << 24) | 1 */
    /* 0x3f10001c, (0x5a << 24) | 0x20 */

  ret = target_core_mem_write(&t->core[0], MEM_ACCESS_SIZE_32, 0x3f100024,
    (0x5a << 24) | 1);
  if (ret)
    return ret;

  /* Write basicly should not complete */
  return target_core_mem_write(&t->core[0], MEM_ACCESS_SIZE_32, 0x3f10001c,
    (0x5a << 24) | 0x20);
}

int target_soft_reset(struct target *t)
{
  int ret = raspberrypi_soft_reset(t);
  if (!ret) {
    t->attached = false;
    t->core[0].halted = false;
  }

  return ret;
}

static void target_init_state(struct target *t)
{
  memset(t->breakpoints_sw, 0, sizeof(t->breakpoints_sw));
  memset(t->breakpoints_hw, 0, sizeof(t->breakpoints_hw));
}

int target_init(struct target *t)
{
  int i;
  int num_devs;
  int ret = 0;

  target_init_state(t);

  jtag_init();
  jtag_reset();

  num_devs = jtag_scan_num_devs();
  if (!num_devs)
    return -EIO;

  t->idcode = jtag_read_idcode();
  if (!t->idcode || t->idcode == 0xffffffff)
    return -EIO;

  if (!adiv5_dap_init(&t->dap))
    return -EIO;

  if (!target_parse_rom(t))
    return -EIO;

  t->core[0].cti = 0x80018000;
  t->core[1].cti = 0x80019000;
  t->core[2].cti = 0x8001a000;
  t->core[3].cti = 0x8001b000;

  for (i = 0; i < 4 && !ret; ++i)
    ret = aarch64_init(&t->core[i].a64, &t->dap, t->core[i].debug,
      t->core[i].cti);

  if (!ret) {
    t->attached = true;
    t->core[0].halted = false;
  }
  return ret;
}

int target_mem_read(struct target *t, mem_access_size_t access_size,
  uint64_t addr, size_t count, void (*cb)(uint64_t, mem_access_size_t))
{
  return target_core_mem_read(&t->core[0], access_size, addr, count, cb);
}

int target_mem_write(struct target *t, mem_access_size_t access_size,
  uint64_t addr, uint64_t value)
{
  return target_core_mem_write(&t->core[0], access_size, addr, value);
}

int target_reg_write_64(struct target *t, uint32_t reg_id, uint64_t value,
  bool sync)
{
  return target_core_reg_write64(&t->core[0], reg_id, value, sync);
}

int target_reg_read_64(struct target *t, uint32_t reg_id, uint64_t *out,
  bool direct)
{
  return target_core_reg_read64(&t->core[0], reg_id, out, direct);
}

void target_get_halt_reason(struct target *t, const char **str)
{
  target_core_get_halt_reason(&t->core[0], str);
}
