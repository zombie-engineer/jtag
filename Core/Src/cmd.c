#include "cmd.h"
#include <aarch64.h>
#include <stdlib.h>
#include <string.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define is_digit10(__c) (__c >= '0' && __c <= '9')

#define SKIP_SPACES(__ptr, __end) \
  for (; __ptr != __end && *__ptr && *__ptr == ' '; ++__ptr);

#define CHRCMP2(__str, __c0, __c1) \
  ((__str)[0] == (__c0) && (__str)[1] == (__c1))

#define CHRCMP3(__str, __c0, __c1, __c2) \
  ((__str)[0] == (__c0) && (__str)[1] == (__c1) && (__str)[2] == (__c2))

static inline int parse_reg_name(bool is_write, const char **ptr,
  const char *end)
{
  const char *p = *ptr;
  int reg_idx;
  int reg_id;

  SKIP_SPACES(p, end);

  if (p == end || !*p)
    return AARCH64_CORE_REG_UNKNOWN;

  /* parsing register name, staring from numeric is invalid */
  if (is_digit10(*p))
    return AARCH64_CORE_REG_UNKNOWN;

  if (p == end)
    return AARCH64_CORE_REG_UNKNOWN;

  if ((!is_write && end - p == 2) || (is_write && end - p > 3 && p[2] == ' ')) {
    if (CHRCMP2(p, 'p', 'c')) {
      reg_id = AARCH64_CORE_REG_PC;
      p += 2;
      goto out;
    }
    if (CHRCMP2(p, 's', 'p')) {
      reg_id = AARCH64_CORE_REG_SP;
      p += 2;
      goto out;
    }
  }

  if (*p != 'x')
    return AARCH64_CORE_REG_UNKNOWN;

  p++;
  if (!is_digit10(*p))
    return AARCH64_CORE_REG_UNKNOWN;

  reg_idx = *p - '0';
  p++;

  if (p != end && is_digit10(*p)) {
    reg_idx = reg_idx * 10 + *p - '0';
    p++;
  }

  /* I don't expect 3 digit register names */
  if (p != end && is_digit10(*p))
    return AARCH64_CORE_REG_UNKNOWN;

  reg_id = AARCH64_CORE_REG_X0 + reg_idx;

out:
  *ptr = p;
  return reg_id;
}

static inline bool cmdline_parse_read_write_cmd(const char *l,
  const char *end, struct cmd *c)
{
  /*
   * Memory read / write:
   * access size: r8, r16, r32, r64
   * number of elements: r32/3
   * + address argument: r32 0x00000000, r64/8 0x00000000
   * + write arg: w32 0x00000000 0xffffffff, w64 0x00000000 0xffff111100002222
   * Register read / write:
   * rr register_name
   * rw register_name 0xffff11
   */
  int reg_id;
  const char *old_l;
  bool is_write;
  uint64_t addr;
  uint64_t value;
  int count;
  mem_access_size_t access_size;

  SKIP_SPACES(l, end);

  /* smallest line is 5 chars rr X */
  if (end - l < 4)
    return false;

  if (CHRCMP3(l, 'r', 'r', ' ') || CHRCMP3(l, 'r', 'w', ' ')) {
    /* Register read / write */

    is_write = l[1] == 'w';
    l += 3;
    reg_id = parse_reg_name(is_write, &l, end);
    if (reg_id == AARCH64_CORE_REG_UNKNOWN)
      return false;

    SKIP_SPACES(l, end);

    if (is_write) {
      old_l = l;
      value = strtoull(l, (char **)&l, 0);
      if (l == old_l)
        return false;

      SKIP_SPACES(l, end);
    }

    if (l != end)
      return false;

    c->cmd = CMD_TARGET_REG_ACCESS;
    c->arg0 = reg_id;

    if (is_write) {
      c->arg1 = value & 0xffffffff;
      c->arg2 = (value >> 32) & 0xffffffff;
    }

    c->is_write = is_write;
    return true;
  }

  /* Should be mem read or write */
  if (l[0] != 'r' && l[0] != 'w')
    return false;

  is_write = l[0] == 'w';
  l++;

  if (end - l < 2)
    return false;

  if (l[0] == '8') {
    access_size = MEM_ACCESS_SIZE_8;
    l++;
    goto parse_count;
  }

  if (end - l < 3)
    return false;

  if (CHRCMP2(l, '1', '6'))
    access_size = MEM_ACCESS_SIZE_16;
  else if (CHRCMP2(l, '3', '2'))
    access_size = MEM_ACCESS_SIZE_32;
  else if (CHRCMP2(l, '6', '4'))
    access_size = MEM_ACCESS_SIZE_64;
  else
    return false;
  l += 2;

parse_count:
  count = 1;
  if (l[0] == '/') {
    l++;
    old_l = l;
    count = strtol(l, (char **)&l, 0);
    if (l == old_l)
      return false;
  }

  if (l[0] != ' ')
    return false;

  SKIP_SPACES(l, end);
  old_l = l;
  addr = strtoull(l, (char **)&l, 0);
  if (l == old_l)
    return false;

  if (is_write) {
    if (l[0] != ' ')
      return false;
    SKIP_SPACES(l, end);
    old_l = l;
    value = strtoull(l, (char **)&l, 0);
    if (l == old_l)
      return false;
  }

  c->arg0 = addr & 0xffffffff;
  c->arg1 = (addr >> 32) & 0xffffffff;
  c->cmd = CMD_TARGET_MEM_ACCESS;
  c->is_write = is_write;
  c->access_size = access_size;
  c->count = count;
  if (is_write) {
    c->arg2 = value & 0xffffffff;
    c->arg3 = (value >> 32) & 0xffffffff;
  }
  return true;
}

bool cmdbuf_parse(struct cmd *c, const char *buf, const char *end)
{
  const char *p = buf;

  c->arg0 = 0;
  c->arg1 = 0;
  c->arg2 = 0;
  c->arg3 = 0;

  if (buf == end) {
    c->cmd = CMD_NONE;
    return true;
  }

  if (!strncmp(p, "status", 6)) {
    c->cmd = CMD_TARGET_STATUS;
    return true;
  }
  if (!strncmp(p, "dumpregs", 8)) {
    c->cmd = CMD_TARGET_DUMP_REGS;
    return true;
  }
  if (!strncmp(p, "init", 4)) {
    c->cmd = CMD_TARGET_INIT;
    return true;
  }
  else if (!strncmp(p, "halt", 4)) {
    c->cmd = CMD_TARGET_HALT;
    return true;
  }
  else if (!strncmp(p, "resume", 6)) {
    c->cmd = CMD_TARGET_RESUME;
    return true;
  }
  else if (p[0] == 's' && p[1] == 0) {
    c->cmd = CMD_TARGET_STEP;
    return true;
  }
  else if (!strncmp(p, "srst", 4)) {
    c->cmd = CMD_TARGET_SOFT_RESET;
    return true;
  }
  else if (cmdline_parse_read_write_cmd(buf, end, c))
    return true;

  return false;
}
