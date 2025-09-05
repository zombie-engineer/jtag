#include <cmd.h>
#include "Core/Inc/aarch64.h"
#include <string.h>
#include <stdio.h>

struct testcase {
  const char *line;
  struct cmd cmd;
};

#define ARRAY_SIZE(__a) (sizeof(__a) / sizeof(__a[0]))

#define CMD_ASSERT(__f)\
  if (c.__f != t->cmd.__f) { \
    printf(": mismatch for cmd." #__f ", expected: %08x, got: %08x. ", \
      t->cmd.__f, c.__f); \
    goto failed; \
  }\

bool run_testcase(struct testcase *t)
{
  struct cmd c = { 0 };
  printf("CASE \"%s\"", t->line);
  if (!cmdbuf_parse(&c, t->line, t->line + strlen(t->line))) {
    printf(": could not parse. ");
    goto failed;
  }

  CMD_ASSERT(cmd);
  CMD_ASSERT(arg0);
  CMD_ASSERT(arg1);
  CMD_ASSERT(arg2);
  CMD_ASSERT(arg3);
  CMD_ASSERT(count);
  CMD_ASSERT(access_size);
  CMD_ASSERT(is_write);

  printf(" \033[32mpassed\033[0m\n");
  return true;
failed:
  printf("\033[31mfailed\033[0m\n");
  return false;
}

#define T(__line, __cmd, __a0, __a1, __a2, __a3, __cnt, __access_sz, __is_wr) \
  { .line = __line, .cmd = { \
      .cmd = CMD_TARGET_ ## __cmd, \
      .arg0 = __a0, \
      .arg1 = __a1, \
      .arg2 = __a2, \
      .arg3 = __a3, \
      .count = __cnt, \
      .access_size = MEM_ACCESS_SIZE_ ## __access_sz, \
      .is_write = __is_wr \
    } \
  }

struct testcase cases[] = {
  T("r8 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 8, false),
  T("r16 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 16, false),
  T("r32 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 32, false),
  T("r64 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 64, false),
  T("r64  0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 64, false),
  T("r64   0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 64, false),
  T("r64/1 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 64, false),
  T("r64/2 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 2, 64, false),
  T("r64/3 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 3, 64, false),
  T("r64/12 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 12, 64, false),
  T("r64/512 0x3f000000", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, false),
  T("r64/512 0x3f000000  ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, false),
  T("  r64/512 0x3f000000  ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, false),
  T(" r64/512  0x3f000000  ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, false),
  T(" r64/512  0x3f000000 ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, false),
  T("w8 0x3f000000 0xff", MEM_ACCESS, 0x3f000000, 0, 0xff, 0, 1, 8, true),
  T("w16 0x3f000000 0xffff", MEM_ACCESS, 0x3f000000, 0, 0xffff, 0, 1, 16, true),
  T("w32 0x3f000000 0x7fffffff", MEM_ACCESS, 0x3f000000, 0, 0x7fffffff, 0, 1, 32, true),
  T("w64 0x3f000000 0x1111111122222222", MEM_ACCESS, 0x3f000000, 0, 0x22222222, 0x11111111, 1, 64, true),
  T("w64  0x3f000000 0", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 64, true),
  T("w64   0x3f000000 0", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 64, true),
  T("w64/1 0x3f000000 0", MEM_ACCESS, 0x3f000000, 0, 0, 0, 1, 64, true),
  T("w64/2 0x3f000000 0", MEM_ACCESS, 0x3f000000, 0, 0, 0, 2, 64, true),
  T("w64/3 0x3f000000 0", MEM_ACCESS, 0x3f000000, 0, 0, 0, 3, 64, true),
  T("w64/12 0x3f000000 0", MEM_ACCESS, 0x3f000000, 0, 0, 0, 12, 64, true),
  T("w64/512 0x3f000000 0", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, true),
  T("w64/512 0x3f000000 0 ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, true),
  T("  w64/512 0x3f000000 0 ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, true),
  T(" w64/512  0x3f000000 0  ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, true),
  T(" w64/512  0x3f000000 0 ", MEM_ACCESS, 0x3f000000, 0, 0, 0, 512, 64, true),
  T("rr x0", REG_ACCESS, AARCH64_CORE_REG_X0, 0, 0, 0, 0, 8, false),
  T("rr x1", REG_ACCESS, AARCH64_CORE_REG_X1, 0, 0, 0, 0, 8, false),
  T("rr x30", REG_ACCESS, AARCH64_CORE_REG_X30, 0, 0, 0, 0, 8, false),
  T("rr pc", REG_ACCESS, AARCH64_CORE_REG_PC, 0, 0, 0, 0, 8, false),
  T("rr sp", REG_ACCESS, AARCH64_CORE_REG_SP, 0, 0, 0, 0, 8, false),
  T("rw x0 0", REG_ACCESS, AARCH64_CORE_REG_X0, 0, 0, 0, 0, 8, true),
  T("rw x1 0", REG_ACCESS, AARCH64_CORE_REG_X1, 0, 0, 0, 0, 8, true),
  T("rw x30 0", REG_ACCESS, AARCH64_CORE_REG_X30, 0, 0, 0, 0, 8, true),
  T("rw pc 0", REG_ACCESS, AARCH64_CORE_REG_PC, 0, 0, 0, 0, 8, true),
  T("rw sp 0", REG_ACCESS, AARCH64_CORE_REG_SP, 0, 0, 0, 0, 8, true),

  T(" rr  x0 ", REG_ACCESS, AARCH64_CORE_REG_X0, 0, 0, 0, 0, 8, false),
  T(" rr x30  ", REG_ACCESS, AARCH64_CORE_REG_X30, 0, 0, 0, 0, 8, false),
  T(" rw x0 0   ", REG_ACCESS, AARCH64_CORE_REG_X0, 0, 0, 0, 0, 8, true),
  T(" rw x30      0  ", REG_ACCESS, AARCH64_CORE_REG_X30, 0, 0, 0, 0, 8, true),
  T(" rw pc    0     ", REG_ACCESS, AARCH64_CORE_REG_PC, 0, 0, 0, 0, 8, true),
  T("bps 0xffff000000112233", BREAKPOINT, 0x00112233, 0xffff0000, 0, 0, 0, 8, false),
  T("bpsd 1", BREAKPOINT, 1, 0, 0, 1, 0, 8, false),
  T("bph 0xffff000000112233", BREAKPOINT, 0x00112233, 0xffff0000, 1, 0, 0, 8, false),
  T("bphd 1", BREAKPOINT, 1, 0, 1, 1, 0, 8, false),
};

int main()
{
  for (int i = 0; i < ARRAY_SIZE(cases); ++i)
    run_testcase(&cases[i]);
  return 0;
}
