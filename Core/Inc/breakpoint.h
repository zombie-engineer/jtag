#pragma once

struct breakpoint {
  uint64_t addr;
  uint32_t instr;
  bool skipped;
  bool enabled;
  bool busy;
} __attribute__((packed));
