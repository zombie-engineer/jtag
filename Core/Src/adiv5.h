#pragma once
#include <stdint.h>
#include <stdbool.h>

#define DP 0
#define AP 1

#define OP_READ 0
#define OP_WRITE 1

#define DP_REG_ADDR_CTRL_STAT 0x04
#define DP_REG_ADDR_DLCR      0x14
#define DP_REG_ADDR_TARGETID  0x24
#define DP_REG_ADDR_DLPIDR    0x34

struct adiv5_dap {
  uint32_t idcode;
  uint32_t target_id;
  uint32_t dpidr;
  uint32_t dlpidr;
  uint32_t base;
  uint32_t idr;
  uint32_t ctl_stat;
  uint32_t select;
  uint32_t ir;
  uint32_t csw;
};

bool adiv5_transfer(struct adiv5_dap *d, int type, int op, int reg_addr,
  uint32_t value_in, uint32_t *value_out);

static inline bool adiv5_read_ctr_stat(struct adiv5_dap *d, uint32_t *out)
{
  return adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, out);
}

bool adiv5_write_ctr_stat(struct adiv5_dap *d, uint32_t value);

bool adiv5_reg_write(int addr, uint32_t value);

bool adiv5_dap_init(struct adiv5_dap *d);
void adiv5_mem_ap_read_word_drw(struct adiv5_dap *d, uint32_t addr,
  uint32_t *out);

void adiv5_mem_ap_read_word(struct adiv5_dap *d, uint32_t addr, uint32_t *out);
void adiv5_mem_ap_write_word(struct adiv5_dap *d, uint32_t addr,
  uint32_t value);

void adiv5_parse_rom(uint32_t rom_addr, void (*rom_entry_cb)(uint32_t rom));
