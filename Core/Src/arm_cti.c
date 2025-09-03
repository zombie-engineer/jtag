#include "arm_cti.h"
#include "arm_cti_regs.h"
#include "adiv5.h"

static void cti_mod_channel_bits(struct adiv5_dap *d, uint32_t regaddr,
  uint32_t mask, uint32_t value)
{
  uint32_t tmp = 0;
  adiv5_mem_ap_read_word(d, regaddr, &tmp);
  tmp &= ~mask;
  tmp |= value & mask;
  adiv5_mem_ap_write_word(d, regaddr, tmp);
}

void cti_gate_channel(struct adiv5_dap *d, uint32_t cti_base, int channel)
{
  cti_mod_channel_bits(d, cti_base + CTIGATE, 1 << channel, 0);
}

void cti_ungate_channel(struct adiv5_dap *d, uint32_t cti_base, int channel)
{
  cti_mod_channel_bits(d, cti_base + CTIGATE, 1 << channel, 0xffffffff);
}

void cti_pulse_event(struct adiv5_dap *d, uint32_t cti_base, int event)
{
  adiv5_mem_ap_write_word(d, cti_base + CTIAPPPULSE, 1 << event);
}

void cti_init(struct adiv5_dap *d, uint32_t cti_base)
{
  uint32_t reg = 0;
  /* Enable CTI multiplexing */
  adiv5_mem_ap_write_word(d, cti_base + CTICONTROL, 1);
  adiv5_mem_ap_read_word(d, cti_base + CTICONTROL, &reg);
  adiv5_mem_ap_write_word(d, cti_base + CTIGATE, 0);
  adiv5_mem_ap_write_word(d, cti_base + CTIOUTEN0, 1 << CTI_EVENT_HALT);
  adiv5_mem_ap_write_word(d, cti_base + CTIOUTEN1, 1 << CTI_EVENT_RESUME);
}

void cti_ack(struct adiv5_dap *d, uint32_t cti_base)
{
  uint32_t reg;

  adiv5_mem_ap_read_word(d, cti_base + CTITRIGINSTATUS, &reg);
  adiv5_mem_ap_read_word(d, cti_base + CTITRIGOUTSTATUS, &reg);
  adiv5_mem_ap_write_word(d, cti_base + CTIINTACK, 1);
  adiv5_mem_ap_read_word(d, cti_base + CTITRIGOUTSTATUS, &reg);

  adiv5_mem_ap_read_word(d, cti_base + CTITRIGOUTSTATUS, &reg);
  adiv5_mem_ap_write_word(d, cti_base + CTIAPPPULSE, 1<<1);
  adiv5_mem_ap_read_word(d, cti_base + CTITRIGOUTSTATUS, &reg);
}

void cti_ack2(struct adiv5_dap *d, uint32_t cti_base, uint32_t event)
{
  uint32_t reg;
  adiv5_mem_ap_write_word(d, cti_base + CTIINTACK, 1<<event);
  do {
    adiv5_mem_ap_read_word(d, cti_base + CTITRIGOUTSTATUS, &reg);
  } while (reg & (1 << event));
}
