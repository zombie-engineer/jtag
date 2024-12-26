#pragma once
#include "adiv5.h"
#include "arm_cmsis_regs.h"

struct arm_cmsis_regs {
  uint32_t cdbgpwrupreq;
  uint32_t cdbgpwrupack;
  uint32_t itctrl;
  uint32_t claimset;
  uint32_t claimclr;
  uint32_t devaff0;
  uint32_t devaff1;
  uint32_t lar;
  uint32_t lsr;
  uint32_t authstatus;
  uint32_t devarch;
  uint32_t devid;
  uint32_t devtype;
  uint32_t pidr4;
  uint32_t pidr5;
  uint32_t pidr6;
  uint32_t pidr7;
  uint32_t pidr0;
  uint32_t pidr1;
  uint32_t pidr2;
  uint32_t pidr3;
  uint32_t cidr0;
  uint32_t cidr1;
  uint32_t cidr2;
  uint32_t cidr3;
};

static inline void cmsis_read_regs(struct adiv5_dap *d, uint32_t baseaddr,
  struct arm_cmsis_regs *r)
{
  adiv5_mem_ap_read_word(d, baseaddr + CMSIS_REG_DEVARCH, &r->devarch);
  adiv5_mem_ap_read_word(d, baseaddr + CMSIS_REG_DEVID, &r->devid);
  adiv5_mem_ap_read_word(d, baseaddr + CMSIS_REG_DEVTYPE, &r->devtype);
  adiv5_mem_ap_read_word(d, baseaddr + CMSIS_REG_DEVAFF0, &r->devaff0);
  adiv5_mem_ap_read_word(d, baseaddr + CMSIS_REG_DEVAFF1, &r->devaff1);
  adiv5_mem_ap_read_word(d, baseaddr + CMSIS_REG_AUTHSTATUS, &r->authstatus);
}

