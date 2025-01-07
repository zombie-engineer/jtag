#include "adiv5.h"
#include "jtag.h"
#include <stdbool.h>

#define DPACC 0b1010
#define APACC 0b1011

#define ACK_OK 0b010

#define ADI_READ_BIT (1<<0)
#define ADI_WRITE_BIT (0<<0)
#define ADI_ADDR_BITS(__a) (((__a >> 2) & 3) << 1)
#define ADI_MSG_LEN 35
#define ADI_RESP_STATUS_MASK 7
#define ADI_RESP2DATA(__r) (uint32_t)(__r >> 3)

#define DP_REG_ADDR_DPIDR     0
#define DP_REG_ADDR_SELECT    8
#define DP_REG_ADDR_RDBUFF    0xc

#define AP_REG_ADDR_CSW  0x00
#define AP_REG_ADDR_TAR  0x04
#define AP_REG_ADDR_DRW  0x0c
#define AP_REG_ADDR_BD0  0x10
#define AP_REG_ADDR_CFG  0xf4
#define AP_REG_ADDR_BASE 0xf8
#define AP_REG_ADDR_IDR  0xfc

bool last_success = true;

void adiv5_write_ir(struct adiv5_dap *d, uint8_t ir)
{
  uint64_t out_value = 0;

  if (d->ir == ir)
    return;

  jtag_to_state(JTAG_STATE_SHIFT_IR);
  jtag_shift(ir, &out_value, 4);
  jtag_to_state(JTAG_STATE_RUN_TEST_IDLE);
  d->ir = ir;
}

#define CTRL_STAT_CDBGRSTREQ   (1<<26)
#define CTRL_STAT_CDBGRSTACK   (1<<27)
#define CTRL_STAT_CDBGPWRUPREQ (1<<28)
#define CTRL_STAT_CDBGPWRUPACK (1<<29)
#define CTRL_STAT_CSYSPWRUPREQ (1<<30)
#define CTRL_STAT_CSYSPWRUPACK (1<<31)

#define CDBGRSTACK_NUM_REPS 5000

static bool adiv5_dap_dbg_power_up(struct adiv5_dap *d)
{
  if (!adiv5_write_ctr_stat(d, d->ctl_stat |
    CTRL_STAT_CDBGPWRUPREQ | CTRL_STAT_CSYSPWRUPREQ))
    return false;

  while (1) {
    if (!adiv5_read_ctr_stat(d, &d->ctl_stat))
      return false;

    if (d->ctl_stat & CTRL_STAT_CDBGPWRUPACK)
      break;
  }

#if 0
  if (!adiv5_write_ctr_stat(dap.ctl_stat | CTRL_STAT_CSYSPWRUPREQ))
    return false;

  while(1) {
    if (!adiv5_read_ctr_stat(&dap.ctl_stat))
      return false;

    if (dap.ctl_stat & CTRL_STAT_CSYSPWRUPACK)
      break;
  }
#endif
  return true;
}

static bool adiv5_dap_dbg_power_down(struct adiv5_dap *d)
{
  const uint32_t ack_mask = CTRL_STAT_CSYSPWRUPACK;// | CTRL_STAT_CDBGPWRUPACK;

  if (!adiv5_write_ctr_stat(d, 0))
    return false;

  while (1) {
    if (!adiv5_read_ctr_stat(d, &d->ctl_stat))
      return false;

    if (!(d->ctl_stat & ack_mask))
      break;
  }

  return true;
}


bool adiv5_dap_reset(struct adiv5_dap *d)
{
  size_t i;
  bool timeout = true;

  if (!adiv5_write_ctr_stat(d, d->ctl_stat | CTRL_STAT_CDBGRSTREQ))
    return false;

  for (i = 0; i < CDBGRSTACK_NUM_REPS; ++i) {
    if (!adiv5_read_ctr_stat(d, &d->ctl_stat))
      return false;

    if (d->ctl_stat & CTRL_STAT_CDBGRSTACK) {
      timeout = false;
      break;
    }
  }

  if (timeout)
    return false;

  if (!adiv5_write_ctr_stat(d, d->ctl_stat & ~CTRL_STAT_CDBGRSTREQ))
    return false;

  for (i = 0; i < CDBGRSTACK_NUM_REPS; ++i) {
    if (!adiv5_read_ctr_stat(d, &d->ctl_stat))
      return false;

    if (!(d->ctl_stat & CTRL_STAT_CDBGRSTACK))
      return true;
  }

  return false;
}

static inline bool adiv5_read_rdbuf_resp(struct adiv5_dap *d, uint32_t *out)
{
  uint64_t dap_response;
  uint64_t dr_value;
  int ack;

  adiv5_write_ir(d, DPACC);
  dr_value = ADI_ADDR_BITS(DP_REG_ADDR_RDBUFF) | ADI_READ_BIT;

  while(1) {
    jtag_to_state(JTAG_STATE_SHIFT_DR);
    jtag_shift(dr_value, &dap_response, ADI_MSG_LEN);
    ack = dap_response & ADI_RESP_STATUS_MASK;
    if (ack == ACK_OK) {
      *out = ADI_RESP2DATA(dap_response);
      return true;
    }
  }
  return false;
}

static bool adiv5_do_shift(struct adiv5_dap *d, int op, int addr,
  uint32_t value_in, uint32_t *value_out)
{
  uint64_t dap_response;

  uint64_t dr_value = (op == OP_READ ? ADI_READ_BIT : ADI_WRITE_BIT)
    | ADI_ADDR_BITS(addr)
    | (((uint64_t)(op == OP_WRITE ? value_in : 0)) << 3);

  jtag_to_state(JTAG_STATE_SHIFT_DR);
  jtag_shift(dr_value, &dap_response, 35);
  return adiv5_read_rdbuf_resp(d, value_out);
}

static bool adiv5_write_select(struct adiv5_dap *d, int dpbank, int apbank,
  int apsel)
{
  uint64_t select = (dpbank & 0xf)
    | ((apbank & 0xf) << 4)
    | ((apsel & 0xf) << 24);

  if (select == d->select)
    return true;

  adiv5_write_ir(d, DPACC);

  if (adiv5_reg_write(DP_REG_ADDR_SELECT, select)) {
    d->select = select;
    return true;
  }

  return false;
}

bool adiv5_transfer(struct adiv5_dap *d, int type, int op, int reg_addr,
  uint32_t value_in, uint32_t *value_out)
{
  uint32_t v = 0;
  int ap_bank = 0;
  int dp_bank = 0;

  int addr = reg_addr & 0xf;
  if (type == DP)
    dp_bank = (reg_addr >> 4) & 0xf;
  else
    ap_bank = (reg_addr >> 4) & 0xf;

  if (!adiv5_write_select(d, dp_bank, ap_bank, 0))
    return false;

  if (type == DP)
    adiv5_write_ir(d, DPACC);
  else
    adiv5_write_ir(d, APACC);

  if (!adiv5_do_shift(d, op, addr, value_in, &v))
    return false;

  *value_out = v;
  return true;
}

bool adiv5_dap_init_power(struct adiv5_dap *d)
{
  adiv5_write_ir(d, DPACC);

  if (!adiv5_read_ctr_stat(d, &d->ctl_stat))
    return false;

  if (!adiv5_dap_dbg_power_down(d))
    return false;

  if (!adiv5_dap_reset(d))
    return false;

  if (!adiv5_dap_dbg_power_up(d))
    return false;
  
  return true;
}

bool adiv5_write_ctr_stat(struct adiv5_dap *d, uint32_t value)
{
  /* WRITE to SELECT */
  if (!adiv5_write_select(d, 0, 0, 0))
    return false;

  adiv5_write_ir(d, DPACC);
  return adiv5_reg_write(DP_REG_ADDR_CTRL_STAT, value);
}

bool adiv5_reg_write(int addr, uint32_t value)
{
  uint64_t dap_response;
  int ack;

  uint64_t dr_value = 0;

  /* Actual register value at [34:3] */
  dr_value |= ((uint64_t)value) << 3;

  /* A[3:2] of address value at [2:1] */
  dr_value |= ((addr >> 2) & 3) << 1;

  /* RnW is 0 for WRITE OP */

  jtag_to_state(JTAG_STATE_SHIFT_DR);
  jtag_shift(dr_value, &dap_response, 35);
  jtag_to_state(JTAG_STATE_SHIFT_DR);

  dr_value = ((0xc >> 2) & 3) << 1;
  dr_value |= 1;

  jtag_shift(dr_value, &dap_response, 35);
  ack = dap_response & 7;

  return ack == ACK_OK;
}

static void adiv5_mem_ap_init(struct adiv5_dap *d)
{
  adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_CSW, 0, &d->csw);
  adiv5_transfer(d, AP, OP_WRITE, AP_REG_ADDR_CSW,  d->csw | 0xffffffff, &d->csw);
  adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_CSW, 0, &d->csw);
  adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
}

#define ADIV5_CSW_SIZE_POS     0
#define ADIV5_CSW_ADDRINCR_POS 4

#define ADIV5_CSW_SIZE_MASK     (7 << ADIV5_CSW_SIZE_POS)
#define ADIV5_CSW_ADDRINCR_MASK (3 << ADIV5_CSW_ADDRINCR_POS)

void adiv5_mem_ap_set_csw(struct adiv5_dap *d, int op_size, bool addrinc)
{
  uint32_t new_csw = d->csw;
  new_csw &= ~ADIV5_CSW_SIZE_MASK;
  new_csw &= ~ADIV5_CSW_ADDRINCR_MASK;

  if (addrinc)
    new_csw |= (1 << ADIV5_CSW_ADDRINCR_POS) & ADIV5_CSW_ADDRINCR_MASK;

  if (op_size == 1)
    new_csw |= (0 << ADIV5_CSW_SIZE_POS) & ADIV5_CSW_SIZE_MASK;
  else if (op_size == 2)
    new_csw |= (1 << ADIV5_CSW_SIZE_POS) & ADIV5_CSW_SIZE_MASK;
  else if (op_size == 4)
    new_csw |= (2 << ADIV5_CSW_SIZE_POS) & ADIV5_CSW_SIZE_MASK;
  else if (op_size == 8)
    new_csw |= (3 << ADIV5_CSW_SIZE_POS) & ADIV5_CSW_SIZE_MASK;
  else if (op_size == 16)
    new_csw |= (4 << ADIV5_CSW_SIZE_POS) & ADIV5_CSW_SIZE_MASK;
  else if (op_size == 32)
    new_csw |= (5 << ADIV5_CSW_SIZE_POS) & ADIV5_CSW_SIZE_MASK;

  adiv5_transfer(d, AP, OP_WRITE, AP_REG_ADDR_CSW, new_csw, &d->csw);
}

#define EXIT_ON_FAIL() \
  do { \
    if (!success) \
      return false; \
  } while(0)


bool adiv5_dap_init(struct adiv5_dap *d)
{
  uint32_t reg;
  bool success;

  d->select = 0xffffffff;
  d->ir = 0xffffffff;

  success = adiv5_dap_init_power(d);
  EXIT_ON_FAIL();

  success = adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_TARGETID, 0, &d->target_id);
  EXIT_ON_FAIL();

  success = adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_BASE, 0, &d->base);
  EXIT_ON_FAIL();

  success = adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_DPIDR, 0, &d->dpidr);
  EXIT_ON_FAIL();

  success = adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_DLPIDR, 0, &d->dlpidr);
  EXIT_ON_FAIL();

  success = adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_IDR, 0, &d->idr);
  EXIT_ON_FAIL();

  success = adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_CFG, 0, &reg);
  EXIT_ON_FAIL();

  success = adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
  EXIT_ON_FAIL();

  adiv5_mem_ap_init(d);
  return true;
}

void adiv5_mem_ap_read_word_drw(struct adiv5_dap *d, uint32_t addr,
  uint32_t *out)
{
  uint32_t data = 0;
  uint32_t check_tar = 0;

  adiv5_transfer(d, AP, OP_WRITE, AP_REG_ADDR_TAR, addr, &data);
  adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_TAR, 0, &check_tar);
  adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_DRW, 0, out);
  adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
  if (d->ctl_stat & 0x20) {
    last_success = false;
    adiv5_transfer(d, DP, OP_WRITE, DP_REG_ADDR_CTRL_STAT, d->ctl_stat,
      &data);

    adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
  }
  else
    last_success = true;
}


void adiv5_mem_ap_read_word(struct adiv5_dap *d, uint32_t addr, uint32_t *out)
{
  uint32_t data = 0;
  uint32_t check_tar = 0;
  adiv5_transfer(d, AP, OP_WRITE, AP_REG_ADDR_TAR, addr & 0xfffffff0, &data);
  adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_TAR, 0, &check_tar);
  adiv5_transfer(d, AP, OP_READ, AP_REG_ADDR_BD0 | (addr & 0xc), 0, out);
  adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
  if (d->ctl_stat & 0x20) {
    last_success = false;
    adiv5_transfer(d, DP, OP_WRITE, DP_REG_ADDR_CTRL_STAT, d->ctl_stat, &data);
    adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
  }
  else
    last_success = true;
}


void adiv5_mem_ap_write_word(struct adiv5_dap *d, uint32_t addr,
  uint32_t value)
{
  uint32_t data = 0;
  adiv5_transfer(d, AP, OP_WRITE, AP_REG_ADDR_TAR, addr, &data);
  adiv5_transfer(d, AP, OP_WRITE, AP_REG_ADDR_DRW, value, &data);
  adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
  if (d->ctl_stat & 0x20) {
    adiv5_transfer(d, DP, OP_WRITE, DP_REG_ADDR_CTRL_STAT, d->ctl_stat, &data);
    adiv5_transfer(d, DP, OP_READ, DP_REG_ADDR_CTRL_STAT, 0, &d->ctl_stat);
    last_success = false;
  }
  else
    last_success = true;
}

