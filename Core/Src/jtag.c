#include "jtag.h"
#include "jtag_gpio.h"

// static const int irlen0 = 5;
// static const int irlen1 = 4;

static uint64_t tdo_history = 0;

static jtag_state_t jtag_state;

static uint8_t jtag_next(uint8_t tms, uint8_t databit)
{
  uint8_t tdo_value;
  if (tms)
    TMS_HI();
  else
    TMS_LO();

  if (databit)
    TDI_HI();
  else
    TDI_LO();

  tdo_value = TDO_READ();
  TCLK_HI();
  TCLK_LO();

  tdo_history = (tdo_history >> 1) | (((uint64_t)tdo_value) << 63);

  return tdo_value;
}

static void jtag_tms_seq(uint32_t value, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    jtag_next(value & 1, 1);
    value >>= 1;
  }
}

void jtag_soft_reset(void)
{
  jtag_tms_seq(0b11111, 5);
  jtag_state = JTAG_STATE_TEST_LOGIC_RESET;
}

bool jtag_to_state(jtag_state_t to)
{
  const struct jtag_state_path *path;

  path = jtag_get_state_path(jtag_state, to);
  if (!path)
    return false;

  jtag_tms_seq(path->path, path->n);
  jtag_state = to;
  return true;
}

#if 0
void jtag_shift_to_update_ir(void)
{
  jtag_tms_seq(0x03, 4);
}
#endif

void jtag_init(void)
{
  jtag_tms_init();
  for (int i = 0; i < 50; ++i)
    jtag_next(1, 0);

  jtag_tms_seq(0xe73c, 16);
  jtag_soft_reset();
}

void jtag_reset(void)
{
  TRST_LO();
  for (volatile int i = 0; i < 10000; ++i)
    asm("nop");
  TRST_HI();
  jtag_soft_reset();
}

void jtag_shift(uint64_t value, uint64_t *out_value, size_t num_bits)
{
  size_t i;
  uint64_t out = 0;
  uint64_t out_bit;
  uint8_t tms;

  for (i = 0; i < num_bits; ++i) {
    uint8_t bit = (value >> i) & 1;

    tms = (i == num_bits - 1) ? 1: 0;

    out_bit = jtag_next(tms, bit) & 1;
    if (tms)
      jtag_state++;

    out |= out_bit << i;
  }

  *out_value = out;
}

int jtag_scan_num_devs(void)
{
  __attribute__((unused)) int num_devs;
  size_t i;
  size_t irchain_len = 4;

  jtag_to_state(JTAG_STATE_SHIFT_IR);

  /* Set to BYPASS mode */
  for (i = 0; i < irchain_len; ++i)
    jtag_next(0, 1);

  /* SHIFT-IR to SHIFT-DR */
  jtag_to_state(JTAG_STATE_SHIFT_DR);

  /* Set a lot of zeroes */
  for (i = 0; i < 500; ++i)
    jtag_next(0, 0);

  /* Expect delay of 2 clocks before seeing first 1 */
  for (i = 0; i < 500; ++i)
    if (jtag_next(0, 1))
      break;
  
  /* SHIFT-DR to Run-Test/Idle */
  jtag_soft_reset();
  num_devs = i;
  return num_devs;
}

uint32_t jtag_read_idcode(void)
{
  uint32_t idcode;
  int last_ir;

  jtag_to_state(JTAG_STATE_SHIFT_DR);
  idcode = 0;
  for (int i = 0; i < 32; ++i) {
    last_ir = jtag_next(0, 0);
    idcode |= last_ir << i;
  }

  return idcode;
}
