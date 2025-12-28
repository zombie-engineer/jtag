from sd import *
import struct
import time

SDHCI_BASE = 0x3f300000
SDHCI_ARG2      = SDHCI_BASE + 0x00
SDHCI_BLKSZCNT  = SDHCI_BASE + 0x04
SDHCI_ARG1      = SDHCI_BASE + 0x08
SDHCI_CMDTM     = SDHCI_BASE + 0x0c
SDHCI_RESP0     = SDHCI_BASE + 0x10
SDHCI_RESP1     = SDHCI_BASE + 0x14
SDHCI_RESP2     = SDHCI_BASE + 0x18
SDHCI_RESP3     = SDHCI_BASE + 0x1c
SDHCI_DATA      = SDHCI_BASE + 0x20
SDHCI_STATUS    = SDHCI_BASE + 0x24
SDHCI_CONTROL0  = SDHCI_BASE + 0x28
SDHCI_CONTROL1  = SDHCI_BASE + 0x2c
SDHCI_INTERRUPT = SDHCI_BASE + 0x30
SDHCI_INT_MASK  = SDHCI_BASE + 0x34
SDHCI_INT_EN    = SDHCI_BASE + 0x38
SDHCI_CONTROL2  = SDHCI_BASE + 0x3c
SDHCI_CAPS_0    = SDHCI_BASE + 0x40
SDHCI_CAPS_1    = SDHCI_BASE + 0x44

SDHCI_STATUS_CMD_INHIBIT  = 1<<0
SDHCI_STATUS_DAT_INHIBIT  = 1<<1
SDHCI_STATUS_DAT_ACTIVE   = 1<<2
SDHCI_STATUS_WRITE_ACTIVE = 1<<8
SDHCI_STATUS_READ_ACTIVE  = 1<<9

SDHCI_CONTROL1_INT_CLK_ENA       = 0
SDHCI_CONTROL1_INT_CLK_STABLE    = 1
SDHCI_CONTROL1_SD_CLK_ENA        = 2
SDHCI_CONTROL1_PLL_ENA           = 3
SDHCI_CONTROL1_CLK_GENERATOR_SEL = 4
SDHCI_CONTROL1_SDCLK_DIV_HI_BITS = 6
SDHCI_CONTROL1_SDCLK_DIV_LO_BITS = 8


RESP_TYPE_NONE         = 0
RESP_TYPE_136_BITS     = 1
RESP_TYPE_48_BITS      = 2
RESP_TYPE_48_BITS_BUSY = 3


cmdtm_resp_types = {
  RESP_TYPE_R1  : RESP_TYPE_48_BITS,
  RESP_TYPE_R1b : RESP_TYPE_48_BITS_BUSY,
  RESP_TYPE_R2  : RESP_TYPE_136_BITS,
  RESP_TYPE_R3  : RESP_TYPE_48_BITS,
  RESP_TYPE_R6  : RESP_TYPE_48_BITS,
  RESP_TYPE_R7  : RESP_TYPE_48_BITS,
  RESP_TYPE_NA  : RESP_TYPE_NONE
}


def bitplace(v, bitpos, width):
  return (v & ((1 << width) - 1)) << bitpos


def byteswap32(v):
  b0 = v & 0xff
  b1 = (v >> 8) & 0xff
  b2 = (v >> 16) & 0xff
  b3 = (v >> 24) & 0xff
  return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3


def gen_cmdtm(is_acmd, cmd_idx):
  dbase = sdhc.sd_acommands if is_acmd else sdhc.sd_commands
  resp_type, crc, direction, is_data, multiblock = dbase[cmd_idx]
  resp_type = cmdtm_resp_types[resp_type]
  return \
      bitplace(direction ,  4, 1) \
    | bitplace(multiblock,  5, 1) \
    | bitplace(resp_type , 16, 2) \
    | bitplace(crc       , 19, 1) \
    | bitplace(is_data   , 21, 1) \
    | bitplace(cmd_idx   , 24, 6) \



class SDHCI:
  def __init__(self, t, log):
    self.__t = t
    self.__log = log

  def arg2_read(self):
    return self.__t.mem_read32(SDHCI_ARG2)[0]

  def blkszcnt_read(self):
    return self.__t.mem_read32(SDHCI_BLKSZCNT)[0]

  def blkszcnt_write(self, v):
    self.__t.mem_write32(SDHCI_BLKSZCNT, v)

  def arg1_read(self):
    return self.__t.mem_read32(SDHCI_ARG1)[0]

  def arg1_write(self, v):
    return self.__t.mem_write32(SDHCI_ARG1, v)

  def cmdtm_read(self):
    return self.__t.mem_read32(SDHCI_CMDTM)[0]

  def cmdtm_write(self, v):
    return self.__t.mem_write32(SDHCI_CMDTM, v)

  def resp0_read(self):
    return self.__t.mem_read32(SDHCI_RESP0)[0]

  def resp1_read(self):
    return self.__t.mem_read32(SDHCI_RESP1)[0]

  def resp2_read(self):
    return self.__t.mem_read32(SDHCI_RESP2)[0]

  def resp3_read(self):
    return self.__t.mem_read32(SDHCI_RESP3)[0]

  def data_read(self):
    return self.__t.mem_read32(SDHCI_DATA)[0]

  def status_read(self):
    return self.__t.mem_read32(SDHCI_STATUS)[0]

  def control0_write(self, v):
    self.__t.mem_write32(SDHCI_CONTROL0, v)

  def control0_read(self):
    return self.__t.mem_read32(SDHCI_CONTROL0)[0]

  def control1_read(self):
    return self.__t.mem_read32(SDHCI_CONTROL1)[0]

  def control1_write(self, v):
    self.__t.mem_write32(SDHCI_CONTROL1, v)

  def interrupt_read(self):
    return self.__t.mem_read32(SDHCI_INTERRUPT)[0]

  def interrupt_write(self, v):
    self.__t.mem_write32(SDHCI_INTERRUPT, v)

  def int_mask_read(self):
    return self.__t.mem_read32(SDHCI_INT_MASK)[0]

  def int_mask_write(self, v):
    self.__t.mem_write32(SDHCI_INT_MASK, v)

  def int_en_read(self):
    return self.__t.mem_read32(SDHCI_INT_EN)[0]

  def int_en_write(self, v):
    self.__t.mem_write32(SDHCI_INT_EN, v)

  def control2_read(self):
    return self.__t.mem_read32(SDHCI_CONTROL2)[0]

  def control2_write(self, v):
    self.__t.mem_write32(SDHCI_CONTROL2, v)

  def caps_0_read(self):
    return self.__t.mem_read32(SDHCI_CAPS_0)[0]

  def caps_1_read(self):
    return self.__t.mem_read32(SDHCI_CAPS_1)[0]

  def set_clock(self, setup):
 #     div = 4 if setup else 64
 #     v = self.control1_read()
 #     v &= ~(0xf << 16)
 #     v |= 0xb << 16
 #     v |= 1
 #     v |= div << 8
 #     self.control1_write(v)
    INTERNAL_CLK_ENABLE = 1
    SD_CLK_ENABLE       = 1 << 2
    if setup:
      div10 = 64
    else:
      div10 = 5

    div_hi = (div10 >> 8) & 3
    div_lo = div10 & 0xff
    timeout = 0x0f << 16

    self.control1_write(
      (div_lo << 8) | (div_hi << 6)
      | SD_CLK_ENABLE
      | INTERNAL_CLK_ENABLE
      | timeout)
    while True:
      v = self.control1_read()
      if v & (1<<1):
        break

  def sd_clock_start(self):
    v = self.control1_read()
    v |= 1 << SDHCI_CONTROL1_SD_CLK_ENA
    self.control1_write(v)
    v = self.control1_read()

  def sd_clock_stop(self):
    v = self.control1_read()
    v &= ~(1 << SDHCI_CONTROL1_SD_CLK_ENA)
    self.control1_write(v)
    v = self.control1_read()

  def sw_reset_cmd(self):
    v = self.control1_read()
    v |= 1<<25
    self.control1_write(v)
    while True:
      v = self.control1_read()
      if ((v & (1<<25)) == 0):
        break
    self.__log.info('reset cmd done')

  def sw_reset_all(self):
    v = self.control1_read()
    v |= 1<<24
    self.control1_write(v)
    while True:
      v = self.control1_read()
      if ((v & (1<<24)) == 0):
        break
    self.__log.info('emmc sw reset all done')

  def internal_clock_stop(self):
    v = self.control1_read()
    v &= ~(1<<SDHCI_CONTROL1_INT_CLK_ENA)
    self.control1_write(v)
    while True:
      v = self.control1_read()
      if (v & (1<<SDHCI_CONTROL1_INT_CLK_STABLE)) == 0:
        break

  def internal_clock_start(self):
    v = self.control1_read()
    v |= 1 << SDHCI_CONTROL1_INT_CLK_ENA
    self.control1_write(v)
    while True:
      v = self.control1_read()
      if (v & (1<<SDHCI_CONTROL1_INT_CLK_STABLE)):
        break

  def set_bus_width4(self):
    v = self.control0_read()
    CONTROL0_DWIDTH4_BIT = 1 << 1
    v |= CONTROL0_DWIDTH4_BIT
    self.control0_write(v)
    self.__log.info('data bus width set to 4')

  def set_high_speed(self):
    intr = self.interrupt_read()
    self.__log.info(f'Setting high speed bit. Interrupt: {intr:08x}')

    if intr:
      self.interrupt_write(intr)

    # self.sd_clock_stop()
    v = self.control0_read()
    CONTROL0_HIGH_SPEED_BIT = 1 << 2
    v |= CONTROL0_HIGH_SPEED_BIT
    self.control0_write(v)
    self.set_clock(setup=False)
    # self.sd_clock_start()
    intr = self.interrupt_read()
    if intr:
      self.interrupt_write(intr)
    self.__log.info(f'High speed bit is set. Interrupt: {intr:08x}')
    time.sleep(1)

  def stop(self):
    self.internal_clock_stop()
    self.internal_clock_stop()

  def reset(self):
    # Software reset.
    self.sw_reset_all()
    # Restart internal SDHCI clock
    self.internal_clock_stop()
    self.internal_clock_start()
    # Set CONTROL0 to default
    self.control0_write(0)
    # Set CONTROL1 to default
    self.control2_write(0)
    self.set_clock(setup=True)
    self.sd_clock_start()
    self.int_mask_write(0xffffffff)
    self.int_en_write(0xffffffff)

  def interrupt_dump_err_values(self, intr):
    warnstring = f'intr {intr:08x}'
    errbits = [
      'CMD_TIMEOUT', 'CMD_CRC', 'CMD_END_BIT', 'CMD_IDX',
      'DAT_TIMEOUT', 'DAT_CRC', 'DAT_END_BIT', 'CURR_LIM',
      'AUTO_CMD', 'ADMA', 'TUNING', 'RESP',
      'INTR12', 'INTR13', 'INTR14', 'INTR15'
    ]
    for i in range(16):
      if intr & (1<<(i+16)):
        warnstring += f',{errbits[i]}'
    self.__log.warning(warnstring)

  def cmd_wait_intr_done(self):
    INTR_CMD_DONE  = 1 << 0
    INTR_DATA_DONE = 1 << 1
    INTR_ERR       = 1 << 15

    while True:
      status = self.status_read()
      self.__log.debug(f'status(after): {status:08x}')
      intr = self.interrupt_read()
      if intr & INTR_CMD_DONE:
        break
      if intr & INTR_ERR:
        self.interrupt_dump_err_values(intr)
        raise Exception(f'interrupt {intr:08x}')
    self.interrupt_write(intr)
    intr2 = self.interrupt_read()
    if intr2 & intr:
      self.__log.warning(f"leftover interrupt: {intr2:08x}")


  def cmd(self, is_acmd, cmd_idx, blksize, arg1, read_resp=False, data_size=0):
    cmd_type = 'ACMD' if is_acmd else 'CMD'
    self.__log.debug(f'Running {cmd_type}{cmd_idx}')
    while True:
      status = self.status_read()
      self.__log.debug(f'status:{status:08x}')
      if (status & 1) == 0:
        break

    self.blkszcnt_write(blksize)

    self.arg1_write(arg1)

    intr = self.interrupt_read()
    if intr:
      self.__log.warning(f'clearing stale interrupt {intr:08x}')
      self.interrupt_write(intr)

    self.cmdtm_write(gen_cmdtm(is_acmd, cmd_idx))

    self.cmd_wait_intr_done()
    data = None
    resp0 = None
    resp1 = None
    resp2 = None
    resp3 = None

    if read_resp:
      resp0 = self.resp0_read()
      resp1 = self.resp1_read()
      resp2 = self.resp2_read()
      resp3 = self.resp3_read()

    data = b''
    num_words32 = int(data_size / 4)

    old_debug_value = self.__t.debug_tty_read
    old_debug_value2 = self.__t.debug_tty_write
    self.__t.debug_tty_read = False
    self.__t.debug_tty_write = False
    for i in range(num_words32):
      done = float(i) / num_words32
      full = int(10 * done)
      cursor = '=' * full + '>'

      iters = 0
      while True:
        status = self.status_read()
        print(f'\r{cursor}{i}/{num_words32} {status:08x} ({iters})', end='')
        iters += 1
        if status & (1<<9):
          break
      d = self.data_read()
      print(f'\r{cursor}{i}/{num_words32} {status:08x} {d:08x}', end='')
      data += struct.pack('I', d)
    if num_words32:
      print()
    self.__t.debug_tty_read = old_debug_value
    self.__t.debug_tty_write = old_debug_value2

    self.__log.debug('Finished CMD{}, resp {:08x} {:08x} {:08x} {:08x}, data:{}'.format(
      cmd_idx,
      resp0 or 0,
      resp1 or 0,
      resp2 or 0,
      resp3 or 0,
      data))

    return sdhc.cmd_result(status, data, resp0, resp1, resp2, resp3)

