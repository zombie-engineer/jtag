import time
import sdhc
import bcm_gpio
import struct
# [31:29] State machine state Used for debugging SDHOST internal FSM state.
# [28:24] DMA FIFO status / level Approximate FIFO depth (0–15); for tuning performance or debugging.
# [1] Reset done flag May indicate completion of a soft reset.
# [0] Controller active / in-command  Indicates SDHOST is actively processing a command or busy.

PERIPHERAL_BASE = 0x3f000000
SDHOST_BASE = PERIPHERAL_BASE + 0x202000
SDHOST_CMD   = SDHOST_BASE + 0x00
SDHOST_ARG   = SDHOST_BASE + 0x04
SDHOST_TOUT  = SDHOST_BASE + 0x08
SDHOST_CDIV  = SDHOST_BASE + 0x0c
SDHOST_RESP0 = SDHOST_BASE + 0x10
SDHOST_RESP1 = SDHOST_BASE + 0x14
SDHOST_RESP2 = SDHOST_BASE + 0x18
SDHOST_RESP3 = SDHOST_BASE + 0x1c
SDHOST_HSTS  = SDHOST_BASE + 0x20
SDHOST_CNT   = SDHOST_BASE + 0x28
SDHOST_VDD   = SDHOST_BASE + 0x30
SDHOST_EDM   = SDHOST_BASE + 0x34
SDHOST_HCFG  = SDHOST_BASE + 0x38
SDHOST_HBCT  = SDHOST_BASE + 0x3c
SDHOST_DATA  = SDHOST_BASE + 0x40
SDHOST_HBLC  = SDHOST_BASE + 0x50


SDHOST_CMD_NEW_FLAG     = (1<<15)
SDHOST_CMD_FAILED       = (1<<14)
SDHOST_CMD_BUSY         = (1<<11)
SDHOST_CMD_NO_RESPONSE  = (1<<10)
SDHOST_CMD_RESPONSE_136 = (1<<9)
SDHOST_CMD_READ         = (1<<6)
SDHOST_CMD_COMMAND_MASK = 0x003f

SDHSTS_BUSY_IRPT    = 0x400
SDHSTS_BLOCK_IRPT   = 0x200
SDHSTS_SDIO_IRPT    = 0x100
SDHSTS_REW_TIME_OUT = 0x80
SDHSTS_CMD_TIME_OUT = 0x40
SDHSTS_CRC16_ERROR  = 0x20
SDHSTS_CRC7_ERROR   = 0x10
SDHSTS_FIFO_ERROR   = 0x08
SDHSTS_DATA_FLAG    = 0x01

SDHSTS_TRANSFER_ERROR_MASK = SDHSTS_CRC7_ERROR | SDHSTS_CRC16_ERROR | SDHSTS_REW_TIME_OUT | SDHSTS_FIFO_ERROR
SDHSTS_ERROR_MASK = SDHSTS_CMD_TIME_OUT | SDHSTS_TRANSFER_ERROR_MASK


SDHOST_HSTS_DATA = 1

class SDHOST_REGS:
  def __init__(self, t):
    self.__t = t

  def cmd_read(self):
    return self.__t.mem_read32(SDHOST_CMD)

  def cmd_write(self, v):
    return self.__t.mem_write32(SDHOST_CMD, v)

  def arg_read(self):
    return self.__t.mem_read32(SDHOST_ARG)

  def arg_write(self, v):
    return self.__t.mem_write32(SDHOST_ARG, v)

  def tout_read(self):
    return self.__t.mem_read32(SDHOST_TOUT)

  def tout_write(self, v):
    return self.__t.mem_write32(SDHOST_TOUT, v)

  def cdiv_read(self):
    return self.__t.mem_read32(SDHOST_CDIV)

  def cdiv_write(self, v):
    return self.__t.mem_write32(SDHOST_CDIV, v)

  def resp0_read(self):
    return self.__t.mem_read32(SDHOST_RESP0)

  def resp1_read(self):
    return self.__t.mem_read32(SDHOST_RESP1)

  def resp2_read(self):
    return self.__t.mem_read32(SDHOST_RESP2)

  def resp3_read(self):
    return self.__t.mem_read32(SDHOST_RESP3)

  def hsts_read(self):
    return self.__t.mem_read32(SDHOST_HSTS)

  def hsts_write(self, v):
    return self.__t.mem_write32(SDHOST_HSTS, v)

  def cnt_read(self):
    return self.__t.mem_read32(SDHOST_CNT)

  def cnt_write(self, v):
    return self.__t.mem_write32(SDHOST_CNT, v)

  def vdd_read(self):
    return self.__t.mem_read32(SDHOST_VDD)

  def vdd_write(self, v):
    return self.__t.mem_write32(SDHOST_VDD, v)

  def edm_read(self):
    return self.__t.mem_read32(SDHOST_EDM)

  def edm_write(self, v):
    return self.__t.mem_write32(SDHOST_EDM, v)

  def hcfg_read(self):
    return self.__t.mem_read32(SDHOST_HCFG)

  def hcfg_write(self, v):
    return self.__t.mem_write32(SDHOST_HCFG, v)

  def hbct_read(self):
    return self.__t.mem_read32(SDHOST_HBCT)

  def hbct_write(self, v):
    return self.__t.mem_write32(SDHOST_HBCT, v)

  def data_read(self):
    return self.__t.mem_read32(SDHOST_DATA)

  def data_write(self, v):
    return self.__t.mem_write32(SDHOST_DATA, v)

  def hblc_read(self):
    return self.__t.mem_read32(SDHOST_HBLC)

  def hblc_write(self, v):
    return self.__t.mem_write32(SDHOST_HBLC, v)


class SDHOST:
  def __init__(self, gpio, t, log):
    self.__gpio = gpio
    self.__r = SDHOST_REGS(t)
    self.__t = t
    self.__log = log

  def init_gpio(self):
    # Configure GPIO 48–53 for SDHOST (ALT0 function = 4)
    for pin in range(48, 54):
      # 4 = ALT0
      self.__gpio.pin_configure(pin, 4)
    for pin in range(49, 54):
      self.__gpio.pin_config_pull_up_down(pin, bcm_gpio.GPIO_PULL_UP)


  def power_down(self):
    self.__r.vdd_write(0)

  def power_up(self):
    self.__r.vdd_write(1)

  def reset(self):

    print('SDHOST::reset')
    print(f'HCFG:0x{self.__r.hcfg_read():08x}')
    print(f'HSTS:0x{self.__r.hsts_read():08x}')
    print(f'VDD:0x{self.__r.vdd_read():08x}')
    print(f'CDIV:0x{self.__r.cdiv_read():08x}')
    print(f'TOUT:0x{self.__r.tout_read():08x}')

    self.power_down()
    self.__r.cmd_write(0)
    self.__r.arg_write(0)
    self.__r.tout_write(0xf00000)
    self.__r.cdiv_write(0)
    self.__r.hsts_write(0x7f8)
    self.__r.hcfg_write(0)
    self.__r.hbct_write(0)
    self.__r.hblc_write(0)
    edm = self.__r.edm_read()
    print(f'EDM:0x{edm:08x}')
    time.sleep(300 * 0.001 * 0.001)
    self.power_up()

    hcfg = self.__r.hcfg_read()
    hcfg &= ~0x4
    self.__r.hcfg_write(hcfg)
    self.__r.hcfg_write(0x8 | 0x2)
    self.__r.cdiv_write(0x7ff)
    time.sleep(300 * 0.001 * 0.001)
    print('reset done')
    print(f'HCFG:0x{self.__r.hcfg_read():08x}')
    print(f'HSTS:0x{self.__r.hsts_read():08x}')
    print(f'VDD:0x{self.__r.vdd_read():08x}')
    print(f'CDIV:0x{self.__r.cdiv_read():08x}')
    print(f'TOUT:0x{self.__r.tout_read():08x}')

  def __gen_cmd(self, is_acmd, cmd_idx):
    dbase = sdhc.sd_acommands if is_acmd else sdhc.sd_commands
    resp_type, crc, direction, is_data, multiblock = dbase[cmd_idx]
    result = cmd_idx & SDHOST_CMD_COMMAND_MASK
    result |= SDHOST_CMD_NEW_FLAG
    if resp_type == sdhc.RESP_TYPE_NA:
      result |= SDHOST_CMD_NO_RESPONSE
    if resp_type == sdhc.RESP_TYPE_R2:
      result |= SDHOST_CMD_RESPONSE_136
    if direction == sdhc.DIR_CARD2HOST:
      result |= SDHOST_CMD_READ
    return result

  def __wait_data_rdy(self):
    while True:
      status = self.__r.hsts_read()
      if status & SDHOST_HSTS_DATA:
        print(f'status: {status:08x}')
        return status

  def cmd(self, is_acmd, cmd_idx, blksize, arg, read_resp=False, data_size=0):
    if blksize:
      num_blocks = int(data_size / blksize)
    else:
      num_blocks = data_size
    print(f'CMD{cmd_idx}, blksize: {blksize}, data_size: {data_size}, numblocks:{num_blocks}')

    self.__t.debug_tty_read = True
    self.__t.debug_tty_write = True
    cmd_type = 'ACMD' if is_acmd else 'CMD'
    self.__log.debug(f'Running {cmd_type}{cmd_idx}')

    while True:
      cmdreg = self.__r.cmd_read()
      self.__log.info(f'cmd:{cmdreg:08x}')
      if cmdreg & SDHOST_CMD_NEW_FLAG == 0:
        break

    hsts = self.__r.hsts_read()
    if hsts & SDHSTS_ERROR_MASK:
      print('Resetting error status {hsts:08x}')
      self.__r.hsts_write(hsts)

    self.__r.hbct_write(blksize)
    self.__r.hblc_write(num_blocks)
    self.__r.arg_write(arg)
#
#    intr = self.interrupt_read()
#    if intr:
#      self.__log.warning(f'clearing stale interrupt {intr:08x}')
#      self.interrupt_write(intr)
#
    cmdreg = self.__gen_cmd(is_acmd, cmd_idx)
    print(f'CMD{cmd_idx}: {cmdreg:08x}, arg:{arg:08x}')
    self.__r.cmd_write(cmdreg)

    while True:
      cmdreg = self.__r.cmd_read()
      self.__log.debug(f'cmd:{cmdreg:08x}')
      if cmdreg & SDHOST_CMD_NEW_FLAG == 0:
        break

    if cmdreg & SDHOST_CMD_FAILED:
      hsts = self.__r.hsts_read()
      raise Exception(f'CMD{cmd_idx} failed CMD:{cmdreg:08x}, HSTS:{hsts:08x}')

    data = None
    resp0 = None
    resp1 = None
    resp2 = None
    resp3 = None

    if read_resp:
      resp0 = self.__r.resp0_read()
      resp1 = self.__r.resp1_read()
      resp2 = self.__r.resp2_read()
      resp3 = self.__r.resp3_read()

    data = b''
    num_words32 = int(data_size / 4)

    old_debug_value = self.__t.debug_tty_read
    old_debug_value2 = self.__t.debug_tty_write
    self.__t.debug_tty_read = False
    self.__t.debug_tty_write = False
    status = 0
    for i in range(num_words32):
      done = float(i) / num_words32
      full = int(10 * done)
      cursor = '=' * full + '>'

      iters = 0
      status = self.__wait_data_rdy()
      d = self.__r.data_read()
      print(f'\r{cursor}{i}/{num_words32} {status:08x} {d:08x}', end='')
      data += struct.pack('I', d)

    if num_words32:
      print()
    print('>done')

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

  def set_bus_width4(self):
    hcfg = self.__r.hcfg_read()
    self.__r.hcfg_write(hcfg | 4)
    print('sdhost: set_bus_width4')

  def set_high_speed(self):
    print('sdhost: set_high_speed')
