import serial
import time
import sys
import struct
import time

RESP_TYPE_NONE         = 0
RESP_TYPE_136_BITS     = 1
RESP_TYPE_48_BITS      = 2
RESP_TYPE_48_BITS_BUSY = 3

RESP_TYPE_R1  = 1
RESP_TYPE_R1b = 2
RESP_TYPE_R2  = 3
RESP_TYPE_R3  = 4
RESP_TYPE_R6  = 5
RESP_TYPE_R7  = 6
RESP_TYPE_NA  = 7

cmdtm_resp_types = {
  RESP_TYPE_R1  : RESP_TYPE_48_BITS,
  RESP_TYPE_R1b : RESP_TYPE_48_BITS_BUSY,
  RESP_TYPE_R2  : RESP_TYPE_136_BITS,
  RESP_TYPE_R3  : RESP_TYPE_48_BITS,
  RESP_TYPE_R6  : RESP_TYPE_48_BITS,
  RESP_TYPE_R7  : RESP_TYPE_48_BITS,
  RESP_TYPE_NA  : RESP_TYPE_NONE
}

DIR_UNKNOWN   = 0
DIR_HOST2CARD = 0
DIR_CARD2HOST = 1

CMD6_CHECK_FUNCTION  = 0
CMD6_SWITCH_FUNCTION = 1


# Entry format CMD_ID : (RESP_TYPE, CHECK_CRC, DIR, HAS_DATA, MULTIBLOCK)
sd_commands = {
   0 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
   1 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
   2 : (RESP_TYPE_R2,   0, DIR_UNKNOWN  , 0, 0),
   3 : (RESP_TYPE_R6,   1, DIR_UNKNOWN  , 0, 0),
   4 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
   5 : (RESP_TYPE_R1b,  0, DIR_UNKNOWN  , 0, 0),
   6 : (RESP_TYPE_R1,   1, DIR_CARD2HOST, 1, 0),
   7 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
   8 : (RESP_TYPE_R7,   1, DIR_UNKNOWN  , 0, 0),
   9 : (RESP_TYPE_R2,   0, DIR_UNKNOWN  , 0, 0),
  10 : (RESP_TYPE_R2,   0, DIR_UNKNOWN  , 0, 0),
  11 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  12 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  13 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  14 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  15 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  16 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  17 : (RESP_TYPE_R1,   1, DIR_CARD2HOST, 1, 0),
  18 : (RESP_TYPE_R1,   1, DIR_CARD2HOST, 1, 1),
  19 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  20 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  21 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  22 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  23 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  24 : (RESP_TYPE_R1,   1, DIR_HOST2CARD, 1, 0),
  25 : (RESP_TYPE_R1,   1, DIR_HOST2CARD, 1, 1),
  26 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  27 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  28 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  29 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  30 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  31 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  32 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  33 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  34 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  35 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  36 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  37 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  38 : (RESP_TYPE_R1b,  1, DIR_UNKNOWN  , 0, 0),
  39 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  40 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  41 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  42 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  43 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  44 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  45 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  46 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  47 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  48 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  49 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  50 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  51 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  52 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  53 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  54 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  55 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  56 : (RESP_TYPE_R1,   1, DIR_UNKNOWN  , 0, 0),
  57 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  58 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0),
  59 : (RESP_TYPE_NA,   0, DIR_UNKNOWN  , 0, 0)
}

sd_acommands = {
   0 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   1 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   2 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   3 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   4 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   5 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   6 : (RESP_TYPE_R1,    1, DIR_UNKNOWN, 0, 0),
   7 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   8 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
   9 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  10 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  11 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  12 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  13 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  14 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  15 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  16 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  17 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  18 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  19 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  20 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  21 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  22 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  23 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  24 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  25 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  26 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  27 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  28 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  29 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  30 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  31 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  32 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  33 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  34 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  35 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  36 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  37 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  38 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  39 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  40 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  41 : (RESP_TYPE_R3,    0, DIR_UNKNOWN, 0, 0),
  42 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  43 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  44 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  45 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  46 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  47 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  48 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  49 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  50 : (RESP_TYPE_R1,    1, DIR_UNKNOWN, 0, 0),
  51 : (RESP_TYPE_R1,    1, DIR_CARD2HOST, 1, 0),
  52 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  53 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  54 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  55 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  56 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  57 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  58 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0),
  59 : (RESP_TYPE_NA,    0, DIR_UNKNOWN, 0, 0)
}

CARD_STATE_IDLE           = 0
CARD_STATE_READY          = 1
CARD_STATE_IDENTIFICATION = 2
CARD_STATE_STANDBY        = 3
CARD_STATE_TRAN           = 4
CARD_STATE_DATA           = 5
CARD_STATE_RECV           = 5
CARD_STATE_PROG           = 6
CARD_STATE_DISCARD        = 7
CARD_STATE_UNKNOWN        = 0xff

card_states = {
  CARD_STATE_IDLE            : 'idle',
  CARD_STATE_READY           : 'ready',
  CARD_STATE_IDENTIFICATION  : 'identification',
  CARD_STATE_STANDBY         : 'standby',
  CARD_STATE_TRAN            : 'tran',
  CARD_STATE_DATA            : 'data',
  CARD_STATE_RECV            : 'recv',
  CARD_STATE_PROG            : 'prog',
  CARD_STATE_DISCARD         : 'discard',
  CARD_STATE_UNKNOWN         : 'unknown',
}

all_states = [
  CARD_STATE_IDLE,
  CARD_STATE_READY,
  CARD_STATE_IDENTIFICATION,
  CARD_STATE_STANDBY,
  CARD_STATE_TRAN,
  CARD_STATE_DATA,
  CARD_STATE_RECV,
  CARD_STATE_PROG,
  CARD_STATE_DISCARD
]

cmd_valid_states = {
  0  : all_states,
  2  : [CARD_STATE_READY],
  3  : [CARD_STATE_IDENTIFICATION],
  6  : [CARD_STATE_TRAN],
  7  : [CARD_STATE_TRAN, CARD_STATE_STANDBY],
  8  : [CARD_STATE_IDLE],
  9  : [CARD_STATE_STANDBY],
  13 : all_states,
  17 : [CARD_STATE_TRAN],
  55 : all_states,
}

acmd_valid_states = {
   6 : [CARD_STATE_TRAN],
  13 : [CARD_STATE_TRAN],
  41 : [CARD_STATE_IDLE, CARD_STATE_READY],
  51 : [CARD_STATE_TRAN],
}

cmd_target_states = {
  0  : CARD_STATE_IDLE,
  2  : CARD_STATE_IDENTIFICATION,
  3  : CARD_STATE_STANDBY,
  6  : None,
  7  : CARD_STATE_TRAN,
  8  : CARD_STATE_IDLE,
  9  : CARD_STATE_STANDBY,
  13 : None,
  17 : None,
  55 : None,
}

acmd_target_states = {
   6 : None,
  13 : None,
  41 : CARD_STATE_READY,
  51 : None,
}

def target_states(is_acmd, cmd_idx):
  states = acmd_target_states if is_acmd else cmd_target_states
  return states[cmd_idx]

def bitplace(v, bitpos, width):
  return (v & ((1 << width) - 1)) << bitpos

def byteswap32(v):
  b0 = v & 0xff
  b1 = (v >> 8) & 0xff
  b2 = (v >> 16) & 0xff
  b3 = (v >> 24) & 0xff
  return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3


def gen_cmdtm(is_acmd, cmd_idx):
  dbase = sd_acommands if is_acmd else sd_commands
  resp_type, crc, direction, is_data, multiblock = dbase[cmd_idx]
  resp_type = cmdtm_resp_types[resp_type]
  return \
      bitplace(direction ,  4, 1) \
    | bitplace(multiblock,  5, 1) \
    | bitplace(resp_type , 16, 2) \
    | bitplace(crc       , 19, 1) \
    | bitplace(is_data   , 21, 1) \
    | bitplace(cmd_idx   , 24, 6) \


def cmd_resp_type(is_acmd, cmd_idx):
  dbase = sd_acommands if is_acmd else sd_commands
  resp_type, crc, direction, is_data, multiblock = dbase[cmd_idx]
  return resp_type


class TargetStatus:
  def __init__(self, attached=False, halted=False):
    self.attached = attached
    self.halted = halted


class Target:
  def __init__(self, s):
    self.__status = TargetStatus()
    self.__s = s
    self.__debug_tty = True

  def write(self, data):
    data = (data + '\r\n').encode('utf-8')
    if self.__debug_tty:
      print(f'tty_write:{data}')
    self.__s.write(data)

  def wait_cursor(self):
    line = ''
    last_lines = []
    while True:
      c = self.__s.read().decode('utf-8')
      line += c
      if '>' in line:
        break
      if '\r\n' in line:
        last_lines.append(line[:-2])
        line = ''
      if not c:
        break
      if c == '>':
        break
    if not last_lines[-1]:
      last_lines = last_lines[:-1]
    if self.__debug_tty:
      print(last_lines)
    return last_lines[-1]

  def to_cursor(self):
    self.write('')
    self.wait_cursor()

  def init(self):
    self.write('init')
    self.wait_cursor()
    self.update_status()

  def update_status(self):
    while True:
      self.write('status')
      status = self.wait_cursor()
      print(status)
      if status.strip():
        break

    status = status.split(':')[1]
    self.__status.attached = 'not attached' not in status
    if self.__status.attached:
      self.__status.halted = 'halted' in status

  def get_status(self):
    return self.__status

  def halt(self):
    self.write('halt')
    self.wait_cursor()
    self.update_status()

  def reset(self):
    self.write('srst')
    self.wait_cursor()
    self.update_status()

  def mem_read32(self, address):
    self.write(f'mrw 0x{address:08x}')
    lines = self.wait_cursor()
    value = int(lines, base=0)
    return value

  def mem_write32(self, address, value):
    self.write(f'mww 0x{address:08x} 0x{value:08x}')
    self.wait_cursor()


SDHC_BASE = 0x3f300000
SDHC_ARG2      = SDHC_BASE + 0x00
SDHC_BLKSZCNT  = SDHC_BASE + 0x04
SDHC_ARG1      = SDHC_BASE + 0x08
SDHC_CMDTM     = SDHC_BASE + 0x0c
SDHC_RESP0     = SDHC_BASE + 0x10
SDHC_RESP1     = SDHC_BASE + 0x14
SDHC_RESP2     = SDHC_BASE + 0x18
SDHC_RESP3     = SDHC_BASE + 0x1c
SDHC_DATA      = SDHC_BASE + 0x20
SDHC_STATUS    = SDHC_BASE + 0x24
SDHC_CONTROL0  = SDHC_BASE + 0x28
SDHC_CONTROL1  = SDHC_BASE + 0x2c
SDHC_INTERRUPT = SDHC_BASE + 0x30
SDHC_INT_MASK  = SDHC_BASE + 0x34
SDHC_INT_EN    = SDHC_BASE + 0x38
SDHC_CONTROL2  = SDHC_BASE + 0x3c
SDHC_CAPS_0    = SDHC_BASE + 0x40
SDHC_CAPS_1    = SDHC_BASE + 0x44

SDHC_CONTROL1_INT_CLK_ENA       = 0
SDHC_CONTROL1_INT_CLK_STABLE    = 1
SDHC_CONTROL1_SD_CLK_ENA        = 2
SDHC_CONTROL1_PLL_ENA           = 3
SDHC_CONTROL1_CLK_GENERATOR_SEL = 4
SDHC_CONTROL1_SDCLK_DIV_HI_BITS = 6
SDHC_CONTROL1_SDCLK_DIV_LO_BITS = 8

class cmd_result:
  def __init__(self, status, data, resp0, resp1, resp2, resp3):
    self.status = status
    self.data = data
    self.resp0 = resp0
    self.resp1 = resp1
    self.resp2 = resp2
    self.resp3 = resp3

class SDHC:
  def __init__(self, t):
    self.__t = t

  def arg2_read(self):
    return self.__t.mem_read32(SDHC_ARG2)

  def blkszcnt_read(self):
    return self.__t.mem_read32(SDHC_BLKSZCNT)

  def blkszcnt_write(self, v):
    self.__t.mem_write32(SDHC_BLKSZCNT, v)

  def arg1_read(self):
    return self.__t.mem_read32(SDHC_ARG1)

  def arg1_write(self, v):
    return self.__t.mem_write32(SDHC_ARG1, v)

  def cmdtm_read(self):
    return self.__t.mem_read32(SDHC_CMDTM)

  def cmdtm_write(self, v):
    return self.__t.mem_write32(SDHC_CMDTM, v)

  def resp0_read(self):
    return self.__t.mem_read32(SDHC_RESP0)

  def resp1_read(self):
    return self.__t.mem_read32(SDHC_RESP1)

  def resp2_read(self):
    return self.__t.mem_read32(SDHC_RESP2)

  def resp3_read(self):
    return self.__t.mem_read32(SDHC_RESP3)

  def data_read(self):
    return self.__t.mem_read32(SDHC_DATA)

  def status_read(self):
    return self.__t.mem_read32(SDHC_STATUS)

  def control0_write(self, v):
    self.__t.mem_write32(SDHC_CONTROL0, v)

  def control0_read(self):
    return self.__t.mem_read32(SDHC_CONTROL0)

  def control1_read(self):
    return self.__t.mem_read32(SDHC_CONTROL1)

  def control1_write(self, v):
    self.__t.mem_write32(SDHC_CONTROL1, v)

  def interrupt_read(self):
    return self.__t.mem_read32(SDHC_INTERRUPT)

  def interrupt_write(self, v):
    self.__t.mem_write32(SDHC_INTERRUPT, v)

  def int_mask_read(self):
    return self.__t.mem_read32(SDHC_INT_MASK)

  def int_mask_write(self, v):
    self.__t.mem_write32(SDHC_INT_MASK, v)

  def int_en_read(self):
    return self.__t.mem_read32(SDHC_INT_EN)

  def int_en_write(self, v):
    self.__t.mem_write32(SDHC_INT_EN, v)

  def control2_read(self):
    return self.__t.mem_read32(SDHC_CONTROL2)

  def control2_write(self, v):
    self.__t.mem_write32(SDHC_CONTROL2, v)

  def caps_0_read(self):
    return self.__t.mem_read32(SDHC_CAPS_0)

  def caps_1_read(self):
    return self.__t.mem_read32(SDHC_CAPS_1)

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
      div10 = 256
    else:
      div10 = 4

    div_hi = (div10 >> 8) & 3
    div_lo = div10 & 0xff
    timeout = 0x0b << 16

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
    v |= 1 << SDHC_CONTROL1_SD_CLK_ENA
    self.control1_write(v)
    v = self.control1_read()

  def sd_clock_stop(self):
    v = self.control1_read()
    v &= ~(1 << SDHC_CONTROL1_SD_CLK_ENA)
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
    print('reset cmd done')

  def sw_reset_all(self):
    v = self.control1_read()
    v |= 1<<24
    self.control1_write(v)
    while True:
      v = self.control1_read()
      if ((v & (1<<24)) == 0):
        break
    print('emmc sw reset all done')

  def internal_clock_stop(self):
    v = self.control1_read()
    v &= ~(1<<SDHC_CONTROL1_INT_CLK_ENA)
    self.control1_write(v)
    while True:
      v = self.control1_read()
      if (v & (1<<SDHC_CONTROL1_INT_CLK_STABLE)) == 0:
        break

  def internal_clock_start(self):
    v = self.control1_read()
    v |= 1 << SDHC_CONTROL1_INT_CLK_ENA
    self.control1_write(v)
    while True:
      v = self.control1_read()
      if (v & (1<<SDHC_CONTROL1_INT_CLK_STABLE)):
        break

  def set_bus_width4(self):
    v = self.control0_read()
    CONTROL0_DWIDTH4_BIT    = 1<<1
    v |= CONTROL0_DWIDTH4_BIT
    self.control0_write(v)
    print('data bus width set to 4')

  def set_high_speed(self):
    v = self.control0_read()
    CONTROL0_HIGH_SPEED_BIT = 1<<2
    v |= CONTROL0_HIGH_SPEED_BIT
    self.control0_write(v)
    print('High speed bit is set')

  def reset(self):
    # Software reset.
    self.sw_reset_all()
    # Restart internal SDHC clock
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

  def cmd_wait_intr_done(self):
    INTR_CMD_DONE  = 1 << 0
    INTR_DATA_DONE = 1 << 1
    INTR_ERR       = 1 << 15

    while True:
      intr = self.interrupt_read()
      if intr & INTR_CMD_DONE:
        break
      if intr & INTR_ERR:
        break
    self.interrupt_write(intr)


  def cmd(self, is_acmd, cmd_idx, blksize, arg1, read_resp=False, data_size=0):
    cmd_type = 'ACMD' if is_acmd else 'CMD'
    print(f'Running {cmd_type}{cmd_idx}')
    while True:
      status = self.status_read()
      if (status & 1) == 0:
        break

    self.blkszcnt_write(blksize)

    self.arg1_write(arg1)

    intr = self.interrupt_read()
    if intr:
      print(f'clearing stale interrupt {intr:08x}')
      self.interrupt_write(intr)

    self.cmdtm_write(gen_cmdtm(is_acmd, cmd_idx))
    status = self.status_read()
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

    for i in range(num_words32):
      done = float(i) / num_words32
      full = int(10 * done)
      cursor = '=' * full + '>'

      print(f'\r{cursor}{i}/{num_words32}', end='')
      while True:
        status = self.status_read()
        if status & (1<<9):
          break
      data += struct.pack('I', self.data_read())
    if num_words32:
      print()

    print('Finished CMD{}, resp {:08x} {:08x} {:08x} {:08x}, data:{}'.format(
      cmd_idx,
      resp0 or 0,
      resp1 or 0,
      resp2 or 0,
      resp3 or 0,
      data))
    return cmd_result(status, data, resp0, resp1, resp2, resp3)


def assert_state(is_acmd, cmd_idx, state):
  if not is_acmd and cmd_idx == 0:
    return

  states_map = acmd_valid_states if is_acmd else cmd_valid_states
  good_states = states_map[cmd_idx]
  if state not in good_states:
    msg = '{}CMD{} called not from one of expected states.\n'.format(
      'A' if is_acmd else '',
      cmd_idx)
    good_states_list = ', '.join([card_states[i] for i in good_states])
    msg += f'Allowed states are: {good_states_list}.\n'
    msg += f'Current state: {card_states[state]}\n'
    raise Exception(msg)


def parse_r6(r: cmd_result):
  rca   = (r.resp0 >> 16) & 0xffff
  old_state = (r.resp0 >> 9) & 0xf

  for i in range(0, 9):
    if r.resp0 & (1<<i):
      print(f'bit {i}')
  if (r.resp0 >> 13) & 1:
    print('bit 19')
  if (r.resp0 >> 14) & 1:
    print('bit 22')
  if (r.resp0 >> 15) & 1:
    print('bit 23')
  print(f'prev_state={card_states[old_state]}, rca={rca}')
  return rca, old_state


class SD:
  def __init__(self, sdhc: SDHC):
    self.__sdhc = sdhc
    self.__state = CARD_STATE_UNKNOWN


  def do_cmd(self, is_acmd, cmd_idx, blksize, arg1, read_resp, data_size):
    resp_type = cmd_resp_type(is_acmd, cmd_idx)
    assert_state(is_acmd, cmd_idx, self.__state)
    ret = self.__sdhc.cmd(is_acmd, cmd_idx, blksize, arg1, read_resp,
      data_size)

    if resp_type == RESP_TYPE_R6:
      rca, old_state = parse_r6(ret)
      if old_state != self.__state:
        raise Exception('Prev state incorrect. assumed:{}, was:{}'.format(
          card_states[self.__state], card_states[state]))

    new_state = target_states(is_acmd, cmd_idx)
    if new_state is not None and new_state != self.__state:
      print('State change \'{}\'->\'{}\''.format(card_states[self.__state],
        card_states[new_state]))
      self.__state = new_state
    else:
      print('State is same \'{}\''.format(card_states[self.__state]))

    return ret

  def cmd0(self):
    # GO_IDLE_STATE 
    ret = self.do_cmd(False, 0, 0, 0, read_resp=False, data_size=0)
    self.__state = CARD_STATE_IDLE
    return ret

  def cmd2(self):
    # ALL_SEND_CID
    ret = self.do_cmd(False, 2, 0, 0, read_resp=True, data_size=0)
    self.__state = CARD_STATE_IDENTIFICATION
    print(f'CID: {ret.resp0:08x}{ret.resp1:08x}{ret.resp2:08x}{ret.resp3:08x}')
    print(f'CID: {ret.resp3:08x}{ret.resp2:08x}{ret.resp1:08x}{ret.resp0:08x}')
    cid_raw = (ret.resp3 << 96)|(ret.resp2 << 64)|(ret.resp1 << 32)|ret.resp0
    # Last byte of CID is not reported, so result in RESP0..3 registers is
    # shifted by 1 bite. We return CID as if CRC byte is removed to be able
    # to parse that
    cid_raw = cid_raw << 8
    return cid_raw

  def cmd3(self):
    # SEND_RELATIVE_ADDR
    ret = self.do_cmd(False, 3, 0, 0, read_resp=True, data_size=0)
    rca, old_state = parse_r6(ret)
    return rca

  def cmd6(self, mode, power_limit, drive_strength, cmd_sys, access_mode):
    # SWITCH FUNCTION
    arg =  (access_mode    & 0xf) << 0
    arg |= (cmd_sys        & 0xf) << 4
    arg |= (drive_strength & 0xf) << 8
    arg |= (power_limit    & 0xf) << 12
    arg |= 0xff << 16
    arg |= (mode & 1) << 31

    blksizecount = (1<<16) | 64
    ret = self.do_cmd(False, 6, blksizecount, arg, read_resp=True, data_size=64)
    data = [i for i in ret.data]
    return data

  def cmd7(self, rca):
    # SELECT_CARD
    arg = (rca << 16) & 0xffffffff
    ret = self.do_cmd(False, 7, 0, arg, read_resp=True, data_size=0)
    self.__state = CARD_STATE_TRAN
    return ret

  def cmd8(self):
    # SEND_IF_COND
    # Response type is R7, so no state information
    # Possible only in IDLE state and returns to IDLE

    check_pattern = 0xdd
    vhs = 1
    arg = (1 << 8) | check_pattern
    ret = self.do_cmd(False, 8, 0, arg, read_resp=True, data_size=0)
    if ret.resp0 & 0xff != check_pattern:
      print('CMD8 failed, pattern mismatch')
      return False
    return True

  def cmd9(self, rca):
    # SEND_CSD
    arg = (rca << 16) & 0xffffffff
    ret = self.do_cmd(False, 9, 0, arg, read_resp=True, data_size=0)
    print(f'CID: {ret.resp0:08x}{ret.resp1:08x}{ret.resp2:08x}{ret.resp3:08x}')
    print(f'CID: {ret.resp3:08x}{ret.resp2:08x}{ret.resp1:08x}{ret.resp0:08x}')
    csd_raw = (ret.resp3 << 96)|(ret.resp2 << 64)|(ret.resp1 << 32)|ret.resp0
    return csd_raw << 8

  def cmd13(self, rca):
    # SEND_STATUS
    arg = (0<<15) | (rca << 16)
    ret = self.do_cmd(False, 13, 0, arg, read_resp=True, data_size=0)
    state = (ret.resp0 >> 9) & 0xf
    print(f'state={card_states[state]}')
    return state

  def cmd55(self, rca):
    arg = (rca << 16) & 0xffffffff
    return self.do_cmd(False, 55, 0, arg, read_resp=True, data_size=0)

  def cmd17(self, block_idx):
    # READ_SINGLE_BLOCK
    arg = block_idx
    blksize = (1<<16) | 512
    return self.do_cmd(False, 17, blksize, arg, read_resp=True, data_size=512)

  def acmd6(self, rca, bus_width_4):
    # SET_BUS_WIDTH
    self.cmd55(rca)
    arg = 2 if bus_width_4 else 0
    return self.do_cmd(True, 6, 0, arg, read_resp=True, data_size=0)

  def acmd41(self, rca, arg):
    self.cmd55(rca)
    return self.do_cmd(True, 41, 0, arg, read_resp=True, data_size=0)

  def acmd51(self, rca):
    # SEND_SCR
    self.cmd55(rca)
    arg = 0
    blksize = (1<<16) | 8
    ret = self.do_cmd(True, 51, blksize, arg, read_resp=True, data_size=8)
    return struct.unpack('>Q', ret.data)[0]


class Soc:
  def __init__(self, t: Target):
    self.__t = t
    self.sdhc = SDHC(self.__t)
    self.sd = SD(self.sdhc)

  def reset(self):
    self.__t.reset()


def parse_scr(v):
  print(f'SCR: 0x{v:016x}')
  scr_struct            = (v >> 60) & 0xf
  sd_spec               = (v >> 56) & 0xf
  data_stat_after_erase = (v >> 55) & 1
  sd_security           = (v >> 52) & 7
  sd_bus_widths         = (v >> 48) & 0xf
  sd_spec3              = (v >> 47) & 1
  ex_security           = (v >> 43) & 0xf
  sd_spec4              = (v >> 42) & 1
  sd_specx              = (v >> 38) & 0xf
  cmd_support           = (v >> 32) & 0x1f
  ver = '1.0'
  if sd_spec == 1:
    ver = '1.1'
  elif sd_spec == 2:
    if sd_spec3 == 0:
      ver = '2.0'
    elif sd_spec3 == 1:
      if sd_spec4 == 0:
        ver = '3.0'
      else:
        ver = '4.xx'
      if sd_specx == 1:
        ver = '5.xx'
      elif sd_specx == 2:
        ver = '6.xx'
      elif sd_specx == 3:
        ver = '7.xx'
      elif sd_specx == 4:
        ver = '8.xx'
      elif sd_specx == 5:
        ver = '9.xx'
  print(f'scr_struct: {scr_struct}')
  print(f'sd_spec: {sd_spec}')
  print(f'sd_spec3: {sd_spec3}')
  print(f'sd_spec4: {sd_spec4}')
  print(f'sd_specx: {sd_specx}')
  print(f'sd_bus_widths: {sd_bus_widths:x}')
  print(f'ver: {ver}')


def parse_cid(cid):
  print(f'Parsing CID :{hex(cid)}')
  mid = cid >> 120
  oid_raw = (cid >> 104) & 0xffff
  pname_raw = (cid >> 64) & 0xffffffffff
  rev = (cid >> 56) & 0xff
  sn = (cid >> 24) & 0xffffffff
  date_raw = (cid >> 8) & 0xfff
  month = date_raw & 0xf
  year = ((date_raw >> 4) & 0xff) + 2000
  oid = struct.pack('>H', oid_raw).decode('utf-8')
  pname = struct.pack('>q', pname_raw)[3:].decode('utf-8')

  print(f'MID: {hex(mid)} OID:{oid} Product name: {pname}')
  print(f'Rev: {rev}, SN:{sn:08x}, Manufacture date: {month:02} {year}')


def parse_csd(v):
  print(f'CSD {v:032x}')
  csd_ver = (v >> 126) & 3
  reserved = (v >> 120) & 0x3f
  data_read_access_time = (v >> 112) & 0xff
  data_read_access_time_clk = (v >> 104) & 0xff
  max_data_rate = (v >> 96) & 0xff
  card_command_classes = (v >> 84) & 0xfff
  max_read_block = (v >> 80) & 0xf
  part_read_allowed = (v >> 79) & 1
  write_block_misalignment = (v >> 78) & 1
  read_block_misalignment = (v >> 77) & 1
  dsr = (v >> 76) & 1
  reserved = (v >> 70) & 0x3f
  device_size = (v >> 48) & ((1<<22) - 1)
  reserved = (v >> 47) & 1
  erase_single_block_en = (v >> 46) & 1
  erase_sector_size = (v >> 39) & 1
  write_speed_factor = (v >> 26) & 7
  max_wr_block_len = (v >> 22) & 0xf
  print('CSD version: {}, data read access time {} {} CLK'.format(
    csd_ver, data_read_access_time, data_read_access_time_clk))
  print(f'Max data rate: {max_data_rate}, CCC: {card_command_classes:x}')
  print('Max read block: {} ({} bytes), part: {}'.format(max_read_block,
    1<<max_read_block,
    'allowed' if part_read_allowed else 'disallowed'))
  print(f'Write block misalignment: {write_block_misalignment}')
  print(f'Read block misalignment: {read_block_misalignment}')
  print(f'DSR implemented: {dsr}, device size: {device_size}')
  print(f'Erase single block: {erase_single_block_en}, sect sz: {erase_sector_size}')
  print(f'Write speed factor: {write_speed_factor}')
  print(f'Max write data block length: {max_wr_block_len} ({1<<max_wr_block_len} bytes)')


def parse_cmd6_check_data(check_data):
  v = 0
  for i, value in enumerate(check_data):
    v = (v << 8) | value

  print(f'{v:x}')
  version = (v >> 368) & 0xff

  fn_sel = [(v >> (376 + i * 4)) & 0xf for i in range(6)]
  fn_supp = [(v >> (400 + i * 16)) & 0xffff for i in range(6)]
  max_power = (v >> 496) & 0xffff

  print(f'version: {version}')
  string_sel  = [f'g{i+1}:{v:x}' for i, v in enumerate(fn_sel)]
  string_supp = [f'g{i+1}:{v:04x}' for i, v in enumerate(fn_supp)]
  
  print(f'Selected functions: {string_sel}')
  print(f'Supported functions: {string_supp}')
  print(f'Max power: {max_power:04x}')
  return fn_sel, fn_supp


def target_attach_and_halt(ttydev, baudrate):
  s = serial.Serial(ttydev, baudrate, timeout=100)
  print(f"Opened {ttydev} with baud rate {baudrate}")
  t = Target(s)
  t.update_status()
  status = t.get_status()
  if not status.attached:
    t.init()
    status = t.get_status()
  if not status.halted:
    t.halt()
    status = t.get_status()

  soc = Soc(t)
  return soc


def sd_init_ident_mode(sd):
  # Physical Layer Simplified Specification Version 9.00
  # State machine to go through identification mode states to data transfer
  # mode STANDBY state

  # CMD0 brings card to IDLE
  sd.cmd0()
  # CMD8 is mandatory part of init process
  sd.cmd8()
  # ACMD41 does IDLE->READY state transition
  ret = sd.acmd41(rca=0, arg=0)
  arg = (ret.resp0 & 0x00ff8000) | (1<<30)
  while True:
    ret = sd.acmd41(rca=0, arg=0x40ff8000)
    if ret.resp0 & (1<<31):
      break

  # CMD2 SEND_ALL_CID to receive card's CID register contents
  # CMD2 does READY->IDENT state transition
  cid = sd.cmd2()
  parse_cid(cid)
  # CMD3 SEND_RELATIVE_ADDR
  # CMD3 does IDENT->STANDBY state transition
  rca = sd.cmd3()
  # CMD13 SEND_STATUS to ensure status is STANDBY
  state = sd.cmd13(rca)
  if state != CARD_STATE_STANDBY:
    raise Exception('Failed to put card in \'standby\' state')
  return rca

def sd_set_high_speed(sd, sdhc):
  check_data = sd.cmd6(CMD6_CHECK_FUNCTION, 0xf, 0xf, 0xf, 0xf)
  fn_sel, fn_sup = parse_cmd6_check_data(check_data)
  access_mode_sup_bits = fn_sup[0]
  ACCESS_MODE_SUPP_SDR12  = 0
  ACCESS_MODE_SUPP_SDR25  = 1
  ACCESS_MODE_SUPP_SDR50  = 2
  ACCESS_MODE_SUPP_SDR104 = 3
  ACCESS_MODE_SUPP_DDR50  = 4
  sdr25_ok = False
  if access_mode_sup_bits & (1<<ACCESS_MODE_SUPP_SDR25):
    sdr25_ok = True

  print('sdr25: {}'.format('yes' if sdr25_ok else 'no'))
  if sdr25_ok:
    sd.cmd6(CMD6_SWITCH_FUNCTION, 0xf, 0xf, 0xf, ACCESS_MODE_SUPP_SDR25)
    fn_sel, fn_sup = parse_cmd6_check_data(check_data)
  check_data = sd.cmd6(CMD6_CHECK_FUNCTION, 0, 0, 0, 1)
  fn_sel, fn_sup = parse_cmd6_check_data(check_data)
  sdhc.set_high_speed()


def sd_init_data_transfer_mode(sd, sdhc, rca):
  # Guide card through data transfer mode states into 'tran' state
  # CMD9 SEND_CSD to get and parse CSD registers
  csd = sd.cmd9(rca)
  parse_csd(csd)
  # CMD7 SELECT_CARD to select card, making it to 'tran' state
  sd.cmd7(rca)
  # ACMD51 SEND_SCR
  scr = sd.acmd51(rca)
  parse_scr(scr)
  sd.acmd6(rca, bus_width_4=True)
  sdhc.set_bus_width4()
  sd_set_high_speed(sd, sdhc)


def action_sdhc(soc):
  sdhc = soc.sdhc
  sd = soc.sd
  sdhc.reset()
  print('SDHC internal and SD clocks running')
  rca = sd_init_ident_mode(sd)
  sd_init_data_transfer_mode(sd, sdhc, rca)
  time.sleep(0.5)
  r = sd.cmd17(0)
  with open('/tmp/bin', 'wb') as f:
    f.write(r.data)

def do_main(action):
  soc = target_attach_and_halt('/dev/ttyACM0', 115200 * 8)

  if action == 'rst':
    soc.reset()
  else:
    action_sdhc(soc)
  sys.exit(0)


def main(action):
  try:
    do_main(action)
  except serial.SerialException as e:
    print(f"Serial error: {e}")
  except KeyboardInterrupt:
    print("Exiting program.")

if __name__ == "__main__":
  action = ''
  if len(sys.argv) > 1:
    action = sys.argv[1]
  main(action)
