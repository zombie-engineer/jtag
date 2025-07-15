import serial
import sys
import struct
import time
import logging
import argparse
from bcm_sdhci import SDHCI
from bcm_sdhost import SDHOST
import bcm_gpio
import bcm_clock_manager
import sdhc

CMD6_CHECK_FUNCTION  = 0
CMD6_SWITCH_FUNCTION = 1

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

#def measure_vpu_clock(t):
#  div = 1000
#  crystal = 19200000


def target_states(is_acmd, cmd_idx):
  states = acmd_target_states if is_acmd else cmd_target_states
  return states[cmd_idx]


def cmd_resp_type(is_acmd, cmd_idx):
  dbase = sdhc.sd_acommands if is_acmd else sdhc.sd_commands
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
    self.debug_tty_read = True
    self.debug_tty_write = True

  def write(self, data):
    data = (data + '\r\n').encode('utf-8')
    if self.debug_tty_write:
      logging.debug(f'tty_write:{data}')
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
    if self.debug_tty_read:
      logging.debug(f'wait_cursor: last_lines: {last_lines}')
    return last_lines

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
      status = status[-1]
      if status.strip():
        break

    status = status.split(':')[1]
    self.__status.attached = 'not attached' not in status
    if self.__status.attached:
      self.__status.halted = 'halted' in status

  def get_status(self):
    return self.__status

  def dump_regs(self):
    self.write('dumpregs')
    lines = self.wait_cursor()
    regs = lines[lines.index('dumpregs') + 1:lines.index('done')]
    for regentry in regs:
      core, name, value = regentry.split(',')
      logging.info(f'core {core}, {name}: {value}')

  def write_reg(self, regname, value):
    self.write(f'rw {regname} {value}')
    self.wait_cursor()

  def read_reg(self, regname):
    self.write(f'rr {regname}')
    lines = self.wait_cursor()
    return lines

  def resume(self):
    self.write('resume')
    self.wait_cursor()
    self.update_status()

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
    lines = lines[-1]
    value = int(lines, base=0)
    return value

  def mem_write32(self, address, value):
    self.write(f'mww 0x{address:08x} 0x{value:08x}')
    self.wait_cursor()

  def mem_read(self, address, size):
    self.write(f'mr 0x{address:08x} {size}')
    lines = self.wait_cursor()
    logging.debug(lines)


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


def parse_r6(r: sdhc.cmd_result):
  print(f'R6 response is: {r.resp0:08x}')
  rca   = (r.resp0 >> 16) & 0xffff
  old_state = (r.resp0 >> 9) & 0xf

  for i in range(0, 9):
    if r.resp0 & (1<<i):
      logging.debug(f'bit {i}')
  if (r.resp0 >> 13) & 1:
    logging.debug('bit 19')
  if (r.resp0 >> 14) & 1:
    logging.debug('bit 22')
  if (r.resp0 >> 15) & 1:
    logging.debug('bit 23')
  logging.info(f'prev_state={card_states[old_state]}, rca={rca}')
  return rca, old_state


class SD:
  def __init__(self, sdhc: SDHCI, sdhost: SDHOST, use_sdhost):
    self.__sdhc = sdhc
    self.__sdhost = sdhost
    self.__state = CARD_STATE_UNKNOWN
    if use_sdhost:
      self.__sdctrl = self.__sdhost
      self.__sdhc.stop()
      self.__sdhost.init_gpio()
    else:
      self.__sdctrl = self.__sdhc
      self.__sdhost.stop()

  def __set_state(self, state):
    old_state = card_states[self.__state]
    new_state = card_states[state]
    print(f"SD::state: {old_state} -> {new_state}")
    self.__state = state

  def reset(self):
    self.__sdctrl.reset()

  def do_cmd(self, is_acmd, cmd_idx, blksize, arg1, read_resp, data_size):
    resp_type = cmd_resp_type(is_acmd, cmd_idx)
    assert_state(is_acmd, cmd_idx, self.__state)
    ret = self.__sdctrl.cmd(is_acmd, cmd_idx, blksize, arg1, read_resp,
      data_size)
    if resp_type == sdhc.RESP_TYPE_R6:
      rca, old_state = parse_r6(ret)
      if old_state != self.__state:
        raise Exception('Prev state incorrect. assumed:{}, was:{}'.format(
          card_states[self.__state], card_states[state]))

    new_state = target_states(is_acmd, cmd_idx)
    if new_state is not None and new_state != self.__state:
      logging.debug('State change \'{}\'->\'{}\''.format(card_states[self.__state],
        card_states[new_state]))
      self.__set_state(new_state)
    else:
      logging.debug('State is same \'{}\''.format(card_states[self.__state]))

    return ret

  def cmd0(self):
    # GO_IDLE_STATE 
    ret = self.do_cmd(False, 0, 0, 0, read_resp=False, data_size=0)
    self.__set_state(CARD_STATE_IDLE)
    return ret

  def cmd2(self):
    # ALL_SEND_CID
    ret = self.do_cmd(False, 2, 0, 0, read_resp=True, data_size=0)
    self.__set_state(CARD_STATE_IDENTIFICATION)
    logging.debug(f'CID: {ret.resp0:08x}{ret.resp1:08x}{ret.resp2:08x}{ret.resp3:08x}')
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

    blksize = 64
    ret = self.do_cmd(False, 6, blksize, arg, read_resp=True, data_size=64)
    data = [i for i in ret.data]
    return data

  def cmd7(self, rca):
    # SELECT_CARD
    arg = (rca << 16) & 0xffffffff
    ret = self.do_cmd(False, 7, 0, arg, read_resp=True, data_size=0)
    self.__set_state(CARD_STATE_TRAN)
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
      logging.warning('CMD8 failed, pattern mismatch')
      return False
    return True

  def cmd9(self, rca):
    # SEND_CSD
    arg = (rca << 16) & 0xffffffff
    ret = self.do_cmd(False, 9, 0, arg, read_resp=True, data_size=0)
    logging.info(f'CSD: {ret.resp0:08x}{ret.resp1:08x}{ret.resp2:08x}{ret.resp3:08x}')
    csd_raw = (ret.resp3 << 96)|(ret.resp2 << 64)|(ret.resp1 << 32)|ret.resp0
    if self.__sdctrl == self.__sdhc:
      csd_raw <<= 8
    return csd_raw

  def cmd13(self, rca):
    # SEND_STATUS
    arg = (0<<15) | ((rca << 16) & 0xffff0000)
    ret = self.do_cmd(False, 13, 0, arg, read_resp=True, data_size=0)
    state = (ret.resp0 >> 9) & 0xf
    logging.debug(f'state={card_states[state]}')
    return state

  def cmd55(self, rca):
    arg = (rca << 16) & 0xffffffff
    return self.do_cmd(False, 55, 0, arg, read_resp=True, data_size=0)

  def cmd17(self, block_idx):
    # READ_SINGLE_BLOCK
    arg = block_idx
    blksize = 512
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
    # For SDHCI blksize = (1<<16) | 8
    blksize = 8
    ret = self.do_cmd(True, 51, blksize, arg, read_resp=True, data_size=8)
    return struct.unpack('>Q', ret.data)[0]

  def set_bus_width4(self):
    self.__sdctrl.set_bus_width4()

  def set_high_speed(self):
    self.__sdctrl.set_high_speed()


class Soc:
  def __init__(self, t: Target, logger):
    self.__t = t
    self.clockman = bcm_clock_manager.BCMClockManager(t, logger)
    self.gpio = bcm_gpio.BCM_GPIO(t)
    self.sd = SD(
      SDHCI(self.__t, logger),
      SDHOST(self.gpio, self.__t, logger), use_sdhost=True)

  def read_mem32(self, address):
    return self.__t.mem_read32(address)

  def write_mem32(self, address, v):
    self.__t.mem_write32(address, v)

  def reset(self):
    self.__t.reset()

  def resume(self):
    self.__t.write_reg('pc', 0x80000 + 4)
    logging.debug(self.__t.read_reg('pc'))
    self.__t.resume()


def parse_scr(v):
  logging.info(f'SCR: 0x{v:016x}')
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
  logging.debug(f'scr_struct: {scr_struct}')
  logging.debug(f'sd_spec: {sd_spec}')
  logging.debug(f'sd_spec3: {sd_spec3}')
  logging.debug(f'sd_spec4: {sd_spec4}')
  logging.debug(f'sd_specx: {sd_specx}')
  logging.debug(f'sd_bus_widths: {sd_bus_widths:x}')
  logging.debug(f'ver: {ver}')


def parse_cid(cid):
  logging.debug(f'Parsing CID :{hex(cid)}')
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
  logging.info(f'CID: MID: {hex(mid)} OID:{oid} Product name: {pname}')
  logging.info(f'CID: rev: {rev}, SN:{sn:08x}, Manufacture date: {month:02} {year}')


def parse_csd(v):
  logging.info(f'CSD {v:032x}')
  csd_version  = (v >> 126) & 3
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
  logging.info('CSD version: {}, data read access time {} {} CLK'.format(
    csd_version, data_read_access_time, data_read_access_time_clk))
  logging.info(f'Max data rate: {max_data_rate}, CCC: {card_command_classes:x}')
  logging.info('Max read block: {} ({} bytes), part: {}'.format(max_read_block,
    1<<max_read_block,
    'allowed' if part_read_allowed else 'disallowed'))
  logging.info(f'Write block misalignment: {write_block_misalignment}')
  logging.info(f'Read block misalignment: {read_block_misalignment}')
  logging.info(f'DSR implemented: {dsr}, device size: {device_size}')
  logging.info(f'Erase single block: {erase_single_block_en}, sect sz: {erase_sector_size}')
  logging.info(f'Write speed factor: {write_speed_factor}')
  logging.info(f'Max write data block length: {max_wr_block_len} ({1<<max_wr_block_len} bytes)')


def parse_cmd6_check_data(check_data):
  v = 0
  for i, value in enumerate(check_data):
    v = (v << 8) | value

  version = (v >> 368) & 0xff

  fn_sel = [(v >> (376 + i * 4)) & 0xf for i in range(6)]
  fn_supp = [(v >> (400 + i * 16)) & 0xffff for i in range(6)]
  max_power = (v >> 496) & 0xffff

  logging.info(f'version: {version}')
  string_sel  = [f'g{i+1}:{v:x}' for i, v in enumerate(fn_sel)]
  string_supp = [f'g{i+1}:{v:04x}' for i, v in enumerate(fn_supp)]
  
  logging.info(f'Selected functions: {string_sel}')
  logging.info(f'Supported functions: {string_supp}')
  logging.info(f'Max power: {max_power:04x}')
  return fn_sel, fn_supp


def target_attach_and_halt(ttydev, baudrate):
  s = serial.Serial(ttydev, baudrate, timeout=100)
  logging.info(f"Opened {ttydev} with baud rate {baudrate}")
  t = Target(s)
  t.update_status()
  status = t.get_status()
  if not status.attached:
    t.init()
    status = t.get_status()
  if not status.halted:
    t.halt()
    status = t.get_status()
  t.dump_regs()
  t.mem_read(0x80000, 24)

  soc = Soc(t, logging)
  return soc


CARD_CAPACITY_SDHC = 'SDHC'
CARD_CAPACITY_SDSC = 'SDSC'
CARD_CAPACITY_SDXC = 'SDXC'
CARD_CAPACITY_SDUC = 'SDUC'

capacity_sizes = {
  CARD_CAPACITY_SDSC : (2, 2),
  CARD_CAPACITY_SDHC : (2, 32),
  CARD_CAPACITY_SDXC : (32, 2048),
  CARD_CAPACITY_SDUC : (2048, 128*1024),
}

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
  print(f'ACMD41: resp: {ret.resp0:08x}')
  capacity = CARD_CAPACITY_SDSC
  UHS_II_support = False
  SDUC_support = False
  while True:
    ret = sd.acmd41(rca=0, arg=0x40ff8000)
    print(f'ACMD41 repeat: arg=0x40ff8000, resp: {ret.resp0:08x}')
    if ret.resp0 & (1<<31):
      break

  if ret.resp0 & (1<<30):
    capacity = CARD_CAPACITY_SDHC
  if ret.resp0 & (1<<29):
    UHS_II_support = True
  if ret.resp0 & (1<<29):
    SDUC_support = True

  logging.info(f'Card capacity is {capacity} {capacity_sizes[capacity]}')
  logging.info(f'UHS-II {UHS_II_support}')

  # CMD2 SEND_ALL_CID to receive card's CID register contents
  # CMD2 does READY->IDENT state transition
  cid = sd.cmd2()
  parse_cid(cid)
  # CMD3 SEND_RELATIVE_ADDR
  # CMD3 does IDENT->STANDBY state transition
  rca = sd.cmd3()
  logging.info(f'rca is {rca}')
  # CMD13 SEND_STATUS to ensure status is STANDBY
  state = sd.cmd13(rca)
  if state != CARD_STATE_STANDBY:
    raise Exception('Failed to put card in \'standby\' state')
  return rca

def sd_set_high_speed(sd):
  logging.info(
    'Switching card from DEFAULT SPEED (25MHz)to HIGH SPEED (50MHz) mode')

  logging.info('Checking if card supports HIGH SPEED mode')

  check_data = sd.cmd6(CMD6_CHECK_FUNCTION, 0xf, 0xf, 0xf, 0xf)
  fn_sel, fn_sup = parse_cmd6_check_data(check_data)
  access_mode_sup_bits = fn_sup[0]
  ACCESS_MODE_SUPP_SDR12  = 0
  ACCESS_MODE_SUPP_SDR25  = 1
  ACCESS_MODE_SUPP_SDR50  = 2
  ACCESS_MODE_SUPP_SDR104 = 3
  ACCESS_MODE_SUPP_DDR50  = 4

  can_highspeed = access_mode_sup_bits & (1<<ACCESS_MODE_SUPP_SDR25)
  if not can_highspeed:
    logging.warning('Card does not support HIGH SPEED mode')
    return

  logging.info('Card supports HIGH SPEED mode, switching')
  sd.cmd6(CMD6_SWITCH_FUNCTION, 0xf, 0xf, 0xf, ACCESS_MODE_SUPP_SDR25)
  fn_sel, fn_sup = parse_cmd6_check_data(check_data)
  check_data = sd.cmd6(CMD6_CHECK_FUNCTION, 0, 0, 0, 1)
  fn_sel, fn_sup = parse_cmd6_check_data(check_data)
  sd.set_high_speed()
  logging.info('Check after high speed switch')
  check_data = sd.cmd6(CMD6_CHECK_FUNCTION, 0, 0, 0, 1)
  fn_sel, fn_sup = parse_cmd6_check_data(check_data)
  logging.info('Card successfully switched to HIGH SPEED mode')


def sd_init_data_transfer_mode(sd, rca):
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
  time.sleep(100 * 0.001 * 0.001)
  sd.set_bus_width4()
  sd_set_high_speed(sd)


def read_sector(sd):
  r = sd.cmd17(0)
  with open('/tmp/bin', 'wb') as f:
    f.write(r.data)


def action_sdhc(soc):
  sd = soc.sd
  sd.reset()

  logging.info('SDHC internal and SD clocks running')
  rca = sd_init_ident_mode(sd)
  sd_init_data_transfer_mode(sd, rca)
  read_sector(sd)


def do_main(action):
  soc = target_attach_and_halt('/dev/ttyACM0', 115200 * 8)
  if action == 'reset':
    logging.info('resetting')
    soc.reset()
  elif action == 'resume':
    logging.info('resuming')
    soc.resume()
  elif action == 'halt':
    pass
  elif action.startswith('r32'):
    addr = int(action[3:], 0)
    v = soc.read_mem32(addr)
    print(f'read_mem32 result: 0x{addr:08x}: 0x{v:08x}')
  else:
    action_sdhc(soc)
  sys.exit(0)


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument('-v', '--verbose',
    help='print verbose logs',
    action='store_const',
    dest='loglevel',
    const=logging.DEBUG,
    default=logging.INFO)

  parser.add_argument("action", nargs="?", default="default",
    help="Action to perform")

  args = parser.parse_args()
  print(args)
  logging.basicConfig(format='%(levelname)s:%(message)s', level=args.loglevel)
  try:
    do_main(args.action)
  except serial.SerialException as e:
    print(f"Serial error: {e}")
  except KeyboardInterrupt:
    print("Exiting program.")

if __name__ == "__main__":
  main()
