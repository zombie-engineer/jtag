import serial
import sys
import struct
import time
import logging
import argparse
import bcm_spi as spi
import bcm_gpio
import bcm_clock_manager
import sdhc
from bcm_sdhci import SDHCI
from bcm_sdhost import SDHOST
import ili9341
from elftools.elf.elffile import ELFFile
import ctypes

#def measure_vpu_clock(t):
#  div = 1000
#  crystal = 19200000

def get_symbol_address(elf_path: str, symbol_name: str) -> int | None:
  """
  Return the virtual address of a global variable/function symbol
  from an ELF file. Returns None if not found.
  """
  with open(elf_path, 'rb') as f:
    elf = ELFFile(f)
    # Iterate over all sections looking for symbol tables
    for section in elf.iter_sections():
      if not isinstance(section, type(elf.get_section_by_name('.symtab'))):
        continue
      if section['sh_entsize'] == 0:
        continue
      for symbol in section.iter_symbols():
        if symbol.name == symbol_name:
          return symbol.entry['st_value']
  return None

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
  def __init__(self, s: serial.Serial, logger, logger_tty):
    self.__status = TargetStatus()
    self.__s = s
    self.__log = logger
    self.__log_tty = logger_tty

  def write(self, data):
    data = (data + '\r\n').encode('utf-8')
    self.__log_tty.debug(f'tty_write:{data}')
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

    if self.__log_tty.getEffectiveLevel() == logging.DEBUG:
      self.__log_tty.debug(f'wait_cursor: received:')
      for l in last_lines:
        self.__log_tty.debug(f'  [ "{l}"' + ' ' * (32 - len(l)) + ']')

    if not last_lines[-1]:
      last_lines = last_lines[:-1]


    err = 0
    if last_lines[-1].startswith('done'):
      doneline = last_lines.pop()
      err = int(doneline.split()[1])
    return err, last_lines

  def to_cursor(self):
    self.write('')
    self.wait_cursor()

  def init(self):
    self.write('init')
    self.wait_cursor()
    self.update_status()

  def check_is_halted(self):
    self.write('?halted')
    err, lines = self.wait_cursor()
    self.__log.debug(err, lines)
    return err == 0

  def update_status(self):
    while True:
      self.write('status')
      err, status_lines = self.wait_cursor()
      status = status_lines[-1]
      if status.strip():
        break

    status = status.split(':')[1].strip()
    self.__log.debug(f'reported target status "{status}"')
    self.__status.attached = 'not attached' not in status
    if self.__status.attached:
      self.__status.halted = 'halted' in status

  def get_status(self):
    return self.__status

  def dump_regs(self):
    response = {}
    self.write('dumpregs')
    err, lines = self.wait_cursor()
    regs = lines[lines.index('dumpregs') + 1 :]
    for regentry in regs:
      core, name, value = regentry.split(',')
      name = name.strip()
      response.setdefault(int(core), {})[name] = int(value, 0)
      self.__log.debug(f'core {core}, {name}: {value}')
    return response

  def write_reg(self, regname, value):
    self.write(f'rw {regname} {value}')
    err, lines = self.wait_cursor()
    return err == 0

  def read_reg(self, regname):
    self.write(f'rr {regname}')
    err, lines = self.wait_cursor()
    value = int(lines[1], 0) if err == 0 else 0
    return err == 0, value

  def resume(self):
    self.write('resume')
    self.wait_cursor()
    self.update_status()

  def step(self):
    self.write('s')
    self.wait_cursor()

  def halt(self):
    self.write('halt')
    self.wait_cursor()
    self.update_status()

  def reset(self):
    self.write('srst')
    self.wait_cursor()
    self.update_status()

  def __mem_read(self, size, address, count):
    countstr = ''

    if count < 1 or count > 0xffff:
      self.__log.error("invalid argument 'count': {count}")
      return []
    if size not in [32, 64]:
      self.__log.error("invalid argument 'size': {size}")
      return []

    if count > 1:
      countstr = f'/{count}'

    cmd = f'r{size}{countstr} 0x{address:08x}'
    self.write(cmd)
    err, lines = self.wait_cursor()
    if err:
      self.__log.error(f'Mem read cmd failed, err:{err}, details:{lines}')
      return []

    if self.__log.getEffectiveLevel() == logging.DEBUG:
      self.__log.debug(f'__mem_read: result:')
      for l in lines:
        self.__log.debug(f'  {l}')
    assert len(lines)
    assert lines[0] == cmd
    lines.pop(0)
    return [int(i, base=0) for i in lines]

  def mem_read32(self, address, count=1):
    return self.__mem_read(32, address, count)

  def mem_read64(self, address, count=1):
    return self.__mem_read(64, address, count)

  def mem_write32(self, address, value):
    self.write(f'w32 0x{address:016x} 0x{value:08x}')
    err, lines = self.wait_cursor()
    if err:
      self.__log.error('Failed to write', lines)
    return err == 0

  def mem_write64(self, address, value):
    self.write(f'w64 0x{address:016x} 0x{value:016x}')
    err, lines = self.wait_cursor()
    if err:
      self.__log.error('Failed to write', lines)
    return err == 0

  def breakpoint(self, addr, kind, add=True, hardware=False):
    action = 'setting' if add else 'removing'
    self.__log.debug(
      f'{action} breakpoint, addr:0x{addr:016x}, kind:{kind}, hw:{hardware}')

    cmd = 'bp'
    if hardware:
      cmd += 'h'
    else:
      cmd += 's'
    if not add:
      cmd += 'd'

    self.write(f'{cmd} 0x{addr:016x}')
    err, lines = self.wait_cursor()
    if err:
      self.__log.error('Failed to write', lines)
      return False
    return True


class Soc:
  def __init__(self, t: Target, logger):
    self.__t = t
    self.clockman = bcm_clock_manager.BCMClockManager(t, logger)
    self.gpio = bcm_gpio.BCM_GPIO(t)
    self.spi = spi.BCM_SPI(t)
    self.spi.reset()
    self.spi.fifo_flush(True, True)

    self.sd = sdhc.SD(
      SDHCI(self.__t, logger),
      SDHOST(self.gpio, self.__t, logger), use_sdhost=True)

  def mem_read32(self, address):
    return self.__t.mem_read32(address)[0]

  def mem_read64(self, address):
    return self.__t.mem_read64(address)[0]

  def mem_read(self, address, size):
    word_size = 8
    if size == 0:
      return b''

    print(f'target mem_read {address}, {size}')
    count = max(int(size / word_size), 1)
    extra_bytes = count * word_size - size 
    if word_size == 4:
      values = self.__t.mem_read32(address, count)
    elif word_size == 8:
      values = self.__t.mem_read64(address, count)

    result = bytearray()
    for i in values:
      result.extend(i.to_bytes(word_size, 'little'))
    return result[:-extra_bytes]

  def mem_write32(self, address, v):
    self.__t.mem_write32(address, v)

  def reset(self):
    self.__t.reset()

  def resume(self):
    self.__t.write_reg('pc', 0x80000 + 4)
    err, pc_value = self.__t.read_reg('pc')
    logging.debug(pc_value)
    self.__t.resume()


  def display(self):
    d = ili9341.ILI9341(self.spi, self.gpio, gpio_pin_blk=19, gpio_pin_dc=13, gpio_pin_reset=6)
    cs = self.spi.regs.spi_cs_write(0)
    cs = self.spi.regs.spi_cs_read()
    print(f'SPI_CS:{cs:08x} ', end = '')
    print(spi.spi_cs_to_str(cs))
    d.write_cmd(ili9341.ILI9341_CMD_SOFT_RESET)
    d.write_cmd(ili9341.ILI9341_CMD_DISPLAY_OFF)
    d.write_cmd(ili9341.ILI9341_CMD_MEM_ACCESS_CONTROL)
    d.write_data([0xe8])
    d.write_cmd(ili9341.ILI9341_CMD_COLMOD)
    d.write_data([0x66])
    d.write_cmd(ili9341.ILI9341_CMD_SLEEP_OUT)
    d.write_cmd(ili9341.ILI9341_CMD_DISPLAY_ON)
    d.write_cmd(ili9341.ILI9341_CMD_CASET)
    d.write_data(ili9341.ili9341_caset_get_arg(20, 25))
    d.write_cmd(ili9341.ILI9341_CMD_PASET)
    d.write_data(ili9341.ili9341_paset_get_arg(20, 25))
    d.write_cmd(ili9341.ILI9341_CMD_RAMWR)
    while True:
      for i in range(5):
        d.write_data([0xff, 0x00, 0x00])
      for i in range(5):
        d.write_data([0xff, 0xff, 0x00])
      for i in range(5):
        d.write_data([0, 0xff, 0x00])
      for i in range(5):
        d.write_data([0, 0, 0xff])




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
  log_tty = logging.getLogger('D_TTY')
  log = logging.getLogger('D_STA')
  t = Target(s, log, log_tty)
  t.update_status()
  status = t.get_status()
  if not status.attached:
    t.init()
    status = t.get_status()
    if not status.attached:
      print('Target not attached, exiting ...')
      sys.exit(-1)

  if not status.halted:
    t.halt()
    status = t.get_status()
  t.dump_regs()
  t.mem_read32(0x80000, 24)

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
  sdhc.parse_scr(scr, logging)
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


class ListHead(ctypes.Structure):
  _fields_ = [
    ("next", ctypes.c_uint64),
    ("prev", ctypes.c_uint64),
  ]

class Event(ctypes.Structure):
    pass

class Stack(ctypes.Structure):
    pass

class Task(ctypes.Structure):
  _fields_ = [
    ("scheduler_list", ListHead),
    ("name", ctypes.c_char * 16),
    ("task_id", ctypes.c_uint32),
    ("cpuctx", ctypes.c_void_p),
    ("next_wakeup_time", ctypes.c_uint64),
    ("scheduler_request", ctypes.c_uint8),
    ("wait_event", ctypes.POINTER(Event)),
    ("stack", ctypes.POINTER(Stack)),
  ]


class Timer(ctypes.Structure):
    pass  # unknown layout, just need a placeholder

class Scheduler(ctypes.Structure):
  _fields_ = [
    ("current", ctypes.c_uint64),
    ("runnable", ListHead),
    ("blocked_on_event", ListHead),
    ("blocked_on_timer", ListHead),
    ("sched_timer", ctypes.POINTER(Timer)),
    ("timer_interval_ms", ctypes.c_int),
    ("idle_task", ctypes.POINTER(Task)),
    ("ticks", ctypes.c_uint64),
    ("needs_resched", ctypes.c_bool),
  ]

def bytes_to_scheduler(raw_bytes):
  buf = (ctypes.c_char * len(raw_bytes)).from_buffer_copy(raw_bytes)
  return ctypes.cast(ctypes.pointer(buf), ctypes.POINTER(Scheduler)).contents

def bytes_to_task(raw_bytes):
  buf = (ctypes.c_char * len(raw_bytes)).from_buffer_copy(raw_bytes)
  return ctypes.cast(ctypes.pointer(buf), ctypes.POINTER(Task)).contents

def do_main(ttydev, action, elf):
  soc = target_attach_and_halt(ttydev, 115200 * 8)
  if action == 'reset':
    logging.info('resetting')
    soc.reset()
  if action == 'display':
    logging.info('display')
    soc.display()
  elif action == 'resume':
    logging.info('resuming')
    soc.resume()
  elif action == 'halt':
    pass
  elif action.startswith('r32'):
    addr = int(action[3:], 0)
    v = soc.mem_read32(addr)
    print(f'mem_read32 result: 0x{addr:08x}: 0x{v:08x}')
  elif action.startswith('r64'):
    addr = int(action[3:], 0)
    v = soc.mem_read64(addr)
    print(f'mem_read64 result: 0x{addr:08x}: 0x{v:08x}')
  elif action == 'initsd':
    action_sdhc(soc)
  elif action == 'script':
    addr = get_symbol_address(elf, '__current_cpuctx')
    v = soc.mem_read64(addr)
    print(f'__current_cpuctx at 0x{addr:016x} = 0x{v:016x}')
    sched_addr = get_symbol_address(elf, 'sched')
    rawbytes = soc.mem_read(sched_addr, ctypes.sizeof(Scheduler))
    sched = bytes_to_scheduler(rawbytes)
    print(f'scheduler:0x{sched_addr:016x}')
    print(f'scheduler.current:0x{sched.current:016x}')
    print(f'runlist:next=0x{sched.runnable.next:016x},prev=0x{sched.runnable.prev:016x}')
    rawbytes = soc.mem_read(sched.current, ctypes.sizeof(Task))
    print(f'task at 0x{sched.current:016x}, size={len(rawbytes)}')
    task = bytes_to_task(rawbytes)
    task_name = task.name.decode('utf-8')
    print(f'task:{task.task_id}, name:{task_name}, cpuctx:0x{task.cpuctx:016x}')
  sys.exit(0)


def main(ttydev):
  parser = argparse.ArgumentParser()

  parser.add_argument('-v', '--verbose',
    help='print verbose logs',
    action='store_const',
    dest='loglevel',
    const=logging.DEBUG,
    default=logging.INFO)

  parser.add_argument('-t', '--tty',
    help='tty device',
    dest='ttydev',
    default='/dev/ttyJTAG')

  parser.add_argument('-e', '--elf',
    help='elf file',
    dest='elf',
    default='')

  parser.add_argument("action", nargs="?", default="default",
    help="Action to perform")

  args = parser.parse_args()
  print(args)
  logging.basicConfig(format='%(levelname)s:%(message)s', level=args.loglevel)
  try:
    do_main(args.ttydev, args.action, args.elf)
  except serial.SerialException as e:
    print(f"Serial error: {e}")
  except KeyboardInterrupt:
    print("Exiting program.")

if __name__ == "__main__":
  main(sys.argv[1])
