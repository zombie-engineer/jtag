import argparse
import socket
import struct
import sys
import threading
import binascii
import errno
import time
import control
import logging
from typing import Optional, Tuple, Dict
import serial
import re
import json


os_json = None

class OsTask:
  def __init__(self, tid, addr, name, cpuctx, current):
    self.tid = tid
    self.address = addr
    self.name = name
    self.cpuctx = cpuctx
    self.is_current = current

  def __repr__(self):
    result = '* ' if self.is_current else ''
    result += f'tid:{self.tid} addr:0x{self.address:016x}'
    result += f' "{self.name}" ctx:0x{self.cpuctx:016x}'
    return result


# AArch64 registers layout matches default one in gdb.
# For greater details generation of target.xml should be supported by this
# module
AARCH64_NUM_REGS = 33
AARCH64_REGSET_SIZE = AARCH64_NUM_REGS * 8
REG_ORDER = [f'x{i}' for i in range(31)] + ['sp', 'pc']
REG_NAMES = [ 'x0', 'x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9',
  'x10', 'x11', 'x12', 'x13', 'x14', 'x15', 'x16', 'x17', 'x18', 'x19',
  'x20', 'x21', 'x22', 'x23', 'x24', 'x25', 'x26', 'x27', 'x28', 'x29',
  'x30', 'sp', 'pc', 'pstate' ]

STOP_SIGNAL = 5

def calc_checksum(data: bytes) -> int:
  return sum(data) % 256


def swap_bytes_64(value: int) -> int:
  if value & ~0xffffffffffffffff:
    raise ValueError("Value must be a 64-bit integer")
  return int.from_bytes(value.to_bytes(8, byteorder='little'), byteorder='big')

def to_hex(data: bytes) -> str:
  return binascii.hexlify(data).decode()

def from_hex(s: str) -> bytes:
  return binascii.unhexlify(s.encode())

def to_word_aligned(val, word_size):
  return int((val + word_size - 1) / word_size) * word_size

def get_tasks_brief(backend, log):
  tasks_addr = int(os_json['tasks_array'], 16)
  task_size = os_json['task_size']
  task_current = int(os_json['task_current'], 16)
  task_offset_name = os_json['task_offset_name']
  task_offset_cpuctx = os_json['task_offset_cpuctx']
  # task_current_addr = backend.read_mem(task_current, 8)
  tasks = [
  ]

  for i in range(12):
    memaddr_cpuctx_addr = tasks_addr + i * task_size + task_offset_cpuctx
    rawbytes = backend.read_mem(memaddr_cpuctx_addr, 8)
    cpuctx_addr = int.from_bytes(rawbytes, 'little')
    print(f'{memaddr_cpuctx_addr:016x}: {cpuctx_addr:016x}')
    if not cpuctx_addr:
      break

    tasks.append(cpuctx_addr)
  return tasks

def get_tasks(backend, log):
  tasks_addr = int(os_json['tasks_array'], 16)
  task_size = os_json['task_size']
  task_current = int(os_json['task_current'], 16)
  task_offset_name = os_json['task_offset_name']
  task_offset_id = os_json['task_offset_id']
  task_offset_cpuctx = os_json['task_offset_cpuctx']

  task_current_addr = backend.read_mem(task_current, 8)
  task_current_addr = int.from_bytes(task_current_addr, 'little')
  log.debug(f'current_task: 0x{task_current_addr:016x}')

  tasks = [
  ]

  for i in range(12):
    task_addr = tasks_addr + i * task_size
    task_raw = backend.read_mem(task_addr, task_size)
    task_name = task_raw[task_offset_name: task_offset_name + 16]
    task_name = task_name.split(b'\x00', 1)
    task_name = task_name[0].decode('ascii')
    if len(task_name) == 0:
      break
    task_id = task_raw[task_offset_id: task_offset_id + 4]
    task_id = int.from_bytes(task_id, 'little') + 1
    task_cpuctx = task_raw[task_offset_cpuctx: task_offset_cpuctx + 8]
    task_cpuctx = int.from_bytes(task_cpuctx, 'little')
    current = task_addr == task_current_addr
    log.info(f'found task, tid:{task_id}, name:"{task_name}"')
    tasks.append(OsTask(task_id, task_addr, task_name, task_cpuctx,
      current))

  return tasks


class JTAGBackend:
  def __init__(self, target: control.Target, logger):
    self.__log = logger
    self.__t = target
    self.__t.init()
    self.__t.halt()
    self.__tasks = None
    self.__tasks_brief = get_tasks_brief(self, self.__log)
    self.__task_names = [None for i in self.__tasks_brief]
    self.__task_cpuctx_addrs = [None for i in self.__tasks_brief]
    self.__current_task_cached_addr = None
    for i, t in enumerate(self.__tasks_brief):
      self.__log.debug(f'{t:016x}')
    self.__reg_tid = None # self.get_current_task().tid

  def update_tasks_on_halt(self):
    self.__tasks = None
    self.__tasks_brief = get_tasks_brief(self, self.__log)
    self.__task_names = [None for i in self.__tasks_brief]
    self.__current_task_cached_addr = None
    for i, t in enumerate(self.__tasks_brief):
      self.__log.debug(f'{t:016x}')
    self.__reg_tid = None # self.get_current_task().tid

  def disconnect(self):
    if self.__t:
      self.__t = None

  def resume(self) -> None:
    self.__t.resume()

  def reset(self) -> None:
    self.__t.reset()

  def step(self) -> None:
    self.__t.step()

  def interrupt(self) -> None:
    self.__log.info('interrupt')
    self.__t.halt()

  def wait_stop(self) -> bool:
    halted = self.__t.check_is_halted()
    if halted:
      self.update_tasks_on_halt()
    return halted

  def get_reg_tid(self):
    if self.__reg_tid is None:
      self.__reg_tid = self.get_current_task_tid()
    return self.__reg_tid

  def set_reg_tid(self, tid):
    if tid == 0:
      self.__reg_tid = self.get_current_task_tid()
    else:
      self.__reg_tid = tid
    self.__log.debug(f'current tid for \'g\' ops set to \'{self.__reg_tid}\'')

  def read_all_registers(self):
    reg_tid = self.get_reg_tid()
    self.__log.debug(f'read_all_registers from tid {reg_tid}')
    if reg_tid == self.get_current_task_tid():
      regs = self.__t.dump_regs()[0]
    else:
      cpuctx = self.get_task_cpuctx_addr(reg_tid)
      regs_raw = self.read_mem(cpuctx, AARCH64_REGSET_SIZE)
      regs = {}
      for r in REG_ORDER:
        regs[r] = int.from_bytes(regs_raw[:8], 'little')
        regs_raw = regs_raw[8:]
      if self.__log.getEffectiveLevel() == logging.DEBUG:
        self.__log.debug(f'TID {reg_tid} regs:')
        for k,v in regs.items():
          self.__log.debug(f'  {k}: 0x{v:016x}')
    b = bytearray()
    for r in REG_ORDER:
      b.extend(regs[r].to_bytes(8, 'little'))
    return bytes(b)

  def reg_id_to_name(self, reg_id) -> str:
    if reg_id <= 30:
      return f'x{reg_id}'
    elif reg_id == 31:
      return 'sp'
    elif reg_id == 32:
      return 'pc'
    elif reg_id == 33:
      return 'cpsr'
    elif reg_id == 66:
      return 'fpsr'
    elif reg_id == 67:
      return 'fpcr'
    return None

  def write_register(self, reg_id, regval) -> bool:
    regname = self.reg_id_to_name(reg_id)
    if not regname:
      self.__log.warn('Could not find register with id {reg_id}')
      return False
    self.__log.debug(
      'Writing to register {regname} (#{reg_id}), value:0x{regval:016x}')
    return self.__t.write_reg(regname, swap_bytes_64(regval))

  def read_register(self, reg_id):
    regname = self.reg_id_to_name(reg_id)
    if not regname:
      self.__log.warn('Could not find register with id {reg_id}')
      return False, 0
    return self.__t.read_reg(regname)

  def write_all_registers(self, raw: bytes) -> bool:
    num_regs = int(len(raw) / 8)
    allregs = struct.unpack('<' + 'Q' * num_regs, raw)
    for i, regval in enumerate(allregs):
      self.__log.debug(f'writing {REG_NAMES[i]} = 0x{regval:016x}')
      if not self.__t.write_reg(REG_NAMES[i], regval):
        return False
    return True

  def read_mem(self, addr: int, length: int) -> Optional[bytes]:
    self.__log.debug(f'read mem: addr 0x{addr:016x}, length:{length}')
    if not length:
      return b''

    if length <= 4:
      wordsize = 4
      addrmask = 0xfffffffffffffffc
      read_words_fn = self.__t.mem_read32
    else:
      wordsize = 8
      addrmask = 0xfffffffffffffff8
      read_words_fn = self.__t.mem_read64

    addr_aligned = addr & addrmask
    head_skip = addr - addr_aligned
    read_length = head_skip + length
    read_length_aligned = to_word_aligned(read_length, wordsize)
    num_words = int(read_length_aligned / wordsize)
    tail_skip = num_words * wordsize - read_length
    values = read_words_fn(addr_aligned, num_words)

    result = bytearray()
    for i in values:
      tmp = i.to_bytes(wordsize, 'little')
      result.extend(tmp)

    if tail_skip:
      result = result[:-tail_skip]
    return result[head_skip:]

  def write_mem(self, addr: int, data: bytes) -> bool:
    self.__log.debug(f'write_mem: {addr:016x}, data:{data}')
    if len(data) == 4:
      return self.__t.mem_write32(addr, int().from_bytes(data, 'little'))
    if len(data) == 8:
      return self.__t.mem_write64(addr, int().from_bytes(data, 'little'))
    self.__log.debug(f'write_mem: data length unsupported {len(data)}')
    return False

  def set_sw_break(self, addr: int, kind: int) -> bool:
    self.__log.info(f'Software breakpoint at 0x{addr:016x}')
    return self.__t.breakpoint(addr, kind, add=True, hardware=False)

  def clr_sw_break(self, addr: int, kind: int) -> bool:
    return self.__t.breakpoint(addr, kind, add=False, hardware=False)

  def set_hw_break(self, addr: int, kind: int) -> bool:
    return self.__t.breakpoint(addr, kind, add=True, hardware=True)

  def clr_hw_break(self, addr: int, kind: int) -> bool:
    return self.__t.breakpoint(addr, kind, add=False, hardware=True)

  def get_all_tids(self):
    return [i + 1 for i, _ in enumerate(self.__tasks_brief)]

  def get_thread_by_tid(self, tid):
    for i in self.__tasks:
      if i.tid == tid:
        return i
    return None

  def get_thread_name(self, tid):
    if self.__task_names[tid - 1] is None:
      tasks_addr = int(os_json['tasks_array'], 16)
      task_size = os_json['task_size']
      task_offset_name = os_json['task_offset_name']
      task_name_addr = tasks_addr + (tid - 1) * task_size + task_offset_name
      rawbytes = self.read_mem(task_name_addr, 16)
      task_name = rawbytes.split(b'\x00', 1)[0].decode('ascii')
      print(f'task name is {task_name}')
      self.__task_names[tid - 1] = task_name
    return self.__task_names[tid - 1]

  def get_current_task_tid(self):
    if self.__current_task_cached_addr is None:
      current_task_addr_addr = int(os_json['task_current'], 16)
      rawbytes = self.read_mem(current_task_addr_addr, 8)
      self.__current_task_cached_addr = int.from_bytes(rawbytes, 'little')
    assert(self.__current_task_cached_addr)
    tasks_addr = int(os_json['tasks_array'], 16)
    task_size = os_json['task_size']
    return int((self.__current_task_cached_addr - tasks_addr) / task_size) + 1

  def get_task_cpuctx_addr(self, tid):
    if self.__task_cpuctx_addrs[tid - 1] is None:
      tasks_addr = int(os_json['tasks_array'], 16)
      task_size = os_json['task_size']
      cpuctx_offset = os_json['task_offset_cpuctx']
      cpuctx_addr_addr = tasks_addr + (tid - 1) * task_size + cpuctx_offset
      rawbytes = self.read_mem(cpuctx_addr_addr, 8)
      self.__task_cpuctx_addrs[tid - 1] = int.from_bytes(rawbytes, 'little')
    return self.__task_cpuctx_addrs[tid - 1]


  def get_current_task(self):
    return None


STATE_NORMAL = 1
STATE_WAIT_STOP = 2

class RSPServer:
  def __init__(self, host: str, port: int, backend: JTAGBackend,
    logger_rsp, logger):

    self.host = host
    self.port = port
    self.backend = backend
    self.sock = None
    self.conn = None
    self.addr = None
    self.no_ack_mode = False
    self.state = STATE_NORMAL
    self.__log_rsp = logger_rsp
    self.__log = logger

  def send(self, packet: str):
    if not self.conn:
      return

    self.__log_rsp.debug(f'sending: "{packet}"')
    self.conn.sendall(packet.encode())

  def send_packet(self, payload: str):
    csum = calc_checksum(payload.encode())
    packet = f'${payload}#{csum:02x}'
    self.send(packet)
    if not self.no_ack_mode:
      # Optionally wait for '+' from GDB; many stubs omit waiting.
      try:
        self.conn.settimeout(0.5)
        ack = self.conn.recv(1)
        # ignore content; GDB should send '+'
      except Exception:
        pass
      finally:
        self.conn.settimeout(None)

  def send_ok(self):
    self.send_packet('OK')

  def send_error(self, code: int = 1):
    self.send_packet(f'E{code:02x}')

  def send_signal(self, sig: int = 5):
    self.send_packet(f'S{sig:02x}')

  def send_stop_packet(self, breakpoint):
    sig = 5 # TRAP
    tid = self.backend.get_current_task_tid()
    packet = f'T{sig:02x}thread:{tid:02};'
    if breakpoint:
      packet += 'reason:swbreak;'
    self.send_packet(packet)

  def on_break_request(self):
    # Async break (Ctrl-C)
    self.__log_rsp.debug('\x03 async break')
    self.backend.interrupt()
    # Report stop
    self.send_signal(5)

  def client_read_cmd_line(self):
    # read until '#'
    data = bytearray()
    while True:
      ch = self.conn.recv(1)
      if ch == b"#":
        break
      data.extend(ch)
    return data

  def client_read_test_csum(self, data):
    csum_bytes = self.conn.recv(2)
    chksum = int(csum_bytes.decode(), 16)
    return calc_checksum(bytes(data)) == chksum

  def client_read_cmd(self):
    data = self.client_read_cmd_line()
    csum_ok = self.client_read_test_csum(data)
    if not self.no_ack_mode:
      self.send('+' if csum_ok else '-')

    self.__log_rsp.debug(
      f'received: data:{data}, ack_mode:{self.no_ack_mode}, csum:{csum_ok}')
    return data.decode() if csum_ok else None

  def conn_read_byte_with_timeout(self):
    self.conn.settimeout(0.5)
    try:
      b = self.conn.recv(1)
      return b
    except socket.timeout as e:
      self.__log_rsp.warn(f'exception while reading byte: {e}')
      raise
    finally:
      self.conn.settimeout(None)

  # Receive command packet from gdb. Returning None signals that client (gdb)
  # has disconnected
  def client_get_cmd(self, use_timeout) -> Optional[str]:
    while True:
      if use_timeout:
        b = self.conn_read_byte_with_timeout()
      else:
        b = self.conn.recv(1)
      if not b:
        return None

      if b == b"\x03":
        # Special case of Ctrl+C on gdb side
        self.on_break_request()
        continue

      if b != b'$':
        continue

      packet = self.client_read_cmd()
      if packet is None:
        continue

      return packet

  def on_breakpoint(self, cmd):
    # Breakpoints Z/z type,addr,kind
    add = cmd[0] == 'Z'
    body = cmd[1:]
    type_s, addr_s, kind_s = body.split(",")
    btype = int(type_s, 10)
    addr = int('0x' + addr_s, 16)
    kind = int(kind_s, 16)
    ok = False
    if btype == 0:  # swbp
      ok = self.backend.set_sw_break(addr, kind) if add else self.backend.clr_sw_break(addr, kind)
    elif btype == 1:  # hwbp
      ok = self.backend.set_hw_break(addr, kind) if add else self.backend.clr_hw_break(addr, kind)
    else:
      ok = False
    self.send_ok() if ok else self.send_error(1)

  def on_memory_read(self, cmd):
    body = cmd[1:]
    # maddr,length
    addr_s, len_s = body.split(",")
    addr = int(addr_s, 16)
    length = int(len_s, 16)
    data = self.backend.read_mem(addr, length)
    if len(data) == length:
      self.send_packet(to_hex(data))
    else:
      self.__log.error(f'data is {data}, len:{len(data)}, should_be:{length}')
      self.send_error(1)

  def on_memory_write(self, cmd):
    body = cmd[1:]
    # Maddr,length:data
    # Example of a command: Mffff000000373628,8:ff00000000000000
    addr_len, hexdata = body.split(":", 1)
    addr_s, len_s = addr_len.split(",")
    addr = int(addr_s, 16)
    length = int(len_s, 16)
    data = from_hex(hexdata)
    ok = len(data) == length
    if ok:
      ok = self.backend.write_mem(addr, data)
    if ok:
      self.send_ok()
    else:
      self.send_error(1)


  def on_reg_access(self, cmd):
    read = cmd[0] == 'p'
    body = cmd[1:]
    if not read:
      reg_id, value = body.split('=')
      reg_id = int('0x' + reg_id, 16)
      value = int('0x' + value, 16)
      self.__log.debug(f'requested register write reg {reg_id} = 0x{value:016x}')
      if self.backend.write_register(reg_id, value):
        self.send_ok()
      else:
        self.send_error(1)
      return

    reg_id = int('0x' + body, 16)
    success, value = self.backend.read_register(reg_id)
    if success:
      self.send_packet(f'{value:016x}')
    else:
      self.send_error(1)

  def on_q_supported(self, cmd):
    self.send_packet(';'.join([
      "PacketSize=4000",
      "swbreak+",
      "hwbreak+",
      "vContSupported+",
      "QStartNoAckMode+"
    ]))
 
  def on_q_start_no_ack_mode(self, cmd):
    self.no_ack_mode = True
    # acknowledge the command itself
    self.send('+')
    self.send_ok()

  def on_v_must_reply_empty(self, cmd):
    self.send_packet("")

  def on_q_attached(self, cmd):
    self.send_packet("1")

  def q_xfer_features_read(self, cmd):
    # Not serving target.xml; reply with 'l' (end of data)
    self.send_packet("l")

  def on_qf_thread_info(self, cmd):
    tids = self.backend.get_all_tids()
    if tids:
      self.send_packet("m" + ",".join(str(t) for t in tids))
    else:
      self.send_packet("l")

  def on_qs_thread_info(self, cmd):
    self.send_packet("l")

  def on_q_thread_extra_info(self, cmd):
    tid = int(cmd.split(',')[1])
    name = self.backend.get_thread_name(tid)
    if name is not None:
      name = to_hex(name.encode('ascii'))
      self.__log.info(f'requested extra info for thread {tid}. Name={name}')
      self.send_packet(name)
    else:
      self.__log.warn(f'requested extra info for unknown thread {tid}')
      self.send_error(1)

  def on_q_offsets(self, cmd):
    self.send_packet('Text=0;Data=0;Bss=0')

  def on_q_current_thread(self, cmd):
    tid = self.backend.get_current_task_tid()
    self.send_packet(f"QC{tid:x}")

  def on_q_thread_alive(self, cmd):
    # Thread-alive query 'Txx'
    self.send_packet("OK")

  def on_set_current_thread(self, cmd):
    tid = int(cmd[2:])
    if cmd[1] == 'g':
      self.__log.info(f'Set register tid to {tid}')
      self.backend.set_reg_tid(tid)
    # Set thread for step/continue or for reg access.
    # Expect 'Hc{tid}' / 'Hg{tid}'
    self.send_ok()

  def on_q_stop_reason(self, cmd):
    self.send_stop_packet(breakpoint=False)

  def on_read_all_regs(self, cmd):
    data = self.backend.read_all_registers()
    self.send_packet(to_hex(data))

  def on_write_all_regs(self, cmd):
    raw = from_hex(cmd[1:])
    ok = len(raw) == AARCH64_REGSET_SIZE
    if ok:
      ok = self.backend.write_all_registers(raw)

    if ok:
      self.send_ok()
    else:
      self.send_error(1)

  def on_v_cont_q(self, cmd):
    self.send_packet("vCont;c;s")

  def __resume(self, step: bool):
    if step:
      self.backend.step()
    else:
      self.backend.resume()

    if not self.backend.wait_stop():
      self.state = STATE_WAIT_STOP
    else:
      self.send_stop_packet(True)

  def on_v_cont(self, cmd):
    # Example formats:
    # vCont;c or vCont;s or per-thread forms. We'll just handle global c/s.
    self.__resume(step=";s" in cmd)

  def on_resume(self, cmd):
    self.__resume(step=False)

  def on_step(self, cmd):
    self.__resume(step=True)

  def on_remote_cmd(self, cmd):
    cmd = cmd[len('qRcmd,'):]
    cmd = from_hex(cmd).decode()
    self.__log.debug(f'received remote command from gdb: "{cmd}"')
    if cmd == 'reset':
      self.backend.reset()
    self.send_ok()

  def handle_cmd(self, cmd: str):
    self.__log.debug(f'handling RSP command: "{cmd}"')
    dispatch_table = [
      (r'^qSupported'         , self.on_q_supported),
      (r'^QStartNoAckMode$'   , self.on_q_start_no_ack_mode),
      (r'^vMustReplyEmpty'    , self.on_v_must_reply_empty),
      (r'^qAttached$'         , self.on_q_attached),
      (r'^qXfer:features:read', self.q_xfer_features_read),
      (r'^qfThreadInfo$'      , self.on_qf_thread_info),
      (r'^qsThreadInfo$'      , self.on_qs_thread_info),
      (r'^qThreadExtraInfo'   , self.on_q_thread_extra_info),
      (r'^qOffsets$'          , self.on_q_offsets),
      (r'^qC$'                , self.on_q_current_thread),
      (r'^T'                  , self.on_q_thread_alive),
      (r'^H[cg]'              , self.on_set_current_thread),
      (r'^\?$'                , self.on_q_stop_reason),
      (r'^g$'                 , self.on_read_all_regs),
      (r'^G$'                 , self.on_write_all_regs),
      (r'^[Pp]'               , self.on_reg_access),
      (r'^m'                  , self.on_memory_read),
      (r'^M'                  , self.on_memory_write),
      (r'^[Zz]'               , self.on_breakpoint),
      (r'^vCont[?]$'          , self.on_v_cont_q),
      (r'^vCont;'             , self.on_v_cont),
      (r'^c$'                 , self.on_resume),
      (r'^s$'                 , self.on_step),
      (r'^qRcmd,'             , self.on_remote_cmd),
    ]

    for regex, handler in dispatch_table:
      if re.match(regex, cmd):
        handler(cmd)
        return

    # Handler not found, send empty packet
    self.send_packet("")

  def run_session_loop(self):
    self.state = STATE_NORMAL
    while True:
      if self.state == STATE_WAIT_STOP:
        self.__log.info('will wait for stop...')
        if self.backend.wait_stop():
          self.__log.info('stopped')
          self.state = STATE_NORMAL
          self.send_stop_packet(True)

      try:
        cmd = self.client_get_cmd(self.state == STATE_WAIT_STOP)
      except socket.timeout as e:
        self.__log.debug('timeout..')
        continue

      self.state = STATE_NORMAL
      if cmd is None:
        self.__log.info("[-] Disconnected")
        return
      self.handle_cmd(cmd)


  def serve(self):
    # self.backend.connect()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      s.bind((self.host, self.port))
      s.listen(1)
      self.__log.info(f"[+] Listening on {self.host}:{self.port}")
      self.conn, self.addr = s.accept()
      self.__log.info(f"[+] GDB connected from {self.addr}")
      with self.conn:
        self.run_session_loop()


def main():
  global os_json

  p = argparse.ArgumentParser(
    description="AArch64 GDB RSP stub bridging to a JTAG over /dev/ttyACM*")

  p.add_argument("--host", default="0.0.0.0")
  p.add_argument("--port", type=int, default=1234)
  p.add_argument("--serial", default="/dev/ttyACM0")
  p.add_argument("--baud", type=int, default=115200)
  p.add_argument('-v', '--verbose', help='print verbose logs',
    action='store_const', dest='loglevel', const=logging.DEBUG,
    default=logging.INFO)

  p.add_argument('--log-dbg-tty',
    help='Log tty messages to/from debugger',
    action='store_const', dest='log_level_debugger_tty', const=logging.DEBUG,
    default=logging.INFO)

  p.add_argument('--log-dbg',
    help='Log debugger communication statuses',
    action='store_const', dest='log_level_debugger', const=logging.DEBUG,
    default=logging.INFO)

  p.add_argument('--log-rsp',
    help='Log RSP (remote serial protocol) messages',
    action='store_const', dest='log_level_rsp', const=logging.DEBUG,
    default=logging.INFO)

  p.add_argument("--osdesc")
  args = p.parse_args()
  logging_fmt='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
  logging_datefmt='%s'

  logging.basicConfig(format=logging_fmt, datefmt=logging_datefmt,
    level=args.loglevel)

  logger_gdb_server = logging.getLogger('SRV')

  logger_dbg_tty = logging.getLogger('D_TTY')
  logger_dbg = logging.getLogger('D_STA')
  logger_rsp = logging.getLogger('RSP')

  logger_gdb_server.setLevel(args.loglevel)
  logger_dbg_tty.setLevel(args.log_level_debugger_tty)
  logger_dbg.setLevel(args.log_level_debugger)
  logger_rsp.setLevel(args.log_level_rsp)

  os_json = json.loads(open(args.osdesc, 'rt').read())

  debuger_serial = serial.Serial(args.serial, args.baud, timeout=100)
  logger_gdb_server.info(f'Opened {args.serial} with baud rate {args.baud}')

  debugger = control.Target(debuger_serial, logger_dbg, logger_dbg_tty)

  backend = JTAGBackend(debugger, logger_gdb_server)
  server = RSPServer(args.host, args.port, backend, logger_rsp,
    logger_gdb_server)

  server.serve()


if __name__ == "__main__":
  main()

