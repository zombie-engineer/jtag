#!/usr/bin/env python3

"""
Minimalistic GDB server proxy that is part of this scheme:
gdb->[gdbserver.py]->control.py->stm32discovery JTAG firmware->Raspberry Pi
over /dev/ttyACM* using a simple placeholder protocol.

Features implemented (stubbed where necessary):
- TCP socket listener for GDB Remote Serial Protocol (RSP)
- Packet framing, checksum verify, and acknowledgements (+/-)
- Optional no-ack mode via QStartNoAckMode
- Basic queries: qSupported, qAttached, qfThreadInfo/qsThreadInfo, qC, T, qXfer:features:read (stub 'l')
- Register read/write: g/G (AArch64 zeros unless backend supplies real values)
- Memory read/write: m/M (bridged to backend)
- Continue/step: c/s and vCont;c;s (bridged to backend)
- Breakpoints: Z/z (types 0=swbp, 1=hwbp; others stubbed)
- Interrupt (^C) mapping to backend.break

Backend abstraction:
- JTAGBackend: skeleton using pyserial to talk to /dev/ttyACM* with a placeholder ASCII
  protocol. Replace send_cmd/parse_* with your real device protocol.

This is a skeleton intended for you to fill in backend calls.
"""

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

AARCH64_NUM_REGS = 33
AARCH64_REGSET_SIZE = AARCH64_NUM_REGS * 8

def swap_bytes_64(value: int) -> int:
    """Swap bytes of a 64-bit integer (endianness conversion)."""
    if not (0 <= value < 1 << 64):
        raise ValueError("Value must be a 64-bit integer")
    # Convert to bytes, reverse, convert back to int
    return int.from_bytes(value.to_bytes(8, byteorder='little'), byteorder='big')

def to_hex(data: bytes) -> str:
  return binascii.hexlify(data).decode()

def from_hex(s: str) -> bytes:
  return binascii.unhexlify(s.encode())

def to_word_aligned(val, word_size):
  return int((val + word_size - 1) / word_size) * word_size



class JTAGBackend:
  """Skeleton backend that speaks to a custom JTAG over /dev/ttyACM* using pyserial.
  Replace send_cmd/parse_* bodies with your device's real protocol.
  """
  def __init__(self, port: str = "/dev/ttyACM0", baud: int = 115200):
    self.__port = port
    self.__baud = baud
    self.__t = None

  def connect(self):
    s = serial.Serial(self.__port, self.__baud, timeout=100)
    self.__t = control.Target(s)
    logging.info(f"Opened {self.__port} with baud rate {self.__baud}")
    self.__t.init()
    self.__t.halt()

  def disconnect(self):
    if self.__t:
      self.__t = None

  def send_cmd(self, cmd: str, payload: bytes = b"") -> bytes:
    """Send a line command like: CMD <hex>\n and read a single line reply."""
    self.__t.write(payload)
    line = (cmd + (" " + to_hex(payload) if payload else "") + "\n").encode()
    self.ser.write(line)
    self.ser.flush()
    return self.ser.readline().strip()

  def continue_(self) -> None:
    self.__t.resume()

  def step(self) -> None:
    self.__t.step()

  def interrupt(self) -> None:
    print('interrupt')
    self.__t.halt()

  def wait_stop(self, timeout: Optional[float] = None) -> Tuple[str, Optional[int]]:
    # Polling example; replace with event-driven read from device
    t0 = time.time()
    while not self.__t.check_is_halted():
      print('is not halted yet')
      time.sleep(1)
    return ("signal", 5)

  def read_all_registers(self) -> bytes:
    print('read_all_registers')
    all_regs = self.__t.dump_regs()
    order = [f"x{i}" for i in range(31)] + ["sp", "pc"]#, "pstate"]
    regs = all_regs[0]
    b = bytearray()
    for r in order:
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
    print('regname', regname, reg_id)
    if not regname:
      return False
    return self.__t.write_reg(regname, swap_bytes_64(regval))

  def read_register(self, reg_id):
    regname = self.reg_id_to_name(reg_id)
    if not regname:
      return False, 0
    return self.__t.read_reg(regname)

  def write_all_registers(self, raw: bytes) -> bool:
    num_regs = int(len(raw) / 8)
    regnames = [
      'x0', 'x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9',
      'x10', 'x11', 'x12', 'x13', 'x14', 'x15', 'x16', 'x17', 'x18', 'x19',
      'x20', 'x21', 'x22', 'x23', 'x24', 'x25', 'x26', 'x27', 'x28', 'x29',
      'x30', 'sp', 'pc', 'pstate' ]

    allregs = struct.unpack('<' + 'Q' * num_regs, raw)
    for i, regval in enumerate(allregs):
      print(f'writing {regnames[i]} = 0x{regval:016x}')
      if not self.__t.write_reg(regnames[i], regval):
        return False
    return True

  def read_mem(self, addr: int, length: int) -> Optional[bytes]:
    print(f'read mem: addr 0x{addr:016x}, length:{length}')
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
    print(f'write_mem: {addr:016x}, data:{data}')
    if len(data) == 4:
      return self.__t.mem_write32(addr, int().from_bytes(data, 'little'))
    if len(data) == 8:
      return self.__t.mem_write64(addr, int().from_bytes(data, 'little'))
    print(f'write_mem: data length unsupported {len(data)}')
    return False

  def set_sw_break(self, addr: int, kind: int) -> bool:
    print('set_sw_break')
    return self.__t.breakpoint(addr, kind, add=True, hardware=False)

  def clr_sw_break(self, addr: int, kind: int) -> bool:
    return self.__t.breakpoint(addr, kind, add=False, hardware=False)

  def set_hw_break(self, addr: int, kind: int) -> bool:
    return self.__t.breakpoint(addr, kind, add=True, hardware=True)

  def clr_hw_break(self, addr: int, kind: int) -> bool:
    return self.__t.breakpoint(addr, kind, add=False, hardware=True)

  def list_threads(self):
    return [1]

  def current_thread(self):
    return 1

  def set_thread(self, tid: int) -> bool:
    return tid == 1


class RSPServer:
  def __init__(self, host: str, port: int, backend: JTAGBackend):
    self.host = host
    self.port = port
    self.backend = backend
    self.sock = None
    self.conn = None
    self.addr = None
    self.no_ack_mode = False
    self.current_thread = 1

  # -------------------- Packet framing & IO --------------------
  @staticmethod
  def _csum(data: bytes) -> int:
    return sum(data) % 256

  def _send_raw(self, b: bytes):
    if self.conn:
      self.conn.sendall(b)

  def send_packet(self, payload: str):
    pkt = b"$" + payload.encode() + b"#" + f"{self._csum(payload.encode()):02x}".encode()
    print(pkt)
    self._send_raw(pkt)
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
    self.send_packet("OK")

  def send_error(self, code: int = 1):
    self.send_packet(f"E{code:02x}")

  def send_signal(self, sig: int = 5):
    self.send_packet(f"S{sig:02x}")

  def recv_packet(self) -> Optional[str]:
    data = bytearray()
    while True:
      b = self.conn.recv(1)
      if not b:
        return None
      # Async break (Ctrl-C)
      if b == b"\x03":
        print('\x03 async break')
        # Interrupt request
        try:
          self.backend.interrupt()
        except Exception:
          pass
        # Report stop
        self.send_signal(5)
        continue
      if b == b"$":
        data = bytearray()
        # read until '#'
        while True:
          ch = self.conn.recv(1)
          if ch == b"#":
            break
          data.extend(ch)
        # read checksum
        rx = self.conn.recv(2)
        try:
          rx_csum = int(rx.decode(), 16)
        except Exception:
          rx_csum = -1
        calc = self._csum(bytes(data))
        if rx_csum == calc:
          if not self.no_ack_mode:
            self._send_raw(b"+")
          return data.decode()
        else:
          if not self.no_ack_mode:
            self._send_raw(b"-")
          # continue loop to read next packet

  def on_breakpoint(self, pkt):
    add = pkt[0] == 'Z'
    body = pkt[1:]
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

  def on_memory_read(self, body):
    # maddr,length
    addr_s, len_s = body.split(",")
    addr = int(addr_s, 16)
    length = int(len_s, 16)
    data = self.backend.read_mem(addr, length)
    if len(data) == length:
      self.send_packet(to_hex(data))
    else:
      print(f'error, data is {data}, len:{len(data)}, should_be:{length}')
      self.send_error(1)

  def on_reg_access(self, body, read):
    if not read:
      reg_id, value = body.split('=')
      reg_id = int('0x' + reg_id, 16)
      value = int('0x' + value, 16)
      print(f'requested register write reg {reg_id} = 0x{value:016x}')
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

  def on_memory_write(self, body):
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

  def on_q_supported(self):
    # You can add more tokens as you implement them.
    self.send_packet("PacketSize=4000;swbreak+;hwbreak+;vContSupported+;QStartNoAckMode+")
 
  def on_q_start_no_ack_mode(self):
    self.no_ack_mode = True
    self._send_raw(b"+")  # acknowledge the command itself
    self.send_ok()

  def on_v_must_reply_empty(self):
    self.send_packet("")

  def on_q_attached(self):
    self.send_packet("1")

  def q_xfer_features_read(self):
    # Not serving target.xml; reply with 'l' (end of data)
    self.send_packet("l")

  def handle(self, pkt: str):
    # Queries first
    if pkt.startswith("qSupported"):
      return self.on_q_supported()
    if pkt == "QStartNoAckMode":
      return self.on_q_start_no_ack_mode()
    if pkt.startswith("vMustReplyEmpty"):
      return self.on_v_must_reply_empty()
    if pkt == "qAttached":
      return self.on_q_attached()
    if pkt.startswith("qXfer:features:read"):
      return self.q_xfer_features_read()
    if pkt == "qfThreadInfo":
      tids = self.backend.list_threads()
      if tids:
        self.send_packet("m" + ",".join(str(t) for t in tids))
      else:
        self.send_packet("l")
      return
    if pkt == "qsThreadInfo":
      self.send_packet("l")
      return
    if pkt == 'qOffsets':
      self.send_packet('Text=0;Data=0;Bss=0')
      return
    if pkt == "qC":
      self.send_packet(f"QC{self.backend.current_thread():x}")
      return
    if pkt.startswith("T"):
      # Thread-alive query 'Txx'
      self.send_packet("OK")
      return
    if pkt.startswith("Hc") or pkt.startswith("Hg"):
      # Set thread for step/continue or for reg access. Expect 'Hc{tid}' / 'Hg{tid}'
      self.send_ok()
      return
    if pkt == "?":
      self.send_signal(5)
      return

    # Registers
    if pkt == "g":
      raw = self.backend.read_all_registers()
      self.send_packet(to_hex(raw))
      return

    if pkt.startswith("G"):
      raw = from_hex(pkt[1:])
      ok = len(raw) == AARCH64_REGSET_SIZE
      if ok:
        ok = self.backend.write_all_registers(raw)
      if ok:
        self.send_ok()
      else:
        self.send_error(1)
      return

    if pkt[0] in ['P', 'p']:
      return self.on_reg_access(pkt[1:], pkt[0] == 'p')
    if pkt.startswith("m"):
      return self.on_memory_read(pkt[1:])

    if pkt.startswith("M"):
      return self.on_memory_write(pkt[1:])

    # Breakpoints Z/z type,addr,kind
    if pkt.startswith("Z") or pkt.startswith("z"):
      return self.on_breakpoint(pkt)

    # vCont
    if pkt == "vCont?":
      self.send_packet("vCont;c;s")
      return
    if pkt.startswith("vCont;"):
      # Example formats: vCont;c or vCont;s or per-thread forms. We'll just handle global c/s.
      if ";s" in pkt:
        try:
          self.backend.step()
        except Exception:
          pass
        reason, sig = self.backend.wait_stop()
        self.send_signal(sig or 5)
      else:
        try:
          self.backend.continue_()
        except Exception:
          pass
        reason, sig = self.backend.wait_stop()
        self.send_signal(sig or 5)
      return

    # Classic continue/step
    if pkt == "c":
      try:
        self.backend.continue_()
      except Exception:
        pass
      reason, sig = self.backend.wait_stop()
      self.send_signal(sig or 5)
      return
    if pkt == "s":
      self.backend.step()
      reason, sig = self.backend.wait_stop()
      self.send_signal(sig or 5)
      return

    # Unknown -> empty response
    self.send_packet("")

  # ----------------------------- Server loop -----------------------------

  def serve(self):
    self.backend.connect()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      s.bind((self.host, self.port))
      s.listen(1)
      print(f"[+] Listening on {self.host}:{self.port}")
      self.conn, self.addr = s.accept()
      print(f"[+] GDB connected from {self.addr}")
      with self.conn:
        while True:
          pkt = self.recv_packet()
          if pkt is None:
            print("[-] Disconnected")
            break
          # print for debugging
          print(f"[>] {pkt}")
          self.handle(pkt)


def main():
  import argparse
  p = argparse.ArgumentParser(
    description="AArch64 GDB RSP stub bridging to a JTAG over /dev/ttyACM*")

  p.add_argument("--host", default="0.0.0.0")
  p.add_argument("--port", type=int, default=1234)
  p.add_argument("--serial", default="/dev/ttyACM0")
  p.add_argument("--baud", type=int, default=115200)
  p.add_argument('-v', '--verbose', help='print verbose logs',
    action='store_const', dest='loglevel', const=logging.DEBUG,
    default=logging.INFO)
  args = p.parse_args()
  logging.basicConfig(format='%(levelname)s:%(message)s', level=args.loglevel)

  backend = JTAGBackend(args.serial, args.baud)
  server = RSPServer(args.host, args.port, backend)
  server.serve()


if __name__ == "__main__":
  main()

