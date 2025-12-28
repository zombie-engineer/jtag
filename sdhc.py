from sd import *
from bcm_sdhci import SDHCI
from bcm_sdhost import SDHOST


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


class cmd_result:
  def __init__(self, status, data, resp0, resp1, resp2, resp3):
    self.status = status
    self.data = data
    self.resp0 = resp0
    self.resp1 = resp1
    self.resp2 = resp2
    self.resp3 = resp3

def parse_scr(v, log):
  log.info(f'SCR: 0x{v:016x}')
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
  log.debug(f'scr_struct: {scr_struct}')
  log.debug(f'sd_spec: {sd_spec}')
  log.debug(f'sd_spec3: {sd_spec3}')
  log.debug(f'sd_spec4: {sd_spec4}')
  log.debug(f'sd_specx: {sd_specx}')
  log.debug(f'sd_bus_widths: {sd_bus_widths:x}')
  log.debug(f'ver: {ver}')


def parse_r6(r: cmd_result):
  self.__log.debug(f'R6 response is: {r.resp0:08x}')
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


class SD:
  def __init__(self, sdhc: SDHCI, sdhost: SDHOST, use_sdhost):
    self.__sdhc = sdhc
    self.__sdhost = sdhost
    self.__state = CARD_STATE_UNKNOWN
    self.__use_sdhost = use_sdhost

  def init(self):
    if self.__use_sdhost:
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
