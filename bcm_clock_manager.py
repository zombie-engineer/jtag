
PERIPHERAL_BASE = 0x3f000000
BASEADDR = PERIPHERAL_BASE + 0x101000

REG_WRITE_PASSWORD = 0x5a000000

CLOCK_ID_GP0    = 0
CLOCK_ID_GP1    = 1
CLOCK_ID_GP2    = 2
CLOCK_ID_H264   = 3
CLOCK_ID_ISP    = 4
CLOCK_ID_V3D    = 5
CLOCK_ID_UART   = 6
CLOCK_ID_PWM    = 7
CLOCK_ID_SDHOST = 8
CLOCK_ID_EMMC   = 9
CLOCK_ID_PIXEL  = 10
CLOCK_ID_VEC    = 11
CLOCK_ID_HDMI   = 12
CLOCK_ID_DPI    = 13
CLOCK_ID_VPU    = 14

clock_reg_offsets = {
  CLOCK_ID_VPU:    0x08,
  CLOCK_ID_GP0:    0x70,
  CLOCK_ID_GP1:    0x78,
  CLOCK_ID_GP2:    0x80,
  CLOCK_ID_H264:   0x88,
  CLOCK_ID_ISP:    0x90,
  CLOCK_ID_V3D:    0x98,
  CLOCK_ID_UART:   0xA0,
  CLOCK_ID_PWM:    0xA8,
  CLOCK_ID_SDHOST: 0xB0,
  CLOCK_ID_EMMC:   0xB8,
  CLOCK_ID_PIXEL:  0xC0,
  CLOCK_ID_VEC:    0xC8,
  CLOCK_ID_HDMI:   0xD0,
  CLOCK_ID_DPI:    0xD8,
}

clock_names = {
  CLOCK_ID_VPU:    "vpu",
  CLOCK_ID_GP0:    "gp0",
  CLOCK_ID_GP1:    "gp1",
  CLOCK_ID_GP2:    "gp2",
  CLOCK_ID_H264:   "h264",
  CLOCK_ID_ISP:    "isp",
  CLOCK_ID_V3D:    "v3d",
  CLOCK_ID_UART:   "pl011",
  CLOCK_ID_PWM:    "pwm",
  CLOCK_ID_SDHOST: "sdhost",
  CLOCK_ID_EMMC:   "emmc",
  CLOCK_ID_PIXEL:  "pixel",
  CLOCK_ID_VEC:    "video_encoder",
  CLOCK_ID_HDMI:   "hdmi_core",
  CLOCK_ID_DPI:    "dpi",
}

name_to_id = {
  "vpu":           CLOCK_ID_VPU,
  "gp0":           CLOCK_ID_GP0,
  "gp1":           CLOCK_ID_GP1,
  "gp2":           CLOCK_ID_GP2,
  "h264":          CLOCK_ID_H264,
  "isp":           CLOCK_ID_ISP,
  "v3d":           CLOCK_ID_V3D,
  "pl011":         CLOCK_ID_UART,
  "pwm":           CLOCK_ID_PWM,
  "sdhost":        CLOCK_ID_SDHOST,
  "emmc":          CLOCK_ID_EMMC,
  "pixel":         CLOCK_ID_PIXEL,
  "video_encoder": CLOCK_ID_VEC,
  "hdmi_core":     CLOCK_ID_HDMI,
  "dpi":           CLOCK_ID_DPI,
}


class BCMClockManagerRegs:
  def __init__(self, t, log):
    self.__t = t
    self.log = log

  def _resolve_clock_id(self, clock_id):
    """Convert a string tag or integer clock_id to a valid offset."""
    if isinstance(clock_id, str):
      clock_id = name_to_id[clock_id]
    return clock_reg_offsets[clock_id]

  def cm_ctl_read(self, clock_id):
    reg_addr = BASEADDR + self._resolve_clock_id(clock_id)
    ret = self.__t.mem_read32(reg_addr)
    # print(f'cm_ctl_read: 0x{reg_addr:08x}: 0x{ret:08x}')
    return ret

  def cm_div_read(self, clock_id):
    reg_addr = BASEADDR + self._resolve_clock_id(clock_id) + 4
    ret = self.__t.mem_read32(reg_addr)
    # print(f'cm_div_read: 0x{reg_addr:08x}: 0x{ret:08x}')
    return ret


class BCMClockManager:
  def __init__(self, t, log):
    self.__t = t
    self.log = log
    self.regs = BCMClockManagerRegs(t, log)



