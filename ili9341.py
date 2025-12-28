# horizontal refresh direction
ILI9341_CMD_MEM_ACCESS_CONTROL_MH = 1<<2
# pixel format default is bgr, with this flag it's rgb 
ILI9341_CMD_MEM_ACCESS_CONTROL_BGR = 1<<3
# horizontal refresh direction
ILI9341_CMD_MEM_ACCESS_CONTROL_ML = 1<<4
# swap rows and columns default is PORTRAIT, this flags makes it ALBUM 
ILI9341_CMD_MEM_ACCESS_CONTROL_MV = 1<<5
# column address order 
ILI9341_CMD_MEM_ACCESS_CONTROL_MX = 1<<6
# row address order
ILI9341_CMD_MEM_ACCESS_CONTROL_MY = 1<<7

def byte_extract(a, idx):
  return (a >> (idx * 8)) & 0xff

def ili9341_caset_paset_make_arg(a0, a1):
  return [
    byte_extract(a0, 1),
    byte_extract(a0, 0),
    byte_extract(a1, 1),
    byte_extract(a1, 0)
  ]

def ili9341_caset_get_arg(x0, x1):
  return ili9341_caset_paset_make_arg(x0, x1)

def ili9341_paset_get_arg(y0, y1):
  return ili9341_caset_paset_make_arg(y0, y1)

ILI9341_CMD_SOFT_RESET            = 0x01
ILI9341_CMD_READ_ID               = 0x04
ILI9341_CMD_SLEEP_OUT             = 0x11
ILI9341_CMD_DISPLAY_OFF           = 0x28
ILI9341_CMD_DISPLAY_ON            = 0x29
ILI9341_CMD_CASET                 = 0x2a
ILI9341_CMD_PASET                 = 0x2b
ILI9341_CMD_RAMWR                 = 0x2c
ILI9341_CMD_MEM_ACCESS_CONTROL    = 0x36
ILI9341_CMD_COLMOD                = 0x3a
ILI9341_CMD_WRITE_MEMORY_CONTINUE = 0x3c
ILI9341_CMD_POWER_CTL_A           = 0xcb
ILI9341_CMD_POWER_CTL_B           = 0xcf
ILI9341_CMD_TIMING_CTL_A          = 0xe8
ILI9341_CMD_TIMING_CTL_B          = 0xea
ILI9341_CMD_POWER_ON_SEQ          = 0xed
ILI9341_CMD_PUMP_RATIO            = 0xf7
ILI9341_CMD_POWER_CTL_1           = 0xc0
ILI9341_CMD_POWER_CTL_2           = 0xc1
ILI9341_CMD_VCOM_CTL_1            = 0xc5
ILI9341_CMD_VCOM_CTL_2            = 0xc7
ILI9341_CMD_FRAME_RATE_CTL        = 0xb1
ILI9341_CMD_BLANK_PORCH           = 0xb5
ILI9341_CMD_DISPL_FUNC            = 0xb6

# ILI9341 displays are able to update at any rate between 61Hz to up to
# 119Hz. Default at power on is 70Hz.
ILI9341_FRAMERATE_61_HZ = 0x1F
ILI9341_FRAMERATE_63_HZ = 0x1E
ILI9341_FRAMERATE_65_HZ = 0x1D
ILI9341_FRAMERATE_68_HZ = 0x1C
ILI9341_FRAMERATE_70_HZ = 0x1B
ILI9341_FRAMERATE_73_HZ = 0x1A
ILI9341_FRAMERATE_76_HZ = 0x19
ILI9341_FRAMERATE_79_HZ = 0x18
ILI9341_FRAMERATE_83_HZ = 0x17
ILI9341_FRAMERATE_86_HZ = 0x16
ILI9341_FRAMERATE_90_HZ = 0x15
ILI9341_FRAMERATE_95_HZ = 0x14
ILI9341_FRAMERATE_100_HZ = 0x13
ILI9341_FRAMERATE_106_HZ = 0x12
ILI9341_FRAMERATE_112_HZ = 0x11
ILI9341_FRAMERATE_119_HZ = 0x10
ILI9341_UPDATE_FRAMERATE = ILI9341_FRAMERATE_119_HZ


class ILI9341:
  def __init__(self, spi, gpio, gpio_pin_blk, gpio_pin_dc, gpio_pin_reset):
    self.spi = spi
    self.gpio = gpio
    self.gpio_pin_blk = gpio_pin_blk
    self.gpio_pin_dc = gpio_pin_dc
    print('pin_dc', self.gpio_pin_dc)
    self.gpio_pin_reset = gpio_pin_reset

  def dc_clear(self):
    print('DC clear')
    self.gpio.pin_out_clear(self.gpio_pin_dc)

  def dc_set(self):
    print('DC set')
    self.gpio.pin_out_set(self.gpio_pin_dc)

  def write_cmd(self, cmd):
    self.dc_clear()
    self.spi.transfer([cmd])
    self.dc_set()

  def write_data(self, data):
    self.spi.transfer(data)

