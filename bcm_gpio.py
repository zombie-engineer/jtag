import struct
import time

# Constants for BCM GPIO base (Raspberry Pi 3B+)
PERIPHERAL_BASE = 0x3f000000
GPIO_BASE = PERIPHERAL_BASE + 0x200000
GPIO_LEN = 0xB4  # Length of mapped GPIO register block

# Register offsets (in bytes)
GPFSEL    = GPIO_BASE + 0x00
GPSET     = GPIO_BASE + 0x1c
GPCLR     = GPIO_BASE + 0x28
GPLEV     = GPIO_BASE + 0x34
GPPUD     = GPIO_BASE + 0x94
GPPUDCLK0 = GPIO_BASE + 0x98
GPPUDCLK1 = GPIO_BASE + 0x9c

PUP_PDN_CNTRL_REG = GPIO_BASE + 0xe4  # For Pi 4+, use appropriate offset
GPIO_PULL_NONE = 0
GPIO_PULL_DOWN = 1
GPIO_PULL_UP   = 2

class BCM_GPIO_REGS:
    def __init__(self, t):
        self.__t = t

    def gpfsel_read(self, index):
        return self.__t.mem_read32(GPFSEL + index * 4)

    def gpfsel_write(self, index, value):
        self.__t.mem_write32(GPFSEL + index * 4, value)

    def gpset_write(self, index, value):
        self.__t.mem_write32(GPSET + index * 4, value)

    def gpclr_write(self, index, value):
        self.__t.mem_write32(GPCLR + index * 4, value)

    def gplev_read(self, index):
        return self.__t.mem_read32(GPLEV + index * 4)

    def gppud_write(self, value):
        self.__t.mem_write32(GPPUD, value)

    def gppud_read(self):
        return self.__t.mem_read32(GPPUD)

    def gppudclk0_write(self, value):
        self.__t.mem_write32(GPPUDCLK0, value)

    def gppudclk1_write(self, value):
        self.__t.mem_write32(GPPUDCLK1, value)



class BCM_GPIO:
    def __init__(self, t):
        self.regs = BCM_GPIO_REGS(t)

    def pin_configure(self, pin, mode):
        """mode: 0=input, 1=output, 2-7=alt functions"""
        reg_index = pin // 10
        bit = (pin % 10) * 3
        reg = self.regs.gpfsel_read(reg_index)
        reg &= ~(0b111 << bit)
        reg |= ((mode & 0b111) << bit)
        self.regs.gpfsel_write(reg_index, reg)

    def pin_out_set(self, pin):
        reg_index = pin // 32
        bit = pin % 32
        self.regs.gpset_write(reg_index, 1 << bit)

    def pin_out_clear(self, pin):
        reg_index = pin // 32
        bit = pin % 32
        self.regs.gpclr_write(reg_index, 1 << bit)

    def pin_in_read(self, pin):
        reg_index = pin // 32
        bit = pin % 32
        value = self.regs.gplev_read(reg_index)
        return (value >> bit) & 0x1

    def pin_config_pull_up_down(self, pin, pull):
      self.regs.gppud_write(pull)
      time.sleep(0.0001)
      if pin < 32:
        self.regs.gppudclk0_write(1 << pin)
      else:
        self.regs.gppudclk1_write(1 << (pin - 32))
      time.sleep(0.0001)
      self.regs.gppud_write(GPIO_PULL_NONE)
      self.regs.gppudclk0_write(0)
      self.regs.gppudclk1_write(0)
