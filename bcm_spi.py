import time
# Constants for BCM SPI base (Raspberry Pi 3B+)
PERIPHERAL_BASE = 0x3f000000
SPI_BASE = PERIPHERAL_BASE + 0x204000

# Register offsets (in bytes)
SPI_CS   = SPI_BASE + 0x00
SPI_FIFO = SPI_BASE + 0x04
SPI_CLK  = SPI_BASE + 0x08
SPI_DLEN = SPI_BASE + 0x0c
SPI_LTOH = SPI_BASE + 0x10
SPI_DC   = SPI_BASE + 0x14

SPI_CS_CS       = 3<<0
SPI_CS_CPHA     = 1<<2
SPI_CS_CPOL     = 1<<3
SPI_CS_CLEAR    = 3<<4
SPI_CS_CLEAR_TX = 1<<4
SPI_CS_CLEAR_RX = 1<<5
SPI_CS_CSPOL    = 1<<6
SPI_CS_TA       = 1<<7
SPI_CS_DMAEN    = 1<<8
SPI_CS_INTD     = 1<<9
SPI_CS_INTR     = 1<<10
SPI_CS_ADCS     = 1<<11
SPI_CS_REN      = 1<<12
SPI_CS_LEN      = 1<<13
SPI_CS_DONE     = 1<<16
SPI_CS_RXD      = 1<<17
SPI_CS_TXD      = 1<<18
SPI_CS_RXR      = 1<<19
SPI_CS_RXF      = 1<<20
SPI_CS_CSPOL0   = 1<<21
SPI_CS_CSPOL1   = 1<<22
SPI_CS_CSPOL2   = 1<<23
SPI_CS_DMA_LEN  = 1<<24
SPI_CS_LEN_LONG = 1<<25

spi_cs_bit_to_desc = [
  (SPI_CS_CS, "CS"),
  (SPI_CS_CPHA, "CPHA"),
  (SPI_CS_CPOL, "CPOL"),
  (SPI_CS_CLEAR, "CLEAR"),
  (SPI_CS_CSPOL, "CSPOL"),
  (SPI_CS_TA, "TA (transfer active)"),
  (SPI_CS_DMAEN, "DMAEN (DMA enabled)"),
  (SPI_CS_INTD, "INTD (interrupt on DONE"),
  (SPI_CS_INTR, "INTR (interrupt on RXR)"),
  (SPI_CS_ADCS, "ADCS (auto deassert CS"),
  (SPI_CS_REN, "REN (read enabled)"),
  (SPI_CS_LEN, "LEN (LoSSI)"),
  (SPI_CS_DONE, "DONE (transfer done)"),
  (SPI_CS_RXD, "RXD (RX fifo has data)"),
  (SPI_CS_TXD, "TXD (TX fifo accepts)"),
  (SPI_CS_RXR, "RXR (RX full)"),
  (SPI_CS_RXF, "RXF"),
  (SPI_CS_CSPOL0, "CSPOL0"),
  (SPI_CS_CSPOL1, "CSPOL1"),
  (SPI_CS_CSPOL2, "CSPOL2"),
  (SPI_CS_DMA_LEN, "DMA_LEN"),
  (SPI_CS_LEN_LONG, "LEN_LONG"),
]

def spi_cs_to_str(cs):
  result = ''
  already = False
  for flag, desc in spi_cs_bit_to_desc:
    if cs & flag:
      prefix = ', ' if already else ''
      result += f'{prefix}{desc}'
      already = True
  return result

class BCM_SPI_REGS:
  def __init__(self, t):
    self.__t = t

  def spi_cs_write(self, v):
    self.__t.mem_write32(SPI_CS, v)

  def spi_cs_read(self):
    return self.__t.mem_read32(SPI_CS)[0]

  def spi_fifo_write(self, v):
    print(v)
    self.__t.mem_write32(SPI_FIFO, v)

  def spi_fifo_read(self):
    return self.__t.mem_read32(SPI_FIFO)

  def spi_clk_write(self, v):
    self.__t.mem_write32(SPI_CLK, v)

  def spi_clk_read(self):
    return self.__t.mem_read32(SPI_CLK)

  def spi_dlen_write(self, v):
    self.__t.mem_write32(SPI_DLEN, v)

  def spi_dlen_read(self):
    return self.__t.mem_read32(SPI_DLEN)

class BCM_SPI:
  def __init__(self, t):
    self.regs = BCM_SPI_REGS(t)

  def reset(self):
    self.regs.spi_cs_write(0)

  def fifo_flush(self, flush_rx, flush_tx):
    cs = self.regs.spi_cs_read()
    if flush_rx:
      cs |= SPI_CS_CLEAR_RX
    if flush_tx:
      cs |= SPI_CS_CLEAR_TX
    self.regs.spi_cs_write(cs)

  def __transfer_one_read_rx(self):
    result = []
    cs = 0
    while cs & SPI_CS_RXD:
      rx = self.regs.spi_fifo_read()
      result.append(rx)
      cs = self.regs.spi_cs_read()
      print(f'spi::transfer: wait RXD, cs: {cs:08x}, rx:{rx:02x}')
    return result

  def __transfer_one_wait_txd(self):
    cs = 0
    while (cs & SPI_CS_TXD) == 0:
      cs = self.regs.spi_cs_read()
      print(f'spi::transfer: wait TXD: {cs:08x} {spi_cs_to_str(cs)}')

  def __transfer_one_wait_done(self):
    cs = 0
    while (cs & SPI_CS_DONE) == 0:
      cs = self.regs.spi_cs_read()
      print(f'wait DONE: {cs:08x} {spi_cs_to_str(cs)}')

  def __transfer_one(self, value):
    rx_result = self.__transfer_one_read_rx()
    self.__transfer_one_wait_txd()
    self.regs.spi_fifo_write(value)
    rx_result.append(self.regs.spi_fifo_read())
    self.__transfer_one_wait_done()
    print(f'spi::transfer: written "{value:02x}"')
    rx_result += self.__transfer_one_read_rx()
    return rx_result

  def transfer(self, data):
    result = []
    cs = self.regs.spi_cs_read()
    cs |= SPI_CS_TA
    self.regs.spi_cs_write(cs)

    for value in data:
      result.append(self.__transfer_one(value))

    print('spi write done', data, result)
    return result
