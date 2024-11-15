define trst_lo
  printf "TRST_LO\n"
  p JTAG_RST_GPIO_Port.ODR &= ~JTAG_RST_Pin
end

define trst_hi
  printf "TRST_HI\n"
  p JTAG_RST_GPIO_Port.ODR |= JTAG_RST_Pin
end

define tck_lo
  printf "TCK_LO\n"
  p JTAG_TCK_GPIO_Port.ODR &= ~JTAG_TCK_Pin
end

define tck_hi
  printf "TCK_HI\n"
  p JTAG_TCK_GPIO_Port.ODR |= JTAG_TCK_Pin
end

define tms_lo
  printf "TMS_LO\n"
  p JTAG_TMS_GPIO_Port.ODR &= ~JTAG_TMS_Pin
end

define tms_hi
  printf "TMS_HI\n"
  p JTAG_TMS_GPIO_Port.ODR |= JTAG_TMS_Pin
end

define tdi_lo
  printf "TDI_LO\n"
  p JTAG_TDI_GPIO_Port.ODR &= ~JTAG_TDI_Pin
end

define tdi_hi
  printf "TDI_HI\n"
  p JTAG_TDI_GPIO_Port.ODR |= JTAG_TDI_Pin
end

define tdo_get
  set $__v = JTAG_TDO_GPIO_Port.IDR & JTAG_TDO_Pin
  printf "TDO %x\n", $__v
end

define rtck_get
  set $__v = JTAG_RTCK_GPIO_Port.IDR & JTAG_RTCK_Pin
  printf "RTCK: %08x\n", $__v
end


define jtag_clock_bit
  set $__tms = $arg0
  set $__tdi = $arg1
  if $__tms
    tms_hi
  else
    tms_lo
  end
  if $__tdi
    tdi_hi
  else
    tdi_lo
  end
  tck_hi
  rtck_get
  tdo_get
  tck_lo
  rtck_get
end

define jtag_clock_bits
  set $__v = $arg0
  set $__nr = $arg1
  while $__nr
    set $__nr = $__nr - 1
    jtag_clock_bit 0 $__v & 1
    set $__v = $__v >> 1
    printf "%08x\n", $__v
  end
end
