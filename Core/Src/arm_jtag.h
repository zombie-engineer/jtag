#pragma once
#define JTAG_CYCLE_TCK() \
  TCLK_LO(); \
  HAL_Delay(1); \
  TCLK_HI(); \
  HAL_Delay(1);

// JTAG Set IR
//   ir:     IR value
//   return: none
static void JTAG_IR(uint32_t ir)
{
  uint32_t n;

  TMS_HI();
  JTAG_CYCLE_TCK();                         /* Select-DR-Scan */
  JTAG_CYCLE_TCK();                         /* Select-IR-Scan */
  PIN_TMS_CLR();
  JTAG_CYCLE_TCK();                         /* Capture-IR */
  JTAG_CYCLE_TCK();                         /* Shift-IR */

  PIN_TDI_OUT(1U);

  for (n = DAP_Data.jtag_dev.ir_before[DAP_Data.jtag_dev.index]; n; n--) {
    JTAG_CYCLE_TCK();                       /* Bypass before data */
  }

  for (n = DAP_Data.jtag_dev.ir_length[DAP_Data.jtag_dev.index] - 1U; n; n--) {
    JTAG_CYCLE_TDI(ir);                     /* Set IR bits (except last) */
    ir >>= 1;
  }

  n = DAP_Data.jtag_dev.ir_after[DAP_Data.jtag_dev.index];

  if (n) {
    JTAG_CYCLE_TDI(ir);                     /* Set last IR bit */
    PIN_TDI_OUT(1U);
    for (--n; n; n--) {
      JTAG_CYCLE_TCK();                     /* Bypass after data */
    }
    TMS_HI();
    JTAG_CYCLE_TCK();                       /* Bypass & Exit1-IR */
  } else {
    TMS_HI();
    JTAG_CYCLE_TDI(ir);                     /* Set last IR bit & Exit1-IR */
  }

  JTAG_CYCLE_TCK();                         /* Update-IR */
  PIN_TMS_CLR();
  JTAG_CYCLE_TCK();                         /* Idle */
  PIN_TDI_OUT(1U);
}
