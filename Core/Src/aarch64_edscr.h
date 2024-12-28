#pragma once
#include <stdint.h>
#include <stdbool.h>

#define EDSCR_STATUS 0
#define EDSCR_STATUS_MASK 0x3f
#define EDSCR_ERR         6
#define EDSCR_A           7
#define EDSCR_EL          8
#define EDSCR_EL_WIDTH    2

#define EDSCR_RW          10
/* Halt event */
#define EDSCR_HDE         14
#define EDSCR_NSE         15
#define EDSCR_SDD         16
#define EDSCR_NS          18
#define EDSCR_SC2         19
#define EDSCR_MA          20
#define EDSCR_TDA         21
#define EDSCR_INTdis      22
#define EDSCR_ITE         24
#define EDSCR_PIPE_ADV    25
#define EDSCR_TXU         26
#define EDSCR_RXO         27
#define EDSCR_ITO         28
#define EDSCR_TX_FULL     29
#define EDSCR_RX_FULL     30
#define EDSCR_TFO         31

#define EDSCR_STATUS_PE_RESTARTING        0b000001
#define EDSCR_STATUS_PE_NON_DEBUG         0b000010
#define EDSCR_STATUS_BREAKPOINT           0b000111
#define EDSCR_STATUS_EXT_DEBUG_REQ        0b010011
#define EDSCR_STATUS_HLT_STEP_NORM        0b011011
#define EDSCR_STATUS_HLT_STEP_EXCL        0b011111
#define EDSCR_STATUS_OS_UNLK_CATCH        0b100011
#define EDSCR_STATUS_RST_CATCH            0b100111
#define EDSCR_STATUS_WATCHPOINT           0b101011
#define EDSCR_STATUS_HLT_INSTR            0b101111
#define EDSCR_STATUS_SW_DBG_ACCESS        0b110011
#define EDSCR_STATUS_EXCEPT_CATCH         0b110111
#define EDSCR_STATUS_HLT_STEP_NO_SYNDROME 0b111011

static inline int aarch64_edscr_get_status(uint32_t edscr)
{
  return edscr & EDSCR_STATUS_MASK;
}

static inline bool aarch64_edscr_is_error(uint32_t edscr)
{
  return (edscr >> EDSCR_ERR) & 1;
}

static inline bool aarch64_edscr_is_tx_full(uint32_t edscr)
{
  return (edscr >> EDSCR_TX_FULL) & 1;
}

static inline int aarch64_edscr_get_el(uint32_t edscr)
{
  return (edscr >> EDSCR_EL) & ((1 << EDSCR_EL_WIDTH) - 1);
}
