#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "jtag_state.h"

int jtag_scan_num_devs(void);

uint32_t jtag_read_idcode(void);

bool jtag_to_state(jtag_state_t to);

void jtag_write_ir(uint8_t ir);

void jtag_shift(uint64_t value, uint64_t *out_value, size_t num_bits);

void jtag_init(void);

void jtag_reset(void);

void jtag_soft_reset(void);

