#pragma once
#include "adiv5.h"
#include <stdint.h>

#define CTI_EVENT_HALT 0
#define CTI_EVENT_RESUME 1

void cti_init(struct adiv5_dap *d, uint32_t cti_base);

void cti_pulse_event(struct adiv5_dap *d, uint32_t cti_base, int event);

void cti_gate_channel(struct adiv5_dap *d, uint32_t cti_base, int channel);

void cti_ungate_channel(struct adiv5_dap *d, uint32_t cti_base, int channel);

void cti_ack(struct adiv5_dap *d, uint32_t cti_base);
void cti_ack2(struct adiv5_dap *d, uint32_t cti_base, uint32_t event);
