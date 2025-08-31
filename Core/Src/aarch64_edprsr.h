#pragma once
#include <stdint.h>
#include <stdbool.h>

#define AARCH64_EDPRSR_BIT_PU     0
#define AARCH64_EDPRSR_BIT_SPD    1
#define AARCH64_EDPRSR_BIT_R      2
#define AARCH64_EDPRSR_BIT_SR     3
#define AARCH64_EDPRSR_BIT_HALTED 4
#define AARCH64_EDPRSR_BIT_OSLK   5
#define AARCH64_EDPRSR_BIT_DLK    6
#define AARCH64_EDPRSR_BIT_EDAD   7
#define AARCH64_EDPRSR_BIT_SDAD   8
#define AARCH64_EDPRSR_BIT_EPMAD  9
#define AARCH64_EDPRSR_BIT_SPMAD  10
#define AARCH64_EDPRSR_BIT_SDR    11
#define AARCH64_EDPRSR_BIT_ETAD   12
#define AARCH64_EDPRSR_BIT_STAD   13
#define AARCH64_EDPRSR_BIT_EDADE  14
#define AARCH64_EDPRSR_BIT_ETADE  15
#define AARCH64_EDPRSR_BIT_EPMADE 16

static inline bool aarch64_edprsr_is_halted(uint32_t edprsr)
{
  return (edprsr >> AARCH64_EDPRSR_BIT_HALTED) & 1;
}
