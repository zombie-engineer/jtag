#pragma once
#include <stdint.h>
#include <stdbool.h>

#define EDPRSR_PU     0
#define EDPRSR_SPD    1
#define EDPRSR_R      2
#define EDPRSR_SR     3
#define EDPRSR_HALTED 4
#define EDPRSR_OSLK   5
#define EDPRSR_DLK    6
#define EDPRSR_EDAD   7
#define EDPRSR_SDAD   8
#define EDPRSR_EPMAD  9
#define EDPRSR_SPMAD  10
#define EDPRSR_SDR    11
#define EDPRSR_ETAD   12
#define EDPRSR_STAD   13
#define EDPRSR_EDADE  14
#define EDPRSR_ETADE  15
#define EDPRSR_EPMADE 16

static inline bool aarch64_edprsr_is_halted(uint32_t edprsr)
{
  return (edprsr >> EDPRSR_HALTED) & 1;
}
