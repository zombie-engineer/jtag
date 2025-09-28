#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * 4-byte Aarch64 instruction that MOVes contents of register Xn to
 * DBGDTR_EL0, with 'n' - being register index from 0 to 30,
 * x0, x1, x2, ..., x30
 * DBGDTR_EL0 is a system register, accessible from software that is
 * mapped to external debug registers DBGDTRTX_EL0 and DBGDTRRX_EL0 like that:
 * DBGDTRTX_EL0[31:0] <- DBGDTR_EL0[31:0]
 * DBGDTRRX_EL0[31:0] <- DBGDTR_EL0[63:32]
 */
#define AARCH64_INSTR_MSR_DBGDTR_EL0(__r) (0xd5130400 | (__r & 0x1f))
#define AARCH64_INSTR_MRS_DBGDTR_EL0(__r) (0xd5330400 | (__r & 0x1f))
#define op0 19
#define op1 16
#define CRn 12
#define CRm 8
#define op2 5

#define MRSMSR_SYSREG(_op0, _op1, _CRn, _CRm, _op2) \
      (_op0 << op0) \
    | (_op1 << op1) \
    | (_CRn << CRn) \
    | (_CRm << CRm) \
    | (_op2 << op2) \

#define DBGDTR_EL0 MRSMSR_SYSREG(0b10, 0b011,      0, 0b0100,     0)
#define DLR_EL0    MRSMSR_SYSREG(0b11, 0b011, 0b0100, 0b0101, 0b001)
#define X0         0
#define CTR_EL0    MRSMSR_SYSREG(0b11, 0b011, 0b0000, 0b0000, 0b001)
#define CLIDR_EL1  MRSMSR_SYSREG(0b11, 0b001, 0b0000, 0b0000, 0b001)
#define CSSELR_EL1 MRSMSR_SYSREG(0b11, 0b010, 0b0000, 0b0000, 0b000)
#define VBAR_EL1   MRSMSR_SYSREG(0b11, 0b000, 0b1100, 0b0000, 0b000)
/* d538c000 */

#define AARCH64_I_MSR(__dstreg, __srcreg) \
  (0xd5100000 | __dstreg | (__srcreg & 0x1f))

#define AARCH64_I_MRS(__dstreg, __srcreg) \
  (0xd5300000 | __srcreg | (__dstreg & 0x1f))

/* 0xb8404401 */
#define AARCH64_LDR_POSTINC_32(__addrreg, __dstreg) \
  (0xb8400400 | ((4 & 0x1ff) << 12) | (__addrreg << 5) | __dstreg)

#define AARCH64_LDR_POSTINC_64(__addrreg, __dstreg) \
  (0xf8400400 | ((8 & 0x1ff) << 12) | (__addrreg << 5) | __dstreg)

#define AARCH64_BRK(__im) (0xd4200000 | ((__im & 0xffff) << 5))
#define AARCH64_HLT(__im) (0xd4400000 | ((__im & 0xffff) << 5))

#define OPCODE_DC_CIVAC_X0 0xd50b7e20
#define OPCODE_DSB_ISH     0xd5033b9f
#define OPCODE_IC_IVAU_X0  0xd50b7520
#define OPCODE_ISB         0xd5033fdf

#define MRS_X0_DLR_EL0 AARCH64_I_MRS(X0, DLR_EL0)
#define MOV_X0_SP         0x910003e0
#define MRS_X0_SCTLR_EL1  0xd5381000
#define MRS_X0_ESR_EL2    0xd53c5200
#define MRS_X0_FAR_EL2    0xd53c6000
#define MRS_X0_DISR_EL1   0xd538c120
#define MRS_X0_DSPSR_EL0  0xd53b4500
#define MRS_X0_FPSR_EL0   0xd53b4420
#define MRS_X0_FPCR_EL0   0xd53b4400
#define MRS_X0_CTR_EL0    AARCH64_I_MRS(X0, CTR_EL0)
#define MRS_X0_VBAR_EL1   AARCH64_I_MRS(X0, VBAR_EL1)
#define MRS_X0_CLIDR_EL1  AARCH64_I_MRS(X0, CLIDR_EL1)
#define MRS_X0_CSSELR_EL1 AARCH64_I_MRS(X0, CSSELR_EL1)

static inline bool aarch64_get_mrs_opcode(uint32_t *opcode, uint32_t reg_id)
{
  switch (reg_id) {
    case AARCH64_CORE_REG_PC        : *opcode = MRS_X0_DLR_EL0  ; break;
    case AARCH64_CORE_REG_SP        : *opcode = MOV_X0_SP       ; break;
    case AARCH64_CORE_REG_SCTLR_EL1 : *opcode = MRS_X0_SCTLR_EL1; break;
    case AARCH64_CORE_REG_ESR_EL2   : *opcode = MRS_X0_ESR_EL2  ; break;
    case AARCH64_CORE_REG_FAR_EL2   : *opcode = MRS_X0_FAR_EL2  ; break;
    case AARCH64_CORE_REG_DISR_EL1  : *opcode = MRS_X0_DISR_EL1 ; break;
    /* DSPSR_EL0 and CPSR are both PSTATE */
    case AARCH64_CORE_REG_DSPSR_EL0 : *opcode = MRS_X0_DSPSR_EL0; break;
    case AARCH64_CORE_REG_CPSR      : *opcode = MRS_X0_DSPSR_EL0; break;
    case AARCH64_CORE_REG_FPSR      : *opcode = MRS_X0_FPSR_EL0 ; break;
    case AARCH64_CORE_REG_FPCR      : *opcode = MRS_X0_FPCR_EL0 ; break;
    case AARCH64_CORE_REG_CTR_EL0   : *opcode = MRS_X0_CTR_EL0  ; break;
    case AARCH64_CORE_REG_VBAR_EL1  : *opcode = MRS_X0_VBAR_EL1 ; break;
    case AARCH64_CORE_REG_CLIDR_EL1 : *opcode = MRS_X0_CLIDR_EL1; break;
    case AARCH64_CORE_REG_CSSELR_EL1: *opcode = MRS_X0_CSSELR_EL1; break;
    default: return false;
  }
  return true;
}

