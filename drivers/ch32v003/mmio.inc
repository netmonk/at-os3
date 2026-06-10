# ==============================================================================
# drivers/ch32v003/mmio.inc - MMIO Helper Macros (RV32E-safe)
# ==============================================================================
#
# PURPOSE:
#   Provide safe read-modify-write macros for MMIO register access.
#   Abstracts common patterns while enforcing RV32E register constraints.
#
# CONSTRAINTS:
#   - RISC-V: ori/andi immediates are 12-bit signed only.
#     Any mask/value > 0x7FF MUST use: li + or/and.
#   - RV32E: Only x0..x15 exist (t0..t2, s0..s1, a0..a5 available).
#     DO NOT use t3..t6, a6/a7, s2..s11.
#
# REGISTER USAGE: t0, t1, t2 only (RV32E-safe)
# ==============================================================================

    .ifndef __MMIO_INC__
    .set __MMIO_INC__, 1

# --------------------------------------------------------------------------
# Basic RMW helpers
# --------------------------------------------------------------------------

# Read 32-bit MMIO reg into dst (dst is a register name)
.macro MMIO_RD32 dst, addr
    li   t0, \addr
    lw   \dst, 0(t0)
.endm

# Write 32-bit MMIO reg from src (src is a register name)
.macro MMIO_WR32 addr, src
    li   t0, \addr
    sw   \src, 0(t0)
.endm

# OR-in bits (RMW): reg = reg | mask
.macro MMIO_SET_BITS addr, mask
    li   t0, \addr
    lw   t1, 0(t0)
    li   t2, \mask
    or   t1, t1, t2
    sw   t1, 0(t0)
.endm

# Clear bits (RMW): reg = reg & ~mask
.macro MMIO_CLR_BITS addr, mask
    li   t0, \addr
    lw   t1, 0(t0)
    li   t2, \mask
    not  t2, t2
    and  t1, t1, t2
    sw   t1, 0(t0)
.endm

# Write a field: reg = (reg & ~field_mask) | field_value
.macro MMIO_WRITE_FIELD addr, field_mask, field_value
    li   t0, \addr
    lw   t1, 0(t0)

    li   t2, \field_mask
    not  t2, t2
    and  t1, t1, t2

    li   t2, \field_value
    or   t1, t1, t2

    sw   t1, 0(t0)
.endm

# --------------------------------------------------------------------------
# Polling helpers
# --------------------------------------------------------------------------

# Wait until (reg & mask) != 0  (bit(s) set)
.macro MMIO_WAIT_BIT_SET addr, mask, label
\label:
    li   t0, \addr
    lw   t1, 0(t0)
    li   t2, \mask
    and  t1, t1, t2               # t1 becomes 0/non-zero
    beqz t1, \label
.endm

# Wait until (reg & mask) == 0 (bit(s) cleared)
.macro MMIO_WAIT_BIT_CLR addr, mask, label
\label:
    li   t0, \addr
    lw   t1, 0(t0)
    li   t2, \mask
    and  t1, t1, t2
    bnez t1, \label
.endm

# Wait until (reg & field_mask) == field_value
.macro MMIO_WAIT_FIELD_EQ addr, field_mask, field_value, label
\label:
    li   t0, \addr
    lw   t1, 0(t0)
    li   t2, \field_mask
    and  t1, t1, t2
    li   t2, \field_value
    bne  t1, t2, \label
.endm

# --------------------------------------------------------------------------
# Optional ordering/barrier helpers
# --------------------------------------------------------------------------

.macro MMIO_FENCE
    fence iorw, iorw
.endm

    .endif # __MMIO_INC__
