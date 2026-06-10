# ==============================================================================
# core/trap.S - Exception and IRQ Entry Points
# ==============================================================================
#
# PURPOSE:
#   Provides the canonical entry points for all exceptions and interrupts.
#   Dispatches IRQs to registered driver handlers via a function pointer table.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): Single event pipeline for all interrupts
#   - Article 5 (ISR Discipline): Minimal ISR context, dispatch only
#   - Article 9 (Structural Enforcement): All IRQs must pass through irq_entry
#   - PO-4 (ISR Safety Proof): Bounded execution, no allocation
#
# ==============================================================================

# ------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------

.equ MAX_IRQ, 64                # Maximum number of IRQ handlers supported
.equ MCAUSE_IRQ_BIT, 31         # Bit 31 of mcause indicates interrupt (vs exception)

# ==============================================================================
# BSS Section - Trap State Storage
# ==============================================================================

.section .bss
.align 4

# ------------------------------------------------------------------------------
# trap_snapshot_* - Exception diagnostic registers
# ------------------------------------------------------------------------------
# PURPOSE: Store exception state for debugging/handling
# Written by trap_entry, read by exception handlers

trap_snapshot_mcause:  .word 0  # Exception cause code
trap_snapshot_mepc:    .word 0  # Program counter at exception
trap_snapshot_mtval:   .word 0  # Trap value (bad address, instruction, etc.)

# ------------------------------------------------------------------------------
# isr_table - IRQ handler dispatch table
# ------------------------------------------------------------------------------
# PURPOSE: Array of function pointers indexed by IRQ number
# Drivers register their ISR here during init
# Entry format: 4-byte pointer to ISR function (0 = no handler)
# Not .globl so that nobody can write directly and break Article 9 (Non-Bypassability) / PO-6

isr_table:
    .rept MAX_IRQ
        .word 0                 # Initialize all entries to NULL
    .endr

# ==============================================================================
# TEXT Section - Handler Code
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# trap_entry - Synchronous Exception Handler
# ==============================================================================
#
# PURPOSE:
#   Handles synchronous exceptions (ecall, illegal instruction, etc.)
#   Captures diagnostic state and advances PC past the faulting instruction.
#
# INPUTS:
#   - mcause: Exception cause code
#   - mepc:   Faulting instruction address
#   - mtval:  Additional trap information
#
# OUTPUTS:
#   - trap_snapshot_*: Saved diagnostic state
#   - mepc: Advanced by 4 to skip ecall instruction
#
# NOTE:
#   mepc += 4 assumes a 32-bit instruction (e.g. ecall).
#   CH32V003 supports the C extension; a 16-bit faulting instruction
#   would require +2 instead. This handler is only expected for ecall.
#
# REGISTERS USED:
#   t0, t1, t2, a0 (caller-saved, safe to clobber in trap context)
#
# ==============================================================================

.globl trap_entry
trap_entry:
    csrr t0, mcause             # t0 = mcause: read exception cause code
    csrr t1, mepc               # t1 = mepc: read faulting PC address
    csrr t2, mtval              # t2 = mtval: read trap value (bad addr/insn)

    la   a0, trap_snapshot_mcause  # a0 = address of snapshot storage
    sw   t0, 0(a0)              # Store mcause at offset 0
    sw   t1, 4(a0)              # Store mepc at offset 4
    sw   t2, 8(a0)              # Store mtval at offset 8

    addi t1, t1, 4              # t1 = mepc + 4: advance past ecall instruction
    csrw mepc, t1               # Write new return address to mepc

    mret                        # Return from machine mode exception

# ==============================================================================
# irq_entry - Asynchronous Interrupt Handler (IRQ Dispatcher)
# ==============================================================================
#
# PURPOSE:
#   Generic entry point for all hardware interrupts.
#   Reads mcause to determine IRQ number, looks up handler in isr_table,
#   and dispatches to the registered driver ISR.
#
# INPUTS:
#   - mcause: IRQ number in bits [30:0], bit 31 set for interrupt
#
# OUTPUTS:
#   - Calls registered ISR if present
#   - Returns via mret
#
# REGISTERS:
#   - Saves/restores ra, t0-t2, a0-a5 (10 registers total)
#   - Context: Minimal set required for ISR dispatch and event_enqueue calls
#   - Stack frame: 40 bytes (10 words × 4 bytes)
#
# CONSTITUTIONAL NOTES:
#   - PO-4 (ISR Safety Proof): Execution is bounded (single table lookup + ISR call)
#   - Article 5 (ISR Discipline): Only dispatches, no protocol logic here
#
# ==============================================================================

.globl irq_entry
irq_entry:
    # --------------------------------------------------------------------------
    # HPE (Hardware Prologue/Epilogue) handles register save/restore
    # --------------------------------------------------------------------------
    # With HWSTKEN enabled in INTSYSCR (CSR 0x804), the QingKe core
    # automatically saves caller-saved registers (ra, t0-t2, a0-a5; RV32E has no
    # a6/a7) to the RAM stack on IRQ entry and restores them on mret.
    #
    # IMPORTANT: HPE must be explicitly enabled during MCU init (see init.S).
    # It is NOT active by default; this handler assumes it has been configured.
    #
    # This eliminates ~20 cycles of manual save/restore overhead.
    # Stack usage is identical (hardware uses SP), just faster.
    # --------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    # Extract IRQ number from mcause
    # mcause format: bit[31]=1 for IRQ, bits[30:0]=IRQ number
    # --------------------------------------------------------------------------
    csrr t0, mcause             # t0 = mcause: read interrupt cause
    li   t1, 0x7FFFFFFF         # t1 = mask for lower 31 bits
    and  t0, t0, t1             # t0 = IRQ number (clear bit 31)

    # --------------------------------------------------------------------------
    # Bounds check: ensure IRQ number < MAX_IRQ
    # --------------------------------------------------------------------------
    li   t1, MAX_IRQ            # t1 = maximum valid IRQ number
    bgeu t0, t1, irq_no_handler # If IRQ >= MAX_IRQ, skip dispatch

    # --------------------------------------------------------------------------
    # Look up ISR in dispatch table
    # Address = isr_table + (IRQ_number * 4)
    # --------------------------------------------------------------------------
    la   t1, isr_table          # t1 = base address of ISR table
    slli t2, t0, 2              # t2 = IRQ * 4 (word offset)
    add  t1, t1, t2             # t1 = address of ISR pointer
    lw   t2, 0(t1)              # t2 = ISR function pointer

    # --------------------------------------------------------------------------
    # Check if handler is registered (non-NULL)
    # --------------------------------------------------------------------------
    beqz t2, irq_no_handler     # If ISR == NULL, skip call

    # --------------------------------------------------------------------------
    # Call the registered ISR
    # Pass IRQ number in a0 for handler reference
    # --------------------------------------------------------------------------
    mv   a0, t0                 # a0 = IRQ number (argument to ISR)
    jalr ra, t2, 0              # Call ISR: ra = return addr, jump to t2

irq_no_handler:
    # --------------------------------------------------------------------------
    # Return from interrupt - HPE restores registers automatically
    # --------------------------------------------------------------------------
    mret                        # Return from machine mode interrupt

# ==============================================================================
# register_isr - Register an ISR Handler
# ==============================================================================
#
# PURPOSE:
#   Allows drivers to register their interrupt service routines.
#   Called during driver initialization.
#
# INPUTS:
#   a0 - IRQ number (0 to MAX_IRQ-1)
#   a1 - Pointer to ISR handler function
#
# OUTPUTS:
#   None
#
# SIDE EFFECTS:
#   Writes handler pointer into isr_table[irq].
#
# ABI CONTRACT:
#   This is the only way for drivers to register interrupt handlers.
#   Direct modification of isr_table is forbidden
#   (Article 9 Non-Bypassability; Article 11 ABI Sanctity).
#
# ==============================================================================

.globl register_isr
register_isr:
    # --------------------------------------------------------------------------
    # Bounds check
    # --------------------------------------------------------------------------
    li   t0, MAX_IRQ            # t0 = maximum valid IRQ
    bgeu a0, t0, 1f             # If IRQ >= MAX_IRQ, return without registering

    # --------------------------------------------------------------------------
    # Calculate table entry address and store handler
    # --------------------------------------------------------------------------
    la   t0, isr_table          # t0 = base of ISR table
    slli t1, a0, 2              # t1 = IRQ * 4 (word offset)
    add  t0, t0, t1             # t0 = address of entry
    sw   a1, 0(t0)              # Store ISR pointer in table

1:
    ret                         # Return to caller
