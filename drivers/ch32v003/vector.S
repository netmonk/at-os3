# ==============================================================================
# drivers/ch32v003/vector.S - CH32V003 Interrupt Vector Table (ACTIVE)
# ==============================================================================
#
# PURPOSE:
#   Active 256-entry interrupt vector table for CH32V003 hardware builds.
#   Routes all interrupts/exceptions to core handlers (trap_entry, irq_entry).
#   This is the vector table linked at address 0x00000000 for CH32V003 targets.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): All control flow originates from events
#   - Article 10 (Layer Roles): Drivers own hardware-specific vector tables
#   - PO-2 (Canonical Path Proof): Single canonical entry point for all events
#
# MEMORY LAYOUT:
#   Placed at address 0x00000000 in flash (see linker script)
#   Each entry is 4 bytes (one jump instruction)
#
# ==============================================================================

.section .vectors, "ax"         # Allocatable + executable section
.option norvc                   # Disable compressed instructions (each entry must be 4 bytes)
.align 2                        # Align to 4-byte boundary (required for mtvec)

.globl vector_base              # Export symbol for linker and mtvec setup

# ==============================================================================
# vector_base - Start of the 256-entry vector table
# ==============================================================================
#
# Entry 0:  Reset vector -> _start (system initialization)
# Entry 1-2: Reserved
# Entry 3:  Exception/trap -> trap_entry (synchronous exceptions)
# Entry 4-255: IRQ vectors -> irq_entry (asynchronous interrupts)
#
# bootstrap.S sets mtvec = (vector_base | 1), i.e. MODE0=1, MODE1=0 (RM 6.5.3.2):
#   MODE0=1 -> vectored: hardware entry = vector_base + (cause * 4)
#   MODE1=0 -> each table slot holds a jump instruction (the `j` entries below)
# So on interrupt/exception the core executes the `j` at vector_base + cause*4.
#
# ==============================================================================

vector_base:
    j _start                    # Entry 0: Reset - jump to startup code

    .rept 2                     # Entries 1-2: Reserved vectors
        j default_handler       # Jump to infinite loop (should never occur)
    .endr

    j trap_entry                # Entry 3: Exceptions (ecall, illegal insn, etc.)

    .rept (256 - 4)             # Entries 4-255: All IRQ vectors
        j irq_entry             # Route to generic IRQ dispatcher
    .endr

# ==============================================================================
# default_handler - Infinite loop for unhandled vectors
# ==============================================================================
#
# PURPOSE:
#   Catches any jump to reserved/unused vector entries.
#   Halts the system in a safe infinite loop for debugging.
#
# INPUTS:  None
# OUTPUTS: None (never returns)
#
# ==============================================================================

.globl default_handler
default_handler:
    j default_handler           # Infinite loop - system halted

.option rvc                     # Re-enable compressed instructions for rest of code
