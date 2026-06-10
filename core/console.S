# ==============================================================================
# core/console.S - Console Output System Service
# ==============================================================================
#
# PURPOSE:
#   Provides the console output ABI for all kernel and sub-FSM code.
#   Wraps hardware-specific UART primitives in a layer-compliant interface.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 8 (Service): Provides infrastructure capability, decides nothing
#   - Article 10 (Layer Roles): Core service wraps driver primitives
#   - Article 11 (ABI Sanctity): Explicit interface for console output
#   - PO-5 (Layer Integrity Proof): subfsm/ calls only this, never drivers/
#
# EXPORTED FUNCTIONS:
#   console_init        - Initialize console UART service
#   console_putc        - Blocking single char (init/banner only)
#
# ==============================================================================

# ------------------------------------------------------------------------------
# External declarations
# ------------------------------------------------------------------------------

.extern uart_hw_init
.extern uart_hw_putc_poll

# ==============================================================================
# TEXT Section - Console Functions
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# console_init - Initialize Console Subsystem
# ==============================================================================
#
# PURPOSE:
#   Initialize the console by calling the hardware UART init.
#   Called by drivers/ch32v003/init.S during startup.
#
# INPUTS: None
# OUTPUTS: None
#
# CALLS: uart_hw_init (must save/restore ra)
#
# ==============================================================================

.globl console_init
console_init:
    # --------------------------------------------------------------------------
    # Save RA (we make multiple calls)
    # --------------------------------------------------------------------------
    addi sp, sp, -4
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Initialize hardware UART
    # --------------------------------------------------------------------------
    call uart_hw_init

    # --------------------------------------------------------------------------
    # Restore RA and return
    # --------------------------------------------------------------------------
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# console_putc - Output Single Character (BLOCKING)
# ==============================================================================
#
# PURPOSE:
#   Output a single character to the console.
#   Wraps uart_hw_putc_poll.
#
# WARNING: BLOCKING FUNCTION
#   This function loops until TX is ready. DO NOT call from FSM handlers.
#   Use ONLY in: console_init, startup banners, and radio host TX paths.
#
# INPUTS:
#   a0 - character to output (low 8 bits)
#
# OUTPUTS:
#   None
#
# CALLS: uart_hw_putc_poll (must save/restore ra)
#
# ==============================================================================

.globl console_putc
console_putc:
    # --------------------------------------------------------------------------
    # Save RA (we call uart_hw_putc_poll)
    # --------------------------------------------------------------------------
    addi sp, sp, -4
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Call hardware putc (a0 already has the character)
    # --------------------------------------------------------------------------
    call uart_hw_putc_poll

    # --------------------------------------------------------------------------
    # Restore RA and return
    # --------------------------------------------------------------------------
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

