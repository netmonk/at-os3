# ==============================================================================
# drivers/ch32v003/bootstrap.S - System Bootstrap and Initialization
# ==============================================================================
#
# PURPOSE:
#   System entry point (_start) and initialization sequence.
#   Configures stack, clears BSS, sets up clock/mtvec/PFIC, calls init
#   functions, then transfers control to event_loop.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 10 (Layer Roles): Bootstrap is a distinct concern from the event
#     system. Hardware-specific init (PFIC, INTSYSCR, mtvec) belongs here, not
#     in the portable event pipeline.
#   - PO-2 (Canonical Path Proof): After bootstrap, all execution flows through event_loop
#
# REGISTER CONSTRAINTS (RV32E):
#   Only registers x0-x15 available: zero, ra, sp, gp, tp, t0-t2, s0-s1, a0-a5
#
# ==============================================================================

# ------------------------------------------------------------------------------
# External Declarations
# ------------------------------------------------------------------------------

.extern flash_latency_48mhz
.extern clock_init_48mhz
.extern vector_base
.extern driver_init
.extern console_init
.extern kernel_init
.extern event_loop

# ------------------------------------------------------------------------------
# PFIC System Control Register (SLEEPDEEP bit)
# ------------------------------------------------------------------------------
.equ PFIC_SCTLR, 0xE000ED10
.equ RCC_AHBPCENR, 0x40021014      # unused here; defined for reference only

# ==============================================================================
# TEXT Section - Startup
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# _start - System Entry Point
# ==============================================================================
#
# PURPOSE:
#   Initialize the system and enter the event loop.
#   Called from reset vector (vector_base entry 0).
#
# SEQUENCE:
#   1.  Initialize stack pointer
#   2.  Clear BSS section
#   3.  Flash latency + 48MHz clock init
#   4.  Configure interrupt vector table (mtvec)
#   5.  Call driver_init (peripheral setup, ISR registration)
#   6.  Call console_init (UART, must precede kernel_init)
#   7.  Call kernel_init (root table, timer, .kernel_init functions)
#   8.  Enable peripheral IRQs in PFIC (centralized)
#   9.  Enable HPE (hardware prologue/epilogue for IRQ save/restore)
#   10. Enable interrupts globally and enter event_loop (never returns)
#
# INPUTS:  None (called at reset)
# OUTPUTS: None (never returns)
#
# ==============================================================================

.globl _start
_start:
    # --------------------------------------------------------------------------
    # Disable interrupts during init
    # --------------------------------------------------------------------------
    csrci mstatus, 0x8

    # --------------------------------------------------------------------------
    # Step 1: Initialize stack pointer
    # --------------------------------------------------------------------------
    la   sp, _stack_top

    # --------------------------------------------------------------------------
    # Step 2: Clear BSS section
    # --------------------------------------------------------------------------
    la   t0, _bss_start
    la   t1, _bss_end

bss_clear_loop:
    bgeu t0, t1, bss_clear_done
    sw   zero, 0(t0)
    addi t0, t0, 4
    j    bss_clear_loop

bss_clear_done:
    # --------------------------------------------------------------------------
    # Step 3: Hardware initialization (flash, clock)
    # --------------------------------------------------------------------------
    call flash_latency_48mhz
    call clock_init_48mhz

    # --------------------------------------------------------------------------
    # Step 4: Configure mtvec for vectored interrupt mode
    # --------------------------------------------------------------------------
    la   t0, vector_base
    ori  t0, t0, 1                  # Set mode bits to 01 (vectored)
    csrw mtvec, t0

    # --------------------------------------------------------------------------
    # Step 5: Call driver initialization
    # --------------------------------------------------------------------------
    call driver_init

    # --------------------------------------------------------------------------
    # Step 6: Initialize console subsystem (BEFORE kernel_init)
    # --------------------------------------------------------------------------
    # Console init MUST be called before kernel_init because .kernel_init
    # functions (e.g. at_init) emit startup banners via console_putc.
    # This sets up UART hardware and registers the RX ISR.
    call console_init

    # --------------------------------------------------------------------------
    # Step 7: Call kernel initialization
    # --------------------------------------------------------------------------
    # This initializes root_table, timer subsystem, and runs .kernel_init
    call kernel_init

    # --------------------------------------------------------------------------
    # Step 8: Enable all peripheral IRQs in PFIC (centralized)
    # --------------------------------------------------------------------------
    # PFIC_IENR1/2 are write-only. Writing 1 enables, writing 0 has no effect.
    # We enable all IRQs here after all init functions have registered their ISRs.
    #
    # Interrupt mapping (from CH32V003 reference manual Table 6-1):
    #   IRQ 12 = SysTick  (timer)     -> PFIC_IENR1 bit 12
    #   IRQ 20 = EXTI7_0  (button)    -> PFIC_IENR1 bit 20
    #   IRQ 32 = USART1   (console)   -> PFIC_IENR2 bit 0
    #
    li   t0, 0xE000E100             # PFIC_IENR1
    li   t1, ((1 << 12) | (1 << 20))  # SysTick + EXTI
    sw   t1, 0(t0)

    li   t0, 0xE000E104             # PFIC_IENR2
    li   t1, (1 << 0)               # USART1 (interrupt 32 = bit 0 of IENR2)
    sw   t1, 0(t0)

    # --------------------------------------------------------------------------
    # Step 9: Enable Hardware Prologue/Epilogue (HPE)
    # --------------------------------------------------------------------------
    # INTSYSCR CSR (0x804) controls QingKe-specific interrupt features:
    #   bit 0: HWSTKEN - Hardware stack enable (auto save/restore regs on IRQ)
    #   bit 1: INESTEN - Interrupt nesting enable (2-level nesting)
    # With HPE enabled, hardware automatically saves/restores caller-saved
    # registers to RAM stack on IRQ entry/exit, reducing latency by ~17 cycles.
    li   t0, 0x3                    # HWSTKEN | INESTEN
    csrw 0x804, t0                  # Enable HPE in INTSYSCR

    # --------------------------------------------------------------------------
    # Step 10: Enable interrupts and enter event loop
    # --------------------------------------------------------------------------
    # Enable machine external interrupts (MEIE) and global interrupt enable (MIE)
    # CRITICAL: This MUST be the last step before event_loop - all subsystems
    # must be fully initialized before any IRQ can fire.
    li   t0, (1 << 11)              # MEIE bit (bit 11 in mie)
    csrs mie, t0                    # Enable external interrupts
    csrsi mstatus, 0x8              # Set MIE (global interrupt enable)
    j    event_loop
