# ==============================================================================
# drivers/ch32v003/systick.S - SysTick Hardware Timer Driver
# ==============================================================================
#
# PURPOSE:
#   Owns the CH32V003 SysTick compare timer. Exposes a small, architecture-
#   neutral ABI so that core/timer.S (the portable software-timer service) holds
#   no hardware addresses or IRQ numbers.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 5 (ISR Discipline): systick_isr only clears the flag, captures the
#     current count, and calls back the core service. No scheduling logic here.
#   - Article 10 (Layer Roles): All SysTick MMIO and IRQ numbers live here.
#   - Article 12 (Architecture Isolation): keeps these SoC details out of core/.
#   - PO-4 (ISR Safety Proof): bounded, no allocation, no blocking.
#
# ABI EXPOSED TO core/timer.S:
#   systick_init()                 - Start the counter, register systick_isr
#   systick_now()           -> a0  - Current monotonic count (32-bit)
#   systick_set_compare(a0)        - Arm compare at deadline + enable compare IRQ
#   systick_disable_compare()      - Disable compare IRQ (no timers armed)
#
# CORE CALLBACK (provided by core/timer.S):
#   timer_tick(a0=now)             - Service expired timers on each compare match
#
# HARDWARE:
#   CH32V003 SysTick: 32-bit up counter at HCLK, compare generates IRQ 12.
#   CTLR: control (enable, interrupt enable, clock source)
#   SR:   status (compare match flag, RW0)
#   CNTL: current counter (read-only for us)
#   CMPLR: compare value (we write the next deadline)
#
# ==============================================================================

# ------------------------------------------------------------------------------
# CH32V003 SysTick Registers
# ------------------------------------------------------------------------------
.equ STK_CTLR,          0xE000F000       # SysTick control register
.equ STK_SR,            0xE000F004       # SysTick status register
.equ STK_CNTL,          0xE000F008       # Counter register (32-bit)
.equ STK_CMPLR,         0xE000F010       # Compare register (32-bit)

# STK_CTLR bits
.equ STK_CTLR_STE,      (1 << 0)         # Counter enable
.equ STK_CTLR_STIE,     (1 << 1)         # Interrupt enable
.equ STK_CTLR_STCLK,    (1 << 2)         # Clock source: 1=HCLK, 0=HCLK/8
.equ STK_CTLR_STRE,     (1 << 3)         # Auto-reload enable

# STK_SR bits
.equ STK_SR_CNTIF,      (1 << 0)         # Compare match flag (RW0)

# PFIC IRQ number (enable itself is centralized in drivers/ch32v003/bootstrap.S)
.equ IRQ_SYSTICK,       12               # SysTick IRQ number

.equ SYSTICK_NO_COMPARE, 0xFFFFFFFF      # Park compare out of range until armed

# ------------------------------------------------------------------------------
# External Declarations
# ------------------------------------------------------------------------------
.extern register_isr
.extern timer_tick                       # core/timer.S callback

# ==============================================================================
# TEXT Section
# ==============================================================================
.section .text
.align 2

# ==============================================================================
# systick_init - Configure SysTick and register its ISR
# ==============================================================================
#
# PURPOSE:
#   Start the counter in upcount mode at HCLK (interrupt disabled until a timer
#   is armed), clear any pending flag, park the compare out of range, and
#   register systick_isr with the core dispatcher.
#
# INPUTS: None
# OUTPUTS: None
# CALLS: register_isr (must save/restore ra)
#
# ==============================================================================

.globl systick_init
systick_init:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # Counter running, HCLK source; interrupt OFF until first arm
    li   t0, STK_CTLR
    li   t1, (STK_CTLR_STE | STK_CTLR_STCLK)
    sw   t1, 0(t0)

    # Clear any pending compare flag (CNTIF is RW0)
    li   t0, STK_SR
    sw   zero, 0(t0)

    # Park compare so no interrupt fires until armed
    li   t0, STK_CMPLR
    li   t1, SYSTICK_NO_COMPARE
    sw   t1, 0(t0)

    # Register SysTick ISR with the core dispatcher
    li   a0, IRQ_SYSTICK
    la   a1, systick_isr
    call register_isr

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# systick_now - Read current SysTick counter (32-bit)
# ==============================================================================
# INPUTS: None ; OUTPUTS: a0 = current count ; REGISTERS: t0
# ==============================================================================

.globl systick_now
systick_now:
    li   t0, STK_CNTL
    lw   a0, 0(t0)
    ret

# ==============================================================================
# systick_set_compare - Arm the compare at a deadline and enable its IRQ
# ==============================================================================
# INPUTS: a0 = absolute deadline (32-bit) ; REGISTERS: t0, t1
# ==============================================================================

.globl systick_set_compare
systick_set_compare:
    li   t0, STK_CMPLR
    sw   a0, 0(t0)                   # set compare to earliest deadline
    li   t0, STK_CTLR
    lw   t1, 0(t0)
    ori  t1, t1, STK_CTLR_STIE       # enable compare interrupt
    sw   t1, 0(t0)
    ret

# ==============================================================================
# systick_disable_compare - Disable the compare interrupt (no timers armed)
# ==============================================================================
# INPUTS: None ; REGISTERS: t0, t1, t2
# ==============================================================================

.globl systick_disable_compare
systick_disable_compare:
    li   t0, STK_CTLR
    lw   t1, 0(t0)
    li   t2, ~STK_CTLR_STIE
    and  t1, t1, t2
    sw   t1, 0(t0)
    ret

# ==============================================================================
# systick_isr - SysTick compare-match ISR (IRQ 12)
# ==============================================================================
#
# PURPOSE:
#   Clear the compare flag, capture the current count, and hand off to the core
#   service. All deadline scheduling lives in core/timer.S (timer_tick).
#
# INPUTS: a0 = IRQ number (12, passed by irq_entry; not consumed)
#
# ISR SAFETY PROOF (PO-4):
#   - Fixed instruction sequence + one bounded callback; no loops here.
#   - No allocation, no blocking.
#
# ==============================================================================

.globl systick_isr
systick_isr:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # Clear SysTick compare flag (CNTIF is RW0)
    li   t0, STK_SR
    sw   zero, 0(t0)

    # Capture current count and hand off to the portable timer service
    li   t0, STK_CNTL
    lw   a0, 0(t0)                   # a0 = now
    call timer_tick

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret
