# ==============================================================================
# core/timer.S - Tickless Software Timer Service (hardware-agnostic)
# ==============================================================================
#
# PURPOSE:
#   Provides software timers multiplexed onto a single hardware compare timer.
#   Implements tickless idle - no periodic interrupts when idle.
#   On timer expiry, enqueues EV_KIND_TIMER event with id=timer_id.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): Timer expiry generates events, not callbacks
#   - Article 8 (Service): Core service; owns no hardware, decides scheduling only
#   - Article 10 (Layer Roles): All SysTick hardware lives in drivers/<arch>/systick.S
#   - Article 12 (Architecture Isolation): No MMIO addresses or IRQ numbers here
#   - Article 13 (Timer Discipline): Absolute deadlines, no hidden reordering
#   - PO-4 (ISR Safety Proof): timer_tick is O(TIMER_MAX), bounded
#   - PO-7 (Temporal Coherence Proof): Deadlines are absolute, preserving causality
#
# HARDWARE ABI (provided by drivers/<arch>/systick.S):
#   systick_init()                 - Start the compare timer, register its ISR
#   systick_now()           -> a0  - Current monotonic count (32-bit)
#   systick_set_compare(a0)        - Arm compare at deadline + enable its IRQ
#   systick_disable_compare()      - Disable the compare IRQ (no timers armed)
#   The arch ISR calls back timer_tick(a0=now) on each compare match.
#
# API (to upper layers):
#   timer_arm_delta(tid, delta)    - Arm timer with relative deadline
#   timer_cancel(tid)              - Cancel armed timer
#   time_now()                     - Get current time (32-bit)
#
# ==============================================================================

.include "core/events.inc"

# ==============================================================================
# Constants
# ==============================================================================

.equ TIMER_MAX,         EV_TIMER_ID_MAX  # Number of software timers (from events.inc)
.equ TIMER_INVALID,     0xFFFFFFFF       # Marker for inactive timer

# ------------------------------------------------------------------------------
# Hardware ABI (drivers/<arch>/systick.S)
# ------------------------------------------------------------------------------
.extern systick_init
.extern systick_now
.extern systick_set_compare
.extern systick_disable_compare

# ==============================================================================
# BSS Section - Timer State
# ==============================================================================

.section .bss
.align 4

# ------------------------------------------------------------------------------
# timer_deadlines - Array of software timer deadlines
# ------------------------------------------------------------------------------
# Each entry: 32-bit absolute deadline (compare-timer count).
# Value of TIMER_INVALID (0xFFFFFFFF) means timer is not armed.
#
# Index = timer ID (0 to TIMER_MAX-1)

timer_deadlines:
    .space (TIMER_MAX * 4)          # Initialized to TIMER_INVALID at runtime

# ==============================================================================
# TEXT Section - Timer Functions
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# timer_init - Initialize Timer Subsystem
# ==============================================================================
#
# PURPOSE:
#   Initialize all software timers to inactive state and bring up the hardware
#   compare timer via the systick driver.
#
# INPUTS: None
# OUTPUTS: None
# REGISTERS: t0, t1, t2, a0, a1, ra
#
# ==============================================================================

.globl timer_init
timer_init:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Step 1: Initialize all timer slots to INVALID
    # --------------------------------------------------------------------------
    la   t0, timer_deadlines
    li   t1, TIMER_MAX
    li   t2, TIMER_INVALID
timer_init_loop:
    beqz t1, timer_init_hw
    sw   t2, 0(t0)
    addi t0, t0, 4
    addi t1, t1, -1
    j    timer_init_loop

timer_init_hw:
    # --------------------------------------------------------------------------
    # Step 2: Bring up the hardware compare timer (driver owns the registers)
    # --------------------------------------------------------------------------
    call systick_init

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# time_now - Get Current Time (32-bit)
# ==============================================================================
#
# PURPOSE:
#   Read the current monotonic count from the hardware compare timer.
#
# INPUTS: None
# OUTPUTS: a0 = current time (32-bit)
#
# ==============================================================================

.globl time_now
time_now:
    j    systick_now                # tail call: returns count in a0

# ==============================================================================
# timer_arm_abs - Arm Timer with Absolute Deadline
# ==============================================================================
#
# PURPOSE:
#   Schedule a timer to expire at an absolute time.
#   When it expires, EV_KIND_TIMER event with id=tid will be enqueued.
#
# INPUTS:
#   a0 - tid (timer ID, 0 to TIMER_MAX-1)
#   a1 - deadline_abs (absolute compare-timer count)
#
# OUTPUTS:
#   a0 - 0 on success, -1 on invalid tid
#
# REGISTERS: t0, t1, ra
#
# ==============================================================================

# NOTE: timer_arm_abs is not .globl; it is only called via timer_arm_delta.
# Mark .globl here if direct absolute-deadline arming is needed externally.
timer_arm_abs:
    addi sp, sp, -4
    sw   ra, 0(sp)
    # --------------------------------------------------------------------------
    # Bounds check: tid < TIMER_MAX
    # --------------------------------------------------------------------------
    li   t0, TIMER_MAX
    bgeu a0, t0, timer_arm_fail

    # --------------------------------------------------------------------------
    # Store deadline in timer_deadlines[tid]
    # --------------------------------------------------------------------------
    la   t0, timer_deadlines
    slli t1, a0, 2                  # t1 = tid * 4
    add  t0, t0, t1                 # t0 = &timer_deadlines[tid]
    sw   a1, 0(t0)                  # Store deadline

    # --------------------------------------------------------------------------
    # Re-arm hardware compare to the earliest deadline (enables its IRQ)
    # --------------------------------------------------------------------------
    call timer_update_compare

    li   a0, 0
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

timer_arm_fail:
    li   a0, -1
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# timer_arm_delta - Arm Timer with Relative Deadline
# ==============================================================================
#
# PURPOSE:
#   Schedule a timer to expire after a delay from now.
#
# INPUTS:
#   a0 - tid (timer ID)
#   a1 - delta (ticks from now)
#
# OUTPUTS:
#   a0 - 0 on success, -1 on invalid tid
#
# ==============================================================================

.globl timer_arm_delta
timer_arm_delta:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   a0, 4(sp)                  # Save tid

    # Get current time
    call time_now
    mv   t0, a0                     # t0 = now

    # Calculate absolute deadline
    lw   a0, 4(sp)                  # Restore tid
    add  a1, t0, a1                 # a1 = now + delta (wraps naturally)

    lw   ra, 0(sp)
    addi sp, sp, 8

    # Tail call to timer_arm_abs
    j    timer_arm_abs

# ==============================================================================
# timer_cancel - Cancel Armed Timer
# ==============================================================================
#
# PURPOSE:
#   Cancel a previously armed timer.
#
# INPUTS:
#   a0 - tid (timer ID)
#
# OUTPUTS:
#   a0 - 0 on success, -1 on invalid tid
#
# ==============================================================================

.globl timer_cancel
timer_cancel:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # Bounds check: tid < TIMER_MAX
    li   t0, TIMER_MAX
    bgeu a0, t0, timer_cancel_fail

    # Mark timer slot inactive
    la   t0, timer_deadlines
    slli t1, a0, 2                  # t1 = tid * 4
    add  t0, t0, t1                 # t0 = &timer_deadlines[tid]
    li   t2, TIMER_INVALID
    sw   t2, 0(t0)

    # Refresh hardware compare / interrupt enable from remaining timers
    call timer_update_compare

    li   a0, 0
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

timer_cancel_fail:
    li   a0, -1
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# timer_update_compare - Re-arm Hardware Compare to Earliest Deadline
# ==============================================================================
#
# PURPOSE:
#   Scan all timers and arm the hardware compare at the earliest deadline.
#   If no timers are armed, disable the compare interrupt.
#
# INPUTS: None
# OUTPUTS: None
# REGISTERS: t0, t1, t2, a0, a1, a2, ra
#
# ==============================================================================

timer_update_compare:
    addi sp, sp, -4
    sw   ra, 0(sp)

    la   t0, timer_deadlines
    li   t1, TIMER_MAX
    li   a0, TIMER_INVALID          # a0 = earliest deadline found
    li   a2, 0                      # a2 = found flag (0 = none, 1 = have earliest)

find_earliest_loop:
    beqz t1, find_earliest_done
    lw   t2, 0(t0)                  # t2 = deadline[i]

    # Skip invalid timers
    li   a1, TIMER_INVALID
    beq  t2, a1, find_earliest_next

    # First valid timer becomes earliest
    beqz a2, found_first

    # Check if this deadline is earlier using signed 32-bit subtraction.
    # Assumption: no two deadlines are more than 2^31 ticks apart,
    # which makes signed difference valid for half-period wraparound.
    sub  a1, t2, a0                 # a1 = deadline - current_earliest
    bltz a1, found_earlier          # If negative, t2 is earlier

    j    find_earliest_next

found_first:
    mv   a0, t2
    li   a2, 1
    j    find_earliest_next

found_earlier:
    mv   a0, t2                     # Update earliest

find_earliest_next:
    addi t0, t0, 4
    addi t1, t1, -1
    j    find_earliest_loop

find_earliest_done:
    # a0 = earliest deadline (or TIMER_INVALID if none)

    # Check if any timers are armed
    li   t0, TIMER_INVALID
    beq  a0, t0, tuc_disable

    # Arm hardware compare at earliest deadline (driver also enables the IRQ)
    call systick_set_compare
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

tuc_disable:
    # No timers armed - disable the compare interrupt
    call systick_disable_compare
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# timer_tick - Service Expired Timers (called by the arch SysTick ISR)
# ==============================================================================
#
# PURPOSE:
#   Invoked by drivers/<arch>/systick.S on each compare match, with the captured
#   current count. Checks all timers, enqueues events for expired ones, then
#   re-arms the hardware compare for the next deadline.
#
# INPUTS:
#   a0 - now (current count, captured by the arch ISR)
#
# OUTPUTS: None
#
# PO-4 PROOF:
#   - Loop is bounded by TIMER_MAX (constant)
#   - Each iteration is O(1)
#   - Total time = O(TIMER_MAX) = O(8) = bounded
#
# ==============================================================================

.globl timer_tick
timer_tick:
    addi sp, sp, -20
    sw   ra, 0(sp)
    sw   s0, 4(sp)                  # s0 = current time (now)
    sw   s1, 8(sp)                  # s1 = timer index
    mv   s0, a0                     # s0 = now (captured by the arch ISR)

    # --------------------------------------------------------------------------
    # Scan timers for expired ones
    # --------------------------------------------------------------------------
    la   t0, timer_deadlines
    li   s1, 0                      # s1 = timer index

timer_tick_loop:
    li   t1, TIMER_MAX
    bgeu s1, t1, timer_tick_done

    # Load deadline for this timer
    slli t1, s1, 2
    add  t2, t0, t1                 # t2 = &timer_deadlines[s1]
    sw   t2, 12(sp)                 # Save pointer for later
    lw   a2, 0(t2)                  # a2 = deadline

    # Skip invalid timers
    li   t1, TIMER_INVALID
    beq  a2, t1, timer_tick_next

    # Check if expired: (now - deadline) >= 0 with wraparound
    # If deadline <= now (accounting for wrap), timer has expired
    sub  t1, s0, a2                 # t1 = now - deadline
    bltz t1, timer_tick_next        # If negative, not yet expired

    # --------------------------------------------------------------------------
    # Timer expired - enqueue event
    # --------------------------------------------------------------------------

    # Mark timer as inactive
    lw   t2, 12(sp)                 # Restore deadline pointer
    li   t1, TIMER_INVALID
    sw   t1, 0(t2)

    # Build event header: kind=TIMER, id=tid, flags=0
    li   a0, (EV_KIND_TIMER << EV_KIND_SHIFT)
    slli t1, s1, EV_ID_SHIFT
    or   a0, a0, t1                 # a0 = header

    # data0 = scheduled deadline
    mv   a1, a2                     # a1 = deadline (already in a2)

    # data1 = 0 (reserved)
    li   a2, 0

    # Save loop state
    sw   t0, 16(sp)                 # Save base pointer

    # Enqueue timer event
    call event_enqueue

    # Restore loop state
    lw   t0, 16(sp)

timer_tick_next:
    addi s1, s1, 1
    j    timer_tick_loop

timer_tick_done:
    # --------------------------------------------------------------------------
    # Re-arm hardware compare for the next deadline
    # --------------------------------------------------------------------------
    call timer_update_compare

    lw   ra, 0(sp)
    lw   s0, 4(sp)
    lw   s1, 8(sp)
    addi sp, sp, 20
    ret
