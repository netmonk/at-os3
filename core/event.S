# ==============================================================================
# core/event.S - Event Queue and Main Dispatch Loop
# ==============================================================================
#
# PURPOSE:
#   Implements the core event-driven execution model.
#   Provides the event queue and main event dispatch loop.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 2 (Event Time Model): Execution is exclusively event-driven
#   - Article 4 (Canonical Event Pipeline): All control originates from events
#   - PO-1 (Event Origin Proof): Every event has an explicit enqueue point
#   - PO-2 (Canonical Path Proof): Single canonical event pipeline
#   - PO-7 (Temporal Coherence Proof): FIFO ordering preserves causality
#
# EVENT FORMAT (3 words = 12 bytes):
#   w0: kind(8) | id(8) | flags(16)
#   w1: data0
#   w2: data1
#
# REGISTER CONSTRAINTS (RV32E):
#   Only registers x0-x15 available: zero, ra, sp, gp, tp, t0-t2, s0-s1, a0-a5
#
# ==============================================================================

# ------------------------------------------------------------------------------
# External Declarations
# ------------------------------------------------------------------------------

.extern kernel_dispatch

# ------------------------------------------------------------------------------
# Include Event Constants
# ------------------------------------------------------------------------------

.include "core/events.inc"

# ==============================================================================
# BSS Section - Event Queue Storage
# ==============================================================================

.section .bss
.align 4

# ------------------------------------------------------------------------------
# Event Queue Ring Buffer (3-word events)
# ------------------------------------------------------------------------------
# Structure: Circular buffer with head (write) and tail (read) pointers
# Each event is 12 bytes: [header:4][data0:4][data1:4]
# ISRs write at head, event_loop reads at tail

event_queue:
    .rept EVENT_QUEUE_SIZE
        .word 0                     # header (kind:8|id:8|flags:16)
        .word 0                     # data0
        .word 0                     # data1
    .endr

event_head:     .word 0             # Write index (modified by ISRs)

event_tail:     .word 0             # Read index (modified by event_loop only)

event_drop:     .word 0             # Dropped events counter

# ==============================================================================
# TEXT Section - Event Loop and Enqueue
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# event_loop - Main Event Dispatch Loop (Meta-FSM)
# ==============================================================================
#
# PURPOSE:
#   The core execution loop implementing the kernel meta-FSM.
#   Two states: RUN (process events) and IDLE (wait for interrupt).
#
# META-FSM STATES:
#   RUN:  Queue not empty -> dequeue event, dispatch via root_table
#   IDLE: Queue empty -> WFI (tickless - timer compare already set)
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 2 (Event Time Model): Meta-FSM advances only on events
#   - Article 4 (Canonical Event Pipeline): No execution without an event
#   - PO-3 (FSM Ownership Proof): Each event dispatched to exactly one handler
#
# ALGORITHM (Race-Condition-Safe):
#   1. Disable interrupts (csrci mstatus, 0x8)
#   2. Check if queue has events (head != tail)
#   3. If queue not empty: re-enable, dequeue, dispatch, goto 1
#   4. If queue empty: re-enable, WFI, goto 1
#
# REGISTERS USED: t0, t1, t2, a0, a1, a2, a3 (all caller-saved)
#
# ==============================================================================

.globl event_loop
event_loop:
    # ==========================================================================
    # META-FSM: Check queue state (critical section)
    # ==========================================================================
    csrci mstatus, 0x8              # Disable interrupts

    la   t0, event_head
    lw   t1, 0(t0)                  # t1 = head index
    la   a2, event_tail
    lw   t2, 0(a2)                  # t2 = tail index
    bne  t1, t2, event_dequeue      # If head != tail, have events (-> RUN)

    # ==========================================================================
    # META-FSM STATE: IDLE
    # ==========================================================================
    # Queue empty - enter low-power wait
    # Tickless: timer compare already armed by timer subsystem

event_wait:
    csrsi mstatus, 0x8              # Re-enable interrupts
    # Re-check queue after enabling interrupts to avoid lost wakeup
    la   t0, event_head
    lw   t1, 0(t0)
    la   t0, event_tail
    lw   t2, 0(t0)
    bne  t1, t2, event_loop         # Event arrived while IRQs were off
    wfi                             # Wait For Interrupt
    j    event_loop                 # After interrupt, check queue again

    # ==========================================================================
    # META-FSM STATE: RUN
    # ==========================================================================

event_dequeue:
    csrsi mstatus, 0x8              # Re-enable interrupts

    # --------------------------------------------------------------------------
    # Dequeue event from tail position
    # t2 = tail index, a2 = &event_tail
    # --------------------------------------------------------------------------
    la   a3, event_queue

    # Calculate slot address: base + tail * EVENT_SIZE (12)
    # tail * 12 = tail * 8 + tail * 4 = (tail << 3) + (tail << 2)
    slli t0, t2, 3                  # t0 = tail * 8
    slli t1, t2, 2                  # t1 = tail * 4
    add  t0, t0, t1                 # t0 = tail * 12
    add  a3, a3, t0                 # a3 = address of event at tail

    # Read 3-word event
    lw   a0, 0(a3)                  # a0 = header (kind|id|flags)
    lw   a1, 4(a3)                  # a1 = data0
    lw   t0, 8(a3)                  # t0 = data1 (save temporarily)

    # --------------------------------------------------------------------------
    # Advance tail pointer (circular)
    # --------------------------------------------------------------------------
    addi t2, t2, 1
    andi t2, t2, EVENT_QUEUE_MASK
    sw   t2, 0(a2)                  # Store new tail index

    # --------------------------------------------------------------------------
    # Clear dequeued slot (for debugging)
    # --------------------------------------------------------------------------
    sw   zero, 0(a3)
    sw   zero, 4(a3)
    sw   zero, 8(a3)

    # --------------------------------------------------------------------------
    # Skip zero-header events (defensive: slots are explicitly cleared on dequeue,
    # so a zero header should not occur under normal operation)
    # --------------------------------------------------------------------------
    beqz a0, event_loop             # If header == 0, skip

    # --------------------------------------------------------------------------
    # Dispatch to kernel
    # --------------------------------------------------------------------------
    mv   a2, t0                     # a2 = data1

    addi sp, sp, -4
    sw   ra, 0(sp)

    call kernel_dispatch            # Dispatch event to registered handler

    lw   ra, 0(sp)
    addi sp, sp, 4

    j    event_loop                 # Process next event

# ==============================================================================
# event_enqueue - Add 3-Word Event to Queue (Atomic, ISR-Safe)
# ==============================================================================
#
# PURPOSE:
#   Enqueues a 3-word event. The ONLY way to submit events to the kernel.
#   Must be bounded-time and allocation-free (Article 5, PO-4).
#
# INPUTS:
#   a0 - header (kind:8 | id:8 | flags:16)
#   a1 - data0
#   a2 - data1
#
# OUTPUTS:
#   a0 - 0 on success, -1 if queue full
#
# ISR SAFETY:
#   - No allocation
#   - Bounded execution (O(1))
#   - No blocking
#   - ATOMIC: IRQs disabled during critical section
#
# ATOMICITY:
#   This function is safe to call from BOTH ISR context AND main thread.
#   IRQs are disabled around head/tail manipulation to prevent races.
#   Previous MIE state is preserved and restored.
#
# ABI CONTRACT:
#   This is the ONLY way for ISRs to signal events to the main loop.
#   Direct queue manipulation is forbidden (Article 9 Structural Enforcement, PO-6).
#
# ==============================================================================

.globl event_enqueue
event_enqueue:
    # --------------------------------------------------------------------------
    # Save callee-saved register and disable interrupts atomically
    # --------------------------------------------------------------------------
    addi sp, sp, -4
    sw   s0, 0(sp)                  # Save s0 (callee-saved)
    csrrci s0, mstatus, 0x8         # s0 = old mstatus, clear MIE (IRQ off)

    # --------------------------------------------------------------------------
    # CRITICAL SECTION START - IRQs disabled
    # --------------------------------------------------------------------------

    # Read current head and tail
    la   a3, event_head
    lw   t0, 0(a3)                  # t0 = current head index
    la   a4, event_tail
    lw   t1, 0(a4)                  # t1 = current tail index

    # Calculate next head position
    addi t2, t0, 1
    andi t2, t2, EVENT_QUEUE_MASK   # Wrap around

    # Check if queue is full (next_head == tail)
    beq  t2, t1, enqueue_full

    # Write event at current head position
    la   a4, event_queue            # a4 reused here (was &event_tail, now event_queue base)
    # head * 12 = (head << 3) + (head << 2)
    slli a5, t0, 3                  # a5 = head * 8
    slli t1, t0, 2                  # t1 = head * 4  (reuse t1, tail no longer needed)
    add  a5, a5, t1                 # a5 = head * 12
    add  a4, a4, a5                 # a4 = address of slot

    sw   a0, 0(a4)                  # Store header
    sw   a1, 4(a4)                  # Store data0
    sw   a2, 8(a4)                  # Store data1

    # Advance head pointer
    sw   t2, 0(a3)                  # Store new head index

    # --------------------------------------------------------------------------
    # CRITICAL SECTION END - Restore IRQ state and return success
    # --------------------------------------------------------------------------
    csrw mstatus, s0                # Restore previous MIE state
    lw   s0, 0(sp)                  # Restore s0
    addi sp, sp, 4
    li   a0, 0
    ret

enqueue_full:
    # Track drop (inside critical section - safe from ISR races)
    la   t1, event_drop
    lw   t0, 0(t1)
    addi t0, t0, 1
    sw   t0, 0(t1)

    # --------------------------------------------------------------------------
    # CRITICAL SECTION END - Queue full path
    # --------------------------------------------------------------------------
    csrw mstatus, s0                # Restore previous MIE state
    lw   s0, 0(sp)                  # Restore s0
    addi sp, sp, 4
    li   a0, -1
    ret
