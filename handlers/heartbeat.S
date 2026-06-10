# ==============================================================================
# handlers/heartbeat.S - Heartbeat Timer Handler
# ==============================================================================
#
# PURPOSE:
#   Periodic timer handler demonstrating the event registration pattern.
#   Rearms a timer and increments a counter on each tick.
#   Proves the timer and event dispatch system works.
#
# CATEGORY: Handler (NOT an FSM)
#   - Reacts to timer events with immediate action
#   - Has counter context but NO state transitions
#   - Same behavior regardless of history
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): Only responds to timer events
#   - Article 7 (Handler): Reacts to one event, no state transitions
#   - Article 9 (Structural Enforcement): Registers via register_activity ABI only
#
# DESIGN:
#   - Uses timer ID 0 for periodic heartbeat
#   - On each timer event: increment counter, rearm timer
#   - Counter visible in heartbeat_counter for debugging
#
# AUTO-REGISTRATION:
#   heartbeat_init_ptr placed in .kernel_init section.
#   kernel_init calls heartbeat_init during startup.
#
# ==============================================================================

.include "core/events.inc"

# ------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------

.equ HEARTBEAT_TIMER_ID,    0           # Timer ID we use
.ifndef HEARTBEAT_PERIOD
.equ HEARTBEAT_PERIOD,      48000000    # 1 second at 48MHz (CH32V003 default)
.endif
.equ HEARTBEAT_MAGIC,       0x48423030  # "HB00"


# ==============================================================================
# BSS Section - Handler Context
# ==============================================================================

.section .bss
.align 4

# ------------------------------------------------------------------------------
# heartbeat_counter - Handler context structure
# ------------------------------------------------------------------------------
# Offset 0: magic    - Signature "HB00"
# Offset 4: counter  - Number of heartbeats (timer events received)

heartbeat_counter:
    .word 0                         # magic
    .word 0                         # counter

# ==============================================================================
# KERNEL_INIT Section - Auto-Registration Pointer
# ==============================================================================

.section .kernel_init, "a"
.align 2
heartbeat_init_ptr:
    .word heartbeat_init

# ==============================================================================
# TEXT Section - Handler Code
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# heartbeat_init - Initialize and Register Heartbeat Handler
# ==============================================================================
#
# PURPOSE:
#   Called by kernel_init during startup.
#   Initializes context and registers handler.
#
# INPUTS: None
# OUTPUTS: None
#
# ==============================================================================

.globl heartbeat_init
heartbeat_init:
    addi sp, sp, -16
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Initialize context structure
    # --------------------------------------------------------------------------
    la   t0, heartbeat_counter
    li   t1, HEARTBEAT_MAGIC
    sw   t1, 0(t0)                  # magic = "HB00"
    sw   zero, 4(t0)                # counter = 0

    # --------------------------------------------------------------------------
    # Initialize LED via driver layer (Article 10 — Layer Roles)
    # --------------------------------------------------------------------------
    call led_init

    # --------------------------------------------------------------------------
    # Register handler for EV_KIND_TIMER, id=HEARTBEAT_TIMER_ID
    # --------------------------------------------------------------------------
    li   a0, EV_KIND_TIMER
    li   a1, HEARTBEAT_TIMER_ID
    la   a2, heartbeat_handler
    call register_activity

    # --------------------------------------------------------------------------
    # Arm initial timer
    # --------------------------------------------------------------------------
    li   a0, HEARTBEAT_TIMER_ID
    li   a1, HEARTBEAT_PERIOD
    call timer_arm_delta

    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ==============================================================================
# heartbeat_handler - Timer Event Handler
# ==============================================================================
#
# PURPOSE:
#   Called by kernel_dispatch when EV_KIND_TIMER, id=0 event arrives.
#   Increments counter and rearms timer.
#
# INPUTS:
#   a0 - header (kind:8 | id:8 | flags:16)
#   a1 - data0 (scheduled deadline)
#   a2 - data1 (reserved)
#
# OUTPUTS: None
#
# BEHAVIOR:
#   1. Increment heartbeat counter
#   2. Rearm timer for next period
#
# NOTE: This is a Handler (Article 7), not an FSM.
#       It has counter context but no state transitions — same action every tick.
#
# ==============================================================================

heartbeat_handler:
    addi sp, sp, -16
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Increment counter
    # --------------------------------------------------------------------------
    la   t0, heartbeat_counter
    lw   t1, 4(t0)                  # t1 = counter
    addi t1, t1, 1
    sw   t1, 4(t0)                  # counter++

    # --------------------------------------------------------------------------
    # Toggle LED via driver layer (Article 10 — Layer Roles)
    # --------------------------------------------------------------------------
    call led_toggle

    # --------------------------------------------------------------------------
    # Rearm timer for next heartbeat
    # --------------------------------------------------------------------------
    li   a0, HEARTBEAT_TIMER_ID
    li   a1, HEARTBEAT_PERIOD
    call timer_arm_delta

    lw   ra, 0(sp)
    addi sp, sp, 16
    ret
