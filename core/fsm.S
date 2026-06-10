# ==============================================================================
# core/fsm.S - Table-Driven FSM Engine
# ==============================================================================
#
# PURPOSE:
#   Provides the canonical FSM execution engine for all true FSMs.
#   Implements table-driven dispatch: table[state][event] → (next_state, action)
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): All transitions triggered by events
#   - Article 6 (FSM): fsm_step is the canonical table-driven FSM executor
#   - Article 9 (Structural Enforcement): No direct state modification allowed
#   - Article 11 (ABI Sanctity): fsm_step is the ONLY state writer
#
# GUARANTEES:
#   1. ONE event → ONE step maximum
#   2. ONE step → AT MOST one transition
#   3. Action CANNOT modify state directly
#   4. next_state is determined BEFORE action executes
#   5. state is written AFTER action returns
#
# RULE: DECLARATIVE EFFECTS ONLY
#   Actions may call event_enqueue() to emit events.
#   These events are queued and processed in FUTURE steps.
#   NEVER immediate execution. Sequence:
#     1. action() → event_enqueue(EV_X)
#     2. action() returns
#     3. fsm_step writes next_state
#     4. fsm_step returns
#     5. event_loop iterates
#     6. EV_X is dequeued
#     7. fsm_step(EV_X) → new step
#
# ==============================================================================

# ------------------------------------------------------------------------------
# FSM Instance Layout (BSS)
# ------------------------------------------------------------------------------
# All FSM instances MUST follow this layout for fsm_step compatibility.
#
# Offset  0: magic       (4 bytes) - Signature for validation
# Offset  4: state       (1 byte)  - Current state (ONLY written by fsm_step)
# Offset  5: event_bits  (1 byte)  - log2(num_events) for shift-based indexing
# Offset  6: num_states  (1 byte)  - Number of states (for bounds check)
# Offset  7: num_events  (1 byte)  - Number of events (for bounds check)
# Offset  8: table       (4 bytes) - Pointer to transition table (.rodata)
# Offset 12: actions     (4 bytes) - Pointer to action table (.rodata)
# Offset 16: data        (N bytes) - FSM-specific data
#
# CONSTRAINT: num_events MUST be a power of 2 (1, 2, 4, 8, 16, ...)
#             This allows fast shift-based indexing without mul instruction.

.equ FSM_MAGIC,      0
.equ FSM_STATE,      4
.equ FSM_EVENT_BITS, 5
.equ FSM_NUM_STATES, 6
.equ FSM_NUM_EVENTS, 7
.equ FSM_TABLE,      8
.equ FSM_ACTIONS,   12
.equ FSM_DATA,      16

# Magic value for FSM instances
.equ FSM_MAGIC_VALUE, 0x46534D30    # "FSM0"

# ------------------------------------------------------------------------------
# Transition Table Entry Layout (.rodata)
# ------------------------------------------------------------------------------
# Each entry is 2 bytes:
#   Byte 0: next_state
#   Byte 1: action_id
#
# Table is indexed as: table[state * num_events + event_id]
# Total size: num_states * num_events * 2 bytes

# ------------------------------------------------------------------------------
# Action Table Layout (.rodata)
# ------------------------------------------------------------------------------
# Array of 4-byte function pointers indexed by action_id.
# Action signature: void action(a0=fsm_instance, a1=data0, a2=data1)
# Actions MUST NOT write to FSM_STATE.

# ==============================================================================
# TEXT Section
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# fsm_step - Execute ONE FSM Step (Table-Driven)
# ==============================================================================
#
# PURPOSE:
#   The ONLY function that may write to FSM_STATE.
#   Looks up transition in table, executes action, updates state.
#
# INPUTS:
#   a0 = pointer to FSM instance
#   a1 = event_id (0..num_events-1)
#   a2 = event data0
#   a3 = event data1
#
# OUTPUTS:
#   a0 = 0 on success
#   a0 = -1 on error (event_id out of bounds)
#   a0 = -2 on error (state out of bounds / corruption)
#   a0 = -3 on error (invalid magic)
#
# REGISTERS (RV32E safe):
#   Caller-saved: t0, t1, t2, a0-a5
#   Callee-saved: s0, s1 (saved/restored)
#
# ABI CONTRACT:
#   This is the ONLY function permitted to write FSM_STATE.
#   Any other write to FSM_STATE is a constitutional violation.
#
# ==============================================================================

.globl fsm_step
fsm_step:
    # --------------------------------------------------------------------------
    # Prologue: save context
    # --------------------------------------------------------------------------
    addi sp, sp, -20
    sw   ra,  0(sp)
    sw   s0,  4(sp)             # FSM instance pointer
    sw   s1,  8(sp)             # next_state (preserved across action call)
    sw   a2, 12(sp)             # data0
    sw   a3, 16(sp)             # data1

    mv   s0, a0                 # s0 = FSM instance

    # --------------------------------------------------------------------------
    # Validate magic (catch corruption / wrong pointer)
    # --------------------------------------------------------------------------
    lw   t0, FSM_MAGIC(s0)
    li   t1, FSM_MAGIC_VALUE
    bne  t0, t1, fsm_step_bad_magic

    # --------------------------------------------------------------------------
    # Bounds check: event_id < num_events
    # --------------------------------------------------------------------------
    lbu  t0, FSM_NUM_EVENTS(s0) # t0 = num_events
    bgeu a1, t0, fsm_step_bad_event

    # --------------------------------------------------------------------------
    # Bounds check: current_state < num_states
    # --------------------------------------------------------------------------
    lbu  t1, FSM_STATE(s0)      # t1 = current_state
    lbu  t2, FSM_NUM_STATES(s0) # t2 = num_states
    bgeu t1, t2, fsm_step_bad_state

    # --------------------------------------------------------------------------
    # Calculate table index: state * num_events + event_id
    # Using shift: state << event_bits + event_id
    # (num_events must be power of 2)
    # --------------------------------------------------------------------------
    lbu  t0, FSM_EVENT_BITS(s0) # t0 = log2(num_events)
    sll  t1, t1, t0             # t1 = state << event_bits = state * num_events
    add  t1, t1, a1             # t1 = index
    slli t1, t1, 1              # t1 = byte offset (2 bytes per entry)

    # --------------------------------------------------------------------------
    # Load transition entry: [next_state, action_id]
    # --------------------------------------------------------------------------
    lw   t0, FSM_TABLE(s0)      # t0 = table base
    add  t0, t0, t1             # t0 = &table[index]
    lbu  s1, 0(t0)              # s1 = next_state (SAVED for after action)
    lbu  t2, 1(t0)              # t2 = action_id

    # --------------------------------------------------------------------------
    # Load action function pointer
    # --------------------------------------------------------------------------
    lw   t0, FSM_ACTIONS(s0)    # t0 = actions table base
    slli t1, t2, 2              # t1 = action_id * 4
    add  t0, t0, t1             # t0 = &actions[action_id]
    lw   t0, 0(t0)              # t0 = action function pointer

    # --------------------------------------------------------------------------
    # Call action(fsm_instance, data0, data1)
    # Action MUST NOT modify FSM_STATE
    # --------------------------------------------------------------------------
    mv   a0, s0                 # a0 = FSM instance
    lw   a1, 12(sp)             # a1 = data0
    lw   a2, 16(sp)             # a2 = data1
    jalr ra, t0, 0              # Call action

    # --------------------------------------------------------------------------
    # UNIQUE POINT OF STATE WRITE
    # This is the ONLY place in the entire system where FSM_STATE is written
    # --------------------------------------------------------------------------
    sb   s1, FSM_STATE(s0)      # state = next_state

    # --------------------------------------------------------------------------
    # Return success
    # --------------------------------------------------------------------------
    li   a0, 0
    j    fsm_step_return

fsm_step_bad_magic:
    li   a0, -3
    j    fsm_step_return

fsm_step_bad_event:
    li   a0, -1
    j    fsm_step_return

fsm_step_bad_state:
    li   a0, -2

fsm_step_return:
    lw   ra,  0(sp)
    lw   s0,  4(sp)
    lw   s1,  8(sp)
    # a2 (data0) and a3 (data1) were saved on entry at offsets 12 and 16
    # but are caller-saved and intentionally not restored; sp+20 cleans the full frame
    addi sp, sp, 20
    ret

# ==============================================================================
# fsm_init - Initialize FSM Instance
# ==============================================================================
#
# PURPOSE:
#   Initialize an FSM instance with table pointers and initial state.
#   Called once during system init.
#
# INPUTS:
#   a0 = pointer to FSM instance (BSS)
#   a1 = pointer to transition table (.rodata)
#   a2 = pointer to action table (.rodata)
#   a3 = event_bits (log2 of num_events, e.g., 1 for 2 events, 2 for 4 events)
#   a4 = num_states
#   a5 = initial_state (usually 0)
#
# OUTPUTS:
#   None
#
# CONSTRAINT:
#   num_events = 2^event_bits (must be power of 2)
#
# RV32E NOTE:
#   a5 is the last argument register on RV32E (only a0-a5 available).
#   This function uses all 6 integer argument registers.
#
# ==============================================================================

.globl fsm_init
fsm_init:
    # Write magic
    li   t0, FSM_MAGIC_VALUE
    sw   t0, FSM_MAGIC(a0)

    # Write state and dimensions
    sb   a5, FSM_STATE(a0)      # initial_state
    sb   a3, FSM_EVENT_BITS(a0) # log2(num_events)
    sb   a4, FSM_NUM_STATES(a0) # num_states

    # Calculate num_events = 1 << event_bits for bounds check
    li   t0, 1
    sll  t0, t0, a3             # t0 = 1 << event_bits = num_events
    sb   t0, FSM_NUM_EVENTS(a0)

    # Write table pointers
    sw   a1, FSM_TABLE(a0)
    sw   a2, FSM_ACTIONS(a0)

    ret

# ==============================================================================
# fsm_get_state - Read Current State (for debugging/monitoring)
# ==============================================================================
#
# INPUTS:
#   a0 = pointer to FSM instance
#
# OUTPUTS:
#   a0 = current state (or -1 if invalid magic)
#
# ==============================================================================

.globl fsm_get_state
fsm_get_state:
    lw   t0, FSM_MAGIC(a0)
    li   t1, FSM_MAGIC_VALUE
    bne  t0, t1, fsm_get_state_bad_magic

    lbu  a0, FSM_STATE(a0)
    ret

fsm_get_state_bad_magic:
    li   a0, -1
    ret
