# ==============================================================================
# core/kernel.S - Kernel Meta-FSM and Root Dispatch Table
# ==============================================================================
#
# PURPOSE:
#   Implements the kernel meta-FSM with O(1) event dispatch.
#   Provides the register_activity ABI for sub-FSM registration.
#   Manages .kernel_init section iteration for auto-registration.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): All execution via event dispatch
#   - Article 6 (FSM): Meta-FSM qualifies as the root FSM governing all dispatch
#   - Article 9 (Structural Enforcement): root_table is not directly accessible
#   - Article 11 (ABI Sanctity): register_activity is the ONLY registration path
#   - PO-3 (FSM Ownership Proof): Each (kind,id) maps to exactly one handler
#   - PO-6 (Non-Bypassability Proof): Bounds checking enforced on all registrations
#
# ARCHITECTURE:
#   root_table[kind][id] = function pointer (or NULL)
#   Handler signature: void handler(a0=header, a1=data0, a2=data1)
#
# ==============================================================================

.include "core/events.inc"

# ==============================================================================
# BSS Section - Root Dispatch Table and Statistics
# ==============================================================================

.section .bss
.align 4

# ------------------------------------------------------------------------------
# root_table - O(1) Event Dispatch Table
# ------------------------------------------------------------------------------
# 2D array: root_table[EV_KIND_MAX][EV_ID_MAX]
# Each entry is a 4-byte function pointer (NULL = no handler)
# Total size: EV_KIND_MAX * EV_ID_MAX * 4 bytes
#
# ACCESS: ONLY via register_activity and kernel_dispatch
# NOT EXPORTED: Direct manipulation is forbidden (Article 9 — Structural Enforcement)

root_table:
    .rept (EV_KIND_MAX * EV_ID_MAX)
        .word 0
    .endr

# ==============================================================================
# TEXT Section - Kernel Functions
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# register_activity - Register Event Handler (ABI Entry Point)
# ==============================================================================
#
# PURPOSE:
#   The ONLY way to register event handlers with the kernel.
#   Associates a function pointer with a (kind, id) pair.
#
# INPUTS:
#   a0 - kind (0 to EV_KIND_MAX-1)
#   a1 - id   (0 to EV_ID_MAX-1)
#   a2 - handler function pointer (NULL to unregister)
#
# OUTPUTS:
#   a0 - 0 on success, -1 on bounds error
#
# ABI CONTRACT:
#   - Bounds checking is MANDATORY (PO-6)
#   - Registration replaces any existing handler silently
#   - Handler signature: void fn(a0=header, a1=data0, a2=data1)
#
# REGISTERS: t0, t1, t2 (caller-saved)
#
# ==============================================================================

.globl register_activity
register_activity:
    # --------------------------------------------------------------------------
    # Bounds check: kind < EV_KIND_MAX
    # --------------------------------------------------------------------------
    li   t0, EV_KIND_MAX
    bgeu a0, t0, reg_activity_fail  # kind >= MAX is invalid

    # --------------------------------------------------------------------------
    # Bounds check: id < EV_ID_MAX
    # --------------------------------------------------------------------------
    li   t0, EV_ID_MAX
    bgeu a1, t0, reg_activity_fail  # id >= MAX is invalid

    # --------------------------------------------------------------------------
    # Calculate table offset: (kind * EV_ID_MAX + id) * 4
    # --------------------------------------------------------------------------
    slli t1, a0, EV_ID_MAX_LOG2     # t1 = kind * EV_ID_MAX
    add  t1, t1, a1                 # t1 = kind * EV_ID_MAX + id
    slli t1, t1, 2                  # t1 = offset in bytes (index * 4)

    # --------------------------------------------------------------------------
    # Store handler pointer in table
    # --------------------------------------------------------------------------
    la   t0, root_table
    add  t0, t0, t1                 # t0 = address of table entry
    sw   a2, 0(t0)                  # Store handler (or NULL to unregister)

    # --------------------------------------------------------------------------
    # Return success
    # --------------------------------------------------------------------------
    li   a0, 0
    ret

reg_activity_fail:
    li   a0, -1
    ret

# ==============================================================================
# kernel_dispatch - Dispatch Event to Registered Handler
# ==============================================================================
#
# PURPOSE:
#   Look up handler for (kind, id) and call it with event data.
#   Called by event_loop after dequeue.
#
# INPUTS:
#   a0 - event header (kind:8 | id:8 | flags:16)
#   a1 - data0
#   a2 - data1
#
# OUTPUTS:
#   a0 - 0 if handler called, -1 if no handler
#
# BEHAVIOR:
#   - Extracts kind and id from header
#   - Bounds checks kind and id
#   - Looks up root_table[kind][id]
#   - If non-NULL, calls handler(header, data0, data1)
#   - Updates statistics
#
# REGISTERS: t0, t1, t2, a3, a4, a5, ra (caller-saved + ra for nested call)
#
# ==============================================================================

.globl kernel_dispatch
kernel_dispatch:
    # --------------------------------------------------------------------------
    # Save context for nested call
    # --------------------------------------------------------------------------
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   a0, 4(sp)                  # Save header for handler call
    sw   a1, 8(sp)                  # Save data0
    sw   a2, 12(sp)                 # Save data1

    # --------------------------------------------------------------------------
    # Extract kind from header: kind = (header >> 24) & 0xFF
    # --------------------------------------------------------------------------
    srli t0, a0, EV_KIND_SHIFT      # t0 = header >> 24
    andi t0, t0, 0xFF               # t0 = kind

    # --------------------------------------------------------------------------
    # Bounds check kind
    # --------------------------------------------------------------------------
    li   t1, EV_KIND_MAX
    bgeu t0, t1, dispatch_no_handler

    # --------------------------------------------------------------------------
    # Extract id from header: id = (header >> 16) & 0xFF
    # --------------------------------------------------------------------------
    lw   a0, 4(sp)                  # Reload header
    srli t1, a0, EV_ID_SHIFT        # t1 = header >> 16
    andi t1, t1, 0xFF               # t1 = id

    # --------------------------------------------------------------------------
    # Bounds check id
    # --------------------------------------------------------------------------
    li   t2, EV_ID_MAX
    bgeu t1, t2, dispatch_no_handler

    # --------------------------------------------------------------------------
    # Calculate table offset: (kind * EV_ID_MAX + id) * 4
    # --------------------------------------------------------------------------
    slli a3, t0, EV_ID_MAX_LOG2     # a3 = kind * EV_ID_MAX
    add  a3, a3, t1                 # a3 = kind * EV_ID_MAX + id
    slli a3, a3, 2                  # a3 = byte offset

    # --------------------------------------------------------------------------
    # Load handler pointer
    # --------------------------------------------------------------------------
    la   a4, root_table
    add  a4, a4, a3                 # a4 = &root_table[kind][id]
    lw   a5, 0(a4)                  # a5 = handler pointer

    # --------------------------------------------------------------------------
    # Check if handler is registered
    # --------------------------------------------------------------------------
    beqz a5, dispatch_no_handler

    # --------------------------------------------------------------------------
    # Call handler: handler(header, data0, data1)
    # --------------------------------------------------------------------------
    lw   a0, 4(sp)                  # a0 = header
    lw   a1, 8(sp)                  # a1 = data0
    lw   a2, 12(sp)                 # a2 = data1
    jalr ra, a5, 0                  # Call handler

    # --------------------------------------------------------------------------
    # Return success
    # --------------------------------------------------------------------------
    li   a0, 0
    j    dispatch_return

dispatch_no_handler:
    li   a0, -1

dispatch_return:
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ==============================================================================
# kernel_init - Initialize Kernel and Run .kernel_init Functions
# ==============================================================================
#
# PURPOSE:
#   Initialize kernel data structures and call all sub-FSM init functions
#   registered in the .kernel_init section.
#
# SEQUENCE:
#   1. Clear kernel statistics
#   2. Clear root_table
#   3. Initialize timer subsystem
#   4. Iterate .kernel_init section, calling each init function
#
# .kernel_init SECTION FORMAT:
#   Each entry is a 4-byte function pointer.
#   Functions are called in order with no arguments.
#   Functions should call register_activity to register their handlers.
#
# INPUTS: None
# OUTPUTS: None
#
# REGISTERS: t0, t1, t2, a0, a1, a2, ra (all caller-saved)
#
# ==============================================================================

.globl kernel_init
kernel_init:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Step 1: root_table is already zeroed by BSS clear
    # --------------------------------------------------------------------------
    # The BSS clear in _start already zeroes root_table, so no explicit clear
    # is needed here. Handlers register themselves via the .kernel_init pass below.

    # --------------------------------------------------------------------------
    # Step 2: Initialize timer subsystem
    # --------------------------------------------------------------------------
    call timer_init

    # --------------------------------------------------------------------------
    # Step 3: Iterate .kernel_init section
    # --------------------------------------------------------------------------
    # The linker defines __kernel_init_start and __kernel_init_end
    # Each entry is a 4-byte function pointer

    la   t0, __kernel_init_start
    la   t1, __kernel_init_end

kernel_init_loop:
    bgeu t0, t1, kernel_init_done   # If start >= end, done
    lw   t2, 0(t0)                  # t2 = init function pointer
    beqz t2, kernel_init_skip       # Skip NULL entries

    # Call init function (no arguments)
    # Save t0, t1 across call
    addi sp, sp, -8
    sw   t0, 0(sp)
    sw   t1, 4(sp)
    jalr ra, t2, 0                  # Call init function
    lw   t0, 0(sp)
    lw   t1, 4(sp)
    addi sp, sp, 8

kernel_init_skip:
    addi t0, t0, 4                  # Next entry
    j    kernel_init_loop

kernel_init_done:
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# Weak Symbols for Optional Components
# ==============================================================================
# Fallback stub used only in bare builds that exclude timer.S entirely.
# In normal builds, timer.S provides the real timer_init and this stub is unused.

.weak timer_init
timer_init:
    ret
