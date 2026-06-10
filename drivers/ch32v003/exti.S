# ==============================================================================
# drivers/ch32v003/exti.S - External Interrupt Handler
# ==============================================================================
#
# PURPOSE:
#   Minimal ISR for EXTI interrupts. Clears hardware flag and enqueues event.
#   Handles EXTI line 2 (LoRa DIO0 on PC2).
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 5 (ISR Discipline): Only ack + enqueue, no logic
#   - Article 10 (Layer Roles): Hardware access stays in drivers/
#   - PO-4 (ISR Safety Proof): Bounded time, no allocation, no blocking
#   - PO-1 (Event Origin Proof): Explicit enqueue point for EXTI events
#
# EVENT FORMAT:
#   Line 2 (DIO0): kind=IRQ, id=EV_IRQ_LORA_DIO0, flags=line_mask
#   data0: timestamp, data1: 0
#
# ==============================================================================

.include "core/events.inc"

# ------------------------------------------------------------------------------
# CH32V003 Register Addresses
# ------------------------------------------------------------------------------

.equ EXTI_INTFR,    0x40010414      # EXTI interrupt flag register (W1C)
.equ EXTI_LINE2,    (1 << 2)        # EXTI line 2 bit (DIO0/PC2)
.equ EXTI7_0_MASK,  0xFF            # IRQ 20 covers EXTI lines 7..0
.equ STK_CNTL,      0xE000F008      # SysTick counter low

# ==============================================================================
# TEXT Section - ISR Code
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# exti_isr - EXTI Interrupt Service Routine
# ==============================================================================
#
# PURPOSE:
#   Handle EXTI7_0 interrupt (IRQ 20). Called by core's irq_entry dispatcher.
#   Checks which line(s) triggered and enqueues appropriate event(s).
#
# INPUTS:
#   a0 - IRQ number (20, passed by irq_entry — not consumed; overwritten immediately)
#
# ISR SAFETY PROOF (PO-4):
#   - Execution time: O(1) - bounded checks, no unbounded loops
#   - No allocation: only uses registers and stack
#   - No blocking: no waits
#
# ==============================================================================

.globl exti_isr
exti_isr:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)                  # s0 = pending flags
    sw   s1, 8(sp)                  # s1 = timestamp

    # --------------------------------------------------------------------------
    # Step 1: Read and acknowledge pending EXTI7_0 flags
    # --------------------------------------------------------------------------
    li   t0, EXTI_INTFR
    lw   s0, 0(t0)                  # s0 = pending lines
    andi s0, s0, EXTI7_0_MASK       # IRQ 20 covers only lines 7..0
    beqz s0, exti_done
    sw   s0, 0(t0)                  # W1C: ack all pending EXTI7_0 lines

    # --------------------------------------------------------------------------
    # Step 2: Check line 2 (LoRa DIO0)
    # --------------------------------------------------------------------------
    andi t0, s0, EXTI_LINE2
    beqz t0, exti_done

    li   t0, STK_CNTL
    lw   s1, 0(t0)                  # s1 = timestamp

    # Enqueue DIO0 event
    # Header: kind=EV_KIND_IRQ (bits[31:24]), id=EV_IRQ_LORA_DIO0 (bits[23:16]),
    #         flags=EXTI_LINE2 bitmask (bits[15:0])
    li   a0, ((EV_KIND_IRQ << EV_KIND_SHIFT) | (EV_IRQ_LORA_DIO0 << EV_ID_SHIFT) | EXTI_LINE2)
    mv   a1, s1
    li   a2, 0
    call event_enqueue

exti_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret
