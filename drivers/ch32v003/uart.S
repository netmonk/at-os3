# ==============================================================================
# drivers/ch32v003/uart.S - USART1 Hardware Driver
# ==============================================================================
#
# PURPOSE:
#   Low-level USART1 hardware primitives. No business logic.
#   Exposes only hardware init, ISR, and polling transmit.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 5 (ISR Discipline): ISR only reads byte and enqueues event
#   - Article 10 (Layer Roles): Hardware access stays in drivers/
#   - PO-4 (ISR Safety Proof): Bounded time, no allocation, no blocking
#   - PO-5 (Layer Integrity Proof): Only core/console.S calls these functions
#
# EXPORTED FUNCTIONS (for core/console.S only):
#   uart_hw_init          - Initialize USART1 hardware and register ISR
#   uart_hw_putc_poll     - Blocking single-byte transmit (init/banner only)
#
# ==============================================================================

.include "core/events.inc"

# ------------------------------------------------------------------------------
# USART1 Register Addresses
# ------------------------------------------------------------------------------

.equ USART1_BASE,   0x40013800
.equ USART1_STATR,  0x40013800      # Status register (offset 0x00)
.equ USART1_DATAR,  0x40013804      # Data register (offset 0x04)
.equ USART1_BRR,    0x40013808      # Baud rate register (offset 0x08)
.equ USART1_CTLR1,  0x4001380C      # Control register 1 (offset 0x0C)

# STATR bits
.equ USART_TXE,     (1 << 7)        # Transmit data register empty
.equ USART_TC,      (1 << 6)        # Transmission complete
.equ USART_RXNE,    (1 << 5)        # Read data register not empty

# CTLR1 bits
.equ USART_UE,      (1 << 13)       # USART enable
.equ USART_RXNEIE,  (1 << 5)        # RXNE interrupt enable
.equ USART_TE,      (1 << 3)        # Transmitter enable
.equ USART_RE,      (1 << 2)        # Receiver enable

# Combined CTLR1 value: UE + TE + RE + RXNEIE
.equ USART_CTLR1_VAL, (USART_UE | USART_TE | USART_RE | USART_RXNEIE)

# Baud rate: 115200 @ 48MHz HCLK
# USART_BRR holds DIV_Mantissa[15:4] and DIV_Fraction[3:0] (RM 12.10.3).
# baud = HCLK / (16 * USARTDIV), so USARTDIV = 48000000 / (16 * 115200) = 26.0417
#   DIV_Mantissa = 26 (0x1A), DIV_Fraction = round(0.0417 * 16) = 1
#   BRR = (26 << 4) | 1 = 0x1A1   (actual baud 115108, +0.08%)
.equ USART_BRR_VAL, 0x1A1

# ------------------------------------------------------------------------------
# RCC and GPIO Registers
# ------------------------------------------------------------------------------

.equ RCC_APB2PCENR, 0x40021018      # APB2 peripheral clock enable
.equ RCC_USART1EN,  (1 << 14)       # USART1 clock enable bit
.equ RCC_IOPDEN,    (1 << 5)        # GPIO port D clock enable (PD5=TX, PD6=RX)

.equ GPIOD_CFGLR,   0x40011400      # Port D configuration low register

# GPIO config for PD5 (TX) and PD6 (RX)
# PD5: CNF=10 (AF push-pull), MODE=11 (50MHz output) -> 0xB at bits [23:20]
# PD6: CNF=01 (floating input), MODE=00 (input) -> 0x4 at bits [27:24]
.equ GPIO_PD5_SHIFT, 20
.equ GPIO_PD6_SHIFT, 24
.equ GPIO_PD5_MASK,  (0xF << 20)
.equ GPIO_PD6_MASK,  (0xF << 24)
.equ GPIO_PD5_VAL,   (0xB << 20)    # AF push-pull, 50MHz
.equ GPIO_PD6_VAL,   (0x4 << 24)    # Floating input

# ------------------------------------------------------------------------------
# PFIC (Interrupt Controller)
# ------------------------------------------------------------------------------

.equ PFIC_IENR2,    0xE000E104      # Interrupt enable register 2
.equ IRQ_USART1,    32              # USART1 PFIC interrupt number (cause value, not vector index)
.equ PFIC_USART1,   (1 << 0)        # Bit 0 in IENR2 (interrupt 32)

# ------------------------------------------------------------------------------
# Software RX ring (single-producer ISR, single-consumer soft handler)
# ------------------------------------------------------------------------------
.equ UART_RX_RING_SIZE,  32
.equ UART_RX_RING_MASK,  (UART_RX_RING_SIZE - 1)

# ==============================================================================
# BSS - RX ring state
# ==============================================================================

.section .bss
.align 4

uart_rx_ring:
    .space UART_RX_RING_SIZE

uart_rx_ring_head:
    .word 0

uart_rx_ring_tail:
    .word 0

uart_rx_pump_pending:
    .word 0

# ==============================================================================
# TEXT Section - Hardware Functions
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# uart_hw_init - Initialize USART1 Hardware
# ==============================================================================
#
# PURPOSE:
#   Configure USART1 for 115200 8N1, register ISR, enable IRQ.
#   Called by core/console.S console_init.
#
# INPUTS: None
# OUTPUTS: None
#
# REGISTERS MODIFIED: t0, t1, t2, a0, a1 (caller-saved)
# CALLS: register_isr (must save/restore ra)
#
# ==============================================================================

.globl uart_hw_init
uart_hw_init:
    # --------------------------------------------------------------------------
    # Save RA (we call register_isr)
    # --------------------------------------------------------------------------
    addi sp, sp, -4
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Step 1: Enable USART1 and GPIOD clocks
    # --------------------------------------------------------------------------
    li   t0, RCC_APB2PCENR
    lw   t1, 0(t0)
    li   t2, (RCC_USART1EN | RCC_IOPDEN)
    or   t1, t1, t2
    sw   t1, 0(t0)

    # --------------------------------------------------------------------------
    # Step 2: Configure GPIO PD5 (TX) and PD6 (RX)
    # --------------------------------------------------------------------------
    li   t0, GPIOD_CFGLR
    lw   t1, 0(t0)
    # Clear PD5 and PD6 config bits
    li   t2, ~(GPIO_PD5_MASK | GPIO_PD6_MASK)
    and  t1, t1, t2
    # Set PD5 = AF push-pull 50MHz, PD6 = floating input
    li   t2, (GPIO_PD5_VAL | GPIO_PD6_VAL)
    or   t1, t1, t2
    sw   t1, 0(t0)

    # --------------------------------------------------------------------------
    # Step 3: Set baud rate
    # --------------------------------------------------------------------------
    li   t0, USART1_BRR
    li   t1, USART_BRR_VAL
    sw   t1, 0(t0)

    # --------------------------------------------------------------------------
    # Step 4: Enable USART with TX, RX, and RX interrupt
    # --------------------------------------------------------------------------
    li   t0, USART1_CTLR1
    li   t1, USART_CTLR1_VAL
    sw   t1, 0(t0)

    # --------------------------------------------------------------------------
    # Step 5: Register ISR with core dispatcher
    # --------------------------------------------------------------------------
    li   a0, IRQ_USART1             # Interrupt 32 (USART1)
    la   a1, uart_hw_rx_isr         # ISR handler
    call register_isr

    # --------------------------------------------------------------------------
    # Step 6: Reset software RX ring state
    # --------------------------------------------------------------------------
    la   t0, uart_rx_ring_head
    sw   zero, 0(t0)
    la   t0, uart_rx_ring_tail
    sw   zero, 0(t0)
    la   t0, uart_rx_pump_pending
    sw   zero, 0(t0)

    # --------------------------------------------------------------------------
    # Restore RA and return
    # --------------------------------------------------------------------------
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# uart_hw_rx_isr - USART1 Receive Interrupt Service Routine
# ==============================================================================
#
# PURPOSE:
#   Minimal ISR for USART1 RX. Reads byte from DATAR, pushes it into the
#   software RX ring, and enqueues at most one EV_SOFT_RADIO_RX_PUMP.
#   Called by core/trap.S irq_entry.
#
# INPUTS:
#   a0 - IRQ number (32, passed by irq_entry)
#
# OUTPUTS:
#   None
#
# ISR SAFETY PROOF (PO-4):
#   - Execution time: O(1) - fixed instruction sequence
#   - No allocation: only registers and stack
#   - No blocking: no loops, no waits
#   - Shared state: bounded ring writes + at most one event enqueue
#
# ==============================================================================

.globl uart_hw_rx_isr
uart_hw_rx_isr:
    # --------------------------------------------------------------------------
    # Save RA (we may call event_enqueue)
    # --------------------------------------------------------------------------
    addi sp, sp, -4
    sw   ra, 0(sp)

    # --------------------------------------------------------------------------
    # Step 1: Check RXNE flag (defensive - ISR shouldn't fire without it)
    # --------------------------------------------------------------------------
    li   t0, USART1_STATR
    lw   t1, 0(t0)
    li   t2, USART_RXNE
    and  t1, t1, t2
    beqz t1, uart_hw_rx_isr_done    # No data, spurious interrupt

    # --------------------------------------------------------------------------
    # Step 2: Read byte from DATAR (this clears RXNE)
    # --------------------------------------------------------------------------
    li   t0, USART1_DATAR
    lw   t2, 0(t0)                  # t2 = received byte (only low 8 bits valid)
    andi t2, t2, 0xFF               # Mask to byte

    # --------------------------------------------------------------------------
    # Step 3: Push byte into software RX ring if space is available
    # --------------------------------------------------------------------------
    la   t0, uart_rx_ring_head
    lw   t1, 0(t0)                  # t1 = head
    addi a1, t1, 1
    andi a1, a1, UART_RX_RING_MASK  # a1 = next_head
    la   a2, uart_rx_ring_tail
    lw   a2, 0(a2)                  # a2 = tail
    beq  a1, a2, uart_hw_rx_isr_done # ring full, drop byte

    la   a2, uart_rx_ring
    add  a2, a2, t1
    sb   t2, 0(a2)
    sw   a1, 0(t0)                  # head = next_head

    # --------------------------------------------------------------------------
    # Step 4: Schedule a single deferred RX pump if not already pending
    # --------------------------------------------------------------------------
    la   t0, uart_rx_pump_pending
    lw   t1, 0(t0)
    bnez t1, uart_hw_rx_isr_done
    li   t1, 1
    sw   t1, 0(t0)

    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_RADIO_RX_PUMP << EV_ID_SHIFT))
    li   a1, 0
    li   a2, 0
    call event_enqueue

uart_hw_rx_isr_done:
    # --------------------------------------------------------------------------
    # Restore RA and return to irq_entry
    # --------------------------------------------------------------------------
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# uart_hw_putc_poll - Blocking Single-Byte Transmit
# ==============================================================================
#
# PURPOSE:
#   Low-level polling transmit. Waits for TXE then writes byte.
#   Called by core/console.S only.
#
# INPUTS:
#   a0 - byte to transmit (low 8 bits)
#
# OUTPUTS:
#   None
#
# NOTE: This is blocking. Acceptable for debug output.
#       Does NOT call any other function, so no need to save RA.
#
# ==============================================================================

.globl uart_hw_putc_poll
uart_hw_putc_poll:
    # --------------------------------------------------------------------------
    # Poll STATR until TXE is set
    # --------------------------------------------------------------------------
    li   t0, USART1_STATR
    li   t1, USART_TXE
1:
    lw   t2, 0(t0)                  # Read STATR
    and  t2, t2, t1                 # Check TXE bit
    beqz t2, 1b                     # Loop until TXE set

    # --------------------------------------------------------------------------
    # Write byte to DATAR
    # --------------------------------------------------------------------------
    li   t0, USART1_DATAR
    sw   a0, 0(t0)                  # Write byte (only low 8 bits used by HW)

    ret

# ==============================================================================
# uart_hw_rx_ring_get - Pop one byte from the software RX ring
# ==============================================================================
#
# PURPOSE:
#   Dequeue one byte from the software RX ring.
#   Called by at_fsm.S at_rx_handler on each EV_SOFT_RADIO_RX_PUMP.
#
# INPUTS: None
#
# OUTPUTS:
#   a0 = 1 if a byte was returned, 0 if ring empty
#   a1 = byte value (valid only when a0 == 1)
#
# NOTES:
#   - Clears uart_rx_pump_pending when the ring becomes empty (no more pumps needed).
#   - Uses a short critical section (IRQ disable/restore) to avoid races with the RX ISR.
#
# REGISTERS: t0, t1, t2, a0, a1, a3 (caller-saved)
#
# ==============================================================================

.globl uart_hw_rx_ring_get
uart_hw_rx_ring_get:
    csrrci a3, mstatus, 0x8

    la   t0, uart_rx_ring_head
    lw   t1, 0(t0)                  # t1 = head
    la   t0, uart_rx_ring_tail
    lw   t2, 0(t0)                  # t2 = tail
    beq  t1, t2, uart_hw_rx_ring_empty

    la   t0, uart_rx_ring
    add  t0, t0, t2
    lbu  a1, 0(t0)

    la   t0, uart_rx_ring_tail
    addi t2, t2, 1
    andi t2, t2, UART_RX_RING_MASK
    sw   t2, 0(t0)

    li   a0, 1
    j    uart_hw_rx_ring_restore

uart_hw_rx_ring_empty:
    la   t0, uart_rx_pump_pending
    sw   zero, 0(t0)
    li   a0, 0
    li   a1, 0

uart_hw_rx_ring_restore:
    andi a3, a3, 0x8
    beqz a3, uart_hw_rx_ring_restore_done
    csrsi mstatus, 0x8

uart_hw_rx_ring_restore_done:
    ret
