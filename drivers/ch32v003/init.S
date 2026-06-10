# ==============================================================================
# drivers/ch32v003/init.S - CH32V003 Hardware Initialization
# ==============================================================================
#
# PURPOSE:
#   Initialize CH32V003-specific hardware peripherals.
#   This is the only file that contains CH32V003 register addresses.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 10 (Layer Roles): Hardware init belongs in drivers/
#   - Article 11 (ABI Sanctity): Exposes only driver_init to core
#   - PO-5 (Layer Boundary): No core code references these addresses
#
# HARDWARE CONFIGURED:
#   - RCC: Enable clocks for AFIO, GPIOC
#   - AFIO: Route EXTI2 to port C (LoRa DIO0)
#   - EXTI: Line 2 rising edge trigger (LoRa DIO0)
#   - PFIC: Priority config + Enable IRQ 20 (EXTI7_0)
#
# ==============================================================================

# ------------------------------------------------------------------------------
# CH32V003 Register Addresses
# ------------------------------------------------------------------------------
# These addresses are specific to CH32V003 and must not leak to core/

# RCC (Reset and Clock Control)
.equ RCC_APB2PCENR, 0x40021018  # APB2 peripheral clock enable register

# AFIO (Alternate Function I/O)
.equ AFIO_EXTICR,   0x40010008  # External interrupt configuration register

# EXTI (External Interrupt)
.equ EXTI_INTENR,   0x40010400  # Interrupt enable register
.equ EXTI_FTENR,    0x4001040C  # Falling trigger enable register (unused: DIO0 is rising-edge only)
.equ EXTI_INTFR,    0x40010414  # Interrupt flag register (W1C)

# PFIC (Programmable Fast Interrupt Controller)
.equ PFIC_IENR1,    0xE000E100  # Interrupt enable register 1
.equ PFIC_IPRIOR,   0xE000E400  # Interrupt priority registers base (byte per IRQ)

# IRQ numbers for priority configuration (from CH32V003 Table 6-1)
.equ IRQ_SYSTICK,   12          # SysTick timer IRQ
.equ IRQ_USART1,    32          # USART1 global interrupt

# Priority values (bit 7 = preemption level with 2-level nesting)
.equ PRIO_HIGH,     0x80        # High preemption (can preempt PRIO_LOW)
.equ PRIO_LOW,      0x00        # Low preemption (can be preempted)

# ------------------------------------------------------------------------------
# Bit Definitions
# ------------------------------------------------------------------------------

# RCC_APB2PCENR bits
.equ RCC_AFIOEN,    (1 << 0)    # AFIO clock enable
.equ RCC_IOPCEN,    (1 << 4)    # GPIO port C clock enable

# AFIO EXTICR for line 2 (LoRa DIO0 on PC2)
.equ AFIO_EXTI2_SHIFT, 4        # EXTI2 source starts at bit 4
.equ AFIO_EXTI2_MASK, (3 << 4)  # 2 bits for EXTI2 source
.equ AFIO_EXTI2_PORTC, (2 << 4) # Port C = 0b10

# EXTI lines
.equ EXTI_LINE2,    (1 << 2)    # EXTI line 2 bit (DIO0)

# EXTI rising edge register
.equ EXTI_RTENR,    0x40010408  # Rising trigger enable register

# PFIC IRQ 20 (EXTI7_0)
.equ IRQ_EXTI7_0,   20          # IRQ number for EXTI lines 7-0
.equ PFIC_IRQ20,    (1 << 20)   # Bit 20 in IENR1

# ==============================================================================
# TEXT Section - Initialization Code
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# driver_init - Main Driver Initialization Entry Point
# ==============================================================================
#
# PURPOSE:
#   Initialize all CH32V003 hardware and register ISRs with core.
#   Called once by core/bootstrap.S (_start) after BSS is cleared.
#
# ABI CONTRACT:
#   - Called by core via `call driver_init`
#   - Must preserve s0-s11 (callee-saved registers)
#   - May use a0-a7, t0-t6 freely
#   - Must call register_isr to register interrupt handlers
#
# SEQUENCE:
#   1. Enable peripheral clocks (RCC)
#   2. Configure EXTI line 2 for rising edge (LoRa DIO0)
#   3. Register EXTI ISR with core
#   4. Configure interrupt priorities for nesting
#   (Step 5 — PFIC IRQ enable — moved to drivers/ch32v003/bootstrap.S)
#
# NOTE: Console init and global interrupt enable are done by drivers/ch32v003/bootstrap.S
#       after kernel_init, preserving layer sovereignty (Article 10).
#
# ==============================================================================

.globl driver_init
driver_init:
    # --------------------------------------------------------------------------
    # Save return address (we'll call register_isr)
    # --------------------------------------------------------------------------
    addi sp, sp, -4             # Allocate stack space
    sw   ra, 0(sp)              # Save return address

    # ==========================================================================
    # Step 1: Enable peripheral clocks (RCC_APB2PCENR)
    # ==========================================================================
    # Enable clocks for AFIO and GPIOC before accessing their registers

    li   t0, RCC_APB2PCENR      # t0 = RCC_APB2PCENR address
    lw   t1, 0(t0)              # t1 = current register value
    ori  t1, t1, RCC_AFIOEN     # Set bit 0: enable AFIO clock
    ori  t1, t1, RCC_IOPCEN     # Set bit 4: enable GPIOC clock (DIO0)
    sw   t1, 0(t0)              # Write back to register

    # ==========================================================================
    # Step 2: Configure EXTI line 2 for rising edge (LoRa DIO0 on PC2)
    # ==========================================================================
    # DIO0 goes high when TX_DONE or RX_DONE occurs

    # Configure AFIO: EXTI2 source = Port C (0b10)
    li   t0, AFIO_EXTICR
    lw   t1, 0(t0)
    li   t2, ~AFIO_EXTI2_MASK
    and  t1, t1, t2             # Clear EXTI2 field
    ori  t1, t1, AFIO_EXTI2_PORTC
    sw   t1, 0(t0)

    # Enable EXTI line 2 interrupt
    li   t0, EXTI_INTENR
    lw   t1, 0(t0)
    ori  t1, t1, EXTI_LINE2
    sw   t1, 0(t0)

    # Set rising edge trigger for line 2
    li   t0, EXTI_RTENR
    lw   t1, 0(t0)
    ori  t1, t1, EXTI_LINE2
    sw   t1, 0(t0)

    # Clear any pending interrupt on line 2
    li   t0, EXTI_INTFR
    li   t1, EXTI_LINE2
    sw   t1, 0(t0)

    # ==========================================================================
    # Step 3: Register EXTI ISR with core dispatcher
    # ==========================================================================
    # Use core ABI to register our interrupt handler

    li   a0, IRQ_EXTI7_0        # a0 = IRQ number (20)
    la   a1, exti_isr           # a1 = address of our ISR handler
    call register_isr           # Register with core's ISR table

    # ==========================================================================
    # Step 4: Configure interrupt priorities for nesting
    # ==========================================================================
    # Priority scheme (2-level nesting, bit 7 = preemption level):
    #   SysTick (IRQ 12): PRIO_HIGH (0x80) - can preempt others
    #   EXTI    (IRQ 20): PRIO_LOW  (0x00) - can be preempted by SysTick
    #   USART1  (IRQ 32): PRIO_LOW  (0x00) - can be preempted by SysTick
    #
    # This ensures timer accuracy is preserved even during UART/button handling.

    li   t0, PFIC_IPRIOR
    li   t1, PRIO_HIGH
    sb   t1, IRQ_SYSTICK(t0)    # SysTick = high priority
    li   t1, PRIO_LOW
    sb   t1, IRQ_USART1(t0)     # USART1 = low priority
    sb   t1, IRQ_EXTI7_0(t0)    # EXTI = low priority

    # ==========================================================================
    # Step 5: PFIC IRQ enable - MOVED to drivers/ch32v003/bootstrap.S
    # ==========================================================================
    # NOTE: PFIC_IENR1/2 writes are centralized in drivers/ch32v003/bootstrap.S
    # after all init functions complete. IRQ 12 (SysTick), IRQ 20 (EXTI), IRQ 32 (USART1).

    # --------------------------------------------------------------------------
    # Restore and return
    # --------------------------------------------------------------------------
    # NOTE: console_init and global interrupt enable are handled by drivers/ch32v003/bootstrap.S
    #       after kernel_init completes. This preserves:
    #       - Article 10 (Layer Roles): drivers/ does not call core/
    #       - Init safety: IRQs stay off until all subsystems are ready
    lw   ra, 0(sp)              # Restore return address
    addi sp, sp, 4              # Deallocate stack
    ret                         # Return to core
