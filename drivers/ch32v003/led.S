# ==============================================================================
# drivers/ch32v003/led.S - LED Driver (PD4 + PD2)
# ==============================================================================
#
# PURPOSE:
#   Hardware driver for onboard LEDs on PD4 and PD2.
#   Provides heartbeat and activity toggle primitives for upper layers.
#
# LAYER: drivers/ (Article 10 — Layer Roles: hardware interaction only)
#
# API:
#   led_init            - Enable GPIOD clock, configure PD4/PD2 as outputs
#   led_toggle          - XOR GPIOD_OUTDR bit 4 (heartbeat LED)
#   led_activity_on     - Drive PD2 low  (active-low LED)
#   led_activity_off    - Drive PD2 high (active-low LED)
#
# ==============================================================================

# ------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------

.equ RCC_APB2PCENR,    0x40021018      # APB2 peripheral clock enable
.equ RCC_IOPDEN,       (1 << 5)        # GPIOD clock enable bit
.equ GPIOD_CFGLR,      0x40011400      # GPIOD config low register
.equ GPIOD_OUTDR,      0x4001140C      # GPIOD output data register
.equ LED_PIN,          4               # PD4 heartbeat LED
.equ ACTIVITY_LED_PIN, 2               # PD2 radio activity LED

# ==============================================================================
# TEXT Section
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# led_init - Initialize LED GPIO
# ==============================================================================
#
# Enables GPIOD clock, configures PD4 and PD2 as push-pull outputs (10MHz),
# PD4 off and PD2 released high (activity LED is active-low).
# Leaf function — no stack frame needed.
#
# INPUTS: None
# OUTPUTS: None
# CLOBBERS: t0, t1, t2
#
# ==============================================================================

.globl led_init
led_init:
    # Enable GPIOD clock
    li   t0, RCC_APB2PCENR
    lw   t1, 0(t0)
    li   t2, RCC_IOPDEN
    or   t1, t1, t2
    sw   t1, 0(t0)

    # Configure PD4 and PD2 as push-pull outputs (10MHz)
    li   t0, GPIOD_CFGLR
    lw   t1, 0(t0)
    li   t2, ~((0xF << 16) | (0xF << 8))
    and  t1, t1, t2
    li   t2, ((0x1 << 16) | (0x1 << 8))
    or   t1, t1, t2
    sw   t1, 0(t0)

    # Start with heartbeat LED off and activity LED off (active-low => high)
    li   t0, GPIOD_OUTDR
    lw   t1, 0(t0)
    li   t2, ~(1 << LED_PIN)
    and  t1, t1, t2
    li   t2, (1 << ACTIVITY_LED_PIN)
    or   t1, t1, t2
    sw   t1, 0(t0)

    ret

# ==============================================================================
# led_toggle - Toggle LED
# ==============================================================================
#
# XOR GPIOD_OUTDR bit 4 to toggle LED state.
# Leaf function — no stack frame needed.
#
# INPUTS: None
# OUTPUTS: None
# CLOBBERS: t0, t1, t2
#
# ==============================================================================

.globl led_toggle
led_toggle:
    li   t0, GPIOD_OUTDR
    lw   t1, 0(t0)
    li   t2, (1 << LED_PIN)
    xor  t1, t1, t2
    sw   t1, 0(t0)
    ret

# ==============================================================================
# led_activity_on - Turn Activity LED On
# ==============================================================================
#
# Clear GPIOD_OUTDR bit 2 to turn on the radio activity LED.
# Leaf function — no stack frame needed.
#
# INPUTS: None
# OUTPUTS: None
# CLOBBERS: t0, t1, t2
#
# ==============================================================================

.globl led_activity_on
led_activity_on:
    li   t0, GPIOD_OUTDR
    lw   t1, 0(t0)
    li   t2, ~(1 << ACTIVITY_LED_PIN)
    and  t1, t1, t2
    sw   t1, 0(t0)
    ret

# ==============================================================================
# led_activity_off - Turn Activity LED Off
# ==============================================================================
#
# Set GPIOD_OUTDR bit 2 to turn off the radio activity LED.
# Leaf function — no stack frame needed.
#
# INPUTS: None
# OUTPUTS: None
# CLOBBERS: t0, t1, t2
#
# ==============================================================================

.globl led_activity_off
led_activity_off:
    li   t0, GPIOD_OUTDR
    lw   t1, 0(t0)
    li   t2, (1 << ACTIVITY_LED_PIN)
    or   t1, t1, t2
    sw   t1, 0(t0)
    ret
