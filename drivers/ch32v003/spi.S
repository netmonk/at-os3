# ==============================================================================
# drivers/ch32v003/spi.S - SPI1 Hardware Driver
# ==============================================================================
#
# PURPOSE:
#   Low-level SPI1 master driver for CH32V003.
#   Provides blocking transfer functions for LoRa module communication.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 10 (Layer Roles): Hardware-only driver, no business logic
#   - Article 11 (ABI Sanctity): Explicit interface, no internal exposure
#
# PIN MAPPING (SPI1_RM=0, default):
#   PC5 = SCK   (AF push-pull)
#   PC6 = MOSI  (AF push-pull)
#   PC7 = MISO  (floating input)
#   PC4 = CS    (GPIO output, active low - software controlled)
#
# ==============================================================================

.include "drivers/ch32v003/mmio.inc"
.include "drivers/ch32v003/spi.inc"

# ==============================================================================
# SPI1 Register Definitions
# ==============================================================================

.equ SPI1_BASE,         0x40013000
.equ SPI1_CTLR1,        (SPI1_BASE + 0x00)
.equ SPI1_CTLR2,        (SPI1_BASE + 0x04)
.equ SPI1_STATR,        (SPI1_BASE + 0x08)
.equ SPI1_DATAR,        (SPI1_BASE + 0x0C)

# CTLR1 bits
.equ SPI_CPHA,          (1 << 0)        # Clock phase
.equ SPI_CPOL,          (1 << 1)        # Clock polarity
.equ SPI_MSTR,          (1 << 2)        # Master mode
.equ SPI_BR_SHIFT,      3               # Baud rate divisor shift
.equ SPI_BR_MASK,       (0x7 << 3)      # Baud rate mask
.equ SPI_SPE,           (1 << 6)        # SPI enable
.equ SPI_LSBFIRST,      (1 << 7)        # LSB first
.equ SPI_SSI,           (1 << 8)        # Internal slave select
.equ SPI_SSM,           (1 << 9)        # Software slave management
.equ SPI_DFF,           (1 << 11)       # Data frame format (0=8bit, 1=16bit)

# STATR bits
.equ SPI_RXNE,          (1 << 0)        # RX not empty
.equ SPI_TXE,           (1 << 1)        # TX empty
.equ SPI_BSY,           (1 << 7)        # Busy

.equ BUSY_TIMEOUT_LOOPS, 500000        # SX1262 BUSY timeout, coarse ~100 ms bound

# Baud rate divisor values (fPCLK/N)
.equ SPI_BR_DIV2,       0               # fPCLK/2   = 24MHz @ 48MHz
.equ SPI_BR_DIV4,       1               # fPCLK/4   = 12MHz @ 48MHz
.equ SPI_BR_DIV8,       2               # fPCLK/8   = 6MHz  @ 48MHz
.equ SPI_BR_DIV16,      3               # fPCLK/16  = 3MHz  @ 48MHz
.equ SPI_BR_DIV32,      4               # fPCLK/32  = 1.5MHz @ 48MHz
.equ SPI_BR_DIV64,      5               # fPCLK/64  = 750kHz @ 48MHz
.equ SPI_BR_DIV128,     6               # fPCLK/128 = 375kHz @ 48MHz
.equ SPI_BR_DIV256,     7               # fPCLK/256 = 187kHz @ 48MHz

# ==============================================================================
# GPIO and LoRa pin definitions — see spi.inc
# ==============================================================================

# RCC clock enable
.equ RCC_BASE,          0x40021000
.equ RCC_APB2PCENR,     (RCC_BASE + 0x18)
.equ RCC_SPI1EN,        (1 << 12)       # SPI1 clock enable
.equ RCC_IOPCEN,        (1 << 4)        # GPIOC clock enable

# ==============================================================================
# TEXT Section
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# spi_init - Initialize SPI1 as master
# ==============================================================================
#
# PURPOSE:
#   Configure SPI1 hardware and GPIO pins for master mode.
#   Sets up for LoRa communication (Mode 0: CPOL=0, CPHA=0).
#
# INPUTS:
#   a0 = baud rate divisor (0-7, use SPI_BR_DIVx constants)
#
# OUTPUTS:
#   None
#
# CLOBBERS:
#   t0, t1, t2
#
# ==============================================================================

.globl spi_init
spi_init:
    # --------------------------------------------------------------------------
    # Enable clocks for GPIOC and SPI1
    # --------------------------------------------------------------------------
    MMIO_SET_BITS RCC_APB2PCENR, (RCC_IOPCEN | RCC_SPI1EN)

    # --------------------------------------------------------------------------
    # Configure GPIO: PC4=CS, PC5=SCK, PC6=MOSI, PC7=MISO
    # --------------------------------------------------------------------------
    MMIO_WRITE_FIELD GPIOC_CFGLR, SPI_GPIO_MASK, SPI_GPIO_CFG

    # --------------------------------------------------------------------------
    # Set CS high and RESET high (both inactive) before enabling SPI
    # --------------------------------------------------------------------------
    li   t0, GPIOC_BSHR
    li   t1, (SPI_CS_HIGH | LORA_RESET_HIGH)
    sw   t1, 0(t0)

    # Default RF switch off until the radio explicitly enters TX or RX mode.
.ifdef CONFIG_RADIO_SX1278
    li   t1, (LORA_TXEN_LOW | LORA_RXEN_LOW)
    sw   t1, 0(t0)
    .else
        .ifdef CONFIG_RADIO_SX1262
    li   t1, LORA_RXEN_LOW
    sw   t1, 0(t0)
        .else
            .error "No radio module selected: define CONFIG_RADIO_SX1278 or CONFIG_RADIO_SX1262"
        .endif
.endif

    # --------------------------------------------------------------------------
    # Configure SPI1: Master, software NSS, 8-bit, MSB first
    # --------------------------------------------------------------------------
    # CTLR1 = SSM | SSI | MSTR | (BR << 3)
    # Mode 0: CPOL=0, CPHA=0 (default)
    li   t0, SPI1_CTLR1
    li   t1, (SPI_SSM | SPI_SSI | SPI_MSTR)
    andi a0, a0, 0x7              # Sanitize BR value
    slli a0, a0, SPI_BR_SHIFT
    or   t1, t1, a0
    sw   t1, 0(t0)

    # --------------------------------------------------------------------------
    # Enable SPI
    # --------------------------------------------------------------------------
    lw   t1, 0(t0)
    ori  t1, t1, SPI_SPE
    sw   t1, 0(t0)

    ret

# ==============================================================================
# spi_transfer - Transfer one byte (full duplex)
# ==============================================================================
#
# PURPOSE:
#   Send one byte and receive one byte simultaneously.
#   Blocks until transfer complete.
#
# INPUTS:
#   a0 = byte to send
#
# OUTPUTS:
#   a0 = byte received
#
# CLOBBERS:
#   t0, t1
#
# ==============================================================================

.equ SPI_TIMEOUT_LOOPS, 10000           # SPI timeout: ~10k poll iterations, order of ~1 ms at 48MHz (coarse safety bound)

.globl spi_transfer
spi_transfer:
    # --------------------------------------------------------------------------
    # Wait for TXE (TX buffer empty) with timeout
    # --------------------------------------------------------------------------
    li   t0, SPI1_STATR
    li   t2, SPI_TIMEOUT_LOOPS
1:
    lw   t1, 0(t0)
    andi t1, t1, SPI_TXE
    bnez t1, 1f                         # TXE=1, continue
    addi t2, t2, -1
    bnez t2, 1b
    li   a0, 0xFF                       # Timeout, return 0xFF
    ret
1:
    # --------------------------------------------------------------------------
    # Write data to DATAR
    # --------------------------------------------------------------------------
    li   t0, SPI1_DATAR
    sw   a0, 0(t0)

    # --------------------------------------------------------------------------
    # Wait for RXNE (RX buffer not empty) with timeout
    # --------------------------------------------------------------------------
    li   t0, SPI1_STATR
    li   t2, SPI_TIMEOUT_LOOPS
2:
    lw   t1, 0(t0)
    andi t1, t1, SPI_RXNE
    bnez t1, 2f                         # RXNE=1, continue
    addi t2, t2, -1
    bnez t2, 2b
    li   a0, 0xFF                       # Timeout, return 0xFF
    ret
2:

    # --------------------------------------------------------------------------
    # Read received data
    # --------------------------------------------------------------------------
    li   t0, SPI1_DATAR
    lw   a0, 0(t0)
    andi a0, a0, 0xFF             # Mask to 8 bits

    ret

# ==============================================================================
# spi_cs_low - Assert CS (active low)
# ==============================================================================
#
# PURPOSE:
#   Pull CS pin low to select the slave device.
#
# CLOBBERS:
#   t0, t1
#
# ==============================================================================

.globl spi_cs_low
spi_cs_low:
    li   t0, GPIOC_BSHR
    li   t1, SPI_CS_LOW           # Write to reset bits (high 16 bits)
    sw   t1, 0(t0)
    ret

# ==============================================================================
# spi_cs_high - Deassert CS
# ==============================================================================
#
# PURPOSE:
#   Pull CS pin high to deselect the slave device.
#
# CLOBBERS:
#   t0, t1
#
# ==============================================================================

.globl spi_cs_high
spi_cs_high:
    li   t0, GPIOC_BSHR
    li   t1, SPI_CS_HIGH          # Write to set bits (low 16 bits)
    sw   t1, 0(t0)
    ret

# ==============================================================================
# LoRa Hardware Control Functions
# ==============================================================================

# ------------------------------------------------------------------------------
# lora_reset - Hardware reset the LoRa module
# ------------------------------------------------------------------------------
#
# PURPOSE:
#   Perform hardware reset: RESET low for ~1ms, then high.
#   Caller should wait ~10ms after for module ready.
#
# CLOBBERS:
#   t0, t1, a0
#
# ------------------------------------------------------------------------------

.globl lora_reset
lora_reset:
    # RESET low
    li   t0, GPIOC_BSHR
    li   t1, LORA_RESET_LOW
    sw   t1, 0(t0)

    # Delay ~1ms @ 48MHz (simple loop, ~48000 cycles)
    li   a0, 12000
1:  addi a0, a0, -1
    bnez a0, 1b

    # RESET high
    li   t1, LORA_RESET_HIGH
    sw   t1, 0(t0)

    ret

.globl lora_wait_busy
lora_wait_busy:
.ifdef CONFIG_RADIO_SX1278
    # SX1278 drives no BUSY signal. The symbol is kept as an inert service hook
    # for shared low-level code paths that want a bounded wait point.
    li   a0, 0
    ret
    .else
        .ifdef CONFIG_RADIO_SX1262
    li   t2, BUSY_TIMEOUT_LOOPS
    li   t0, GPIOC_BASE + 0x08          # GPIOC_INDR

1:
    lw   t1, 0(t0)
    and  t1, t1, LORA_BUSY_MASK
    beqz t1, lora_wait_busy_ok

    addi t2, t2, -1
    bnez t2, 1b

    li   a0, -1
    ret

lora_wait_busy_ok:
    li   a0, 0
    ret
        .else
            .error "No radio module selected: define CONFIG_RADIO_SX1278 or CONFIG_RADIO_SX1262"
        .endif
.endif
