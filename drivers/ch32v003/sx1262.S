# ==============================================================================
# drivers/ch32v003/sx1262.S - SX1262 LoRa Radio Driver
# ==============================================================================
#
# PURPOSE:
#   Low-level driver for Semtech SX1262 LoRa transceiver.
#   Used with Ebyte E22-xxxM modules.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 10 (Layer Roles): Hardware-only driver
#   - Article 11 (ABI Sanctity): Explicit interface, callee-saved registers
#     for all values that must survive function calls.
#
# ABI CONTRACT (RV32E):
#   All exported functions follow the RV32E calling convention:
#   - CLOBBERS: a0, a1, t0, t1, t2 (caller-saved)
#   - PRESERVES: s0, s1, ra (callee-saved, saved/restored in prologue/epilogue)
#   - Callers MUST save their own t0-t2 before calling if values are needed after.
#   - All values that must survive internal calls use s0/s1, NOT t0 with stack reloads.
#
# SX1262 COMMAND INTERFACE:
#   Unlike SX1276, SX1262 uses command-based SPI protocol.
#   All commands require BUSY check after completion.
#
# HARDWARE NOTES (Ebyte E22 modules):
#   - RF switch control requires BOTH TXEN and RXEN pins:
#     * TX mode: TXEN=HIGH, RXEN=LOW
#     * RX mode: TXEN=LOW,  RXEN=HIGH
#   - DIO2 can be configured to control TXEN (SetDIO2AsRfSwitch)
#   - RXEN must be externally controlled (GPIO or tied to GND for TX-only)
#   - If RXEN is left floating, RF power will NOT reach the antenna!
#
# ==============================================================================

# External dependencies
.extern spi_init
.extern spi_transfer
.extern spi_cs_low
.extern spi_cs_high
.extern lora_reset
.extern lora_wait_busy


.include "drivers/ch32v003/spi.inc"

# ==============================================================================
# SX1262 Command Opcodes
# ==============================================================================

# Operational commands
.equ SX_SET_SLEEP,              0x84
.equ SX_SET_STANDBY,            0x80
.equ SX_SET_FS,                 0xC1
.equ SX_SET_TX,                 0x83
.equ SX_SET_RX,                 0x82
.equ SX_STOP_TIMER_ON_PREAMBLE, 0x9F
.equ SX_SET_CAD,                0xC5
.equ SX_SET_TX_CONTINUOUS,      0xD1
.equ SX_SET_TX_INFINITE_PREAMBLE, 0xD2
.equ SX_SET_REGULATOR_MODE,     0x96
.equ SX_CALIBRATE,              0x89
.equ SX_CALIBRATE_IMAGE,        0x98
.equ SX_SET_PA_CONFIG,          0x95
.equ SX_SET_RX_TX_FALLBACK,     0x93

# Register and buffer access
.equ SX_WRITE_REGISTER,         0x0D
.equ SX_READ_REGISTER,          0x1D
.equ SX_WRITE_BUFFER,           0x0E
.equ SX_READ_BUFFER,            0x1E

# DIO and IRQ commands
.equ SX_SET_DIO_IRQ_PARAMS,     0x08
.equ SX_GET_IRQ_STATUS,         0x12
.equ SX_CLR_IRQ_STATUS,         0x02
.equ SX_SET_DIO2_AS_RF_SW,      0x9D
.equ SX_SET_DIO3_AS_TCXO,       0x97

# RF and packet commands
.equ SX_SET_RF_FREQUENCY,       0x86
.equ SX_SET_PACKET_TYPE,        0x8A
.equ SX_GET_PACKET_TYPE,        0x11
.equ SX_SET_TX_PARAMS,          0x8E
.equ SX_SET_MODULATION_PARAMS,  0x8B
.equ SX_SET_PACKET_PARAMS,      0x8C
.equ SX_SET_BUFFER_BASE_ADDR,   0x8F
.equ SX_SET_LORA_SYMB_TIMEOUT,  0xA0

# Status commands
.equ SX_GET_STATUS,             0xC0
.equ SX_GET_RX_BUFFER_STATUS,   0x13
.equ SX_GET_PACKET_STATUS,      0x14
.equ SX_GET_RSSI_INST,          0x15
.equ SX_GET_STATS,              0x10
.equ SX_RESET_STATS,            0x00
.equ SX_GET_DEVICE_ERRORS,      0x17
.equ SX_CLR_DEVICE_ERRORS,      0x07

# ==============================================================================
# SX1262 Constants
# ==============================================================================

# Standby modes
.equ SX_STDBY_RC,               0x00    # 13MHz RC oscillator
.equ SX_STDBY_XOSC,             0x01    # 32MHz XOSC/TCXO

# Packet types
.equ SX_PKT_TYPE_GFSK,          0x00
.equ SX_PKT_TYPE_LORA,          0x01

# Regulator modes
.equ SX_REGULATOR_LDO,          0x00
.equ SX_REGULATOR_DCDC,         0x01

# PA config (for SX1262 at +22dBm - same as ESP library)
.equ SX_PA_DUTY_CYCLE,          0x04    # Optimal for 22dBm
.equ SX_PA_HP_MAX,              0x07    # For 22dBm
.equ SX_PA_DEVICE_SEL,          0x00    # SX1262

# Ramp times
.equ SX_RAMP_10US,              0x00
.equ SX_RAMP_20US,              0x01
.equ SX_RAMP_40US,              0x02
.equ SX_RAMP_80US,              0x03
.equ SX_RAMP_200US,             0x04
.equ SX_RAMP_800US,             0x05
.equ SX_RAMP_1700US,            0x06
.equ SX_RAMP_3400US,            0x07

# IRQ masks
.equ SX_IRQ_TX_DONE,            (1 << 0)
.equ SX_IRQ_RX_DONE,            (1 << 1)
.equ SX_IRQ_PREAMBLE_DETECTED,  (1 << 2)
.equ SX_IRQ_SYNC_WORD_VALID,    (1 << 3)
.equ SX_IRQ_HEADER_VALID,       (1 << 4)
.equ SX_IRQ_HEADER_ERR,         (1 << 5)
.equ SX_IRQ_CRC_ERR,            (1 << 6)
.equ SX_IRQ_CAD_DONE,           (1 << 7)
.equ SX_IRQ_CAD_DETECTED,       (1 << 8)
.equ SX_IRQ_TIMEOUT,            (1 << 9)
.equ SX_IRQ_ALL,                0x03FF

# LoRa spreading factors
.equ SX_SF5,                    0x05
.equ SX_SF6,                    0x06
.equ SX_SF7,                    0x07
.equ SX_SF8,                    0x08
.equ SX_SF9,                    0x09
.equ SX_SF10,                   0x0A
.equ SX_SF11,                   0x0B
.equ SX_SF12,                   0x0C

# LoRa bandwidths
.equ SX_BW_7,                   0x00    # 7.81 kHz
.equ SX_BW_10,                  0x08    # 10.42 kHz
.equ SX_BW_15,                  0x01    # 15.63 kHz
.equ SX_BW_20,                  0x09    # 20.83 kHz
.equ SX_BW_31,                  0x02    # 31.25 kHz
.equ SX_BW_41,                  0x0A    # 41.67 kHz
.equ SX_BW_62,                  0x03    # 62.50 kHz
.equ SX_BW_125,                 0x04    # 125 kHz
.equ SX_BW_250,                 0x05    # 250 kHz
.equ SX_BW_500,                 0x06    # 500 kHz

# LoRa coding rates
.equ SX_CR_4_5,                 0x01
.equ SX_CR_4_6,                 0x02
.equ SX_CR_4_7,                 0x03
.equ SX_CR_4_8,                 0x04

# ==============================================================================
# TEXT Section
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# sx1262_init - Initialize SX1262 radio
# ==============================================================================
# INPUTS:  None
# OUTPUTS: a0 = 0 on success, -1 on failure
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_init
sx1262_init:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # Initialize SPI at full production speed
    li   a0, 1                      # DIV4 = 12MHz SPI clock (SX1262 max: 16MHz)
    call spi_init

    # Hardware reset
    call lora_reset

    # Wait ~10ms for startup
    li   a0, 120000
1:  addi a0, a0, -1
    bnez a0, 1b

    # Wait for BUSY low
    call lora_wait_busy

    # Set standby mode (RC oscillator)
    li   a0, SX_STDBY_RC
    call sx1262_set_standby

    # Set packet type to LoRa
    li   a0, SX_PKT_TYPE_LORA
    call sx1262_set_packet_type

    # Set DIO2 as RF switch control (common on Ebyte modules)
    li   a0, 1
    call sx1262_set_dio2_rf_switch

    # Set regulator mode (DC-DC for better efficiency)
    li   a0, SX_REGULATOR_DCDC
    call sx1262_set_regulator_mode

    # =========================================================================
    # E22 TCXO Configuration (CRITICAL for Ebyte modules)
    # =========================================================================
    # E22 modules use a 32MHz TCXO powered via DIO3 (internal, not exposed).
    # Must configure BEFORE calibration, otherwise calibration fails.
    #
    # Voltage: 0x07 = 3.3V
    # Delay: 640 ticks × 15.625µs = 10ms for TCXO stabilization
    #
    # NOTE: XOSC_START_ERR (0x20) is EXPECTED after this command per datasheet:
    # "The XOSC_START_ERR flag will be raised at POR or at wake-up from Sleep
    #  mode in a cold-start condition, when a TCXO is used. It is an expected
    #  behaviour... The user should simply clear this flag."
    # =========================================================================
    li   a0, 0x07                   # TCXO voltage 3.3V
    li   a1, 640                    # Delay ~10ms
    call sx1262_set_dio3_tcxo

    # Clear expected XOSC_START_ERR from TCXO cold start
    call sx1262_clear_device_errors

    # Calibrate all blocks (now with valid TCXO clock)
    li   a0, 0x7F
    call sx1262_calibrate

    # Calibrate image for 863-870 MHz band
    # Values from working RNode SX126x implementation:
    #   863-870 MHz: freq1=0xD7, freq2=0xDB
    #   902-928 MHz: freq1=0xE1, freq2=0xE9
    li   a0, 0xD7
    li   a1, 0xDB
    call sx1262_calibrate_image

    # Set overcurrent protection to 60mA (from ESP library)
    li   a0, 0x18               # 60mA / 2.5mA = 24 = 0x18
    call sx1262_set_ocp

    # Use the TCXO/XOSC in standby for subsequent RX/TX operations.
    li   a0, SX_STDBY_XOSC
    call sx1262_set_standby

    # Match upstream SX126x sensitivity tuning on E22 modules.
    li   a0, 0x08AC
    li   a1, 0x96
    call sx1262_write_register

    # Success
    li   a0, 0

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1262_cmd_1 - Send command with 1 byte parameter
# ==============================================================================
# INPUTS:  a0 = command opcode, a1 = parameter byte
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_cmd_1
sx1262_cmd_1:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = opcode (survives all calls)
    mv   s1, a1                 # s1 = param (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    mv   a0, s0
    call spi_transfer

    mv   a0, s1
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_set_standby - Enter standby mode
# ==============================================================================
# INPUTS:  a0 = standby config (SX_STDBY_RC or SX_STDBY_XOSC)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_standby
sx1262_set_standby:
    li   t0, GPIOC_BSHR
    li   t1, LORA_RXEN_LOW
    sw   t1, 0(t0)
    mv   a1, a0
    li   a0, SX_SET_STANDBY
    j    sx1262_cmd_1

# ==============================================================================
# sx1262_set_packet_type - Set packet type (GFSK/LoRa)
# ==============================================================================
# INPUTS:  a0 = packet type (SX_PKT_TYPE_LORA or SX_PKT_TYPE_GFSK)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_packet_type
sx1262_set_packet_type:
    mv   a1, a0
    li   a0, SX_SET_PACKET_TYPE
    j    sx1262_cmd_1

# ==============================================================================
# sx1262_set_regulator_mode - Set power regulator
# ==============================================================================
# INPUTS:  a0 = regulator mode (SX_REGULATOR_LDO or SX_REGULATOR_DCDC)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_regulator_mode
sx1262_set_regulator_mode:
    mv   a1, a0
    li   a0, SX_SET_REGULATOR_MODE
    j    sx1262_cmd_1

# ==============================================================================
# sx1262_set_dio2_rf_switch - Configure DIO2 as RF switch
# ==============================================================================
# INPUTS:  a0 = enable (0 or 1)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_dio2_rf_switch
sx1262_set_dio2_rf_switch:
    mv   a1, a0
    li   a0, SX_SET_DIO2_AS_RF_SW
    j    sx1262_cmd_1

# ==============================================================================
# sx1262_calibrate - Calibrate radio blocks
# ==============================================================================
# INPUTS:  a0 = calibration mask (0x7F for all)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_calibrate
sx1262_calibrate:
    mv   a1, a0
    li   a0, SX_CALIBRATE
    j    sx1262_cmd_1

# ==============================================================================
# sx1262_set_buffer_base - Set TX and RX buffer base addresses
# ==============================================================================
# INPUTS:  a0 = TX base address, a1 = RX base address
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_buffer_base
sx1262_set_buffer_base:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = TX base (survives all calls)
    mv   s1, a1                 # s1 = RX base (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_BUFFER_BASE_ADDR
    call spi_transfer

    mv   a0, s0                 # TX base
    call spi_transfer

    mv   a0, s1                 # RX base
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_set_dio3_tcxo - Configure DIO3 to power TCXO
# ==============================================================================
# PURPOSE:
#   Enable TCXO power supply via DIO3.
#   MUST be called before calibration on modules with TCXO (like E22).
#   DIO3 is internal to the module, not exposed as pin.
#
# INPUTS:  a0 = tcxoVoltage (0x01=1.7V .. 0x07=3.3V), a1 = delay (×15.625µs)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_dio3_tcxo
sx1262_set_dio3_tcxo:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = voltage (survives all calls)
    mv   s1, a1                 # s1 = delay (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_DIO3_AS_TCXO
    call spi_transfer

    # tcxoVoltage byte
    andi a0, s0, 0x07
    call spi_transfer

    # delay[23:16]
    srli a0, s1, 16
    andi a0, a0, 0xFF
    call spi_transfer

    # delay[15:8]
    srli a0, s1, 8
    andi a0, a0, 0xFF
    call spi_transfer

    # delay[7:0]
    andi a0, s1, 0xFF
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_set_ocp - Set overcurrent protection
# ==============================================================================
# INPUTS:  a0 = OCP value (current = value * 2.5mA, e.g., 0x18 = 60mA)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# Register 0x08E7 is the OCP configuration register
# ==============================================================================

.equ SX_REG_OCP,                0x08E7
.equ SX_REG_TX_CLAMP_CONFIG,    0x08D8
.equ SX_REG_LNA,                0x08AC
.equ SX_REG_BW500_FIX,          0x0889  # errata 15.1: bit2=0 when BW=500, 1 otherwise

.globl sx1262_set_ocp
sx1262_set_ocp:
    mv   a1, a0
    li   a0, SX_REG_OCP
    j    sx1262_write_register

# ==============================================================================
# sx1262_calibrate_image - Calibrate for specific frequency band
# ==============================================================================
# INPUTS:  a0 = freq1, a1 = freq2 (e.g., 0xD7/0xDB for 863-870 MHz per SX1262 datasheet)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_calibrate_image
sx1262_calibrate_image:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = freq1 (survives all calls)
    mv   s1, a1                 # s1 = freq2 (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_CALIBRATE_IMAGE
    call spi_transfer

    mv   a0, s0
    call spi_transfer

    mv   a0, s1
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_set_dio_irq - Configure DIO1 IRQ mapping
# ==============================================================================
# PURPOSE:
#   Map specified IRQ sources to DIO1 pin.
#   Used to get hardware interrupt on TX_DONE/RX_DONE.
#
# INPUTS:  a0 = IRQ mask, a1 = DIO1 mask
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# SetDioIrqParams: opcode + IrqMask(16) + DIO1Mask(16) + DIO2Mask(16) + DIO3Mask(16)
# ==============================================================================

.globl sx1262_set_dio_irq
sx1262_set_dio_irq:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = IRQ mask (survives all calls)
    mv   s1, a1                 # s1 = DIO1 mask (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_DIO_IRQ_PARAMS
    call spi_transfer

    # IrqMask high byte
    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer

    # IrqMask low byte
    andi a0, s0, 0xFF
    call spi_transfer

    # DIO1Mask high byte
    srli a0, s1, 8
    andi a0, a0, 0xFF
    call spi_transfer

    # DIO1Mask low byte
    andi a0, s1, 0xFF
    call spi_transfer

    # DIO2Mask = 0
    li   a0, 0
    call spi_transfer
    li   a0, 0
    call spi_transfer

    # DIO3Mask = 0
    li   a0, 0
    call spi_transfer
    li   a0, 0
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_set_frequency - Set RF frequency
# ==============================================================================
# INPUTS:  a0 = pre-calculated freq_reg (freq_hz * 2^25 / 32e6)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# NOTE: 868 MHz = 0x36400000, 915 MHz = 0x39800000
# ==============================================================================

.globl sx1262_set_frequency
sx1262_set_frequency:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = freq_reg (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_RF_FREQUENCY
    call spi_transfer

    # Send 4 bytes (MSB first) — s0 survives every call
    srli a0, s0, 24
    call spi_transfer
    srli a0, s0, 16
    andi a0, a0, 0xFF
    call spi_transfer
    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer
    andi a0, s0, 0xFF
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1262_set_tx_power - Set TX power
# ==============================================================================
# INPUTS:  a0 = power in dBm (-9 to +22 for SX1262)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_tx_power
sx1262_set_tx_power:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = power (survives all calls)

    # Better resistance to antenna mismatch (datasheet section 15.2).
    li   a0, SX_REG_TX_CLAMP_CONFIG
    call sx1262_read_register
    ori  s1, a0, 0x1E
    li   a0, SX_REG_TX_CLAMP_CONFIG
    mv   a1, s1
    call sx1262_write_register

    call lora_wait_busy

    # SetPaConfig for high power
    call spi_cs_low
    li   a0, SX_SET_PA_CONFIG
    call spi_transfer
    li   a0, SX_PA_DUTY_CYCLE   # paDutyCycle
    call spi_transfer
    li   a0, SX_PA_HP_MAX       # hpMax
    call spi_transfer
    li   a0, SX_PA_DEVICE_SEL   # deviceSel (0 = SX1262)
    call spi_transfer
    li   a0, 0x01               # paLut (always 1)
    call spi_transfer
    call spi_cs_high
    call lora_wait_busy

    # SetTxParams
    call spi_cs_low
    li   a0, SX_SET_TX_PARAMS
    call spi_transfer
    mv   a0, s0                 # power from s0
    call spi_transfer
    li   a0, SX_RAMP_40US       # Match upstream SX126x TX ramp
    call spi_transfer
    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_config_lora - Configure LoRa modulation
# ==============================================================================
# INPUTS:  a0 = SF, a1 = BW (SX1262 code), a2 = CR, a3 = LDRO mode (0=off 1=on 2=auto)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# NOTE: CR and LDRO mode kept on stack (only 2 callee-saved regs in RV32E)
# ==============================================================================

.globl sx1262_config_lora
sx1262_config_lora:
    addi sp, sp, -20
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a2, 12(sp)             # CR on stack
    sw   a3, 16(sp)             # LDRO mode on stack
    mv   s0, a0                 # s0 = SF (survives all calls)
    mv   s1, a1                 # s1 = BW (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_MODULATION_PARAMS
    call spi_transfer

    mv   a0, s0                 # SF
    call spi_transfer

    mv   a0, s1                 # BW
    call spi_transfer

    lw   a0, 12(sp)             # CR
    call spi_transfer

    # LDRO: 0=force off, 1=force on, 2=auto based on symbol time >= 16.38 ms
    lw   t2, 16(sp)             # t2 = LDRO mode
    li   t0, 1
    beq  t2, t0, sx1262_config_lora_ldro_enable
    li   t0, 2
    bne  t2, t0, sx1262_config_lora_ldro_off  # mode 0: force off
    # auto: BW62 => SF>=10, BW125 => SF>=11, BW250 => SF>=12, else off
    li   t0, SX_BW_62
    beq  s1, t0, sx1262_config_lora_ldro_bw62
    li   t0, SX_BW_125
    beq  s1, t0, sx1262_config_lora_ldro_bw125
    li   t0, SX_BW_250
    beq  s1, t0, sx1262_config_lora_ldro_bw250
sx1262_config_lora_ldro_off:
    li   a0, 0x00
    j    sx1262_config_lora_ldro_done
sx1262_config_lora_ldro_bw62:
    li   t0, SX_SF10
    bgeu s0, t0, sx1262_config_lora_ldro_enable
    j    sx1262_config_lora_ldro_off
sx1262_config_lora_ldro_bw125:
    li   t0, SX_SF11
    bgeu s0, t0, sx1262_config_lora_ldro_enable
    j    sx1262_config_lora_ldro_off
sx1262_config_lora_ldro_bw250:
    li   t0, SX_SF12
    bgeu s0, t0, sx1262_config_lora_ldro_enable
    j    sx1262_config_lora_ldro_off
sx1262_config_lora_ldro_enable:
    li   a0, 0x01
sx1262_config_lora_ldro_done:
    call spi_transfer

    # Unused trailing ModParam bytes (SetModulationParams is 9 bytes total).
    li   a0, 0x00
    call spi_transfer
    li   a0, 0x00
    call spi_transfer
    li   a0, 0x00
    call spi_transfer
    li   a0, 0x00
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    # Errata 15.1: BW500 degrades modulation quality on remote receivers.
    # Reg 0x0889 bit2: 0 when BW=500, 1 for all other LoRa BWs.
    li   a0, SX_REG_BW500_FIX
    call sx1262_read_register
    li   t0, SX_BW_500
    beq  s1, t0, sx1262_config_lora_bw500_fix_clear
    ori  a0, a0, 0x04
    j    sx1262_config_lora_bw500_fix_write
sx1262_config_lora_bw500_fix_clear:
    andi a0, a0, 0xFB
sx1262_config_lora_bw500_fix_write:
    mv   a1, a0
    li   a0, SX_REG_BW500_FIX
    call sx1262_write_register

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 20
    ret

# ==============================================================================
# sx1262_write_buffer - Write data to TX buffer
# ==============================================================================
# INPUTS:  a0 = buffer pointer, a1 = length
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_write_buffer
sx1262_write_buffer:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = buffer ptr (survives all calls)
    mv   s1, a1                 # s1 = length (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_WRITE_BUFFER
    call spi_transfer

    li   a0, 0x00               # offset = 0
    call spi_transfer

    # Send data bytes — s0, s1 survive all calls by ABI guarantee
sx1262_write_buffer_loop:
    beqz s1, sx1262_write_buffer_done
    lbu  a0, 0(s0)
    addi s0, s0, 1
    addi s1, s1, -1
    call spi_transfer
    j    sx1262_write_buffer_loop

sx1262_write_buffer_done:
    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_write_buffer_head_payload - Write one header byte plus a payload slice
# ==============================================================================
# INPUTS:
#   a0 = header byte
#   a1 = payload pointer
#   a2 = payload length
# CLOBBERS: a0, a1, a2, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_write_buffer_head_payload
sx1262_write_buffer_head_payload:
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a0, 12(sp)             # header byte
    mv   s0, a1                 # s0 = payload ptr
    mv   s1, a2                 # s1 = payload len

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_WRITE_BUFFER
    call spi_transfer

    li   a0, 0x00               # offset = 0
    call spi_transfer

    lw   a0, 12(sp)             # 1-byte RNode modem header
    call spi_transfer

sx1262_write_buffer_head_payload_loop:
    beqz s1, sx1262_write_buffer_head_payload_done
    lbu  a0, 0(s0)
    addi s0, s0, 1
    addi s1, s1, -1
    call spi_transfer
    j    sx1262_write_buffer_head_payload_loop

sx1262_write_buffer_head_payload_done:
    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ==============================================================================
# sx1262_set_tx - Start transmission
# ==============================================================================
# INPUTS:  a0 = timeout (0 = no timeout, 24-bit)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_tx
sx1262_set_tx:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = timeout (survives all calls)

    li   t0, GPIOC_BSHR
    li   t1, LORA_RXEN_LOW
    sw   t1, 0(t0)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_TX
    call spi_transfer

    # 3-byte timeout (MSB first) — s0 survives every call
    srli a0, s0, 16
    andi a0, a0, 0xFF
    call spi_transfer
    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer
    andi a0, s0, 0xFF
    call spi_transfer

    call spi_cs_high
    # Don't wait busy - TX is in progress

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1262_set_rx - Start reception
# ==============================================================================
# INPUTS:  a0 = timeout (0 = single, 0xFFFFFF = continuous)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_rx
sx1262_set_rx:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = timeout (survives all calls)

    li   t0, GPIOC_BSHR
    li   t1, LORA_RXEN_HIGH
    sw   t1, 0(t0)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_RX
    call spi_transfer

    # 3-byte timeout (MSB first) — s0 survives every call
    srli a0, s0, 16
    andi a0, a0, 0xFF
    call spi_transfer
    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer
    andi a0, s0, 0xFF
    call spi_transfer

    call spi_cs_high
    # Don't wait busy - RX is in progress

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1262_get_irq_status - Get IRQ status flags
# ==============================================================================
# OUTPUTS: a0 = IRQ status (16 bits)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_get_irq_status
sx1262_get_irq_status:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_GET_IRQ_STATUS
    call spi_transfer

    li   a0, 0x00               # mandatory NOP byte: SX1262 returns device status here (discard)
    call spi_transfer

    li   a0, 0x00               # IRQ high byte
    call spi_transfer
    mv   s0, a0                 # s0 = high byte (survives calls)

    li   a0, 0x00               # IRQ low byte
    call spi_transfer
    # a0 = low byte — combine with s0
    slli s0, s0, 8
    or   s0, s0, a0             # s0 = (high << 8) | low

    call spi_cs_high

    mv   a0, s0                 # Return result

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1262_get_status - Get radio status byte
# ==============================================================================
# OUTPUTS: a0 = SX1262 status byte
# CLOBBERS: a0, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_get_status
sx1262_get_status:
    addi sp, sp, -4
    sw   ra, 0(sp)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_GET_STATUS
    call spi_transfer

    li   a0, 0x00
    call spi_transfer
    andi a0, a0, 0xFF

    call spi_cs_high

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1262_clear_irq - Clear IRQ flags
# ==============================================================================
# INPUTS:  a0 = IRQ mask to clear
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_clear_irq
sx1262_clear_irq:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = IRQ mask (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_CLR_IRQ_STATUS
    call spi_transfer

    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer

    andi a0, s0, 0xFF
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret


# ==============================================================================
# sx1262_clear_device_errors - Clear device error flags
# ==============================================================================
# PURPOSE:
#   Clear all device error flags. MUST be called after SetDIO3AsTCXOCtrl
#   because XOSC_START_ERR is expected when using TCXO (per datasheet).
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_clear_device_errors
sx1262_clear_device_errors:
    addi sp, sp, -4
    sw   ra, 0(sp)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_CLR_DEVICE_ERRORS
    call spi_transfer

    # Two zero bytes required
    li   a0, 0x00
    call spi_transfer
    li   a0, 0x00
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# Register addresses (for ReadRegister command)
# ==============================================================================
.equ SX_REG_LORA_SYNC_MSB,      0x0740  # LoRa Sync Word MSB (default 0x14)
.equ SX_REG_LORA_SYNC_LSB,      0x0741  # LoRa Sync Word LSB (default 0x24)
.equ SX_REG_IQ_POLARITY,        0x0736  # IQ polarity fix register


# ==============================================================================
# sx1262_read_register - Read a 8-bit register
# ==============================================================================
# INPUTS:  a0 = register address (16-bit)
# OUTPUTS: a0 = register value
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_read_register
sx1262_read_register:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = address (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_READ_REGISTER
    call spi_transfer

    # Address high byte
    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer

    # Address low byte
    andi a0, s0, 0xFF
    call spi_transfer

    # Byte 3: NOP - returns status (discard)
    li   a0, 0x00
    call spi_transfer

    # Byte 4: NOP - returns data@address
    li   a0, 0x00
    call spi_transfer
    andi s0, a0, 0xFF           # s0 = result (survives spi_cs_high)

    call spi_cs_high

    mv   a0, s0                 # Return value

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1262_write_register - Write a 8-bit register
# ==============================================================================
# INPUTS:  a0 = register address (16-bit), a1 = value to write (8-bit)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_write_register
sx1262_write_register:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = address (survives all calls)
    mv   s1, a1                 # s1 = value (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_WRITE_REGISTER
    call spi_transfer

    # Address high byte
    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer

    # Address low byte
    andi a0, s0, 0xFF
    call spi_transfer

    # Data byte
    andi a0, s1, 0xFF
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_set_sync_word - Set LoRa sync word
# ==============================================================================
# INPUTS:  a0 = sync word (16-bit, e.g., 0x1424 for public, 0x3444 for private)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# NOTE: SX1262 sync word mapping from SX127x:
#   - SX127x 0x12 (public)  → SX1262 0x3444
#   - SX127x 0x34 (private) → SX1262 0x1424
#   Yes, it's inverted from what you'd expect!
# ==============================================================================

.globl sx1262_set_sync_word
sx1262_set_sync_word:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = sync word (survives all calls)

    # Write MSB to 0x0740
    li   a0, SX_REG_LORA_SYNC_MSB
    srli a1, s0, 8
    andi a1, a1, 0xFF
    call sx1262_write_register

    # Write LSB to 0x0741
    li   a0, SX_REG_LORA_SYNC_LSB
    andi a1, s0, 0xFF
    call sx1262_write_register

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret



# ==============================================================================
# sx1262_set_iq_polarity - Configure IQ polarity (errata 15.4 register workaround)
# ==============================================================================
# PURPOSE:
#   Fix known SX1262 hardware bug (datasheet section 15.4).
#   When using inverted IQ, packet losses occur for longer packets.
#   Bit 2 at address 0x0736 must be set correctly.
#
# INPUTS:  a0 = IQ setting (0 = standard, 1 = inverted)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_set_iq_polarity
sx1262_set_iq_polarity:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = IQ setting (survives all calls)

    # Read current register value
    li   a0, SX_REG_IQ_POLARITY
    call sx1262_read_register
    mv   s1, a0                 # s1 = current value (survives all calls)

    # Check if inverted or standard
    beqz s0, 1f

    # Inverted IQ: clear bit 2
    andi s1, s1, 0xFB
    j    2f

1:  # Standard IQ: set bit 2
    ori  s1, s1, 0x04

2:  # Write back
    li   a0, SX_REG_IQ_POLARITY
    mv   a1, s1
    call sx1262_write_register

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_set_packet_params - Configure LoRa packet parameters
# ==============================================================================
# INPUTS:  a0 = preamble length (symbols), a1 = payload length (bytes),
#          a2 = implicit flag (0=explicit header, 1=implicit/fixed length),
#          a3 = CRC (0=off, 1=on), a4 = IQI (0=standard, 1=inverted)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# NOTE: also applies the IQ polarity register fix (errata 15.4)
# ==============================================================================

.globl sx1262_set_packet_params
sx1262_set_packet_params:
    addi sp, sp, -20
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a3, 12(sp)             # CRC on stack
    sw   a4, 16(sp)             # IQI on stack
    mv   s0, a0                 # s0 = preamble (survives all calls)
    mv   s1, a1                 # s1 = payload len (survives all calls)
    # a2 = implicit flag (used immediately, no need to save)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_SET_PACKET_PARAMS
    call spi_transfer

    # Preamble length MSB
    srli a0, s0, 8
    andi a0, a0, 0xFF
    call spi_transfer

    # Preamble length LSB
    andi a0, s0, 0xFF
    call spi_transfer

    # Header type: 0=explicit (variable length), 1=implicit (fixed length)
    andi a0, a2, 0x01
    call spi_transfer

    # Payload length
    andi a0, s1, 0xFF
    call spi_transfer

    # CRC: 0=off, 1=on
    lw   a0, 12(sp)
    andi a0, a0, 0x01
    call spi_transfer

    # InvertIQ: 0=standard, 1=inverted
    lw   a0, 16(sp)
    andi a0, a0, 0x01
    call spi_transfer

    # Unused trailing params.
    li   a0, 0x00
    call spi_transfer
    li   a0, 0x00
    call spi_transfer
    li   a0, 0x00
    call spi_transfer

    call spi_cs_high
    call lora_wait_busy

    # Errata 15.4: IQ polarity register must match InvertIQ setting.
    lw   a0, 16(sp)
    call sx1262_set_iq_polarity

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 20
    ret


# ==============================================================================
# sx1262_get_rx_buffer_status - Get RX payload length and buffer offset
# ==============================================================================
# PURPOSE:
#   Query radio for received payload size and start offset in buffer.
#   SPI command 0x13: opcode + NOP(status) + NOP(PayloadLen) + NOP(RxStartPtr)
#
# OUTPUTS: a0 = payload length (bytes), a1 = RX buffer start offset
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_get_rx_buffer_status
sx1262_get_rx_buffer_status:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_GET_RX_BUFFER_STATUS
    call spi_transfer

    li   a0, 0x00               # NOP for status byte (discard)
    call spi_transfer

    li   a0, 0x00               # NOP for PayloadLengthRx
    call spi_transfer
    andi s0, a0, 0xFF           # s0 = payload length (survives calls)

    li   a0, 0x00               # NOP for RxStartBufferPointer
    call spi_transfer
    andi s1, a0, 0xFF           # s1 = buffer offset (survives calls)

    call spi_cs_high

    mv   a0, s0                 # a0 = payload length
    mv   a1, s1                 # a1 = buffer offset

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_get_packet_status - Get LoRa packet RSSI and SNR
# ==============================================================================
# PURPOSE:
#   Read signal quality of last received packet.
#   SPI command 0x14: opcode + NOP(status) + RssiPkt + SnrPkt + SignalRssiPkt
#
# OUTPUTS: a0 = RssiPkt raw (RSSI in dBm = -a0/2)
#          a1 = SnrPkt raw (SNR in dB = a1/4, signed)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_get_packet_status
sx1262_get_packet_status:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_GET_PACKET_STATUS
    call spi_transfer

    li   a0, 0x00               # NOP for status (discard)
    call spi_transfer

    li   a0, 0x00               # RssiPkt
    call spi_transfer
    andi s0, a0, 0xFF           # s0 = RSSI raw (survives calls)

    li   a0, 0x00               # SnrPkt
    call spi_transfer
    andi s1, a0, 0xFF           # s1 = SNR raw (survives calls)

    li   a0, 0x00               # SignalRssiPkt (discard)
    call spi_transfer

    call spi_cs_high

    mv   a0, s0                 # a0 = RssiPkt raw
    mv   a1, s1                 # a1 = SnrPkt raw

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1262_read_buffer - Read data from SX1262 RX buffer
# ==============================================================================
# PURPOSE:
#   Read received payload from radio's internal buffer into MCU RAM.
#   SPI command 0x1E: opcode + offset + NOP(status) + NOP×N (data bytes)
#
# INPUTS:  a0 = MCU dest buffer pointer, a1 = SX1262 buffer offset, a2 = length
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1262_read_buffer
sx1262_read_buffer:
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a1, 12(sp)             # buffer offset (used once, then discarded)
    mv   s0, a0                 # s0 = dest buffer ptr (survives all calls)
    mv   s1, a2                 # s1 = length (survives all calls)

    call lora_wait_busy
    call spi_cs_low

    li   a0, SX_READ_BUFFER
    call spi_transfer

    lw   a0, 12(sp)             # offset byte
    call spi_transfer

    li   a0, 0x00               # NOP for status byte (discard)
    call spi_transfer

    # Read data bytes — s0, s1 survive all calls by ABI guarantee
sx1262_read_buffer_loop:
    beqz s1, sx1262_read_buffer_done

    li   a0, 0x00               # NOP to clock out data byte
    call spi_transfer
    sb   a0, 0(s0)              # Store to MCU buffer
    addi s0, s0, 1
    addi s1, s1, -1
    j    sx1262_read_buffer_loop

sx1262_read_buffer_done:
    call spi_cs_high

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret
