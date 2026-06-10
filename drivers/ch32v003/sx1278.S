# ==============================================================================
# drivers/ch32v003/sx1278.S - SX1278 LoRa Radio Driver
# ==============================================================================
#
# PURPOSE:
#   Low-level driver for Semtech SX1276/77/78/79 LoRa transceiver.
#   Used with Ebyte E32-400M30S modules (SX1278, 400 MHz LF band).
#
# SX1278 CHARACTERISTICS:
#   - SPI: register-based (addr byte + data)
#   - No BUSY pin: never call lora_wait_busy
#   - IRQ flags: 8-bit (RegIrqFlags 0x12), cleared by writing 1
#   - DIO0: main IRQ pin (TxDone / RxDone)
#   - Frequency step: F_step = 32 MHz / 2^19 = 61.035 Hz
#     => Frf = freq_hz * 2^19 / 32e6
#   - No image calibration command
#   - LowFrequencyModeOn must be set for 400 MHz band operation
#
# PIN MAPPING:
#   PC0 = RESET  (output, active low)
#   PC1 = TX_EN  (GPIO output, active high - E32 RF switch TX enable)
#   PC2 = DIO0   (EXTI source: TxDone or RxDone depending on mode)
#   PC3 = RX_EN  (GPIO output, active high - E32 RF switch RX enable)
#   PC4 = CS     (SPI chip select, active low)
#   PC5 = SCK
#   PC6 = MOSI
#   PC7 = MISO
#
# SPI PROTOCOL (CPOL=0, CPHA=0 - Motorola mode 0):
#   Write reg:  CS low | send (0x80 | addr) | send data | CS high
#   Read reg:   CS low | send (addr & 0x7F) | recv data | CS high
#   Burst FIFO: CS low | send (0x80 | 0x00) | send N bytes | CS high
#               (FIFO addr 0x00 does NOT auto-increment)
#
# PA CONFIGURATION (E32-400M30S module):
#   The E32 module routes the SX1278 PA_BOOST output through an external
#   power amplifier stage to achieve up to 30 dBm output. PA_BOOST pin
#   (PaSelect=1) must be used. The SX1278 itself is configured for up
#   to +17 dBm; the module PA adds the remaining gain.
#
# ABI CONTRACT (RV32E calling convention):
#   CLOBBERS:  a0, a1, t0, t1, t2
#   PRESERVES: s0, s1, ra (saved/restored in prologue/epilogue)
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 10 (Layer Roles): Hardware-only, no domain logic
#   - Article 11 (ABI Sanctity): Explicit interface, ABI preserved
#
# ==============================================================================

.extern spi_init
.extern spi_transfer
.extern spi_cs_low
.extern spi_cs_high
.extern lora_reset

.include "drivers/ch32v003/spi.inc"

# ==============================================================================
# SX1278 Register Map (LoRa mode)
# ==============================================================================

.equ REG_FIFO,                  0x00    # FIFO data input/output
.equ REG_OP_MODE,               0x01    # Operating mode
.equ REG_FR_MSB,                0x06    # RF carrier frequency MSB
.equ REG_FR_MID,                0x07    # RF carrier frequency MID
.equ REG_FR_LSB,                0x08    # RF carrier frequency LSB
.equ REG_PA_CONFIG,             0x09    # PA selection and output power
.equ REG_PA_RAMP,               0x0A    # PA ramp time
.equ REG_OCP,                   0x0B    # Overcurrent protection
.equ REG_LNA,                   0x0C    # LNA gain
.equ REG_FIFO_ADDR_PTR,         0x0D    # SPI FIFO pointer
.equ REG_FIFO_TX_BASE_ADDR,     0x0E    # TX base address in FIFO
.equ REG_FIFO_RX_BASE_ADDR,     0x0F    # RX base address in FIFO
.equ REG_FIFO_RX_CURRENT_ADDR,  0x10    # Start of last received packet
.equ REG_IRQ_FLAGS_MASK,        0x11    # IRQ mask (1 = masked)
.equ REG_IRQ_FLAGS,             0x12    # IRQ flags (write 1 to clear)
.equ REG_RX_NB_BYTES,           0x13    # Number of received bytes
.equ REG_PKT_SNR_VALUE,         0x19    # Last packet SNR (signed /4)
.equ REG_PKT_RSSI_VALUE,        0x1A    # Last packet RSSI raw
.equ REG_RSSI_VALUE,            0x1B    # Current RSSI
.equ REG_MODEM_CONFIG1,         0x1D    # BW, CR, header mode
.equ REG_MODEM_CONFIG2,         0x1E    # SF, CRC, RX timeout MSB
.equ REG_SYMB_TIMEOUT_LSB,      0x1F    # RX symbol timeout LSB
.equ REG_PREAMBLE_MSB,          0x20    # Preamble length MSB
.equ REG_PREAMBLE_LSB,          0x21    # Preamble length LSB
.equ REG_PAYLOAD_LENGTH,        0x22    # Payload length (implicit mode)
.equ REG_MAX_PAYLOAD_LENGTH,    0x23    # Max payload length
.equ REG_MODEM_CONFIG3,         0x26    # LDRO, AGC auto
.equ REG_FEI_MSB,               0x28    # Frequency error estimate MSB (LoRa)
.equ REG_FEI_MID,               0x29    # Frequency error estimate MID
.equ REG_FEI_LSB,               0x2A    # Frequency error estimate LSB
.equ REG_DETECT_OPTIMIZE,       0x31    # Detection optimize (SF6 vs SF7-12)
.equ REG_INVERT_IQ,             0x33    # IQ inversion
.equ REG_DETECTION_THRESHOLD,   0x37    # Detection threshold
.equ REG_SYNC_WORD,             0x39    # LoRa sync word
.equ REG_INVERT_IQ2,            0x3B    # IQ inversion companion register
.equ REG_DIO_MAPPING1,          0x40    # DIO0..DIO3 mapping
.equ REG_DIO_MAPPING2,          0x41    # DIO4, DIO5 mapping
.equ REG_PA_DAC,                0x4D    # High power +20 dBm control

# ==============================================================================
# RegOpMode values (LoRa, LF band = 400 MHz)
# ==============================================================================
# Bit 7: LongRangeMode (1=LoRa, can only change in SLEEP)
# Bit 3: LowFrequencyModeOn (1=LF band registers, required for 400 MHz)
# Bits 2:0: Mode (000=SLEEP 001=STDBY 011=TX 101=RXCONTINUOUS 110=RXSINGLE)

.equ OPMODE_SLEEP,              0x88    # LoRa, LF, SLEEP
.equ OPMODE_STDBY,              0x89    # LoRa, LF, STDBY
.equ OPMODE_TX,                 0x8B    # LoRa, LF, TX
.equ OPMODE_RXCONTINUOUS,       0x8D    # LoRa, LF, RXCONTINUOUS
.equ OPMODE_RXSINGLE,           0x8E    # LoRa, LF, RXSINGLE

# ==============================================================================
# IRQ flag bits (RegIrqFlags 0x12)
# ==============================================================================

.equ IRQ_RX_TIMEOUT,            (1 << 7)
.equ IRQ_RX_DONE,               (1 << 6)
.equ IRQ_PAYLOAD_CRC_ERROR,     (1 << 5)
.equ IRQ_VALID_HEADER,          (1 << 4)
.equ IRQ_TX_DONE,               (1 << 3)
.equ IRQ_CAD_DONE,              (1 << 2)
.equ IRQ_ALL,                   0xFF

# ==============================================================================
# RegDioMapping1 values for DIO0
# ==============================================================================

.equ DIO0_RX_DONE,              0x00    # DIO0 = RxDone (RX mode)
.equ DIO0_TX_DONE,              0x40    # DIO0 = TxDone (TX mode)
.equ DIO1_RX_TIMEOUT,           0x00    # DIO1 = RxTimeout (default)

# ==============================================================================
# BW values for RegModemConfig1 bits [7:4]
# ==============================================================================

.equ BW_7K8,                    0x00
.equ BW_10K4,                   0x10
.equ BW_15K6,                   0x20
.equ BW_20K8,                   0x30
.equ BW_31K25,                  0x40
.equ BW_41K7,                   0x50
.equ BW_62K5,                   0x60
.equ BW_125K,                   0x70
.equ BW_250K,                   0x80
.equ BW_500K,                   0x90

# CR values for RegModemConfig1 bits [3:1]

.equ CR_4_5,                    0x02
.equ CR_4_6,                    0x04
.equ CR_4_7,                    0x06
.equ CR_4_8,                    0x08

# ==============================================================================
# TEXT Section
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# sx1278_write_reg - Write a single register
# ==============================================================================
# INPUTS:  a0 = register address (7-bit), a1 = value to write
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_write_reg
sx1278_write_reg:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = address
    mv   s1, a1                 # s1 = value

    call spi_cs_low

    ori  a0, s0, 0x80           # set write bit (MSB = 1)
    call spi_transfer

    mv   a0, s1
    call spi_transfer

    call spi_cs_high

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1278_read_reg - Read a single register
# ==============================================================================
# INPUTS:  a0 = register address (7-bit)
# OUTPUTS: a0 = register value
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_read_reg
sx1278_read_reg:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = address

    call spi_cs_low

    andi a0, s0, 0x7F           # clear write bit (MSB = 0 = read)
    call spi_transfer

    li   a0, 0x00               # clock out data byte
    call spi_transfer
    andi s0, a0, 0xFF           # s0 = result

    call spi_cs_high

    mv   a0, s0

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1278_init - Initialize SX1278 radio
# ==============================================================================
# INPUTS:  None
# OUTPUTS: a0 = 0 on success
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# SEQUENCE:
#   1. SPI init (6 MHz = DIV8 for 48 MHz AHB)
#   2. Hardware reset
#   3. Wait 10 ms for POR
#   4. Enter FSK SLEEP then switch to LoRa mode (LongRangeMode only in SLEEP)
#   5. Enter LoRa STDBY
#   6. Set FIFO base addresses (TX=0x80, RX=0x00)
#   7. Configure LNA: max gain + HF boost
#   8. Enable AGC auto
# ==============================================================================

.globl sx1278_init
sx1278_init:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # SPI at DIV8 = 6 MHz (safe margin under SX1278 10 MHz SPI max)
    li   a0, 2
    call spi_init

    # Hardware reset: lora_reset pulls RESET low ~1 ms then releases (POR wait below)
    call lora_reset

    # Wait ~10 ms (480000 cycles at 48 MHz)
    li   a0, 480000
1:  addi a0, a0, -1
    bnez a0, 1b

    # Ensure FSK SLEEP (to guarantee LongRangeMode can be written next)
    li   a0, REG_OP_MODE
    li   a1, 0x00               # FSK, SLEEP
    call sx1278_write_reg

    # Short delay for mode transition
    li   a0, 1000
2:  addi a0, a0, -1
    bnez a0, 2b

    # Switch to LoRa mode in SLEEP (LongRangeMode=1, LowFrequencyModeOn=1)
    li   a0, REG_OP_MODE
    li   a1, OPMODE_SLEEP
    call sx1278_write_reg

    # Short delay for LoRa mode to settle
    li   a0, 1000
3:  addi a0, a0, -1
    bnez a0, 3b

    # Enter STDBY
    li   a0, REG_OP_MODE
    li   a1, OPMODE_STDBY
    call sx1278_write_reg

    # FIFO base addresses: TX at 0x80, RX at 0x00 (non-overlapping 128 byte halves)
    li   a0, REG_FIFO_TX_BASE_ADDR
    li   a1, 0x80
    call sx1278_write_reg

    li   a0, REG_FIFO_RX_BASE_ADDR
    li   a1, 0x00
    call sx1278_write_reg

    # LNA: max gain (G1), HF LNA boost 150% (LnaBoostHf=11)
    # RegLna[7:5]=001(G1) [4:3]=00(LF default) [2]=0(reserved) [1:0]=11(boost)
    li   a0, REG_LNA
    li   a1, 0x23
    call sx1278_write_reg

    # ModemConfig3: AGC auto on (bit 2), LDRO off (set per packet config)
    li   a0, REG_MODEM_CONFIG3
    li   a1, 0x04
    call sx1278_write_reg

    li   a0, 0
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_set_standby - Enter LoRa STDBY mode
# ==============================================================================
# INPUTS:  a0 = ignored
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_set_standby
sx1278_set_standby:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # RF switch: both off in STDBY
    li   t0, GPIOC_BSHR
    li   t1, (LORA_TXEN_LOW | LORA_RXEN_LOW)
    sw   t1, 0(t0)

    li   a0, REG_OP_MODE
    li   a1, OPMODE_STDBY
    call sx1278_write_reg

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_set_sleep - Enter LoRa SLEEP mode
# ==============================================================================
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_set_sleep
sx1278_set_sleep:
    addi sp, sp, -4
    sw   ra, 0(sp)

    li   a0, REG_OP_MODE
    li   a1, OPMODE_SLEEP
    call sx1278_write_reg

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_set_frequency - Set RF carrier frequency
# ==============================================================================
# INPUTS:  a0 = Frf (24-bit, pre-calculated: freq_hz * 2^19 / 32000000)
#               Example: 434 MHz -> 0x6C8000
#                        433.175 MHz -> 0x6C4B33
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# NOTE: Must be called in STDBY or SLEEP mode (not during TX/RX).
# ==============================================================================

.globl sx1278_set_frequency
sx1278_set_frequency:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = Frf (survives all calls)

    srli a1, s0, 16
    andi a1, a1, 0xFF
    li   a0, REG_FR_MSB
    call sx1278_write_reg

    srli a1, s0, 8
    andi a1, a1, 0xFF
    li   a0, REG_FR_MID
    call sx1278_write_reg

    andi a1, s0, 0xFF
    li   a0, REG_FR_LSB
    call sx1278_write_reg

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1278_set_tx_power - Set TX output power
# ==============================================================================
# INPUTS:  a0 = power in dBm (2 to 17 for PA_BOOST on SX1278)
#               The E32-400M30S external PA adds additional gain.
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# PA_BOOST path (PaSelect=1): Pout = 17 - (15 - OutputPower) [dBm]
#   => OutputPower = power - 2  (valid range: power 2..17 dBm)
#
# OCP set to 150 mA (OcpTrim=0x12 -> Imax = -30 + 10*18; OcpOn=1).
# PA ramp: 40 us (PaRamp=0x08).
# ==============================================================================

.globl sx1278_set_tx_power
sx1278_set_tx_power:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a0                 # s0 = requested power dBm

    # Clamp to [2, 17]
    li   t0, 2
    bge  s0, t0, 1f
    mv   s0, t0
1:  li   t0, 17
    ble  s0, t0, 2f
    mv   s0, t0

    # RegPaConfig: PaSelect=1 (PA_BOOST), MaxPower=7 (bits 6:4), OutputPower=power-2
2:  addi s0, s0, -2             # OutputPower = power - 2
    andi s0, s0, 0x0F           # mask to 4 bits
    ori  s0, s0, 0xF0           # PaSelect=1 | MaxPower=7 | OutputPower

    li   a0, REG_PA_CONFIG
    mv   a1, s0
    call sx1278_write_reg

    # OCP: enabled, OcpTrim=0x12 -> Imax = -30 + 10*18 = 150 mA
    li   a0, REG_OCP
    li   a1, 0x32               # OcpOn=1 (bit5), OcpTrim=0x12
    call sx1278_write_reg

    # PA ramp: 40 us
    li   a0, REG_PA_RAMP
    li   a1, 0x08               # 40 us ramp
    call sx1278_write_reg

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ==============================================================================
# sx1278_config_lora - Configure LoRa modulation parameters
# ==============================================================================
# INPUTS:
#   a0 = spreading factor (6 to 12)
#   a1 = bandwidth code (BW_7K8 .. BW_500K, upper nibble of RegModemConfig1)
#   a2 = coding rate code (CR_4_5 .. CR_4_8, bits [3:1] of RegModemConfig1)
#   a3 = CRC PHY enable (0=off, non-zero=on)
#   a4 = LDRO mode (0=force off, 1=force on, 2=auto)
#
# REGISTERS WRITTEN:
#   RegModemConfig1 (0x1D): Bw[7:4] | CR[3:1], header set by packet params
#   RegModemConfig2 (0x1E): SF[7:4] | TxCont=0 | CrcOn=a3 | SymbTimeout=0
#   RegModemConfig3 (0x26): LDRO from a4 | AgcAutoOn=1
#   RegDetectOptimize (0x31): 0x03 (SF7-12) or 0x05 (SF6)
#   RegDetectionThreshold (0x37): 0x0A (SF7-12) or 0x0C (SF6)
#
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# NOTE: CR and SF must be saved to stack (only 2 callee-saved regs in RV32E)
# ==============================================================================

.globl sx1278_config_lora
sx1278_config_lora:
    addi sp, sp, -24
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a2, 12(sp)             # CR on stack
    sw   a3, 16(sp)             # CRC enable
    sw   a4, 20(sp)             # LDRO mode
    mv   s0, a0                 # s0 = SF
    mv   s1, a1                 # s1 = BW code (upper nibble)

    # RegModemConfig1: BW[7:4] | CR[3:1]. Header mode is applied later by
    # sx1278_set_packet_params after the packet length profile is known.
    lw   t0, 12(sp)             # CR
    or   t0, t0, s1             # BW | CR
    andi t0, t0, 0xFE           # explicit header (ImplicitHeaderModeOn=0)
    li   a0, REG_MODEM_CONFIG1
    mv   a1, t0
    call sx1278_write_reg

    # RegModemConfig2: SF[7:4] | TxCont=0 | CrcOn | SymbTimeout[9:8]=0
    slli t0, s0, 4              # SF in bits [7:4]
    andi t0, t0, 0xF0
    lw   t1, 16(sp)
    beqz t1, sx1278_crc_done
    ori  t0, t0, 0x04           # RxPayloadCrcOn = 1
sx1278_crc_done:
    li   a0, REG_MODEM_CONFIG2
    mv   a1, t0
    call sx1278_write_reg

    li   t0, 0x04               # AgcAutoOn=1, LDRO=0 (default)
    lw   t1, 20(sp)             # LDRO mode: 0=off, 1=on, 2=auto
    beqz t1, ldro_done
    li   t2, 1
    beq  t1, t2, ldro_enable

    # Auto LDRO: required when symbol duration > 16 ms.
    # RadioLib autoLDRO() follows T_sym = 2^SF / BW. Keep the thresholds
    # explicit so each BW/SF cause remains distinguishable in the register image.
    li   t1, BW_7K8
    beq  s1, t1, check_ldro_sf7
    li   t1, BW_10K4
    beq  s1, t1, check_ldro_sf8
    li   t1, BW_15K6
    beq  s1, t1, check_ldro_sf8
    li   t1, BW_20K8
    beq  s1, t1, check_ldro_sf9
    li   t1, BW_31K25
    beq  s1, t1, check_ldro_sf9
    li   t1, BW_41K7
    beq  s1, t1, check_ldro_sf10
    li   t1, BW_62K5
    beq  s1, t1, check_ldro_sf10
    li   t1, BW_125K
    beq  s1, t1, check_ldro_sf11
    li   t1, BW_250K
    beq  s1, t1, check_ldro_sf12
    j    ldro_done

check_ldro_sf7:
    li   t1, 7
    bge  s0, t1, ldro_enable
    j    ldro_done

check_ldro_sf8:
    li   t1, 8
    bge  s0, t1, ldro_enable
    j    ldro_done

check_ldro_sf9:
    li   t1, 9
    bge  s0, t1, ldro_enable
    j    ldro_done

check_ldro_sf10:
    li   t1, 10
    bge  s0, t1, ldro_enable
    j    ldro_done

check_ldro_sf11:
    li   t1, 11
    bge  s0, t1, ldro_enable
    j    ldro_done

check_ldro_sf12:
    li   t1, 12
    bge  s0, t1, ldro_enable
    j    ldro_done

ldro_enable:
    ori  t0, t0, 0x08           # LowDataRateOptimize = 1

ldro_done:
    li   a0, REG_MODEM_CONFIG3
    mv   a1, t0
    call sx1278_write_reg

    # Detection optimize and threshold (differ for SF6 vs SF7-12)
    li   t0, 6
    beq  s0, t0, detect_sf6

    # SF7-12
    li   a0, REG_DETECT_OPTIMIZE
    li   a1, 0x03
    call sx1278_write_reg
    li   a0, REG_DETECTION_THRESHOLD
    li   a1, 0x0A
    call sx1278_write_reg
    j    detect_done

detect_sf6:
    li   a0, REG_DETECT_OPTIMIZE
    li   a1, 0x05
    call sx1278_write_reg
    li   a0, REG_DETECTION_THRESHOLD
    li   a1, 0x0C
    call sx1278_write_reg

detect_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 24
    ret

# ==============================================================================
# sx1278_set_sync_word - Set LoRa sync word
# ==============================================================================
# INPUTS:  a0 = sync word (8-bit used; 0x12=private LoRa, 0x34=LoRaWAN)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_set_sync_word
sx1278_set_sync_word:
    addi sp, sp, -4
    sw   ra, 0(sp)

    mv   a1, a0
    andi a1, a1, 0xFF
    li   a0, REG_SYNC_WORD
    call sx1278_write_reg

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_set_iq_invert - Configure LoRa IQ inversion
# ==============================================================================
# INPUTS:  a0 = 0 normal IQ, non-zero inverted IQ
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# SX127x LoRa convention:
#   normal   RegInvertIQ=0x27, RegInvertIQ2=0x1D
#   inverted RegInvertIQ=0x66, RegInvertIQ2=0x19
# ==============================================================================

.globl sx1278_set_iq_invert
sx1278_set_iq_invert:
    addi sp, sp, -4
    sw   ra, 0(sp)

    beqz a0, sx1278_set_iq_normal

    li   a0, REG_INVERT_IQ
    li   a1, 0x66
    call sx1278_write_reg
    li   a0, REG_INVERT_IQ2
    li   a1, 0x19
    call sx1278_write_reg
    j    sx1278_set_iq_done

sx1278_set_iq_normal:
    li   a0, REG_INVERT_IQ
    li   a1, 0x27
    call sx1278_write_reg
    li   a0, REG_INVERT_IQ2
    li   a1, 0x1D
    call sx1278_write_reg

sx1278_set_iq_done:
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_set_packet_params - Configure LoRa packet parameters
# ==============================================================================
# INPUTS:
#   a0 = preamble length (symbols, typically 8)
#   a1 = payload length (bytes; used only in implicit header mode, set anyway)
#   a2 = implicit header flag (0=explicit, non-zero=implicit)
# REGISTERS WRITTEN:
#   RegPreambleMsb/Lsb (0x20/0x21): preamble length
#   RegPayloadLength (0x22): payload length
#   RegMaxPayloadLength (0x23): 0xFF (no filtering)
# Header mode follows a2. CRC is set in sx1278_config_lora via ModemConfig2.
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_set_packet_params
sx1278_set_packet_params:
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a2, 12(sp)
    mv   s0, a0                 # s0 = preamble length
    mv   s1, a1                 # s1 = payload length

    srli a1, s0, 8
    andi a1, a1, 0xFF
    li   a0, REG_PREAMBLE_MSB
    call sx1278_write_reg

    andi a1, s0, 0xFF
    li   a0, REG_PREAMBLE_LSB
    call sx1278_write_reg

    mv   a1, s1
    li   a0, REG_PAYLOAD_LENGTH
    call sx1278_write_reg

    li   a0, REG_MAX_PAYLOAD_LENGTH
    li   a1, 0xFF
    call sx1278_write_reg

    li   a0, REG_MODEM_CONFIG1
    call sx1278_read_reg
    lw   t0, 12(sp)
    beqz t0, sx1278_pkt_explicit
    ori  a1, a0, 0x01           # ImplicitHeaderModeOn=1
    j    sx1278_pkt_header_write
sx1278_pkt_explicit:
    andi a1, a0, 0xFE           # ImplicitHeaderModeOn=0
sx1278_pkt_header_write:
    li   a0, REG_MODEM_CONFIG1
    call sx1278_write_reg

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ==============================================================================
# sx1278_set_buffer_base - Set FIFO TX and RX base addresses
# ==============================================================================
# INPUTS:  a0 = TX base address (typically 0x80)
#          a1 = RX base address (typically 0x00)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_set_buffer_base
sx1278_set_buffer_base:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = TX base
    mv   s1, a1                 # s1 = RX base

    mv   a1, s0
    li   a0, REG_FIFO_TX_BASE_ADDR
    call sx1278_write_reg

    mv   a1, s1
    li   a0, REG_FIFO_RX_BASE_ADDR
    call sx1278_write_reg

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1278_write_buffer - Write payload bytes to TX FIFO
# ==============================================================================
# INPUTS:  a0 = source buffer pointer, a1 = length (bytes)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# SEQUENCE:
#   1. Read RegFifoTxBaseAddr (0x0E) to get TX base
#   2. Write RegFifoAddrPtr (0x0D) = TX base
#   3. Burst write to RegFifo (0x00): send data bytes
# ==============================================================================

.globl sx1278_write_buffer
sx1278_write_buffer:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0                 # s0 = source ptr
    mv   s1, a1                 # s1 = length

    # Point FIFO ptr to TX base
    li   a0, REG_FIFO_TX_BASE_ADDR
    call sx1278_read_reg        # a0 = TX base value
    mv   a1, a0
    li   a0, REG_FIFO_ADDR_PTR
    call sx1278_write_reg

    # Burst write to FIFO (address 0x00, no auto-increment for FIFO)
    call spi_cs_low
    li   a0, (0x80 | REG_FIFO)  # write bit set
    call spi_transfer

sx1278_write_buffer_loop:
    beqz s1, sx1278_write_buffer_done
    lbu  a0, 0(s0)
    addi s0, s0, 1
    addi s1, s1, -1
    call spi_transfer
    j    sx1278_write_buffer_loop

sx1278_write_buffer_done:
    call spi_cs_high

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1278_set_dio_irq - Configure DIO0 interrupt mapping
# ==============================================================================
# INPUTS:  a0 = mode: 0 = RX (DIO0=RxDone), 1 = TX (DIO0=TxDone)
#               a1 = ignored
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# RegDioMapping1 (0x40) bits [7:6]: DIO0 mapping
#   00 = RxDone, 01 = TxDone
# DIO1 always configured as RxTimeout (bits [5:4] = 00)
# ==============================================================================

.globl sx1278_set_dio_irq
sx1278_set_dio_irq:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # a0=0 -> DIO0_RX_DONE (0x00), a0!=0 -> DIO0_TX_DONE (0x40)
    beqz a0, 1f
    li   a1, DIO0_TX_DONE
    j    2f
1:  li   a1, DIO0_RX_DONE
2:  li   a0, REG_DIO_MAPPING1
    call sx1278_write_reg

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_set_tx - Start LoRa transmission
# ==============================================================================
# INPUTS:  a0 = ignored
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# SX1278 TX: write OPMODE_TX to RegOpMode. After TxDone, radio auto-returns
# to STDBY. The TxDone interrupt fires on DIO0 (must pre-configure DIO0).
# ==============================================================================

.globl sx1278_set_tx
sx1278_set_tx:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # RF switch: TXEN=H, RXEN=L
    li   t0, GPIOC_BSHR
    li   t1, (LORA_TXEN_HIGH | LORA_RXEN_LOW)
    sw   t1, 0(t0)

    li   a0, REG_OP_MODE
    li   a1, OPMODE_TX
    call sx1278_write_reg

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_set_rx - Start LoRa continuous reception
# ==============================================================================
# INPUTS:  a0 = ignored
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
#
# RxDone fires on DIO0. In RXCONTINUOUS mode, the radio stays in RX
# after each packet; the MCU must re-arm DIO0 and re-read the FIFO.
# ==============================================================================

.globl sx1278_set_rx
sx1278_set_rx:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # RF switch: TXEN=L, RXEN=H
    li   t0, GPIOC_BSHR
    li   t1, (LORA_TXEN_LOW | LORA_RXEN_HIGH)
    sw   t1, 0(t0)

    li   a0, REG_OP_MODE
    li   a1, OPMODE_RXCONTINUOUS
    call sx1278_write_reg

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_get_irq_status - Read IRQ flags
# ==============================================================================
# OUTPUTS: a0 = IRQ flags byte (RegIrqFlags 0x12, 8 bits)
#               IRQ_RX_DONE (bit6), IRQ_TX_DONE (bit3), IRQ_PAYLOAD_CRC_ERROR (bit5)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_get_irq_status
sx1278_get_irq_status:
    addi sp, sp, -4
    sw   ra, 0(sp)

    li   a0, REG_IRQ_FLAGS
    call sx1278_read_reg        # a0 = IRQ flags

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_clear_irq - Clear IRQ flags
# ==============================================================================
# INPUTS:  a0 = mask of flags to clear (write 1 to clear on SX1278)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_clear_irq
sx1278_clear_irq:
    addi sp, sp, -4
    sw   ra, 0(sp)

    mv   a1, a0
    li   a0, REG_IRQ_FLAGS
    call sx1278_write_reg       # writing 1 clears the flag

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# sx1278_get_rx_buffer_status - Get received payload length and FIFO start addr
# ==============================================================================
# OUTPUTS:
#   a0 = payload length (RegRxNbBytes 0x13)
#   a1 = FIFO start address of last packet (RegFifoRxCurrentAddr 0x10)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_get_rx_buffer_status
sx1278_get_rx_buffer_status:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)

    li   a0, REG_RX_NB_BYTES
    call sx1278_read_reg
    mv   s0, a0                 # s0 = payload length

    li   a0, REG_FIFO_RX_CURRENT_ADDR
    call sx1278_read_reg
    mv   s1, a0                 # s1 = FIFO start address

    mv   a0, s0
    mv   a1, s1

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1278_read_buffer - Read received payload from FIFO
# ==============================================================================
# INPUTS:
#   a0 = destination buffer pointer (MCU RAM)
#   a1 = FIFO start address (from sx1278_get_rx_buffer_status a1 output)
#   a2 = length in bytes
# CLOBBERS: a0, a1, a2, t0, t1, t2
# PRESERVES: s0, s1
#
# SEQUENCE:
#   1. Write RegFifoAddrPtr = fifo_start_addr
#   2. Burst read from RegFifo (0x00)
# ==============================================================================

.globl sx1278_read_buffer
sx1278_read_buffer:
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a1, 12(sp)             # fifo_start_addr (used once)
    mv   s0, a0                 # s0 = dest ptr
    mv   s1, a2                 # s1 = length

    # Set FIFO pointer to start of received packet
    lw   a1, 12(sp)
    li   a0, REG_FIFO_ADDR_PTR
    call sx1278_write_reg

    # Burst read from FIFO
    call spi_cs_low
    li   a0, REG_FIFO            # read bit: MSB = 0
    call spi_transfer

sx1278_read_buffer_loop:
    beqz s1, sx1278_read_buffer_done
    li   a0, 0x00               # clock out data
    call spi_transfer
    sb   a0, 0(s0)
    addi s0, s0, 1
    addi s1, s1, -1
    j    sx1278_read_buffer_loop

sx1278_read_buffer_done:
    call spi_cs_high

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ==============================================================================
# sx1278_get_packet_status - Get RSSI and SNR of last received packet
# ==============================================================================
# OUTPUTS:
#   a0 = PacketRssi raw (RSSI[dBm] = -164 + a0  for LF band / 400 MHz)
#   a1 = PacketSnr raw (SNR[dB] = signed(a1) / 4)
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_get_packet_status
sx1278_get_packet_status:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)

    li   a0, REG_PKT_SNR_VALUE
    call sx1278_read_reg
    mv   s1, a0                 # s1 = SNR raw

    li   a0, REG_PKT_RSSI_VALUE
    call sx1278_read_reg
    mv   s0, a0                 # s0 = RSSI raw

    mv   a0, s0
    mv   a1, s1

    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ==============================================================================
# sx1278_get_frequency_error - Get signed LoRa FEI raw value
# ==============================================================================
# OUTPUTS:
#   a0 = signed 20-bit FEI raw value, sign-extended to 32 bits.
#        Convert to Hz with: fei_hz = fei_raw * bw_hz / 953670
#        (= fei_raw * bw_kHz / 954, F_XOSC = 32 MHz; done in lora_fei_raw_to_hz).
# CLOBBERS: a0, a1, t0, t1, t2
# PRESERVES: s0, s1
# ==============================================================================

.globl sx1278_get_frequency_error
sx1278_get_frequency_error:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)

    li   a0, REG_FEI_MSB
    call sx1278_read_reg
    andi s0, a0, 0x0F
    slli s0, s0, 16

    li   a0, REG_FEI_MID
    call sx1278_read_reg
    slli a0, a0, 8
    or   s0, s0, a0

    li   a0, REG_FEI_LSB
    call sx1278_read_reg
    or   s0, s0, a0

    # Sign-extend 20-bit two's-complement value.
    slli s0, s0, 12
    srai s0, s0, 12
    mv   a0, s0

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret
