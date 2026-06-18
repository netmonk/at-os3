# ==============================================================================
# drivers/ch32v003/radio_sx1262.S - Radio ABI backend for SX1262/E22
# ==============================================================================
#
# PURPOSE:
#   Compile-time selected radio facade for the SX1262 driver. This backend adapts
#   the current lora_fsm.S radio_* ABI, which still carries a few SX1278-shaped
#   values, to the SX1262 command interface.
#
# LIMITS:
#   - Frequency error is reported as 0: SX1262 has no direct SX1278 FEI register.
#   - Packet params currently follow the imported driver behavior: explicit
#     header, CRC on, standard IQ.
#   - PA constants are inherited from os3-rnode and target +22 dBm SX1262 setup;
#     E22-900M30S high-power behavior still needs hardware validation.
#
# ==============================================================================

.extern sx1262_init
.extern sx1262_set_sync_word
.extern sx1262_set_iq_polarity
.extern sx1262_set_frequency
.extern sx1262_config_lora
.extern sx1262_set_tx_power
.extern sx1262_set_dio_irq
.extern sx1262_set_packet_params
.extern sx1262_set_buffer_base
.extern sx1262_write_buffer
.extern sx1262_set_tx
.extern sx1262_get_irq_status
.extern sx1262_clear_irq
.extern sx1262_set_standby
.extern sx1262_get_rx_buffer_status
.extern sx1262_read_buffer
.extern sx1262_set_rx
.extern sx1262_get_packet_status
.extern sx1262_get_status

# SX1262 IRQ masks
.equ SX1262_IRQ_TX_DONE,        (1 << 0)
.equ SX1262_IRQ_RX_DONE,        (1 << 1)
.equ SX1262_IRQ_CRC_ERR,        (1 << 6)
.equ SX1262_IRQ_TIMEOUT,        (1 << 9)
.equ SX1262_IRQ_RX_MASK,        (SX1262_IRQ_RX_DONE | SX1262_IRQ_CRC_ERR | SX1262_IRQ_TIMEOUT)
.equ SX1262_IRQ_TX_MASK,        (SX1262_IRQ_TX_DONE | SX1262_IRQ_TIMEOUT)

# lora_fsm.S currently consumes SX1278-shaped IRQ flags.
.equ RADIO_IRQ_TX_DONE,         (1 << 3)
.equ RADIO_IRQ_RX_DONE,         (1 << 6)
.equ RADIO_IRQ_CRC_ERR,         (1 << 5)
.equ RADIO_IRQ_TIMEOUT,         (1 << 7)

# SX1262 LoRa bandwidth codes
.equ SX1262_BW_7K8,             0x00
.equ SX1262_BW_10K4,            0x08
.equ SX1262_BW_15K6,            0x01
.equ SX1262_BW_20K8,            0x09
.equ SX1262_BW_31K25,           0x02
.equ SX1262_BW_41K7,            0x0A
.equ SX1262_BW_62K5,            0x03
.equ SX1262_BW_125,             0x04
.equ SX1262_BW_250,             0x05
.equ SX1262_BW_500,             0x06

.section .text
.align 2

.globl radio_init
radio_init:
    j sx1262_init

.globl radio_set_sync_word
radio_set_sync_word:
    li   t0, 0x12
    beq  a0, t0, radio_sync_public
    li   t0, 0x34
    beq  a0, t0, radio_sync_private
    # Custom sync word: SX127x byte 0xXY -> SX1262 word 0xX4Y4
    # SX1262 uses only the upper nibble of each byte on air; lower nibble = 0x4.
    andi a0, a0, 0xFF
    andi t0, a0, 0xF0           # byte0 high nibble (upper nibble of input)
    ori  t0, t0, 0x04           # byte0 = (sw & 0xF0) | 0x04
    slli t1, a0, 4              # shift lower nibble to upper position
    andi t1, t1, 0xF0           # byte1 high nibble
    ori  t1, t1, 0x04           # byte1 = ((sw << 4) & 0xF0) | 0x04
    slli t0, t0, 8
    or   a0, t0, t1             # a0 = (byte0 << 8) | byte1
    j    sx1262_set_sync_word
radio_sync_public:
    li   a0, 0x1424    # SX127x 0x12 = nibbles [1,2] on air
    j    sx1262_set_sync_word
radio_sync_private:
    li   a0, 0x3444    # SX127x 0x34 = nibbles [3,4] on air
    j    sx1262_set_sync_word

.globl radio_set_iq_invert
radio_set_iq_invert:
    j sx1262_set_iq_polarity

.globl radio_set_frequency
radio_set_frequency:
    # lora_fsm.S currently computes the SX127x Frf value:
    #   freq_hz * 2^19 / 32e6
    # SX1262 SetRfFrequency expects:
    #   freq_hz * 2^25 / 32e6
    # The factor between both is 64.
    slli a0, a0, 6
    j    sx1262_set_frequency

.globl radio_config_lora
radio_config_lora:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)

    mv   s0, a0                 # SF
    mv   a0, a1                 # SX1278 BW code -> SX1262 BW code
    call radio_sx1262_bw_from_sx1278
    mv   a1, a0

    srli a2, a2, 1              # SX1278 CR bits 0x02..0x08 -> SX1262 CR 1..4
    mv   a0, s0
    mv   a3, a4                 # a4 = LDRO mode from lora_fsm -> a3 for sx1262_config_lora
    call sx1262_config_lora

    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

radio_sx1262_bw_from_sx1278:
    li   t0, 0x00
    beq  a0, t0, 1f
    li   t0, 0x10
    beq  a0, t0, 2f
    li   t0, 0x20
    beq  a0, t0, 3f
    li   t0, 0x30
    beq  a0, t0, 4f
    li   t0, 0x40
    beq  a0, t0, 5f
    li   t0, 0x50
    beq  a0, t0, 6f
    li   t0, 0x60
    beq  a0, t0, 7f
    li   t0, 0x70
    beq  a0, t0, 8f
    li   t0, 0x80
    beq  a0, t0, 9f
    li   a0, SX1262_BW_500
    ret
1:  li   a0, SX1262_BW_7K8
    ret
2:  li   a0, SX1262_BW_10K4
    ret
3:  li   a0, SX1262_BW_15K6
    ret
4:  li   a0, SX1262_BW_20K8
    ret
5:  li   a0, SX1262_BW_31K25
    ret
6:  li   a0, SX1262_BW_41K7
    ret
7:  li   a0, SX1262_BW_62K5
    ret
8:  li   a0, SX1262_BW_125
    ret
9:  li   a0, SX1262_BW_250
    ret

.globl radio_set_tx_power
radio_set_tx_power:
    j sx1262_set_tx_power

.globl radio_set_dio_irq
radio_set_dio_irq:
    beqz a0, radio_set_dio_irq_rx
    li   a0, SX1262_IRQ_TX_MASK
    li   a1, SX1262_IRQ_TX_MASK
    j    sx1262_set_dio_irq
radio_set_dio_irq_rx:
    li   a0, SX1262_IRQ_RX_MASK
    li   a1, SX1262_IRQ_RX_MASK
    j    sx1262_set_dio_irq

.globl radio_set_packet_params
radio_set_packet_params:
    j sx1262_set_packet_params

.globl radio_set_buffer_base
radio_set_buffer_base:
    # lora_fsm passes TX=0x80, RX=0x00 (SX1278 FIFO convention).
    # sx1262_write_buffer always writes at FIFO offset 0x00, so TX base must be 0x00.
    li   a0, 0x00
    li   a1, 0x80
    j    sx1262_set_buffer_base

.globl radio_write_buffer
radio_write_buffer:
    j sx1262_write_buffer

.globl radio_set_tx
radio_set_tx:
    j sx1262_set_tx

.globl radio_set_sleep
radio_set_sleep:
    # The imported SX1262 init/programming path uses standby for command mode.
    li   a0, 0x01
    j    sx1262_set_standby

.globl radio_get_irq_status
radio_get_irq_status:
    addi sp, sp, -4
    sw   ra, 0(sp)
    call sx1262_get_irq_status
    mv   t0, a0
    li   a0, 0

    andi t1, t0, SX1262_IRQ_TX_DONE
    beqz t1, 1f
    ori  a0, a0, RADIO_IRQ_TX_DONE
1:
    andi t1, t0, SX1262_IRQ_RX_DONE
    beqz t1, 2f
    ori  a0, a0, RADIO_IRQ_RX_DONE
2:
    andi t1, t0, SX1262_IRQ_CRC_ERR
    beqz t1, 3f
    ori  a0, a0, RADIO_IRQ_CRC_ERR
3:
    li   t1, SX1262_IRQ_TIMEOUT
    and  t1, t0, t1
    beqz t1, 4f
    ori  a0, a0, RADIO_IRQ_TIMEOUT
4:
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

.globl radio_clear_irq
radio_clear_irq:
    mv   t0, a0
    li   a0, 0

    andi t1, t0, RADIO_IRQ_TX_DONE
    beqz t1, 1f
    ori  a0, a0, SX1262_IRQ_TX_DONE
1:
    andi t1, t0, RADIO_IRQ_RX_DONE
    beqz t1, 2f
    ori  a0, a0, SX1262_IRQ_RX_DONE
2:
    andi t1, t0, RADIO_IRQ_CRC_ERR
    beqz t1, 3f
    ori  a0, a0, SX1262_IRQ_CRC_ERR
3:
    andi t1, t0, RADIO_IRQ_TIMEOUT
    beqz t1, 4f
    li   t1, SX1262_IRQ_TIMEOUT
    or   a0, a0, t1
4:
    j    sx1262_clear_irq

.globl radio_set_standby
radio_set_standby:
    j sx1262_set_standby

.globl radio_get_rx_buffer_status
radio_get_rx_buffer_status:
    j sx1262_get_rx_buffer_status

.globl radio_read_buffer
radio_read_buffer:
    j sx1262_read_buffer

.globl radio_set_rx
radio_set_rx:
    # lora_fsm.S passes 0 for continuous RX on SX1278. SX1262 continuous RX is
    # encoded as 0xFFFFFF.
    li   a0, 0xFFFFFF
    j    sx1262_set_rx

.globl radio_get_packet_status
radio_get_packet_status:
    j sx1262_get_packet_status

.globl radio_get_frequency_error
radio_get_frequency_error:
    li   a0, 0
    ret

.globl radio_probe
radio_probe:
    addi sp, sp, -4
    sw   ra, 0(sp)
    call sx1262_get_status
    andi a0, a0, 0xFF
    li   t0, 0x62
    slli t0, t0, 8
    or   a0, a0, t0
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret
