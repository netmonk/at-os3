# ==============================================================================
# drivers/ch32v003/radio_sx1278.S - Radio ABI backend for SX1278
# ==============================================================================
#
# PURPOSE:
#   Thin compile-time selected radio facade for the SX1278 driver. This file owns
#   no radio policy; it only maps the stable radio_* ABI used by lora_fsm.S to
#   the existing sx1278_* low-level driver entry points.
#
# ==============================================================================

.extern sx1278_init
.extern sx1278_set_sync_word
.extern sx1278_set_iq_invert
.extern sx1278_set_frequency
.extern sx1278_config_lora
.extern sx1278_set_tx_power
.extern sx1278_set_dio_irq
.extern sx1278_set_packet_params
.extern sx1278_set_buffer_base
.extern sx1278_write_buffer
.extern sx1278_set_tx
.extern sx1278_set_sleep
.extern sx1278_get_irq_status
.extern sx1278_clear_irq
.extern sx1278_set_standby
.extern sx1278_get_rx_buffer_status
.extern sx1278_read_buffer
.extern sx1278_set_rx
.extern sx1278_get_packet_status
.extern sx1278_get_frequency_error

.section .text
.align 2

.globl radio_init
radio_init:
    j sx1278_init

.globl radio_set_sync_word
radio_set_sync_word:
    j sx1278_set_sync_word

.globl radio_set_iq_invert
radio_set_iq_invert:
    j sx1278_set_iq_invert

.globl radio_set_frequency
radio_set_frequency:
    j sx1278_set_frequency

.globl radio_config_lora
radio_config_lora:
    j sx1278_config_lora

.globl radio_set_tx_power
radio_set_tx_power:
    j sx1278_set_tx_power

.globl radio_set_dio_irq
radio_set_dio_irq:
    j sx1278_set_dio_irq

.globl radio_set_packet_params
radio_set_packet_params:
    j sx1278_set_packet_params

.globl radio_set_buffer_base
radio_set_buffer_base:
    j sx1278_set_buffer_base

.globl radio_write_buffer
radio_write_buffer:
    j sx1278_write_buffer

.globl radio_set_tx
radio_set_tx:
    j sx1278_set_tx

.globl radio_set_sleep
radio_set_sleep:
    j sx1278_set_sleep

.globl radio_get_irq_status
radio_get_irq_status:
    j sx1278_get_irq_status

.globl radio_clear_irq
radio_clear_irq:
    j sx1278_clear_irq

.globl radio_set_standby
radio_set_standby:
    j sx1278_set_standby

.globl radio_get_rx_buffer_status
radio_get_rx_buffer_status:
    j sx1278_get_rx_buffer_status

.globl radio_read_buffer
radio_read_buffer:
    j sx1278_read_buffer

.globl radio_set_rx
radio_set_rx:
    j sx1278_set_rx

.globl radio_get_packet_status
radio_get_packet_status:
    j sx1278_get_packet_status

.globl radio_get_frequency_error
radio_get_frequency_error:
    j sx1278_get_frequency_error

.globl radio_probe
radio_probe:
    addi sp, sp, -4
    sw   ra, 0(sp)
    li   a0, 0x42
    call sx1278_read_reg
    andi a0, a0, 0xFF
    li   t0, 0x12
    slli t0, t0, 8
    or   a0, a0, t0
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret
