# ==============================================================================
# subfsm/lora_fsm.S - LoRa Radio FSM (Table-Driven)
# ==============================================================================
#
# PURPOSE:
#   True FSM managing LoRa radio state (IDLE/TX/RX).
#   It owns the SX1278 configuration and only communicates with the host-facing
#   radio FSM through the canonical event queue.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): Host protocol talks to radio by events only
#   - Article 6 (FSM): State transitions via fsm_step ONLY
#   - Article 9 (Structural Enforcement): Registers via register_activity ABI only
#   - Article 10 (Layer Roles): Host FSM never calls the radio driver directly
#   - Article 11 (ABI Sanctity): Shared radio state is published only after an
#     explicit notify event back to the host layer
#
# EVENT MODEL:
#   Host -> radio FSM -> EV_SOFT_LORA_CMD -> this FSM
#   Radio IRQ -> this FSM -> EV_SOFT_RADIO_NOTIFY -> host AT interface
#
# ============================================================================== 

.include "core/events.inc"

# ------------------------------------------------------------------------------
# FSM Layout (must match core/fsm.S)
# ------------------------------------------------------------------------------
.equ FSM_MAGIC,      0
.equ FSM_STATE,      4
.equ FSM_EVENT_BITS, 5
.equ FSM_NUM_STATES, 6
.equ FSM_NUM_EVENTS, 7
.equ FSM_TABLE,      8
.equ FSM_ACTIONS,   12
.equ FSM_DATA,      16

# ------------------------------------------------------------------------------
# LoRa FSM Constants
# ------------------------------------------------------------------------------

# States
.equ LORA_STATE_IDLE,       0
.equ LORA_STATE_TX_ACTIVE,  1
.equ LORA_STATE_RX_ACTIVE,  2
.equ LORA_NUM_STATES,       3

# Events (internal to this FSM)
.equ LORA_EV_DIO0,         0
# event slot 1 is intentionally reserved/no-op; raw AT operation uses one
# physical LoRa packet per TX command, while the 16-column table leaves room.
.equ LORA_EV_CMD_RX,       2
.equ LORA_EV_CFG,          3
.equ LORA_EV_RADIO_ON,     4
.equ LORA_EV_RADIO_OFF,    5
.equ LORA_EV_TX_FRAME,     6
.equ LORA_EV_SET_PARAMS,   7           # Atomic SF+BW+CR+preamble (was pad column)
.equ LORA_EV_RESET,        8           # Hardware reset + full reconfig + RX re-arm
.equ LORA_NUM_EVENTS,      16          # Must be power of 2
.equ LORA_EVENT_BITS,      4           # log2(16)

# SX1278 constants local to this FSM (RegIrqFlags 0x12, 8-bit)
.equ SX_STDBY_RC,          0x00        # kept for ABI compat, ignored by sx1278_set_standby
.equ SX_STDBY_XOSC,        0x01        # kept for ABI compat, ignored by sx1278_set_standby
.equ SX_IRQ_TX_DONE,       (1 << 3)    # bit 3 = TxDone
.equ SX_IRQ_RX_DONE,       (1 << 6)    # bit 6 = RxDone
.equ SX_IRQ_CRC_ERR,       (1 << 5)    # bit 5 = PayloadCrcError
.equ SX_IRQ_TIMEOUT,       (1 << 7)    # bit 7 = RxTimeout
.equ SX_IRQ_RX_MASK,       (SX_IRQ_RX_DONE | SX_IRQ_CRC_ERR | SX_IRQ_TIMEOUT)
.equ SX_IRQ_TX_MASK,       (SX_IRQ_TX_DONE)
# Raw LoRa: TX and RX carry the bare payload. A LoRa physical packet is at most
# 255 bytes, and RX delivers exactly one packet per RxDone: one packet == one
# +RCV line.
# Therefore the maximum logical frame equals the raw packet capacity; any
# payload that would not fit a single packet cannot be received coherently and
# must be rejected upstream (at_fsm bounds AT+SEND to AT_TX_MAX <= 255).
.equ LORA_RAW_PACKET_MAX,  255
.equ LORA_FRAME_MAX,       LORA_RAW_PACKET_MAX  # 1 packet = 1 +RCV
.equ LORA_RX_BUF_SIZE,     LORA_RAW_PACKET_MAX

# Supported config values (minimal first milestone)
.equ LORA_DEFAULT_FREQ_HZ,  433175000   # TinyGS UHF default (AT+BAND overrides)
# Legacy named constants kept for internal use
.equ LORA_FREQ_868_HZ,     868000000
.equ LORA_FREQ_915_HZ,     915000000
# All 10 AT / SX1278 bandwidth codes (Hz values sent via LORA_CMD_SET_BW)
.equ LORA_BW_7K8_HZ,       7800
.equ LORA_BW_10K4_HZ,      10400
.equ LORA_BW_15K6_HZ,      15600
.equ LORA_BW_20K8_HZ,      20800
.equ LORA_BW_31K25_HZ,     31250
.equ LORA_BW_41K7_HZ,      41700
.equ LORA_BW_62K5_HZ,      62500
.equ LORA_BW_125_HZ,       125000
.equ LORA_BW_250_HZ,       250000
.equ LORA_BW_500_HZ,       500000
# SX1278 BW codes (upper nibble of RegModemConfig1)
.equ LORA_SX_BW_7K8,       0x00
.equ LORA_SX_BW_10K4,      0x10
.equ LORA_SX_BW_15K6,      0x20
.equ LORA_SX_BW_20K8,      0x30
.equ LORA_SX_BW_31K25,     0x40
.equ LORA_SX_BW_41K7,      0x50
.equ LORA_SX_BW_62K5,      0x60
.equ LORA_SX_BW_125,       0x70
.equ LORA_SX_BW_250,       0x80
.equ LORA_SX_BW_500,       0x90
.equ LORA_SX_CR_4_5,       0x02
.equ LORA_SX_CR_4_6,       0x04
.equ LORA_SX_CR_4_7,       0x06
.equ LORA_SX_CR_4_8,       0x08
.equ LORA_SYNC_WORD_PUBLIC, 0x12
.equ LORA_DEFAULT_PREAMBLE, 8
.equ LORA_PREAMBLE_LEN,    8
.equ LORA_ACTIVITY_TIMER_ID, 7
.equ LORA_ACTIVITY_PULSE_TICKS, 1200000

# LoRa-specific data offsets (after FSM_DATA)
.equ LORA_DATA_TX_COUNT,   0
.equ LORA_DATA_RX_COUNT,   4
.equ LORA_DATA_FREQ_HZ,    8
.equ LORA_DATA_FREQ_REG,   12
.equ LORA_DATA_BW_HZ,      16
.equ LORA_DATA_BW_CODE,    20
.equ LORA_DATA_TXP,        24
.equ LORA_DATA_SF,         28
.equ LORA_DATA_CR,         32
.equ LORA_DATA_RADIO_ON,   36
.equ LORA_DATA_TX_PTR,     40
.equ LORA_DATA_TX_LEN,     44
.equ LORA_DATA_CFG_DIRTY,  48
.equ LORA_DATA_CAL_HZ,     52
.equ LORA_DATA_PREAMBLE,   56
.equ LORA_DATA_IQI,        60
.equ LORA_DATA_SYNC_WORD,  64
.equ LORA_DATA_CRC,        68
.equ LORA_DATA_LDRO,       72
.equ LORA_DATA_IMPLICIT_LEN, 76

# ------------------------------------------------------------------------------
# External declarations
# ------------------------------------------------------------------------------
.extern fsm_init
.extern fsm_step
.extern register_activity
.extern event_enqueue
.extern timer_arm_delta
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
.extern led_activity_on
.extern led_activity_off

# ==============================================================================
# BSS Section - FSM Instance
# ==============================================================================

.section .bss
.align 4

.globl lora_fsm
lora_fsm:
    .word 0                         # +0:  magic (set by fsm_init)
    .byte 0                         # +4:  state
    .byte 0                         # +5:  event_bits
    .byte 0                         # +6:  num_states
    .byte 0                         # +7:  num_events
    .word 0                         # +8:  table ptr
    .word 0                         # +12: actions ptr
lora_fsm_data:
    .word 0                         # +16: tx_count
    .word 0                         # +20: rx_count
    .word 0                         # +24: frequency_hz shadow
    .word 0                         # +28: frequency_reg shadow
    .word 0                         # +32: bandwidth_hz shadow
    .word 0                         # +36: bandwidth_code shadow
    .word 0                         # +40: tx_power shadow
    .word 0                         # +44: spreading factor shadow
    .word 0                         # +48: coding rate shadow
    .word 0                         # +52: radio_on shadow
    .word 0                         # +56: tx payload pointer
    .word 0                         # +60: tx payload length
    .word 0                         # +64: modem config dirty flag
    .word 0                         # +68: last calibrated frequency band
    .word 0                         # +72: preamble length (symbols)
    .word 0                         # +76: IQ inversion flag (0 normal, 1 inverted)
    .word 0                         # +80: LoRa sync word (8-bit SX127x value)
    .word 0                         # +84: PHY CRC flag (0 off, 1 on)
    .word 0                         # +88: LDRO mode (0 off, 1 on, 2 auto)
    .word 0                         # +92: implicit header length (0 explicit)
lora_rx_buffer:
    .space LORA_RX_BUF_SIZE
.align 4
.globl lora_rx_buffer
lora_last_irq_flags:
    .word 0
lora_last_rx_len:
    .word 0
.globl lora_last_rssi_raw
lora_last_rssi_raw:
    .word 0
.globl lora_last_snr_raw
lora_last_snr_raw:
    .word 0
.globl lora_last_fei_hz
lora_last_fei_hz:
    .word 0

# Compatibility symbol for stats display
.globl lora_state
lora_state = lora_fsm_data

# ==============================================================================
# RODATA Section - Transition Table / Actions
# ==============================================================================

.section .rodata
.align 2

lora_fsm_table:
    # State 0 (IDLE)
    .byte LORA_STATE_IDLE,       0  # DIO0       -> IDLE,      nop
    .byte LORA_STATE_IDLE,       0  # reserved   -> IDLE,      nop
    .byte LORA_STATE_RX_ACTIVE,  2  # CMD_RX     -> RX_ACTIVE, start_rx
    .byte LORA_STATE_IDLE,       3  # CFG        -> IDLE,      cfg
    .byte LORA_STATE_RX_ACTIVE,  2  # RADIO_ON   -> RX_ACTIVE, start_rx
    .byte LORA_STATE_IDLE,       4  # RADIO_OFF  -> IDLE,      standby
    .byte LORA_STATE_TX_ACTIVE,  5  # TX_FRAME   -> TX_ACTIVE, tx_frame
    .byte LORA_STATE_IDLE,       8  # SET_PARAMS -> IDLE,      set_params
    .byte LORA_STATE_RX_ACTIVE,  9  # RESET      -> RX_ACTIVE, reset
    .byte LORA_STATE_IDLE,       0  # pad        -> IDLE,      nop
    .byte LORA_STATE_IDLE,       0  # pad        -> IDLE,      nop
    .byte LORA_STATE_IDLE,       0  # pad        -> IDLE,      nop
    .byte LORA_STATE_IDLE,       0  # pad        -> IDLE,      nop
    .byte LORA_STATE_IDLE,       0  # pad        -> IDLE,      nop
    .byte LORA_STATE_IDLE,       0  # pad        -> IDLE,      nop
    .byte LORA_STATE_IDLE,       0  # pad        -> IDLE,      nop

    # State 1 (TX_ACTIVE)
    .byte LORA_STATE_TX_ACTIVE,  6  # DIO0       -> TX_ACTIVE, tx_done
    .byte LORA_STATE_TX_ACTIVE,  0  # reserved   -> TX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  2  # CMD_RX     -> RX_ACTIVE, start_rx
    .byte LORA_STATE_TX_ACTIVE,  3  # CFG        -> TX_ACTIVE, cfg shadow only
    .byte LORA_STATE_TX_ACTIVE,  0  # RADIO_ON   -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  0  # RADIO_OFF  -> TX_ACTIVE, nop (retry later)
    .byte LORA_STATE_TX_ACTIVE,  0  # TX_FRAME   -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  8  # SET_PARAMS -> TX_ACTIVE, set_params (shadow only)
    .byte LORA_STATE_RX_ACTIVE,  9  # RESET      -> RX_ACTIVE, reset/abort TX
    .byte LORA_STATE_TX_ACTIVE,  0  # pad        -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  0  # pad        -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  0  # pad        -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  0  # pad        -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  0  # pad        -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  0  # pad        -> TX_ACTIVE, nop
    .byte LORA_STATE_TX_ACTIVE,  0  # pad        -> TX_ACTIVE, nop

    # State 2 (RX_ACTIVE)
    .byte LORA_STATE_RX_ACTIVE,  7  # DIO0       -> RX_ACTIVE, rx_irq
    .byte LORA_STATE_RX_ACTIVE,  0  # reserved   -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  0  # CMD_RX     -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  3  # CFG        -> RX_ACTIVE, cfg + rearm RX
    .byte LORA_STATE_RX_ACTIVE,  2  # RADIO_ON   -> RX_ACTIVE, start_rx
    .byte LORA_STATE_IDLE,       4  # RADIO_OFF  -> IDLE,      standby
    .byte LORA_STATE_TX_ACTIVE,  5  # TX_FRAME   -> TX_ACTIVE, tx_frame
    .byte LORA_STATE_RX_ACTIVE,  8  # SET_PARAMS -> RX_ACTIVE, set_params + rearm
    .byte LORA_STATE_RX_ACTIVE,  9  # RESET      -> RX_ACTIVE, reset
    .byte LORA_STATE_RX_ACTIVE,  0  # pad        -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  0  # pad        -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  0  # pad        -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  0  # pad        -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  0  # pad        -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  0  # pad        -> RX_ACTIVE, nop
    .byte LORA_STATE_RX_ACTIVE,  0  # pad        -> RX_ACTIVE, nop

lora_fsm_actions:
    .word lora_action_nop
    .word lora_action_nop             # reserved no-op action slot
    .word lora_action_start_rx
    .word lora_action_cfg
    .word lora_action_standby
    .word lora_action_tx_frame
    .word lora_action_tx_done
    .word lora_action_rx_irq
    .word lora_action_set_params
    .word lora_action_reset

# ==============================================================================
# KERNEL_INIT Section - Auto-Registration Pointer
# ==============================================================================

.section .kernel_init, "a"
.align 2
lora_init_ptr:
    .word lora_init

# ==============================================================================
# TEXT Section - FSM Code
# ==============================================================================

.section .text
.align 2

.globl lora_init
lora_init:
    addi sp, sp, -4
    sw   ra, 0(sp)

    la   a0, lora_fsm
    la   a1, lora_fsm_table
    la   a2, lora_fsm_actions
    li   a3, LORA_EVENT_BITS
    li   a4, LORA_NUM_STATES
    li   a5, LORA_STATE_IDLE
    call fsm_init

    # Reset counters and install default 433.175 MHz/SF12/BW125/CR4/5 profile.
    la   t0, lora_fsm_data
    sw   zero, LORA_DATA_TX_COUNT(t0)
    sw   zero, LORA_DATA_RX_COUNT(t0)
    li   t1, LORA_DEFAULT_FREQ_HZ
    sw   t1, LORA_DATA_FREQ_HZ(t0)
    sw   zero, LORA_DATA_FREQ_REG(t0)  # computed by lora_cfg_freq on first apply
    li   t1, LORA_BW_125_HZ
    sw   t1, LORA_DATA_BW_HZ(t0)
    li   t1, LORA_SX_BW_125
    sw   t1, LORA_DATA_BW_CODE(t0)
    li   t1, 15                        # 15 dBm TX power (PA_BOOST max for E32)
    sw   t1, LORA_DATA_TXP(t0)
    li   t1, 12                        # SF12 default
    sw   t1, LORA_DATA_SF(t0)
    li   t1, 5                         # CR 4/5
    sw   t1, LORA_DATA_CR(t0)
    sw   zero, LORA_DATA_RADIO_ON(t0)
    sw   zero, LORA_DATA_TX_PTR(t0)
    sw   zero, LORA_DATA_TX_LEN(t0)
    li   t1, 1
    sw   t1, LORA_DATA_CFG_DIRTY(t0)
    sw   zero, LORA_DATA_CAL_HZ(t0)
    li   t1, LORA_DEFAULT_PREAMBLE
    sw   t1, LORA_DATA_PREAMBLE(t0)
    sw   zero, LORA_DATA_IQI(t0)
    li   t1, LORA_SYNC_WORD_PUBLIC
    sw   t1, LORA_DATA_SYNC_WORD(t0)
    li   t1, 1
    sw   t1, LORA_DATA_CRC(t0)
    li   t1, 2
    sw   t1, LORA_DATA_LDRO(t0)
    sw   zero, LORA_DATA_IMPLICIT_LEN(t0)

    # Bring the radio into a known initialized state once at boot.
    call sx1278_init

    li   a0, EV_KIND_IRQ
    li   a1, EV_IRQ_LORA_DIO0
    la   a2, lora_dispatch
    call register_activity

    li   a0, EV_KIND_SOFT
    li   a1, EV_SOFT_LORA_RX
    la   a2, lora_dispatch
    call register_activity

    li   a0, EV_KIND_SOFT
    li   a1, EV_SOFT_LORA_CMD
    la   a2, lora_dispatch
    call register_activity

    li   a0, EV_KIND_TIMER
    li   a1, LORA_ACTIVITY_TIMER_ID
    la   a2, lora_activity_timer_handler
    call register_activity

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ------------------------------------------------------------------------------
# lora_dispatch - Map kernel events to internal FSM events
# ------------------------------------------------------------------------------
lora_dispatch:
    addi sp, sp, -20
    sw   ra, 0(sp)
    sw   a1, 4(sp)                  # data0
    sw   a2, 8(sp)                  # data1
    sw   a0, 12(sp)                 # header

    EV_GET_KIND a0, t0
    li   t1, EV_KIND_IRQ
    beq  t0, t1, lora_dispatch_dio0

    li   t1, EV_KIND_SOFT
    bne  t0, t1, lora_dispatch_done

    EV_GET_ID a0, t0
    li   t1, EV_SOFT_LORA_RX
    beq  t0, t1, lora_dispatch_cmd_rx
    li   t1, EV_SOFT_LORA_CMD
    beq  t0, t1, lora_dispatch_host_cmd
    j    lora_dispatch_done

lora_dispatch_dio0:
    li   a1, LORA_EV_DIO0
    j    lora_dispatch_call

lora_dispatch_cmd_rx:
    li   a1, LORA_EV_CMD_RX
    j    lora_dispatch_call

lora_dispatch_host_cmd:
    lw   t0, 12(sp)
    li   t1, EV_FLAGS_MASK
    and  t0, t0, t1                 # opcode from flags
    li   t1, LORA_CMD_SET_FREQ
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_BW
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_TXP
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_SF
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_CR
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_PREAMBLE
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_IQI
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_SYNC_WORD
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_PKT_PROFILE
    beq  t0, t1, lora_dispatch_cfg
    li   t1, LORA_CMD_SET_PARAMS
    beq  t0, t1, lora_dispatch_set_params
    li   t1, LORA_CMD_SET_STATE
    beq  t0, t1, lora_dispatch_state
    li   t1, LORA_CMD_TX_FRAME
    beq  t0, t1, lora_dispatch_tx_frame
    li   t1, LORA_CMD_RESET
    beq  t0, t1, lora_dispatch_reset
    j    lora_dispatch_done

# Real radio reset: hardware reset + full reconfig, then re-arm RX.
# Routed through fsm_step so reset remains a table-declared FSM transition.
lora_dispatch_reset:
    li   a1, LORA_EV_RESET
    li   a2, 0
    li   a3, 0
    j    lora_dispatch_do_step

lora_dispatch_cfg:
    li   a1, LORA_EV_CFG
    lw   a2, 4(sp)                  # value
    mv   a3, t0                     # opcode
    j    lora_dispatch_do_step

# Atomic SF+BW+CR+preamble: packed profile travels in data0 as a single event,
# so the FSM applies the whole profile or none of it (no partial config).
lora_dispatch_set_params:
    li   a1, LORA_EV_SET_PARAMS
    lw   a2, 4(sp)                  # packed profile (data0)
    li   a3, 0
    j    lora_dispatch_do_step

lora_dispatch_state:
    lw   t1, 4(sp)                  # requested state in data0
    beqz t1, lora_dispatch_state_off
    li   a1, LORA_EV_RADIO_ON
    lw   a2, 4(sp)
    li   a3, LORA_CMD_SET_STATE
    j    lora_dispatch_do_step
lora_dispatch_state_off:
    li   a1, LORA_EV_RADIO_OFF
    lw   a2, 4(sp)
    li   a3, LORA_CMD_SET_STATE
    j    lora_dispatch_do_step

lora_dispatch_tx_frame:
    li   a1, LORA_EV_TX_FRAME
    lw   a2, 4(sp)                  # ptr
    lw   a3, 8(sp)                  # len
    j    lora_dispatch_do_step

lora_dispatch_call:
    lw   a2, 4(sp)
    lw   a3, 8(sp)

lora_dispatch_do_step:
    la   a0, lora_fsm
    call fsm_step

lora_dispatch_done:
    lw   ra, 0(sp)
    addi sp, sp, 20
    ret

# ------------------------------------------------------------------------------
# lora_emit_notify - Publish radio result to the host layer (at_fsm notify handler)
# ------------------------------------------------------------------------------
# INPUTS:
#   a0 = RADIO_NOTIFY_* code
#   a1 = data0 payload
#   a2 = data1 payload

lora_emit_notify:
    addi sp, sp, -4
    sw   ra, 0(sp)
    li   t0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_RADIO_NOTIFY << EV_ID_SHIFT))
    or   a0, a0, t0
    call event_enqueue
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ------------------------------------------------------------------------------
# lora_report_* - push authoritative radio state back to host layer
# ------------------------------------------------------------------------------
lora_report_shadow:
    la   t0, lora_fsm_data
    add  t0, t0, a1
    lw   a1, 0(t0)
    li   a2, 0
    j    lora_emit_notify

lora_report_freq:
    li   a0, RADIO_NOTIFY_REPORT_FREQ
    li   a1, LORA_DATA_FREQ_HZ
    j    lora_report_shadow

lora_report_bw:
    li   a0, RADIO_NOTIFY_REPORT_BW
    li   a1, LORA_DATA_BW_HZ
    j    lora_report_shadow

lora_report_txp:
    li   a0, RADIO_NOTIFY_REPORT_TXP
    li   a1, LORA_DATA_TXP
    j    lora_report_shadow

lora_report_sf:
    li   a0, RADIO_NOTIFY_REPORT_SF
    li   a1, LORA_DATA_SF
    j    lora_report_shadow

lora_report_cr:
    li   a0, RADIO_NOTIFY_REPORT_CR
    li   a1, LORA_DATA_CR
    j    lora_report_shadow

lora_report_state:
    li   a0, RADIO_NOTIFY_REPORT_STATE
    li   a1, LORA_DATA_RADIO_ON
    j    lora_report_shadow

# ------------------------------------------------------------------------------
# lora_apply_common - Program SX1278 from current shadow config
# ------------------------------------------------------------------------------
lora_apply_common:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)

    la   s0, lora_fsm_data
    lw   t0, LORA_DATA_CFG_DIRTY(s0)
    beqz t0, lora_apply_common_done

    call sx1278_set_sleep

    lw   a0, LORA_DATA_SYNC_WORD(s0)
    call sx1278_set_sync_word

    # Compute Frf from freq_hz if FREQ_REG is 0 (first apply or frequency changed)
    lw   a0, LORA_DATA_FREQ_REG(s0)
    bnez a0, lora_apply_common_freq_ready
    lw   a0, LORA_DATA_FREQ_HZ(s0)
    call lora_frf_from_hz
    sw   a0, LORA_DATA_FREQ_REG(s0)
lora_apply_common_freq_ready:
    lw   a0, LORA_DATA_FREQ_REG(s0)
    call sx1278_set_frequency

    lw   a0, LORA_DATA_SF(s0)
    lw   a1, LORA_DATA_BW_CODE(s0)
    lw   a2, LORA_DATA_CR(s0)
    addi a2, a2, -4                 # host CR 5..8 -> 1..4
    slli a2, a2, 1                  # SX1278: CR in bits[3:1] -> multiply by 2: 0x02..0x08
    lw   a3, LORA_DATA_CRC(s0)
    lw   a4, LORA_DATA_LDRO(s0)
    call sx1278_config_lora

    lw   a0, LORA_DATA_IQI(s0)
    call sx1278_set_iq_invert

    lw   a0, LORA_DATA_TXP(s0)
    call sx1278_set_tx_power
    sw   zero, LORA_DATA_CFG_DIRTY(s0)

lora_apply_common_done:
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ------------------------------------------------------------------------------
# lora_program_rx - Apply config and enter continuous RX
# ------------------------------------------------------------------------------
lora_program_rx:
    addi sp, sp, -4
    sw   ra, 0(sp)

    call lora_apply_common
    li   a0, 0                      # RX mode: DIO0 = RxDone
    call sx1278_set_dio_irq
    # RegPayloadLength and RegModemConfig1 header mode are radio parameters;
    # enter SLEEP before touching them, even on RX re-arm with unchanged values.
    call sx1278_set_sleep
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_PREAMBLE(t0)
    lw   a1, LORA_DATA_IMPLICIT_LEN(t0)
    sltu a2, zero, a1               # implicit if len > 0
    bnez a1, lora_program_rx_len_ready
    li   a1, 255
lora_program_rx_len_ready:
    call sx1278_set_packet_params
    li   a0, 0x80                   # TX base = 0x80
    li   a1, 0x00                   # RX base = 0x00
    call sx1278_set_buffer_base
    li   a0, 0                      # arg ignored by SX1278 (always RXCONTINUOUS)
    call sx1278_set_rx

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ------------------------------------------------------------------------------
# lora_queue_soft_rx - request RX re-arm in a future FSM step
# ------------------------------------------------------------------------------
lora_queue_soft_rx:
    addi sp, sp, -4
    sw   ra, 0(sp)
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_RX << EV_ID_SHIFT))
    li   a1, 0
    li   a2, 0
    call event_enqueue
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ------------------------------------------------------------------------------
# lora_start_tx_packet - send one raw LoRa packet
# ------------------------------------------------------------------------------
# TX is raw LoRa. RX reads the same payload bytes in lora_action_rx_irq.
lora_start_tx_packet:
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   zero, 12(sp)

    la   t0, lora_fsm_data
    lw   t1, LORA_DATA_TX_LEN(t0)
    mv   s1, t1                     # one logical payload == one on-air packet
    beqz s1, lora_start_tx_packet_done
    # Raw payload: full 255-byte packet capacity, no header overhead
    li   a5, LORA_RAW_PACKET_MAX
    bleu s1, a5, lora_start_tx_packet_len_ok
    mv   s1, a5
lora_start_tx_packet_len_ok:
    sw   s1, 12(sp)                 # packet byte count
    lw   s0, LORA_DATA_TX_PTR(t0)

    li   a0, SX_STDBY_XOSC          # arg ignored by SX1278 (always LoRa STDBY)
    call sx1278_set_standby

    call lora_apply_common
    li   a0, 1                      # TX mode: DIO0 = TxDone
    call sx1278_set_dio_irq
    # TX packet length updates RegPayloadLength; keep the same sleep-before-
    # radio-parameter-write rule as RX configuration.
    call sx1278_set_sleep
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_PREAMBLE(t0)
    lw   a1, 12(sp)                 # packet length = payload bytes
    lw   t1, LORA_DATA_IMPLICIT_LEN(t0)
    sltu a2, zero, t1               # implicit if configured len > 0
    call sx1278_set_packet_params
    li   a0, 0x80                   # TX base = 0x80
    li   a1, 0x00                   # RX base = 0x00
    call sx1278_set_buffer_base
    mv   a0, s0                     # payload ptr
    lw   a1, 12(sp)                 # payload length
    call sx1278_write_buffer        # raw write, no header prepended
    li   a0, 0                      # arg ignored by SX1278
    call sx1278_set_tx

lora_start_tx_packet_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ------------------------------------------------------------------------------
# lora_begin_tx_packet - latch logical payload and start one raw packet
# ------------------------------------------------------------------------------
# INPUTS:
#   a0 = payload pointer
#   a1 = payload length
lora_begin_tx_packet:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a1

    li   t0, LORA_FRAME_MAX
    bleu s0, t0, lora_begin_tx_len_ok
    mv   s0, t0
lora_begin_tx_len_ok:
    beqz s0, lora_begin_tx_done
    beqz a0, lora_begin_tx_done

    la   t0, lora_fsm_data
    sw   a0, LORA_DATA_TX_PTR(t0)
    sw   s0, LORA_DATA_TX_LEN(t0)
    call lora_start_tx_packet

lora_begin_tx_done:
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ------------------------------------------------------------------------------
# lora_activity_pulse - turn on activity LED and schedule auto-off
# ------------------------------------------------------------------------------
lora_activity_pulse:
    addi sp, sp, -4
    sw   ra, 0(sp)
    call led_activity_on
    li   a0, LORA_ACTIVITY_TIMER_ID
    li   a1, LORA_ACTIVITY_PULSE_TICKS
    call timer_arm_delta
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ------------------------------------------------------------------------------
# lora_activity_timer_handler - auto-clear radio activity LED pulse
# ------------------------------------------------------------------------------
.globl lora_activity_timer_handler
lora_activity_timer_handler:
    j    led_activity_off

# ==============================================================================
# Actions - Called by fsm_step
# ============================================================================== 

lora_action_nop:
    ret

lora_action_start_rx:
    addi sp, sp, -4
    sw   ra, 0(sp)
    la   t0, lora_fsm_data
    li   t1, 1
    sw   t1, LORA_DATA_RADIO_ON(t0)
    call lora_program_rx
    call lora_report_state
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

lora_action_standby:
    addi sp, sp, -4
    sw   ra, 0(sp)
    li   a0, SX_STDBY_RC
    call sx1278_set_standby
    la   t0, lora_fsm_data
    sw   zero, LORA_DATA_RADIO_ON(t0)
    call lora_report_state
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# a0 = FSM pointer (passed by fsm_step to all actions), a1 = config value, a2 = LORA_CMD_SET_* opcode
lora_action_cfg:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a1                     # value
    mv   s1, a2                     # opcode

    li   t0, LORA_CMD_SET_FREQ
    beq  s1, t0, lora_cfg_freq
    li   t0, LORA_CMD_SET_BW
    beq  s1, t0, lora_cfg_bw
    li   t0, LORA_CMD_SET_TXP
    beq  s1, t0, lora_cfg_txp
    li   t0, LORA_CMD_SET_SF
    beq  s1, t0, lora_cfg_sf
    li   t0, LORA_CMD_SET_CR
    beq  s1, t0, lora_cfg_cr
    li   t0, LORA_CMD_SET_PREAMBLE
    beq  s1, t0, lora_cfg_preamble
    li   t0, LORA_CMD_SET_IQI
    beq  s1, t0, lora_cfg_iqi
    li   t0, LORA_CMD_SET_SYNC_WORD
    beq  s1, t0, lora_cfg_sync_word
    li   t0, LORA_CMD_SET_PKT_PROFILE
    beq  s1, t0, lora_cfg_pkt_profile
    j    lora_cfg_done

lora_cfg_freq:
    beqz s0, lora_cfg_report_freq
    # Accept any frequency: store Hz, clear Frf (recomputed on next apply)
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_FREQ_HZ(t2)
    sw   zero, LORA_DATA_FREQ_REG(t2)  # 0 = must recompute on next apply
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx
lora_cfg_report_freq:
    call lora_report_freq
    j    lora_cfg_done

lora_cfg_bw:
    beqz s0, lora_cfg_report_bw
    # Map Hz value to SX1278 BW code using lookup table
    la   t2, lora_bw_hz_table
    la   t1, lora_bw_code_table
    li   t0, 0                      # index
lora_cfg_bw_search:
    li   a3, 10
    bgeu t0, a3, lora_cfg_report_bw # not found → report current
    slli a4, t0, 2
    add  a4, t2, a4
    lw   a4, 0(a4)                  # hz from table
    beq  s0, a4, lora_cfg_bw_found
    addi t0, t0, 1
    j    lora_cfg_bw_search
lora_cfg_bw_found:
    slli a5, t0, 2
    add  a5, t1, a5
    lw   a5, 0(a5)                  # SX code from table
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_BW_HZ(t2)
    sw   a5, LORA_DATA_BW_CODE(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx
lora_cfg_report_bw:
    call lora_report_bw
    j    lora_cfg_done

lora_cfg_txp:
    li   t0, 22
    bgtu s0, t0, lora_cfg_report_txp  # reject values > 22 dBm; echo current shadow back to host
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_TXP(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx
lora_cfg_report_txp:
    call lora_report_txp
    j    lora_cfg_done

lora_cfg_sf:
    li   t0, 5
    bltu s0, t0, lora_cfg_report_sf
    li   t0, 12
    bgtu s0, t0, lora_cfg_report_sf
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_SF(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx
lora_cfg_report_sf:
    call lora_report_sf
    j    lora_cfg_done

lora_cfg_cr:
    li   t0, 5
    bltu s0, t0, lora_cfg_report_cr
    li   t0, 8
    bgtu s0, t0, lora_cfg_report_cr
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_CR(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx
lora_cfg_report_cr:
    call lora_report_cr
    j    lora_cfg_done

lora_cfg_preamble:
    beqz s0, lora_cfg_done          # 0 = query (not supported for preamble, ignore)
    li   t0, 4
    bltu s0, t0, lora_cfg_done      # min 4 symbols
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_PREAMBLE(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx

lora_cfg_iqi:
    li   t0, 1
    bgtu s0, t0, lora_cfg_done      # AT layer validates; keep driver-facing bound here too
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_IQI(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx

lora_cfg_sync_word:
    li   t0, 255
    bgtu s0, t0, lora_cfg_done
    la   t2, lora_fsm_data
    sw   s0, LORA_DATA_SYNC_WORD(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx

lora_cfg_pkt_profile:
    la   t2, lora_fsm_data
    srli t0, s0, LORA_PKT_CRC_SHIFT
    andi t0, t0, 0x01
    sw   t0, LORA_DATA_CRC(t2)
    srli t0, s0, LORA_PKT_LDRO_SHIFT
    andi t0, t0, 0x03
    sw   t0, LORA_DATA_LDRO(t2)
    srli t0, s0, LORA_PKT_LEN_SHIFT
    andi t0, t0, 0xFF
    sw   t0, LORA_DATA_IMPLICIT_LEN(t2)
    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(t2)
    j    lora_cfg_apply_if_rx

lora_cfg_apply_if_rx:
    lbu  t0, FSM_STATE(a0)         # a0 = FSM pointer (invariant in all actions); read current state
    li   t1, LORA_STATE_RX_ACTIVE
    bne  t0, t1, lora_cfg_emit_report
    call lora_program_rx

lora_cfg_emit_report:
    li   t0, LORA_CMD_SET_FREQ
    beq  s1, t0, lora_cfg_report_freq
    li   t0, LORA_CMD_SET_BW
    beq  s1, t0, lora_cfg_report_bw
    li   t0, LORA_CMD_SET_TXP
    beq  s1, t0, lora_cfg_report_txp
    li   t0, LORA_CMD_SET_SF
    beq  s1, t0, lora_cfg_report_sf
    li   t0, LORA_CMD_SET_CR
    beq  s1, t0, lora_cfg_report_cr

lora_cfg_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ------------------------------------------------------------------------------
# lora_action_set_params - apply a full SF+BW+CR+preamble profile atomically
# ------------------------------------------------------------------------------
# a0 = FSM pointer, a1 = packed profile (see LORA_PARAMS_* in events.inc).
# All fields are pre-validated by at_fsm before the event is enqueued, so this
# action just unpacks, writes every shadow, marks the config dirty, and (only if
# the radio is currently in RX) reprograms RX so the new profile takes effect.
# Single event => no partially-applied configuration is ever observable (#3).
lora_action_set_params:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    lbu  s0, FSM_STATE(a0)         # remember current state for the RX re-arm test
    la   a0, lora_fsm_data

    # SF (bits [3:0])
    srli t0, a1, LORA_PARAMS_SF_SHIFT
    andi t0, t0, 0xF
    sw   t0, LORA_DATA_SF(a0)

    # CR host 5..8 (bits [11:8])
    srli t0, a1, LORA_PARAMS_CR_SHIFT
    andi t0, t0, 0xF
    sw   t0, LORA_DATA_CR(a0)

    # Preamble (bits [22:12])
    srli t0, a1, LORA_PARAMS_PRE_SHIFT
    andi t0, t0, 0x7FF
    sw   t0, LORA_DATA_PREAMBLE(a0)

    # BW code index (bits [7:4]) -> Hz + SX1278 register code via lookup tables
    srli t0, a1, LORA_PARAMS_BW_SHIFT
    andi t0, t0, 0xF
    slli t1, t0, 2                 # index * 4
    la   t2, lora_bw_hz_table
    add  t2, t2, t1
    lw   t2, 0(t2)
    sw   t2, LORA_DATA_BW_HZ(a0)
    la   t2, lora_bw_code_table
    add  t2, t2, t1
    lw   t2, 0(t2)
    sw   t2, LORA_DATA_BW_CODE(a0)

    li   t0, 1
    sw   t0, LORA_DATA_CFG_DIRTY(a0)

    # Reprogram RX only if currently listening; otherwise shadow is enough and
    # apply happens on the next RX entry / TX (matches lora_cfg_apply_if_rx).
    li   t0, LORA_STATE_RX_ACTIVE
    bne  s0, t0, lora_set_params_done
    call lora_program_rx

lora_set_params_done:
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# ------------------------------------------------------------------------------
# lora_action_reset - hardware reset + full reconfig + RX re-arm
# ------------------------------------------------------------------------------
# Called only through fsm_step via LORA_EV_RESET. The transition table sets the
# next FSM state to RX_ACTIVE; this action updates radio data shadows and hardware
# but does not write FSM_STATE directly.
lora_action_reset:
    addi sp, sp, -4
    sw   ra, 0(sp)

    call sx1278_init

    la   t0, lora_fsm_data
    li   t1, 1
    sw   t1, LORA_DATA_RADIO_ON(t0)
    sw   t1, LORA_DATA_CFG_DIRTY(t0)    # force full reprogram on next apply
    sw   zero, LORA_DATA_FREQ_REG(t0)   # force Frf recompute from freq_hz
    sw   zero, LORA_DATA_TX_PTR(t0)     # abort any in-flight TX
    sw   zero, LORA_DATA_TX_LEN(t0)

    call lora_program_rx

    li   a0, RADIO_NOTIFY_READY
    li   a1, 0
    li   a2, 0
    call lora_emit_notify

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# a1 = payload ptr, a2 = payload len
lora_action_tx_frame:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    mv   s0, a2
    beqz s0, lora_action_tx_frame_done
    mv   a0, a1
    mv   a1, s0
    call lora_begin_tx_packet

lora_action_tx_frame_done:
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# Called on TX_ACTIVE + DIO0.
# On final TX outcome, request RX re-entry as a future event to avoid blocking
# the main event loop on a potentially wedged radio reconfiguration path.
lora_action_tx_done:
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)

    call sx1278_get_irq_status
    mv   s0, a0
    la   t0, lora_last_irq_flags
    sw   s0, 0(t0)
    mv   a0, s0
    call sx1278_clear_irq

    andi t0, s0, SX_IRQ_TX_DONE
    bnez t0, lora_tx_done_confirmed
    andi t0, s0, SX_IRQ_TIMEOUT
    bnez t0, lora_tx_done_timeout

    la   t0, lora_fsm_data
    sw   zero, LORA_DATA_TX_PTR(t0)
    sw   zero, LORA_DATA_TX_LEN(t0)
    li   a0, RADIO_NOTIFY_TX_UNEXPECTED
    mv   a1, s0
    li   a2, 0
    call lora_emit_notify
    j    lora_tx_done_rearm

lora_tx_done_timeout:
    la   t0, lora_fsm_data
    sw   zero, LORA_DATA_TX_PTR(t0)
    sw   zero, LORA_DATA_TX_LEN(t0)
    li   a0, RADIO_NOTIFY_TX_TIMEOUT
    mv   a1, s0
    li   a2, 0
    call lora_emit_notify
    j    lora_tx_done_rearm

lora_tx_done_confirmed:
    la   t0, lora_fsm_data
    sw   zero, LORA_DATA_TX_PTR(t0)
    sw   zero, LORA_DATA_TX_LEN(t0)
    lw   t1, LORA_DATA_TX_COUNT(t0)
    addi t1, t1, 1
    sw   t1, LORA_DATA_TX_COUNT(t0)
    call lora_activity_pulse
    li   a0, RADIO_NOTIFY_TX_DONE
    mv   a1, s0
    li   a2, 0
    call lora_emit_notify
    j    lora_tx_done_rearm

lora_tx_done_rearm:
    call lora_queue_soft_rx

lora_tx_done_done:
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# Called on RX_ACTIVE + DIO0 -> RX_ACTIVE
#
# Raw LoRa reception: reads the full SX1278 payload directly into lora_rx_buffer.
# CRC validation is done by SX1278 hardware (PayloadCrcError flag).
# Stack: 0=ra 4=s0(irqflags) 8=s1(len) 12=fifo_addr 16=rssi_raw 20=snr_raw

lora_action_rx_irq:
    addi sp, sp, -24
    sw   ra,  0(sp)
    sw   s0,  4(sp)
    sw   s1,  8(sp)

    call sx1278_get_irq_status
    mv   s0, a0                     # s0 = IRQ flags
    mv   a0, s0
    call sx1278_clear_irq

    # CRC error takes priority (SX1278 may set both RxDone+CrcError)
    andi t0, s0, SX_IRQ_CRC_ERR
    bnez t0, lora_rx_crc_err

    # Check RxDone
    andi t0, s0, SX_IRQ_RX_DONE
    beqz t0, lora_rx_reenter        # spurious or timeout -> just re-arm

    # Get payload length and FIFO start address
    call sx1278_get_rx_buffer_status
    mv   s1, a0                     # s1 = payload length (bytes)
    sw   a1, 12(sp)                 # save fifo_start_addr

    beqz s1, lora_rx_reenter

    # Clamp to rx buffer size
    li   t0, LORA_RX_BUF_SIZE
    bleu s1, t0, lora_rx_len_ok
    mv   s1, t0
lora_rx_len_ok:

    # Read RSSI and SNR before FIFO access
    call sx1278_get_packet_status
    sw   a0, 16(sp)                 # rssi_raw
    sw   a1, 20(sp)                 # snr_raw

    call sx1278_get_frequency_error # a0 = signed FEI raw
    la   t0, lora_fsm_data
    lw   a1, LORA_DATA_BW_HZ(t0)
    call lora_fei_raw_to_hz
    la   t0, lora_last_fei_hz
    sw   a0, 0(t0)

    # Read raw payload directly into lora_rx_buffer (no header byte skipped)
    la   a0, lora_rx_buffer
    lw   a1, 12(sp)
    mv   a2, s1
    call sx1278_read_buffer

    # Store metrics
    lw   t0, 16(sp)
    la   t1, lora_last_rssi_raw
    sw   t0, 0(t1)
    lw   t0, 20(sp)
    la   t1, lora_last_snr_raw
    sw   t0, 0(t1)
    la   t0, lora_last_rx_len
    sw   s1, 0(t0)

    # Increment RX counter
    la   t0, lora_fsm_data
    lw   t1, LORA_DATA_RX_COUNT(t0)
    addi t1, t1, 1
    sw   t1, LORA_DATA_RX_COUNT(t0)

    call lora_activity_pulse

    # Emit RX_READY: data0=len, data1=(rssi_raw | snr_raw<<8)
    lw   t0, 16(sp)
    lw   t1, 20(sp)
    andi t0, t0, 0xFF
    andi t1, t1, 0xFF
    slli t1, t1, 8
    or   a2, t0, t1
    mv   a1, s1
    li   a0, RADIO_NOTIFY_RX_READY
    call lora_emit_notify
    j    lora_rx_reenter

lora_rx_crc_err:
    li   a0, RADIO_NOTIFY_CRC_ERR
    li   a1, 0
    li   a2, 0
    call lora_emit_notify

lora_rx_reenter:
    li   a0, 0
    call sx1278_set_rx

    lw   s1,  8(sp)
    lw   s0,  4(sp)
    lw   ra,  0(sp)
    addi sp, sp, 24
    ret


# ==============================================================================
# FEI conversion per SX1276/7/8/9 datasheet section 4.1.5:
#   Ferror_Hz = fei_raw * 2^24 / Fxtal * BW[kHz] / 500
# With Fxtal=32 MHz:
#   Ferror_Hz ~= fei_raw * BW[kHz] / 954
# Product fits in 32 bits: abs(fei_raw)<=524288, BW<=500 kHz.
# ==============================================================================

lora_fei_raw_to_hz:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)

    mv   s0, a0                    # signed raw, keep original sign
    bgez a0, lora_fei_abs_ready
    sub  a0, zero, a0
lora_fei_abs_ready:
    mv   s1, a0                    # abs(raw)

    mv   a0, a1                    # bw_hz
    li   a1, 1000
    call lora_udiv32               # a0 = bw_khz
    mv   a1, a0                    # multiplier = bw_khz
    mv   a0, s1                    # multiplicand = abs(raw)
    call lora_umul32               # a0 = abs(raw) * bw_khz

    li   a1, 954
    call lora_udiv32               # a0 = abs(error_hz)
    bgez s0, lora_fei_done
    sub  a0, zero, a0

lora_fei_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# Unsigned multiply without M extension. INPUTS a0=unsigned, a1=unsigned.
lora_umul32:
    li   t0, 0
lora_umul_loop:
    beqz a1, lora_umul_done
    andi t1, a1, 1
    beqz t1, lora_umul_skip_add
    add  t0, t0, a0
lora_umul_skip_add:
    slli a0, a0, 1
    srli a1, a1, 1
    j    lora_umul_loop
lora_umul_done:
    mv   a0, t0
    ret


# ==============================================================================
# Frf computation: freq_hz * 2^19 / 32e6, two-step, no 32-bit overflow
# ==============================================================================

lora_udiv32:
    mv   a2, a0
    li   a0, 0
    li   a3, 0
    li   t2, 32
lora_udiv32_loop:
    slli a3, a3, 1
    srli t0, a2, 31
    or   a3, a3, t0
    slli a2, a2, 1
    slli a0, a0, 1
    bltu a3, a1, lora_udiv32_skip
    sub  a3, a3, a1
    ori  a0, a0, 1
lora_udiv32_skip:
    addi t2, t2, -1
    bnez t2, lora_udiv32_loop
    mv   a1, a3
    ret

lora_frf_from_hz:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    li   a1, 1000000
    call lora_udiv32            # a0=freq_mhz, a1=rem_hz
    mv   s1, a0
    mv   a0, a1
    li   a1, 1000
    call lora_udiv32            # a0=freq_khz_rem
    slli a0, a0, 14
    li   a1, 1000
    call lora_udiv32            # a0=frac
    slli s1, s1, 14
    add  a0, s1, a0
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

.section .rodata
.align 2

lora_bw_hz_table:
    .word 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000

lora_bw_code_table:
    .word 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90
