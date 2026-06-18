# ==============================================================================
# subfsm/at_fsm.S - AT Command Interface
# ==============================================================================
#
# Implements a raw LoRa AT command protocol on UART1.
# The CH32V003 acts as a modem: Python/host sends AT commands, the MCU
# translates them to SPI operations on the SX1278 via lora_fsm.S.
#
# PROTOCOL (extensions marked with [*]):
#   AT                              → +OK
#   AT+RESET                        → +RESET\r\n+READY
#   AT+MODE=<0|1>                   → +OK  (0=RX active, 1=standby)
#   AT+MODE?                        → +MODE=<n>\r\n+OK
#   AT+BAND=<hz>                    → +OK  (e.g. 433175000)
#   AT+BAND?                        → +BAND=<hz>\r\n+OK
#   AT+PARAMETER=<SF>,<BW>,<CR>,<P> → +OK
#   AT+PARAMETER?                   → +PARAMETER=<SF>,<BW>,<CR>,<P>\r\n+OK
#   AT+PKT=<CRC>,<LDRO>,<LEN>       → +OK  (LEN=0 explicit, >0 implicit)
#   AT+PKT?                         → +PKT=<CRC>,<LDRO>,<LEN>\r\n+OK
#   AT+CRFOP=<0-15>                 → +OK  (TX power dBm from 0 to 15)
#   AT+CRFOP?                       → +CRFOP=<n>\r\n+OK
#   AT+IQI=<0|1>                    → +OK  (0=normal IQ, 1=inverted IQ)
#   AT+IQI?                         → +IQI=<n>\r\n+OK
#   AT+ADDRESS=<0-65535>            → +OK  (prepended to +RCV addr field)
#   AT+ADDRESS?                     → +ADDRESS=<n>\r\n+OK
#   AT+NETWORKID=<0-16>             → +OK  (stored, not used in raw LoRa)
#   AT+NETWORKID?                   → +NETWORKID=<n>\r\n+OK
#   AT+SEND=<addr>,<len>,<hexdata>  → +OK (then +SENDED when TX done) [*hex]
#   AT+VER?                         → +VER=at-os3-0.1.0\r\n+OK
#   AT+UID?                         → +UID=000000000000\r\n+OK
#   AT+FACTORY                      → defaults + +FACTORY\r\n
#
# NOTIFICATIONS (unsolicited):
#   +RCV=<addr>,<len>,<hexdata>,<rssi_dbm>,<snr_db>,<freq_err_hz>\r\n  [*hex]
#       (freq_err_hz: signed SX1278 FEI measured on the received packet)
#   +SENDED\r\n
#   +ERR=<code>\r\n   (1=CRC, 4=unknown command)
#   +READY\r\n        (startup / after reset)
#
# [*] DATA ENCODING EXTENSION:
#   Payload data uses uppercase hex (2 chars/byte) for binary
#   safety. AT+SEND=0,4,DEADBEEF sends 4 bytes [0xDE 0xAD 0xBE 0xEF].
#
# DEFAULTS (AT+FACTORY):
#   BAND=433175000, SF=12, BW=7 (125kHz), CR=1 (4/5), PREAMBLE=8, CRFOP=15
#   ADDRESS=0, NETWORKID=0
#
# BW CODES:
#   0=7.8k 1=10.4k 2=15.6k 3=20.8k 4=31.25k 5=41.7k 6=62.5k
#   7=125k(default) 8=250k 9=500k
#
# CR CODES: 1=4/5, 2=4/6, 3=4/7, 4=4/8
#   Internal lora_fsm uses 5-8 (host value) — at_fsm adds 4 on send.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): Host commands reach the radio only by
#     enqueuing EV_SOFT_LORA_CMD; results return as EV_SOFT_RADIO_NOTIFY events.
#   - Article 7 (Handler): Stateless event reactions (RX pump + radio notify);
#     this module holds shadow config but performs no FSM state transitions.
#   - Article 9 (Structural Enforcement): Registers via register_activity only.
#   - Article 10 (Layer Roles): Talks to the radio through events, never the
#     driver — with one explicit, read-only exception (AT+REG, see below).
#
# INLINE MARKER LEGEND — the "(#n)" tags in the parser map to the constitution:
#   (#1) Article 3.1 Distinguishability / PO-8: every malformed line is rejected,
#        never silently coerced into a valid command (strict field/end checks).
#   (#2) PO-8 Degeneracy Audit: lengths are bounds-checked and rejected (not
#        truncated), so one logical payload == exactly one on-air packet == one +RCV.
#   (#3) PO-7 Temporal Coherence: the event is enqueued first and the shadow config
#        is committed only if it was accepted — no partial or false state is shown.
#   (#4) PO-7 Temporal Coherence: +READY/+SENDED are emitted only after the radio
#        action actually completes, so each acknowledgement is causally truthful.
#   (#5) Article 3.1 Distinguishability / PO-8: out-of-range values are rejected.
#   (#8) Article 10 Layer Roles / Article 9 Structural Enforcement: the host layer
#        never reaches hardware except through the canonical event pipeline.
#
# ==============================================================================

.include "core/events.inc"
.include "subfsm/lora_data.inc"

# ------------------------------------------------------------------------------
# External dependencies
# ------------------------------------------------------------------------------
.extern lora_fsm_data
.extern register_activity
.extern event_enqueue
.extern uart_hw_rx_ring_get
.extern console_putc
.extern lora_rx_buffer          # received packet data
.extern lora_last_rssi_raw
.extern lora_last_snr_raw
.extern lora_last_fei_hz
.extern radio_probe
.ifdef CONFIG_RADIO_SX1278
.extern sx1278_read_reg         # diagnostic register read (AT+REG)
.endif

# ------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------
.equ AT_LINE_MAX,   128         # max command line length (chars)
.equ AT_TX_MAX,     128         # max TX payload buffer (decoded bytes)

.equ AT_DEFAULT_FREQ,   433175000
.equ AT_DEFAULT_SF,     12
.equ AT_DEFAULT_BW,     7       # AT BW code: 125 kHz
.equ AT_DEFAULT_CR,     1       # AT CR code: 4/5
.equ AT_DEFAULT_PRE,    8
.equ AT_DEFAULT_POWER,  15
.equ AT_DEFAULT_ADDR,      0
.equ AT_DEFAULT_NETID,     0
.equ AT_DEFAULT_SYNC_WORD, 0x12      # LoRa public sync word (matches LORA_SYNC_WORD_PUBLIC)

# NOTE: the AT BW-code → Hz conversion lives in lora_fsm.S (lora_bw_hz_table).
# AT+PARAMETER ships the raw BW code inside the packed LORA_CMD_SET_PARAMS event,
# so at_fsm no longer needs its own Hz table.

# ==============================================================================
# BSS Section
# ==============================================================================

.section .bss
.align 4

at_line_buf:    .space AT_LINE_MAX
at_line_len:    .word 0
at_uart_synced: .word 0

# AT-layer-only shadows (no lora_fsm_data counterpart or incompatible encoding)
at_cfg_bw:      .word 0     # AT BW code 0-9 (lora_fsm_data stores Hz, not AT code)
at_cfg_addr:    .word 0
at_cfg_netid:   .word 0
# All other radio config (freq, sf, cr, preamble, power, iqi, crc, ldro, len,
# syncword) is read directly from lora_fsm_data — single source of truth.

at_tx_buf:      .space AT_TX_MAX
at_tx_len:      .word 0

# ==============================================================================
# RODATA Section - constant strings
# ==============================================================================

.section .rodata
.align 2

at_str_ok:      .ascii "+OK\r\n"
at_str_ok_end:
.equ AT_STR_OK_LEN, (at_str_ok_end - at_str_ok)

at_str_ready:   .ascii "+READY\r\n"
at_str_ready_end:
.equ AT_STR_READY_LEN, (at_str_ready_end - at_str_ready)

at_str_reset:   .ascii "+RESET\r\n"
at_str_reset_end:
.equ AT_STR_RESET_LEN, (at_str_reset_end - at_str_reset)

at_str_sended:  .ascii "+SENDED\r\n"
at_str_sended_end:
.equ AT_STR_SENDED_LEN, (at_str_sended_end - at_str_sended)

at_str_factory: .ascii "+FACTORY\r\n"
at_str_factory_end:
.equ AT_STR_FACTORY_LEN, (at_str_factory_end - at_str_factory)

at_str_ver:     .ascii "+VER=at-os3-0.1.0\r\n"
at_str_ver_end:
.equ AT_STR_VER_LEN, (at_str_ver_end - at_str_ver)

at_str_uid:     .ascii "+UID=000000000000\r\n"
at_str_uid_end:
.equ AT_STR_UID_LEN, (at_str_uid_end - at_str_uid)

at_str_crlf:    .ascii "\r\n"
at_str_comma:   .ascii ","
at_kw_probeeq:  .ascii "+PROBE="

at_str_err1:    .ascii "+ERR=1\r\n"
at_str_err1_end:
.equ AT_STR_ERR1_LEN, (at_str_err1_end - at_str_err1)

at_str_err4:    .ascii "+ERR=4\r\n"
at_str_err4_end:
.equ AT_STR_ERR4_LEN, (at_str_err4_end - at_str_err4)

at_hex_chars:   .ascii "0123456789ABCDEF"

# ==============================================================================
# KERNEL_INIT Section
# ==============================================================================

.section .kernel_init, "a"
.align 2
at_init_ptr:    .word at_init

# ==============================================================================
# TEXT Section
# ==============================================================================

.section .text
.align 2

# ==============================================================================
# at_init - Initialize AT FSM, register handlers, emit +READY
# ==============================================================================

.globl at_init
at_init:
    addi sp, sp, -4
    sw   ra, 0(sp)

    # Initialize AT-layer-only shadows (defaults match lora_defaults_apply)
    la   t0, at_cfg_bw
    li   t1, AT_DEFAULT_BW
    sw   t1, 0(t0)
    la   t0, at_cfg_addr
    sw   zero, 0(t0)
    la   t0, at_cfg_netid
    sw   zero, 0(t0)
    la   t0, at_line_len
    sw   zero, 0(t0)
    la   t0, at_uart_synced
    sw   zero, 0(t0)

    # Register handler for UART RX pump events
    li   a0, EV_KIND_SOFT
    li   a1, EV_SOFT_RADIO_RX_PUMP
    la   a2, at_rx_handler
    call register_activity

    # Register handler for radio notifications
    li   a0, EV_KIND_SOFT
    li   a1, EV_SOFT_RADIO_NOTIFY
    la   a2, at_notify_handler
    call register_activity

    # Emit startup banner
    la   a0, at_str_ready
    li   a1, AT_STR_READY_LEN
    call at_emit_buf

    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# at_rx_handler - Drain UART RX ring, accumulate line, parse on newline
# ==============================================================================

at_rx_handler:
    addi sp, sp, -4
    sw   ra, 0(sp)

at_rx_loop:
    call uart_hw_rx_ring_get    # a0=1 if byte available, a1=byte
    beqz a0, at_rx_done

    li   t0, '\n'
    beq  a1, t0, at_rx_newline
    li   t0, '\r'
    beq  a1, t0, at_rx_loop    # ignore CR

    # Accumulate into line buffer if space available
    la   t0, at_line_len
    lw   t1, 0(t0)
    li   t2, (AT_LINE_MAX - 1)
    bgeu t1, t2, at_rx_loop    # buffer full, discard
    la   t2, at_line_buf
    add  t2, t2, t1
    sb   a1, 0(t2)
    addi t1, t1, 1
    sw   t1, 0(t0)
    j    at_rx_loop

at_rx_newline:
    # Parse the accumulated line
    la   a0, at_line_buf
    la   t0, at_line_len
    lw   a1, 0(t0)
    sw   zero, 0(t0)            # reset length
    beqz a1, at_rx_loop         # empty line, ignore
    # Null-terminate at buf[len] so the strict parsers (at_parse_uint_strict,
    # at_expect_end, at_match_kw) never read past the real line into stale buffer
    # bytes from a previous command (buf is AT_LINE_MAX, len capped at
    # AT_LINE_MAX-1, so buf[len] is always a valid index).
    add  t1, a0, a1
    sb   zero, 0(t1)
    call at_parse_line
    # Re-enqueue one RX_PUMP so LoRa FSM events triggered by this command
    # execute before the next AT line is parsed. Without this, a setter and a
    # query in the same UART burst are parsed back-to-back in one event pass,
    # and the query reads stale lora_fsm_data (the LoRa event hasn't run yet).
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_RADIO_RX_PUMP << EV_ID_SHIFT))
    li   a1, 0
    li   a2, 0
    call event_enqueue            # if queue full: bytes remain in ring, ISR will retry

at_rx_done:
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ==============================================================================
# at_notify_handler - Translate RADIO_NOTIFY events to AT output on UART
# ==============================================================================
# a0 = event header (flags = RADIO_NOTIFY_* code)
# a1 = data0 (rx_len for RX_READY; irq_flags for TX events)
# a2 = data1 ((rssi_raw | snr_raw<<8) for RX_READY)

at_notify_handler:
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a1, 12(sp)             # save data0

    slli t0, a0, 16             # extract notify code from flags (low 16 bits)
    srli t0, t0, 16

    li   t1, RADIO_NOTIFY_TX_DONE
    beq  t0, t1, at_notify_tx_done

    li   t1, RADIO_NOTIFY_TX_TIMEOUT
    beq  t0, t1, at_notify_done

    li   t1, RADIO_NOTIFY_TX_UNEXPECTED
    beq  t0, t1, at_notify_tx_unexpected

    li   t1, RADIO_NOTIFY_RX_READY
    beq  t0, t1, at_notify_rx_ready

    li   t1, RADIO_NOTIFY_CRC_ERR
    beq  t0, t1, at_notify_crc_err

    li   t1, RADIO_NOTIFY_READY
    beq  t0, t1, at_notify_ready

    j    at_notify_done

at_notify_tx_done:
    la   a0, at_str_sended
    li   a1, AT_STR_SENDED_LEN
    call at_emit_buf
    j    at_notify_done

at_notify_ready:
    # Emitted by lora_fsm only AFTER a real radio reset + RX re-arm (#4),
    # so +READY here is truthful (the host can now use the radio again).
    la   a0, at_str_ready
    li   a1, AT_STR_READY_LEN
    call at_emit_buf
    j    at_notify_done

at_notify_tx_unexpected:
    la   a0, at_str_err1
    li   a1, AT_STR_ERR1_LEN
    call at_emit_buf
    j    at_notify_done

at_notify_crc_err:
    la   a0, at_str_err1
    li   a1, AT_STR_ERR1_LEN
    call at_emit_buf
    j    at_notify_done

at_notify_rx_ready:
    # a2 = (rssi_raw & 0xFF) | ((snr_raw & 0xFF) << 8)
    # data0 (saved at 12(sp)) = rx_len
    mv   s0, a2                 # s0 = packed rssi/snr
    lw   s1, 12(sp)             # s1 = rx_len

    # +RCV=<addr>,
    li   a0, '+'
    call console_putc
    li   a0, 'R'
    call console_putc
    li   a0, 'C'
    call console_putc
    li   a0, 'V'
    call console_putc
    li   a0, '='
    call console_putc

    # addr = at_cfg_addr
    la   t0, at_cfg_addr
    lw   a0, 0(t0)
    call at_emit_uint
    li   a0, ','
    call console_putc

    # len = rx_len
    mv   a0, s1
    call at_emit_uint
    li   a0, ','
    call console_putc

    # hexdata from lora_rx_buffer
    la   a0, lora_rx_buffer
    mv   a1, s1
    call at_emit_hex
    li   a0, ','
    call console_putc

    # rssi_dbm = -164 + rssi_raw (LF port, 400 MHz band)
    andi t0, s0, 0xFF           # rssi_raw (0-255)
    li   t1, -164
    add  a0, t1, t0             # rssi_dbm (signed)
    call at_emit_int
    li   a0, ','
    call console_putc

    # snr_db = sign_extend(snr_raw) >> 2
    srli t0, s0, 8
    andi t0, t0, 0xFF           # snr_raw as unsigned byte
    slli t0, t0, 24
    srai t0, t0, 24             # sign extend to 32-bit
    srai a0, t0, 2              # divide by 4 (arithmetic shift)
    call at_emit_int
    li   a0, ','
    call console_putc

    # frequency_error_hz from SX1278 FEI, measured on the received packet
    la   t0, lora_last_fei_hz
    lw   a0, 0(t0)
    call at_emit_int

    # \r\n
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc

at_notify_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ==============================================================================
# at_parse_line - Parse and dispatch one complete AT command line
# ==============================================================================
# a0 = pointer to line buffer (not null-terminated, no trailing \r\n)
# a1 = length in bytes

at_parse_line:
    addi sp, sp, -32
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    # 12/16/20/24(sp) = scratch slots A..D (field stash across calls that clobber
    # all caller-saved regs: at_parse_uint_strict, at_expect_*, event_enqueue).
    # AT+PARAMETER uses all four (SF/BW/CR/preamble); other commands use 12(sp).
    mv   s0, a0                 # s0 = buf ptr
    mv   s1, a1                 # s1 = length

    # After flashing/opening CDC, the first UART line can contain stale or
    # probe bytes before the host's first AT.  Until we have seen one AT prefix,
    # resynchronize to the first "AT" inside the line, or silently discard a
    # line with no AT at all.  Once synced, strict parsing below is unchanged.
    la   t0, at_uart_synced
    lw   t0, 0(t0)
    bnez t0, at_parse_check_prefix

    li   t2, 2
    bltu s1, t2, at_parse_done
    lbu  t0, 0(s0)
    li   t1, 'A'
    bne  t0, t1, at_parse_resync_scan
    lbu  t0, 1(s0)
    li   t1, 'T'
    beq  t0, t1, at_parse_mark_synced

at_parse_resync_scan:
    mv   t0, s0                 # scan pointer
    mv   t1, s1                 # remaining length
at_parse_resync_loop:
    li   t2, 2
    bltu t1, t2, at_parse_done  # no AT in this unsynced startup line
    lbu  a0, 0(t0)
    li   a1, 'A'
    bne  a0, a1, at_parse_resync_next
    lbu  a0, 1(t0)
    li   a1, 'T'
    beq  a0, a1, at_parse_resync_found
at_parse_resync_next:
    addi t0, t0, 1
    addi t1, t1, -1
    j    at_parse_resync_loop

at_parse_resync_found:
    mv   s0, t0                 # parse from recovered AT prefix
    mv   s1, t1

at_parse_mark_synced:
    la   t0, at_uart_synced
    li   t1, 1
    sw   t1, 0(t0)

at_parse_check_prefix:
    # Must start with "AT"
    li   t2, 2
    bltu s1, t2, at_parse_err4
    lbu  t0, 0(s0)
    li   t1, 'A'
    bne  t0, t1, at_parse_err4
    lbu  t0, 1(s0)
    li   t1, 'T'
    bne  t0, t1, at_parse_err4

    # Bare "AT" (len==2)
    li   t2, 2
    beq  s1, t2, at_parse_ok

    # Must be followed by '+' or bad command
    lbu  t0, 2(s0)
    li   t1, '+'
    bne  t0, t1, at_parse_err4

    # Point to command name (after "AT+")
    addi s0, s0, 3
    addi s1, s1, -3

    # Dispatch on first char of command name
    beqz s1, at_parse_err4
    lbu  t0, 0(s0)

    li   t1, 'R'
    beq  t0, t1, at_try_reset
    li   t1, 'M'
    beq  t0, t1, at_try_mode
    li   t1, 'B'
    beq  t0, t1, at_try_band
    li   t1, 'P'
    beq  t0, t1, at_try_parameter
    li   t1, 'C'
    beq  t0, t1, at_try_crfop
    li   t1, 'A'
    beq  t0, t1, at_try_address
    li   t1, 'N'
    beq  t0, t1, at_try_networkid
    li   t1, 'S'
    beq  t0, t1, at_try_syncword
    li   t1, 'V'
    beq  t0, t1, at_try_ver
    li   t1, 'U'
    beq  t0, t1, at_try_uid
    li   t1, 'F'
    beq  t0, t1, at_try_factory
    li   t1, 'I'
    beq  t0, t1, at_try_ipr     # AT+IPR / AT+IQI
    j    at_parse_err4

# --- AT+RESET ---
at_try_reset:
    # Expect "RESET"
    mv   a0, s0
    la   a1, at_kw_reset
    li   a2, 5
    call at_match_kw
    beqz a0, at_try_reg         # not "RESET" → maybe "REG"
    addi a0, s0, 5             # reject suffix: "AT+RESETxx" must end here (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    # Real reset via lora_fsm (#1, layering #8): enqueue LORA_CMD_RESET, which
    # runs sx1278_init (hardware reset + full reconfig), re-arms RX, and then
    # emits RADIO_NOTIFY_READY.  +READY is therefore printed by at_notify_handler
    # only AFTER the real reset completes — never optimistically here (#4).
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_RESET)
    li   a1, 0
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4     # queue full → report failure, do not ack (#3)
    la   a0, at_str_reset
    li   a1, AT_STR_RESET_LEN
    call at_emit_buf           # +RESET ack now; +READY arrives via notify
    j    at_parse_done

# --- AT+REG=<hh>  (diagnostic: read SX1278 register, addr in 2 hex digits) ---
# CONSTITUTIONAL EXCEPTION (Article 10 layer roles):
#   This is the ONE place where at_fsm calls the radio driver directly
#   (sx1278_read_reg).  It is a deliberately isolated, READ-ONLY hardware probe:
#   it never mutates radio or FSM state, only reads a single register and echoes
#   it.  Its sole purpose is to prove the SPI link + chip are alive when the
#   normal +OK path (which only proves UART+parser) cannot.  AT+REG=42 must
#   return +REG=12 (RegVersion).  Because it is pure observation with no side
#   effects, it does not weaken the event pipeline or FSM ownership invariants.
#   Strict format required: "REG=" + exactly 2 hex digits + end of line.
# s0 = command ptr (preserved). Uses 16(sp) as the 1-byte value scratch.
at_try_reg:
    mv   a0, s0
    la   a1, at_kw_reg
    li   a2, 3
    call at_match_kw
    beqz a0, at_parse_err4
.ifndef CONFIG_RADIO_SX1278
    j    at_parse_err4
.endif
    addi a0, s0, 3            # expect '=' after "REG"
    li   a1, '='
    call at_expect_char       # a0 = ptr to first hex digit, or 0
    beqz a0, at_parse_err4
    addi a1, sp, 16           # decode exactly 1 byte (2 hex digits) into 16(sp)
    li   a2, 1
    call at_parse_hex_exact   # a0 = ptr past hex, or 0 on bad/short hex
    beqz a0, at_parse_err4
    call at_expect_end        # reject trailing junk: "AT+REG=42xx" → +ERR (#1)
    beqz a0, at_parse_err4
    lbu  a0, 16(sp)           # register address
.ifdef CONFIG_RADIO_SX1278
    call sx1278_read_reg      # a0 = register value (preserves s0/s1)
.else
    li   a0, 0
.endif
    sb   a0, 16(sp)           # store value byte for at_emit_hex
    # emit "+REG="
    la   a0, at_kw_regeq
    li   a1, 5
    call at_emit_buf
    # emit value byte as 2 hex chars
    addi a0, sp, 16
    li   a1, 1
    call at_emit_hex
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+MODE ---
at_try_mode:
    mv   a0, s0
    la   a1, at_kw_mode
    li   a2, 4
    call at_match_kw
    beqz a0, at_parse_err4
    lbu  t0, 4(s0)
    li   t1, '?'
    beq  t0, t1, at_mode_query
    li   t1, '='
    bne  t0, t1, at_parse_err4
    # Parse value strictly: at least one digit, then end of line
    addi a0, s0, 5
    call at_parse_uint_strict   # a0=value, a1=ptr, a2=ndigits
    beqz a2, at_parse_err4      # empty field (#1)
    sw   a0, 12(sp)            # stash mode value
    mv   a0, a1
    call at_expect_end         # reject trailing junk (#1)
    beqz a0, at_parse_err4
    lw   a0, 12(sp)
    li   t0, 1                  # mode must be 0 or 1 (#5)
    bgtu a0, t0, at_parse_err4
    # Enqueue first; commit shadow only if the event was accepted (#3)
    beqz a0, at_mode_set_rx
    # mode=1: standby (radio off)
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_STATE)
    li   a1, 0                  # state=0 → radio off
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4
    j    at_parse_ok
at_mode_set_rx:
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_STATE)
    li   a1, 1                  # state=1 → radio on (RX)
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4
    j    at_parse_ok
at_mode_query:
    addi a0, s0, 5             # ptr after '?': reject "AT+MODE?xx" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    li   a0, 'M'
    call console_putc
    li   a0, 'O'
    call console_putc
    li   a0, 'D'
    call console_putc
    li   a0, 'E'
    call console_putc
    li   a0, '='
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_RADIO_ON(t0)
    xori a0, a0, 1
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+BAND ---
at_try_band:
    mv   a0, s0
    la   a1, at_kw_band
    li   a2, 4
    call at_match_kw
    beqz a0, at_parse_err4
    lbu  t0, 4(s0)
    li   t1, '?'
    beq  t0, t1, at_band_query
    li   t1, '='
    bne  t0, t1, at_parse_err4
    addi a0, s0, 5
    call at_parse_uint_strict   # a0=freq_hz, a1=ptr, a2=ndigits
    beqz a2, at_parse_err4      # empty field (#1)
    sw   a0, 12(sp)            # stash freq
    mv   a0, a1
    call at_expect_end         # reject trailing junk: "AT+BAND=433175000xx" (#1)
    beqz a0, at_parse_err4
    lw   a0, 12(sp)
    # Validate against SX1278 range 137-1020 MHz (#3); rejects 0 and garbage
    li   t0, 137000000
    bltu a0, t0, at_parse_err4
    li   t0, 1020000000
    bgtu a0, t0, at_parse_err4
    # Send to FSM; commit shadow only if the event was accepted (#3)
    li   t1, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_FREQ)
    mv   a1, a0
    mv   a0, t1
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4
    j    at_parse_ok
at_band_query:
    addi a0, s0, 5             # ptr after '?': reject "AT+BAND?xx" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    li   a0, 'B'
    call console_putc
    li   a0, 'A'
    call console_putc
    li   a0, 'N'
    call console_putc
    li   a0, 'D'
    call console_putc
    li   a0, '='
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_FREQ_HZ(t0)
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+PARAMETER ---
at_try_parameter:
    mv   a0, s0
    la   a1, at_kw_parameter
    li   a2, 9
    call at_match_kw
    beqz a0, at_try_pkt
    lbu  t0, 9(s0)
    li   t1, '?'
    beq  t0, t1, at_parameter_query
    li   t1, '='
    bne  t0, t1, at_parse_err4

    # Parse SF,BW,CR,Preamble strictly: each field non-empty, single ',' between
    # them (at_expect_char), end of line after preamble.  Stack slots survive the
    # calls (all clobber caller-saved regs):
    #   12(sp)=SF  16(sp)=BW(AT code)  20(sp)=CR(AT code)  24(sp)=preamble
    addi a0, s0, 10
    call at_parse_uint_strict   # SF
    beqz a2, at_parse_err4
    sw   a0, 12(sp)
    mv   a0, a1
    li   a1, ','
    call at_expect_char         # verify single ',' (#1)
    beqz a0, at_parse_err4
    call at_parse_uint_strict   # BW code
    beqz a2, at_parse_err4
    sw   a0, 16(sp)
    mv   a0, a1
    li   a1, ','
    call at_expect_char
    beqz a0, at_parse_err4
    call at_parse_uint_strict   # CR code
    beqz a2, at_parse_err4
    sw   a0, 20(sp)
    mv   a0, a1
    li   a1, ','
    call at_expect_char
    beqz a0, at_parse_err4
    call at_parse_uint_strict   # Preamble
    beqz a2, at_parse_err4
    sw   a0, 24(sp)
    mv   a0, a1
    call at_expect_end          # nothing may follow preamble (#1)
    beqz a0, at_parse_err4

    # Validate ALL fields before enqueuing anything (#2,#3).
    # 12=SF 16=BW 20=CR 24=preamble on stack.
    # SF in [6,12]
    lw   t0, 12(sp)
    li   t1, 6
    bltu t0, t1, at_parse_err4
    li   t1, 12
    bgtu t0, t1, at_parse_err4
    # BW code in [0,9]
    lw   t0, 16(sp)
    li   t1, 9
    bgtu t0, t1, at_parse_err4
    # CR (AT code) in [1,4]
    lw   t0, 20(sp)
    li   t1, 1
    bltu t0, t1, at_parse_err4
    li   t1, 4
    bgtu t0, t1, at_parse_err4
    # Preamble in [4,1023]
    lw   t0, 24(sp)
    li   t1, 4
    bltu t0, t1, at_parse_err4
    li   t1, 1023
    bgtu t0, t1, at_parse_err4

    # Pack the whole validated profile into one word and emit ONE event so the
    # radio applies all four fields atomically — no partial config possible (#3).
    #   SF[3:0] | BW[7:4] | CR_host[11:8] | preamble[22:12], CR_host = AT code + 4.
    lw   t0, 12(sp)                    # SF
    lw   t1, 16(sp)                    # BW code
    slli t1, t1, LORA_PARAMS_BW_SHIFT
    or   t0, t0, t1
    lw   t1, 20(sp)                    # CR (AT code 1-4)
    addi t1, t1, 4                     # → host 5-8
    slli t1, t1, LORA_PARAMS_CR_SHIFT
    or   t0, t0, t1
    lw   t1, 24(sp)                    # preamble
    slli t1, t1, LORA_PARAMS_PRE_SHIFT
    or   t0, t0, t1
    mv   a1, t0                        # packed profile in data0
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_PARAMS)
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4            # queue full → +ERR, shadows untouched (#3)

    # Commit only the AT-layer BW shadow (AT code); sf/cr/preamble live in lora_fsm_data.
    lw   t0, 16(sp)
    la   t1, at_cfg_bw
    sw   t0, 0(t1)
    j    at_parse_ok

at_try_pkt:
    mv   a0, s0
    la   a1, at_kw_pkt
    li   a2, 3
    call at_match_kw
    beqz a0, at_try_probe
    lbu  t0, 3(s0)
    li   t1, '?'
    beq  t0, t1, at_pkt_query
    li   t1, '='
    bne  t0, t1, at_parse_err4

    # Parse CRC,LDRO,LEN as one packet profile. CRC is 0/1, LDRO is
    # 0=force off, 1=force on, 2=auto, LEN 0=explicit or 1..255=implicit.
    addi a0, s0, 4
    call at_parse_uint_strict
    beqz a2, at_parse_err4
    sw   a0, 12(sp)
    mv   a0, a1
    li   a1, ','
    call at_expect_char
    beqz a0, at_parse_err4
    call at_parse_uint_strict
    beqz a2, at_parse_err4
    sw   a0, 16(sp)
    mv   a0, a1
    li   a1, ','
    call at_expect_char
    beqz a0, at_parse_err4
    call at_parse_uint_strict
    beqz a2, at_parse_err4
    sw   a0, 20(sp)
    mv   a0, a1
    call at_expect_end
    beqz a0, at_parse_err4

    lw   t0, 12(sp)
    li   t1, 1
    bgtu t0, t1, at_parse_err4
    lw   t0, 16(sp)
    li   t1, 2
    bgtu t0, t1, at_parse_err4
    lw   t0, 20(sp)
    li   t1, 255
    bgtu t0, t1, at_parse_err4

    lw   t0, 12(sp)
    slli t0, t0, LORA_PKT_CRC_SHIFT
    lw   t1, 16(sp)
    slli t1, t1, LORA_PKT_LDRO_SHIFT
    or   t0, t0, t1
    lw   t1, 20(sp)
    slli t1, t1, LORA_PKT_LEN_SHIFT
    or   t0, t0, t1
    li   t1, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_PKT_PROFILE)
    mv   a1, t0
    mv   a0, t1
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4
    j    at_parse_ok

at_try_probe:
    mv   a0, s0
    la   a1, at_kw_probe
    li   a2, 6
    call at_match_kw
    beqz a0, at_parse_err4
    addi a0, s0, 6
    call at_expect_end
    beqz a0, at_parse_err4

    call radio_probe
    srli t0, a0, 8
    sb   t0, 16(sp)
    sb   a0, 17(sp)

    la   a0, at_kw_probeeq
    li   a1, 7
    call at_emit_buf
    addi a0, sp, 16
    li   a1, 2
    call at_emit_hex
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

at_pkt_query:
    addi a0, s0, 4
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    la   a0, at_kw_pkt
    li   a1, 3
    call at_emit_buf
    li   a0, '='
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_CRC(t0)
    call at_emit_uint
    li   a0, ','
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_LDRO(t0)
    call at_emit_uint
    li   a0, ','
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_IMPLICIT_LEN(t0)
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

at_parameter_query:
    addi a0, s0, 10            # ptr after '?': reject "AT+PARAMETER?xx" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    # +PARAMETER=<SF>,<BW>,<CR>,<PRE>
    li   a0, '+'
    call console_putc
    # "PARAMETER="
    la   a0, at_kw_parameter
    li   a1, 9
    call at_emit_buf
    li   a0, '='
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_SF(t0)         # SF: same encoding
    call at_emit_uint
    li   a0, ','
    call console_putc
    la   t0, at_cfg_bw
    lw   a0, 0(t0)                    # BW: AT code (lora_fsm_data stores Hz)
    call at_emit_uint
    li   a0, ','
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_CR(t0)
    addi a0, a0, -4                   # host 5-8 → AT code 1-4
    call at_emit_uint
    li   a0, ','
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_PREAMBLE(t0)   # preamble: same encoding
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+CRFOP ---
at_try_crfop:
    mv   a0, s0
    la   a1, at_kw_crfop
    li   a2, 5
    call at_match_kw
    beqz a0, at_parse_err4
    lbu  t0, 5(s0)
    li   t1, '?'
    beq  t0, t1, at_crfop_query
    li   t1, '='
    bne  t0, t1, at_parse_err4
    addi a0, s0, 6
    call at_parse_uint_strict   # a0=power, a1=ptr, a2=ndigits
    beqz a2, at_parse_err4      # empty field (#1)
    sw   a0, 12(sp)
    mv   a0, a1
    call at_expect_end         # reject trailing junk (#1)
    beqz a0, at_parse_err4
    lw   a0, 12(sp)
    # Validate power 0-15 (AT CRFOP range) (#3)
    li   t0, 15
    bgtu a0, t0, at_parse_err4
    li   t1, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_TXP)
    mv   a1, a0
    mv   a0, t1
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4
    j    at_parse_ok
at_crfop_query:
    addi a0, s0, 6             # ptr after '?': reject "AT+CRFOP?xx" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    la   a0, at_kw_crfop
    li   a1, 5
    call at_emit_buf
    li   a0, '='
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_TXP(t0)
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+ADDRESS ---
at_try_address:
    mv   a0, s0
    la   a1, at_kw_address
    li   a2, 7
    call at_match_kw
    beqz a0, at_parse_err4
    lbu  t0, 7(s0)
    li   t1, '?'
    beq  t0, t1, at_addr_query
    li   t1, '='
    bne  t0, t1, at_parse_err4
    addi a0, s0, 8
    call at_parse_uint_strict   # a0=addr, a1=ptr, a2=ndigits
    beqz a2, at_parse_err4      # empty field (#1)
    sw   a0, 12(sp)
    mv   a0, a1
    call at_expect_end         # reject trailing junk (#1)
    beqz a0, at_parse_err4
    lw   a0, 12(sp)
    li   t0, 65535             # address 0..65535 (#5)
    bgtu a0, t0, at_parse_err4
    la   t0, at_cfg_addr
    sw   a0, 0(t0)
    j    at_parse_ok
at_addr_query:
    addi a0, s0, 8             # ptr after '?': reject "AT+ADDRESS?xx" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    la   a0, at_kw_address
    li   a1, 7
    call at_emit_buf
    li   a0, '='
    call console_putc
    la   t0, at_cfg_addr
    lw   a0, 0(t0)
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+NETWORKID ---
at_try_networkid:
    mv   a0, s0
    la   a1, at_kw_networkid
    li   a2, 9
    call at_match_kw
    beqz a0, at_parse_err4
    lbu  t0, 9(s0)
    li   t1, '?'
    beq  t0, t1, at_netid_query
    li   t1, '='
    bne  t0, t1, at_parse_err4
    addi a0, s0, 10
    call at_parse_uint_strict   # a0=netid, a1=ptr, a2=ndigits
    beqz a2, at_parse_err4      # empty field (#1)
    sw   a0, 12(sp)
    mv   a0, a1
    call at_expect_end         # reject trailing junk (#1)
    beqz a0, at_parse_err4
    lw   a0, 12(sp)
    li   t0, 16                # network id 0..16 (#5)
    bgtu a0, t0, at_parse_err4
    la   t0, at_cfg_netid
    sw   a0, 0(t0)
    j    at_parse_ok
at_netid_query:
    addi a0, s0, 10            # ptr after '?': reject "AT+NETWORKID?xx" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    la   a0, at_kw_networkid
    li   a1, 9
    call at_emit_buf
    li   a0, '='
    call console_putc
    la   t0, at_cfg_netid
    lw   a0, 0(t0)
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+SYNCWORD ---
at_try_syncword:
    mv   a0, s0
    la   a1, at_kw_syncword
    li   a2, 8
    call at_match_kw
    beqz a0, at_try_send
    lbu  t0, 8(s0)
    li   t1, '?'
    beq  t0, t1, at_syncword_query
    li   t1, '='
    bne  t0, t1, at_parse_err4
    addi a0, s0, 9
    call at_parse_uint_strict
    beqz a2, at_parse_err4
    sw   a0, 12(sp)
    mv   a0, a1
    call at_expect_end
    beqz a0, at_parse_err4
    lw   a1, 12(sp)
    li   t0, 255
    bgtu a1, t0, at_parse_err4
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_SYNC_WORD)
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4
    j    at_parse_ok

at_syncword_query:
    addi a0, s0, 9             # reject "AT+SYNCWORD?xx" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    la   a0, at_kw_syncword
    li   a1, 8
    call at_emit_buf
    li   a0, '='
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_SYNC_WORD(t0)
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+SEND ---
at_try_send:
    mv   a0, s0
    la   a1, at_kw_send
    li   a2, 4
    call at_match_kw
    beqz a0, at_parse_err4
    lbu  t0, 4(s0)
    li   t1, '='
    bne  t0, t1, at_parse_err4

    # Parse addr (ignored, we always transmit) — must be a non-empty number.
    addi a0, s0, 5
    call at_parse_uint_strict   # a0=addr, a1=ptr, a2=ndigits
    beqz a2, at_parse_err4      # empty addr field (#1)
    mv   a0, a1
    li   a1, ','
    call at_expect_char         # verify single ',' (#1)
    beqz a0, at_parse_err4

    # Parse len (declared decoded-byte count).
    call at_parse_uint_strict   # a0=len, a1=ptr, a2=ndigits
    beqz a2, at_parse_err4      # empty len field (#1)
    beqz a0, at_parse_err4      # len 0 is invalid for SEND (#2)
    li   t0, AT_TX_MAX          # reject (not truncate) len > AT_TX_MAX (#2)
    bgtu a0, t0, at_parse_err4
    sw   a0, 12(sp)             # save declared byte count
    mv   a0, a1
    li   a1, ','
    call at_expect_char         # verify single ',' before hexdata (#1)
    beqz a0, at_parse_err4

    # Decode EXACTLY <len> bytes of hex into at_tx_buf.  at_parse_hex_exact
    # fails on any non-hex char or if fewer than <len> bytes are present;
    # at_expect_end then rejects extra hex/junk, so the on-air payload always
    # matches the declared length and fits one raw packet = one +RCV (#2).
    la   a1, at_tx_buf
    lw   a2, 12(sp)            # declared byte count
    call at_parse_hex_exact    # a0 = ptr past hex, or 0 on bad/short hex
    beqz a0, at_parse_err4
    call at_expect_end         # reject trailing junk / extra hex (#1,#2)
    beqz a0, at_parse_err4

    # Send TX frame event
    la   t0, at_tx_buf
    lw   a2, 12(sp)            # len
    li   a3, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_TX_FRAME)
    mv   a1, t0                # ptr
    mv   a0, a3
    call event_enqueue
    bnez a0, at_parse_err4     # queue full → +ERR, no false +OK (#3)
    j    at_parse_ok

# --- AT+VER? ---  (keyword includes the '?', so nothing may follow it)
at_try_ver:
    mv   a0, s0
    la   a1, at_kw_ver
    li   a2, 4
    call at_match_kw
    beqz a0, at_parse_err4
    addi a0, s0, 4             # reject "AT+VER?junk" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    la   a0, at_str_ver
    li   a1, AT_STR_VER_LEN
    call at_emit_buf
    j    at_parse_ok

# --- AT+UID? ---  (keyword includes the '?', so nothing may follow it)
at_try_uid:
    mv   a0, s0
    la   a1, at_kw_uid
    li   a2, 4
    call at_match_kw
    beqz a0, at_parse_err4
    addi a0, s0, 4             # reject "AT+UID?junk" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    la   a0, at_str_uid
    li   a1, AT_STR_UID_LEN
    call at_emit_buf
    j    at_parse_ok

# --- AT+IPR / AT+IQI ---
# AT+IPR is a host baud setting; we never change baud, just ack "IPR" form.
# AT+IQI controls LoRa IQ inversion for satellites using inverted IQ.
at_try_ipr:
    mv   a0, s0
    la   a1, at_kw_ipr
    li   a2, 3
    call at_match_kw
    beqz a0, at_try_iqi
    j    at_parse_ok

at_try_iqi:
    mv   a0, s0
    la   a1, at_kw_iqi
    li   a2, 3
    call at_match_kw
    beqz a0, at_parse_err4     # bare "AT+I..." that is not IPR/IQI → +ERR
    lbu  t0, 3(s0)
    li   t1, '?'
    beq  t0, t1, at_iqi_query
    li   t1, '='
    bne  t0, t1, at_parse_err4
    addi a0, s0, 4
    call at_parse_uint_strict
    beqz a2, at_parse_err4
    sw   a0, 12(sp)
    mv   a0, a1
    call at_expect_end
    beqz a0, at_parse_err4
    lw   a0, 12(sp)
    li   t0, 1
    bgtu a0, t0, at_parse_err4
    li   t1, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_SET_IQI)
    mv   a1, a0
    mv   a0, t1
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4
    j    at_parse_ok

at_iqi_query:
    addi a0, s0, 4
    call at_expect_end
    beqz a0, at_parse_err4
    li   a0, '+'
    call console_putc
    la   a0, at_kw_iqi
    li   a1, 3
    call at_emit_buf
    li   a0, '='
    call console_putc
    la   t0, lora_fsm_data
    lw   a0, LORA_DATA_IQI(t0)
    call at_emit_uint
    li   a0, '\r'
    call console_putc
    li   a0, '\n'
    call console_putc
    j    at_parse_ok

# --- AT+FACTORY ---
at_try_factory:
    mv   a0, s0
    la   a1, at_kw_factory
    li   a2, 7
    call at_match_kw
    beqz a0, at_parse_err4
    addi a0, s0, 7             # reject "AT+FACTORYjunk" (#1)
    call at_expect_end
    beqz a0, at_parse_err4
    # Enqueue FIRST: commit nothing until the event is accepted (#3).
    # LORA_CMD_FACTORY resets lora_fsm_data and reinits hardware; +READY follows.
    li   a0, ((EV_KIND_SOFT << EV_KIND_SHIFT) | (EV_SOFT_LORA_CMD << EV_ID_SHIFT) | LORA_CMD_FACTORY)
    li   a1, 0
    li   a2, 0
    call event_enqueue
    bnez a0, at_parse_err4     # queue full → +ERR, no shadow touched
    # Reset AT-layer-only shadows; radio config is reset by lora_action_factory
    # via lora_defaults_apply — no duplicate state here (#3).
    la   t0, at_cfg_bw
    li   t1, AT_DEFAULT_BW
    sw   t1, 0(t0)
    la   t0, at_cfg_addr
    sw   zero, 0(t0)
    la   t0, at_cfg_netid
    sw   zero, 0(t0)
    la   a0, at_str_factory
    li   a1, AT_STR_FACTORY_LEN
    call at_emit_buf
    j    at_parse_done          # no +OK; +READY arrives via lora_fsm notify

at_parse_err4:
    la   a0, at_str_err4
    li   a1, AT_STR_ERR4_LEN
    call at_emit_buf
    j    at_parse_done

at_parse_ok:
    la   a0, at_str_ok
    li   a1, AT_STR_OK_LEN
    call at_emit_buf

at_parse_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 32
    ret

# ==============================================================================
# Keyword tables (rodata, for at_match_kw comparisons)
# ==============================================================================

.section .rodata
.align 1
at_kw_reset:     .ascii "RESET"
at_kw_mode:      .ascii "MODE"
at_kw_band:      .ascii "BAND"
at_kw_parameter: .ascii "PARAMETER"
at_kw_pkt:       .ascii "PKT"
at_kw_probe:     .ascii "PROBE?"
at_kw_crfop:     .ascii "CRFOP"
at_kw_address:   .ascii "ADDRESS"
at_kw_networkid: .ascii "NETWORKID"
at_kw_syncword:  .ascii "SYNCWORD"
at_kw_send:      .ascii "SEND"
at_kw_ver:       .ascii "VER?"
at_kw_uid:       .ascii "UID?"
at_kw_factory:   .ascii "FACTORY"
at_kw_ipr:       .ascii "IPR"
at_kw_iqi:       .ascii "IQI"
at_kw_reg:       .ascii "REG"
at_kw_regeq:     .ascii "+REG="

# ==============================================================================
# Utility Functions
# ==============================================================================

.section .text
.align 2

# ------------------------------------------------------------------------------
# at_match_kw - Compare buffer against keyword
# a0 = buf ptr, a1 = keyword ptr, a2 = keyword length
# Returns a0 = 1 if match, 0 if mismatch
# Clobbers: t0, t1, t2
# ------------------------------------------------------------------------------
at_match_kw:
    mv   t2, a2
at_match_kw_loop:
    beqz t2, at_match_kw_ok
    lbu  t0, 0(a0)
    lbu  t1, 0(a1)
    bne  t0, t1, at_match_kw_fail
    addi a0, a0, 1
    addi a1, a1, 1
    addi t2, t2, -1
    j    at_match_kw_loop
at_match_kw_ok:
    li   a0, 1
    ret
at_match_kw_fail:
    li   a0, 0
    ret

# ------------------------------------------------------------------------------
# Strict parsing primitives
# ------------------------------------------------------------------------------
# These replace the previous "advance the pointer and hope" style: every field
# boundary is now checked explicitly (digit / separator / end), so a malformed
# command can never be silently accepted.  The line buffer is NUL-terminated by
# at_rx_newline, so all of these stop safely at end of line.
# Pointers are real RAM addresses (never 0), so a0=0 is an unambiguous failure.

# at_expect_char - require a0[0] == a1, consume it
#   a0 = ptr, a1 = expected char
#   returns a0 = ptr+1 on match, a0 = 0 on mismatch.  Clobbers t0.
at_expect_char:
    lbu  t0, 0(a0)
    bne  t0, a1, at_expect_char_fail
    addi a0, a0, 1
    ret
at_expect_char_fail:
    li   a0, 0
    ret

# at_expect_end - require a0[0] == '\0' (end of line)
#   a0 = ptr
#   returns a0 = 1 if end-of-line, a0 = 0 otherwise.  Clobbers t0.
at_expect_end:
    lbu  t0, 0(a0)
    bnez t0, at_expect_end_no
    li   a0, 1
    ret
at_expect_end_no:
    li   a0, 0
    ret

# at_parse_uint_strict - parse a run of decimal digits
#   a0 = ptr
#   returns a0 = value, a1 = ptr just past the last digit, a2 = digit count
#   (a2 == 0 means "no digit here" — caller must reject empty fields).
#   Stops at the first non-digit (separator or NUL).  Clobbers t0, t1, t2.
at_parse_uint_strict:
    li   t0, 0                  # accumulator
    li   a2, 0                  # digit count
at_pus_loop:
    lbu  t1, 0(a0)
    li   t2, '0'
    bltu t1, t2, at_pus_done
    li   t2, '9'
    bgtu t1, t2, at_pus_done
    slli t2, t0, 3              # t0*8
    add  t2, t2, t0             # *9
    add  t2, t2, t0             # *10
    addi t1, t1, -48
    add  t0, t2, t1
    addi a0, a0, 1
    addi a2, a2, 1
    j    at_pus_loop
at_pus_done:
    mv   a1, a0                 # ptr past digits
    mv   a0, t0                 # value
    ret

# at_parse_hex_exact - decode EXACTLY a2 bytes of hex (2 chars each)
#   a0 = hex string ptr, a1 = output buffer ptr, a2 = exact byte count
#   returns a0 = ptr just past the consumed hex on success,
#           a0 = 0 on failure (any non-hex char or fewer bytes than requested).
#   Saves s0, s1; uses 12(sp) as the remaining-byte counter. Clobbers t0-t2,a*.
at_parse_hex_exact:
    addi sp, sp, -16
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   a2, 12(sp)             # remaining byte count
    mv   s0, a0                 # hex input ptr
    mv   s1, a1                 # output buffer ptr
at_phe_loop:
    lw   t2, 12(sp)
    beqz t2, at_phe_ok
    lbu  t0, 0(s0)
    call at_hex_nibble          # a0 = high nibble or -1 (clobbers t0,t1)
    li   t1, -1
    beq  a0, t1, at_phe_fail
    slli a2, a0, 4              # a2 = high nibble << 4 (a2 survives next call)
    addi s0, s0, 1
    lbu  t0, 0(s0)
    call at_hex_nibble          # a0 = low nibble or -1
    li   t1, -1
    beq  a0, t1, at_phe_fail
    or   a2, a2, a0            # full byte
    sb   a2, 0(s1)
    addi s0, s0, 1
    addi s1, s1, 1
    lw   t2, 12(sp)
    addi t2, t2, -1
    sw   t2, 12(sp)
    j    at_phe_loop
at_phe_ok:
    mv   a0, s0                 # ptr just past consumed hex
    j    at_phe_ret
at_phe_fail:
    li   a0, 0
at_phe_ret:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 16
    ret

# ------------------------------------------------------------------------------
# at_emit_buf - Emit raw buffer to UART
# a0 = ptr, a1 = length
# Clobbers: t0, t1, t2, a0, a1 + console_putc clobbers
# ------------------------------------------------------------------------------
at_emit_buf:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0
    mv   s1, a1
at_emit_buf_loop:
    beqz s1, at_emit_buf_done
    lbu  a0, 0(s0)
    call console_putc
    addi s0, s0, 1
    addi s1, s1, -1
    j    at_emit_buf_loop
at_emit_buf_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# ------------------------------------------------------------------------------
# at_emit_uint - Emit unsigned decimal integer to UART
# a0 = value (uint32)
# Saves/restores: ra, s0 (digit count), s1 (buffer ptr)
# Clobbers: t0-t2, a0-a3
# ------------------------------------------------------------------------------
at_emit_uint:
    # Frame: 0=ra 4=s0 8=s1 | 12-byte digit buffer on top
    addi sp, sp, -8
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    # Allocate 12-byte digit buffer and save s1 inside it at the top
    addi sp, sp, -16
    sw   s1, 12(sp)
    mv   s1, sp                 # s1 = digit buffer ptr (stable across calls)
    li   s0, 0                  # s0 = digit count (stable across calls)
    mv   a1, a0
    # Special case: 0
    bnez a1, at_emit_uint_loop
    li   t0, '0'
    sb   t0, 0(s1)
    li   s0, 1
    j    at_emit_uint_print
at_emit_uint_loop:
    beqz a1, at_emit_uint_print
    # Divide a1 by 10: quotient in a0, remainder in a1
    call at_udiv10              # a0 = quotient, a1 = remainder
    addi t0, a1, '0'           # digit char
    add  a2, s1, s0
    sb   t0, 0(a2)
    addi s0, s0, 1
    mv   a1, a0
    j    at_emit_uint_loop
at_emit_uint_print:
    # Print digits in reverse order
    beqz s0, at_emit_uint_done
    addi s0, s0, -1
    add  a2, s1, s0
    lbu  a0, 0(a2)
    call console_putc
    j    at_emit_uint_print
at_emit_uint_done:
    lw   s1, 12(sp)
    addi sp, sp, 16
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 8
    ret

# at_udiv10: divide a1 by 10, return quotient in a0, remainder in a1
# Binary long division, 32-bit. Clobbers: t0, t1, t2, a2, a3.
at_udiv10:
    mv   a2, a1         # working dividend
    li   a0, 0          # quotient
    li   a3, 0          # remainder
    li   t2, 32
at_udiv10_loop:
    slli a3, a3, 1
    srli t0, a2, 31
    or   a3, a3, t0
    slli a2, a2, 1
    slli a0, a0, 1
    li   t1, 10
    bltu a3, t1, at_udiv10_skip
    sub  a3, a3, t1
    ori  a0, a0, 1
at_udiv10_skip:
    addi t2, t2, -1
    bnez t2, at_udiv10_loop
    mv   a1, a3
    ret

# ------------------------------------------------------------------------------
# at_emit_int - Emit signed decimal integer to UART
# a0 = value (int32)
# Clobbers: t0, a0 + at_emit_uint clobbers
# ------------------------------------------------------------------------------
at_emit_int:
    addi sp, sp, -4
    sw   ra, 0(sp)
    # Check sign
    bgez a0, at_emit_int_pos
    li   t0, '-'
    addi sp, sp, -4
    sw   a0, 0(sp)
    mv   a0, t0
    call console_putc
    lw   a0, 0(sp)
    addi sp, sp, 4
    neg  a0, a0                 # negate
at_emit_int_pos:
    call at_emit_uint
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret

# ------------------------------------------------------------------------------
# at_emit_hex - Emit binary buffer as uppercase hex to UART
# a0 = ptr, a1 = length (bytes)
# Clobbers: t0-t2, a0-a3
# ------------------------------------------------------------------------------
at_emit_hex:
    addi sp, sp, -12
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    mv   s0, a0
    mv   s1, a1
at_emit_hex_loop:
    beqz s1, at_emit_hex_done
    lbu  t0, 0(s0)
    # High nibble
    srli a0, t0, 4
    la   t1, at_hex_chars
    add  t1, t1, a0
    lbu  a0, 0(t1)
    call console_putc
    # Low nibble
    lbu  t0, 0(s0)
    andi a0, t0, 0x0F
    la   t1, at_hex_chars
    add  t1, t1, a0
    lbu  a0, 0(t1)
    call console_putc
    addi s0, s0, 1
    addi s1, s1, -1
    j    at_emit_hex_loop
at_emit_hex_done:
    lw   s1, 8(sp)
    lw   s0, 4(sp)
    lw   ra, 0(sp)
    addi sp, sp, 12
    ret

# at_hex_nibble: convert single hex char in t0 to value in a0 (-1 if not hex)
at_hex_nibble:
    mv   a0, t0
    li   t1, '0'
    bltu a0, t1, at_hex_nibble_fail
    li   t1, '9'
    bleu a0, t1, at_hex_nibble_dec
    li   t1, 'A'
    bltu a0, t1, at_hex_nibble_fail
    li   t1, 'F'
    bleu a0, t1, at_hex_nibble_upper
    li   t1, 'a'
    bltu a0, t1, at_hex_nibble_fail
    li   t1, 'f'
    bleu a0, t1, at_hex_nibble_lower
at_hex_nibble_fail:
    li   a0, -1
    ret
at_hex_nibble_dec:
    addi a0, a0, -48
    ret
at_hex_nibble_upper:
    addi a0, a0, -55            # 'A'=65, 65-55=10
    ret
at_hex_nibble_lower:
    addi a0, a0, -87            # 'a'=97, 97-87=10
    ret
