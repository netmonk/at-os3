# ==============================================================================
# core/events.inc - Event System Constants and Definitions
# ==============================================================================
#
# PURPOSE:
#   Define the canonical event format, kinds, and dispatch limits.
#   All event producers and consumers MUST use these definitions.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 4 (Canonical Event Pipeline): All events share this common format
#   - Article 11 (ABI Sanctity / Boundary Contracts): This defines the event ABI
#   - PO-1 (Event Origin Proof): All event types enumerated here
#
# ==============================================================================

.ifndef __EVENTS_INC__
.set __EVENTS_INC__, 1

# ==============================================================================
# EVENT FORMAT (3 words = 12 bytes)
# ==============================================================================
#
# Word 0 (w0): Header
#   bits [31:24] - kind   (8 bits)  - Event category (EV_IRQ, EV_TIMER, etc.)
#   bits [23:16] - id     (8 bits)  - Sub-identifier within kind
#   bits [15:0]  - flags  (16 bits) - Event-specific flags
#
# Word 1 (w1): data0 - Primary payload
# Word 2 (w2): data1 - Secondary payload (optional)
#
# HEADER CONSTRUCTION:
#   w0 = (kind << 24) | (id << 16) | flags
#
# ==============================================================================

.equ EVENT_SIZE,        12          # 3 words per event
.equ EVENT_QUEUE_SIZE,  8           # Number of event slots (power of 2)
.equ EVENT_QUEUE_MASK,  (EVENT_QUEUE_SIZE - 1)

# Header field positions
.equ EV_KIND_SHIFT,     24
.equ EV_ID_SHIFT,       16
.equ EV_FLAGS_MASK,     0xFFFF

# Header extraction masks (applied after loading w0)
.equ EV_KIND_MASK,      0xFF000000
.equ EV_ID_MASK,        0x00FF0000

# ==============================================================================
# EVENT KINDS
# ==============================================================================
# These are the top-level event categories. Each kind has its own ID space.
# kind values are indices into root_table first dimension.

.equ EV_KIND_IRQ,       0           # Hardware interrupt events
.equ EV_KIND_TIMER,     1           # Software timer expiry events
.equ EV_KIND_SOFT,      2           # Software-triggered events
.equ EV_KIND_MAX,       3           # Number of kinds (MUST update if adding)

# ==============================================================================
# EVENT IDs PER KIND
# ==============================================================================
# Maximum number of distinct IDs per kind. Defines root_table second dimension.
# Can be different per kind in theory, but we use uniform sizing for simplicity.

.equ EV_ID_MAX,         8           # Max IDs per kind (2^3 for shift optimization)
.equ EV_ID_MAX_LOG2,    3           # log2(EV_ID_MAX) for table indexing

# ==============================================================================
# SPECIFIC EVENT DEFINITIONS
# ==============================================================================

# --- EV_KIND_IRQ (kind=0) ---
# id = mapped IRQ index (NOT raw IRQ number - fits in EV_ID_MAX)
# data0 = timestamp or IRQ-specific info
# data1 = raw IRQ number or reserved
#
# IRQ ID mapping (id -> hardware IRQ):
.equ EV_IRQ_LORA_DIO0,  3           # EXTI line 2 - LoRa DIO0 (PC2)
# Add more as needed, keep id < EV_ID_MAX

# --- EV_KIND_TIMER (kind=1) ---
# id = timer ID (0 to TIMER_MAX-1)
# data0 = scheduled deadline (ticks)
# data1 = reserved
.equ EV_TIMER_ID_MAX,   8           # Max concurrent software timers

# --- EV_KIND_SOFT (kind=2) ---
# id = application-defined
# data0, data1 = application-defined
.equ EV_SOFT_LORA_RX,        4       # LoRa RX re-arm request
.equ EV_SOFT_LORA_CMD,       5       # Generic host-to-radio request
.equ EV_SOFT_RADIO_NOTIFY,   6       # Generic radio-to-host notification
.equ EV_SOFT_RADIO_RX_PUMP,  7       # Drain UART RX ring into host parser

# EV_SOFT_LORA_CMD flags / opcodes
.equ LORA_CMD_SET_FREQ,         1
.equ LORA_CMD_SET_TXP,          3
.equ LORA_CMD_SET_STATE,        6
.equ LORA_CMD_TX_FRAME,         7
.equ LORA_CMD_RESET,           10       # Hardware reset + full reconfig of the radio
.equ LORA_CMD_SET_PARAMS,      11       # Atomic SF+BW+CR+preamble update (single event)
.equ LORA_CMD_SET_IQI,         12       # Set LoRa IQ inversion: 0=normal, 1=inverted
.equ LORA_CMD_SET_SYNC_WORD,   13       # Set SX127x LoRa sync word, 8-bit
.equ LORA_CMD_SET_PKT_PROFILE, 14       # Atomic CRC+LDRO+implicit length profile
.equ LORA_CMD_FACTORY,         15       # Full factory reset: config defaults + hardware reinit

# Packed payload layout for LORA_CMD_SET_PARAMS (data0):
#   bits [3:0]   SF        (6..12)
#   bits [7:4]   BW code   (0..9, AT/SX1278 index)
#   bits [11:8]  CR        (host 5..8 = 4/5..4/8)
#   bits [22:12] preamble  (4..1023 symbols)
# Packing all four fields in one event makes AT+PARAMETER atomic: either the
# whole profile is accepted or none of it, so the queue can never apply a
# partial config (PO-7 Temporal Coherence, PO-8 Degeneracy).
.equ LORA_PARAMS_SF_SHIFT,    0
.equ LORA_PARAMS_BW_SHIFT,    4
.equ LORA_PARAMS_CR_SHIFT,    8
.equ LORA_PARAMS_PRE_SHIFT,   12

# Packed payload layout for LORA_CMD_SET_PKT_PROFILE (data0):
#   bits [0]     CRC PHY enable     (0=off, 1=on)
#   bits [2:1]   LDRO mode          (0=force off, 1=force on, 2=auto)
#   bits [10:3]  implicit length    (0=explicit header, 1..255=implicit)
.equ LORA_PKT_CRC_SHIFT,      0
.equ LORA_PKT_LDRO_SHIFT,     1
.equ LORA_PKT_LEN_SHIFT,      3

# EV_SOFT_RADIO_NOTIFY flags
.equ RADIO_NOTIFY_TX_DONE,      1
.equ RADIO_NOTIFY_TX_TIMEOUT,   2
.equ RADIO_NOTIFY_TX_UNEXPECTED, 3
.equ RADIO_NOTIFY_RX_READY,     4
.equ RADIO_NOTIFY_CRC_ERR,      5
.equ RADIO_NOTIFY_RX_TIMEOUT,   6
.equ RADIO_NOTIFY_READY,        13       # Radio reset complete + RX re-armed
# ==============================================================================
# RESERVED VALUES
# ==============================================================================

.equ EVENT_NONE,        0           # Invalid/empty event (header == 0)

# ==============================================================================
# HELPER MACROS
# ==============================================================================

# Build event header from kind, id, flags
# Usage: EV_HEADER kind_val, id_val, flags_val, dest_reg
.macro EV_HEADER kind_val, id_val, flags_val, dest_reg
    li   \dest_reg, ((\kind_val << EV_KIND_SHIFT) | (\id_val << EV_ID_SHIFT) | \flags_val)
.endm

# Extract kind from header in src_reg, result in dest_reg
.macro EV_GET_KIND src_reg, dest_reg
    srli \dest_reg, \src_reg, EV_KIND_SHIFT
    andi \dest_reg, \dest_reg, 0xFF
.endm

# Extract id from header in src_reg, result in dest_reg
.macro EV_GET_ID src_reg, dest_reg
    srli \dest_reg, \src_reg, EV_ID_SHIFT
    andi \dest_reg, \dest_reg, 0xFF
.endm

.endif # __EVENTS_INC__
