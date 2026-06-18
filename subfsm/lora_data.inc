# ==============================================================================
# subfsm/lora_data.inc - lora_fsm_data field offsets (public interface)
# ==============================================================================
#
# Single source of truth for lora_fsm_data layout.
# Included by both lora_fsm.S (owner) and at_fsm.S (query reader).
#
# Constitutional compliance:
#   Article 3.1 (Distinguishability): eliminates the dual-shadow degeneracy by
#   making at_fsm queries read the radio layer's single authoritative state.
#
# ==============================================================================

.ifndef __LORA_DATA_INC__
.set __LORA_DATA_INC__, 1

# Offsets from lora_fsm_data label
.equ LORA_DATA_TX_COUNT,     0
.equ LORA_DATA_RX_COUNT,     4
.equ LORA_DATA_FREQ_HZ,      8
.equ LORA_DATA_FREQ_REG,     12
.equ LORA_DATA_BW_HZ,        16
.equ LORA_DATA_BW_CODE,      20
.equ LORA_DATA_TXP,          24
.equ LORA_DATA_SF,           28
.equ LORA_DATA_CR,           32
.equ LORA_DATA_RADIO_ON,     36
.equ LORA_DATA_TX_PTR,       40
.equ LORA_DATA_TX_LEN,       44
.equ LORA_DATA_CFG_DIRTY,    48
.equ LORA_DATA_CAL_HZ,       52
.equ LORA_DATA_PREAMBLE,     56
.equ LORA_DATA_IQI,          60
.equ LORA_DATA_SYNC_WORD,    64
.equ LORA_DATA_CRC,          68
.equ LORA_DATA_LDRO,         72
.equ LORA_DATA_IMPLICIT_LEN, 76

.endif # __LORA_DATA_INC__
