# at-os3 LoRa Modem AT Command Reference

This document describes the AT command interface implemented by the CH32V003
firmware in `subfsm/at_fsm.S`.

The interface is a raw SX1278 LoRa modem AT command set. It uses
hexadecimal payload encoding for binary-safe TX/RX data and exposes
SX1278-oriented radio controls.

## Transport

- UART: firmware default is 115200 baud, 8 data bits, no parity, 1 stop bit.
- Command line termination: `\n` executes a command. `\r` is ignored, so `\r\n`
  is accepted. A bare `\r` does not execute the line.
- Command syntax is uppercase and strict.
- Unknown or malformed commands return `+ERR=4`.
- Empty lines are ignored.
- The command line buffer is 127 characters plus NUL termination. Extra input
  bytes before the newline are discarded.
- On startup, before the first valid `AT` prefix has been seen, the parser may
  resynchronize to the first `AT` inside an input line. After sync, parsing is
  strict from the start of each line.

## General Responses

Successful commands usually return:

```text
+OK
```

On the wire this is:

```text
+OK\r\n
```

This document writes responses without escaping CR/LF unless the exact byte
sequence matters.

Error responses:

| Response | Meaning |
|---|---|
| `+ERR=1` | LoRa PHY CRC error reported by the radio |
| `+ERR=4` | Malformed command, unsupported command, out-of-range value, or internal command queue full |

Unsolicited responses:

| Response | Meaning |
|---|---|
| `+READY` | Modem firmware is ready; also emitted after a completed radio reset |
| `+SENDED` | A queued TX frame completed |
| `+RCV=...` | A LoRa packet was received |
| `+ERR=1` | A received LoRa packet failed PHY CRC |

## Defaults

Power-on and `AT+FACTORY` shadow defaults:

| Field | Default |
|---|---:|
| Frequency | `433175000` Hz |
| Spreading factor | `12` |
| Bandwidth code | `7` (`125 kHz`) |
| Coding rate code | `1` (`4/5`) |
| Preamble | `8` symbols |
| TX power | `15` dBm |
| IQ inversion | `0` normal |
| PHY CRC | `1` enabled |
| LDRO | `2` auto |
| Packet length mode | `0` explicit header |
| Address | `0` |
| Network ID | `0` |
| Mode | `0` RX active |

## Bandwidth Codes

`AT+PARAMETER` uses the AT bandwidth code, not a kHz value.

| Code | SX1278 bandwidth |
|---:|---:|
| `0` | `7.8 kHz` |
| `1` | `10.4 kHz` |
| `2` | `15.6 kHz` |
| `3` | `20.8 kHz` |
| `4` | `31.25 kHz` |
| `5` | `41.7 kHz` |
| `6` | `62.5 kHz` |
| `7` | `125 kHz` |
| `8` | `250 kHz` |
| `9` | `500 kHz` |

## Coding Rate Codes

`AT+PARAMETER` uses AT coding rate codes.

| Code | LoRa coding rate |
|---:|---:|
| `1` | `4/5` |
| `2` | `4/6` |
| `3` | `4/7` |
| `4` | `4/8` |

Internally, the radio FSM converts these to SX1278 host values `5..8`.

## Commands

### `AT`

Ping the parser.

```text
AT
```

Response:

```text
+OK
```

### `AT+RESET`

Request a real radio reset through the LoRa FSM. The command immediately emits
`+RESET`; `+READY` is emitted later, only after the radio reset and RX re-arm
complete.

```text
AT+RESET
```

Response:

```text
+RESET
```

Later unsolicited response:

```text
+READY
```

Malformed suffixes such as `AT+RESET1` are rejected.

### `AT+MODE=<mode>`

Set radio operating state.

```text
AT+MODE=0
AT+MODE=1
```

Values:

| Value | Meaning |
|---:|---|
| `0` | RX active |
| `1` | Standby / radio off |

Response:

```text
+OK
```

Out-of-range values return `+ERR=4`.

### `AT+MODE?`

Read shadow mode.

```text
AT+MODE?
```

Response:

```text
+MODE=<mode>
+OK
```

### `AT+BAND=<frequency_hz>`

Set carrier frequency in Hz.

```text
AT+BAND=436995000
```

Accepted range:

```text
137000000 .. 1020000000
```

Response:

```text
+OK
```

Notes:

- This is the SX1278 synthesizer range accepted by firmware.
- The attached RF module, matching network, antenna, or front-end may have a
  much narrower usable range.
- The radio FSM recomputes the SX1278 `Frf` register from this value.

### `AT+BAND?`

Read shadow carrier frequency.

```text
AT+BAND?
```

Response:

```text
+BAND=<frequency_hz>
+OK
```

### `AT+PARAMETER=<sf>,<bw>,<cr>,<preamble>`

Atomically set the LoRa modulation profile.

```text
AT+PARAMETER=8,6,3,8
```

Fields:

| Field | Range | Meaning |
|---|---:|---|
| `sf` | `6..12` | LoRa spreading factor |
| `bw` | `0..9` | Bandwidth code, see "Bandwidth Codes" |
| `cr` | `1..4` | Coding rate code, see "Coding Rate Codes" |
| `preamble` | `4..1023` | Preamble length in symbols |

Response:

```text
+OK
```

The four fields are packed into one event and applied atomically by the radio
FSM. If any field is invalid, nothing is committed and the response is
`+ERR=4`.

### `AT+PARAMETER?`

Read the shadow LoRa modulation profile.

```text
AT+PARAMETER?
```

Response:

```text
+PARAMETER=<sf>,<bw>,<cr>,<preamble>
+OK
```

### `AT+PKT=<crc>,<ldro>,<len>`

Set packet profile fields not covered by `AT+PARAMETER`.

```text
AT+PKT=1,2,0
```

Fields:

| Field | Range | Meaning |
|---|---:|---|
| `crc` | `0..1` | LoRa PHY CRC disable/enable |
| `ldro` | `0..2` | Low Data Rate Optimize mode |
| `len` | `0..255` | Header mode / implicit payload length |

`ldro` values:

| Value | Meaning |
|---:|---|
| `0` | Force LDRO off |
| `1` | Force LDRO on |
| `2` | Auto LDRO according to symbol time |

`len` values:

| Value | Meaning |
|---:|---|
| `0` | Explicit header mode |
| `1..255` | Implicit header mode with fixed payload length |

Response:

```text
+OK
```

Notes:

- This is a at-os3 extension.
- `AT+PKT` is required for TinyGS profiles that explicitly control CRC, LDRO,
  or implicit header length.

### `AT+PKT?`

Read the packet profile shadow state.

```text
AT+PKT?
```

Response:

```text
+PKT=<crc>,<ldro>,<len>
+OK
```

### `AT+CRFOP=<power_dbm>`

Set SX1278 PA_BOOST TX power.

```text
AT+CRFOP=5
```

Accepted range:

```text
0 .. 15
```

Response:

```text
+OK
```

Notes:

- The value is treated as dBm by the firmware's SX1278 power mapping.
- This command does not imply that the attached board is legally allowed to
  transmit at that power in every band or region.

### `AT+CRFOP?`

Read shadow TX power.

```text
AT+CRFOP?
```

Response:

```text
+CRFOP=<power_dbm>
+OK
```

### `AT+IQI=<mode>`

Set LoRa IQ inversion.

```text
AT+IQI=0
AT+IQI=1
```

Values:

| Value | Meaning |
|---:|---|
| `0` | Normal IQ |
| `1` | Inverted IQ |

Response:

```text
+OK
```

### `AT+IQI?`

Read shadow IQ inversion state.

```text
AT+IQI?
```

Response:

```text
+IQI=<mode>
+OK
```

### `AT+SYNCWORD=<syncword>`

Set SX1278 LoRa sync word.

```text
AT+SYNCWORD=18
```

Accepted range:

```text
0 .. 255
```

Response:

```text
+OK
```

Notes:

- Decimal input is used. `18` sets register value `0x12`.
- There is currently no `AT+SYNCWORD?` query command.

### `AT+ADDRESS=<address>`

Set the local shadow address used in `+RCV` reports.

```text
AT+ADDRESS=0
```

Accepted range:

```text
0 .. 65535
```

Response:

```text
+OK
```

Notes:

- This firmware uses raw LoRa packets.
- The address is not encoded into transmitted packets and is not used for RX
  filtering.
- The address is only printed as the first field of `+RCV`.

### `AT+ADDRESS?`

Read shadow address.

```text
AT+ADDRESS?
```

Response:

```text
+ADDRESS=<address>
+OK
```

### `AT+NETWORKID=<network_id>`

Set shadow network ID.

```text
AT+NETWORKID=0
```

Accepted range:

```text
0 .. 16
```

Response:

```text
+OK
```

Notes:

- This value is stored for host-side compatibility.
- It is not used by the raw LoRa modem path.
- LoRa network identity is normally represented by the SX1278 sync word, set
  with `AT+SYNCWORD`.

### `AT+NETWORKID?`

Read shadow network ID.

```text
AT+NETWORKID?
```

Response:

```text
+NETWORKID=<network_id>
+OK
```

### `AT+SEND=<address>,<length>,<hexdata>`

Transmit one raw LoRa packet.

```text
AT+SEND=0,4,70696E67
```

This sends four bytes:

```text
70 69 6E 67
```

which is ASCII `ping`.

Fields:

| Field | Range | Meaning |
|---|---:|---|
| `address` | Decimal, non-empty | Parsed for compatibility; ignored by raw TX |
| `length` | `1..128` | Decoded payload length in bytes |
| `hexdata` | exactly `length * 2` hex chars | Binary payload |

Response when the frame is accepted into the event queue:

```text
+OK
```

Later unsolicited response when TX completes:

```text
+SENDED
```

Important constraints:

- Payload data is hexadecimal, not raw ASCII.
- Both uppercase and lowercase hex digits are accepted.
- The number of hex characters must exactly match `length * 2`.
- Extra trailing characters are rejected.
- `length=0` is rejected.
- `length>128` is rejected.
- Each accepted `AT+SEND` maps to exactly one on-air LoRa packet.

### `AT+REG=<hh>`

Read one SX1278 register for diagnostics.

```text
AT+REG=42
```

Response:

```text
+REG=<vv>
+OK
```

Example for `RegVersion`:

```text
+REG=12
+OK
```

Fields:

| Field | Meaning |
|---|---|
| `hh` | Register address, exactly two hex digits |
| `vv` | Register value, exactly two uppercase hex digits |

Notes:

- This is a at-os3 diagnostic extension.
- It is read-only.
- It is the only AT command that directly reads the radio driver instead of
  going through the event pipeline.
- Strict format is required: exactly two hex digits and then end of line.

### `AT+VER?`

Read firmware version string.

```text
AT+VER?
```

Response:

```text
+VER=at-os3-0.1.0
+OK
```

### `AT+UID?`

Read fixed UID string.

```text
AT+UID?
```

Response:

```text
+UID=000000000000
+OK
```

### `AT+IPR`

Accepted for host compatibility.

```text
AT+IPR
```

Response:

```text
+OK
```

Notes:

- This firmware does not change UART baud rate.
- The current parser accepts any command name beginning with `IPR`, for example
  `AT+IPR`, `AT+IPR=115200`, or `AT+IPR?`.
- No baud-rate argument is validated or applied. Use this only as a
  compatibility no-op.

### `AT+FACTORY`

Reset shadow AT configuration to defaults.

```text
AT+FACTORY
```

Response:

```text
+FACTORY
```

No extra `+OK` is emitted.

Notes:

- This resets the AT shadow configuration values.
- It does not currently enqueue a radio reconfiguration event by itself.
- To force hardware back to the default radio profile, follow with `AT+RESET`
  or explicitly send the desired `AT+BAND`, `AT+PARAMETER`, `AT+PKT`,
  `AT+SYNCWORD`, `AT+IQI`, and `AT+CRFOP` commands.

## Receive Notification

When a LoRa packet is received and accepted by the PHY, the firmware emits:

```text
+RCV=<addr>,<len>,<hexdata>,<rssi_dbm>,<snr_db>,<freq_err_hz>
```

Fields:

| Field | Meaning |
|---|---|
| `addr` | Current shadow address from `AT+ADDRESS`; not the sender address |
| `len` | Decoded payload length in bytes |
| `hexdata` | Uppercase hex payload, `len * 2` characters |
| `rssi_dbm` | Packet RSSI in dBm, computed as `RegPktRssiValue - 164` for LF band |
| `snr_db` | Packet SNR in dB, computed from signed `RegPktSnrValue / 4` and truncated by arithmetic shift |
| `freq_err_hz` | Signed SX1278 LoRa FEI converted to Hz for the active bandwidth |

Example:

```text
+RCV=0,4,70696E67,-57,7,1234
```

The example payload is ASCII `ping`.

If the radio reports a PHY CRC error, no `+RCV` is emitted; the AT layer emits:

```text
+ERR=1
```

## TX Completion Notification

After an accepted `AT+SEND`, TX completion emits:

```text
+SENDED
```

TX timeout and unexpected TX IRQ conditions are currently silent at the AT
layer. The radio FSM returns to RX internally.

## Strict Parsing Rules

Except for the compatibility no-op `AT+IPR`, the parser rejects ambiguous
input. These examples return `+ERR=4`:

```text
AT+MODE=0x
AT+BAND=436995000junk
AT+PARAMETER=8,6,3
AT+PARAMETER=8,6,3,8,1
AT+PKT=1,2,0x
AT+SEND=0,4,70696E
AT+SEND=0,4,70696E6700
AT+REG=4200
AT+VER?x
```

Numeric fields are decimal unless explicitly documented as hex. The only
command input fields that use hexadecimal are:

- `AT+SEND` payload
- `AT+REG` register address

## Typical LoRa RX Setup

```text
AT
AT+BAND=436995000
AT+PARAMETER=8,6,3,8
AT+PKT=1,2,0
AT+SYNCWORD=18
AT+IQI=0
AT+CRFOP=5
AT+MODE=0
```

Expected command responses are `+OK` for every command above.

## Typical TX Test

On receiver:

```text
AT+BAND=436995000
AT+PARAMETER=8,6,3,8
AT+PKT=1,2,0
AT+SYNCWORD=18
AT+IQI=0
AT+MODE=0
```

On transmitter:

```text
AT+BAND=436995000
AT+PARAMETER=8,6,3,8
AT+PKT=1,2,0
AT+SYNCWORD=18
AT+IQI=0
AT+CRFOP=5
AT+SEND=0,4,70696E67
```

Expected transmitter output:

```text
+OK
+SENDED
```

Expected receiver output:

```text
+RCV=0,4,70696E67,<rssi_dbm>,<snr_db>,<freq_err_hz>
```

## Compatibility Notes

- This is a raw LoRa modem interface, not a network stack.
- `ADDRESS` and `NETWORKID` are compatibility shadows; they do not alter raw
  LoRa packet contents.
- Binary payloads use uppercase hex encoding.
- Some changes are queued through the event pipeline. `+OK` means the command
  was accepted into the firmware path, not that a packet was already sent.
- Commands that change SX1278 radio parameters are applied by the radio FSM,
  which handles the required sleep/configure/RX sequencing.
