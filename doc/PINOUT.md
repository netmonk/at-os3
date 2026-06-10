# at-os3 Pinout

This document describes the CH32V003 pin usage expected by the current
firmware.

## Target

- MCU: CH32V003
- Radio: SX1278-compatible LoRa front end, currently wired through an
  Ebyte E32-style module pinout
- Host interface: USART1 at 115200 8N1

## Radio SPI And Control Pins

All radio SPI and RF-control pins are on GPIOC.

| CH32V003 pin | Direction | Function | Active level | Notes |
|---|---:|---|---|---|
| `PC0` | output | `RESET` | low | SX1278/module reset |
| `PC1` | output | `TX_EN` | high | E32 RF switch TX enable |
| `PC2` | input | `DIO0` | rising edge | SX1278 `RxDone` / `TxDone`, EXTI line 2 |
| `PC3` | output | `RX_EN` | high | E32 RF switch RX enable |
| `PC4` | output | `CS` / `NSS` | low | SPI chip select |
| `PC5` | AF output | `SCK` | n/a | SPI1 clock |
| `PC6` | AF output | `MOSI` | n/a | SPI1 MOSI |
| `PC7` | input | `MISO` | n/a | SPI1 MISO |

## RF Switch States

| Radio state | `TX_EN` (`PC1`) | `RX_EN` (`PC3`) |
|---|---:|---:|
| Boot / idle before radio init | low | low |
| Sleep | low | low |
| Standby | low | low |
| RX continuous | low | high |
| TX | high | low |

`DIO0` is routed to EXTI line 2 and handled through the canonical event path:

```text
EXTI IRQ -> event_enqueue -> event_loop -> lora_fsm
```

## Host UART

USART1 is used as the AT command transport.

| CH32V003 pin | Direction | Function | Notes |
|---|---:|---|---|
| `PD5` | output | USART1 TX | AF push-pull, 115200 8N1 |
| `PD6` | input | USART1 RX | floating input, 115200 8N1 |

## LEDs

| CH32V003 pin | Direction | Function | Active level | Notes |
|---|---:|---|---|---|
| `PD4` | output | heartbeat LED | high | toggled by heartbeat handler |
| `PD2` | output | radio activity LED | low | pulsed on radio activity |

## Power

The firmware does not manage power rails. The board must provide:

| Rail | Use |
|---|---|
| `3.3 V` | CH32V003 and SX1278 logic/radio supply, according to board design |
| `5 V` | Optional external RF modules that require 5 V, such as some LNAs or PA/LNA boards |
| `GND` | Common ground between host UART, MCU, radio, and any RF accessory |

Do not assume a debugger/programmer `5 V` pin is a clean RF supply. Measure it
under load if it powers an external RF accessory.

## Firmware Sources

The authoritative pin definitions are in:

- `drivers/ch32v003/spi.inc`
- `drivers/ch32v003/uart.S`
- `drivers/ch32v003/led.S`
- `drivers/ch32v003/init.S`

