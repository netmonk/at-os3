# ==============================================================================
# drivers/ch32v003/spi.inc - GPIO and LoRa pin definitions
# ==============================================================================
#
# PURPOSE:
#   Shared pin and GPIO constants for SPI1 and the LoRa radio control lines.
#   Included by spi.S and the SX1278 radio driver.
#
# PIN MAPPING (GPIOC):
#   PC0 = RESET  (GPIO output, active low)
#   PC1 = TX_EN  (GPIO output, active high - E32 RF switch TX enable)
#   PC2 = DIO0   (input, EXTI source - TxDone/RxDone on SX1278)
#   PC3 = RX_EN  (GPIO output, active high - E32 RF switch RX enable)
#   PC4 = CS     (GPIO output, active low)
#   PC5 = SCK    (AF push-pull)
#   PC6 = MOSI   (AF push-pull)
#   PC7 = MISO   (floating input)
#
# ==============================================================================

    .ifndef __SPI_INC__
    .set __SPI_INC__, 1

# GPIOC registers
.equ GPIOC_BASE,        0x40011000
.equ GPIOC_CFGLR,       (GPIOC_BASE + 0x00)
.equ GPIOC_OUTDR,       (GPIOC_BASE + 0x0C)
.equ GPIOC_BSHR,        (GPIOC_BASE + 0x10)

# GPIO config (CNF[1:0]:MODE[1:0], 4 bits per pin in CFGLR)
# Output 50MHz push-pull = 0x3, AF push-pull = 0xB, Input pull = 0x8, Float = 0x4
.equ SPI_GPIO_MASK,     0xFFFFFFFF
.equ SPI_GPIO_CFG,      0x4BB33833      # PC7=float, PC6/5=AF, PC4/3/1/0=output, PC2=pull

# CS pin (PC4)
.equ SPI_CS_PIN,        4
.equ SPI_CS_HIGH,       (1 << SPI_CS_PIN)
.equ SPI_CS_LOW,        (1 << (SPI_CS_PIN + 16))

# LoRa control pins (E32-400M30S / SX1278)
.equ LORA_RESET_PIN,    0
.equ LORA_TXEN_PIN,     1               # E32 RF switch TX enable
.equ LORA_DIO0_PIN,     2               # DIO0: TxDone / RxDone
.equ LORA_RXEN_PIN,     3               # E32 RF switch RX enable

.equ LORA_RESET_HIGH,   (1 << LORA_RESET_PIN)
.equ LORA_RESET_LOW,    (1 << (LORA_RESET_PIN + 16))
.equ LORA_TXEN_HIGH,    (1 << LORA_TXEN_PIN)
.equ LORA_TXEN_LOW,     (1 << (LORA_TXEN_PIN + 16))
.equ LORA_RXEN_HIGH,    (1 << LORA_RXEN_PIN)
.equ LORA_RXEN_LOW,     (1 << (LORA_RXEN_PIN + 16))
.equ LORA_DIO0_MASK,    (1 << LORA_DIO0_PIN)

    .endif  # __SPI_INC__
