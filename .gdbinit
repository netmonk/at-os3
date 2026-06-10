set architecture riscv:rv32
set remotetimeout 10

target remote :3333

file build/ch32v003/kernel.elf

monitor reset halt
