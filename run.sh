#!/bin/bash

# ==============================================================================
# Kernel Build Script - TinyGS / CH32V003
# ==============================================================================
# Builds the firmware image for the CH32V003 target.
#
# Directory structure:
#   core/     - Hardware-agnostic event system, kernel, timer, FSM engine
#   drivers/  - Hardware-specific initialization and ISRs
#   subfsm/   - True FSMs (table-driven, auto-registered via .kernel_init)
#   handlers/ - Event handlers (stateless, auto-registered via .kernel_init)
#   link/     - Linker scripts
#   build/    - Output directory
# ==============================================================================

set -e

BUILD_DIR=build/ch32v003
mkdir -p $BUILD_DIR
rm -f $BUILD_DIR/*.o $BUILD_DIR/*.elf $BUILD_DIR/*.bin

# ------------------------------------------------------------------------------
# Assembler flags for CH32V003 (RV32EC with Zicsr extension)
# ------------------------------------------------------------------------------
AS_FLAGS="-g -mabi=ilp32e -march=rv32ec_zicsr --warn --fatal-warnings"

# ------------------------------------------------------------------------------
# Assemble core modules (hardware-agnostic)
# ------------------------------------------------------------------------------
echo "Assembling core modules..."
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/trap.o    core/trap.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/event.o   core/event.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/kernel.o  core/kernel.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/timer.o   core/timer.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/console.o core/console.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/fsm.o     core/fsm.S

# ------------------------------------------------------------------------------
# Assemble driver modules (CH32V003-specific)
# ------------------------------------------------------------------------------
echo "Assembling driver modules..."
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/bootstrap.o        drivers/ch32v003/bootstrap.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/drv_init.o         drivers/ch32v003/init.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/drv_exti.o         drivers/ch32v003/exti.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/drv_uart.o         drivers/ch32v003/uart.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/vector-ch32v003.o  drivers/ch32v003/vector.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/flash.o            drivers/ch32v003/flash.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/mcu.o              drivers/ch32v003/mcu.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/drv_spi.o          drivers/ch32v003/spi.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/drv_sx1278.o       drivers/ch32v003/sx1278.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/drv_led.o          drivers/ch32v003/led.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/drv_systick.o      drivers/ch32v003/systick.S

# ------------------------------------------------------------------------------
# Assemble FSM modules
# ------------------------------------------------------------------------------
echo "Assembling FSM modules..."
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/lora_fsm.o subfsm/lora_fsm.S
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/at_fsm.o   subfsm/at_fsm.S

# ------------------------------------------------------------------------------
# Assemble handler modules
# ------------------------------------------------------------------------------
echo "Assembling handler modules..."
riscv32-unknown-elf-as $AS_FLAGS -o $BUILD_DIR/heartbeat.o handlers/heartbeat.S

# ------------------------------------------------------------------------------
# Link all objects
# ------------------------------------------------------------------------------
echo "Linking..."
riscv32-unknown-elf-ld -g -T link/ch32v003.ld \
    $BUILD_DIR/vector-ch32v003.o \
    $BUILD_DIR/bootstrap.o \
    $BUILD_DIR/trap.o \
    $BUILD_DIR/event.o \
    $BUILD_DIR/kernel.o \
    $BUILD_DIR/timer.o \
    $BUILD_DIR/console.o \
    $BUILD_DIR/fsm.o \
    $BUILD_DIR/mcu.o \
    $BUILD_DIR/flash.o \
    $BUILD_DIR/drv_init.o \
    $BUILD_DIR/drv_exti.o \
    $BUILD_DIR/drv_uart.o \
    $BUILD_DIR/drv_spi.o \
    $BUILD_DIR/drv_sx1278.o \
    $BUILD_DIR/drv_led.o \
    $BUILD_DIR/drv_systick.o \
    $BUILD_DIR/lora_fsm.o \
    $BUILD_DIR/at_fsm.o \
    $BUILD_DIR/heartbeat.o \
    -o $BUILD_DIR/kernel.elf

# ------------------------------------------------------------------------------
# Generate binary image
# ------------------------------------------------------------------------------
echo "Generating binary..."
riscv32-unknown-elf-objcopy -O binary $BUILD_DIR/kernel.elf $BUILD_DIR/kernel.bin

# ------------------------------------------------------------------------------
# Print build summary
# ------------------------------------------------------------------------------
echo ""
echo "Build complete:"
ls -la $BUILD_DIR/kernel.elf $BUILD_DIR/kernel.bin

echo ""
echo "Section sizes:"
riscv32-unknown-elf-size $BUILD_DIR/kernel.elf

echo ""
echo ".kernel_init entries:"
riscv32-unknown-elf-nm $BUILD_DIR/kernel.elf | grep -E "(kernel_init|_init_ptr)" || echo "(none found)"
