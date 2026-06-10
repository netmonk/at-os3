/* ==============================================================================
 * link/ch32v003.ld - CH32V003 Linker Script
 * ==============================================================================
 *
 * MEMORY:
 *   Flash: 16KB at 0x00000000
 *   RAM:   2KB at 0x20000000
 *
 * SECTIONS:
 *   .vectors      - Interrupt vector table (flash, at 0x0)
 *   .text         - Code and read-only data (flash)
 *   .kernel_init  - Auto-registration function pointers (flash)
 *   .bss          - Zero-initialized data (RAM)
 *
 * CONSTITUTIONAL COMPLIANCE:
 *   - Article 10 (Layer Roles): Sections enforce layer boundaries
 *   - PO-5 (Layer Integrity Proof): Each section has defined role
 *
 * ============================================================================== */

ENTRY(vector_base)

MEMORY
{
    flash (rx)  : ORIGIN = 0x00000000, LENGTH = 16K
    ram   (xrw) : ORIGIN = 0x20000000, LENGTH = 2K
}

SECTIONS
{
    /* =========================================================================
     * FLASH: Vector Table
     * Must be at address 0x00000000
     * ========================================================================= */
    . = 0x00000000;
    .vectors :
    {
        KEEP(*(.vectors))
    } > flash

    /* =========================================================================
     * FLASH: Code and Read-Only Data
     * ========================================================================= */
    .text :
    {
        *(.text*)
        *(.rodata*)
    } > flash

    /* =========================================================================
     * FLASH: Kernel Init Section (Sub-FSM Auto-Registration)
     * =========================================================================
     * Each sub-FSM places a pointer to its init function here.
     * kernel_init iterates this section calling each function.
     *
     * Usage in sub-FSM:
     *   .section .kernel_init, "a"
     *   .word my_init_function
     * ========================================================================= */
    .kernel_init :
    {
        . = ALIGN(4);
        __kernel_init_start = .;
        KEEP(*(.kernel_init))
        KEEP(*(.kernel_init.*))
        __kernel_init_end = .;
    } > flash

    /* =========================================================================
     * Discard RISC-V attributes (not needed for bare metal)
     * ========================================================================= */
    /DISCARD/ : { *(.riscv.attributes) }

    /* =========================================================================
     * RAM: BSS (Zero-Initialized Data)
     * ========================================================================= */
    . = ALIGN(4);
    .bss (NOLOAD) :
    {
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > ram

    /* =========================================================================
     * RAM: Stack
     * Stack grows downward from top of RAM
     * ========================================================================= */
    _ram_start = ORIGIN(ram);
    _ram_end   = ORIGIN(ram) + LENGTH(ram);
    _stack_top = _ram_end;
}
