# ==============================================================================
# drivers/ch32v003/flash.S - Flash Access Latency Configuration
# ==============================================================================
#
# PURPOSE:
#   Configure flash access wait states for CH32V003 high-speed operation.
#   Required before switching to 48MHz system clock.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 10 (Layer Roles): Hardware config belongs in drivers/
#   - PO-4 (ISR Safety): Not called from ISR context
#
# ==============================================================================

.equ R32_FLASH_ACTLR, 0x40022000
.equ FLASH_LATENCY_1, 0x1           # 01b = 1 wait state for 48MHz

# ==============================================================================
# flash_latency_48mhz - Set Flash Wait State for 48MHz Operation
# ==============================================================================
#
# PURPOSE:
#   Configure 1 wait state for flash access at 48MHz.
#   Must be called before clock_init_48mhz.
#
# INPUTS:  None
# OUTPUTS: None
#
# REGISTERS: t0, t1, t2 (caller-saved)
#
# ==============================================================================

.section .text
.align 2

.globl flash_latency_48mhz
flash_latency_48mhz:
    li   t0, R32_FLASH_ACTLR        # t0 = FLASH_ACTLR address
    lw   t1, 0(t0)                  # t1 = current value
    li   t2, -4                     # t2 = ~0x3 = 0xFFFFFFFC: clears the 2-bit LATENCY field [1:0]
    and  t1, t1, t2                 # Clear latency field
    li   t2, FLASH_LATENCY_1        # t2 = 1 wait state
    or   t1, t1, t2                 # Set new latency

    sw   t1, 0(t0)                  # Write to register
    fence iorw, iorw                # Full memory/IO fence: ensures MMIO write is visible
                                    # to the CPU before any subsequent instruction executes
    ret
