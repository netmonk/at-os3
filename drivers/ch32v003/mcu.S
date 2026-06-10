# ==============================================================================
# drivers/ch32v003/mcu.S - CH32V003 Clock Initialization
# ==============================================================================
#
# PURPOSE:
#   Configure system clock to 48MHz using HSE and PLL.
#   Called during early startup after flash latency is set.
#
# CONSTITUTIONAL COMPLIANCE:
#   - Article 10 (Layer Roles): Hardware config belongs in drivers/
#   - PO-4 (ISR Safety): Not called from ISR context
#
# ==============================================================================

.include "drivers/ch32v003/mmio.inc"

# ------------------------------------------------------------------------------
# RCC Register Addresses
# ------------------------------------------------------------------------------

.equ R32_RCC_CTLR,   0x40021000
.equ R32_RCC_CFGR0,  0x40021004

# ------------------------------------------------------------------------------
# RCC_CTLR Bit Definitions
# ------------------------------------------------------------------------------

.equ RCC_HSEON,      (1 << 16)      # HSE oscillator enable
.equ RCC_HSERDY,     (1 << 17)      # HSE oscillator ready
.equ RCC_HSEBYP,     (1 << 18)      # HSE bypass for active external clock input
.equ RCC_PLLON,      (1 << 24)      # PLL enable
.equ RCC_PLLRDY,     (1 << 25)      # PLL ready
.equ CLOCK_STARTUP_TIMEOUT, 0x40000 # Bounded wait guard for HSE/PLL/SWS startup

# ------------------------------------------------------------------------------
# RCC_CFGR0 Field Definitions
# ------------------------------------------------------------------------------

.equ CFGR_SW_MASK,      (3 << 0)    # SW[1:0] system clock switch
.equ CFGR_SW_PLL,       (2 << 0)    # SW = 10b (PLL selected)
.equ CFGR_SWS_MASK,     (3 << 2)    # SWS[1:0] system clock status
.equ CFGR_SWS_PLL,      (2 << 2)    # SWS = 10b (PLL is source)
.equ CFGR_HPRE_MASK,    (0xF << 4)  # HPRE[3:0] AHB prescaler
.equ CFGR_ADCPRE_MASK,  (0x1F << 11) # ADCPRE[4:0] ADC prescaler
.equ CFGR_PLLSRC_MASK,      (1 << 16) # PLLSRC[16] PLL entry clock source
.equ CFGR_PLLSRC_HSE_MUL2,  (1 << 16) # Select HSE as PLL source (PLL fixed x2: 24 MHz HSE -> 48 MHz)

# ==============================================================================
# clock_init_48mhz - Initialize System Clock to 48MHz
# ==============================================================================
#
# PURPOSE:
#   Configure PLL with HSE * 2 as source to achieve 48MHz system clock.
#
# SEQUENCE:
#   1. Enable HSE bypass for the active external clock, wait for ready
#   2. Configure CFGR0: clear prescalers, select HSE * 2 as PLL source
#   3. Enable PLL, wait for ready
#   4. Switch system clock to PLL
#   5. Wait for SWS to confirm PLL is active system clock source
#
# INPUTS:  None
# OUTPUTS: None
#
# REGISTERS: t0, t1, t2, a3, a4 (caller-saved)
#
# ==============================================================================

.section .text
.align 2

.globl clock_init_48mhz
clock_init_48mhz:
    # Step 1: Enable HSE crystal oscillator and wait for ready
    # HSEBYP must be 0 for passive crystal (official devkit uses 24 MHz crystal)
    # HSEBYP=1 is only for active external clock input — do not set it here
    MMIO_CLR_BITS     R32_RCC_CTLR, (RCC_HSEBYP | RCC_HSEON)
    MMIO_SET_BITS     R32_RCC_CTLR, RCC_HSEON
    li   t2, CLOCK_STARTUP_TIMEOUT
wait_hse:
    li   t0, R32_RCC_CTLR
    lw   t1, 0(t0)
    li   a3, RCC_HSERDY
    and  t1, t1, a3
    bnez t1, wait_hse_done
    addi t2, t2, -1
    bnez t2, wait_hse
    j    clock_init_fail
wait_hse_done:

    # Step 2: Configure CFGR0 before enabling PLL
    MMIO_CLR_BITS    R32_RCC_CFGR0, (CFGR_SW_MASK | CFGR_HPRE_MASK | CFGR_ADCPRE_MASK)
    MMIO_WRITE_FIELD R32_RCC_CFGR0, CFGR_PLLSRC_MASK, CFGR_PLLSRC_HSE_MUL2

    # Step 3: Enable PLL and wait for ready
    MMIO_SET_BITS     R32_RCC_CTLR, RCC_PLLON
    li   t2, CLOCK_STARTUP_TIMEOUT
wait_pll:
    li   t0, R32_RCC_CTLR
    lw   t1, 0(t0)
    li   a3, RCC_PLLRDY
    and  t1, t1, a3
    bnez t1, wait_pll_done
    addi t2, t2, -1
    bnez t2, wait_pll
    j    clock_init_fail
wait_pll_done:

    # Step 4: Switch system clock to PLL
    MMIO_WRITE_FIELD R32_RCC_CFGR0, CFGR_SW_MASK, CFGR_SW_PLL

    # Step 5: Wait for SWS to confirm PLL is system clock source
    li   t2, CLOCK_STARTUP_TIMEOUT
wait_sws:
    li   t0, R32_RCC_CFGR0
    lw   t1, 0(t0)
    li   a3, CFGR_SWS_MASK
    and  t1, t1, a3
    li   a4, CFGR_SWS_PLL
    beq  t1, a4, wait_sws_done
    addi t2, t2, -1
    bnez t2, wait_sws
    j    clock_init_fail
wait_sws_done:

    ret

clock_init_fail:
    j    clock_init_fail
