/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M0 bring-up: minimal boot + debug banner on the X6100 BASE (STM32F427).
 * ======================================================================
 * The M0 milestone (PORTING.md): prove the new repo builds for the F427 with
 * the no-HAL toolchain and that the board reaches main(), establishes a safe
 * clock tree, and prints over the diagnostic USART. No App/Devices code yet.
 *
 * Exit criteria: stable reset; version banner + a 1 Hz heartbeat line on
 * USART1 (PA9/PA10, 115200 8-N-1); PA / charger / relays never asserted.
 *
 * This reuses the hardware-validated foundation in Platform/f427 (bsp_*):
 *   - bsp_gpio_init_safe() asserts the recovered safe GPIO state FIRST
 *     (PA off, charger off, RX-safe) before anything else.
 *   - bsp_clock_init_250mhz() brings up the OEM clock tree.
 */
#include "bsp.h"

#define M0_VERSION "x6100_bb_fw M0-boot 2026-11-18"

int main(void)
{
    /* Safe GPIO FIRST: PA off, charger off, relays safe -- before any
     * nonessential peripheral and before the clock is sped up. */
    bsp_gpio_init_safe();

    /* Safe clock tree, 1 ms timebase, diagnostic USART. */
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== %s ===\r\n", M0_VERSION);
    bsp_uprintf("clock: %s, SystemCoreClock=%lu Hz\r\n",
                BSP_CLK_DESC, (unsigned long)SystemCoreClock);
    if (bsp_clk_fail)
        bsp_uprintf("WARNING: clock init flags = 0x%lx\r\n",
                    (unsigned long)bsp_clk_fail);
    bsp_uprintf("safe GPIO asserted; PA + charger + relays OFF.\r\n");
    bsp_uprintf("heartbeat:\r\n");

    uint32_t beat = 0;
    for (;;) {
        bsp_uprintf("  [%lu] alive\r\n", (unsigned long)beat++);
        bsp_delay_ms(1000);
    }
}
