/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_time -- F427 time base for the App scheduler. The shared Platform/
 * bsp_time.h is the H7/HAL variant; this F427 version is bare-metal (CMSIS
 * only) and is picked up by placing Platform/f427 ahead of Platform on the
 * include path. The scheduler needs only Bsp_Time_GetMs(); Bsp_Time_GetCycles()
 * is provided inline for any DWT-based profiling (compiled out by default).
 */
#pragma once

#include <stdint.h>
#include "stm32f4xx.h"

/* Milliseconds since boot, advanced by the SysTick exception (bsp.c). Advances
 * only while interrupts are enabled. */
uint32_t Bsp_Time_GetMs(void);

/* Busy-wait delay in milliseconds (wraps bsp.c bsp_delay_ms, which works
 * without the SysTick ISR). Used by the host-link deferred work (e.g. the IAP
 * settle before Bsp_Boot_EnterIap). */
void Bsp_Time_DelayMs(uint32_t ms);

/* Free-running CPU-cycle counter (DWT CYCCNT). The app enables it
 * (CoreDebug DEMCR TRCENA + DWT CYCCNTENA); reads 0 until then. 32-bit, wraps
 * ~17 s at 250 MHz -- always take differences as uint32_t. */
__STATIC_FORCEINLINE uint32_t Bsp_Time_GetCycles(void) {
    return DWT->CYCCNT;
}
