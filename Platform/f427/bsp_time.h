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

/* The DWT CYCCNT counts the Cortex-M4 CPU clock, which on the F427 is HCLK.
 * Consumers (e.g. app_protect's ADC-DMA health check) convert cycles to time
 * with this. */
uint32_t Bsp_Time_GetCpuHz(void);

/*
 * Board housekeeping timer, matching the shared Platform/bsp_time.h contract
 * app_protect expects. Only BSP_TIME_TIMER_PROTECT is implemented on the F427,
 * and NOT as a hardware-timer ISR: the backend registers the callback on the
 * cooperative App scheduler (thread context), because every I2C2 user on this
 * board (codec + gauge/charger) runs in the main loop and the DSP lives in the
 * I2S ISRs. Running the protect tick cooperatively serialises all I2C2 access
 * without ISR-context I2C or a bus guard. The callback still must stay short.
 */
typedef enum {
    BSP_TIME_TIMER_PROTECT = 0, /* 10 ms housekeeping/protection tick */
    BSP_TIME_TIMER_CW_KEY,      /* reserved (CW paddle) -- not implemented yet */
    BSP_TIME_TIMER_COUNT
} BspTimeTimer;

typedef void (*BspTimeTimerCallback)(void *ctx);

/* Register a handler and start a board timer. 0 = success, negative = error. */
int Bsp_Time_StartTimer(BspTimeTimer timer, BspTimeTimerCallback callback,
                        void *ctx);
