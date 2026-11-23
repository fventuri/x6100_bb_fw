/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * bsp_timer_sched -- F427 backend for the Bsp_Time board-timer contract used by
 * app_protect, plus Bsp_Time_GetCpuHz(). See Platform/f427/bsp_time.h for why
 * the protect tick runs on the cooperative App scheduler (thread context)
 * instead of a hardware-timer ISR: every I2C2 user on this board runs in the
 * main loop, so a cooperative tick serialises all I2C2 access with no ISR-
 * context I2C and no bus guard. Bsp_Time_GetMs/DelayMs/GetCycles live in bsp.c.
 */
#include "bsp_time.h"

#include "app_scheduler.h"
#include "bsp.h" /* HCLK_HZ */

#define PROTECT_TICK_MS 10u

static BspTimeTimerCallback s_protect_cb;
static void               *s_protect_ctx;

/* Scheduler thunk: the scheduler fn takes no argument, so trampoline through the
 * stored (callback, ctx). Runs in main-loop context every PROTECT_TICK_MS. */
static void protect_thunk(void) {
    if (s_protect_cb != 0) {
        s_protect_cb(s_protect_ctx);
    }
}

uint32_t Bsp_Time_GetCpuHz(void) {
    return HCLK_HZ;
}

int Bsp_Time_StartTimer(BspTimeTimer timer, BspTimeTimerCallback callback,
                        void *ctx) {
    if (callback == 0) {
        return -1;
    }
    if (timer != BSP_TIME_TIMER_PROTECT) {
        /* Only the protection tick is needed in M5a-2; the CW-key timer is a
         * later milestone. */
        return -2;
    }
    s_protect_cb  = callback;
    s_protect_ctx = ctx;
    return App_Scheduler_Add(protect_thunk, PROTECT_TICK_MS);
}
