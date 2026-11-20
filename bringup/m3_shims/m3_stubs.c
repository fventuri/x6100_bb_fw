/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * Stubs for the not-yet-ported system layer the real App/SDR/sdr_spectrum.c
 * links against: the application scheduler. Sdr_Spectrum_Init registers its
 * 40 ms FFT task via App_Scheduler_Add; the stub captures that function pointer
 * so the spike can run the task deterministically (Spike_Spectrum_Task), rather
 * than spinning a scheduler loop. App_Scheduler_Notify is a no-op (the spike
 * reads the results directly instead of handling the telemetry event).
 */
#include "app_scheduler.h"

static AppSchedulerTaskFn s_spectrum_task;

int App_Scheduler_Add(AppSchedulerTaskFn fn, uint32_t period_ms)
{
    (void)period_ms;
    s_spectrum_task = fn;   /* capture Sdr_Spectrum_Task for the spike to call */
    return 0;
}

int App_Scheduler_AddEvent(AppSchedulerEventFn fn, AppSchedulerEvent ev)
{
    (void)fn; (void)ev;
    return 0;
}

void App_Scheduler_Notify(AppSchedulerEvent ev)
{
    (void)ev;
}

void App_Scheduler_Run(void)
{
}

/* Spike hook: run the captured spectrum task once (the 40 ms FFT body). */
void Spike_Spectrum_Task(void)
{
    if (s_spectrum_task != NULL) {
        s_spectrum_task();
    }
}
