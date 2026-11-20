/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * Stubs for the few SDR symbols the RX radio app does not yet link real.
 * The event bus (app_event.c), scheduler (app_scheduler.c) and spectrum
 * (sdr_spectrum.c) are now linked REAL, so only these remain:
 *
 *  - Sdr_NR_*  : noise reduction, default-disabled and out of the AM path;
 *    stubbed to keep its rFFT out of the link.
 *  - Sdr_CW_*  : the real sdr_cw.c drags the keyer/PTT/paddle-EXTI + board
 *    headers (App/board layer, not yet ported); CW is not in the AM path.
 */
#include <stdbool.h>
#include <stdint.h>

#include "sdr_common.h"   /* Sdr_IQ_Data */

/* ---- Noise reduction (stubbed; disabled) ------------------------------ */
void Sdr_NR_Init(void) { }
void Sdr_NR_Process(float *samples, uint32_t n) { (void)samples; (void)n; }

/* ---- CW demod (stubbed) ------------------------------------------------ */
void  Sdr_CW_Init(void) { }
void  Sdr_CW_Demod(const Sdr_IQ_Data *data, float *audio)
{ (void)data; (void)audio; }
float Sdr_CW_GetSignalMetric(void) { return 0.0f; }
float Sdr_CW_Get_Rx_Level_Db(void) { return -140.0f; }
