/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M2 spike stubs for the not-yet-ported system layer, mirroring m1_stubs.c.
 *
 *  - App_Event_* : the SDR modules subscribe at init and sdr_rx publishes its
 *    committed RxCfg. The spike drives the chain directly, so every event
 *    entry point is a no-op; subscribes report success.
 *  - Sdr_Spectrum_* : the AM squelch metric and the RX S-meter read the FFT
 *    spectrum/noise estimate. The real App/SDR/sdr_spectrum.c (with its rFFT) is
 *    a later milestone; here the four accessors the RX path calls return
 *    "no valid noise estimate" / a floor S-meter. These match the real
 *    App/SDR/sdr_spectrum.h declarations (which are sibling-included unchanged).
 */
#include <stdbool.h>
#include <stdint.h>

#include "app_event.h"

/* ---- App event bus (no-op) -------------------------------------------- */

int App_Event_Init(void) { return 0; }

int App_Event_SubscribeRxTx(AppEventRxTxFn cb, void *user)
{ (void)cb; (void)user; return 0; }
int App_Event_SubscribeRxCfg(AppEventRxCfgFn cb, void *user)
{ (void)cb; (void)user; return 0; }
int App_Event_SubscribeTxCfg(AppEventTxCfgFn cb, void *user)
{ (void)cb; (void)user; return 0; }
int App_Event_SubscribeKeyTone(AppEventKeyToneFn cb, void *user)
{ (void)cb; (void)user; return 0; }

int App_Event_NotifyRxTx(AppRadioMode mode) { (void)mode; return 0; }
int App_Event_NotifyRxCfg(const SdrRxCfg *cfg) { (void)cfg; return 0; }
int App_Event_NotifyTxCfg(const SdrTxCfg *cfg) { (void)cfg; return 0; }
int App_Event_NotifyKeyTone(const SdrKeyToneCfg *cfg) { (void)cfg; return 0; }

void App_Event_Process(void) { }

/* ---- Spectrum / noise estimate (floor stubs) -------------------------- */

float Sdr_Spectrum_Get_Noise_Lin(void) { return 0.0f; }
bool  Sdr_Spectrum_Is_Noise_Valid(void) { return false; }
float Sdr_Spectrum_Get_Demod_Noise_Est_Db(void) { return 0.0f; }
const float Sdr_Spectrum_Get_S_meter(void) { return -140.0f; }

/* ---- Noise reduction (stubbed; disabled in this spike) ----------------
 * sdr_rx.c calls Sdr_NR_Init/Process; NR is default-disabled, so Process is a
 * no-op either way. Stubbing it keeps the real rFFT (its only use in the RX
 * set) out of the link. Signatures match the real App/SDR/sdr_nr.h. */
void Sdr_NR_Init(void) { }
void Sdr_NR_Process(float *samples, uint32_t n) { (void)samples; (void)n; }

/* ---- CW demod (stubbed) -----------------------------------------------
 * sdr_rx.c and sdr_squelch.c reference the CW surface, but the real sdr_cw.c
 * pulls in the keyer/PTT/paddle-EXTI and the H7 board headers (main.h,
 * app_ptt, bsp_board/gpio/time) -- the App/board layer, out of M2 scope. CW is
 * not in the AM datapath under test, so stub it. Signatures match sdr_cw.h. */
void  Sdr_CW_Init(void) { }
void  Sdr_CW_Demod(const Sdr_IQ_Data *data, float *audio)
{ (void)data; (void)audio; }
float Sdr_CW_GetSignalMetric(void) { return 0.0f; }
float Sdr_CW_Get_Rx_Level_Db(void) { return -140.0f; }
