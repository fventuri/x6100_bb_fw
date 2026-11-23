/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * m5_stubs -- safe stubs for the TX-side App modules that app_radio.c / the
 * real app_protect reference but the current milestone does NOT bring up yet.
 * TX stays firmly OFF regardless (App_Alc_GetOutputScale() -> 0, Sdr_Tx ->
 * zeros, PA GPIOs safe-off in bsp_board_f427).
 *
 * M5a-2 brings up the REAL app_protect (battery/BMS + provisional PA-temp +
 * protect tick), so App_Protect_IsTxInhibited is NO LONGER stubbed here. The
 * modules app_protect's tick calls that are out of M5a-2 scope are stubbed
 * safe: App_Ptt_ForceRx (force RX -- TX never engages yet anyway), App_Power_Tick
 * (no battery auto-off during bring-up), App_PaTempComp_Tick10ms (no PA-bias DAC
 * writes until M5b), and App_TxMonitor_TakeSwrTrip (SWR/power arming + detector
 * calibration are deferred to M5c, where RF exists). The real modules land in
 * M5b/M5c. (App_Alc_SetPowerLimit/NotifyFrequency + the App_Ptt setters +
 * App_Protect_SetChargeEnabled live in m4_stubs_common.c; not redefined here.)
 */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_alc.h"
#include "app_mic_pre.h"
#include "app_pa_temp_comp.h"
#include "app_power.h"
#include "app_ptt.h"
#include "app_radio.h"
#include "app_telemetry.h"
#include "app_tx_monitor.h"
#include "drv_charger.h"
#include "sdr_common.h"   /* SdrModulation */
#include "sdr_cw.h"
#include "sdr_tx.h"

/* --- ALC: no shaping; report a zero output scale (TX inert) --------------- */
float App_Alc_GetOutputScale(const float *i, const float *q, uint32_t count)
{
    (void)i; (void)q; (void)count;
    return 0.0f;
}
void App_Alc_NotifyTxStart(void)            { }
void App_Alc_SetSwrScanActive(bool active)  { (void)active; }

/* --- protect-tick callees out of M5a-2 scope (safe no-ops) ---------------- *
 * The real app_protect is compiled in M5a-2 and owns App_Protect_IsTxInhibited.
 * Its 10 ms tick also calls these; keep them inert while PA/TX stay off. */
void App_Ptt_ForceRx(void) {
    /* TX never engages in M5a-2, but honour the contract: ensure RX. */
    (void)App_Radio_SetRadioMode(APP_RADIO_MODE_RX);
}
void App_Power_Tick(void)         { }  /* no battery auto-off during bring-up */
void App_PaTempComp_Tick10ms(void) { } /* no PA-bias DAC writes until M5b */

/* SWR/forward-power monitor deferred to M5c (needs RF to calibrate the X6100
 * detector curve); report no trip so app_protect's tick stays quiet. */
bool App_TxMonitor_TakeSwrTrip(void) { return false; }

/* --- charger (bq24715) DEFERRED ------------------------------------------- *
 * The MAX17205 gauge works on I2C1 and gives app_protect all the battery
 * protection it needs (cell OV/UV, pack temp, SOC). The bq24715 CHARGER is a
 * separate charging-management bring-up (its one-time SMBus config per
 * x6100-base/docs/charger.md) and is NOT part of M5a-2. app_protect reads the
 * gauge + charger as one atomic burst when the adapter is present, so a real
 * driver that NAKs would wipe out the gauge telemetry on external DC (observed:
 * data=0 with adapter present, data=1 on battery). These benign stubs report
 * "charger OK, charge managed" so the gauge telemetry stays valid in BOTH
 * modes; NO actual charging happens (enable/inhibit/current are no-ops). The
 * real driver returns in the charging milestone. */
int Drv_Charger_Init(void)                      { return 0; }
int Drv_Charger_EnableCharge(void)              { return 0; }
int Drv_Charger_InhibitCharge(void)             { return 0; }
int Drv_Charger_SetChargeCurrent(uint32_t ma)   { (void)ma; return 0; }
int Drv_Charger_ReadStatus(DrvChargerStatus *status) {
    if (status != 0) {
        status->option = 0u;
        status->ac_ok  = true; /* only read when app_protect sees the adapter */
    }
    return 0;
}

/* --- mic preamp: no-op (TX-only; M5-0 RX never calls Process) ------------- */
void App_MicPre_Init(void)                      { }
void App_MicPre_Reset(void)                     { }
void App_MicPre_SetEnabled(bool enabled)        { (void)enabled; }
void App_MicPre_Process(float *samples, uint32_t n) { (void)samples; (void)n; }

/* --- telemetry TX flag: no-op -------------------------------------------- */
void App_Telemetry_SetTx(bool tx) { (void)tx; }

/* --- CW keyer: no sidetone in M5-0 --------------------------------------- *
 * app_radio's radio_block_cb calls Sdr_CW_KeyerUpdate() every block and adds
 * the returned sidetone buffer when non-NULL. The real keyer (sdr_cw.c) drags
 * the paddle EXTI / TIM7 / board layer, out of scope for the RX-only M5-0, and
 * Sdr_CW_Init() is the m3 no-op stub. Returning NULL = no sidetone injected.
 * (Sdr_CW_Init/Demod/GetSignalMetric/Get_Rx_Level_Db live in m3_app_stubs.c.) */
float *Sdr_CW_KeyerUpdate(void) { return NULL; }

/* --- TX modulator: inert in M5-0 ----------------------------------------- *
 * app_radio calls Sdr_Tx_Init() at bring-up and Sdr_Tx_Process() only in its TX
 * branch, which M5-0 never enters (mode stays RX/OFF; App_Protect_IsTxInhibited
 * is hard-true). Real sdr_tx.c (modulators + CW constant-envelope) lands in M5c.
 * Process fills both output buffers with zeros, honouring its contract. */
void Sdr_Tx_Init(void) { }
void Sdr_Tx_Process(const float *audio48, uint32_t n, SdrModulation mode,
                    float *out_i, float *out_q, float *monitor)
{
    (void)audio48;
    (void)mode;
    for (uint32_t k = 0; k < n; ++k) {
        out_i[k]    = 0.0f;
        out_q[k]    = 0.0f;
        monitor[k]  = 0.0f;
    }
}
