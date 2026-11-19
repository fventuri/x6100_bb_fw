/* SPDX-License-Identifier: GPL-3.0-or-later */
/* M1 spike shim implementations: no-op event subscriptions and a "no valid
 * noise" spectrum, so the real sdr_am.c / sdr_iir.c link and run standalone. */
#include "app_event.h"
#include "sdr_spectrum.h"

int App_Event_SubscribeRxTx(AppEventRxTxFn cb, void *user) { (void)cb; (void)user; return 0; }
int App_Event_SubscribeRxCfg(AppEventRxCfgFn cb, void *user) { (void)cb; (void)user; return 0; }

float Sdr_Spectrum_Get_Noise_Lin(void) { return 0.0f; }
bool  Sdr_Spectrum_Is_Noise_Valid(void) { return false; }
float Sdr_Spectrum_Get_Demod_Noise_Est_Db(void) { return 0.0f; }
