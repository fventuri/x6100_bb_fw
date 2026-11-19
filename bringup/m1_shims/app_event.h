/* SPDX-License-Identifier: GPL-3.0-or-later */
/* M1 spike shim: the event-bus types and subscribe entry points the demod uses
 * at init. Real App/app_event.h pulls app_radio.h + the full dispatcher; for
 * the spike we define just what sdr_am.c references (SdrRxCfg comes from the
 * real sdr_config.h). Subscriptions are no-ops (m1_stubs.c). */
#pragma once

#include <stdint.h>

#include "sdr_common.h"
#include "sdr_config.h"   /* real: SdrRxCfg { rx_mode, filter_low_hz, filter_high_hz } */

typedef enum {
    APP_RADIO_MODE_RX = 0,
    APP_RADIO_MODE_TX,
    APP_RADIO_MODE_OFF
} AppRadioMode;

typedef struct { AppRadioMode mode; } AppEventRxTxEvent;

typedef void (*AppEventRxTxFn)(const AppEventRxTxEvent *evt, void *user);
typedef void (*AppEventRxCfgFn)(const SdrRxCfg *cfg, void *user);

int App_Event_SubscribeRxTx(AppEventRxTxFn cb, void *user);
int App_Event_SubscribeRxCfg(AppEventRxCfgFn cb, void *user);
