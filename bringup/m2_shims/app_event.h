/* SPDX-License-Identifier: GPL-3.0-or-later */
/* M2 spike shim: the event-bus types and entry points the RX chain references.
 * The real App/app_event.h pulls in app_radio.h and the full main-loop
 * dispatcher (App layer, a later milestone); for the spike we declare just the
 * surface the SDR modules use. Subscribes/notifies are no-ops (m2_stubs.c): the
 * spike drives the chain directly, so no event ever needs to be dispatched.
 * SdrRxCfg/SdrTxCfg/SdrKeyToneCfg come from the real sdr_config.h. */
#pragma once

#include <stdint.h>

#include "sdr_common.h"
#include "sdr_config.h"

typedef enum {
    APP_RADIO_MODE_RX = 0,
    APP_RADIO_MODE_TX,
    APP_RADIO_MODE_OFF
} AppRadioMode;

typedef struct { AppRadioMode mode; } AppEventRxTxEvent;

typedef void (*AppEventRxTxFn)(const AppEventRxTxEvent *evt, void *user);
typedef void (*AppEventRxCfgFn)(const SdrRxCfg *cfg, void *user);
typedef void (*AppEventTxCfgFn)(const SdrTxCfg *cfg, void *user);
typedef void (*AppEventKeyToneFn)(const SdrKeyToneCfg *cfg, void *user);

int  App_Event_Init(void);
int  App_Event_SubscribeRxTx(AppEventRxTxFn cb, void *user);
int  App_Event_SubscribeRxCfg(AppEventRxCfgFn cb, void *user);
int  App_Event_SubscribeTxCfg(AppEventTxCfgFn cb, void *user);
int  App_Event_SubscribeKeyTone(AppEventKeyToneFn cb, void *user);
int  App_Event_NotifyRxTx(AppRadioMode mode);
int  App_Event_NotifyRxCfg(const SdrRxCfg *cfg);
int  App_Event_NotifyTxCfg(const SdrTxCfg *cfg);
int  App_Event_NotifyKeyTone(const SdrKeyToneCfg *cfg);
void App_Event_Process(void);
