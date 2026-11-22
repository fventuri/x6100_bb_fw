/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * m4_stubs_rx_log -- logging RX actuator backends for the M4a link/protocol
 * PROBE only (bringup/m4_hostlink_probe.c). They print the decoded tune/mode/
 * vol/rfg request so the probe transcript shows the state machine's output
 * without any radio hardware. The integrated firmware (bringup/m4_rx_host.c)
 * provides the real X6100 versions (VersaClock LO, RF band GPIO/BAND_V, TLV320)
 * instead of these.
 */

#include <stdint.h>

#include "app_fpga.h"
#include "app_radio.h"
#include "app_radio_rfpath.h"
#include "bsp_log.h"

#define LOG_TAG "m4.rxlog"

void App_Fpga_SetRxFrequency(uint32_t freq_hz) {
    LOG_INFO("App_Fpga_SetRxFrequency %lu Hz", (unsigned long)freq_hz);
}
void App_Fpga_SetTxFrequency(uint32_t freq_hz) { (void)freq_hz; }
void App_Fpga_TxMute(uint32_t ms) { (void)ms; }
void App_Fpga_SetSampleRate(uint8_t fftdec) { (void)fftdec; }

void App_RadioRfPath_SetFreq(uint32_t freq_hz) {
    LOG_INFO("App_RadioRfPath_SetFreq %lu Hz", (unsigned long)freq_hz);
}
bool App_RadioRfPath_LpfWillChange(uint32_t freq_hz) {
    (void)freq_hz;
    return false;
}
void App_RadioRfPath_SetRfg(uint8_t rfg) {
    LOG_INFO("App_RadioRfPath_SetRfg %u", (unsigned)rfg);
}
void App_RadioRfPath_SetAtt(bool on) { (void)on; }
void App_RadioRfPath_SetPre(bool on) { (void)on; }

static AppRadioMode s_radio_mode = APP_RADIO_MODE_OFF;

int App_Radio_SetRadioMode(AppRadioMode mode) {
    s_radio_mode = mode;
    return 0;
}
AppRadioMode App_Radio_GetRadioMode(void) { return s_radio_mode; }
void App_Radio_UpdateSpeaker(void) {}
void App_Radio_ApplyAudioConfig(void) { LOG_INFO("App_Radio_ApplyAudioConfig"); }
