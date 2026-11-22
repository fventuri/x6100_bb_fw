/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * m4_stubs_common -- backend stubs shared by the M4a probe and the integrated
 * RX host firmware. These cover the TX / ATU / PA / protection / power / NVRAM
 * surface the clean App host-link stack references but that is M5/M6 territory,
 * plus the CW/NR setters (NR/CW DSP is stubbed; see m3_shims/m3_app_stubs.c) and
 * Drv_Fpga_GetVersion (the one Drv_Fpga_* symbol the RX producer lacks).
 *
 * The RX actuators (App_Fpga_*, App_RadioRfPath_*, App_Radio_*) are NOT here:
 * the probe links logging versions (m4_stubs_rx_log.c), the integrated firmware
 * links the real X6100 hardware backends (bringup/m4_rx_host.c).
 */

#include <stddef.h>
#include <stdint.h>

#include "app_alc.h"
#include "app_atu.h"
#include "app_hostcmd.h"
#include "app_hostlink_ext.h"
#include "app_nvram.h"
#include "app_periph.h"
#include "app_power.h"
#include "app_protect.h"
#include "app_ptt.h"
#include "bsp_boot.h"
#include "bsp_log.h"
#include "drv_fpga.h"
#include "drv_nvram.h"
#include "drv_pa_bias.h"
#include "sdr_cw.h"
#include "sdr_nr.h"

#define LOG_TAG "m4.stub"

/* --- ALC (TX power shaping) ---------------------------------------------- */
void App_Alc_SetPowerLimit(float txpwr_w) { (void)txpwr_w; }
void App_Alc_NotifyFrequency(uint32_t freq_hz) { (void)freq_hz; }

/* --- ATU ------------------------------------------------------------------ */
void App_Atu_RequestTune(void) { LOG_INFO("App_Atu_RequestTune (stub)"); }
void App_Atu_ApplyNetwork(uint8_t c, uint8_t l, uint8_t sw) {
    (void)c;
    (void)l;
    (void)sw;
}
void App_Atu_SetOn(bool on) { (void)on; }
bool App_Atu_IsTuning(void) { return false; }

/* --- PTT (TX keying) ------------------------------------------------------ */
void App_Ptt_SetMtx(bool on) { (void)on; }
void App_Ptt_SetIptt(bool on) { (void)on; }
void App_Ptt_SetCivPtt(bool on) { (void)on; }
void App_Ptt_SetCalibrate(bool on) { (void)on; }
void App_Ptt_SetSwrScan(bool on) { (void)on; }

/* --- protection / PA bias ------------------------------------------------- */
void App_Protect_SetChargeEnabled(bool enabled) { (void)enabled; }
int  Drv_PaBias_SetBias(uint16_t bias_drive, uint16_t bias_final) {
    (void)bias_drive;
    (void)bias_final;
    return 0;
}
int Drv_PaBias_SetOff(bool drive_off, bool final_off) {
    (void)drive_off;
    (void)final_off;
    return 0;
}

/* --- power ---------------------------------------------------------------- */
void App_Power_NotifyHostFrame(void) {}
void App_PowerOff_Request(void) { LOG_INFO("App_PowerOff_Request (stub)"); }

/* --- periph --------------------------------------------------------------- */
uint8_t App_Periph_HwRev(void) { return 0u; }

/* --- NVRAM / calib (static RAM calib) ------------------------------------- */
static DrvNvramCalib s_calib; /* zeroed */

const DrvNvramCalib *App_Nvram_Calib(void) { return &s_calib; }
int App_Nvram_CalibWrite(uint16_t off, const uint8_t *src, size_t len) {
    if (src == NULL || off >= DRV_NVRAM_CALIB_LEN) {
        return -1;
    }
    if (len > (size_t)(DRV_NVRAM_CALIB_LEN - off)) {
        len = (size_t)(DRV_NVRAM_CALIB_LEN - off);
    }
    for (size_t i = 0; i < len; ++i) {
        ((uint8_t *)&s_calib)[off + i] = src[i];
    }
    return 0;
}
int App_Nvram_CalibSetBias(uint16_t bias_drive, uint16_t bias_final) {
    (void)bias_drive;
    (void)bias_final;
    return 0;
}
void App_Nvram_SetChargeEnabled(bool enabled) { (void)enabled; }

/* --- host command handlers ------------------------------------------------ */
int App_HostCmd_CalibDump(void) { return 0; }
int App_HostCmd_CalibSave(void) { return 0; }

/* --- host-link extension register window ---------------------------------- */
bool App_HostLinkExt_Write(uint16_t reg, const uint8_t *data, size_t len) {
    (void)reg;
    (void)data;
    (void)len;
    return false; /* not handled -> fall through to the command block */
}

/* --- CW keyer / noise reduction setters (DSP stubbed; Init/Process/Demod come
 * from m3_shims/m3_app_stubs.c). ------------------------------------------- */
void Sdr_NR_SetThreshold(float nrthr) { (void)nrthr; }
void Sdr_NR_SetEnabled(bool enabled) { (void)enabled; }
void Sdr_CW_SetKeySpeed(uint32_t wpm) { (void)wpm; }
void Sdr_CW_SetKeyMode(SdrCwKeyMode mode) { (void)mode; }
void Sdr_CW_SetIambicMode(SdrCwIambicMode mode) { (void)mode; }
void Sdr_CW_SetSidetoneVolume(uint8_t cwvol) { (void)cwvol; }
void Sdr_CW_SetTraining(bool training) { (void)training; }
void Sdr_CW_SetQskTimeAndRatio(uint32_t qsk_ms, uint32_t dash_ratio_x10) {
    (void)qsk_ms;
    (void)dash_ratio_x10;
}

/* --- boot / IAP ----------------------------------------------------------- */
void Bsp_Boot_EnterIap(void) {
    LOG_INFO("Bsp_Boot_EnterIap (stub)");
    for (;;) {
    }
}

/* --- FPGA version (the one Drv_Fpga_* symbol the RX producer lacks) ------- */
void Drv_Fpga_GetVersion(uint8_t ver[3]) {
    ver[0] = 0u;
    ver[1] = 0u;
    ver[2] = 0u;
}
