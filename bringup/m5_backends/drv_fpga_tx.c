/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * drv_fpga TX seams (F427, no-HAL) -- the transmit half of the Drv_Fpga device
 * surface that app_radio.c drives (Drv_Fpga_TxWrite / Drv_Fpga_SetTxActive).
 * The X6100 has no FPGA host bus; the RX half is faked by the codec I/Q ring in
 * Platform/f427/drv_fpga_codec.c (GetRawWindow/GetPos). These TX seams complete
 * that surface.
 *
 * M5-0 SCOPE: TX is INERT. SetTxActive only latches a flag (no T/R, no PA), and
 * TxWrite ACCEPTS the block (so app_radio's publish loop/ALC run normally) but
 * DISCARDS it -- the I2S3ext TX I/Q bank stays zero-filled, so NO RF is emitted.
 * M5c replaces TxWrite's body with a real Q1.31 staging ring drained into tx_iq.
 */
#include "drv_fpga.h"

static volatile int s_tx_active;   /* latched request; no hardware effect yet */

/* Accept and discard: report the whole block as written so app_radio does not
 * count false drops, but emit nothing (tx_iq stays zero -> no carrier). */
uint32_t Drv_Fpga_TxWrite(const int32_t *i, const int32_t *q, uint32_t count)
{
    (void)i;
    (void)q;
    return count;
}

void Drv_Fpga_SetTxActive(int active) { s_tx_active = active; }

/* TX tuning / mute: the VersaClock LO is driven by the App_Fpga_* path (shared
 * RX/TX synth on the X6100), so these device seams are no-ops here. */
void Drv_Fpga_SetTxFrequency(uint32_t freq_hz) { (void)freq_hz; }
void Drv_Fpga_TxMute(uint32_t ms)              { (void)ms; }
