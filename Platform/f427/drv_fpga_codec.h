/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * drv_fpga_codec -- X6100 (STM32F427) RX-producer backend for the upstream
 * drv_fpga.h contract.
 * ============================================================================
 * The X6200 firmware feeds its RX DSP from an FPGA over a bit-banged GPIOE bus
 * (Devices/drv_fpga.c). The X6100 has no such bus: raw RX I/Q arrives from the
 * codec/FPGA over I2S3 as 128-frame DMA banks at a fixed, externally-clocked
 * 100 kHz (common/i2s.c: i2s_rate_raw = 100000). This backend implements just
 * the RX-producer half of drv_fpga.h -- the part Sdr_Rx_Process() consumes --
 * on top of that codec transport, leaving App/SDR/sdr_rx.c unmodified:
 *
 *   Drv_Fpga_GetRawWindow()      -> driver-owned Q1.31 I/Q ring (separate i/q)
 *   Drv_Fpga_GetPos()            -> ring write index
 *   Drv_Fpga_GetSamplesPerTick() -> M = 2 (100 kHz = base 50 kHz * 2)
 *   Drv_Fpga_GetRawGeneration()  -> constant (X6100 rate is fixed, never retunes)
 *   Drv_Fpga_GetFftdec()/GetRequestedFftdec() -> 2 (M = 8 >> fftdec = 2)
 *
 * Rate mapping (50 kHz re-base): base audio rate = 50 kHz, X6100 native raw
 * rate = 100 kHz = base*2, so M = 2 and fftdec = 2 (M = DRV_FPGA_SAMPLES_PER_
 * TICK_BASE >> fftdec = 8 >> 2 = 2). One I2S bank (128 complex frames @ 100 kHz)
 * is exactly one upstream RX block (DRV_CODEC_BLOCK_FRAMES * M = 64 * 2 = 128),
 * so each bank drives exactly one Sdr_Rx_Process() call.
 *
 * TX, tuning, calibration and status (the rest of drv_fpga.h) are NOT provided
 * here; they belong to later milestones and a separate X6100 driver.
 */
#pragma once

#include <stdint.h>

#include "drv_fpga.h"

/* fftdec for the X6100's fixed 100 kHz raw rate: M = 8 >> 2 = 2. */
#define DRV_FPGA_CODEC_FFTDEC 2u

/* Reset the raw ring, write index and generation. Call before Sdr_Rx_Init(). */
void Drv_Fpga_Codec_Init(void);

/*
 * Ingest one completed I2S3 RX I/Q bank: I2S_BANK_FRAMES (128) stereo frames,
 * 4 halfwords each (ch0 hi, ch0 lo, ch1 hi, ch1 lo), ch0 = I, ch1 = Q. Each
 * 32-bit word is already codec-left-justified, i.e. Q1.31. The frames are
 * deinterleaved into the driver-owned i/q ring and the write index advances by
 * 128. Mirrors what the real DMA1_Stream3 i2s_dsp() hook passes as rx_iq.
 */
void Drv_Fpga_Codec_PushIqBank(const uint16_t *rx_iq);

/*
 * Bump the raw generation. The X6100 rate is fixed, so the hardware never
 * retunes -- but the App RX config path (app_radio_state -> Sdr_Rx_SetConfig)
 * STAGES any config whose fftdec differs from the committed one and only commits
 * it when radio_read_iq() observes a generation change (the X6200 FPGA bumps it
 * on a rate retune). With a constant generation that commit never fires and the
 * staged mode/filter change is lost. The X6100 backend's App_Fpga_SetSampleRate
 * -- the rate-change barrier -- calls this so the staged config commits in step,
 * exactly as the handshake intends; the consumer then does one resync.
 */
void Drv_Fpga_Codec_NotifyRateChange(void);

/* Frames per I2S bank (one upstream RX block). Matches common/i2s.h. */
#define DRV_FPGA_CODEC_BANK_FRAMES 128u
