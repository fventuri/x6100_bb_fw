/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * drv_fpga_codec -- X6100 RX-producer backend for drv_fpga.h. See the header.
 */
#include "drv_fpga_codec.h"

#include <stddef.h>

#include "board_config.h"

/*
 * Driver-owned raw (DSP) window: I and Q in separate Q1.31 buffers, a power-of-
 * two ring so the consumer wraps with a cheap mask. DRV_FPGA_RAW_WINDOW_SAMPLES
 * (= 2 * DRV_CODEC_BLOCK_FRAMES * 8 = 1024) holds eight 128-sample banks.
 */
static int32_t s_raw_i[DRV_FPGA_RAW_WINDOW_SAMPLES];
static int32_t s_raw_q[DRV_FPGA_RAW_WINDOW_SAMPLES];

static const DrvFpgaRawWindow s_window = {
    .i     = s_raw_i,
    .q     = s_raw_q,
    .count = DRV_FPGA_RAW_WINDOW_SAMPLES,
};

/* Write index into the ring, in complex samples. 32-bit aligned: the store that
 * publishes a freshly written bank is atomic against the consumer on this core. */
static volatile uint32_t s_pos;

/* The X6100 raw rate is fixed (externally clocked), so the ring contents never
 * become invalid the way an FPGA fftdec retune would. Start at 1 (non-zero).
 * It is still bumped once by Drv_Fpga_Codec_NotifyRateChange() when the App RX
 * config path crosses a fftdec boundary, so a staged Sdr_Rx config commits (see
 * the header). Not const: the bump is a single-word write from task context. */
static volatile uint32_t s_generation = 1u;

#define RAW_RING_MASK (DRV_FPGA_RAW_WINDOW_SAMPLES - 1u)

_Static_assert((DRV_FPGA_RAW_WINDOW_SAMPLES & RAW_RING_MASK) == 0u,
               "raw window must be a power of two");
_Static_assert((DRV_FPGA_RAW_WINDOW_SAMPLES % DRV_FPGA_CODEC_BANK_FRAMES) == 0u,
               "ring must hold a whole number of I2S banks");
_Static_assert((DRV_FPGA_SAMPLES_PER_TICK_BASE >> DRV_FPGA_CODEC_FFTDEC) == 2u,
               "X6100 fftdec must give M = 2");
_Static_assert(BOARD_AUDIO_SAMPLE_RATE_HZ * 2u == 100000u,
               "X6100 native raw rate must be base*2 = 100 kHz");

void Drv_Fpga_Codec_Init(void)
{
    for (uint32_t n = 0; n < DRV_FPGA_RAW_WINDOW_SAMPLES; ++n) {
        s_raw_i[n] = 0;
        s_raw_q[n] = 0;
    }
    s_pos = 0u;
}

void Drv_Fpga_Codec_PushIqBank(const uint16_t *rx_iq)
{
    if (rx_iq == NULL) {
        return;
    }

    uint32_t w = s_pos;
    for (uint32_t f = 0; f < DRV_FPGA_CODEC_BANK_FRAMES; ++f) {
        const uint16_t *hw4 = &rx_iq[4u * f];
        /* ch0 = I, ch1 = Q; each 32-bit word is codec-left-justified -> Q1.31
         * (common/i2s.c i2s_reconstruct_frame: hi halfword then lo). */
        int32_t i_q31 = (int32_t)(((uint32_t)hw4[0] << 16) | hw4[1]);
        int32_t q_q31 = (int32_t)(((uint32_t)hw4[2] << 16) | hw4[3]);

        uint32_t idx = (w + f) & RAW_RING_MASK;
        s_raw_i[idx] = i_q31;
        s_raw_q[idx] = q_q31;
    }

    /* Publish the whole bank at once (single aligned store). */
    s_pos = (w + DRV_FPGA_CODEC_BANK_FRAMES) & RAW_RING_MASK;
}

/* ---- drv_fpga.h RX-producer contract ---------------------------------- */

const DrvFpgaRawWindow *Drv_Fpga_GetRawWindow(void)
{
    return &s_window;
}

uint32_t Drv_Fpga_GetPos(void)
{
    return s_pos;
}

uint32_t Drv_Fpga_GetRawGeneration(void)
{
    return s_generation;
}

void Drv_Fpga_Codec_NotifyRateChange(void)
{
    s_generation++;
}

uint32_t Drv_Fpga_GetSamplesPerTick(void)
{
    return DRV_FPGA_SAMPLES_PER_TICK_BASE >> DRV_FPGA_CODEC_FFTDEC; /* = 2 */
}

uint8_t Drv_Fpga_GetFftdec(void)
{
    return DRV_FPGA_CODEC_FFTDEC;
}

uint8_t Drv_Fpga_GetRequestedFftdec(void)
{
    return DRV_FPGA_CODEC_FFTDEC;
}
