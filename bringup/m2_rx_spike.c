/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M2 end-to-end RX spike: the real upstream RX pipeline (App/SDR/sdr_rx.c,
 * UNMODIFIED, with the real decimation + AM demod + audio resamplers) fed by
 * the X6100 codec producer backend (Platform/f427/drv_fpga_codec.c), and the
 * demodulated audio upsampled 50 -> 100 kHz into a codec TX bank. This is the
 * M2 "transport graft" proof: codec I/Q -> on-MCU decimation into the RX ring
 * -> demod -> DAC, all on the F427 in Renode, no FPGA bus and no hardware.
 *
 * Input: a synthetic X6100 I2S3 RX I/Q bank (128 frames @ 100 kHz, Q1.31,
 * ch0 = I, ch1 = Q), an AM signal -- complex carrier at +500 Hz amplitude-
 * modulated by a 1 kHz tone at depth 0.5. The carrier amplitude is small
 * because the RX chain runs at the AGC "off" gain (+70 dB); the recovered
 * audio is a clean 1 kHz tone whose amplitude reflects that fixed gain.
 *
 * Flow per bank (what the real DMA1_Stream3 i2s_dsp() hook will do):
 *   1. Drv_Fpga_Codec_PushIqBank()  : deinterleave 128 I/Q frames into the ring
 *   2. Sdr_Rx_Process(audio50, 64)  : ring -> decimate 100->25k -> AM demod ->
 *                                     audio resample -> 64 frames @ 50 kHz
 *   3. upsample 50 -> 100 kHz (x2)  : fill the 128-frame codec TX-audio bank
 *
 * PASS: audio is produced and the recovered tone sits at ~1000 Hz (the
 * gain-independent invariants); the TX bank is populated.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"
#include "drv_codec.h"
#include "drv_fpga_codec.h"
#include "sdr_agc.h"
#include "sdr_am.h"
#include "sdr_audio_resample.h"
#include "sdr_common.h"
#include "sdr_config.h"
#include "sdr_nb.h"
#include "sdr_notch.h"
#include "sdr_nr.h"
#include "sdr_resample.h"
#include "sdr_rx.h"
#include "sdr_squelch.h"

#define RAW_FS    100000.0f                 /* X6100 native raw I/Q rate     */
#define AUDIO_FS  50000.0f                  /* base audio rate (BOARD_...)   */
#define BANK      DRV_FPGA_CODEC_BANK_FRAMES /* 128 complex frames per bank  */
#define AUDIO_N   DRV_CODEC_BLOCK_FRAMES     /* 64 audio frames per RX block */

#define F_OFF     500.0f    /* carrier offset from DC */
#define F_MOD     1000.0f   /* modulating tone        */
#define MOD_DEPTH 0.5f
#define CARRIER_A 2.5e-4f   /* small: chain runs at AGC-off +70 dB */

#define WARMUP_BLOCKS 96u
#define MEAS_BLOCKS   96u

static uint16_t s_rx_bank[BANK * 4u];   /* synthetic I2S3 RX I/Q bank        */
static uint16_t s_tx_bank[BANK * 4u];   /* I2S2ext TX audio bank (100 kHz)   */
static float    s_audio50[AUDIO_N];     /* Sdr_Rx_Process output (50 kHz)    */

static int32_t to_q31(float x) {
    if (x >=  1.0f) return (int32_t)0x7fffffff;
    if (x <= -1.0f) return (int32_t)0x80000000;
    float v = x * 2147483648.0f;           /* 2^31 */
    if (v >  2147483647.0f) v =  2147483647.0f;
    if (v < -2147483648.0f) v = -2147483648.0f;
    return (int32_t)v;
}

static void pack_frame(uint16_t *hw4, int32_t ch0, int32_t ch1) {
    hw4[0] = (uint16_t)((uint32_t)ch0 >> 16);
    hw4[1] = (uint16_t)((uint32_t)ch0 & 0xffffu);
    hw4[2] = (uint16_t)((uint32_t)ch1 >> 16);
    hw4[3] = (uint16_t)((uint32_t)ch1 & 0xffffu);
}

/* Synthesize one 128-frame AM I/Q bank at 100 kHz, starting at raw sample n0. */
static void gen_bank(uint32_t n0) {
    for (uint32_t f = 0; f < BANK; ++f) {
        float t  = (float)(n0 + f) / RAW_FS;
        float a  = 1.0f + MOD_DEPTH * cosf(2.0f * (float)M_PI * F_MOD * t);
        float ph = 2.0f * (float)M_PI * F_OFF * t;
        float i  = CARRIER_A * a * cosf(ph);
        float q  = CARRIER_A * a * sinf(ph);
        pack_frame(&s_rx_bank[4u * f], to_q31(i), to_q31(q));
    }
}

/* Upsample 50 -> 100 kHz by linear x2 interpolation into the codec TX-audio
 * bank (both channels). Returns the number of non-zero output frames. A proper
 * half-band interpolator can replace this; linear preserves the 1 kHz tone and
 * exercises the full output-bank wiring for the M2 graft proof. */
static uint32_t fill_tx_bank(const float *a, uint32_t n) {
    uint32_t nz = 0u;
    for (uint32_t k = 0; k < n; ++k) {
        float s0 = a[k];
        float s1 = (k + 1u < n) ? a[k + 1u] : a[k];
        float mid = 0.5f * (s0 + s1);
        int32_t w0 = to_q31(s0);
        int32_t w1 = to_q31(mid);
        pack_frame(&s_tx_bank[4u * (2u * k + 0u)], w0, w0);
        pack_frame(&s_tx_bank[4u * (2u * k + 1u)], w1, w1);
        if (w0 != 0) nz++;
        if (w1 != 0) nz++;
    }
    return nz;
}

int main(void) {
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== x6100_bb_fw M2 RX transport spike ===\r\n");
    bsp_uprintf("real sdr_rx.c + codec producer on F427\r\n");
    bsp_uprintf("in: AM I/Q bank +%d Hz, tone %d Hz, depth %d%%, raw fs %d Hz\r\n",
                (int)F_OFF, (int)F_MOD, (int)(MOD_DEPTH * 100.0f), (int)RAW_FS);
    bsp_uprintf("M=%u fftdec=%u block=%u -> audio %u @ %d Hz -> x2 -> %u @ %d Hz\r\n",
                (unsigned)Drv_Fpga_GetSamplesPerTick(),
                (unsigned)Drv_Fpga_GetFftdec(),
                (unsigned)(AUDIO_N * Drv_Fpga_GetSamplesPerTick()),
                (unsigned)AUDIO_N, (int)AUDIO_FS,
                (unsigned)BANK, (int)RAW_FS);

    /* Bring up the producer, then the RX DSP modules the AM path uses. */
    Drv_Fpga_Codec_Init();
    Sdr_Resample_Init();
    Sdr_AudioResample_Init();
    Sdr_AM_Init();
    Sdr_NB_Init();
    Sdr_Notch_Init();
    Sdr_NR_Init();
    Sdr_AGC_Init();
    Sdr_Squelch_Init();
    Sdr_Rx_Init();

    /* Select AM at the X6100 fftdec (2 -> M=2 -> 100 kHz source). */
    SdrRxCfg cfg = { SDR_MOD_AM, -4500, 4500, DRV_FPGA_CODEC_FFTDEC };
    Sdr_Rx_SetConfig(&cfg);

    uint32_t n = 0;
    uint32_t produced = 0u, tx_nonzero = 0u;

    for (uint32_t b = 0; b < WARMUP_BLOCKS; ++b) {
        gen_bank(n); n += BANK;
        Drv_Fpga_Codec_PushIqBank(s_rx_bank);
        (void)Sdr_Rx_Process(s_audio50, AUDIO_N);
    }

    double sumsq = 0.0, dc = 0.0;
    float  peak = 0.0f, prev_ac = 0.0f;
    uint32_t nsamp = 0u, sign_changes = 0u;

    for (uint32_t b = 0; b < MEAS_BLOCKS; ++b) {
        gen_bank(n); n += BANK;
        Drv_Fpga_Codec_PushIqBank(s_rx_bank);
        bool ok = Sdr_Rx_Process(s_audio50, AUDIO_N);
        if (ok) produced++;
        tx_nonzero += fill_tx_bank(s_audio50, AUDIO_N);

        for (uint32_t k = 0; k < AUDIO_N; ++k) {
            float v = s_audio50[k];
            dc += 0.001 * ((double)v - dc);          /* slow DC estimate   */
            float ac = v - (float)dc;                /* AC-coupled sample  */
            sumsq += (double)ac * (double)ac;
            float m = fabsf(ac);
            if (m > peak) peak = m;
            if (nsamp > 0u && ((ac >= 0.0f) != (prev_ac >= 0.0f))) sign_changes++;
            prev_ac = ac;
            nsamp++;
        }
    }

    float rms    = sqrtf((float)(sumsq / (double)nsamp));
    float meas_s = (float)nsamp / AUDIO_FS;
    float freq   = (float)sign_changes * 0.5f / meas_s;

    bsp_uprintf("out: %u/%u blocks produced, %u audio samples over %d ms\r\n",
                (unsigned)produced, (unsigned)MEAS_BLOCKS,
                (unsigned)nsamp, (int)(meas_s * 1000.0f));
    bsp_uprintf("     rms(ac)  = %ld (x1e3)\r\n", (long)(rms * 1000.0f + 0.5f));
    bsp_uprintf("     peak(ac) = %ld (x1e3)\r\n", (long)(peak * 1000.0f + 0.5f));
    bsp_uprintf("     freq ~ %ld Hz        expect ~%d\r\n",
                (long)(freq + 0.5f), (int)F_MOD);
    bsp_uprintf("     tx bank frames nonzero = %u / %u\r\n",
                (unsigned)tx_nonzero, (unsigned)(MEAS_BLOCKS * BANK));

    int ok_prod = (produced == MEAS_BLOCKS);
    int ok_rms  = (rms > 1e-4f && isfinite(rms));
    int ok_freq = (freq > 900.0f && freq < 1100.0f);
    int ok_tx   = (tx_nonzero > 0u);
    bsp_uprintf("RESULT: %s (produced %s, level %s, freq %s, tx %s)\r\n",
                (ok_prod && ok_rms && ok_freq && ok_tx) ? "PASS" : "FAIL",
                ok_prod ? "ok" : "BAD", ok_rms ? "ok" : "BAD",
                ok_freq ? "ok" : "BAD", ok_tx ? "ok" : "BAD");
    bsp_uprintf("DONE\r\n");

    for (;;) bsp_delay_ms(1000);
}
