/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M1 App-layer spike: run upstream's real AM demodulator (App/SDR/sdr_am.c,
 * unmodified) on the F427 against CMSIS-DSP, fed by a synthetic baseband AM
 * signal, and confirm it recovers the modulating tone. This is the go/no-go
 * test for the whole "reuse the portable App layer" bet.
 *
 * Signal: complex baseband carrier at +500 Hz, amplitude-modulated by a 1 kHz
 * tone at depth m = 0.5, at the AM RX rate (48000/2 = 24000 Hz). The envelope
 * detector should recover 0.5*cos(2*pi*1000 t): RMS ~0.354, peak ~0.5, 1000 Hz.
 */
#include <math.h>
#include <stdint.h>

#include "bsp.h"
#include "sdr_am.h"
#include "sdr_common.h"

#define FS_HZ     24000.0f   /* SDR_AM_RX_RATE */
#define BLK       32u        /* SDR_AM_RX_BLOCK_SIZE = DRV_CODEC_BLOCK_FRAMES/2 */
#define F_OFF     500.0f     /* carrier offset from DC */
#define F_MOD     1000.0f    /* modulating tone */
#define MOD_DEPTH 0.5f

#define WARMUP_BLOCKS 96u
#define MEAS_BLOCKS   96u

static float s_i[BLK], s_q[BLK];
static float s_audio[BLK];

static void gen_block(uint32_t n0) {
    for (uint32_t k = 0; k < BLK; ++k) {
        float t = (float)(n0 + k) / FS_HZ;
        float a = 1.0f + MOD_DEPTH * cosf(2.0f * (float)M_PI * F_MOD * t);
        float ph = 2.0f * (float)M_PI * F_OFF * t;
        s_i[k] = a * cosf(ph);
        s_q[k] = a * sinf(ph);
    }
}

int main(void) {
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== x6100_bb_fw M1 AM-demod spike ===\r\n");
    bsp_uprintf("upstream App/SDR/sdr_am.c on F427 + CMSIS-DSP\r\n");
    bsp_uprintf("in: AM carrier +%d Hz, tone %d Hz, depth %d%%, fs %d Hz\r\n",
                (int)F_OFF, (int)F_MOD, (int)(MOD_DEPTH * 100.0f), (int)FS_HZ);

    Sdr_AM_Init();

    Sdr_IQ_Data data = { .i = s_i, .q = s_q, .size = BLK };
    uint32_t n = 0;

    for (uint32_t b = 0; b < WARMUP_BLOCKS; ++b) {
        gen_block(n); n += BLK;
        Sdr_AM_Demod(&data, s_audio);
    }

    double sumsq = 0.0;
    float peak = 0.0f;
    uint32_t nsamp = 0, sign_changes = 0;
    float prev = 0.0f;
    for (uint32_t b = 0; b < MEAS_BLOCKS; ++b) {
        gen_block(n); n += BLK;
        Sdr_AM_Demod(&data, s_audio);
        for (uint32_t k = 0; k < BLK; ++k) {
            float v = s_audio[k];
            sumsq += (double)v * (double)v;
            float a = fabsf(v);
            if (a > peak) peak = a;
            if (nsamp > 0 && ((v >= 0.0f) != (prev >= 0.0f))) sign_changes++;
            prev = v;
            nsamp++;
        }
    }

    float rms = sqrtf((float)(sumsq / (double)nsamp));
    /* freq = (cycles) / (time); cycles = sign_changes / 2 */
    float meas_s = (float)nsamp / FS_HZ;
    float freq = (float)sign_changes * 0.5f / meas_s;

    bsp_uprintf("out: samples %u over %d ms\r\n",
                (unsigned)nsamp, (int)(meas_s * 1000.0f));
    bsp_uprintf("     rms  = %ld (x1e3)   expect ~354\r\n", (long)(rms * 1000.0f + 0.5f));
    bsp_uprintf("     peak = %ld (x1e3)   expect ~500\r\n", (long)(peak * 1000.0f + 0.5f));
    bsp_uprintf("     freq ~ %ld Hz        expect ~%d\r\n",
                (long)(freq + 0.5f), (int)F_MOD);

    int ok_rms  = (rms  > 0.28f && rms  < 0.42f);
    int ok_peak = (peak > 0.40f && peak < 0.60f);
    int ok_freq = (freq > 900.0f && freq < 1100.0f);
    bsp_uprintf("RESULT: %s (rms %s, peak %s, freq %s)\r\n",
                (ok_rms && ok_peak && ok_freq) ? "PASS" : "FAIL",
                ok_rms ? "ok" : "BAD", ok_peak ? "ok" : "BAD", ok_freq ? "ok" : "BAD");
    bsp_uprintf("DONE\r\n");

    for (;;) bsp_delay_ms(1000);
}
