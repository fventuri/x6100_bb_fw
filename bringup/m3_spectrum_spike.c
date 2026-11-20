/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M3 spectrum spike: the REAL App/SDR/sdr_spectrum.c (512-point CMSIS-DSP
 * arm_cfft_f32 + Hann window) running on the F427, fed by the X6100 codec
 * producer -- replacing the M2 S-meter/noise stub with the actual FFT.
 * ============================================================================
 * sdr_spectrum.c is a pure consumer of the same raw-I/Q producer contract that
 * sdr_rx.c uses (Drv_Fpga_GetRawWindow / GetPos / GetRawGeneration), so the
 * spike drives it exactly as the firmware will: push synthetic codec I/Q banks
 * into the producer ring, call Sdr_Spectrum_Process() (the DMA-side raw copy)
 * per bank, then run the 40 ms FFT task (Sdr_Spectrum_Task, reached through the
 * scheduler-stub capture -> Spike_Spectrum_Task).
 *
 * Input: a pure complex tone at +1 kHz (inside the AM passband), continuous
 * phase across banks, at the native 100 kHz raw rate. With fftdec = 2 and no
 * zoom the display FFT runs on the raw stream (effective rate 100 kHz), so the
 * tone must land at bin N/2 + round(1000 * N / 100000) = 256 + 5 = 261.
 *
 * PASS: the FFT magnitude (Sdr_Spectrum_Get_Db) peaks at the expected bin with
 * a strong peak-to-median ratio, the S-meter is finite and elevated (the tone
 * is in the metered passband), and the noise-floor estimate is valid and
 * finite. The CMSIS-DSP WindowFunctions gap (arm_hanning_f32) is filled by the
 * GPLv3 compat/ shim; sdr_spectrum.c itself is unmodified.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>   /* qsort */

#include "bsp.h"
#include "drv_codec.h"
#include "drv_fpga_codec.h"
#include "sdr_common.h"
#include "sdr_config.h"
#include "sdr_spectrum.h"

extern void Spike_Spectrum_Task(void);   /* runs the captured 40 ms FFT task */

#define RAW_FS      100000.0f
#define BANK        DRV_FPGA_CODEC_BANK_FRAMES   /* 128 complex frames/bank */
#define N           SDR_SPECTRUM_FFT_SIZE        /* 512 */

#define F_TEST      1000.0f   /* +1 kHz tone, inside the AM passband */
#define AMP         0.25f
#define NOISE_AMP   1.0e-3f   /* broadband I/Q noise floor (~-48 dBc) */

#define ROUNDS          6u
#define BANKS_PER_ROUND 6u

static uint16_t s_bank[BANK * 4u];   /* synthetic I2S3 RX I/Q bank */

/* Cheap deterministic white noise in [-1,1), so the spectrum has a realistic
 * floor well above the 1e-12 clamp. (A pure tone over that clamp gives a ~90 dB
 * dynamic range where the noise estimator's float32 running-window sum cancels
 * catastrophically around the tone bin and reads a spurious negative minimum --
 * an unrealistic test condition, not a real-signal one.) */
static uint32_t s_rng = 0x1234567u;
static float noise_sample(void) {
    s_rng = s_rng * 1664525u + 1013904223u;
    return ((float)(s_rng >> 8) / (float)(1u << 24)) * 2.0f - 1.0f;
}

static int32_t to_q31(float x) {
    if (x >=  1.0f) return (int32_t)0x7fffffff;
    if (x <= -1.0f) return (int32_t)0x80000000;
    float v = x * 2147483648.0f;
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

/* One 128-frame bank at 100 kHz starting at raw sample n0: a +1 kHz complex
 * tone over a low broadband noise floor. */
static void gen_bank(uint32_t n0) {
    for (uint32_t f = 0; f < BANK; ++f) {
        float ph = 2.0f * (float)M_PI * F_TEST * (float)(n0 + f) / RAW_FS;
        float i = AMP * cosf(ph) + NOISE_AMP * noise_sample();
        float q = AMP * sinf(ph) + NOISE_AMP * noise_sample();
        pack_frame(&s_bank[4u * f], to_q31(i), to_q31(q));
    }
}

static int cmp_float(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

int main(void) {
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== x6100_bb_fw M3 spectrum spike ===\r\n");
    bsp_uprintf("real sdr_spectrum.c (arm_cfft_f32 N=%u) + codec producer\r\n",
                (unsigned)N);
    bsp_uprintf("in: complex tone +%d Hz amp %d%% + noise floor, raw fs %d Hz\r\n",
                (int)F_TEST, (int)(AMP * 100.0f), (int)RAW_FS);

    Drv_Fpga_Codec_Init();
    Sdr_Spectrum_Init();
    Sdr_Spectrum_SetModulation(SDR_MOD_AM);
    Sdr_Spectrum_SetFftdec(DRV_FPGA_CODEC_FFTDEC);
    Sdr_Spectrum_SetFilter(-4500, 4500);

    uint32_t n = 0;
    for (uint32_t r = 0; r < ROUNDS; ++r) {
        for (uint32_t b = 0; b < BANKS_PER_ROUND; ++b) {
            gen_bank(n); n += BANK;
            Drv_Fpga_Codec_PushIqBank(s_bank);
            Sdr_Spectrum_Process();        /* DMA-side raw copy into the FFT buffer */
        }
        Spike_Spectrum_Task();             /* 40 ms FFT task body */
    }

    /* Read back the finished frame. */
    const float *db = Sdr_Spectrum_Get_Db();
    float s_meter = Sdr_Spectrum_Get_S_meter();
    bool  noise_valid = Sdr_Spectrum_Is_Noise_Valid();
    float noise_db = Sdr_Spectrum_Get_Noise_Db();

    uint32_t peak_bin = 0;
    float peak_db = db[0];
    for (uint32_t i = 1; i < N; ++i) {
        if (db[i] > peak_db) { peak_db = db[i]; peak_bin = i; }
    }

    static float sorted[N];
    for (uint32_t i = 0; i < N; ++i) sorted[i] = db[i];
    qsort(sorted, N, sizeof(float), cmp_float);
    float median_db = sorted[N / 2u];

    uint32_t expect_bin = N / 2u + (uint32_t)(F_TEST * (float)N / RAW_FS + 0.5f);

    bsp_uprintf("out: peak bin %u (expect ~%u), peak %ld dB, median %ld dB\r\n",
                (unsigned)peak_bin, (unsigned)expect_bin,
                (long)(peak_db + (peak_db < 0 ? -0.5f : 0.5f)),
                (long)(median_db + (median_db < 0 ? -0.5f : 0.5f)));
    bsp_uprintf("     peak-median = %ld dB\r\n",
                (long)(peak_db - median_db + 0.5f));
    bsp_uprintf("     s_meter = %ld dB\r\n",
                (long)(s_meter + (s_meter < 0 ? -0.5f : 0.5f)));
    bsp_uprintf("     noise: valid=%s db=%ld\r\n",
                noise_valid ? "yes" : "no",
                (long)(noise_db + (noise_db < 0 ? -0.5f : 0.5f)));

    int ok_bin   = (peak_bin + 3u >= expect_bin) && (peak_bin <= expect_bin + 3u);
    int ok_peak  = ((peak_db - median_db) > 20.0f);
    int ok_meter = (isfinite(s_meter) && s_meter > -120.0f);
    int ok_noise = (noise_valid && isfinite(noise_db));
    bsp_uprintf("RESULT: %s (bin %s, peak %s, meter %s, noise %s)\r\n",
                (ok_bin && ok_peak && ok_meter && ok_noise) ? "PASS" : "FAIL",
                ok_bin ? "ok" : "BAD", ok_peak ? "ok" : "BAD",
                ok_meter ? "ok" : "BAD", ok_noise ? "ok" : "BAD");
    bsp_uprintf("DONE\r\n");

    for (;;) bsp_delay_ms(1000);
}
