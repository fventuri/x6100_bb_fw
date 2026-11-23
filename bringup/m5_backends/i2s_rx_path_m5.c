/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * i2s_rx_path (M5 bridge) -- the strong i2s_dsp() override for the m5_tx_host
 * build. Forked from Platform/f427/i2s_rx_path.c (M4). The ONLY change: instead
 * of calling Sdr_Rx_Process + packing inline, it drives app_radio.c's registered
 * codec block callback (radio_block_cb) -- the single RX+TX DSP dispatcher we are
 * adopting in M5. radio_block_cb pulls RX I/Q from the Drv_Fpga ring (fed by
 * PushIqBank here), runs the demod chain internally, and writes stereo Q1.31
 * audio at the 50 kHz base rate; this bridge then ×2 half-band-interpolates it to
 * the 100 kHz codec DAC bank, exactly as M4 did for its inline RX output.
 *
 * Volume is NOT applied here (app_radio scales the audio by the per-side DAC gain
 * inside radio_block_cb); g_rx_volume is kept defined for header/compat only.
 *
 * The callback is reached through m5_drv_codec_run_block() (the F427 Drv_Codec
 * backend in m5_tx_host.c), which invokes the registered cb when the codec is
 * started, or zero-fills when it is not -- so a pre-config / MODE_OFF radio stays
 * boot-silent.
 */
#include "i2s_rx_path.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "arm_math.h"

#include "audio_out_interp_filter.h"
#include "drv_codec.h"        /* DRV_CODEC_BLOCK_FRAMES (= 64 base-rate frames) */
#include "drv_fpga_codec.h"   /* Drv_Fpga_Codec_PushIqBank */
#include "i2s.h"              /* I2S_BANK_FRAMES, i2s_dsp(), i2s_bank_count (+ DWT) */

/* Reach the F427 Drv_Codec backend (m5_tx_host.c): run one base-rate block
 * through the registered app_radio callback. in/out are stereo interleaved L,R
 * Q1.31 (2*frames int32). Zero-fills out when the codec is not started. */
extern void m5_drv_codec_run_block(const int32_t *in, int32_t *out, unsigned frames);

volatile uint32_t g_i2s_dsp_cyc;
volatile uint32_t g_i2s_dsp_cyc_max;
volatile float    g_rx_volume = 0.0f;   /* unused in the bridge; app_radio owns level */

#define AUDIO_IN_FRAMES   DRV_CODEC_BLOCK_FRAMES          /* 64  @ 50 kHz  */
#define AUDIO_OUT_FRAMES  I2S_BANK_FRAMES                 /* 128 @ 100 kHz */

_Static_assert(AUDIO_OUT_FRAMES == AUDIO_IN_FRAMES * AUDIO_OUT_INTERP_50_100_FACTOR,
               "x2 interpolation must map one RX block to one I2S bank");

#define INTERP_PHASE_LEN  (AUDIO_OUT_INTERP_50_100_NUM_TAPS / AUDIO_OUT_INTERP_50_100_FACTOR)
#define INTERP_STATE_LEN  (AUDIO_IN_FRAMES + INTERP_PHASE_LEN - 1u)

static arm_fir_interpolate_instance_f32 s_interp;
static float s_interp_state[INTERP_STATE_LEN] __attribute__((aligned(32)));

/* Scratch (file-static: the hook runs in ISR context). */
static int32_t s_in_stereo[2u * AUDIO_IN_FRAMES];    /* mic in (RX: zeros)        */
static int32_t s_out_stereo[2u * AUDIO_IN_FRAMES];   /* app_radio audio (L,R Q31) */
static float   s_audio50[AUDIO_IN_FRAMES];
static float   s_audio100[AUDIO_OUT_FRAMES];

#define Q31_TO_F32  (1.0f / 2147483648.0f)

static inline int32_t to_q31(float x)
{
    if (x >=  1.0f) return (int32_t)0x7fffffff;
    if (x <= -1.0f) return (int32_t)0x80000000;
    float v = x * 2147483648.0f;
    if (v >  2147483647.0f) v =  2147483647.0f;
    if (v < -2147483648.0f) v = -2147483648.0f;
    return (int32_t)v;
}

static inline void pack_mono_frame(uint16_t *hw4, int32_t w)
{
    uint16_t hi = (uint16_t)((uint32_t)w >> 16);
    uint16_t lo = (uint16_t)((uint32_t)w & 0xffffu);
    hw4[0] = hi; hw4[1] = lo;   /* ch0 (left)  */
    hw4[2] = hi; hw4[3] = lo;   /* ch1 (right) */
}

void I2s_Rx_Path_Init(void)
{
    (void)arm_fir_interpolate_init_f32(&s_interp,
                                       AUDIO_OUT_INTERP_50_100_FACTOR,
                                       AUDIO_OUT_INTERP_50_100_NUM_TAPS,
                                       audio_out_interp_50_100_coeffs,
                                       s_interp_state,
                                       AUDIO_IN_FRAMES);
    memset(s_in_stereo, 0, sizeof s_in_stereo);
}

/* Strong override of i2s.c's weak i2s_dsp(). Runs in the DMA1_Stream3 ISR. */
void i2s_dsp(const uint16_t *rx_audio, const uint16_t *rx_iq,
             uint16_t *tx_audio, uint16_t *tx_iq)
{
    (void)rx_audio;   /* codec mic/line-in: used for TX (M5c); RX ignores it      */
    (void)tx_iq;      /* TX I/Q: Drv_Fpga_TxWrite discards (no RF) in M5-0         */

    uint32_t t0 = DWT->CYCCNT;

    if (rx_iq != NULL && tx_audio != NULL) {
        /* 1. Codec I/Q bank -> driver raw ring (app_radio's Sdr_Rx pulls it). */
        Drv_Fpga_Codec_PushIqBank(rx_iq);

        /* 2. Dispatch one base-rate block through app_radio's codec callback.
         *    RX: it runs the demod chain and fills s_out_stereo (L,R Q31 @50k).
         *    radio_block_cb also runs Sdr_Spectrum_Process() itself (S-meter /
         *    noise / waterfall), even in MODE_OFF -- so the bridge must NOT call
         *    it again here (one bank = one spectrum Process). Zero out first so
         *    MODE_OFF / un-started -> silence. */
        memset(s_out_stereo, 0, sizeof s_out_stereo);
        m5_drv_codec_run_block(s_in_stereo, s_out_stereo, AUDIO_IN_FRAMES);

        /* 3. Left channel -> float, ×2 half-band interp 50 -> 100 kHz (state
         *    carried across banks). Mono radio: L drives both DAC channels. */
        for (uint32_t f = 0; f < AUDIO_IN_FRAMES; ++f) {
            s_audio50[f] = (float)s_out_stereo[2u * f] * Q31_TO_F32;
        }
        arm_fir_interpolate_f32(&s_interp, s_audio50, s_audio100, AUDIO_IN_FRAMES);

        /* 4. Pack into the codec TX-audio bank (both channels, Q1.31). */
        for (uint32_t f = 0; f < AUDIO_OUT_FRAMES; ++f) {
            pack_mono_frame(&tx_audio[4u * f], to_q31(s_audio100[f]));
        }
    }

    uint32_t dt = DWT->CYCCNT - t0;
    g_i2s_dsp_cyc = dt;
    if (dt > g_i2s_dsp_cyc_max) g_i2s_dsp_cyc_max = dt;

    i2s_bank_count++;
}
