/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * i2s_rx_path -- X6100 RX-path integration: the strong i2s_dsp() override.
 * See i2s_rx_path.h for the per-bank flow.
 */
#include "i2s_rx_path.h"

#include <stddef.h>
#include <stdint.h>

#include "arm_math.h"

#include "audio_out_interp_filter.h"
#include "drv_codec.h"        /* DRV_CODEC_BLOCK_FRAMES (= 64 base-rate frames) */
#include "drv_fpga_codec.h"   /* Drv_Fpga_Codec_PushIqBank */
#include "i2s.h"              /* I2S_BANK_FRAMES, i2s_dsp(), i2s_bank_count (+ DWT) */
#include "sdr_rx.h"           /* Sdr_Rx_Process */
#include "sdr_spectrum.h"     /* Sdr_Spectrum_Process (S-meter/noise/waterfall) */

/* Per-bank i2s_dsp() cost on the DWT cycle counter, for the real-time CPU-budget
 * gauge (the whole Georgy RX pipeline runs in the DMA ISR). Last and peak; the
 * reader clears the peak. Zero until the app enables DWT->CYCCNT. */
volatile uint32_t g_i2s_dsp_cyc;
volatile uint32_t g_i2s_dsp_cyc_max;

/* Output volume: linear scale on the demodulated audio before it is packed into
 * the codec TX bank. The demod chain (with its AGC) produced full-scale-ish
 * audio before packing. STANDING RULE (Franco): every image MUST boot silent,
 * so the default is 0.0 -- no surprise loud audio out of a fresh flash. The
 * operator sets the level deliberately at runtime via the I2C3 VOLUME command
 * (comfortable AM bench level was ~0.005..0.05). */
volatile float g_rx_volume = 0.0f;

/* One completed I2S bank = one RX block. Sdr_Rx_Process() emits
 * DRV_CODEC_BLOCK_FRAMES (64) mono samples at the 50 kHz base rate; the x2
 * interpolator turns them into I2S_BANK_FRAMES (128) samples at 100 kHz. */
#define AUDIO_IN_FRAMES   DRV_CODEC_BLOCK_FRAMES          /* 64  @ 50 kHz  */
#define AUDIO_OUT_FRAMES  I2S_BANK_FRAMES                 /* 128 @ 100 kHz */

_Static_assert(AUDIO_OUT_FRAMES == AUDIO_IN_FRAMES * AUDIO_OUT_INTERP_50_100_FACTOR,
               "x2 interpolation must map one RX block to one I2S bank");

/* arm_fir_interpolate_f32 state: blockSize + (numTaps/L) - 1 floats. */
#define INTERP_PHASE_LEN  (AUDIO_OUT_INTERP_50_100_NUM_TAPS / AUDIO_OUT_INTERP_50_100_FACTOR)
#define INTERP_STATE_LEN  (AUDIO_IN_FRAMES + INTERP_PHASE_LEN - 1u)

static arm_fir_interpolate_instance_f32 s_interp;
static float s_interp_state[INTERP_STATE_LEN] __attribute__((aligned(32)));

/* Scratch audio buffers. File-static (not on the ISR stack): the hook runs in
 * interrupt context, so keeping these out of the stack bounds ISR stack use. */
static float s_audio50[AUDIO_IN_FRAMES];
static float s_audio100[AUDIO_OUT_FRAMES];

static inline int32_t to_q31(float x)
{
    if (x >=  1.0f) return (int32_t)0x7fffffff;
    if (x <= -1.0f) return (int32_t)0x80000000;
    float v = x * 2147483648.0f;              /* 2^31 */
    if (v >  2147483647.0f) v =  2147483647.0f;
    if (v < -2147483648.0f) v = -2147483648.0f;
    return (int32_t)v;
}

/* Pack one signed-32 sample into both channels of an I2S TX frame (4 halfwords:
 * ch0 hi, ch0 lo, ch1 hi, ch1 lo). Mono audio -> L == R. Inverse of
 * i2s_reconstruct_frame(). */
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
}

/* Strong override of i2s.c's weak i2s_dsp(). Runs in the DMA1_Stream3 ISR. */
void i2s_dsp(const uint16_t *rx_audio, const uint16_t *rx_iq,
             uint16_t *tx_audio, uint16_t *tx_iq)
{
    (void)rx_audio;   /* mic/line-in RX audio: unused on the RX path */
    (void)tx_iq;      /* TX I/Q: driven by the (future) transmit path */

    uint32_t t0 = DWT->CYCCNT;

    /* Defensive: a bank-complete can fire before the DMA bank addresses are
     * valid. Skip processing on a null I/Q source or TX sink (no deref), but
     * still count the serviced bank, as the DSP-less default does. */
    if (rx_iq != NULL && tx_audio != NULL) {
        /* 1. Codec I/Q bank -> driver raw ring. */
        Drv_Fpga_Codec_PushIqBank(rx_iq);

        /* 2. Spectrum: copy new raw I/Q into the FFT accumulation buffer (the
         *    S-meter / noise estimate / waterfall; the 40 ms FFT task consumes
         *    it in task context). Reads the same ring PushIqBank just filled. */
        Sdr_Spectrum_Process();

        /* 3. Real RX DSP: ring -> decimate -> demod -> audio resample -> 50 kHz. */
        (void)Sdr_Rx_Process(s_audio50, AUDIO_IN_FRAMES);

        /* 4. Proper half-band x2 interpolation 50 -> 100 kHz (carries state
         *    across banks via s_interp_state; no block-edge discontinuity). */
        arm_fir_interpolate_f32(&s_interp, s_audio50, s_audio100, AUDIO_IN_FRAMES);

        /* 5. Apply the output volume and pack into the codec TX-audio bank
         *    (both channels, Q1.31). */
        float vol = g_rx_volume;
        for (uint32_t f = 0; f < AUDIO_OUT_FRAMES; ++f) {
            pack_mono_frame(&tx_audio[4u * f], to_q31(s_audio100[f] * vol));
        }
    }

    uint32_t dt = DWT->CYCCNT - t0;
    g_i2s_dsp_cyc = dt;
    if (dt > g_i2s_dsp_cyc_max) g_i2s_dsp_cyc_max = dt;

    i2s_bank_count++;
}
