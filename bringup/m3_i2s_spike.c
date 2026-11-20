/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M3 I2S integration spike: drive the REAL receive datapath + ISR dispatch.
 * ============================================================================
 * Where M2 called Drv_Fpga_Codec_PushIqBank() + Sdr_Rx_Process() directly, M3
 * exercises the actual Devices-layer glue the DMA1_Stream3 ISR runs: the strong
 * i2s_dsp() override (Platform/f427/i2s_rx_path.c), which performs
 *
 *     PushIqBank -> Sdr_Rx_Process (real sdr_rx.c) -> half-band x2 -> pack TX,
 *
 * and the ported transport's dispatcher (Platform/f427/i2s.c:
 * i2s_on_bank_complete), which selects the double-buffer banks and calls
 * i2s_dsp().
 *
 * Renode gaps (see renode/stm32f427.repl) and how the spike handles them:
 *  - No I2S slave + 4-stream DMA transport is modelled, so nothing fills
 *    SPI->DR. The spike supplies its own synthetic 128-frame I/Q banks.
 *  - The DMA double-buffer registers are only partially modelled: M0AR is
 *    retained, but M1AR reads back 0 and the current-target bit DMA_SxCR_CT is
 *    not settable (reads 0) -- both verified. With CT stuck at 0,
 *    i2s_select_bank() always picks M1AR (= null), so the bank *selection*
 *    inside i2s_on_bank_complete() cannot resolve real data here. That
 *    selection (two ported-verbatim lines) is therefore a hardware-only check.
 *
 * So the spike validates in two honest parts:
 *  1. Datapath: call the strong i2s_dsp() directly, once per synthetic bank
 *     (exactly what the ISR invokes after selecting banks), and measure the
 *     recovered tone from the TX-audio bank it writes. This is the real new M3
 *     code (i2s_dsp + the half-band interpolator + the unmodified RX chain).
 *  2. ISR dispatch: call i2s_on_bank_complete() once and confirm it runs and
 *     dispatches to the strong override (i2s_bank_count advances). The
 *     M0/M1/CT selection itself is left to hardware (Renode gap above).
 *
 * Input: the same synthetic AM I/Q as M2 -- complex carrier +500 Hz amplitude-
 * modulated by a 1 kHz tone at depth 0.5, 128 frames @ 100 kHz per bank, Q1.31,
 * ch0 = I, ch1 = Q. The recovered audio is a clean 1 kHz tone at the fixed
 * AGC-off gain.
 *
 * PASS: the ISR dispatcher runs and reaches the strong i2s_dsp (dispatch ok),
 * every bank is processed (i2s_bank_count advances 1:1), and the tone read back
 * from the TX-audio bank sits at ~1000 Hz with non-trivial level.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"

#include "bsp.h"
#include "drv_codec.h"
#include "drv_fpga_codec.h"
#include "i2s.h"
#include "i2s_rx_path.h"
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
#define TX_FS     100000.0f                 /* codec DAC / TX-audio rate     */
#define BANK      I2S_BANK_FRAMES           /* 128 complex frames per bank   */

#define F_OFF     500.0f    /* carrier offset from DC */
#define F_MOD     1000.0f   /* modulating tone        */
#define MOD_DEPTH 0.5f
#define CARRIER_A 2.5e-4f   /* small: chain runs at AGC-off +70 dB */

#define WARMUP_BLOCKS 96u
#define MEAS_BLOCKS   96u

/* One synthetic bank per direction. s_rx_iq / s_tx_audio carry the datapath
 * (direct i2s_dsp calls); the dispatch check points the DMA M0AR registers at
 * them (see stage_dma_banks). */
static uint16_t s_rx_iq[I2S_BANK_HALFWORDS]    __attribute__((aligned(4)));
static uint16_t s_rx_audio[I2S_BANK_HALFWORDS] __attribute__((aligned(4)));
static uint16_t s_tx_audio[I2S_BANK_HALFWORDS] __attribute__((aligned(4)));
static uint16_t s_tx_iq[I2S_BANK_HALFWORDS]    __attribute__((aligned(4)));

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

/* Synthesize one 128-frame AM I/Q bank at 100 kHz starting at raw sample n0. */
static void gen_bank(uint16_t *bank, uint32_t n0) {
    for (uint32_t f = 0; f < BANK; ++f) {
        float t  = (float)(n0 + f) / RAW_FS;
        float a  = 1.0f + MOD_DEPTH * cosf(2.0f * (float)M_PI * F_MOD * t);
        float ph = 2.0f * (float)M_PI * F_OFF * t;
        float i  = CARRIER_A * a * cosf(ph);
        float q  = CARRIER_A * a * sinf(ph);
        pack_frame(&bank[4u * f], to_q31(i), to_q31(q));
    }
}

/* Stage the four DMA1 stream M0AR registers at our synthetic banks for the ISR
 * dispatch check. The streams stay disabled (EN=0): the registers are read as
 * the bank contract, never driven as a live transfer. Only M0AR is reliably
 * modelled by Renode (M1AR/CT are not -- see the file banner), so this staging
 * backs the dispatch check only, not the datapath measurement. */
static void stage_dma_banks(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;   /* DMA1 register access */
    DMA1_Stream0->M0AR = (uint32_t)s_rx_iq;     /* SPI3 RX  (I/Q)   */
    DMA1_Stream3->M0AR = (uint32_t)s_rx_audio;  /* SPI2 RX  (audio) */
    DMA1_Stream4->M0AR = (uint32_t)s_tx_audio;  /* I2S2ext (audio)  */
    DMA1_Stream5->M0AR = (uint32_t)s_tx_iq;     /* I2S3ext (I/Q)    */
}

int main(void) {
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== x6100_bb_fw M3 I2S integration spike ===\r\n");
    bsp_uprintf("real i2s_on_bank_complete() -> i2s_dsp() on F427\r\n");
    bsp_uprintf("in: AM I/Q bank +%d Hz, tone %d Hz, depth %d%%, raw fs %d Hz\r\n",
                (int)F_OFF, (int)F_MOD, (int)(MOD_DEPTH * 100.0f), (int)RAW_FS);

    /* Bring up the producer, the RX DSP modules the AM path uses, and the
     * audio-out interpolator behind the strong i2s_dsp(). */
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
    I2s_Rx_Path_Init();

    SdrRxCfg cfg = { SDR_MOD_AM, -4500, 4500, DRV_FPGA_CODEC_FFTDEC };
    Sdr_Rx_SetConfig(&cfg);

    /* --- Part 2 (done first): ISR-dispatch check. The real dispatcher runs,
     * selects banks from the DMA registers, and must reach the strong i2s_dsp
     * override (bank_count advances). The M0/M1/CT selection resolving to real
     * data is a hardware-only check (Renode gap). */
    stage_dma_banks();
    uint32_t bc0 = i2s_bank_count;
    i2s_on_bank_complete();
    bool dispatch_ok = (i2s_bank_count == bc0 + 1u);

    /* --- Part 1: datapath. Call the strong i2s_dsp() directly with synthetic
     * banks (exactly what the ISR invokes post-selection) and measure the tone
     * from the TX-audio bank it writes. */
    uint32_t n = 0u;

    for (uint32_t b = 0; b < WARMUP_BLOCKS; ++b) {
        gen_bank(s_rx_iq, n); n += BANK;
        i2s_dsp(s_rx_audio, s_rx_iq, s_tx_audio, s_tx_iq);
    }

    double sumsq = 0.0, dc = 0.0;
    float  peak = 0.0f, prev_ac = 0.0f;
    uint32_t nsamp = 0u, sign_changes = 0u;
    uint32_t processed = 0u;
    uint32_t bc_start = i2s_bank_count;

    for (uint32_t b = 0; b < MEAS_BLOCKS; ++b) {
        gen_bank(s_rx_iq, n); n += BANK;
        uint32_t before = i2s_bank_count;
        i2s_dsp(s_rx_audio, s_rx_iq, s_tx_audio, s_tx_iq);
        if (i2s_bank_count == before + 1u) processed++;

        /* i2s_dsp wrote the 100 kHz TX-audio bank; read ch0 back to float. */
        for (uint32_t f = 0; f < BANK; ++f) {
            int32_t w = (int32_t)(((uint32_t)s_tx_audio[4u * f] << 16)
                                  | s_tx_audio[4u * f + 1u]);
            float v = (float)w / 2147483648.0f;
            dc += 0.001 * ((double)v - dc);
            float ac = v - (float)dc;
            sumsq += (double)ac * (double)ac;
            float m = fabsf(ac);
            if (m > peak) peak = m;
            if (nsamp > 0u && ((ac >= 0.0f) != (prev_ac >= 0.0f))) sign_changes++;
            prev_ac = ac;
            nsamp++;
        }
    }

    float rms    = sqrtf((float)(sumsq / (double)nsamp));
    float meas_s = (float)nsamp / TX_FS;
    float freq   = (float)sign_changes * 0.5f / meas_s;
    uint32_t banks_total = i2s_bank_count - bc_start;

    bsp_uprintf("dispatch: i2s_on_bank_complete -> i2s_dsp %s "
                "(M0/M1/CT select = hardware-only)\r\n",
                dispatch_ok ? "ok" : "BAD");
    bsp_uprintf("out: %u/%u banks processed, %u tx-audio samples over %d ms\r\n",
                (unsigned)processed, (unsigned)MEAS_BLOCKS,
                (unsigned)nsamp, (int)(meas_s * 1000.0f));
    bsp_uprintf("     rms(ac)  = %ld (x1e3)\r\n", (long)(rms * 1000.0f + 0.5f));
    bsp_uprintf("     peak(ac) = %ld (x1e3)\r\n", (long)(peak * 1000.0f + 0.5f));
    bsp_uprintf("     freq ~ %ld Hz        expect ~%d\r\n",
                (long)(freq + 0.5f), (int)F_MOD);

    int ok_disp = dispatch_ok && (banks_total == MEAS_BLOCKS);
    int ok_proc = (processed == MEAS_BLOCKS);
    int ok_rms  = (rms > 1e-4f && isfinite(rms));
    int ok_freq = (freq > 900.0f && freq < 1100.0f);
    bsp_uprintf("RESULT: %s (dispatch %s, processed %s, level %s, freq %s)\r\n",
                (ok_disp && ok_proc && ok_rms && ok_freq) ? "PASS" : "FAIL",
                ok_disp ? "ok" : "BAD", ok_proc ? "ok" : "BAD",
                ok_rms ? "ok" : "BAD", ok_freq ? "ok" : "BAD");
    bsp_uprintf("DONE\r\n");

    for (;;) bsp_delay_ms(1000);
}
