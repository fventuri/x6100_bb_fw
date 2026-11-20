/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3).
 *
 * X6100 (STM32F427) I2S sample transport -- ported from the author's
 * x6100-bbfw phase2b (common/i2s.{c,h}). Bare-metal / LL-inline (no HAL):
 * the transport is already CMSIS-register + LL-header-only code, so the port
 * is near-verbatim; only the banner changed. See i2s.c for the architecture
 * summary and x6100-base/docs/{spi_i2s,dma}.md for the recovered contract.
 * ======================================================================
 * Two full-duplex I2S slave pairs (external TLV320AIC3204 = clock master):
 *   SPI2/I2S2 + I2S2ext = AUDIO pair     SPI3/I2S3 + I2S3ext = I/Q pair
 * Receive blocks (SPI2/SPI3) are I2S slave receive; the ext blocks are the
 * complementary slave transmit. Four DMA1 streams carry the four directions
 * as 16-bit, MINC, circular double-buffers of 512 halfwords (128 stereo
 * frames of two 32-bit channels).
 *
 * The weak i2s_dsp() hook is the RX DSP override point: M3's strong override
 * (Platform/f427/i2s_rx_path.c) reads the completed I/Q bank and writes
 * demodulated audio into the TX-audio bank. The DSP-less default (reconstruct
 * RX, zero-fill TX) is kept for a transport-only bring-up.
 */
#ifndef X6100_I2S_H
#define X6100_I2S_H

#include <stdint.h>
#include "stm32f4xx.h"

#define I2S_BANK_HALFWORDS 512u          /* one DMA bank = 512 halfwords   */
#define I2S_BANK_FRAMES    128u          /* = 128 stereo frames (4 hw each) */

/* DSP raw/processing rate constants (spi_i2s.md FUN_08021744). Values are
 * authoritative; absolute rate is the external I2S clock. */
extern volatile float i2s_rate_raw;      /* 100000 : raw I/Q sample rate     */
extern volatile float i2s_rate_primary;  /*  12500 : primary DSP rate        */
extern volatile float i2s_rate_audio;    /*  12500 : audio/baseband rate     */
extern volatile float i2s_rate_alt;      /*  25000 : alternate DSP rate      */

/* Completed/selected bank pointers published by the Stream3 handler
 * (dma.md: 0x200003CC/D4 RX, 0x200003C8/D0 TX -- our own RAM here). */
extern volatile uint16_t *i2s_rx_audio;  /* completed SPI2/I2S2 RX bank  */
extern volatile uint16_t *i2s_rx_iq;     /* completed SPI3/I2S3 RX bank  */
extern volatile uint16_t *i2s_tx_audio;  /* selected  I2S2ext  TX bank   */
extern volatile uint16_t *i2s_tx_iq;     /* selected  I2S3ext  TX bank   */

extern volatile uint32_t i2s_bank_count; /* RX/TX banks serviced by the ISR */
extern volatile uint32_t i2s_isr_entries;/* times DMA1_Stream3_IRQHandler ran */

/* Last reconstructed audio/IQ frame (channel 0/1), for validation read-out. */
extern volatile int32_t i2s_last_audio[2];
extern volatile int32_t i2s_last_iq[2];

/* Configure AF pins, I2S peripherals (slave RX + ext slave TX) and the four
 * DMA1 streams. Leaves I2S disabled; call i2s_start() to go live. */
void i2s_config(void);

/* Enable the four DMA streams, then the I2S RX/TX blocks (startup.md order),
 * and the DMA1_Stream3 transfer-complete interrupt. */
void i2s_start(void);

/* Reconstruct one stereo frame (4 halfwords: Lhi,Llo,Rhi,Rlo) into two signed
 * 32-bit channels. Halfword order is high-then-low per 32-bit I2S word. */
void i2s_reconstruct_frame(const uint16_t *hw4, int32_t *ch0, int32_t *ch1);

/* Completed/selected double-buffer bank = the one NOT currently targeted.
 * After a transfer-complete the DMA has toggled CT to the new target, so the
 * finished bank is M0 when CT=1 and M1 when CT=0 (pure, for the self-test). */
static inline uint16_t *i2s_select_bank(int ct, uint16_t *m0, uint16_t *m1)
{
    return ct ? m0 : m1;
}

/* DSP-less bank service (pure): reconstruct both RX banks to signed-32 (keeping
 * the last frame), zero-fill both TX banks, bump the bank counter. */
void i2s_process_banks(const uint16_t *rx_audio, const uint16_t *rx_iq,
                       uint16_t *tx_audio, uint16_t *tx_iq);

/* Per-bank DSP hook invoked by the Stream3 ISR (weak). The default calls
 * i2s_process_banks() (DSP-less). The RX firmware overrides it with the
 * receive demodulator: read the completed I/Q bank (rx_iq), write demodulated
 * audio into the TX-audio bank (tx_audio -> I2S2ext -> codec DAC). It should
 * still bump i2s_bank_count. Runs in interrupt context every ~1.28 ms. */
void i2s_dsp(const uint16_t *rx_audio, const uint16_t *rx_iq,
             uint16_t *tx_audio, uint16_t *tx_iq);

/* The body of DMA1_Stream3_IRQHandler: read the live DMA current-target bits,
 * select the four banks, publish them, and run i2s_dsp(). */
void i2s_on_bank_complete(void);

#endif /* X6100_I2S_H */
