/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * M3 RX radio: first flashable integrated receiver on the X6100 hardware.
 * ============================================================================
 * The proven X6100 RX hardware bring-up (ported from the author's x6100-bbfw
 * phase3-rx: VersaClock 5P49V60 OUT0=26 MHz codec MCLK + OUT1=4xFtuned LO,
 * TLV320AIC3204 58-step codec startup, I/Q converter, RF band switching +
 * BAND_V + front-end enable) feeding the *ported Georgy pipeline* rather than
 * phase3's own rx_dsp.c:
 *
 *     codec I/Q (I2S3, 100 kHz) -> DMA1_Stream3 ISR -> i2s_dsp() [i2s_rx_path.c]
 *       -> Drv_Fpga_Codec_PushIqBank -> Sdr_Spectrum_Process
 *       -> Sdr_Rx_Process (real sdr_rx.c: decimate/demod/resample -> 50 kHz)
 *       -> half-band x2 -> codec DAC (I2S2ext, 100 kHz) -> HP/speaker.
 *
 * So this exercises M1-M3 (CORDIC, decimation, AM demod, resamplers, the
 * half-band audio-out, the real sdr_spectrum FFT) end to end on the radio.
 *
 * Default: AM, 1410 kHz (WDOV), audible out of reset (HP unmuted, no VOLUME
 * gate). Live retune / probe over the I2C3 control slave (0x72), hardware
 * commands kept from phase3-rx as the no-scope bring-up lifeline.
 *
 * SAFETY: receive-only. PA tuple stays disabled, charger off, TX I/Q pair
 * (I2S3ext) zero-filled. No path asserts TX.
 *
 * This is bench-validated only: Renode models neither the codec, the VersaClock,
 * nor the externally-clocked I2S transport. The hardware bring-up is a faithful
 * transcription of the known-good phase3-rx firmware (same bsp/i2c/i2s surface);
 * the DSP is our Renode-validated M1-M3 chain. CPU budget of the (heavier)
 * pipeline in the per-bank ISR is reported live (cpu_max) -- the one thing that
 * can only be confirmed on hardware.
 */
#include <stdint.h>

#include "bsp.h"
#include "bsp_time.h"
#include "i2c.h"
#include "i2s.h"
#include "i2s_rx_path.h"

#include "app_event.h"
#include "app_scheduler.h"
#include "drv_fpga_codec.h"
#include "drv_codec.h"
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
#include "sdr_spectrum.h"
#include "sdr_squelch.h"

#define FW_VERSION "x6100_bb_fw m3-rx-radio 2026-11-19"

#define RX_TUNED_HZ   1410000u
#define AM_LO_OFFSET  (-100)

#define ADDR_CODEC      0x18u
#define ADDR_CS4272     0x10u
#define ADDR_AUX4A      0x4Au
#define ADDR_AUX4C      0x4Cu
#define ADDR_VERSACLOCK 0x6Au

/* ======================================================================== *
 *  VersaClock 5P49V60 (I2C1 0x6A): OUT0 = 26 MHz codec MCLK, OUT1 = 4xFtuned
 *  LO. Without OUT0 the codec PLL never locks and the I2S banks never advance.
 *  Ported verbatim from phase3-rx (public-spec reimplementation). PE7 enable.
 * ======================================================================== */
static const uint8_t VC_CFG[] = {
    5, 0x00, 0x61, 0x03, 0xFF, 0x00,
    2, 0x0B, 0xC0,
    3, 0x10, 0xA0, 0xCC,
    2, 0x15, 0x0A,
    3, 0x17, 0x06, 0xF0,
    5, 0x1C, 0x9F, 0xFF, 0xE8, 0x80,
    5, 0x22, 0x01, 0x1F, 0x09, 0x28,
    2, 0x2A, 0x04,
    3, 0x2D, 0x01, 0x90,
    2, 0x30, 0x80,
    2, 0x3A, 0x04,
    4, 0x3F, 0x39, 0x80, 0x00,
    2, 0x4A, 0x04,
    2, 0x4F, 0x38,
    3, 0x51, 0x81, 0x02,
    2, 0x5A, 0x04,
    14, 0x5D, 0x2D, 0x10, 0x20,
        0x38, 0x05, 0xBB, 0x04, 0xBB, 0x04, 0x38, 0x05, 0xDF, 0x4C,
    0
};

static void vc_write_seq(const uint8_t *seq)
{
    while (*seq) { uint8_t n = *seq++; i2c_write_raw(I2C1, ADDR_VERSACLOCK, seq, n); seq += n; }
}

static int versaclock_init(void)
{
    static const uint8_t k43[] = {0x76, 0x43}, k63[] = {0x76, 0x63};
    SET_LO(GPIOE, 7); bsp_delay_ms(100);
    SET_HI(GPIOE, 7); bsp_delay_ms(100);
    int present = i2c_probe(I2C1, ADDR_VERSACLOCK);
    vc_write_seq(VC_CFG);                         bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k43, 2); bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k63, 2); bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k43, 2); bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k63, 2); bsp_delay_ms(10);
    return present;
}

#define VC_FVCO_DIV2_HZ  1443000000.0   /* (26 MHz * 111) / 2 */

static void vc_set_out1_hz(uint32_t fout_hz)
{
    double n = VC_FVCO_DIV2_HZ / (double)fout_hz;
    uint32_t intdiv = (uint32_t)n;
    double frac = n - (double)intdiv;
    uint32_t off = (uint32_t)(frac * 16777216.0 + 0.5);

    uint8_t ig[3] = { 0x2D, (uint8_t)((intdiv >> 4) & 0xFF), (uint8_t)((intdiv & 0x0F) << 4) };
    uint8_t fg[5] = { 0x22, (uint8_t)((off >> 22) & 0xFF), (uint8_t)((off >> 14) & 0xFF),
                      (uint8_t)((off >> 6) & 0xFF), (uint8_t)((off & 0x3F) << 2) };
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, ig, sizeof ig);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, fg, sizeof fg);
}

/* ======================================================================== *
 *  Internal DAC: DAC2/PA5 = ACC BAND_V staircase (frequency-dependent).
 * ======================================================================== */
static void dac_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
    DAC->DHR12R1 = 0u;
    DAC->DHR12R2 = 0u;
    DAC->CR = DAC_CR_EN1 | DAC_CR_EN2;
}

static uint16_t band_v_code(uint32_t hz)
{
    if (hz <=  2000000u) return 0x11Du;
    if (hz <=  5000000u) return 0x23Au;
    if (hz <=  7000000u) return 0x358u;
    if (hz <= 10000000u) return 0x475u;
    if (hz <= 14000000u) return 0x593u;
    if (hz <= 18000000u) return 0x6B0u;
    if (hz <= 21000000u) return 0x7CDu;
    if (hz <= 24000000u) return 0x8EBu;
    if (hz <= 28000000u) return 0xA08u;
    if (hz <= 50000000u) return 0xB26u;
    return 0xC43u;
}

/* ======================================================================== *
 *  Codec / converter startup (TLV320AIC3204 @ 0x18 on I2C2). Verbatim from
 *  phase3-rx (OEM 3+58-step sequence + AVDD LDO + HP unmute + 0x4A companion +
 *  CS4272/0x4C I/Q converter). Codec is the I2S clock master.
 * ======================================================================== */
static int cwr(uint8_t addr, const uint8_t *buf, uint32_t n)
{
    return i2c_write_raw(I2C2, addr, buf, n);   /* all these live on I2C2 */
}

static void tlv320_clock_100k(void)
{
    static const uint8_t page0[]     = {0x00, 0x00};
    static const uint8_t pll[]       = {0x04, 0x40, 0xC1, 0x0F, 0x1D, 0x08, 0x74};
    static const uint8_t dosr[]      = {0x0D, 0x00, 0x80};
    static const uint8_t aosr[]      = {0x14, 0x80};
    static const uint8_t ndac_mdac[] = {0x0B, 0x82, 0x84};
    static const uint8_t nadc_madc[] = {0x12, 0x02, 0x04};
    cwr(ADDR_CODEC, page0, sizeof page0);
    cwr(ADDR_CODEC, pll, sizeof pll);
    cwr(ADDR_CODEC, dosr, sizeof dosr);
    cwr(ADDR_CODEC, aosr, sizeof aosr);
    cwr(ADDR_CODEC, ndac_mdac, sizeof ndac_mdac);
    cwr(ADDR_CODEC, nadc_madc, sizeof nadc_madc);
}

static const uint8_t tlv_pre[][2] = {
    {0x04, 0x43}, {0x1D, 0x00}, {0x1E, 0x88}, {0x1B, 0x3C},
};
static const uint8_t tlv_post[][2] = {
    {0x00, 0x00}, {0x19, 0x06}, {0x1A, 0x82}, {0x37, 0x06}, {0x3C, 0x19}, {0x3D, 0x01},
    {0x00, 0x01}, {0x01, 0x08}, {0x0A, 0x03}, {0x3D, 0x00}, {0x03, 0x00}, {0x04, 0x00},
    {0x47, 0x32}, {0x7B, 0x05},
    {0x00, 0x00}, {0x51, 0xC0}, {0x52, 0x00}, {0x53, 0x00}, {0x54, 0x00},
    {0x00, 0x01}, {0x09, 0x3F}, {0x0C, 0x08}, {0x0D, 0x08}, {0x0E, 0x08}, {0x0F, 0x08},
    {0x10, 0x40}, {0x11, 0x40}, {0x12, 0x40}, {0x13, 0x40}, {0x14, 0x25},
    {0x00, 0x00}, {0x3F, 0xD6}, {0x40, 0x00}, {0x41, 0x00}, {0x42, 0x00},
    {0x00, 0x00}, {0x53, 0x6C}, {0x00, 0x00}, {0x54, 0x00}, {0x00, 0x00}, {0x41, 0x00},
    {0x00, 0x00}, {0x42, 0x00},
    {0x00, 0x01}, {0x34, 0x10}, {0x36, 0x40}, {0x00, 0x01}, {0x37, 0x14}, {0x39, 0x80},
};

static void tlv320_startup(void)
{
    static const uint8_t page0[] = {0x00, 0x00};
    static const uint8_t reset[] = {0x01, 0x01};
    cwr(ADDR_CODEC, page0, sizeof page0);
    cwr(ADDR_CODEC, reset, sizeof reset);
    bsp_delay_ms(1);
    {
        static const uint8_t ldo_p1[] = {0x00, 0x01};
        static const uint8_t ldo_on[] = {0x02, 0x01};
        static const uint8_t ldo_p0[] = {0x00, 0x00};
        cwr(ADDR_CODEC, ldo_p1, sizeof ldo_p1);
        cwr(ADDR_CODEC, ldo_on, sizeof ldo_on);
        cwr(ADDR_CODEC, ldo_p0, sizeof ldo_p0);
        bsp_delay_ms(10);
    }
    tlv320_clock_100k();
    for (unsigned i = 0; i < sizeof tlv_pre / 2; i++)  cwr(ADDR_CODEC, tlv_pre[i], 2);
    bsp_delay_ms(10);
    for (unsigned i = 0; i < sizeof tlv_post / 2; i++) cwr(ADDR_CODEC, tlv_post[i], 2);
}

static void codec_unmute_hp(void)
{
    static const uint8_t p1[]  = {0x00, 0x01};
    static const uint8_t hpl[] = {0x10, 0x09};   /* unmute, +9 dB */
    static const uint8_t hpr[] = {0x11, 0x09};
    cwr(ADDR_CODEC, p1, sizeof p1);
    cwr(ADDR_CODEC, hpl, sizeof hpl);
    cwr(ADDR_CODEC, hpr, sizeof hpr);
}

static void aux4a_startup(void)
{
    static const uint8_t w1[] = {0x00, 0x00};
    static const uint8_t w2[] = {0x20, 0x01};
    static const uint8_t w3[] = {0x00, 0x00, 0x80, 0x3F, 0x00};
    static const uint8_t w4[] = {0x0B, 0x00};
    static const uint8_t w5[] = {0x0D, 0x01};
    static const uint8_t w6[] = {0x0E, 0x01};
    cwr(ADDR_AUX4A, w1, sizeof w1);
    cwr(ADDR_AUX4A, w2, sizeof w2);
    cwr(ADDR_AUX4A, w3, sizeof w3);
    cwr(ADDR_AUX4A, w4, sizeof w4);
    bsp_delay_ms(1);
    cwr(ADDR_AUX4A, w5, sizeof w5);
    bsp_delay_ms(1);
    cwr(ADDR_AUX4A, w6, sizeof w6);
}

static int iq_converter_startup(void)
{
    static const uint8_t probe[] = {0x07, 0x03};
    if (i2c_write_raw(I2C2, ADDR_CS4272, probe, sizeof probe) == 0) {
        static const uint8_t seq[][2] = {
            {0x01, 0x81}, {0x02, 0x00}, {0x03, 0x49}, {0x04, 0x00},
            {0x05, 0x00}, {0x06, 0x10}, {0x07, 0x02},
        };
        for (unsigned i = 0; i < sizeof seq / 2; i++) cwr(ADDR_CS4272, seq[i], 2);
        return 1;
    }
    bsp_delay_ms(10);
    static const uint8_t f1[] = {0x10, 0x00};
    static const uint8_t f2[] = {0x10, 0xC2};
    static const uint8_t f3[] = {0x11, 0x80};
    static const uint8_t f4[] = {0x14, 0x00};
    cwr(ADDR_AUX4C, f1, sizeof f1); bsp_delay_ms(10);
    cwr(ADDR_AUX4C, f2, sizeof f2);
    cwr(ADDR_AUX4C, f3, sizeof f3);
    cwr(ADDR_AUX4C, f4, sizeof f4);
    return 0;
}

static void codec_reset_pulse(void)
{
    SET_LO(GPIOE, 15); bsp_delay_ms(1);
    SET_HI(GPIOE, 15); bsp_delay_ms(1);
}

/* ======================================================================== *
 *  RF band switching (direct-GPIO primary + 24-bit SPI6 image secondary) and
 *  the 6-bit cf88 serial chain. Verbatim from phase3-rx.
 * ======================================================================== */
static int rf_primary_index(uint32_t hz)
{
    if (hz < 1500000u || hz > 55000000u) return 0;
    if (hz <= 2200000u)  return 1;
    if (hz <= 3800000u)  return 2;
    if (hz <= 5500000u)  return 3;
    if (hz <= 7500000u)  return 4;
    if (hz <= 13500000u) return 5;
    if (hz <= 21000000u) return 6;
    if (hz <= 33000000u) return 7;
    if (hz <= 50000000u) return 8;
    return 9;
}

static int rf_secondary_index(uint32_t hz)
{
    if (hz <= 2000000u)  return 0;
    if (hz <= 4000000u)  return 1;
    if (hz <= 8000000u)  return 2;
    if (hz <= 15000000u) return 3;
    if (hz <= 22000000u) return 4;
    if (hz <= 30000000u) return 5;
    return 6;
}

static void rf_apply_primary(int idx)
{
    (idx & 1) ? SET_HI(GPIOD, 9)  : SET_LO(GPIOD, 9);
    (idx & 2) ? SET_HI(GPIOD, 10) : SET_LO(GPIOD, 10);
    (idx & 4) ? SET_HI(GPIOD, 11) : SET_LO(GPIOD, 11);
    (idx & 8) ? SET_HI(GPIOD, 13) : SET_LO(GPIOD, 13);
}

static uint32_t rf_image;

static void spi6_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI6);
    M_OUT(GPIOG, 12); M_PP(GPIOG, 12); SPD_HI(GPIOG, 12); SET_LO(GPIOG, 12);
    M_AF(GPIOG, 13); SPD_HI(GPIOG, 13); LL_GPIO_SetAFPin_8_15(GPIOG, LL_GPIO_PIN_13, LL_GPIO_AF_5);
    M_AF(GPIOG, 14); SPD_HI(GPIOG, 14); LL_GPIO_SetAFPin_8_15(GPIOG, LL_GPIO_PIN_14, LL_GPIO_AF_5);
    SPI6->CR1 = 0;
    SPI6->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | (7u << SPI_CR1_BR_Pos) | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
    SPI6->CR1 |= SPI_CR1_SPE;
}

/* Bounded waits: the OEM spins forever on TXE/BSY. A stuck SPI6 should not wedge
 * the whole radio, and the timeout also lets this run under emulators that do
 * not model SPI6 (an F42x-only peripheral). ~1e6 iterations >> one byte time. */
static void spi6_send_byte(uint8_t b)
{
    for (uint32_t t = 0; !(SPI6->SR & SPI_SR_TXE) && t < 1000000u; t++) ;
    *(volatile uint8_t *)&SPI6->DR = b;
}

static void rf_commit_image(uint32_t img)
{
    SET_HI(GPIOG, 12);
    spi6_send_byte((uint8_t)(img >> 16));
    spi6_send_byte((uint8_t)(img >> 8));
    spi6_send_byte((uint8_t)(img));
    for (uint32_t t = 0; (SPI6->SR & SPI_SR_BSY) && t < 1000000u; t++) ;
    SET_LO(GPIOG, 12);
}

static void rf_switch_tune(uint32_t hz)
{
    int pidx = rf_primary_index(hz);
    int sidx = rf_secondary_index(hz);
    int sbit = 13 - sidx;
    rf_apply_primary(pidx);
    rf_image = (rf_image & ~(0x3F80u)) | (1u << sbit);
    rf_commit_image(rf_image);
}

static inline void chain_delay(void) { for (volatile int i = 0; i < 8; i++) ; }

static void rf_chain6(uint8_t word)
{
    for (int bit = 5; bit >= 0; bit--) {
        if ((word >> bit) & 1u) SET_HI(GPIOC, 5); else SET_LO(GPIOC, 5);
        chain_delay();
        SET_HI(GPIOB, 0); chain_delay();
        SET_LO(GPIOB, 0); chain_delay();
    }
    SET_HI(GPIOB, 1); chain_delay();
    SET_LO(GPIOB, 1);
}

/* ======================================================================== *
 *  Mode + tune. apply_tune programs LO/RF/BAND_V; apply_mode configures the
 *  ported RX DSP + spectrum (and the AM LO offset).
 * ======================================================================== */
static volatile int32_t rx_lo_offset_hz = AM_LO_OFFSET;
static uint32_t rx_cur_station_hz = RX_TUNED_HZ;
static int      rx_mode = SDR_MOD_AM;

static void apply_tune(uint32_t station_hz)
{
    rx_cur_station_hz = station_hz;
    uint32_t ftuned = (uint32_t)((int32_t)station_hz + rx_lo_offset_hz);
    rf_chain6(0x0Eu);
    vc_set_out1_hz(4u * ftuned);
    rf_switch_tune(ftuned);
    DAC->DHR12R2 = band_v_code(station_hz);
}

static void apply_mode(int mod)
{
    int32_t lo = 0, hi = 0;
    switch (mod) {
    case SDR_MOD_LSB: lo = -3000; hi = -100;  rx_lo_offset_hz = 0; break;
    case SDR_MOD_USB: lo =   100; hi =  3000; rx_lo_offset_hz = 0; break;
    case SDR_MOD_AM:
    default:          mod = SDR_MOD_AM; lo = -4500; hi = 4500; rx_lo_offset_hz = AM_LO_OFFSET; break;
    }
    rx_mode = mod;
    SdrRxCfg cfg = { (SdrModulation)mod, lo, hi, DRV_FPGA_CODEC_FFTDEC };
    Sdr_Rx_SetConfig(&cfg);
    Sdr_Spectrum_SetModulation((SdrModulation)mod);
    Sdr_Spectrum_SetFilter(lo, hi);
    apply_tune(rx_cur_station_hz);
}

/* ======================================================================== *
 *  I2C3 control slave @ 0x72 (PA8 SCL / PC9 SDA): hardware debug + retune.
 *  The no-scope bring-up lifeline from phase3-rx. 5-byte [op, u32-LE] frames.
 * ======================================================================== */
#define I2C3_ADDR8   0xE4u
#define CMD_TUNE     0x01u   /* u32 Hz */
#define CMD_VOLUME   0x02u   /* f32 linear output volume (g_rx_volume) */
#define CMD_AGC      0x03u   /* u32 AGC mode: 0=off 1=slow 2=fast 3=auto */
#define CMD_LO_OFFSET 0x04u  /* i32 Hz */
#define CMD_MODE     0x05u   /* u8 1=AM 2=LSB 3=USB */
#define CMD_RFPRIM   0x81u
#define CMD_RFSEC    0x82u
#define CMD_BANDV    0x83u
#define CMD_RFIMG    0x84u
#define CMD_SC6      0x85u
#define CMD_GPIO     0x87u
#define CMD_CODEC    0x88u
#define CMD_DUMP     0x8Fu

#define I2C3_FRAME_MAX 16u
static volatile uint8_t  cmd_pending;
static volatile uint8_t  cmd_len;
static volatile uint8_t  cmd_buf[I2C3_FRAME_MAX];
static uint8_t  i2c3_rx[I2C3_FRAME_MAX];
static uint8_t  i2c3_idx;

static void i2c3_slave_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);
    M_AF(GPIOA, 8); M_OD(GPIOA, 8); PULL_UP(GPIOA, 8); SPD_HI(GPIOA, 8);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_4);
    M_AF(GPIOC, 9); M_OD(GPIOC, 9); PULL_UP(GPIOC, 9); SPD_HI(GPIOC, 9);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_9, LL_GPIO_AF_4);
    I2C3->CR1 = I2C_CR1_SWRST; I2C3->CR1 = 0;
    I2C3->CR2 = (PCLK1_HZ / 1000000u) & I2C_CR2_FREQ;
    I2C3->OAR1 = (1u << 14) | I2C3_ADDR8;
    I2C3->OAR2 = 0;
    I2C3->CR1 = I2C_CR1_PE;
    I2C3->CR1 |= I2C_CR1_ACK;
    I2C3->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;
    NVIC_SetPriority(I2C3_EV_IRQn, 6);
    NVIC_EnableIRQ(I2C3_EV_IRQn);
    NVIC_SetPriority(I2C3_ER_IRQn, 6);
    NVIC_EnableIRQ(I2C3_ER_IRQn);
}

void I2C3_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C3->SR1;
    if (sr1 & I2C_SR1_ADDR) { (void)I2C3->SR1; (void)I2C3->SR2; i2c3_idx = 0; }
    if (sr1 & I2C_SR1_RXNE) {
        uint8_t b = (uint8_t)I2C3->DR;
        if (i2c3_idx < sizeof i2c3_rx) i2c3_rx[i2c3_idx++] = b;
    }
    if (sr1 & I2C_SR1_TXE) I2C3->DR = 0x00u;
    if (sr1 & I2C_SR1_STOPF) {
        (void)I2C3->SR1;
        I2C3->CR1 |= I2C_CR1_PE;
        if (i2c3_idx >= 1u && !cmd_pending) {
            for (uint8_t k = 0; k < i2c3_idx; k++) cmd_buf[k] = i2c3_rx[k];
            cmd_len = i2c3_idx;
            cmd_pending = 1;
        }
        i2c3_idx = 0;
    }
}

void I2C3_ER_IRQHandler(void)
{
    I2C3->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
}

static void rf_dump(void)
{
    static const uint8_t vr[] = {0x21, 0x2D, 0x2E, 0x22, 0x23, 0x24, 0x25, 0x9D};
    bsp_uprintf("DUMP VC(I2C1 0x6A):");
    for (unsigned i = 0; i < sizeof vr; i++) {
        uint8_t v = 0; int rc = i2c_read(I2C1, ADDR_VERSACLOCK, vr[i], &v, 1);
        bsp_uprintf(" %02X=%02X%s", vr[i], v, rc ? "!" : "");
    }
    bsp_uprintf("\r\n");
    bsp_uprintf("DUMP BAND_V=0x%03lX RFimg=0x%06lX GPIOD.ODR=0x%08lX (PD9/10/11/13)\r\n",
                (unsigned long)(DAC->DHR12R2 & 0xFFFu), (unsigned long)rf_image,
                (unsigned long)GPIOD->ODR);
}

static void i2c3_poll_commands(void)
{
    if (!cmd_pending) return;
    uint8_t buf[I2C3_FRAME_MAX];
    uint8_t n = cmd_len;
    if (n > I2C3_FRAME_MAX) n = I2C3_FRAME_MAX;
    for (uint8_t k = 0; k < n; k++) buf[k] = cmd_buf[k];
    cmd_pending = 0;
    if (n < 5u) { bsp_uprintf("I2C cmd: short frame (ignored)\r\n"); return; }
    uint8_t  op  = buf[0];
    uint32_t arg = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8)
                 | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
    switch (op) {
    case CMD_TUNE:
        bsp_uprintf("I2C cmd: TUNE %lu Hz\r\n", (unsigned long)arg);
        apply_tune(arg);
        break;
    case CMD_VOLUME: {
        float v; __builtin_memcpy(&v, &arg, sizeof v);
        if (v < 0.0f) v = 0.0f;
        g_rx_volume = v;
        bsp_uprintf("I2C cmd: VOLUME %ld/1000\r\n", (long)(v * 1000.0f));
        break;
    }
    case CMD_AGC: {
        SdrAgcMode m = (arg == 1u) ? SDR_AGC_SLOW : (arg == 2u) ? SDR_AGC_FAST
                     : (arg == 3u) ? SDR_AGC_AUTO : SDR_AGC_OFF;
        Sdr_AGC_SetMode(m);
        bsp_uprintf("I2C cmd: AGC %s (watch cpu_max in alive line)\r\n",
                    m == SDR_AGC_OFF ? "OFF (fixed +70dB)" : m == SDR_AGC_SLOW ? "SLOW"
                    : m == SDR_AGC_FAST ? "FAST" : "AUTO");
        break;
    }
    case CMD_LO_OFFSET:
        rx_lo_offset_hz = (int32_t)arg;
        bsp_uprintf("I2C cmd: LO_OFFSET %ld Hz\r\n", (long)rx_lo_offset_hz);
        apply_tune(rx_cur_station_hz);
        break;
    case CMD_MODE: {
        uint8_t m = (uint8_t)(arg & 0xFFu);
        int mod = (m == 2u) ? SDR_MOD_LSB : (m == 3u) ? SDR_MOD_USB : SDR_MOD_AM;
        apply_mode(mod);
        bsp_uprintf("I2C cmd: MODE %s\r\n",
                    mod == SDR_MOD_AM ? "AM" : mod == SDR_MOD_LSB ? "LSB" : "USB");
        break;
    }
    case CMD_RFPRIM: rf_apply_primary((int)(arg & 0xFu));
        bsp_uprintf("I2C cmd: RFPRIM idx=%lu\r\n", (unsigned long)arg); break;
    case CMD_RFSEC:
        if (arg <= 6u) {
            int sbit = 13 - (int)arg;
            rf_image = (rf_image & ~0x3F80u) | (1u << sbit);
            rf_commit_image(rf_image);
            bsp_uprintf("I2C cmd: RFSEC idx=%lu bit %d img=0x%06lX\r\n",
                        (unsigned long)arg, sbit, (unsigned long)rf_image);
        }
        break;
    case CMD_BANDV: DAC->DHR12R2 = (uint16_t)(arg & 0xFFFu);
        bsp_uprintf("I2C cmd: BANDV=0x%03lX\r\n", (unsigned long)(arg & 0xFFFu)); break;
    case CMD_RFIMG: rf_image = arg & 0xFFFFFFu; rf_commit_image(rf_image);
        bsp_uprintf("I2C cmd: RFIMG=0x%06lX\r\n", (unsigned long)rf_image); break;
    case CMD_SC6: rf_chain6((uint8_t)(arg & 0x3Fu));
        bsp_uprintf("I2C cmd: SC6=0x%02lX\r\n", (unsigned long)(arg & 0x3Fu)); break;
    case CMD_GPIO: {
        static GPIO_TypeDef * const ports[7] =
            { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG };
        unsigned port = arg & 0xFu, pin = (arg >> 4) & 0xFu, mode = (arg >> 8) & 0xFu;
        if (port > 6u || pin > 15u) { bsp_uprintf("I2C cmd: GPIO bad\r\n"); break; }
        GPIO_TypeDef *g = ports[port];
        switch (mode) {
        case 0: M_IN(g, pin); break;
        case 1: M_IN(g, pin); PULL_DN(g, pin); break;
        case 2: M_IN(g, pin); PULL_UP(g, pin); break;
        case 3: SET_LO(g, pin); M_PP(g, pin); M_OUT(g, pin); break;
        case 4: SET_HI(g, pin); M_PP(g, pin); M_OUT(g, pin); break;
        case 5: SET_LO(g, pin); M_OD(g, pin); M_OUT(g, pin); break;
        case 6: SET_HI(g, pin); M_OD(g, pin); M_OUT(g, pin); break;
        default: break;
        }
        bsp_uprintf("I2C cmd: GPIO P%c%u mode=%u IDR=%u\r\n",
                    (char)('A' + port), pin, mode, (unsigned)((g->IDR >> pin) & 1u));
        break;
    }
    case CMD_CODEC: {
        uint8_t page = (uint8_t)(arg & 0xFFu), reg = (uint8_t)((arg >> 8) & 0xFFu),
                val = (uint8_t)((arg >> 16) & 0xFFu);
        uint8_t sel[2] = {0x00u, page}, wr[2] = {reg, val};
        int rc = cwr(ADDR_CODEC, sel, 2); rc |= cwr(ADDR_CODEC, wr, 2);
        bsp_uprintf("I2C cmd: CODEC P%u R0x%02X=0x%02X%s\r\n", page, reg, val, rc ? " NAK" : "");
        break;
    }
    case CMD_DUMP: rf_dump(); break;
    default: bsp_uprintf("I2C cmd: unknown op 0x%02X\r\n", op); break;
    }
}

/* ======================================================================== *
 *  Ported RX DSP bring-up (M1-M3 chain) + the strong i2s_dsp() in i2s_rx_path.
 * ======================================================================== */
static void rx_dsp_init(void)
{
    /* Event bus first: the SDR modules subscribe to RxCfg in their _Init (the
     * AGC/squelch/demods learn the modulation this way), and sdr_rx publishes
     * the committed RxCfg via the bus. App_Event_Process is dispatched from the
     * scheduler loop. */
    App_Event_Init();

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
    Sdr_Spectrum_Init();             /* registers its 40 ms FFT task */
    Sdr_Spectrum_SetFftdec(DRV_FPGA_CODEC_FFTDEC);
    I2s_Rx_Path_Init();
    apply_mode(SDR_MOD_AM);          /* AM config + filter + LO offset (publishes RxCfg) */

    /* Dispatch the queued RxCfg NOW, before enabling the AGC: the subscribers
     * (incl. the AGC) must adopt the AM modulation first. Enabling the AGC at
     * init *before* this ran left it in the SSB group (s_cfg_modulation defaults
     * to LSB until the bus delivers AM) -> SSB-group AGC on an AM signal = a
     * loud tone with no demod (confirmed on hardware). Dispatching first, then
     * SLOW, brings the AGC up correctly in the AM group (cpu ~+1%). */
    App_Event_Process();
    Sdr_AGC_SetMode(SDR_AGC_SLOW);
}

/* 1 s heartbeat task (registered on the scheduler): banks/s, S-meter, noise,
 * and the per-bank DSP CPU-budget gauge. */
static void alive_task(void)
{
    static uint32_t last_banks;
    const uint32_t budget_cyc = (HCLK_HZ / 100000u) * I2S_BANK_FRAMES; /* cyc/1.28 ms bank */
    uint32_t banks = i2s_bank_count;
    uint32_t cyc = g_i2s_dsp_cyc_max; g_i2s_dsp_cyc_max = 0;
    uint32_t pct = budget_cyc ? (cyc * 100u / budget_cyc) : 0u;
    bsp_uprintf("alive t=%lu ms  %s  banks=%lu (+%lu/s)  smeter=%ld dB  "
                "noise=%s%ld  cpu_max=%lu%% (%lu cyc / %lu)\r\n",
                (unsigned long)Bsp_Time_GetMs(),
                rx_mode == SDR_MOD_AM ? "AM" : rx_mode == SDR_MOD_LSB ? "LSB" : "USB",
                (unsigned long)banks, (unsigned long)(banks - last_banks),
                (long)(Sdr_Spectrum_Get_S_meter() + 0.5f),
                Sdr_Spectrum_Is_Noise_Valid() ? "" : "inv ",
                (long)(Sdr_Spectrum_Get_Noise_Db() + 0.5f),
                (unsigned long)pct, (unsigned long)cyc, (unsigned long)budget_cyc);
    last_banks = banks;
}

int main(void)
{
    __enable_irq();
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    /* DWT cycle counter for the per-bank DSP CPU gauge (i2s_rx_path). */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    bsp_uprintf("\r\n=== %s ===\r\n", FW_VERSION);
    bsp_uprintf("clock: HCLK=%lu Hz (%s)\r\n", (unsigned long)HCLK_HZ, BSP_CLK_DESC);
    bsp_uprintf("RX-only: PA disabled, charger off, TX I/Q zero-filled\r\n");

    i2c_pins_init();
    i2c_master_init(I2C1);
    i2c_master_init(I2C2);

    dac_init();
    DAC->DHR12R2 = band_v_code(RX_TUNED_HZ);

    bsp_uprintf("-- VersaClock (I2C1 0x6A): OUT0=26 MHz codec MCLK --\r\n");
    int vc = versaclock_init();
    bsp_uprintf("  0x6A %s\r\n", vc ? "ACK (present)" : "NAK (check PE14/PE7/rail)");

    bsp_uprintf("-- TLV320AIC3204 codec (I2C2 0x18) --\r\n");
    codec_reset_pulse();
    tlv320_startup();
    bsp_delay_ms(100);
    codec_unmute_hp();
    aux4a_startup();
    SET_HI(GPIOG, 8);
    int cs = iq_converter_startup();
    bsp_uprintf("  codec up (3+58 steps), HP unmuted +9 dB; IQ=%s\r\n",
                cs ? "CS4272" : "0x4C fallback");

    bsp_uprintf("-- RX front-end --\r\n");
    spi6_init();
    M_OUT(GPIOD, 15); M_PP(GPIOD, 15); SPD_HI(GPIOD, 15); SET_HI(GPIOD, 15);
    rf_chain6(0x0Eu);
    i2c3_slave_init();
    bsp_uprintf("  PD15 RX-enable high; I2C3 ctl @ 0x72 (TUNE/MODE/LO_OFFSET + hw debug)\r\n");

    bsp_uprintf("-- ported RX DSP (M1-M3 chain) + spectrum --\r\n");
    rx_dsp_init();
    apply_tune(RX_TUNED_HZ);
    bsp_uprintf("  tuned %lu Hz AM; raw=%lu audio_base=%u Hz\r\n",
                (unsigned long)RX_TUNED_HZ, (unsigned long)i2s_rate_raw,
                (unsigned)BOARD_AUDIO_SAMPLE_RATE_HZ);

    bsp_uprintf("-- I2S transport start --\r\n");
    i2s_config();
    i2s_start();
    bsp_uprintf("running: listen on headphone/speaker. per-bank DSP cost gauged below.\r\n");

    /* Register the housekeeping tasks and hand control to the scheduler loop
     * (it dispatches App_Event + the 40 ms spectrum task + these). Never
     * returns. The I2C3 command apply is too heavy for the ISR, so it runs here
     * as a 2 ms task; the heartbeat prints once a second. */
    App_Scheduler_Add(i2c3_poll_commands, 2u);
    App_Scheduler_Add(alive_task, 1000u);
    App_Scheduler_Run();   /* does not return */
    return 0;
}
