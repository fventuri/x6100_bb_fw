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
#include "sdr_cw.h"
#include "sdr_nb.h"
#include "sdr_nbfm.h"
#include "sdr_notch.h"
#include "sdr_nr.h"
#include "sdr_resample.h"
#include "sdr_rx.h"
#include "sdr_spectrum.h"
#include "sdr_squelch.h"
#include "sdr_ssb.h"
#include "sdr_wfm.h"

#include "app_fpga.h"
#include "app_radio.h"
#include "app_radio_rfpath.h"
#include "app_radio_state.h"
#include "app_hostlink.h"

#include "flow_uplink.h"

#define FW_VERSION "x6100_bb_fw m4-rx-host 2026-11-19 (OEM host link)"

#define RX_TUNED_HZ   1410000u
#define AM_LO_OFFSET  (-100)
#define RX_RFG_DEFAULT 63u    /* OEM/GUI default RF gain (reg15) -> PB1 0x25 */

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

/* OEM reg15 RF-gain -> six-bit cf88/PB1 attenuator word (ported verbatim from
 * phase4-linux firmware.c rfg_to_pb1; HW-verified 2026-11-13 on a live 1410 kHz
 * carrier). PB1 0x00 (min atten) .. 0x3F (max atten) spans ~22.7 dB of RX gain;
 * cf88/PB1 IS the RX front-end attenuator. The GUI knob is 0..100 (default 63 ->
 * 0x25). rfg <= 36 pins max atten and OEM adds a DSP input scalar for the fine
 * steps below PB1's floor -- that sub-37 scalar is deferred here (TODO), so the
 * normal 37..100 range (incl. the 63 default) is exact and <=36 is coarse. */
static uint8_t rfg_to_pb1(uint32_t rfg)
{
    rfg &= 0xFFu;
    if (rfg <= 36u)  return 0x3Fu;
    if (rfg <= 100u) return (uint8_t)(100u - rfg);
    return 0x00u;
}

/* ======================================================================== *
 *  X6100 hardware backends under the clean App surface (M4a).
 *  ------------------------------------------------------------------------
 *  The unmodified App host-link stack (App/app_hostlink.c + app_radio_state.c)
 *  drives the radio through these seams; here they map onto the proven X6100
 *  bring-up primitives above (VersaClock LO, RF band GPIO/BAND_V, TLV320), so
 *  the stock Linux host (or phase4-linux-ctl.py) controls tune/mode/vol/AGC.
 *  The X6200 app_fpga.c / app_radio_rfpath.c / app_radio.c are NOT compiled
 *  (their FPGA/PCA9535 hardware is absent on the X6100); these replace them.
 * ======================================================================== */
static volatile int32_t rx_lo_offset_hz = AM_LO_OFFSET;
static uint32_t rx_cur_station_hz = RX_TUNED_HZ;
static int      rx_mode = SDR_MOD_AM;

/* Mode-dependent LO offset: AM/SAM and NFM keep the carrier off DC so the RX
 * DC blocker does not null it; SSB/CW tune zero-beat. Matches phase3/phase4. */
static int32_t lo_offset_for_mode(SdrModulation m)
{
    if (m == SDR_MOD_AM || m == SDR_MOD_SAM) return AM_LO_OFFSET; /* -100 Hz */
    if (m == SDR_MOD_NFM)                    return -200;
    return 0;
}

/* The flow uplink (m4_backends/flow_uplink.c) needs the active mode LO offset to
 * report the absolute I/Q center (flow_info.lo_freq = RX freq + this). */
int32_t m4_rx_lo_offset_hz(void) { return rx_lo_offset_hz; }

/* --- App_Fpga_* : the RX "tuning" actuator = VersaClock OUT1 = 4 x Ftuned.
 * app_radio_state passes the final RX frequency (active VFO + RIT); the mode LO
 * offset is folded in here. TX / sample-rate / mute are RX-inert for M4a. */
void App_Fpga_SetRxFrequency(uint32_t freq_hz)
{
    rx_cur_station_hz = freq_hz;
    rx_mode           = (int)App_RadioState_GetRxMode();
    rx_lo_offset_hz   = lo_offset_for_mode((SdrModulation)rx_mode);
    uint32_t ftuned   = (uint32_t)((int32_t)freq_hz + rx_lo_offset_hz);
    vc_set_out1_hz(4u * ftuned);
}
void App_Fpga_SetTxFrequency(uint32_t freq_hz) { (void)freq_hz; }
void App_Fpga_TxMute(uint32_t ms) { (void)ms; }
void App_Fpga_SetSampleRate(uint8_t fftdec) { (void)fftdec; } /* fixed 100 kHz */

/* --- App_RadioRfPath_* : analog band filters + BAND_V from the tuned freq, plus
 * the reg15 RF-gain attenuator. SetFreq must NOT touch cf88/PB1 -- the attenuator
 * is owned solely by the rfg path (SetRfg); re-pinning it on every retune is the
 * ~8 dB level bug phase4 hit (firmware.c:605-608). att/pre remain deferred. The
 * LPF never pulses on RX so LpfWillChange is false. */
void App_RadioRfPath_SetFreq(uint32_t freq_hz)
{
    rf_switch_tune(freq_hz);
    DAC->DHR12R2 = band_v_code(freq_hz);
}
bool App_RadioRfPath_LpfWillChange(uint32_t freq_hz) { (void)freq_hz; return false; }
/* Drive the cf88/PB1 RF-gain attenuator from the host reg15 rfg, OEM-faithful
 * (see rfg_to_pb1). Default rfg 63 -> PB1 0x25, matching the OEM RX level. This
 * replaces the former boot-pinned attenuator and any digital FLOW_IQ_CAL fudge;
 * the flow IQ scale stays unity (Q31_TO_F32 = 1/2^31, == OEM's 2^-31). */
void App_RadioRfPath_SetRfg(uint8_t rfg) { rf_chain6(rfg_to_pb1(rfg)); }
void App_RadioRfPath_SetAtt(bool on) { (void)on; }
void App_RadioRfPath_SetPre(bool on) { (void)on; }

/* --- App_Radio_* : radio-mode latch + the RX audio (rxvol) codec path. */
static AppRadioMode s_radio_mode = APP_RADIO_MODE_OFF;

int          App_Radio_Init(void) { return 0; } /* codec is already up in main */
int          App_Radio_SetRadioMode(AppRadioMode mode) { s_radio_mode = mode; return 0; }
AppRadioMode App_Radio_GetRadioMode(void) { return s_radio_mode; }
void         App_Radio_UpdateSpeaker(void) {} /* speaker policy deferred */

/* Apply the host RX volume: the HPL/HPR driver analog gain + mute from the
 * app_radio_state rxvol transfer, and the digital left-DAC scale into the
 * i2s_rx_path output gain (g_rx_volume). Boot volume 0 -> muted (boot-silent
 * rule). The TLV320 HP driver gain reg (page1 R0x10/R0x11) is 6-bit signed
 * (-6..+29 dB) with bit6 = mute. */
void App_Radio_ApplyAudioConfig(void)
{
    int8_t  gain_db = App_RadioState_GetHprGainDb();
    bool    mute    = App_RadioState_IsMuted();
    uint8_t hp      = (uint8_t)(((uint8_t)gain_db & 0x3Fu) | (mute ? 0x40u : 0x00u));

    uint8_t p1[]  = {0x00, 0x01};        /* page 1 */
    uint8_t hpl[] = {0x10, hp};
    uint8_t hpr[] = {0x11, hp};
    cwr(ADDR_CODEC, p1, sizeof p1);
    cwr(ADDR_CODEC, hpl, sizeof hpl);
    cwr(ADDR_CODEC, hpr, sizeof hpr);

    g_rx_volume = App_RadioState_GetLeftDacScale();
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
    /* Every demodulator must be initialised before the first i2s_dsp() bank:
     * each _Init sets up its CMSIS biquad/FIR structs (numStages + coeff/state
     * pointers) and subscribes to RxCfg. A mode whose _Init was skipped runs
     * arm_biquad_cascade_df1_f32() with numStages=0, whose `do{}while(--stage)`
     * underflows to ~4e9 NULL-pointer iterations: the DSP ISR never returns,
     * starving the main loop AND the lower-priority I2C3 slave ISR -- which the
     * R16 host sees as "mv64xxx: I2C bus locked". The boot default is LSB, so
     * Sdr_SSB_Init() is mandatory; init every mode so host mode switches are
     * safe too. Mirrors App_Radio_Init() in App/app_radio.c (not compiled). */
    Sdr_SSB_Init();
    Sdr_CW_Init();
    Sdr_AM_Init();
    Sdr_NBFM_Init();
    Sdr_WFM_Init();
    Sdr_NB_Init();
    Sdr_Notch_Init();
    Sdr_NR_Init();
    Sdr_AGC_Init();
    Sdr_Squelch_Init();
    Sdr_Rx_Init();
    Sdr_Spectrum_Init();             /* registers its 40 ms FFT task */
    Sdr_Spectrum_SetFftdec(DRV_FPGA_CODEC_FFTDEC);
    I2s_Rx_Path_Init();
    /* No apply_mode()/AGC here: the modulation, filter, tune and AGC are now
     * owned by the App host-link stack (App_RadioState_Init publishes the
     * initial RxCfg; the host then drives everything). main() flushes the queued
     * config with App_Event_Process() before audio starts. */
}

/* The VersaClock LO offset (lo_offset_for_mode) is folded in by
 * App_Fpga_SetRxFrequency, but app_radio_state's Apply only re-tunes on a
 * FREQUENCY change (app_radio_state.c:346). A pure host mode switch at the same
 * frequency (e.g. SSB->AM) therefore leaves the AM/NFM carrier at DC, where the
 * demod's DC blocker nulls it -> gross distortion. Poll the committed RX mode
 * and re-apply the LO (which recomputes the mode offset) on a change. Runs in
 * the scheduler/main-loop context: the VersaClock I2C1 writes are not ISR-safe.
 */
static void lo_offset_task(void)
{
    static int last_mode = -1;
    int m = (int)App_RadioState_GetRxMode();
    if (m != last_mode) {
        last_mode = m;
        App_Fpga_SetRxFrequency(App_RadioState_GetRxFreq());
    }
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
                App_RadioState_GetRxMode() == SDR_MOD_AM ? "AM"
                : App_RadioState_GetRxMode() == SDR_MOD_LSB ? "LSB"
                : App_RadioState_GetRxMode() == SDR_MOD_USB ? "USB" : "mod",
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
    /* HP driver gain/mute is now owned by the App rxvol path
     * (App_Radio_ApplyAudioConfig below); boot rxvol 0 -> muted (boot-silent). */
    aux4a_startup();
    SET_HI(GPIOG, 8);
    int cs = iq_converter_startup();
    bsp_uprintf("  codec up (3+58 steps); IQ=%s\r\n",
                cs ? "CS4272" : "0x4C fallback");

    bsp_uprintf("-- RX front-end --\r\n");
    spi6_init();
    M_OUT(GPIOD, 15); M_PP(GPIOD, 15); SPD_HI(GPIOD, 15); SET_HI(GPIOD, 15);
    /* Seed the RF-gain attenuator at the OEM default (rfg 63 -> PB1 0x25) so a
     * standalone / no-GUI boot matches the OEM RX level; the host reg15 path
     * (App_RadioRfPath_SetRfg) takes over once the GUI connects. */
    rf_chain6(rfg_to_pb1(RX_RFG_DEFAULT));
    bsp_uprintf("  PD15 RX-enable high\r\n");

    bsp_uprintf("-- ported RX DSP (M1-M3 chain) + spectrum --\r\n");
    rx_dsp_init();

    /* Bring up the clean App host-link stack over the X6100 backends above.
     * App_RadioState_Init tunes the LO to the default freq (App_Fpga backend)
     * and publishes the initial RxCfg; App_HostLink_Init stands up the OEM
     * I2C3 register slave @ 0x72 + the hostlink task. Flush the queued RxCfg to
     * the DSP subscribers, set the RF band/BAND_V for the default freq (Init
     * only moves the LO), and apply the boot (muted) audio config. */
    bsp_uprintf("-- App host link (OEM I2C3 @ 0x72) --\r\n");
    App_RadioState_Init();
    App_HostLink_Init();
    App_Event_Process();
    App_RadioRfPath_SetFreq(App_RadioState_GetRxFreq());
    App_Radio_ApplyAudioConfig();
    bsp_uprintf("  radio awaiting host config; default %lu Hz, boot-silent\r\n",
                (unsigned long)App_RadioState_GetRxFreq());

    bsp_uprintf("-- I2S transport start --\r\n");
    i2s_config();
    i2s_start();

    /* Stand up the OEM uplink: USART6/DMA2 streams the 100 kHz raw I/Q + LO
     * center to the host GUI (waterfall/panadapter + S-meter) every 35 ms. */
    bsp_uprintf("-- OEM flow uplink (USART6 @ 1.5 Mbaud, DMA2) --\r\n");
    Flow_Uplink_Init();
    bsp_uprintf("running: drive via the OEM host protocol (phase4-linux-ctl.py).\r\n");

    /* Hand control to the scheduler loop: it dispatches App_Event, the 40 ms
     * spectrum task, the hostlink task (host command block / command window) and
     * the 1 s heartbeat. Never returns. */
    App_Scheduler_Add(lo_offset_task, 50u);
    App_Scheduler_Add(Flow_Uplink_Tick, 35u);
    App_Scheduler_Add(alive_task, 1000u);
    App_Scheduler_Run();   /* does not return */
    return 0;
}
