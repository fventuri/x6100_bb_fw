/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * m4_hostlink_probe -- M4a increment-1 link + protocol probe.
 * ============================================================================
 * Proves that the UNMODIFIED clean App host-link stack
 *   App/app_hostlink.c  (OEM I2C3 register-map + 56-word control block decode)
 *   App/app_radio_state.c (staged Begin/setters/Apply state machine)
 * compiles and links on the STM32F427, and that an injected OEM control block
 * drives the state machine end-to-end (tune/mode/vol/AGC/rfg -> the logging
 * backend stubs in m4_shims/m4_stubs.c, and the !configured->configured
 * "radio on" transition).
 *
 * No real radio hardware is brought up: only clock + SysTick + USART1 (for the
 * log) and the ported DSP init graph app_radio_state touches. The host side is
 * emulated by capturing the Bsp_I2cSlave callbacks and calling the write hook
 * with a synthetic control block, exactly as the I2C3 ISR would once the real
 * no-HAL slave transport lands in increment 2.
 *
 * Renode: a boot smoke test -- the transcript must show the decoded tune/mode/
 * vol and "host configuration accepted, radio on", with no fault.
 */

#include <stdint.h>
#include <string.h>

#include "bsp.h"
#include "bsp_i2c_slave.h"
#include "bsp_log.h"

#include "app_event.h"
#include "app_scheduler.h"
#include "app_radio_state.h"
#include "app_hostlink.h"

#include "drv_fpga_codec.h"
#include "sdr_common.h"
#include "sdr_resample.h"
#include "sdr_audio_resample.h"
#include "sdr_am.h"
#include "sdr_nb.h"
#include "sdr_notch.h"
#include "sdr_nr.h"
#include "sdr_agc.h"
#include "sdr_squelch.h"
#include "sdr_cw.h"
#include "sdr_compressor.h"
#include "sdr_rx.h"
#include "sdr_spectrum.h"

#define LOG_TAG "m4.probe"
#define FW_VERSION "m4_hostlink_probe (M4a link+protocol probe)"

/* --- emulated I2C3 host transport: capture the App's slave callbacks ------ */

static BspI2cSlaveWriteFn s_on_write;
static BspI2cSlaveReadFn  s_on_read;

int Bsp_I2cSlave_Init(BspI2cSlaveWriteFn on_write, BspI2cSlaveReadFn on_read) {
    if (on_write == NULL || on_read == NULL) {
        return -1;
    }
    s_on_write = on_write;
    s_on_read  = on_read;
    return 0;
}

/* Deliver a write transaction to the host-link layer as the I2C3 ISR would. */
static void host_write(uint16_t reg, const uint8_t *data, size_t len) {
    if (s_on_write != NULL) {
        s_on_write(reg, data, len);
    }
}

/* --- synthetic OEM control block ------------------------------------------ */

/* Georgy's command block is HOSTLINK_CMD_WORDS (54) words at reg 0, one u32
 * per command at cmd*4, little-endian on the wire. We fill the words the RX
 * path acts on; every other word is left 0 (a valid "written" value, distinct
 * from the 0xFFFFFFFF "never written" sentinel that gates host_block_configured). */
#define CMD_WORDS 54u

enum {
    W_VFOA_FREQ = 1,
    W_VFOA_MODE = 4,
    W_VFOA_AGC  = 5,
    W_VFOB_FREQ = 7,
    W_VFO_SEL   = 13,
    W_RXVOL     = 14,
    W_RFG_TXPWR = 15,
};

static void put_word(uint8_t *blk, unsigned idx, uint32_t v) {
    blk[idx * 4u + 0u] = (uint8_t)(v & 0xFFu);
    blk[idx * 4u + 1u] = (uint8_t)((v >> 8) & 0xFFu);
    blk[idx * 4u + 2u] = (uint8_t)((v >> 16) & 0xFFu);
    blk[idx * 4u + 3u] = (uint8_t)((v >> 24) & 0xFFu);
}

static void inject_control_block(void) {
    uint8_t blk[CMD_WORDS * 4u];
    memset(blk, 0, sizeof(blk));

    put_word(blk, W_VFOA_FREQ, 1410000u);       /* WDOV 1410 kHz AM */
    put_word(blk, W_VFOB_FREQ, 1410000u);       /* both VFOs set -> block configured */
    put_word(blk, W_VFOA_MODE, (uint32_t)SDR_MOD_AM); /* OEM enum 6 == SDR_MOD_AM */
    put_word(blk, W_VFOA_AGC, 1u);              /* AGC slow */
    put_word(blk, W_VFO_SEL, 0u);               /* active VFO A */
    put_word(blk, W_RXVOL, 40u);                /* host RX volume 0..100 */
    put_word(blk, W_RFG_TXPWR, 63u);            /* rfg 63 (GUI default), txpwr 0 */

    bsp_uprintf("probe: injecting OEM control block "
                "(freq=1410000 mode=AM agc=slow vol=40 rfg=63)\r\n");
    host_write(0x0000u, blk, sizeof(blk));
}

/* --- probe report task (runs once, ~50 ms after the host block) ----------- */

static void probe_report(void) {
    static bool done;
    if (done) {
        return;
    }
    done = true;

    bsp_uprintf("---- PROBE RESULT ----\r\n");
    bsp_uprintf("configured : %s\r\n",
                App_RadioState_IsConfigured() ? "YES (radio on)" : "no");
    bsp_uprintf("rx freq    : %lu Hz\r\n",
                (unsigned long)App_RadioState_GetRxFreq());
    bsp_uprintf("rx mode    : %u (SDR_MOD_AM=%u)\r\n",
                (unsigned)App_RadioState_GetRxMode(), (unsigned)SDR_MOD_AM);
    bsp_uprintf("rxvol      : (HPR gain %d dB, muted=%u)\r\n",
                (int)App_RadioState_GetHprGainDb(),
                (unsigned)App_RadioState_IsMuted());
    bsp_uprintf("rfg        : %u\r\n", (unsigned)App_RadioState_GetRfg());
    bsp_uprintf("---- PROBE %s ----\r\n",
                App_RadioState_IsConfigured() ? "PASS" : "FAIL");
}

/* --- DSP init graph the host-link state machine touches ------------------- */

static void dsp_init(void) {
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
    Sdr_CW_Init();
    Sdr_Compressor_Init();
    Sdr_Rx_Init();
    Sdr_Spectrum_Init();
}

int main(void) {
    __enable_irq();
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== %s ===\r\n", FW_VERSION);
    bsp_uprintf("clock: HCLK=%lu Hz (%s)\r\n", (unsigned long)HCLK_HZ, BSP_CLK_DESC);

    dsp_init();

    /* Sdr_Spectrum_Init() ran in dsp_init(); app_radio_state's spectrum setters
     * are valid now. */
    if (App_RadioState_Init() != 0) {
        bsp_uprintf("App_RadioState_Init FAILED\r\n");
    }
    if (App_HostLink_Init() != 0) {
        bsp_uprintf("App_HostLink_Init FAILED (i2c slave / scheduler)\r\n");
    }

    /* Emulate the host writing the OEM control block over I2C3. This sets the
     * command-dirty flag and notifies the scheduler; hostlink_task decodes it on
     * the first scheduler pass. */
    inject_control_block();

    App_Scheduler_Add(probe_report, 50u);
    App_Scheduler_Run(); /* dispatches hostlink_task + App_Event + probe_report */
    return 0;
}
