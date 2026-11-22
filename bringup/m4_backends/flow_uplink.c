/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * flow_uplink (F427, no-HAL) -- OEM X6100 USART6 flow-packet uplink.
 * See flow_uplink.h for the contract. Register-level port of phase4-linux
 * firmware.c (usart6_flow_init / flow_build / flow_send / crc32_hw): the wire
 * format, pins (PC6/PC7 AF8), DMA2 Stream 6 channel 5 and the STM32 hardware
 * CRC32 are board-identical.
 *
 * Difference from the reference: the 512-sample I/Q block is NOT filled by the
 * DSP ISR here. The m4 datapath already keeps a driver-owned Q1.31 raw ring
 * (Drv_Fpga_GetRawWindow / Drv_Fpga_GetPos, 8 banks = 1024 complex), so this
 * snapshots the newest 512 samples behind the write head in the main-loop task
 * -- the same "read one block behind" approach sdr_spectrum uses, so no ISR
 * changes and no tearing (512 samples = 4 banks ~= 5 ms of headroom vs a
 * microsecond-scale copy).
 */
#include "flow_uplink.h"

#include <stdint.h>
#include <string.h>

#include "bsp.h"              /* LL + CMSIS device regs, GPIO macros, PCLK2_HZ */
#include "drv_fpga.h"         /* Drv_Fpga_GetRawWindow / Drv_Fpga_GetPos       */
#include "app_radio_state.h"  /* App_RadioState_GetRxFreq                      */

/* The mode-dependent LO offset lives in m4_rx_host.c (set in
 * App_Fpga_SetRxFrequency); the absolute I/Q center = RX freq + that offset. */
extern int32_t m4_rx_lo_offset_hz(void);

/* ---- wire format (phase4-linux firmware.c, OEM BASE 1.1.9) --------------- */
#define FLOW_BAUD        1500000u    /* on-wire rate                          */
#define FLOW_MAGIC       0xAA5555AAu
#define FLOW_BYTES       0x1028u     /* total DMA length (4136 bytes)         */
#define FLOW_CRC_WORDS   0x409u      /* words fed to the CRC engine (4132 B)  */
#define FLOW_CRC_OFF     0x1024u     /* byte offset of the CRC32 field        */
#define FLOW_INFO_OFF    0x1014u     /* byte offset of flow_info_t            */
#define FLOW_IQ_OFF      0x0004u     /* byte offset of the 512-cplx f32 block */
#define FLOW_IQ_SAMPLES  512u        /* fp32 complex count (0x1000 B / 8)     */

#define Q31_TO_F32       (1.0f / 2147483648.0f)   /* Q1.31 -> [-1,1), cal 1.0 */

/* flow_info_t occupies the OEM "0x1014 reserved" 12 bytes -- byte-identical to
 * X6100Control low/flow.h so the GUI decodes our bitfields exactly. */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t lo_freq;             /* I/Q center (antenna Hz) for the panadapter */
    uint8_t  flow_fmt;            /* 0 = fp32 (512 cplx)                        */
    uint8_t  flow_seq_n: 4;       /* block index within one buffer              */
    uint8_t  flow_seq_total: 4;   /* blocks per buffer                          */
    uint8_t  vary_freq: 1;        /* 1 => lo_freq changed vs the last packet    */
    uint8_t  fft_dec: 3;          /* flow decimation exponent (0 = full 100 kHz)*/
    uint16_t audio_in_lvl_db: 7;  /* TX audio-in level (RX: 0)                  */
    uint32_t _pad1: 5;
    uint32_t _pad2;
} flow_info_t;
_Static_assert(sizeof(flow_info_t) == 12u, "flow_info_t must be 12 bytes");

/* 4136-byte packet in DMA-reachable SRAM (.bss on this board is at 0x20000000,
 * not CCM), 4-byte aligned for the CRC engine and the DMA burst. */
static union {
    uint8_t  b[FLOW_BYTES];
    uint32_t w[FLOW_BYTES / 4u];      /* 0x40A = 1034 words */
} flow __attribute__((aligned(4)));

/* STM32 hardware CRC32 (poly 0x04C11DB7, init 0xFFFFFFFF, non-reflected):
 * reset DR, feed n words, read DR. */
static uint32_t crc32_hw(const volatile uint32_t *w, uint32_t n)
{
    CRC->CR = CRC_CR_RESET;
    for (uint32_t i = 0; i < n; i++) {
        CRC->DR = w[i];
    }
    return CRC->DR;
}

/* Snapshot the newest FLOW_IQ_SAMPLES complex samples of the raw 100 kHz ring,
 * converting Q1.31 -> f32 (unity cal), interleaved I,Q into the packet. If the
 * ring is not up yet the region is left zeroed (GUI shows a flat panadapter). */
static void fill_iq(void)
{
    const DrvFpgaRawWindow *win = Drv_Fpga_GetRawWindow();
    float *iq = (float *)&flow.b[FLOW_IQ_OFF];

    if (win == NULL || win->i == NULL || win->count < FLOW_IQ_SAMPLES) {
        memset(iq, 0, FLOW_IQ_SAMPLES * 2u * sizeof(float));
        return;
    }

    uint32_t mask  = win->count - 1u;          /* count is a power of two */
    uint32_t pos   = Drv_Fpga_GetPos();         /* snapshot the write head once */
    uint32_t start = (pos - FLOW_IQ_SAMPLES) & mask;
    for (uint32_t k = 0; k < FLOW_IQ_SAMPLES; k++) {
        uint32_t idx = (start + k) & mask;
        iq[2u * k]      = (float)win->i[idx] * Q31_TO_F32;
        iq[2u * k + 1u] = (float)win->q[idx] * Q31_TO_F32;
    }
}

static void flow_build(void)
{
    static uint32_t prev_lo_freq;

    flow.w[0] = FLOW_MAGIC;                      /* 0x0000 magic              */
    fill_iq();                                   /* 0x0004 live 100 kHz I/Q   */

    /* Telemetry: the real ADC2 ext-voltage + MAX17205 BMS read is a later
     * increment, BUT shipping all-zeros is fatal -- the GUI power widget reads
     * "vext<3 V => on battery" and then "cap=0% => critically low" and cleanly
     * powers the R16 OFF (observed: orderly poweroff ~15 s after the GUI starts;
     * firmware.c:931 "the battery % when on battery (vext<3 V)"). So this interim
     * reports a nominal EXTERNAL supply present (flags bit3 + 13.8 V) -- true for
     * a bench-powered radio -- plus a healthy placeholder battery, so no low-
     * battery shutdown path can fire. Replaced by the real BMS read increment. */
    enum { FLOW_VEXT_DV = 138u, FLOW_VBAT_DV = 82u, FLOW_BATCAP_PCT = 96u };
    flow.w[0x401]  = 0x00000008u;                /* 0x1004 flags: bit3 vext present */
    flow.b[0x100C] = FLOW_VEXT_DV;               /* 0x100C ext voltage (dV): 13.8 V */
    flow.b[0x100D] = FLOW_VBAT_DV;               /* 0x100D battery voltage (dV): 8.2 V */
    flow.b[0x100E] = FLOW_BATCAP_PCT;            /* 0x100E battery capacity %       */

    /* flow_info (rev>=8): absolute I/Q center so the GUI positions the 100 kHz
     * panadapter. lo_freq = RX freq (active VFO + RIT) + mode LO offset, i.e.
     * App_Fpga_SetRxFrequency's ftuned. fp32, no flow decimation this build. */
    uint32_t lo_freq = (uint32_t)((int32_t)App_RadioState_GetRxFreq()
                                  + m4_rx_lo_offset_hz());
    flow_info_t *fi = (flow_info_t *)&flow.b[FLOW_INFO_OFF];
    fi->lo_freq         = lo_freq;
    fi->flow_fmt        = 0u;                    /* fp32, 512 complex         */
    fi->flow_seq_n      = 0u;                    /* one self-contained block  */
    fi->flow_seq_total  = 1u;
    fi->vary_freq       = (lo_freq != prev_lo_freq) ? 1u : 0u;
    fi->fft_dec         = 0u;                    /* full 100 kHz span (no zoom)*/
    fi->audio_in_lvl_db = 0u;
    fi->_pad1           = 0u;
    fi->_pad2           = 0u;
    prev_lo_freq        = lo_freq;

    flow.w[0x408] = 0u;                          /* 0x1020 hkey (deferred)    */
    flow.w[FLOW_CRC_OFF / 4u] = crc32_hw(flow.w, FLOW_CRC_WORDS);
}

/* Kick one 4136-byte DMA burst (a full packet at 1.5 Mbaud is ~28 us). */
static void flow_send(void)
{
    while (DMA2_Stream6->CR & DMA_SxCR_EN) { }   /* previous burst done?      */
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6
                | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
    DMA2_Stream6->M0AR = (uint32_t)flow.b;
    DMA2_Stream6->NDTR = FLOW_BYTES;
    DMA2_Stream6->CR  |= DMA_SxCR_EN;
}

void Flow_Uplink_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_DMA2
                             | LL_AHB1_GRP1_PERIPH_CRC);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

    /* PC6 = USART6_TX, PC7 = USART6_RX, AF8, high speed. */
    M_AF(GPIOC, 6); M_PP(GPIOC, 6); SPD_HI(GPIOC, 6);
    LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_6, LL_GPIO_AF_8);
    M_AF(GPIOC, 7); SPD_HI(GPIOC, 7);
    LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_7, LL_GPIO_AF_8);

    /* USART6 on APB2 (PCLK2). BRR = round(PCLK2 / baud) in OVER16 mode. */
    USART6->CR1 = 0u;
    USART6->BRR = (PCLK2_HZ + FLOW_BAUD / 2u) / FLOW_BAUD;
    USART6->CR3 = USART_CR3_DMAT;                 /* TX served by DMA          */
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE;    /* 8-N-1, transmitter on     */

    /* DMA2 Stream 6, channel 5: memory->peripheral, byte-wide, MINC, one-shot. */
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream6->CR & DMA_SxCR_EN) { }
    DMA2_Stream6->PAR = (uint32_t)&USART6->DR;
    DMA2_Stream6->CR  = (5u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
}

void Flow_Uplink_Tick(void)
{
    flow_build();
    flow_send();
}
