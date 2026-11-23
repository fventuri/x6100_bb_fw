/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 Franco Venturi. Part of x6100_bb_fw (GPLv3). */
/*
 * X6100 (STM32F427) analog monitor -- see drv_adc_mon.h for the channel map.
 * No STM32 HAL: CMSIS device header + direct register writes, matching the rest
 * of the F427 BSP. Two regular-scan ADC groups free-run into circular DMA2
 * buffers with no ADC interrupt (exactly as OEM BASE 1.1.9 arranges them); the
 * DMA refreshes the buffers on its own and Drv_AdcMon_Sample() reads a coherent
 * copy and runs the OEM ADC2 telemetry filter.
 */
#include "drv_adc_mon.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "bsp_irq.h"
#include "bsp_time.h" /* pulls stm32f4xx.h for the register definitions */

#define ADC1_SCAN_LEN 3u                       /* ch0, ch1, ch2 */
#define ADC2_SETS     5u                       /* OEM five-set interleave */
#define ADC2_TRIPLE   3u                       /* ch6, ch7, ch14 */
#define ADC2_BUF_LEN  (ADC2_SETS * ADC2_TRIPLE)
#define ADC_SMP_480   7u                       /* 480-cycle sample time */
#define ADC_RAW_FULL_SCALE 4096u

/* OEM ADC2 telemetry filter: y = 0.92*y + 0.016*sum(5) = 0.92*y + 0.08*mean. */
#define ADC2_FILT_Y   0.92f
#define ADC2_FILT_SUM 0.016f

/* DMA-accessible plain SRAM (the F427 has one contiguous SRAM; no CCM here). */
static uint16_t s_adc1_buf[ADC1_SCAN_LEN] __attribute__((aligned(4)));
static uint16_t s_adc2_buf[ADC2_BUF_LEN]  __attribute__((aligned(4)));

static uint16_t s_adc1_scan[ADC1_SCAN_LEN]; /* last coherent ADC1 triplet */
static float    s_adc2_filt[ADC2_TRIPLE];   /* filtered ch6, ch7, ch14 */
static bool     s_adc2_seeded;

/* Unified snapshot handed to the optional scan callback (indices match the
 * DrvAdcMonChannel enum). */
static volatile uint16_t s_unified[DRV_ADC_MON_CH_COUNT];

static DrvAdcMonScanCallback s_scan_callback;
static uint32_t s_scans;           /* estimated HARDWARE scans (not Sample calls) */
static uint32_t s_overruns;
static uint32_t s_dma_errors;
static uint32_t s_max_gap_cycles;  /* always 0: continuous DMA, no measurable gap */
static uint32_t s_last_scan_cycles;
static uint32_t s_scan_period_cyc; /* CPU cycles per ADC1 scan, computed lazily */
static bool     s_have_last_scan;

static void adc_common_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
    (void)RCC->APB2ENR;
    /* ADCCLK = PCLK2 / 8 = 15.625 MHz (under the 36 MHz max), independent mode. */
    ADC123_COMMON->CCR =
        (ADC123_COMMON->CCR & ~(ADC_CCR_ADCPRE | ADC_CCR_MULTI)) |
        ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;
}

/* Configure one DMA2 stream: circular periph->mem, 16-bit, memory-incrementing. */
static void dma_stream_init(DMA_Stream_TypeDef *st, uint32_t chsel,
                            volatile uint32_t *par, void *mem, uint32_t len) {
    st->CR &= ~DMA_SxCR_EN;
    while (st->CR & DMA_SxCR_EN) {
    }
    st->PAR  = (uint32_t)par;
    st->M0AR = (uint32_t)mem;
    st->NDTR = len;
    st->FCR  = 0u; /* direct mode (FIFO off): one 16-bit transfer per request */
    st->CR   = (chsel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_0 |
               DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
    st->CR |= DMA_SxCR_EN;
}

static void adc1_init(void) {
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT; /* ADON set later */
    /* 480-cycle sample time on ch0, ch1, ch2 (high-impedance detector nodes). */
    ADC1->SMPR2 = (ADC_SMP_480 << (3u * 0u)) | (ADC_SMP_480 << (3u * 1u)) |
                  (ADC_SMP_480 << (3u * 2u));
    ADC1->SMPR1 = 0u;
    ADC1->SQR1  = (ADC1_SCAN_LEN - 1u) << ADC_SQR1_L_Pos;
    ADC1->SQR3  = (0u << 0) | (1u << 5) | (2u << 10); /* SQ1=ch0 SQ2=ch1 SQ3=ch2 */
}

static void adc2_init(void) {
    ADC2->CR1 = ADC_CR1_SCAN;
    ADC2->CR2 = ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT;
    /* ch6, ch7 live in SMPR2 (ch0..9); ch14 in SMPR1 (ch10..18). */
    ADC2->SMPR2 = (ADC_SMP_480 << (3u * 6u)) | (ADC_SMP_480 << (3u * 7u));
    ADC2->SMPR1 = (ADC_SMP_480 << (3u * (14u - 10u)));
    ADC2->SQR1  = (ADC2_TRIPLE - 1u) << ADC_SQR1_L_Pos;
    ADC2->SQR3  = (6u << 0) | (7u << 5) | (14u << 10); /* SQ1=ch6 SQ2=ch7 SQ3=ch14 */
}

static void adc_start(ADC_TypeDef *adc) {
    adc->SR = 0u;
    adc->CR2 |= ADC_CR2_ADON;
    Bsp_Time_DelayMs(1u); /* tSTAB after power-up before the first conversion */
    adc->CR2 |= ADC_CR2_SWSTART;
}

int Drv_AdcMon_Init(void) {
    uint32_t n0;
    uint32_t t0;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;

    adc_common_init();

    /* Clear any stale DMA2 Stream0 / Stream2 flags, then arm the streams first
     * so the first conversions have somewhere to land. */
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                  DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 | DMA_LIFCR_CTCIF2 |
                  DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 |
                  DMA_LIFCR_CFEIF2;

    dma_stream_init(DMA2_Stream0, 0u, &ADC1->DR, s_adc1_buf, ADC1_SCAN_LEN);
    dma_stream_init(DMA2_Stream2, 1u, &ADC2->DR, s_adc2_buf, ADC2_BUF_LEN);

    adc1_init();
    adc2_init();
    adc_start(ADC1);
    adc_start(ADC2);

    /* Confirm the ADC2 DMA is actually advancing before returning. */
    n0 = DMA2_Stream2->NDTR;
    t0 = Bsp_Time_GetMs();
    while (DMA2_Stream2->NDTR == n0) {
        if ((Bsp_Time_GetMs() - t0) > 10u) {
            return -1;
        }
    }
    return 0;
}

int Drv_AdcMon_Sample(void) {
    uint16_t a1[ADC1_SCAN_LEN];
    uint16_t a2[ADC2_BUF_LEN];
    uint32_t n0;
    uint32_t t0;
    uint32_t lisr;
    size_t   i;
    size_t   c;

    /* Liveness: wait for the free-running ADC2 DMA pointer to advance. */
    n0 = DMA2_Stream2->NDTR;
    t0 = Bsp_Time_GetMs();
    while (DMA2_Stream2->NDTR == n0) {
        if ((Bsp_Time_GetMs() - t0) > 10u) {
            return -1;
        }
    }

    /* Copy both free-running buffers. At idle the values are near-constant, so a
     * torn triplet is negligible; the ADC2 filter averages five sets anyway. */
    for (i = 0; i < ADC1_SCAN_LEN; i++) {
        a1[i] = s_adc1_buf[i];
    }
    for (i = 0; i < ADC2_BUF_LEN; i++) {
        a2[i] = s_adc2_buf[i];
    }

    for (i = 0; i < ADC1_SCAN_LEN; i++) {
        s_adc1_scan[i] = a1[i];
    }

    /* OEM telemetry filter over the five interleaved {ch6,ch7,ch14} sets. */
    for (c = 0; c < ADC2_TRIPLE; c++) {
        uint32_t sum = 0u;
        for (i = 0; i < ADC2_SETS; i++) {
            sum += a2[i * ADC2_TRIPLE + c];
        }
        if (!s_adc2_seeded) {
            s_adc2_filt[c] = (float)sum / (float)ADC2_SETS;
        } else {
            s_adc2_filt[c] =
                ADC2_FILT_Y * s_adc2_filt[c] + ADC2_FILT_SUM * (float)sum;
        }
    }
    s_adc2_seeded = true;

    /* Overrun / DMA-error accounting. */
    if (ADC1->SR & ADC_SR_OVR) {
        s_overruns++;
        ADC1->SR &= ~ADC_SR_OVR;
    }
    if (ADC2->SR & ADC_SR_OVR) {
        s_overruns++;
        ADC2->SR &= ~ADC_SR_OVR;
    }
    lisr = DMA2->LISR;
    if (lisr & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF2 |
                DMA_LISR_DMEIF2)) {
        s_dma_errors++;
        DMA2->LIFCR = DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF2 |
                      DMA_LIFCR_CDMEIF2;
    }

    /* Publish the unified snapshot for the optional callback. */
    s_unified[DRV_ADC_MON_FWD]     = s_adc1_scan[0];
    s_unified[DRV_ADC_MON_REFL]    = s_adc1_scan[1];
    s_unified[DRV_ADC_MON_EXT_ALC] = s_adc1_scan[2];
    s_unified[DRV_ADC_MON_VEXT]    = (uint16_t)(s_adc2_filt[0] + 0.5f);
    s_unified[DRV_ADC_MON_PA_CURR] = (uint16_t)(s_adc2_filt[1] + 0.5f);
    s_unified[DRV_ADC_MON_PA_NTC]  = (uint16_t)(s_adc2_filt[2] + 0.5f);

    if (s_scan_callback != 0) {
        s_scan_callback(s_unified, DRV_ADC_MON_CH_COUNT);
    }
    return 0;
}

int Drv_AdcMon_ReadRaw(DrvAdcMonChannel ch, uint16_t *raw) {
    if (raw == 0 || (unsigned)ch >= (unsigned)DRV_ADC_MON_CH_COUNT) {
        return -1;
    }
    switch (ch) {
    case DRV_ADC_MON_FWD:
    case DRV_ADC_MON_REFL:
    case DRV_ADC_MON_EXT_ALC:
        *raw = s_adc1_scan[ch];
        return 0;
    case DRV_ADC_MON_VEXT:
        *raw = (uint16_t)(s_adc2_filt[0] + 0.5f);
        return 0;
    case DRV_ADC_MON_PA_CURR:
        *raw = (uint16_t)(s_adc2_filt[1] + 0.5f);
        return 0;
    case DRV_ADC_MON_PA_NTC:
        *raw = (uint16_t)(s_adc2_filt[2] + 0.5f);
        return 0;
    default:
        return -1;
    }
}

int Drv_AdcMon_ReadVextMv(uint16_t *mv) {
    float v;

    if (mv == 0) {
        return -1;
    }
    v = s_adc2_filt[0] * DRV_ADC_MON_VEXT_SCALE;
    if (v < 0.0f) {
        v = 0.0f;
    } else if (v > 65535.0f) {
        v = 65535.0f;
    }
    *mv = (uint16_t)(v + 0.5f);
    return 0;
}

/* PROVISIONAL PA temp from ch14/PC4 (UNVALIDATED NTC model; see header). The
 * node = Vpullup * R / (Rpull + R) with a separate 5 V pull-up rail, so the
 * reading is not ratiometric to Vref. raw -> Vnode -> Rntc -> 1/T = 1/T0 +
 * ln(R/R25)/B. Not a verified measurement; the probe asks only whether PC4
 * MOVES with PA heat under RF (M5c). */
int Drv_AdcMon_ReadPaTemp(int32_t *temp_mc) {
    uint16_t raw = (uint16_t)(s_adc2_filt[2] + 0.5f);
    float    v_node_mv;
    float    r_ntc;
    float    t_c;

    if (temp_mc == 0) {
        return -1;
    }
    if (raw == 0u || raw >= (DRV_ADC_MON_RAW_FULL_SCALE - 1u)) {
        return -1;
    }
    v_node_mv = (float)raw * (float)DRV_ADC_MON_VREF_MV /
                (float)DRV_ADC_MON_RAW_FULL_SCALE;
    if (v_node_mv >= (float)DRV_ADC_MON_NTC_V_PULLUP_MV) {
        return -1;
    }
    r_ntc = (float)DRV_ADC_MON_NTC_R_PULLUP_OHM * v_node_mv /
            ((float)DRV_ADC_MON_NTC_V_PULLUP_MV - v_node_mv);
    t_c = 1.0f / (1.0f / DRV_ADC_MON_NTC_T0_K +
                  logf(r_ntc / (float)DRV_ADC_MON_NTC_R25_OHM) /
                      (float)DRV_ADC_MON_NTC_BETA_K) -
          DRV_ADC_MON_NTC_KELVIN_TO_C;
    *temp_mc = (int32_t)(t_c * 1000.0f);
    return 0;
}

/* Inverse of the provisional model: the raw code whose divider node equals the
 * given temperature. The NTC code FALLS as temperature rises. */
int Drv_AdcMon_PaRawFromTempMc(int32_t temp_mc, uint16_t *raw) {
    float t_k;
    float r_ntc;
    float v_node_mv;
    float code;

    if (raw == 0) {
        return -1;
    }
    t_k = (float)temp_mc / 1000.0f + DRV_ADC_MON_NTC_KELVIN_TO_C;
    r_ntc = (float)DRV_ADC_MON_NTC_R25_OHM *
            expf((float)DRV_ADC_MON_NTC_BETA_K *
                 (1.0f / t_k - 1.0f / DRV_ADC_MON_NTC_T0_K));
    v_node_mv = (float)DRV_ADC_MON_NTC_V_PULLUP_MV * r_ntc /
                ((float)DRV_ADC_MON_NTC_R_PULLUP_OHM + r_ntc);
    code = v_node_mv * (float)DRV_ADC_MON_RAW_FULL_SCALE /
           (float)DRV_ADC_MON_VREF_MV;
    if (code < 1.0f || code >= (float)(DRV_ADC_MON_RAW_FULL_SCALE - 1u)) {
        return -1;
    }
    *raw = (uint16_t)(code + 0.5f);
    return 0;
}

int Drv_AdcMon_SetScanCallback(DrvAdcMonScanCallback cb) {
    uint32_t irq = Bsp_Irq_SaveLow();

    s_scan_callback = cb;
    Bsp_Irq_RestoreLow(irq);
    return 0;
}

void Drv_AdcMon_TakeStats(DrvAdcMonStats *out) {
    uint32_t irq;
    uint32_t now;

    if (out == 0) {
        return;
    }
    irq = Bsp_Irq_SaveLow();
    /* The free-running DMA has no completion ISR to count hardware scans, so
     * estimate scans from elapsed CPU cycles at each stats read. Doing it here
     * (rather than in Sample(), which runs at irregular 20 ms / 0.5 s rates)
     * keeps s_scans accruing smoothly per caller tick, so app_protect's
     * per-tick DMA-health math sees expected ~= actual (no false "missed"
     * counts). A genuine DMA stall is caught by Sample() returning -1 on a
     * frozen NDTR, not here. */
    now = Bsp_Time_GetCycles();
    if (s_scan_period_cyc == 0u) {
        /* (hz/1000)*us/1000 stays in uint32 (no 64-bit divide). */
        s_scan_period_cyc =
            (Bsp_Time_GetCpuHz() / 1000u) * DRV_ADC_MON_SCAN_PERIOD_US / 1000u;
        if (s_scan_period_cyc == 0u) {
            s_scan_period_cyc = 1u;
        }
    }
    if (s_have_last_scan) {
        s_scans += (now - s_last_scan_cycles) / s_scan_period_cyc;
    } else {
        s_have_last_scan = true;
    }
    s_last_scan_cycles = now;

    out->scans = s_scans;
    out->overruns = s_overruns;
    out->dma_errors = s_dma_errors;
    out->max_gap_cycles = s_max_gap_cycles; /* always 0: continuous DMA */
    Bsp_Irq_RestoreLow(irq);
}
