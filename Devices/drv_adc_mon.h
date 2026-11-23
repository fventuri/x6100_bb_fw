#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * X6100 (STM32F427) analog monitor: two independent regular-scan ADC groups,
 * each free-running into a circular DMA buffer with NO ADC interrupt, exactly
 * as OEM BASE 1.1.9 arranges them (x6100-base/docs/analog.md, VERIFIED):
 *
 *   ADC1  ch0/PA0, ch1/PA1, ch2/PA2   -> DMA2 Stream0 ch0, 3 halfwords
 *         ch0 = forward RF detector, ch1 = reflected RF detector,
 *         ch2 = ACC ALC_V correction input.  Read by the fast TX path.
 *
 *   ADC2  ch6/PA6, ch7/PA7, ch14/PC4  -> DMA2 Stream2 ch1, 15 halfwords
 *         five interleaved {ch6,ch7,ch14} triplets.  ch6 = external supply,
 *         ch7 = PA current feedback, ch14/PC4 = unresolved OEM role (probed
 *         here as a PA-temperature candidate).  Slow telemetry.
 *
 * Drv_AdcMon_Init() enables the clocks, configures both DMA streams and both
 * ADCs, and starts continuous conversions. The DMA keeps the buffers fresh on
 * its own; Drv_AdcMon_Sample() snapshots the ADC1 triplet and runs the OEM
 * ADC2 telemetry filter (y = 0.92*y + 0.08*mean(5), analog.md VERIFIED) over
 * the 15-halfword buffer, then the Read* accessors return the latest values.
 *
 * All functions return 0 on success or a negative error code.
 */

/*
 * ADC1 scan period. ADCCLK = PCLK2 (125 MHz) / 8 = 15.625 MHz; each channel is
 * 480 sampling + 12 conversion = 492 cycles = 31.5 us; the 3-channel scan is
 * ~94.5 us. app_tx_monitor derives its EMA time constants from this value.
 */
#define DRV_ADC_MON_SCAN_PERIOD_US 95u

/*
 * Unified channel index. ADC1 channels occupy 0..2 so the TX-monitor scan
 * callback can read raw[FWD], raw[REFL], raw[EXT_ALC] directly from a snapshot;
 * the ADC2 channels follow. The OEM X6100 has NO dedicated PA thermistor (no NTC
 * in any doc or ADC channel); ch14/PC4 has an unresolved OEM role and is PROBED
 * here as a provisional PA-temp input under DRV_ADC_MON_PA_NTC -- see the NTC
 * model note below; the transfer function is a documented guess, not verified.
 */
typedef enum {
    DRV_ADC_MON_FWD = 0,   /* ADC1 ch0 / PA0 -- forward RF detector (raw) */
    DRV_ADC_MON_REFL,      /* ADC1 ch1 / PA1 -- reflected RF detector (raw) */
    DRV_ADC_MON_EXT_ALC,   /* ADC1 ch2 / PA2 -- ACC ALC_V correction (raw) */
    DRV_ADC_MON_VEXT,      /* ADC2 ch6 / PA6 -- external supply (filtered raw) */
    DRV_ADC_MON_PA_CURR,   /* ADC2 ch7 / PA7 -- PA current feedback (filtered) */
    DRV_ADC_MON_PA_NTC,    /* ADC2 ch14 / PC4 -- unresolved; provisional PA-temp */
    DRV_ADC_MON_CH_COUNT
} DrvAdcMonChannel;

/*
 * External-supply conversion. The OEM multiplies the filtered ADC2 ch6 code by
 * the VERIFIED firmware scale 4.835165 to get a millivolt-domain value (the
 * exact divider/gain is unresolved, so this constant is preserved rather than
 * recomputed; analog.md). Drv_AdcMon_ReadVextMv() returns that value in mV.
 */
#define DRV_ADC_MON_VEXT_SCALE 4.835165f

/* ADC reference, nominal board 3.3 V rail. */
#define DRV_ADC_MON_VREF_MV 3300u

/*
 * PROVISIONAL PA-NTC model for ch14/PC4. The X6100 has no documented PA
 * thermistor; Franco's M5a decision is to probe PC4 as a candidate. These
 * parameters are an UNVALIDATED guess carried over from the X6200 PA NTC (NTC
 * to GND, pull-up to a separate 5 V rail, divider node into the ADC). The
 * absolute temperature is therefore not trustworthy yet; the point of the probe
 * is to see whether PC4 MOVES with PA heat under real RF (M5c). The thresholds
 * are generous (80 C trip maps to a raw code far below the ~920 idle code), so
 * the trip cannot false-fire at idle. Revisit / recalibrate or disarm in M5c.
 */
#define DRV_ADC_MON_NTC_R_PULLUP_OHM 100000u
#define DRV_ADC_MON_NTC_V_PULLUP_MV  5000u
#define DRV_ADC_MON_NTC_R25_OHM      10000u
#define DRV_ADC_MON_NTC_BETA_K       3950u
#define DRV_ADC_MON_NTC_T0_K         298.15f
#define DRV_ADC_MON_NTC_KELVIN_TO_C  273.15f
#define DRV_ADC_MON_RAW_FULL_SCALE   4096u

typedef struct {
    uint32_t scans;          /* successful Sample() refreshes since init */
    uint32_t overruns;       /* ADC overrun (OVR) events observed */
    uint32_t dma_errors;     /* DMA transfer/FIFO error flags observed */
    uint32_t max_gap_cycles; /* longest interval between Sample() calls */
} DrvAdcMonStats;

/*
 * Optional hook fed a unified DRV_ADC_MON_CH_COUNT-entry snapshot by
 * Drv_AdcMon_Sample() (indices FWD/REFL/EXT_ALC from the ADC1 triplet, the
 * ADC2 channels from the filtered states). Kept for app_tx_monitor; it must be
 * short. Not invoked from an ISR in this driver -- it runs in the Sample()
 * caller's context.
 */
typedef void (*DrvAdcMonScanCallback)(const volatile uint16_t *raw, size_t count);

int  Drv_AdcMon_Init(void);
int  Drv_AdcMon_Sample(void);
int  Drv_AdcMon_ReadRaw(DrvAdcMonChannel ch, uint16_t *raw);
int  Drv_AdcMon_ReadVextMv(uint16_t *mv);

/*
 * Provisional PA temperature from ch14/PC4 using the UNVALIDATED NTC model
 * above (see note). ReadPaTemp converts the filtered PC4 code to milli-Celsius;
 * PaRawFromTempMc is its inverse, turning a threshold into a raw code once so
 * app_protect compares codes in the hot path. Both return 0 on success, -1 if
 * the code maps outside the ADC span. NOT a verified measurement yet.
 */
int  Drv_AdcMon_ReadPaTemp(int32_t *temp_mc);
int  Drv_AdcMon_PaRawFromTempMc(int32_t temp_mc, uint16_t *raw);

int  Drv_AdcMon_SetScanCallback(DrvAdcMonScanCallback cb);
void Drv_AdcMon_TakeStats(DrvAdcMonStats *out);
