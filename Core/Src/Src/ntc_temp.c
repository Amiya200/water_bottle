/* ============================================================================
 * ntc_temp.c — NTC thermistor driver  [DEBUG BUILD]
 * Target : STM32F030K6T6 @ 48 MHz
 *
 * Circuit: 3V3 ── R22(10kΩ) ── PA3/ADC_CH3 ── U11 10K-NTC ── GND
 *
 * DEBUG globals (add ALL to STM32CubeIDE Live Expressions):
 *   g_ntc_raw_adc       → raw 12-bit ADC count (0-4095)
 *   g_ntc_adc_avg       → averaged ADC (4 samples)
 *   g_ntc_lut_idx       → which LUT segment was selected
 *   g_ntc_frac          → interpolation fraction (0-100)
 *   g_ntc_temp_x10      → final temperature ×10  (e.g. 253 = 25.3 °C)
 *   g_ntc_fault         → fault code (see enum below)
 *   g_ntc_chselr_before → CHSELR value BEFORE our clear
 *   g_ntc_chselr_after  → CHSELR value AFTER clear+config (must be 0x08)
 *
 * Fault codes  (g_ntc_fault):
 *   0 = OK
 *   1 = adc_avg < 10     → NTC open circuit or PA3 floating
 *   2 = adc_avg > 4085   → NTC/pin shorted to 3V3
 *   3 = idx clamped      → temperature above 80 °C (LUT max)
 *   4 = frac out of range
 *   5 = HAL_ADC_PollForConversion timeout
 * ============================================================================
 */
#pragma GCC optimize("Os")

#include "ntc_temp.h"

/* ── Debug globals — watch ALL of these in Live Expressions ─────────────── */
volatile uint16_t g_ntc_raw_adc        = 0;
volatile uint16_t g_ntc_adc_avg        = 0;
volatile uint8_t  g_ntc_lut_idx        = 0;
volatile int32_t  g_ntc_frac           = 0;
volatile int16_t  g_ntc_temp_x10       = 0;
volatile uint8_t  g_ntc_fault          = 0;
volatile uint32_t g_ntc_chselr_before  = 0;
volatile uint32_t g_ntc_chselr_after   = 0;

/* ---------------------------------------------------------------------------
 * LUT: ADC vs temperature
 * Formula: ADC = round(4096 * R_NTC / (10000 + R_NTC))
 *          R_NTC = 10000 * exp(3950*(1/(T+273.15) - 1/298.15))
 * Range: -5 to 80 °C, step 5 °C, 18 entries
 *
 * Index   Temp(°C)   ADC
 *   0       -5       3338
 *   1        0       3155
 *   2        5       2959
 *   3       10       2738
 *   4       15       2509
 *   5       20       2275
 *   6       25       2048   ← 25°C room temp should read ~2048
 *   7       30       1828
 *   8       35       1614
 *   9       40       1413
 *  10       45       1238
 *  11       50       1082
 *  12       55        937
 *  13       60        816
 *  14       65        708
 *  15       70        613
 *  16       75        531
 *  17       80        461
 * -------------------------------------------------------------------------*/
static const uint16_t s_ntc_lut[NTC_LUT_ENTRIES] = {
    3338, 3155, 2959, 2738, 2509, 2275, 2048, 1828,
    1614, 1413, 1238, 1082,  937,  816,  708,  613,
     531,  461
};

/* ===========================================================================
 * Internal: read one ADC sample from CH3 only
 * ===========================================================================
 */
static uint16_t NTC_ReadADC(NTC_Handle_t *hntc)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Snapshot CHSELR before clearing so you can see what was there */
    g_ntc_chselr_before = hntc->hadc->Instance->CHSELR;

    /* ── CRITICAL FIX ────────────────────────────────────────────────────
     * HAL_ADC_ConfigChannel does CHSELR |= channel_bit, never clears.
     * After MX_ADC_Init, CH2+CH3+CH7 are ALL set.
     * Forward-scan returns CH2 first → wrong pin → garbage temperature.
     * Write 0 here so ONLY CH3 is selected for this conversion.          */
    hntc->hadc->Instance->CHSELR = 0U;

    sConfig.Channel      = ADC_CHANNEL_3;             /* PA3 = NTC_TEMP     */
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; /* max time for 100nF */
    HAL_ADC_ConfigChannel(hntc->hadc, &sConfig);

    /* Should be 0x00000008 (bit 3 = CH3) after this */
    g_ntc_chselr_after = hntc->hadc->Instance->CHSELR;

    HAL_ADC_Start(hntc->hadc);

    if (HAL_ADC_PollForConversion(hntc->hadc, 10U) != HAL_OK) {
        HAL_ADC_Stop(hntc->hadc);
        g_ntc_fault    = 5;
        g_ntc_raw_adc  = 9999;   /* sentinel: timeout */
        return 2048U;
    }

    uint16_t val = (uint16_t)HAL_ADC_GetValue(hntc->hadc);
    HAL_ADC_Stop(hntc->hadc);

    g_ntc_raw_adc = val;
    return val;
}

/* ===========================================================================
 * NTC_Init
 * ===========================================================================
 */
void NTC_Init(NTC_Handle_t *hntc, ADC_HandleTypeDef *hadc)
{
    hntc->hadc          = hadc;
    hntc->last_temp_x10 = 250;    /* 25.0 °C default */
    hntc->valid         = 0;
}

/* ===========================================================================
 * NTC_ReadTemp_x10
 * Returns temperature in tenths of °C.
 * ===========================================================================
 */
int16_t NTC_ReadTemp_x10(NTC_Handle_t *hntc)
{
    g_ntc_fault = 0;

    /* ── 1. Average 4 samples ────────────────────────────────────────── */
    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < 4U; i++) {
        sum += NTC_ReadADC(hntc);
        HAL_Delay(2);
    }
    uint16_t adc_avg = (uint16_t)(sum >> 2);
    g_ntc_adc_avg = adc_avg;

    /* ── 2. Fault detection ──────────────────────────────────────────── */
    if (adc_avg < 10U) {
        g_ntc_fault = 1;   /* open / floating */
        hntc->valid = 0;
        return hntc->last_temp_x10;
    }
    if (adc_avg > (NTC_ADC_RESOLUTION - 11U)) {
        g_ntc_fault = 2;   /* shorted to Vcc */
        hntc->valid = 0;
        return hntc->last_temp_x10;
    }

    /* ── 3. LUT search ───────────────────────────────────────────────── */
    uint8_t idx = 0U;
    for (; idx < (NTC_LUT_ENTRIES - 1U); idx++) {
        if (adc_avg >= s_ntc_lut[idx + 1U]) {
            break;
        }
    }

    if (idx >= (NTC_LUT_ENTRIES - 1U)) {
        idx = NTC_LUT_ENTRIES - 2U;
        g_ntc_fault = 3;
    }
    g_ntc_lut_idx = idx;

    /* ── 4. Linear interpolation ─────────────────────────────────────── */
    int32_t t_lo_x10  = ((int32_t)NTC_LUT_TEMP_MIN
                         + (int32_t)idx        * (int32_t)NTC_LUT_TEMP_STEP) * 10;
    int32_t t_hi_x10  = ((int32_t)NTC_LUT_TEMP_MIN
                         + (int32_t)(idx + 1U) * (int32_t)NTC_LUT_TEMP_STEP) * 10;

    int32_t adc_lo    = (int32_t)s_ntc_lut[idx];
    int32_t adc_hi    = (int32_t)s_ntc_lut[idx + 1U];
    int32_t adc_range = adc_lo - adc_hi;

    if (adc_range == 0) {
        hntc->last_temp_x10 = (int16_t)t_lo_x10;
        hntc->valid         = 1;
        g_ntc_temp_x10      = hntc->last_temp_x10;
        return hntc->last_temp_x10;
    }

    int32_t frac = ((adc_lo - (int32_t)adc_avg) * 100) / adc_range;

    if (frac < 0 || frac > 100) {
        g_ntc_fault = (g_ntc_fault == 0) ? 4U : g_ntc_fault;
    }
    if (frac <   0) { frac =   0; }
    if (frac > 100) { frac = 100; }

    g_ntc_frac = frac;

    int32_t temp_x10 = t_lo_x10 + ((t_hi_x10 - t_lo_x10) * frac) / 100;

    hntc->last_temp_x10 = (int16_t)temp_x10;
    hntc->valid         = 1;
    g_ntc_temp_x10      = hntc->last_temp_x10;

    return hntc->last_temp_x10;
}

/* ===========================================================================
 * NTC_GetTempCelsius — returns float °C (avoid in ISRs)
 * ===========================================================================
 */
float NTC_GetTempCelsius(NTC_Handle_t *hntc)
{
    return (float)NTC_ReadTemp_x10(hntc) / 10.0f;
}

/* ===========================================================================
 * NTC_IsAboveThreshold
 * ===========================================================================
 */
uint8_t NTC_IsAboveThreshold(NTC_Handle_t *hntc, int16_t threshold_x10)
{
    return (NTC_ReadTemp_x10(hntc) > threshold_x10) ? 1U : 0U;
}
