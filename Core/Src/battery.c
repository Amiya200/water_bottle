#pragma GCC optimize("Os")
#include "battery.h"
#include "main.h"

/* ============================================================================
 * battery.c — single-cell Li-ion fuel gauge (voltage based)
 *
 * Hardware (schematic page 5):
 *   BAT+ ── R9(100k) ──+── R10(100k) ── GND      divider = 2.0×
 *                      └── PA7 / ADC_IN7
 *   TP4056 CHRG#  → PB0 (LOW while charging)
 *   TP4056 STDBY# → PB1 (LOW when full)
 *
 * Why this is more than "linear 3.0–4.2 V":
 *   A Li-ion discharge curve is strongly non-linear — most of the usable
 *   capacity sits in a flat 3.6–3.9 V plateau. A linear map reads ~50 % when
 *   the cell is really ~80 % full. A small open-circuit-voltage LUT gives a
 *   far more honest percentage for the LED battery bar (FRD §5.7).
 *
 *   While the charger is attached the pack voltage is pulled up by the charge
 *   current, so the raw voltage is NOT a valid SoC. In that case we hold the
 *   last discharge-based estimate (or report 100 % on STDBY) and let the
 *   charging LED bar reflect progress, exactly as the FRD describes.
 * ==========================================================================*/

/* Open-circuit voltage (mV) → state-of-charge (%) for a typical 3.7 V Li-ion.
 * 11 points, 10 % apart, high→low. Interpolated between points. */
static const uint16_t s_ocv_mv[11] = {
    4200, 4060, 3980, 3920, 3870, 3820, 3790, 3770, 3730, 3690, 3000
};
static const uint8_t s_ocv_pct[11] = {
     100,   90,   80,   70,   60,   50,   40,   30,   20,   10,    0
};

static uint8_t Battery_PctFromMv(uint16_t mv)
{
    if (mv >= s_ocv_mv[0])  return 100U;
    if (mv <= s_ocv_mv[10]) return 0U;
    for (uint8_t i = 0U; i < 10U; i++) {
        uint16_t hi = s_ocv_mv[i];
        uint16_t lo = s_ocv_mv[i + 1U];
        if (mv <= hi && mv >= lo) {
            uint16_t span = (uint16_t)(hi - lo);
            uint8_t  p_hi = s_ocv_pct[i];
            uint8_t  p_lo = s_ocv_pct[i + 1U];
            if (span == 0U) return p_lo;
            /* linear within the segment */
            uint32_t frac = (uint32_t)(mv - lo) * (uint32_t)(p_hi - p_lo);
            return (uint8_t)(p_lo + (frac / span));
        }
    }
    return 0U;
}

static uint16_t Battery_ReadADC(Battery_Handle_t *hbat)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    /* Clear CHSELR first — otherwise CH2/CH3 left selected by the TDS/NTC
     * drivers would be converted ahead of CH7 (F030 scans lowest-first),
     * making the battery read another sensor's pin. Select ONLY CH7 (PA7). */
    hbat->hadc->Instance->CHSELR = 0U;
    sConfig.Channel      = ADC_CHANNEL_7;
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  /* long settle: 100k divider is high-Z */
    HAL_ADC_ConfigChannel(hbat->hadc, &sConfig);

    HAL_ADC_Start(hbat->hadc);
    if (HAL_ADC_PollForConversion(hbat->hadc, 100) != HAL_OK) {
        HAL_ADC_Stop(hbat->hadc);
        return 0U;
    }
    uint16_t val = (uint16_t)HAL_ADC_GetValue(hbat->hadc);
    HAL_ADC_Stop(hbat->hadc);
    return val;
}

void Battery_Init(Battery_Handle_t *hbat, ADC_HandleTypeDef *hadc)
{
    hbat->hadc       = hadc;
    hbat->percent    = 100;
    hbat->voltage_mv = BAT_FULL_MV;
    hbat->status     = BAT_STATUS_DISCHARGING;
    hbat->valid      = 0;
}

void Battery_Update(Battery_Handle_t *hbat)
{
    /* Median-of-8 to reject the worst ADC outliers, then average the middle. */
    uint16_t s[8];
    for (uint8_t i = 0; i < 8; i++) s[i] = Battery_ReadADC(hbat);
    /* simple insertion sort */
    for (uint8_t i = 1; i < 8; i++) {
        uint16_t k = s[i]; int8_t j = (int8_t)(i - 1);
        while (j >= 0 && s[(uint8_t)j] > k) { s[(uint8_t)(j + 1)] = s[(uint8_t)j]; j--; }
        s[(uint8_t)(j + 1)] = k;
    }
    uint32_t adc_avg = ((uint32_t)s[2] + s[3] + s[4] + s[5]) >> 2;  /* middle 4 */

    /* voltage_mv = adc * Vref / 4096 * divider_ratio */
    uint32_t v_adc_mv = adc_avg * BAT_ADC_VREF_MV / BAT_ADC_RESOLUTION;
    uint32_t voltage  = v_adc_mv * BAT_DIVIDER_RATIO_X10 / 10U;

    if (voltage > BAT_FULL_MV)  voltage = BAT_FULL_MV;
    if (voltage < BAT_EMPTY_MV) voltage = BAT_EMPTY_MV;
    hbat->voltage_mv = (uint16_t)voltage;

    uint8_t chrg  = (HAL_GPIO_ReadPin(CHRG_STAT_GPIO_Port,  CHRG_STAT_Pin)  == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t stdby = (HAL_GPIO_ReadPin(STDBY_STAT_GPIO_Port, STDBY_STAT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (stdby) {
        hbat->status  = BAT_STATUS_FULL;
        hbat->percent = 100U;
    } else if (chrg) {
        hbat->status = BAT_STATUS_CHARGING;
        /* Charging voltage is elevated by charge current — not a valid SoC.
         * Hold the last discharge estimate (never let it drop while charging). */
        uint8_t est = Battery_PctFromMv(hbat->voltage_mv);
        if (est > hbat->percent) hbat->percent = est;
    } else {
        hbat->status = BAT_STATUS_DISCHARGING;
        uint8_t est = Battery_PctFromMv(hbat->voltage_mv);
        if (!hbat->valid) {
            hbat->percent = est;
        } else {
            /* IIR smoothing (3:1) to stop the bar jittering on load spikes,
             * and never let it climb while discharging. */
            uint16_t blended = (uint16_t)(((uint16_t)hbat->percent * 3U + est) >> 2);
            hbat->percent = (uint8_t)(blended > hbat->percent ? hbat->percent : blended);
        }
    }
    hbat->valid = 1;
}

uint8_t Battery_GetPercent(Battery_Handle_t *hbat)
{
    if (!hbat->valid) Battery_Update(hbat);
    return hbat->percent;
}

uint16_t Battery_GetVoltageMv(Battery_Handle_t *hbat)
{
    if (!hbat->valid) Battery_Update(hbat);
    return hbat->voltage_mv;
}

uint8_t Battery_IsCharging(Battery_Handle_t *hbat) { return (hbat->status == BAT_STATUS_CHARGING)    ? 1U : 0U; }
uint8_t Battery_IsFull(Battery_Handle_t *hbat)     { return (hbat->status == BAT_STATUS_FULL)         ? 1U : 0U; }
uint8_t Battery_IsLow(Battery_Handle_t *hbat)      { return (hbat->percent <= BAT_LOW_THRESHOLD_PCT) && !Battery_IsCharging(hbat) && !Battery_IsFull(hbat); }
