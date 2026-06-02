#pragma GCC optimize("Os")
#include "battery.h"
#include "main.h"

static uint16_t Battery_ReadADC(Battery_Handle_t *hbat)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    /* Clear CHSELR first — otherwise CH2/CH3 left selected by the TDS/NTC
     * drivers would be converted ahead of CH7 (F030 scans lowest-first),
     * making the battery read another sensor's pin. Select ONLY CH7 (PA7). */
    hbat->hadc->Instance->CHSELR = 0U;
    sConfig.Channel      = ADC_CHANNEL_7;
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(hbat->hadc, &sConfig);

    HAL_ADC_Start(hbat->hadc);
    HAL_ADC_PollForConversion(hbat->hadc, 100);
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
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += Battery_ReadADC(hbat);
    }
    uint16_t adc_avg = (uint16_t)(sum >> 2);

    /* v_adc_mv = adc_avg * 3300 / 4096
     * voltage_mv = v_adc_mv * BAT_DIVIDER_RATIO_X10 / 10
     * Combined: voltage_mv = adc_avg * 3300 * BAT_DIVIDER_RATIO_X10 / (4096 * 10)
     */
    uint32_t v_adc_mv  = (uint32_t)adc_avg * BAT_ADC_VREF_MV / BAT_ADC_RESOLUTION;
    uint32_t voltage   = v_adc_mv * BAT_DIVIDER_RATIO_X10 / 10U;

    if (voltage > BAT_FULL_MV)  voltage = BAT_FULL_MV;
    if (voltage < BAT_EMPTY_MV) voltage = BAT_EMPTY_MV;
    hbat->voltage_mv = (uint16_t)voltage;

    uint32_t range   = BAT_FULL_MV - BAT_EMPTY_MV;
    hbat->percent    = (uint8_t)(((uint32_t)(hbat->voltage_mv - BAT_EMPTY_MV) * 100U) / range);

    uint8_t chrg  = (HAL_GPIO_ReadPin(CHRG_STAT_GPIO_Port,  CHRG_STAT_Pin)  == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t stdby = (HAL_GPIO_ReadPin(STDBY_STAT_GPIO_Port, STDBY_STAT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (stdby) {
        hbat->status  = BAT_STATUS_FULL;
        hbat->percent = 100;
    } else if (chrg) {
        hbat->status = BAT_STATUS_CHARGING;
    } else {
        hbat->status = BAT_STATUS_DISCHARGING;
    }
    hbat->valid = 1;
}

uint8_t Battery_GetPercent(Battery_Handle_t *hbat)
{
    if (!hbat->valid) Battery_Update(hbat);
    return hbat->percent;
}

uint8_t Battery_IsCharging(Battery_Handle_t *hbat) { return (hbat->status == BAT_STATUS_CHARGING)    ? 1U : 0U; }
uint8_t Battery_IsFull(Battery_Handle_t *hbat)     { return (hbat->status == BAT_STATUS_FULL)         ? 1U : 0U; }
uint8_t Battery_IsLow(Battery_Handle_t *hbat)      { return (hbat->percent <= BAT_LOW_THRESHOLD_PCT)  ? 1U : 0U; }
