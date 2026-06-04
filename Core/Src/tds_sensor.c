#pragma GCC optimize("Os")
#include "tds_sensor.h"
#include "main.h"

/* TDS_DRIVE (PA6): toggle AC square wave before ADC sample to avoid
 * electrolysis / electrode polarisation.
 */

static uint16_t TDS_ReadADC(TDS_Handle_t *htds)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    /* HAL_ADC_ConfigChannel only ORs the channel bit into CHSELR; it never
     * clears previously-selected channels. With CH2/CH3/CH7 all enabled from
     * MX_ADC_Init, the F030 forward scanner would convert the lowest selected
     * channel instead of ours. Clear CHSELR so ONLY CH2 (PA2/TDS) is read. */
    htds->hadc->Instance->CHSELR = 0U;
    sConfig.Channel      = ADC_CHANNEL_2;
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(htds->hadc, &sConfig);

    HAL_ADC_Start(htds->hadc);
    HAL_ADC_PollForConversion(htds->hadc, 100);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(htds->hadc);
    HAL_ADC_Stop(htds->hadc);
    return val;
}

void TDS_Init(TDS_Handle_t *htds, ADC_HandleTypeDef *hadc)
{
    htds->hadc     = hadc;
    htds->last_ppm = 0;
    htds->valid    = 0;
    HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_RESET);
}

uint16_t TDS_ReadPPM(TDS_Handle_t *htds, int16_t temp_x10)
{
    /* AC-drive: 10 toggle cycles before sample */
    for (uint8_t i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    uint16_t adc_raw = TDS_ReadADC(htds);
    HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_RESET);

    /* Voltage in mV */
    uint32_t voltage_mv = (uint32_t)adc_raw * TDS_ADC_VREF_MV / TDS_ADC_RESOLUTION;

    /* Temperature compensation: coeff = 1.0 + 0.02*(T - 25)
     * Use integer: coeff_x100 = 100 + 2*(temp_c - 25)
     * where temp_c = temp_x10 / 10.
     * coeff_x100 = 100 + (temp_x10 - 250) / 5
     */
    int32_t temp_c10 = (int32_t)temp_x10;
    int32_t coeff_x1000 = 1000 + (temp_c10 - 250) * 2; /* = 1000*(1 + 0.02*(T-25)) */
    if (coeff_x1000 <= 0) coeff_x1000 = 1;

    /* Compensated voltage in mV × 1000 / coeff → keep in mV */
    uint32_t comp_mv = (voltage_mv * 1000U) / (uint32_t)coeff_x1000;

    /* DFRobot empirical formula, fixed-point only:
     * ppm = (133.42*V^3 - 255.86*V^2 + 857.39*V) * 0.5
     * V = comp_mv / 1000.
     *
     * ppm_x1000 = 66710*mV^3/1e9 - 127930*mV^2/1e6 + 428695*mV/1000
     * Using int64 avoids Cortex-M0 soft-float helpers and keeps accuracy.
     */
    int64_t m  = (int64_t)comp_mv;
    int64_t m2 = m * m;
    int64_t m3 = m2 * m;

    int64_t ppm_x1000 = ((66710LL  * m3) / 1000000000LL)
                      - ((127930LL * m2) / 1000000LL)
                      + ((428695LL * m)  / 1000LL);

    if (ppm_x1000 < 0) ppm_x1000 = 0;
    if (ppm_x1000 > 65535000LL) ppm_x1000 = 65535000LL;

    htds->last_ppm = (uint16_t)((ppm_x1000 + 500LL) / 1000LL);
    htds->valid    = 1;
    return htds->last_ppm;
}

uint8_t TDS_IsAboveThreshold(TDS_Handle_t *htds, uint16_t threshold_ppm, int16_t temp_x10)
{
    return (TDS_ReadPPM(htds, temp_x10) > threshold_ppm) ? 1U : 0U;
}
