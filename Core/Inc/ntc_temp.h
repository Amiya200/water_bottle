/* ============================================================================
 * ntc_temp.h — NTC thermistor driver for HydraSense Smart Unit
 * Target: STM32F030K6T6 @ 48 MHz
 *
 * Circuit (confirmed from schematic):
 *   3V3 ── R22 (10 kΩ) ── NTC_TEMP (PA3 / ADC CH3) ── U11 10K-NTC-Therm ── GND
 *   C36 100 nF across NTC for noise filtering.
 *
 * Usage:
 *   NTC_Handle_t hntc;
 *   NTC_Init(&hntc, &hadc);
 *
 *   int16_t t = NTC_ReadTemp_x10(&hntc);   // e.g. 253 → 25.3 °C
 *   float   f = NTC_GetTempCelsius(&hntc); // e.g. 25.3f
 * ============================================================================
 */
#ifndef NTC_TEMP_H
#define NTC_TEMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * LUT configuration — must match s_ntc_lut[] in ntc_temp.c
 * ---------------------------------------------------------------------------
 * Range  : NTC_LUT_TEMP_MIN to (NTC_LUT_TEMP_MIN + (NTC_LUT_ENTRIES-1)*STEP)
 *        = -5 °C … 80 °C
 * Step   : 5 °C per entry
 * Entries: 18
 * ---------------------------------------------------------------------------
 */
#define NTC_LUT_TEMP_MIN     (-5)    /**< Lowest temperature in LUT (°C)    */
#define NTC_LUT_TEMP_MAX     ( 80)   /**< Highest temperature in LUT (°C)   */
#define NTC_LUT_TEMP_STEP    (  5)   /**< Step between entries (°C)         */
#define NTC_LUT_ENTRIES      ( 18U)  /**< Number of LUT entries             */

/* ADC resolution — 12-bit = 4096 counts */
#define NTC_ADC_RESOLUTION   (4096U)

/* ---------------------------------------------------------------------------
 * Handle structure
 * ---------------------------------------------------------------------------
 */
typedef struct {
    ADC_HandleTypeDef *hadc;         /**< Shared ADC peripheral handle      */
    int16_t            last_temp_x10;/**< Last valid reading (tenths of °C) */
    uint8_t            valid;        /**< 1 = last_temp_x10 is valid        */
} NTC_Handle_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------
 */

/**
 * @brief  Initialise the NTC driver.
 * @param  hntc  Pointer to NTC handle.
 * @param  hadc  Pointer to the ADC handle (shared with TDS / battery).
 * @note   Call after HAL_Init() and MX_ADC_Init().
 */
void    NTC_Init(NTC_Handle_t *hntc, ADC_HandleTypeDef *hadc);

/**
 * @brief  Read temperature.
 * @param  hntc  Pointer to NTC handle.
 * @retval Temperature in tenths of a degree Celsius.
 *         Example: 253 = 25.3 °C,  -12 = -1.2 °C
 * @note   Averages 4 ADC samples (8 ms total).
 *         On open/short fault returns last known-good value and clears valid.
 */
int16_t NTC_ReadTemp_x10(NTC_Handle_t *hntc);

/**
 * @brief  Convenience wrapper — returns temperature as float °C.
 * @param  hntc  Pointer to NTC handle.
 * @retval Temperature in degrees Celsius (float).
 * @note   Uses float division; avoid in tight ISR / low-power loops.
 *         Equivalent to (float)NTC_ReadTemp_x10(hntc) / 10.0f
 */
#ifdef NTC_ENABLE_FLOAT_CELSIUS
float   NTC_GetTempCelsius(NTC_Handle_t *hntc);
#endif

/**
 * @brief  Threshold comparator.
 * @param  hntc           Pointer to NTC handle.
 * @param  threshold_x10  Threshold in tenths of °C (e.g. 400 = 40.0 °C).
 * @retval 1 if current temperature > threshold, 0 otherwise.
 */
uint8_t NTC_IsAboveThreshold(NTC_Handle_t *hntc, int16_t threshold_x10);

#ifdef __cplusplus
}
#endif

#endif /* NTC_TEMP_H */
