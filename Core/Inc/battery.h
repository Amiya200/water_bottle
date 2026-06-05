#ifndef BATTERY_H
#define BATTERY_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* BAT_     → PA7 (ADC_IN7) — voltage divider
 * CHRG_STAT→ PB0 (input)   — TP4056: LOW = charging
 * STDBY_STAT→PB1 (input)   — TP4056: LOW = full/standby
 *
 * BAT_DIVIDER_RATIO as integer × 10  (e.g. 20 = 2.0×)
 * Adjust to match schematic (R1+R2)/R2.
 */

#define BAT_DIVIDER_RATIO_X10   20U    /* 2.0 × 10 — eliminates float multiply */
#define BAT_ADC_VREF_MV         3300U
#define BAT_ADC_RESOLUTION      4096U
#define BAT_FULL_MV             4200U
#define BAT_EMPTY_MV            3000U
#define BAT_LOW_THRESHOLD_PCT   10U

typedef enum {
    BAT_STATUS_DISCHARGING = 0,
    BAT_STATUS_CHARGING,
    BAT_STATUS_FULL,
} Battery_Status_t;

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint8_t  percent;
    uint16_t voltage_mv;
    uint8_t  status;        /* Battery_Status_t stored as byte */
    uint8_t  valid;
} Battery_Handle_t;

void    Battery_Init(Battery_Handle_t *hbat, ADC_HandleTypeDef *hadc);
void    Battery_Update(Battery_Handle_t *hbat);
uint8_t Battery_GetPercent(Battery_Handle_t *hbat);
uint16_t Battery_GetVoltageMv(Battery_Handle_t *hbat);
uint8_t Battery_IsCharging(Battery_Handle_t *hbat);
uint8_t Battery_IsFull(Battery_Handle_t *hbat);
uint8_t Battery_IsLow(Battery_Handle_t *hbat);

#endif /* BATTERY_H */
