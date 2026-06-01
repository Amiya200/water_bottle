#ifndef TDS_SENSOR_H
#define TDS_SENSOR_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* TDS_MCU   → PA2 (ADC_IN2)
 * TDS_DRIVE → PA6 (GPIO output, AC-drive to avoid polarisation)
 * Vref = 3.3 V, ADC 12-bit
 */

#define TDS_ADC_VREF_MV     3300U
#define TDS_ADC_RESOLUTION  4096U

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint16_t last_ppm;      /* PPM fits in uint16 (max ~1000 for drinking water) */
    uint8_t  valid;
} TDS_Handle_t;

void     TDS_Init(TDS_Handle_t *htds, ADC_HandleTypeDef *hadc);
uint16_t TDS_ReadPPM(TDS_Handle_t *htds, int16_t temp_x10);  /* temp_x10 = °C × 10 */
uint8_t  TDS_IsAboveThreshold(TDS_Handle_t *htds, uint16_t threshold_ppm, int16_t temp_x10);

#endif /* TDS_SENSOR_H */
