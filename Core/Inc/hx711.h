#ifndef HX711_H
#define HX711_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* Bit-bang via GPIO:
 *   HX711_DOUT  → PA4 (input)
 *   HX711_SCK   → PA5 (output)
 */

#define HX711_TIMEOUT_MS    500    /* max wait for DOUT LOW */
#define HX711_GAIN_128      1      /* channel A gain 128 (default) */
#define HX711_GAIN_32       2      /* channel B gain 32            */
#define HX711_GAIN_64       3      /* channel A gain 64            */

/* Grams-per-ADC-count — calibrate on first use */
#define HX711_DEFAULT_SCALE  440.0f   /* placeholder */

typedef struct {
    int32_t  tare_offset;     /* raw ADC reading when empty */
    float    scale;           /* grams per raw unit         */
    uint8_t  gain_pulses;     /* 1=128, 2=32, 3=64          */
    uint8_t  is_calibrated;
} HX711_Handle_t;

void    HX711_Init(HX711_Handle_t *hx);
uint8_t HX711_IsReady(void);
int32_t HX711_ReadRaw(HX711_Handle_t *hx);
float   HX711_ReadGrams(HX711_Handle_t *hx);
float   HX711_ReadMillilitres(HX711_Handle_t *hx); /* water ≈ 1 g/ml */
void    HX711_Tare(HX711_Handle_t *hx);
void    HX711_SetScale(HX711_Handle_t *hx, float scale);
void    HX711_PowerDown(void);
void    HX711_PowerUp(void);

#endif /* HX711_H */
