#pragma GCC optimize("Os")
#include "hx711.h"
#include "main.h"   /* for pin defines */
#include <stddef.h>

/* Bit-bang helpers — PA4=DOUT (input), PA5=SCK (output) */
#define DOUT_READ()   HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin)
#define SCK_HIGH()    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET)
#define SCK_LOW()     HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET)

/* HX711 needs ≥ 0.2 µs per SCK edge — a few NOPs satisfy this at 48 MHz */
#define HX711_DELAY()  do { __NOP(); __NOP(); __NOP(); __NOP(); } while(0)

void HX711_Init(HX711_Handle_t *hx)
{
    hx->tare_offset  = 0;
    hx->scale        = HX711_DEFAULT_SCALE;
    hx->gain_pulses  = HX711_GAIN_128;
    hx->is_calibrated = 0;
    SCK_LOW();
}

uint8_t HX711_IsReady(void)
{
    return (DOUT_READ() == GPIO_PIN_RESET) ? 1 : 0;
}

int32_t HX711_ReadRaw(HX711_Handle_t *hx)
{
    /* Wait for DOUT to go LOW (data ready) */
    uint32_t timeout = HAL_GetTick() + HX711_TIMEOUT_MS;
    while (!HX711_IsReady()) {
        if (HAL_GetTick() > timeout) return 0;
    }

    uint32_t raw = 0;
    /* Read 24 bits MSB first */
    for (int8_t i = 23; i >= 0; i--) {
        SCK_HIGH(); HX711_DELAY();
        if (DOUT_READ()) raw |= (1UL << i);
        SCK_LOW();  HX711_DELAY();
    }

    /* Extra pulses select gain for NEXT conversion */
    for (uint8_t p = 0; p < hx->gain_pulses; p++) {
        SCK_HIGH(); HX711_DELAY();
        SCK_LOW();  HX711_DELAY();
    }

    /* Convert from 24-bit two's complement */
    if (raw & 0x800000UL) {
        raw |= 0xFF000000UL;   /* sign-extend to 32 bits */
    }

    return (int32_t)raw;
}

float HX711_ReadGrams(HX711_Handle_t *hx)
{
    int32_t raw = HX711_ReadRaw(hx);
    return (float)(raw - hx->tare_offset) / hx->scale;
}

float HX711_ReadMillilitres(HX711_Handle_t *hx)
{
    /* Water: 1 g ≈ 1 ml at room temperature */
    return HX711_ReadGrams(hx);
}

void HX711_Tare(HX711_Handle_t *hx)
{
    /* Average 5 readings to reduce noise */
    int32_t sum = 0;
    for (uint8_t i = 0; i < 5; i++) {
        sum += HX711_ReadRaw(hx);
        HAL_Delay(10);
    }
    hx->tare_offset  = sum / 5;
    hx->is_calibrated = 1;
}

void HX711_SetScale(HX711_Handle_t *hx, float scale)
{
    hx->scale = scale;
}

void HX711_PowerDown(void)
{
    SCK_HIGH();
    HAL_Delay(1);   /* > 60 µs SCK HIGH → power down */
}

void HX711_PowerUp(void)
{
    SCK_LOW();
    HAL_Delay(1);
}
