/*
 * hx711.c  –  HX711 24-bit load-cell ADC driver
 *
 * More optimised revision
 * ─────────────────────────────────────────────────────────────────────
 * Flash: all weight conversion and calibration math is integer/fixed-point.
 *        This avoids pulling float division helpers into a Cortex-M0 build.
 * RAM  : no static allocations.
 */

#pragma GCC optimize("Os")
#include "hx711.h"
#include "main.h"
#include <stddef.h>

#define DOUT_READ()  HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin)
#define SCK_HIGH()   HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET)
#define SCK_LOW()    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET)

#define HX711_DELAY() do { \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
} while(0)

void HX711_Init(HX711_Handle_t *hx)
{
    hx->tare_offset   = 0;
    hx->scale_x100    = HX711_SCALE_X100_DEFAULT;
    hx->gain_pulses   = HX711_GAIN_128;
    hx->is_calibrated = 0;
    hx->last_raw      = 0;
    hx->last_read_ok  = 0;
    SCK_LOW();
}

uint8_t HX711_IsReady(void)
{
    return (DOUT_READ() == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t HX711_ReadRawSafe(HX711_Handle_t *hx, int32_t *out)
{
    uint32_t t0 = HAL_GetTick();
    while (DOUT_READ() != GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) >= HX711_TIMEOUT_MS) return 0U;
    }

    uint32_t raw = 0U;
    for (int8_t bit = 23; bit >= 0; bit--) {
        SCK_HIGH(); HX711_DELAY();
        if (DOUT_READ() == GPIO_PIN_SET) raw |= (1UL << bit);
        SCK_LOW();  HX711_DELAY();
    }

    for (uint8_t p = 0U; p < hx->gain_pulses; p++) {
        SCK_HIGH(); HX711_DELAY();
        SCK_LOW();  HX711_DELAY();
    }

    if (raw & 0x800000UL) raw |= 0xFF000000UL;
    union { uint32_t u; int32_t i; } pun;
    pun.u = raw;
    *out = pun.i;
    return 1U;
}

static int32_t median_i32(int32_t *a, uint8_t n)
{
    for (uint8_t i = 1U; i < n; i++) {
        int32_t key = a[i];
        int8_t  j   = (int8_t)(i - 1U);
        while (j >= 0 && a[(uint8_t)j] > key) {
            a[(uint8_t)(j + 1U)] = a[(uint8_t)j];
            j--;
        }
        a[(uint8_t)(j + 1U)] = key;
    }
    return a[n >> 1U];
}

uint8_t HX711_ReadRawAveraged(HX711_Handle_t *hx, int32_t *out)
{
    int32_t buf[HX711_AVG_SAMPLES];
    uint8_t got = 0U;

    for (uint8_t i = 0U; i < HX711_AVG_SAMPLES; i++) {
        int32_t s;
        if (HX711_ReadRawSafe(hx, &s)) buf[got++] = s;
    }
    if (got == 0U) { hx->last_read_ok = 0U; return 0U; }

    int32_t med  = median_i32(buf, got);
    int32_t band = (med >= 0) ? (med / 100) : (-med / 100);
    if (band < 200) band = 200;

    int64_t sum = 0;
    uint8_t cnt = 0U;
    for (uint8_t i = 0U; i < got; i++) {
        int32_t d = buf[i] - med;
        if (d < 0) d = -d;
        if (d <= band) { sum += buf[i]; cnt++; }
    }

    int32_t result = (cnt > 0U) ? (int32_t)(sum / (int64_t)cnt) : med;
    hx->last_raw     = result;
    hx->last_read_ok = 1U;
    *out = result;
    return 1U;
}

int32_t HX711_ReadGrams(HX711_Handle_t *hx)
{
    int32_t raw;
    if (!HX711_ReadRawAveraged(hx, &raw)) return 0;
    if (hx->scale_x100 < HX711_SCALE_X100_MIN) return 0;

    int64_t counts_x100 = (int64_t)(raw - hx->tare_offset) * 100LL;
    return (int32_t)(counts_x100 / (int64_t)hx->scale_x100);
}

int32_t HX711_ReadMillilitres(HX711_Handle_t *hx)
{
    return HX711_ReadGrams(hx); /* 1 g water ≈ 1 ml */
}

void HX711_Tare(HX711_Handle_t *hx)
{
    int64_t sum = 0;
    uint8_t cnt = 0U;
    for (uint8_t i = 0U; i < HX711_TARE_SAMPLES; i++) {
        int32_t s;
        if (HX711_ReadRawAveraged(hx, &s)) { sum += s; cnt++; }
        HAL_Delay(10U);
    }
    if (cnt > 0U) hx->tare_offset = (int32_t)(sum / (int64_t)cnt);
}

uint8_t HX711_Calibrate(HX711_Handle_t *hx, int32_t known_grams)
{
    if (known_grams < 1) return 0U;

    int32_t raw;
    if (!HX711_ReadRawAveraged(hx, &raw)) return 0U;

    int32_t counts = raw - hx->tare_offset;
    if (counts < 0) counts = -counts;
    if (counts < 1000L) return 0U;

    int64_t scale = ((int64_t)counts * 100LL) / (int64_t)known_grams;
    if (scale < HX711_SCALE_X100_MIN || scale > 2147483647LL) return 0U;

    hx->scale_x100    = (int32_t)scale;
    hx->is_calibrated = 1U;
    return 1U;
}

void HX711_SetScaleX100(HX711_Handle_t *hx, int32_t scale_x100)
{
    hx->scale_x100    = scale_x100;
    hx->is_calibrated = (scale_x100 >= HX711_SCALE_X100_MIN) ? 1U : 0U;
}

void HX711_PowerDown(void) { SCK_HIGH(); HAL_Delay(1U); }
void HX711_PowerUp(void)   { SCK_LOW();  HAL_Delay(1U); }
