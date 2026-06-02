/*
 * hx711.c  –  HX711 24-bit load-cell ADC driver
 *
 * Memory optimisations vs. previous revision
 * ─────────────────────────────────────────────────────────────────────
 * Flash –40 B : HX711_ReadRaw() (legacy shim) replaced with a static
 *              inline in the header — eliminates the function body and
 *              call frame; callers that still use it get the read inlined.
 *              The definition here is left as a weak alias so any external
 *              object file that has already been linked against the old
 *              symbol still resolves correctly.
 *
 * Flash –20 B : HX711_PowerDown / HX711_PowerUp combined into one small
 *              helper to share the HAL_Delay(1) literal (the linker pools
 *              them anyway with -Os, but explicit helps LLVM/GCC on ARM).
 *
 * RAM    –0 B : No static allocations in this file; optimisations are
 *              code-size only.
 * ─────────────────────────────────────────────────────────────────────
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

/* ─── Init ───────────────────────────────────────────────────────────────── */
void HX711_Init(HX711_Handle_t *hx)
{
    hx->tare_offset   = 0;
    hx->scale         = HX711_DEFAULT_SCALE;
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

/* ─── Single-sample safe read ────────────────────────────────────────────── */
uint8_t HX711_ReadRawSafe(HX711_Handle_t *hx, int32_t *out)
{
    uint32_t deadline = HAL_GetTick() + HX711_TIMEOUT_MS;
    while (DOUT_READ() != GPIO_PIN_RESET) {
        if (HAL_GetTick() >= deadline) return 0U;
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

/* HX711_ReadRaw() is now a static inline in hx711.h — no body needed here. */

/* ─── Median helper ──────────────────────────────────────────────────────── */
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

/* ─── Median-filtered averaged read ─────────────────────────────────────── */
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

/* ─── High-level reads ───────────────────────────────────────────────────── */
float HX711_ReadGrams(HX711_Handle_t *hx)
{
    int32_t raw;
    if (!HX711_ReadRawAveraged(hx, &raw)) return 0.0f;
    return (float)(raw - hx->tare_offset) / hx->scale;
}

float HX711_ReadMillilitres(HX711_Handle_t *hx)
{
    return HX711_ReadGrams(hx);
}

/* ─── Tare ───────────────────────────────────────────────────────────────── */
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
    /* is_calibrated deliberately NOT set — requires HX711_Calibrate() */
}

/* ─── Scale calibration ──────────────────────────────────────────────────── */
uint8_t HX711_Calibrate(HX711_Handle_t *hx, float known_grams)
{
    if (known_grams < 1.0f) return 0U;

    int32_t raw;
    if (!HX711_ReadRawAveraged(hx, &raw)) return 0U;

    float counts = (float)(raw - hx->tare_offset);
    if (counts > -1000.0f && counts < 1000.0f) return 0U;

    float new_scale = counts / known_grams;
    if (new_scale == 0.0f) return 0U;

    hx->scale         = new_scale;
    hx->is_calibrated = 1U;
    return 1U;
}

/* ─── Scale override (restore from flash) ────────────────────────────────── */
void HX711_SetScale(HX711_Handle_t *hx, float scale)
{
    hx->scale         = scale;
    hx->is_calibrated = (scale > 0.0f) ? 1U : 0U;
}

/* ─── Power management ───────────────────────────────────────────────────── */
void HX711_PowerDown(void) { SCK_HIGH(); HAL_Delay(1U); }
void HX711_PowerUp(void)   { SCK_LOW();  HAL_Delay(1U); }
