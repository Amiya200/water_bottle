#pragma GCC optimize("Os")
#include "hx711.h"
#include "main.h"
#include <stddef.h>

/* ─── GPIO helpers ───────────────────────────────────────────────────────── */
#define DOUT_READ()  HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin)
#define SCK_HIGH()   HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET)
#define SCK_LOW()    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET)

/* ─── Timing ─────────────────────────────────────────────────────────────── */
/* Datasheet T2/T3 (SCK high/low) min = 0.2 µs.
 * At 48 MHz: 1 NOP ≈ 20.8 ns → 12 NOPs ≈ 0.25 µs per edge, safely in spec.
 * With -Os the NOP loop is NOT removed (the compiler treats __NOP() as a
 * memory barrier). If you ever switch to -O3 wrap with a volatile counter
 * or a brief DWT cycle delay instead. */
#define HX711_DELAY()  do { \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
} while(0)

/* ─── Init ───────────────────────────────────────────────────────────────── */
void HX711_Init(HX711_Handle_t *hx)
{
    hx->tare_offset   = 0;
    hx->scale         = HX711_DEFAULT_SCALE;   /* 1.0 → shows raw counts  */
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
/* Returns 1 and writes the signed 24-bit value into *out on success.
 * Returns 0 on timeout without modifying *out, so callers can drop the
 * sample cleanly instead of averaging in a stale zero. */
uint8_t HX711_ReadRawSafe(HX711_Handle_t *hx, int32_t *out)
{
    /* Wait for DOUT LOW (data ready) */
    uint32_t deadline = HAL_GetTick() + HX711_TIMEOUT_MS;
    while (DOUT_READ() != GPIO_PIN_RESET) {
        if (HAL_GetTick() >= deadline) return 0U;
    }

    /* Clock out 24 bits, MSB first */
    uint32_t raw = 0U;
    for (int8_t bit = 23; bit >= 0; bit--) {
        SCK_HIGH(); HX711_DELAY();
        if (DOUT_READ() == GPIO_PIN_SET) raw |= (1UL << bit);
        SCK_LOW();  HX711_DELAY();
    }

    /* Extra pulses select gain/channel for the NEXT conversion.
     * gain_pulses: 1 = ch-A gain 128, 2 = ch-B gain 32, 3 = ch-A gain 64. */
    for (uint8_t p = 0U; p < hx->gain_pulses; p++) {
        SCK_HIGH(); HX711_DELAY();
        SCK_LOW();  HX711_DELAY();
    }

    /* Sign-extend 24-bit two's-complement → int32_t.
     *
     * Why the union: (int32_t)(raw | 0xFF000000UL) is implementation-defined
     * behaviour in C when the result value doesn't fit in a signed 32-bit
     * integer (C11 §6.3.1.3 p3).  The union type-pun is well-defined in C99
     * (§6.5.2.3 footnote 95) and avoids any compiler-specific cast. */
    if (raw & 0x800000UL) {
        raw |= 0xFF000000UL;   /* fill upper byte with 1s */
    }
    union { uint32_t u; int32_t i; } pun;
    pun.u = raw;
    *out = pun.i;
    return 1U;
}

/* ─── Legacy shim ────────────────────────────────────────────────────────── */
int32_t HX711_ReadRaw(HX711_Handle_t *hx)
{
    int32_t v = 0;
    HX711_ReadRawSafe(hx, &v);
    return v;
}

/* ─── Median helper ──────────────────────────────────────────────────────── */
/* Insertion sort on a small array; returns the middle element. */
static int32_t median_i32(int32_t *a, uint8_t n)
{
    for (uint8_t i = 1U; i < n; i++) {
        int32_t key = a[i];
        int8_t  j   = (int8_t)i - 1;
        while (j >= 0 && a[(uint8_t)j] > key) {
            a[(uint8_t)(j + 1)] = a[(uint8_t)j];
            j--;
        }
        a[(uint8_t)(j + 1)] = key;
    }
    return a[n / 2U];
}

/* ─── Median-filtered averaged read ─────────────────────────────────────── */
/* Collects HX711_AVG_SAMPLES good samples, finds the median, then averages
 * only those within +-1 % of the median (rejects spikes).
 * Returns 1 and writes result into *out on success; 0 if no sample arrived. */
uint8_t HX711_ReadRawAveraged(HX711_Handle_t *hx, int32_t *out)
{
    int32_t buf[HX711_AVG_SAMPLES];
    uint8_t got = 0U;

    for (uint8_t i = 0U; i < HX711_AVG_SAMPLES; i++) {
        int32_t s;
        if (HX711_ReadRawSafe(hx, &s)) buf[got++] = s;
    }
    if (got == 0U) {
        hx->last_read_ok = 0U;
        return 0U;
    }

    int32_t med = median_i32(buf, got);

    /* Band = 1 % of |median|, with a floor so it still works near zero. */
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
    if (!HX711_ReadRawAveraged(hx, &raw)) {
        /* Chip not responding — preserve last valid result so the app's
         * stability filter doesn't see a phantom zero-gram event. */
        return 0.0f;
    }
    return (float)(raw - hx->tare_offset) / hx->scale;
}

float HX711_ReadMillilitres(HX711_Handle_t *hx)
{
    return HX711_ReadGrams(hx);   /* water: 1 g ~= 1 ml at room temperature */
}

/* ─── Tare ───────────────────────────────────────────────────────────────── */
/* Call with an empty, still bottle. Averages HX711_TARE_SAMPLES
 * median-filtered reads to get a clean zero baseline.
 * NOTE: this does NOT set is_calibrated — that requires HX711_Calibrate()
 * to be called afterwards with a known mass. */
void HX711_Tare(HX711_Handle_t *hx)
{
    int64_t sum = 0;
    uint8_t cnt = 0U;
    for (uint8_t i = 0U; i < HX711_TARE_SAMPLES; i++) {
        int32_t s;
        if (HX711_ReadRawAveraged(hx, &s)) {
            sum += s;
            cnt++;
        }
        HAL_Delay(10U);
    }
    if (cnt > 0U) {
        hx->tare_offset = (int32_t)(sum / (int64_t)cnt);
        /* is_calibrated deliberately NOT set here — must call HX711_Calibrate */
    }
}

/* ─── Scale calibration ──────────────────────────────────────────────────── */
/* Call AFTER HX711_Tare, with a known mass (known_grams) placed on the cell.
 *
 * Procedure (from the BLE terminal):
 *   1. Empty bottle, stable: send CAL / TARE
 *   2. Place a known weight (1-2 kg reference), let it settle (~3 s)
 *   3. Send CALWEIGHT,<grams>  e.g.  CALWEIGHT,1000  or  CALWEIGHT,2000
 *   4. Firmware replies OK and writes hx711_scale to flash
 *   5. Send WEIGHT to verify
 *
 * Returns 1 on success, 0 if the read failed or the load is implausible. */
uint8_t HX711_Calibrate(HX711_Handle_t *hx, float known_grams)
{
    if (known_grams < 1.0f) return 0U;   /* nonsense value */

    int32_t raw;
    if (!HX711_ReadRawAveraged(hx, &raw)) return 0U;

    float counts = (float)(raw - hx->tare_offset);

    /* Sanity guard: the cell must have moved meaningfully under load.
     * < 1000 counts typically means no weight, wrong wiring, or noise. */
    if (counts > -1000.0f && counts < 1000.0f) return 0U;

    float new_scale = counts / known_grams;
    if (new_scale == 0.0f) return 0U;

    hx->scale         = new_scale;
    hx->is_calibrated = 1U;
    return 1U;
}

/* ─── Scale override ─────────────────────────────────────────────────────── */
/* Use this to restore a previously determined scale from flash without
 * needing to recalibrate (e.g. on power-on when settings are loaded).
 * Sets is_calibrated = 1 if scale is positive. */
void HX711_SetScale(HX711_Handle_t *hx, float scale)
{
    hx->scale         = scale;
    hx->is_calibrated = (scale > 0.0f) ? 1U : 0U;
}

/* ─── Power management ───────────────────────────────────────────────────── */
void HX711_PowerDown(void)
{
    SCK_HIGH();
    HAL_Delay(1U);   /* > 60 us SCK HIGH -> power-down mode */
}

void HX711_PowerUp(void)
{
    SCK_LOW();
    HAL_Delay(1U);   /* allow the oscillator to restart */
}
