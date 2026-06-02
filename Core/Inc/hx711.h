#ifndef HX711_H
#define HX711_H

/*
 * hx711.h  –  HX711 24-bit load-cell ADC driver interface
 *
 * RAM (stack) optimisations
 * ─────────────────────────────────────────────────────────────────────
 * –8 B stack : HX711_AVG_SAMPLES reduced 5 → 3.
 *              ReadRawAveraged() allocates int32_t buf[N] on the stack.
 *              5 × 4 = 20 bytes → 3 × 4 = 12 bytes per call (-8 bytes).
 *              Median-of-3 with 1% band rejection is statistically sound
 *              for a low-noise 24-bit load cell.  The reduction also
 *              speeds up each weight poll by ~40 ms (2 fewer chip reads
 *              at ~20 ms/read).
 *
 * –6 B stack : HX711_TARE_SAMPLES reduced 5 → 3 for the same reason.
 *              Tare is called once at calibration, not in the main loop,
 *              so accuracy is maintained by the 3-sample average.
 * ─────────────────────────────────────────────────────────────────────
 */

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ─── Timing and gain ────────────────────────────────────────────────────── */
#define HX711_GAIN_128      1U   /* Channel A, gain 128 (1 extra pulse)  */
#define HX711_GAIN_32       2U   /* Channel B, gain 32  (2 extra pulses) */
#define HX711_GAIN_64       3U   /* Channel A, gain 64  (3 extra pulses) */

#define HX711_TIMEOUT_MS    150U /* max wait for DOUT LOW (data ready)   */

/* Stack-allocated averaging buffer — reduce to save 8 bytes stack RAM  */
#define HX711_AVG_SAMPLES   3U   /* was 5 — saves 8 bytes stack per call */
#define HX711_TARE_SAMPLES  3U   /* was 5 — saves 8 bytes during tare    */

#define HX711_DEFAULT_SCALE 1.0f /* raw counts per gram before calibration */

/* ─── Handle ─────────────────────────────────────────────────────────────── */
typedef struct {
    int32_t  tare_offset;    /* zero baseline (set by HX711_Tare)        */
    float    scale;          /* counts per gram (set by HX711_Calibrate) */
    int32_t  last_raw;       /* last averaged raw reading                */
    uint8_t  gain_pulses;    /* HX711_GAIN_128 / 32 / 64                 */
    uint8_t  is_calibrated;  /* 1 = scale + tare both set and valid       */
    uint8_t  last_read_ok;   /* 1 = last read succeeded (DOUT responded)  */
} HX711_Handle_t;

/* ─── API ────────────────────────────────────────────────────────────────── */
void    HX711_Init(HX711_Handle_t *hx);
uint8_t HX711_IsReady(void);
uint8_t HX711_ReadRawSafe(HX711_Handle_t *hx, int32_t *out);
uint8_t HX711_ReadRawAveraged(HX711_Handle_t *hx, int32_t *out);
float   HX711_ReadGrams(HX711_Handle_t *hx);
float   HX711_ReadMillilitres(HX711_Handle_t *hx);
void    HX711_Tare(HX711_Handle_t *hx);
uint8_t HX711_Calibrate(HX711_Handle_t *hx, float known_grams);
void    HX711_SetScale(HX711_Handle_t *hx, float scale);
void    HX711_PowerDown(void);
void    HX711_PowerUp(void);

/* Legacy shim — prefer HX711_ReadRawSafe() in new code */
static inline int32_t HX711_ReadRaw(HX711_Handle_t *hx)
{
    int32_t v = 0;
    HX711_ReadRawSafe(hx, &v);
    return v;
}

#endif /* HX711_H */
