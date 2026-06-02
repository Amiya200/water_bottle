#ifndef HX711_H
#define HX711_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * HX711 driver — bit-bang via GPIO, STM32F030K6T6 @ 48 MHz
 *
 * Pin assignments (match main.h / MX_GPIO_Init):
 *   DOUT  → HX711_DOUT_GPIO_Port / HX711_DOUT_Pin  (input,  no pull)
 *   SCK   → HX711_SCK_GPIO_Port  / HX711_SCK_Pin   (output, PP)
 *
 * Calibration workflow (2-step, must be done in order):
 *   Step 0 — empty bottle, stable:  TARE / CAL  command  → stores tare_offset
 *   Step 1 — known weight on cell:  CALWEIGHT,<grams>    → stores hx711_scale
 *
 * Both values are persisted via app->settings (tare_offset, hx711_scale).
 * Until step 1 completes, is_calibrated stays 0 and App_TaskWeight returns
 * immediately — current_weight_g stays 0.  This is intentional so the app
 * never reports garbage while uncalibrated.
 *
 * Typical 3 kg cell @ gain 128: scale ≈ 400..600 counts/gram.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ─── Timing ──────────────────────────────────────────────────────────────── */
#define HX711_TIMEOUT_MS     500U   /* max wait for DOUT LOW before giving up */

/* ─── Gain / channel selection (extra pulses after the 24 data bits) ──────── */
#define HX711_GAIN_128       1U     /* channel A, gain 128 (default, best SNR) */
#define HX711_GAIN_32        2U     /* channel B, gain 32                       */
#define HX711_GAIN_64        3U     /* channel A, gain 64                       */

/* ─── Averaging / filtering ───────────────────────────────────────────────── */
#define HX711_TARE_SAMPLES   16U    /* averaged samples used to compute tare    */
#define HX711_AVG_SAMPLES     7U    /* samples per live read (odd → clean median)*/

/* ─── Default scale ───────────────────────────────────────────────────────── */
/* This value is used ONLY when no calibrated scale has been persisted yet.
 * It is deliberately set to 1.0 (raw counts) rather than a guessed grams
 * value — that way WEIGHT / RAWW commands always show something non-zero
 * and the operator can see the chip is alive even before calibration. */
#define HX711_DEFAULT_SCALE  1.0f

/* ─── Handle ──────────────────────────────────────────────────────────────── */
typedef struct {
    int32_t  tare_offset;     /* raw ADC at empty-bottle tare          */
    float    scale;           /* counts per gram (set by CALWEIGHT)    */
    uint8_t  gain_pulses;     /* 1=128, 2=32, 3=64                     */
    uint8_t  is_calibrated;   /* 1 only after BOTH tare AND scale set  */
    /* Diagnostics — set by each successful read, cleared on timeout. */
    int32_t  last_raw;        /* most recent averaged raw count        */
    uint8_t  last_read_ok;    /* 1 = last HX711_ReadRawAveraged() ok   */
} HX711_Handle_t;

/* ─── API ─────────────────────────────────────────────────────────────────── */
void    HX711_Init            (HX711_Handle_t *hx);
uint8_t HX711_IsReady         (void);

/* Low-level: single sample, safe (returns 0 on timeout) */
uint8_t HX711_ReadRawSafe     (HX711_Handle_t *hx, int32_t *out);

/* Median-filtered average of HX711_AVG_SAMPLES samples */
uint8_t HX711_ReadRawAveraged (HX711_Handle_t *hx, int32_t *out);

/* Compatibility shim — returns 0 on timeout */
int32_t HX711_ReadRaw         (HX711_Handle_t *hx);

/* High-level: grams and millilitres (return 0.0f if uncalibrated or no signal) */
float   HX711_ReadGrams       (HX711_Handle_t *hx);
float   HX711_ReadMillilitres (HX711_Handle_t *hx);

/* Tare: averaged empty-bottle zero (sets tare_offset, does NOT set scale) */
void    HX711_Tare            (HX711_Handle_t *hx);

/* Scale calibration: call after Tare with a known mass on the cell.
 * Sets hx->scale and sets is_calibrated = 1.
 * Returns 1 on success, 0 if the chip isn't talking or load is implausible. */
uint8_t HX711_Calibrate       (HX711_Handle_t *hx, float known_grams);

void    HX711_SetScale        (HX711_Handle_t *hx, float scale);
void    HX711_PowerDown       (void);
void    HX711_PowerUp         (void);

#endif /* HX711_H */
