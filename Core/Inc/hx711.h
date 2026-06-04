#ifndef HX711_H
#define HX711_H

/*
 * hx711.h  –  HX711 24-bit load-cell ADC driver interface
 *
 * More RAM/Flash optimised revision
 * ─────────────────────────────────────────────────────────────────────
 * Flash: removes float math from the HX711 path.  Cortex-M0 has no FPU, so
 *        float division/calibration pulls soft-float helper code into .text.
 *        Scale is now stored as fixed-point counts-per-gram × 100.
 * RAM  : handle size stays compact and deterministic; no float fields.
 * Stack: averaging/tare buffers remain 3 samples.
 */

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ─── Timing and gain ────────────────────────────────────────────────────── */
#define HX711_GAIN_128      1U   /* Channel A, gain 128 (1 extra pulse)  */
#define HX711_GAIN_32       2U   /* Channel B, gain 32  (2 extra pulses) */
#define HX711_GAIN_64       3U   /* Channel A, gain 64  (3 extra pulses) */

#define HX711_TIMEOUT_MS    150U /* max wait for DOUT LOW (data ready)   */
#define HX711_AVG_SAMPLES   3U
#define HX711_TARE_SAMPLES  3U

/* Fixed-point scale: counts per gram × 100.
 * Example: 440.25 counts/g = 44025. */
#define HX711_SCALE_X100_DEFAULT 44000L
#define HX711_SCALE_X100_MIN     100L

/* ─── Handle ─────────────────────────────────────────────────────────────── */
typedef struct {
    int32_t  tare_offset;     /* zero baseline raw count                  */
    int32_t  scale_x100;      /* counts-per-gram × 100                    */
    int32_t  last_raw;        /* last averaged raw reading                */
    uint8_t  gain_pulses;     /* HX711_GAIN_128 / 32 / 64                 */
    uint8_t  is_calibrated;   /* 1 = scale + tare both set and valid      */
    uint8_t  last_read_ok;    /* 1 = last read succeeded                  */
} HX711_Handle_t;

/* ─── API ────────────────────────────────────────────────────────────────── */
void    HX711_Init(HX711_Handle_t *hx);
uint8_t HX711_IsReady(void);
uint8_t HX711_ReadRawSafe(HX711_Handle_t *hx, int32_t *out);
uint8_t HX711_ReadRawAveraged(HX711_Handle_t *hx, int32_t *out);
int32_t HX711_ReadGrams(HX711_Handle_t *hx);
int32_t HX711_ReadMillilitres(HX711_Handle_t *hx);
void    HX711_Tare(HX711_Handle_t *hx);
uint8_t HX711_Calibrate(HX711_Handle_t *hx, int32_t known_grams);
void    HX711_SetScaleX100(HX711_Handle_t *hx, int32_t scale_x100);
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
