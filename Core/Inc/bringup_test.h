#ifndef BRINGUP_TEST_H
#define BRINGUP_TEST_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ============================================================================
 * HydraSense — Hardware Bring-up / Self-Test module
 * ----------------------------------------------------------------------------
 * Proves each core subsystem is wired and alive BEFORE the full app stack:
 *   1. WS2812B RGB ring  (PA8 / TIM1_CH1 + DMA)  via WS2812B_SelfTest()
 *   2. NTC temperature   (PA3 / ADC_IN3)
 *   3. TDS sensor        (PA2 / ADC_IN2, drive PA6)
 *   4. HX711 load cell   (PA4 DOUT / PA5 SCK)
 *
 * Every reading is published into the global volatile struct g_test, so you
 * can add 'g_test' to STM32CubeIDE → Live Expressions (Window → Show View →
 * Live Expressions) and watch all values live over SWD.
 *
 * Enable by building with -DBRINGUP_TEST_MODE (Project Properties →
 * C/C++ Build → Settings → MCU GCC Compiler → Preprocessor → Defined symbols).
 *
 * RGB self-test (WS2812B_SelfTest inside BringUp_RunOnce):
 *   Phase 1  ALL RED    800 ms
 *   Phase 2  ALL GREEN  800 ms
 *   Phase 3  ALL BLUE   800 ms
 *   Phase 4  ALL WHITE  800 ms
 *   Phase 5  RAINBOW    1200 ms  (10 hues)
 *   Phase 6  ALTERNATE  800 ms   even=RED, odd=GREEN
 *   Phase 7  PIXEL WALK 120 ms × 10 positions
 *   Phase 8  ALL OFF    300 ms
 *
 * g_test.rgb_send_count > 0  → DMA is completing frames  (PASS)
 * g_test.rgb_send_count == 0 → DMA never completed        (FAIL → check MSP)
 * g_test.rgb_busy             → 1 while frame is in flight
 * ==========================================================================*/

typedef enum {
    TEST_PENDING = 0,   /* not yet run             */
    TEST_PASS    = 1,   /* sane value obtained     */
    TEST_FAIL    = 2,   /* no response / out range */
} TestStatus_t;

typedef struct {
    /* ---- WS2812B RGB ---- */
    volatile uint8_t      rgb_color_index;
    volatile uint8_t      rgb_busy;
    volatile uint32_t     rgb_send_count;
    volatile TestStatus_t rgb_status;

    /* ---- NTC temperature ---- */
    volatile uint16_t     ntc_adc_raw;
    volatile int16_t      ntc_temp_x10;   /* 235 = 23.5 °C */
    volatile TestStatus_t ntc_status;

    /* ---- TDS ---- */
    volatile uint16_t     tds_adc_raw;
    volatile uint16_t     tds_ppm;
    volatile TestStatus_t tds_status;

    /* ---- HX711 load cell ---- */
    volatile uint8_t      hx_ready;
    volatile int32_t      hx_raw;
    volatile int32_t      hx_raw_tare;
    volatile float        hx_grams;
    volatile TestStatus_t hx_status;

    /* ---- overall ---- */
    volatile uint32_t     loop_count;
    volatile uint32_t     uptime_ms;
} BringUpTest_t;

extern BringUpTest_t g_test;

void BringUp_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim_ws);
void BringUp_RunOnce(void);
void BringUp_RunLoop(void);

#endif /* BRINGUP_TEST_H */
