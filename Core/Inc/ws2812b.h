#ifndef WS2812B_H
#define WS2812B_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ─── Configuration ──────────────────────────────────────────── */
#define WS2812B_NUM_LEDS        10U
#define WS2812B_BITS_PER_LED    24U

/*
 * RESET_PULSES = 60 words × 1.25 µs = 75 µs.
 * Datasheet minimum is 50 µs; 75 µs gives 25 µs margin.
 * (Was 50 in the old code → only 62.5 µs, borderline.)
 */
#define WS2812B_RESET_PULSES    60U
#define WS2812B_DMA_BUF_SIZE    (WS2812B_NUM_LEDS * WS2812B_BITS_PER_LED \
                                 + WS2812B_RESET_PULSES)   /* 300 words */

/*
 * DMA buffer is uint16_t (HalfWord).
 * CubeMX / stm32f0xx_hal_msp.c MUST configure DMA1_CH2 as:
 *   Direction      = MEMORY → PERIPHERAL   (not PERIPH → MEMORY !)
 *   MemDataAlign   = HALFWORD
 *   PerDataAlign   = HALFWORD
 * Passing (uint32_t*) to HAL_TIM_PWM_Start_DMA is correct — the HAL API
 * signature requires uint32_t* but the actual transfer width is governed
 * by the DMA config, not the pointer type.
 */

/* TIM1 @ 48 MHz, PSC=0, ARR=59 → 800 kHz, one tick = 1/48 µs ≈ 20.8 ns */
#define WS2812B_T0H             18U   /* 18/48 MHz ≈ 0.375 µs  (~0 bit high) */
#define WS2812B_T1H             36U   /* 36/48 MHz ≈ 0.750 µs  (~1 bit high) */

/* ─── Colour helpers ─────────────────────────────────────────── */
typedef struct { uint8_t r, g, b; } RGB_t;

#define RGB(r,g,b)      ((RGB_t){(r),(g),(b)})
#define RGB_OFF         RGB(0,0,0)
#define RGB_RED         RGB(255,0,0)
#define RGB_GREEN       RGB(0,255,0)
#define RGB_BLUE        RGB(0,0,255)
#define RGB_AMBER       RGB(255,100,0)
#define RGB_ORANGE      RGB(255,50,0)
#define RGB_PURPLE      RGB(128,0,128)
#define RGB_CYAN        RGB(0,200,200)
#define RGB_WHITE       RGB(255,255,255)

/* ─── Pattern IDs ────────────────────────────────────────────── */
typedef enum {
    LED_PATTERN_NONE = 0,
    LED_PATTERN_HYDRATION_HIGH,
    LED_PATTERN_HYDRATION_MID,
    LED_PATTERN_HYDRATION_LOW,
    LED_PATTERN_PURITY_ALERT,
    LED_PATTERN_TEMP_ALERT,
    LED_PATTERN_CALIBRATION,
    LED_PATTERN_DRINK_CONFIRM,
    LED_PATTERN_SYNC_SUCCESS,
    LED_PATTERN_CHARGING_BAR,
    LED_PATTERN_LOW_BATTERY,
    LED_PATTERN_LAMP_MODE,
    LED_PATTERN_REGISTRATION,
    LED_PATTERN_FACTORY_RESET_WARN,
    LED_PATTERN_ERROR,
    LED_PATTERN_ALL_OFF,
} LED_Pattern_t;

/* ─── Public API ─────────────────────────────────────────────── */
void    WS2812B_Init(TIM_HandleTypeDef *htim);
void    WS2812B_Update(void);
void    WS2812B_SetPattern(LED_Pattern_t pattern);
void    WS2812B_SetLampColor(RGB_t color);
void    WS2812B_SetChargingLevel(uint8_t pct);
void    WS2812B_SetCustomReminderColor(RGB_t color);
void    WS2812B_SetAll(RGB_t color);
void    WS2812B_SetPixel(uint8_t idx, RGB_t color);
void    WS2812B_Send(void);
void    WS2812B_SendBlocking(void);     /* Send + wait until DMA done */
void    WS2812B_DMAComplete_Callback(void);
uint8_t WS2812B_IsBusy(void);

/* RGB self-test (call once at startup under BRINGUP_TEST_MODE or from
 * any diagnostic path). Runs a full colour-walk and returns. */
void    WS2812B_SelfTest(void);

/* Exposed for Live Expressions / bring-up test */
extern volatile uint8_t  ws2812b_busy;
extern volatile uint32_t ws2812b_send_count;

#endif /* WS2812B_H */
