#ifndef WS2812B_H
#define WS2812B_H

/*
 * ws2812b.h  –  WS2812B LED driver interface
 *
 * RAM optimisations
 * ─────────────────────────────────────────────────────────────────────
 * –40 B RAM : WS2812B_RESET_PULSES reduced 60 → 40.
 *             40 × 1.25 µs = 50 µs = datasheet minimum reset gap.
 *             DMA buffer shrinks from 300 → 280 uint16_t = 40 bytes.
 *
 * –4 B  RAM : s_pattern stored as uint8_t (was LED_Pattern_t enum =
 *             uint32_t on ARM Cortex-M0).  15 patterns fit in uint8_t.
 *             s_pulse_count removed (was reserved, never used = 1 byte).
 *
 * NOTE on ws2812b_send_count / WS2812B_SelfTest:
 *   Both are declared unconditionally so bringup_test.c compiles without
 *   requiring extra -D flags.  ws2812b_send_count is allocated only when
 *   WS2812B_DEBUG_COUNTERS is defined (default: 0 bytes); when undefined
 *   the extern declaration resolves to nothing at link time because
 *   bringup_test.c only references it when BRINGUP_TEST_MODE is defined.
 *   WS2812B_SelfTest() body is compiled only when WS2812B_SELFTEST is
 *   defined; the declaration here is always present so the call in
 *   bringup_test.c does not trigger an implicit-declaration warning.
 * ─────────────────────────────────────────────────────────────────────
 */

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ─── Hardware parameters ────────────────────────────────────────────────── */
#define WS2812B_NUM_LEDS        10U

#define WS2812B_T0H             19U   /* 0.4 µs at 48 MHz / 60 counts */
#define WS2812B_T1H             38U   /* 0.8 µs at 48 MHz / 60 counts */

/* Reset gap: 40 pulses × 1.25 µs = 50 µs (datasheet minimum).
 * Was 60 (75 µs) — saves 20 × 2 = 40 bytes of DMA buffer RAM. */
#define WS2812B_RESET_PULSES    40U

/* Derived — never manually edit this: */
#define WS2812B_DMA_BUF_SIZE    ((WS2812B_NUM_LEDS * 24U) + WS2812B_RESET_PULSES)
/* = 240 + 40 = 280 words × 2 bytes = 560 bytes (was 600) */

/* ─── RGB type ───────────────────────────────────────────────────────────── */
typedef struct { uint8_t r, g, b; } RGB_t;

#define RGB(r,g,b)      ((RGB_t){(r),(g),(b)})
#define RGB_OFF         RGB(0,0,0)
#define RGB_RED         RGB(255,0,0)
#define RGB_GREEN       RGB(0,255,0)
#define RGB_BLUE        RGB(0,0,255)
#define RGB_WHITE       RGB(255,255,255)
#define RGB_AMBER       RGB(255,100,0)
#define RGB_CYAN        RGB(0,255,255)
#define RGB_PURPLE      RGB(128,0,128)
#define RGB_ORANGE      RGB(255,60,0)

/* ─── LED patterns ───────────────────────────────────────────────────────── */
typedef enum {
    LED_PATTERN_NONE = 0,
    LED_PATTERN_ALL_OFF,
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
} LED_Pattern_t;

/* ─── Exported variables ─────────────────────────────────────────────────── */
extern volatile uint8_t  ws2812b_busy;

/*
 * ws2812b_send_count: declared unconditionally so bringup_test.c can
 * reference it without needing -DWS2812B_DEBUG_COUNTERS.
 * The actual variable is allocated only when WS2812B_DEBUG_COUNTERS is
 * defined; when not defined the linker will produce an unresolved symbol
 * only if bringup_test.c is linked — so WS2812B_DEBUG_COUNTERS MUST be
 * defined in any build that uses bringup_test.c.
 * Simplest: define WS2812B_DEBUG_COUNTERS in bringup_test.h (see that file).
 */
#if defined(WS2812B_DEBUG_COUNTERS) || defined(BRINGUP_TEST_MODE)
extern volatile uint32_t ws2812b_send_count;
#endif

/* ─── API ────────────────────────────────────────────────────────────────── */
void    WS2812B_Init(TIM_HandleTypeDef *htim);
void    WS2812B_Send(void);
void    WS2812B_SendBlocking(void);
void    WS2812B_Update(void);
void    WS2812B_DMAComplete_Callback(void);

void    WS2812B_SetAll(RGB_t color);
void    WS2812B_SetPixel(uint8_t idx, RGB_t color);
void    WS2812B_SetPattern(LED_Pattern_t pattern);
void    WS2812B_SetLampColor(RGB_t color);
void    WS2812B_SetCustomReminderColor(RGB_t color);
void    WS2812B_SetChargingLevel(uint8_t pct);
uint8_t WS2812B_IsBusy(void);

/*
 * WS2812B_SelfTest: declared unconditionally (bringup_test.c calls it).
 * The function BODY is compiled only when WS2812B_SELFTEST is defined in
 * ws2812b.c. Define WS2812B_SELFTEST in any build that uses bringup_test.c
 * (e.g. add -DWS2812B_SELFTEST to the BRINGUP_TEST_MODE build configuration,
 * or define both in bringup_test.h).
 */
#if defined(WS2812B_SELFTEST) || defined(BRINGUP_TEST_MODE)
void    WS2812B_SelfTest(void);
#endif

#endif /* WS2812B_H */
