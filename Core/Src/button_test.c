/*
 * button_test.c  –  PB3 push-button bring-up self-test (HydraSense)
 *
 * Test procedure (LED feedback on the WS2812B ring):
 *   Power-on check : line must read HIGH at rest (pull-up). If it reads LOW
 *                    with nothing pressed -> solid RED + reset buzzer, halt.
 *                    (= short to GND, wrong pin, or missing pull-up)
 *   Idle           : dim blue breathing
 *   Pressed        : solid GREEN          (raw level reaches the MCU)
 *   Held >= 3 s    : AMBER                (long-press tier detected)
 *   Held >= 10 s   : RED                  (very-long / factory-reset tier)
 *   Short release  : single beep, press counted
 *
 * The test exits PASS after 5 counted short presses: 3 green flashes,
 * then control returns to the caller (normal app continues).
 */
#include "main.h"
#include "button_test.h"

#ifdef BUTTON_TEST_MODE

#include "ws2812b.h"
#include "buzzer.h"

#define BTN_TEST_PASS_COUNT   5U      /* short presses required to pass     */
#define BTN_TEST_DEBOUNCE_MS  20U     /* press / release debounce           */
#define BTN_TEST_LONG_MS      3000U   /* long-press tier (matches app)      */
#define BTN_TEST_VLONG_MS     10000U  /* very-long tier  (matches app)      */

static uint8_t BtnTest_RawDown(void)
{
    return (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
           ? 1U : 0U;
}

void Button_Test(void)
{
    uint8_t  presses  = 0U;
    uint8_t  was_down = 0U;
    uint32_t down_ms  = 0U;

    /* ── Sanity: at rest the line MUST be HIGH (internal pull-up). ──────── */
    if (BtnTest_RawDown()) {
        WS2812B_SetAll(RGB(40, 0, 0));          /* stuck-low fault          */
        WS2812B_SendBlocking();
        Buzzer_Play(BUZZER_FACTORY_RESET);
        while (1) { Buzzer_Update(); }          /* halt — wiring fault      */
    }

    while (presses < BTN_TEST_PASS_COUNT)
    {
        uint32_t now  = HAL_GetTick();
        uint8_t  down = BtnTest_RawDown();

        if (down && !was_down) {                            /* press edge   */
            HAL_Delay(BTN_TEST_DEBOUNCE_MS);
            if (BtnTest_RawDown()) {
                was_down = 1U;
                down_ms  = now;
                WS2812B_SetAll(RGB(0, 40, 0));              /* green        */
                WS2812B_SendBlocking();
            }
        }
        else if (down && was_down) {                        /* held         */
            uint32_t held = now - down_ms;
            if (held >= BTN_TEST_VLONG_MS) {
                WS2812B_SetAll(RGB(40, 0, 0));              /* red          */
                WS2812B_SendBlocking();
            } else if (held >= BTN_TEST_LONG_MS) {
                WS2812B_SetAll(RGB(40, 25, 0));             /* amber        */
                WS2812B_SendBlocking();
            }
        }
        else if (!down && was_down) {                       /* release edge */
            HAL_Delay(BTN_TEST_DEBOUNCE_MS);
            was_down = 0U;
            uint32_t held = now - down_ms;
            if (held < BTN_TEST_LONG_MS) {                  /* short press  */
                presses++;
                Buzzer_Play(BUZZER_SINGLE_BEEP);
            }
            WS2812B_SetAll(RGB_OFF);
            WS2812B_SendBlocking();
        }
        else {                                              /* idle breathe */
            uint32_t ph = (now / 10U) % 80U;
            uint8_t  b  = (uint8_t)((ph < 40U) ? ph : (80U - ph));
            WS2812B_SetAll(RGB(0, 0, b));
            WS2812B_SendBlocking();
        }

        Buzzer_Update();
        HAL_Delay(5);
    }

    /* ── PASS: 3 green flashes, then return to the application. ─────────── */
    for (uint8_t i = 0U; i < 3U; i++) {
        WS2812B_SetAll(RGB(0, 40, 0)); WS2812B_SendBlocking(); HAL_Delay(150);
        WS2812B_SetAll(RGB_OFF);       WS2812B_SendBlocking(); HAL_Delay(150);
    }
}

#endif /* BUTTON_TEST_MODE */
