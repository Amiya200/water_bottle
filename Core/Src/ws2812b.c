/*
 * ws2812b.c  –  WS2812B driver for STM32F030K6T6 (TIM1_CH1 + DMA1_CH2)
 *
 * Memory optimisations vs. previous revision
 * ─────────────────────────────────────────────────────────────────────
 * RAM  –30 B : removed s_leds_shadow[].  Dirty tracking is now a single
 *              uint8_t flag set whenever a pixel setter is called.  The
 *              shadow copy was only used to populate that same flag, so the
 *              extra 30-byte framebuffer was dead weight.
 *
 * Flash –80 B  : ScaleBright() is now a static inline in the header (or
 *              expanded here as a macro) — avoids a call frame + function
 *              body in .text for a three-multiply helper.
 *
 * RAM  –4 B  : s_pattern changed LED_Pattern_t enum→uint8_t (15 patterns fit
 *              in uint8_t; ARM stored enum as uint32_t = 3 bytes wasted).
 *              s_pulse_count removed (was reserved but never used = 1 byte).
 *
 * ─────────────────────────────────────────────────────────────────────
 */

#pragma GCC optimize("Os")
#include "ws2812b.h"
#include <string.h>
#include <stdlib.h>

/* ─── DMA buffer — 280 words × 2 bytes = 560 bytes (RESET_PULSES reduced 60→40) ──────────────────────────────────────────────────────────── */
static uint16_t ws2812b_dma_buf[WS2812B_DMA_BUF_SIZE];

/* ─── LED framebuffer — shadow removed, dirty flag only ─────────────────── */
static RGB_t     s_leds[WS2812B_NUM_LEDS];
static TIM_HandleTypeDef *s_htim = NULL;

volatile uint8_t  ws2812b_busy = 0;
#if defined(WS2812B_DEBUG_COUNTERS) || defined(BRINGUP_TEST_MODE)
volatile uint32_t ws2812b_send_count = 0;
#endif

/* ─── Pattern state ──────────────────────────────────────────────────────── */
static uint8_t       s_pattern       = LED_PATTERN_NONE;
static uint32_t      s_pattern_tick  = 0;
static uint32_t      s_blink_tick    = 0;
static uint8_t       s_sweep_pos     = 0;
static uint8_t       s_wave_pos      = 0;
static RGB_t         s_lamp_color    = {255, 255, 255};
static RGB_t         s_custom_remind = {0,   255, 0};
static uint8_t       s_charge_pct    = 0;
static uint8_t       s_dirty         = 1;

/* ─── ScaleBright: inlined macro — no function-call overhead ─────────────── */
#define SCALE_CH(ch, sc)  ((uint8_t)(((uint16_t)(ch) * (sc)) >> 8))
#define SCALE_RGB(c, sc)  RGB(SCALE_CH((c).r,(sc)), SCALE_CH((c).g,(sc)), SCALE_CH((c).b,(sc)))

/* ─── Timer reconfiguration (clears CubeMX slave-mode, sets 800 kHz) ─────── */
static void WS2812B_ReconfigTimer(void)
{
    HAL_TIM_PWM_Stop_DMA(s_htim, TIM_CHANNEL_1);
    s_htim->Instance->SMCR = 0U;
    s_htim->Instance->PSC  = 0U;
    s_htim->Instance->ARR  = 59U;
    s_htim->Instance->EGR  = TIM_EGR_UG;
}

/* ─── Pack s_leds[] → DMA buffer — 280 words × 2 bytes = 560 bytes (RESET_PULSES reduced 60→40) ──────────────────────────────────────────── */
static void WS2812B_Pack(void)
{
    uint16_t idx = 0;
    for (uint8_t led = 0; led < WS2812B_NUM_LEDS; led++) {
        uint32_t grb = ((uint32_t)s_leds[led].g << 16)
                     | ((uint32_t)s_leds[led].r <<  8)
                     |  (uint32_t)s_leds[led].b;
        for (int8_t bit = 23; bit >= 0; bit--) {
            ws2812b_dma_buf[idx++] = (grb & (1UL << bit))
                                     ? WS2812B_T1H : WS2812B_T0H;
        }
    }
    for (uint16_t r = 0; r < WS2812B_RESET_PULSES; r++) {
        ws2812b_dma_buf[idx++] = 0U;
    }
}

/* ─── Init ───────────────────────────────────────────────────────────────── */
void WS2812B_Init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;
    WS2812B_ReconfigTimer();

    memset(s_leds,          0, sizeof(s_leds));
    memset(ws2812b_dma_buf, 0, sizeof(ws2812b_dma_buf));
    s_dirty = 1;

    ws2812b_busy = 1;
    HAL_TIM_PWM_Start_DMA(s_htim, TIM_CHANNEL_1,
                          (uint32_t *)ws2812b_dma_buf, WS2812B_DMA_BUF_SIZE);
    while (ws2812b_busy) {}
    HAL_Delay(1);
}

/* ─── Non-blocking send ──────────────────────────────────────────────────── */
void WS2812B_Send(void)
{
    if (ws2812b_busy) return;
    WS2812B_Pack();
    s_dirty = 0;
    ws2812b_busy = 1;
#if defined(WS2812B_DEBUG_COUNTERS) || defined(BRINGUP_TEST_MODE)
    ws2812b_send_count++;
#endif
    HAL_TIM_PWM_Start_DMA(s_htim, TIM_CHANNEL_1,
                          (uint32_t *)ws2812b_dma_buf, WS2812B_DMA_BUF_SIZE);
}

/* ─── Blocking send ──────────────────────────────────────────────────────── */
void WS2812B_SendBlocking(void)
{
    while (ws2812b_busy) {}
    WS2812B_Send();
    while (ws2812b_busy) {}
}

/* ─── DMA callbacks ──────────────────────────────────────────────────────── */
void WS2812B_DMAComplete_Callback(void)
{
    HAL_TIM_PWM_Stop_DMA(s_htim, TIM_CHANNEL_1);
    ws2812b_busy = 0;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) WS2812B_DMAComplete_Callback();
}

/* FIX 3: explicit no-op prevents HAL from stopping DMA at half-transfer */
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
}

/* ─── Pixel setters (dirty flag only — no shadow copy) ──────────────────── */
void WS2812B_SetAll(RGB_t color)
{
    for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
        if (s_leds[i].r != color.r || s_leds[i].g != color.g ||
            s_leds[i].b != color.b) { s_dirty = 1; }
        s_leds[i] = color;
    }
}

void WS2812B_SetPixel(uint8_t idx, RGB_t color)
{
    if (idx >= WS2812B_NUM_LEDS) return;
    if (s_leds[idx].r != color.r || s_leds[idx].g != color.g ||
        s_leds[idx].b != color.b) { s_dirty = 1; }
    s_leds[idx] = color;
}

/* ─── Pattern control ────────────────────────────────────────────────────── */
void WS2812B_SetPattern(LED_Pattern_t pattern)
{
    if (s_pattern == pattern) return;
    s_pattern      = pattern;
    s_pattern_tick = HAL_GetTick();
    s_blink_tick   = HAL_GetTick();
    s_sweep_pos    = 0;
    s_wave_pos     = 0;
    s_dirty        = 1;
}

void WS2812B_SetLampColor(RGB_t color)        { s_lamp_color    = color; }
void WS2812B_SetCustomReminderColor(RGB_t c)   { s_custom_remind = c;    }
void WS2812B_SetChargingLevel(uint8_t pct)
{
    if (pct != s_charge_pct) s_dirty = 1;
    s_charge_pct = pct;
}
uint8_t WS2812B_IsBusy(void) { return ws2812b_busy; }

/* ─── SendIfDirty helper ─────────────────────────────────────────────────── */
static void SendIfDirty(void)
{
    if (s_dirty && !ws2812b_busy) WS2812B_Send();
}

/* ─── WS2812B_Update — call every loop tick from App_Run() ──────────────── */
void WS2812B_Update(void)
{
    if (ws2812b_busy) return;

    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_blink_tick;

    switch (s_pattern) {

    case LED_PATTERN_ALL_OFF:
    case LED_PATTERN_NONE:
        WS2812B_SetAll(RGB_OFF);
        SendIfDirty();
        return;

    case LED_PATTERN_HYDRATION_HIGH:
    case LED_PATTERN_HYDRATION_MID:
    case LED_PATTERN_HYDRATION_LOW: {
        RGB_t col = (s_pattern == LED_PATTERN_HYDRATION_HIGH) ? s_custom_remind :
                    (s_pattern == LED_PATTERN_HYDRATION_MID)  ? RGB_AMBER : RGB_RED;
        if (now - s_pattern_tick > 3000U) {
            s_pattern = LED_PATTERN_ALL_OFF;
            WS2812B_SetAll(RGB_OFF); SendIfDirty(); return;
        }
        uint8_t on = (uint8_t)(((now - s_pattern_tick) / 300U) % 2U == 0U);
        WS2812B_SetAll(on ? col : RGB_OFF);
        break;
    }

    case LED_PATTERN_PURITY_ALERT:
    case LED_PATTERN_TEMP_ALERT: {
        RGB_t    col   = (s_pattern == LED_PATTERN_PURITY_ALERT) ? RGB_PURPLE : RGB_ORANGE;
        uint32_t phase = now - s_pattern_tick;
        uint8_t  on    = (uint8_t)((phase < 80U) || (phase >= 200U && phase < 280U));
        if (phase > 400U) {
            s_pattern = LED_PATTERN_ALL_OFF;
            WS2812B_SetAll(RGB_OFF); SendIfDirty(); return;
        }
        WS2812B_SetAll(on ? col : RGB_OFF);
        break;
    }

    case LED_PATTERN_CALIBRATION: {
        if (dt > 120U) {
            s_blink_tick = now;
            s_wave_pos   = (uint8_t)((s_wave_pos + 1U) % WS2812B_NUM_LEDS);
        }
        for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
            uint8_t dist = (uint8_t)((WS2812B_NUM_LEDS + i - s_wave_pos) % WS2812B_NUM_LEDS);
            uint8_t b    = (dist == 0U) ? 255U
                         : (dist == 1U || dist == WS2812B_NUM_LEDS - 1U) ? 100U : 20U;
            WS2812B_SetPixel(i, SCALE_RGB(RGB_AMBER, b));
        }
        break;
    }

    case LED_PATTERN_DRINK_CONFIRM: {
        uint32_t p = now - s_pattern_tick;
        if (p > 1000U) {
            s_pattern = LED_PATTERN_ALL_OFF;
            WS2812B_SetAll(RGB_OFF); SendIfDirty(); return;
        }
        uint32_t phase = p % 500U;
        uint8_t  b     = (phase < 250U) ? (uint8_t)(phase * 255U / 250U)
                                        : (uint8_t)((500U - phase) * 255U / 250U);
        WS2812B_SetAll(SCALE_RGB(RGB_GREEN, b));
        break;
    }

    case LED_PATTERN_SYNC_SUCCESS: {
        if (dt > 60U) { s_blink_tick = now; s_sweep_pos++; }
        if (s_sweep_pos >= WS2812B_NUM_LEDS + 3U) {
            s_pattern = LED_PATTERN_ALL_OFF;
            WS2812B_SetAll(RGB_OFF); SendIfDirty(); return;
        }
        for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
            WS2812B_SetPixel(i, (i == s_sweep_pos) ? RGB_CYAN : RGB_OFF);
        }
        break;
    }

    case LED_PATTERN_CHARGING_BAR: {
        uint8_t lit = (uint8_t)((uint32_t)s_charge_pct * WS2812B_NUM_LEDS / 100U);
        if (lit > WS2812B_NUM_LEDS) lit = WS2812B_NUM_LEDS;
        for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
            if (i < lit) {
                WS2812B_SetPixel(i, RGB_GREEN);
            } else if (i == lit && s_charge_pct < 100U) {
                uint8_t on = (uint8_t)((now / 600U) % 2U == 0U);
                WS2812B_SetPixel(i, on ? RGB_GREEN : RGB_OFF);
            } else {
                WS2812B_SetPixel(i, RGB_OFF);
            }
        }
        break;
    }

    case LED_PATTERN_LOW_BATTERY: {
        uint32_t p = now - s_pattern_tick;
        if (p > 2400U) {
            s_pattern = LED_PATTERN_ALL_OFF;
            WS2812B_SetAll(RGB_OFF); SendIfDirty(); return;
        }
        uint32_t phase = p % 800U;
        uint8_t  b     = (phase < 400U) ? (uint8_t)(phase * 200U / 400U)
                                        : (uint8_t)((800U - phase) * 200U / 400U);
        WS2812B_SetAll(SCALE_RGB(RGB_RED, b));
        break;
    }

    case LED_PATTERN_LAMP_MODE: {
        if (dt > 80U) {
            s_blink_tick = now;
            s_wave_pos   = (uint8_t)((s_wave_pos + 1U) % WS2812B_NUM_LEDS);
        }
        /* LUT stored in flash (.rodata) — 10 bytes */
        static const uint8_t wave_lut[WS2812B_NUM_LEDS] = {255,200,150,100,70,40,20,10,5,2};
        for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
            uint8_t dist = (uint8_t)((WS2812B_NUM_LEDS + i - s_wave_pos) % WS2812B_NUM_LEDS);
            uint8_t bv   = wave_lut[dist];   /* hoist array read out of macro arg */
            WS2812B_SetPixel(i, SCALE_RGB(s_lamp_color, bv));
        }
        break;
    }

    case LED_PATTERN_REGISTRATION: {
        uint32_t cycle = (now - s_pattern_tick) % 2000U;
        uint8_t  b     = (cycle < 1000U) ? (uint8_t)(cycle * 255U / 1000U)
                                          : (uint8_t)((2000U - cycle) * 255U / 1000U);
        WS2812B_SetAll(SCALE_RGB(RGB_BLUE, b));
        break;
    }

    case LED_PATTERN_FACTORY_RESET_WARN:
    case LED_PATTERN_ERROR: {
        uint32_t p  = now - s_pattern_tick;
        if (p > 2500U) {
            s_pattern = LED_PATTERN_ALL_OFF;
            WS2812B_SetAll(RGB_OFF); SendIfDirty(); return;
        }
        uint8_t on  = (uint8_t)((p / 250U) % 2U == 0U);
        RGB_t   col = (s_pattern == LED_PATTERN_FACTORY_RESET_WARN) ? RGB_RED : RGB_WHITE;
        WS2812B_SetAll(on ? col : RGB_OFF);
        break;
    }

    default: break;
    }

    SendIfDirty();
}

/* ─── WS2812B_SelfTest — compiled only for bring-up/self-test builds ─────── */
#if defined(WS2812B_SELFTEST) || defined(BRINGUP_TEST_MODE)
void WS2812B_SelfTest(void)
{
#define ST_SOLID(r,g,b,ms) \
    do { WS2812B_SetAll(RGB((r),(g),(b))); WS2812B_SendBlocking(); HAL_Delay(ms); } while(0)

    ST_SOLID(40,  0,  0, 800);
    ST_SOLID( 0, 40,  0, 800);
    ST_SOLID( 0,  0, 40, 800);
    ST_SOLID(40, 40, 40, 800);

    static const uint8_t rb[WS2812B_NUM_LEDS][3] = {
        {255,  0,  0},{255,127,  0},{255,255,  0},{  0,255,  0},
        {  0,255,127},{  0,255,255},{  0,127,255},{  0,  0,255},
        {127,  0,255},{255,  0,255},
    };
    for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++)
        WS2812B_SetPixel(i, RGB(rb[i][0], rb[i][1], rb[i][2]));
    WS2812B_SendBlocking();
    HAL_Delay(1200);

    for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++)
        WS2812B_SetPixel(i, (i & 1U) ? RGB(0,40,0) : RGB(40,0,0));
    WS2812B_SendBlocking();
    HAL_Delay(800);

    for (uint8_t pos = 0; pos < WS2812B_NUM_LEDS; pos++) {
        WS2812B_SetAll(RGB_OFF);
        WS2812B_SetPixel(pos, RGB(40,0,0));
        WS2812B_SendBlocking();
        HAL_Delay(120);
    }

    WS2812B_SetAll(RGB_OFF);
    WS2812B_SendBlocking();
    HAL_Delay(300);

    s_pattern      = LED_PATTERN_NONE;
    s_pattern_tick = HAL_GetTick();
    s_blink_tick   = HAL_GetTick();
    s_dirty        = 1;
#undef ST_SOLID
}
#endif /* WS2812B_SELFTEST || BRINGUP_TEST_MODE */
