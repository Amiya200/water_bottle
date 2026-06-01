/*
 * ws2812b.c  –  WS2812B driver for STM32F030K6T6 (TIM1_CH1 + DMA1_CH2)
 *
 * Protocol  : 800 kHz NZR, GRB byte order, MSB first  (datasheet page 3/5)
 * Mechanism : TIM1 CH1 PWM — each CCR value encodes one bit's HIGH duration.
 *             DMA streams ws2812b_dma_buf[] (uint16_t) into TIM1->CCR1 (HalfWord).
 *
 * ==========================================================================
 * BUG FIXES (5 issues ported from the proven ws2812.c test build)
 * ==========================================================================
 *
 * FIX 1 – DMA direction was PERIPH→MEMORY (stm32f0xx_hal_msp.c)
 *   The CubeMX-generated MSP set Direction = DMA_PERIPH_TO_MEMORY.
 *   Data must flow MEMORY→PERIPHERAL (RAM → TIM1->CCR1).
 *   ** Fix is in stm32f0xx_hal_msp.c — see that file. **
 *
 * FIX 2 – DMA buffer changed from uint8_t to uint16_t
 *   Old code used uint8_t s_dma_buf[] and tried to patch DMA MSIZE at
 *   runtime. The runtime CCR bit-twiddling is fragile and was inconsistent
 *   with the HAL DMA handle's cached Init values, causing DMA config errors
 *   on second and subsequent Send() calls.
 *   FIX: buffer is now uint16_t ws2812b_dma_buf[300]. DMA config (already
 *   HalfWord/HalfWord from CubeMX after FIX 1 corrects the direction) is
 *   used as-is. No runtime patching needed.
 *
 * FIX 3 – Half-transfer callback was missing (advanced timer TIM1)
 *   TIM1 is an advanced timer; its DMA generates both a half-transfer and a
 *   full-transfer IRQ. Without overriding
 *   HAL_TIM_PWM_PulseFinishedHalfCpltCallback as a no-op, some HAL versions
 *   stop the DMA at the halfway point, corrupting the second half of every
 *   LED frame.
 *   FIX: override added as explicit no-op below.
 *
 * FIX 4 – Power-on reset frame missing
 *   On cold boot PA8 floated until TIM1 alt-func became active; LEDs could
 *   latch random noise bits.
 *   FIX: WS2812B_Init() sends one all-zero frame and waits 300 µs.
 *
 * FIX 5 – ws2812b_busy flag race
 *   Old code set s_busy = 1 AFTER calling HAL_TIM_PWM_Start_DMA. A very
 *   fast DMA completion + ISR could clear busy before the assignment ran.
 *   FIX: ws2812b_busy = 1 is set BEFORE starting DMA.
 *
 * ==========================================================================
 * CubeMX / stm32f0xx_hal_msp.c REQUIRED settings for DMA1_Channel2
 * ==========================================================================
 *   Direction           = DMA_MEMORY_TO_PERIPH   ← was wrong (PERIPH_TO_MEMORY)
 *   PeriphInc           = DISABLE
 *   MemInc              = ENABLE
 *   PeriphDataAlignment = HALFWORD
 *   MemDataAlignment    = HALFWORD
 *   Mode                = NORMAL
 *   Priority            = LOW (or MEDIUM)
 *
 * ==========================================================================
 * Buffer layout (300 uint16_t words)
 * ==========================================================================
 *   [0 … 239]   10 LEDs × 24 bits → CCR values (WS2812B_T0H or WS2812B_T1H)
 *   [240 … 299] 60 zero-CCR words → ≥75 µs reset gap
 * ==========================================================================
 */

#pragma GCC optimize("Os")
#include "ws2812b.h"
#include <string.h>
#include <stdlib.h>   /* rand() for SelfTest sparkle */

/* ─── DMA buffer – MUST be uint16_t (HalfWord) ─────────────────────────── */
/*
 * Declared non-static so the bring-up test / Live Expressions can watch it.
 * Passed to HAL as (uint32_t*) because the HAL API requires that pointer
 * type, but the actual transfer width is governed by the DMA configuration
 * (HalfWord / HalfWord). Do NOT change to uint32_t[].
 */
static uint16_t ws2812b_dma_buf[WS2812B_DMA_BUF_SIZE];

/* LED framebuffer and shadow (for dirty tracking) */
static RGB_t    s_leds[WS2812B_NUM_LEDS];
static RGB_t    s_leds_shadow[WS2812B_NUM_LEDS];

static TIM_HandleTypeDef *s_htim = NULL;

/* Exposed for Live Expressions and bring-up test */
volatile uint8_t  ws2812b_busy       = 0;
volatile uint32_t ws2812b_send_count = 0;

/* Pattern state */
static LED_Pattern_t s_pattern       = LED_PATTERN_NONE;
static uint32_t      s_pattern_tick  = 0;
static uint32_t      s_blink_tick    = 0;
static uint8_t       s_pulse_count   = 0;
static uint8_t       s_sweep_pos     = 0;
static uint8_t       s_wave_pos      = 0;
static RGB_t         s_lamp_color    = {255, 255, 255};
static RGB_t         s_custom_remind = {0,   255, 0};
static uint8_t       s_charge_pct    = 0;
static uint8_t       s_dirty         = 1;

/* =========================================================================
 * Internal: reconfigure TIM1 for 800 kHz WS2812B bit clock
 * =========================================================================
 * CubeMX generates TIM1 in slave/external-clock mode (SMCR.SMS = External
 * Clock Mode 1, TS = ITR0) because of the .ioc configuration. In that mode
 * TIM1 counts only on trigger edges from another timer and produces no PWM
 * from its own 48 MHz clock. This function clears SMCR and sets PSC/ARR so
 * TIM1 runs freely at 48 MHz / 60 = 800 kHz.
 */
static void WS2812B_ReconfigTimer(void)
{
    HAL_TIM_PWM_Stop_DMA(s_htim, TIM_CHANNEL_1);

    /* Clear slave mode — TIM1 must run from internal 48 MHz clock */
    s_htim->Instance->SMCR = 0U;

    /* 48 MHz / (ARR+1) = 48e6 / 60 = 800 kHz bit clock */
    s_htim->Instance->PSC = 0U;
    s_htim->Instance->ARR = 59U;

    /* Force register update */
    s_htim->Instance->EGR = TIM_EGR_UG;
}

/* =========================================================================
 * Internal: pack s_leds[] → ws2812b_dma_buf[] and start DMA
 * =========================================================================
 * Datasheet page 5: wire order is G7..G0 | R7..R0 | B7..B0 (GRB, MSB first)
 */
static void WS2812B_Pack(void)
{
    uint16_t idx = 0;
    for (uint8_t led = 0; led < WS2812B_NUM_LEDS; led++) {
        /* Pack as 24-bit GRB */
        uint32_t grb = ((uint32_t)s_leds[led].g << 16)
                     | ((uint32_t)s_leds[led].r <<  8)
                     |  (uint32_t)s_leds[led].b;
        for (int8_t bit = 23; bit >= 0; bit--) {
            ws2812b_dma_buf[idx++] = (grb & (1UL << bit))
                                     ? WS2812B_T1H : WS2812B_T0H;
        }
    }
    /* Reset gap: 60 × 1.25 µs = 75 µs (≥50 µs required) */
    for (uint16_t r = 0; r < WS2812B_RESET_PULSES; r++) {
        ws2812b_dma_buf[idx++] = 0U;
    }
}

/* =========================================================================
 * WS2812B_Init
 * =========================================================================
 * Must be called after MX_TIM1_Init() and MX_DMA_Init().
 */
void WS2812B_Init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;

    WS2812B_ReconfigTimer();

    memset(s_leds,          0,    sizeof(s_leds));
    memset(s_leds_shadow,   0xFF, sizeof(s_leds_shadow)); /* != s_leds → dirty */
    memset(ws2812b_dma_buf, 0,    sizeof(ws2812b_dma_buf));
    s_dirty = 1;

    /*
     * FIX 4: power-on reset frame.
     * Send one complete all-zeros frame so every LED latches 24 zero bits and
     * turns off, clearing any power-on noise. Then wait 1 ms extra margin.
     * The reset gap (60 zero words = 75 µs) is already inside the buffer.
     */
    ws2812b_busy = 1;   /* FIX 5: set BEFORE starting DMA */
    HAL_TIM_PWM_Start_DMA(s_htim, TIM_CHANNEL_1,
                          (uint32_t *)ws2812b_dma_buf,
                          WS2812B_DMA_BUF_SIZE);
    while (ws2812b_busy) { /* wait for callback to clear busy */ }
    HAL_Delay(1);   /* 1 ms extra on top of the 75 µs reset in buffer */
}

/* =========================================================================
 * WS2812B_Send  — non-blocking, returns immediately if DMA is in progress
 * =========================================================================
 */
void WS2812B_Send(void)
{
    if (ws2812b_busy) return;

    WS2812B_Pack();
    memcpy(s_leds_shadow, s_leds, sizeof(s_leds_shadow));
    s_dirty = 0;

    ws2812b_busy = 1;   /* FIX 5: set BEFORE starting DMA */
    ws2812b_send_count++;
    HAL_TIM_PWM_Start_DMA(s_htim, TIM_CHANNEL_1,
                          (uint32_t *)ws2812b_dma_buf,   /* HAL needs uint32_t* */
                          WS2812B_DMA_BUF_SIZE);
}

/* =========================================================================
 * WS2812B_SendBlocking — wait for any running transfer, then send and wait
 * =========================================================================
 * Use in self-test / init paths where you need a guaranteed visible result.
 * Do NOT call from an ISR.
 */
void WS2812B_SendBlocking(void)
{
    while (ws2812b_busy) { /* spin — previous frame still running */ }
    WS2812B_Send();
    while (ws2812b_busy) { /* spin — wait for this frame to finish */ }
}

/* =========================================================================
 * DMA callbacks
 * =========================================================================
 */

/*
 * Full-transfer complete: all CCR values have been written.
 * Stop PWM so PA8 idles LOW (required state before the next reset gap).
 */
void WS2812B_DMAComplete_Callback(void)
{
    HAL_TIM_PWM_Stop_DMA(s_htim, TIM_CHANNEL_1);
    ws2812b_busy = 0;
}

/*
 * HAL hook — TIM1 DMA transfer-complete.
 * Gate on TIM1->Instance to avoid interfering with other timers that may
 * also use HAL_TIM_PWM_PulseFinishedCallback.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        WS2812B_DMAComplete_Callback();
    }
}

/*
 * FIX 3: Half-transfer callback — explicit no-op.
 * TIM1 is an advanced timer; its DMA raises a half-transfer IRQ at the
 * midpoint of the burst. Without this override some HAL versions invoke
 * Stop_DMA here, which kills the transfer after only 150 words and corrupts
 * LEDs 6-10. Override as a strict no-op.
 */
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;   /* intentionally empty — do NOT clear ws2812b_busy here */
}

/* =========================================================================
 * Pixel setters (with dirty tracking)
 * =========================================================================
 */

void WS2812B_SetAll(RGB_t color)
{
    for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
        if (s_leds[i].r != color.r || s_leds[i].g != color.g ||
            s_leds[i].b != color.b) {
            s_dirty = 1;
        }
        s_leds[i] = color;
    }
}

void WS2812B_SetPixel(uint8_t idx, RGB_t color)
{
    if (idx >= WS2812B_NUM_LEDS) return;
    if (s_leds[idx].r != color.r || s_leds[idx].g != color.g ||
        s_leds[idx].b != color.b) {
        s_dirty = 1;
    }
    s_leds[idx] = color;
}

/* =========================================================================
 * Pattern control
 * =========================================================================
 */

void WS2812B_SetPattern(LED_Pattern_t pattern)
{
    if (s_pattern == pattern) return;
    s_pattern      = pattern;
    s_pattern_tick = HAL_GetTick();
    s_blink_tick   = HAL_GetTick();
    s_pulse_count  = 0;
    s_sweep_pos    = 0;
    s_wave_pos     = 0;
    s_dirty        = 1;
}

void WS2812B_SetLampColor(RGB_t color)         { s_lamp_color    = color; }
void WS2812B_SetCustomReminderColor(RGB_t c)    { s_custom_remind = c;    }
void WS2812B_SetChargingLevel(uint8_t pct)
{
    if (pct != s_charge_pct) s_dirty = 1;
    s_charge_pct = pct;
}

uint8_t WS2812B_IsBusy(void) { return ws2812b_busy; }

/* =========================================================================
 * Internal helpers
 * =========================================================================
 */

static RGB_t ScaleBright(RGB_t c, uint8_t b)
{
    return RGB((uint8_t)((uint16_t)c.r * b / 255U),
               (uint8_t)((uint16_t)c.g * b / 255U),
               (uint8_t)((uint16_t)c.b * b / 255U));
}

static void SendIfDirty(void)
{
    if (s_dirty && !ws2812b_busy) WS2812B_Send();
}

/* =========================================================================
 * WS2812B_Update — call every loop tick from App_Run()
 * =========================================================================
 * Non-blocking: returns immediately if DMA is running. All timing uses
 * HAL_GetTick() deltas; never calls HAL_Delay().
 */
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
            WS2812B_SetPixel(i, ScaleBright(RGB_AMBER, b));
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
        WS2812B_SetAll(ScaleBright(RGB_GREEN, b));
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
        WS2812B_SetAll(ScaleBright(RGB_RED, b));
        break;
    }

    case LED_PATTERN_LAMP_MODE: {
        if (dt > 80U) {
            s_blink_tick = now;
            s_wave_pos   = (uint8_t)((s_wave_pos + 1U) % WS2812B_NUM_LEDS);
        }
        static const uint8_t wave_lut[] = {255,200,150,100,70,40,20,10,5,2};
        for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
            uint8_t dist = (uint8_t)((WS2812B_NUM_LEDS + i - s_wave_pos) % WS2812B_NUM_LEDS);
            WS2812B_SetPixel(i, ScaleBright(s_lamp_color, wave_lut[dist]));
        }
        break;
    }

    case LED_PATTERN_REGISTRATION: {
        uint32_t cycle = (now - s_pattern_tick) % 2000U;
        uint8_t  b     = (cycle < 1000U) ? (uint8_t)(cycle * 255U / 1000U)
                                          : (uint8_t)((2000U - cycle) * 255U / 1000U);
        WS2812B_SetAll(ScaleBright(RGB_BLUE, b));
        break;
    }

    case LED_PATTERN_FACTORY_RESET_WARN:
    case LED_PATTERN_ERROR: {
        uint32_t p   = now - s_pattern_tick;
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

/* =========================================================================
 * WS2812B_SelfTest
 * =========================================================================
 * Runs a deterministic colour-walk identical to the proven ws2812.c test
 * build. Call this after WS2812B_Init() to verify the full LED chain is
 * alive before the application starts. Takes ~8 seconds at the default
 * delays.
 *
 * Phases (all at reduced brightness 40/255 to avoid power surge):
 *   1. ALL RED    800 ms
 *   2. ALL GREEN  800 ms
 *   3. ALL BLUE   800 ms
 *   4. ALL WHITE  800 ms
 *   5. RAINBOW    10 unique hues, 1200 ms
 *   6. ALTERNATE  even=RED / odd=GREEN, 800 ms
 *   7. PIXEL WALK single pixel steps through all 10 LEDs, 120 ms each
 *   8. ALL OFF    300 ms
 *
 * GRB wire-packet reference (at R/G/B=40):
 *   Red     GRB = 0x00 0x28 0x00
 *   Green   GRB = 0x28 0x00 0x00
 *   Blue    GRB = 0x00 0x00 0x28
 *   White   GRB = 0x28 0x28 0x28
 */
void WS2812B_SelfTest(void)
{
    /* Helper: set all, send, wait */
#define ST_SOLID(r,g,b,ms) \
    do { \
        WS2812B_SetAll(RGB((r),(g),(b))); \
        WS2812B_SendBlocking(); \
        HAL_Delay(ms); \
    } while(0)

    /* Phase 1-4: solid colours */
    ST_SOLID(40,  0,  0, 800);   /* RED   */
    ST_SOLID( 0, 40,  0, 800);   /* GREEN */
    ST_SOLID( 0,  0, 40, 800);   /* BLUE  */
    ST_SOLID(40, 40, 40, 800);   /* WHITE */

    /* Phase 5: rainbow — 10 distinct hues, stored as {R,G,B} */
    static const uint8_t rb[WS2812B_NUM_LEDS][3] = {
        {255,   0,   0},  /* LED01  Red     GRB=0x00FF00 */
        {255, 127,   0},  /* LED02  Orange  GRB=0x7FFF00 */
        {255, 255,   0},  /* LED03  Yellow  GRB=0xFFFF00 */
        {  0, 255,   0},  /* LED04  Green   GRB=0xFF0000 */
        {  0, 255, 127},  /* LED05  Spring  GRB=0xFF007F */
        {  0, 255, 255},  /* LED06  Cyan    GRB=0xFF00FF */
        {  0, 127, 255},  /* LED07  Azure   GRB=0x7F00FF */
        {  0,   0, 255},  /* LED08  Blue    GRB=0x0000FF */
        {127,   0, 255},  /* LED09  Violet  GRB=0x007FFF */
        {255,   0, 255},  /* LED10  Magenta GRB=0x00FFFF */
    };
    for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
        WS2812B_SetPixel(i, RGB(rb[i][0], rb[i][1], rb[i][2]));
    }
    WS2812B_SendBlocking();
    HAL_Delay(1200);

    /* Phase 6: alternate even=RED / odd=GREEN */
    for (uint8_t i = 0; i < WS2812B_NUM_LEDS; i++) {
        WS2812B_SetPixel(i, (i & 1U) ? RGB(0, 40, 0) : RGB(40, 0, 0));
    }
    WS2812B_SendBlocking();
    HAL_Delay(800);

    /* Phase 7: single RED pixel walk */
    for (uint8_t pos = 0; pos < WS2812B_NUM_LEDS; pos++) {
        WS2812B_SetAll(RGB_OFF);
        WS2812B_SetPixel(pos, RGB(40, 0, 0));
        WS2812B_SendBlocking();
        HAL_Delay(120);
    }

    /* Phase 8: all off */
    WS2812B_SetAll(RGB_OFF);
    WS2812B_SendBlocking();
    HAL_Delay(300);

    /*
     * Restore pattern state so the app's WS2812B_Update() can take over
     * cleanly. Force NONE so the first Update() re-evaluates from scratch.
     */
    s_pattern      = LED_PATTERN_NONE;
    s_pattern_tick = HAL_GetTick();
    s_blink_tick   = HAL_GetTick();
    s_dirty        = 1;

#undef ST_SOLID
}
