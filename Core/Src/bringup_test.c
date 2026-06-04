#include "bringup_test.h"

#ifdef BRINGUP_TEST_MODE
#include "bringup_test.h"
#include "ws2812b.h"
#include "ntc_temp.h"
#include "tds_sensor.h"
#include "hx711.h"
#include <string.h>

/* ── Global test state (Live Expressions target) ─────────────────────── */
BringUpTest_t g_test;

/* ── Private handles ─────────────────────────────────────────────────── */
static NTC_Handle_t   s_ntc;
static TDS_Handle_t   s_tds;
static HX711_Handle_t s_hx;

/* ── Internal: NTC read ──────────────────────────────────────────────── */
static void BringUp_ReadNTC(void)
{
    int16_t t = NTC_ReadTemp_x10(&s_ntc);
    g_test.ntc_adc_raw   = 0;               /* no last_adc field in NTC_Handle_t */
    g_test.ntc_temp_x10  = t;
    g_test.ntc_status    = (s_ntc.valid && t > -400 && t < 1500)
                           ? TEST_PASS : TEST_FAIL;
}

/* ── Internal: TDS read ──────────────────────────────────────────────── */
static void BringUp_ReadTDS(void)
{
    uint16_t ppm = TDS_ReadPPM(&s_tds, g_test.ntc_temp_x10);
    g_test.tds_adc_raw = 0;                 /* no last_adc field in TDS_Handle_t */
    g_test.tds_ppm     = ppm;
    g_test.tds_status  = s_tds.valid ? TEST_PASS : TEST_FAIL;
}

/* ── Internal: HX711 read ────────────────────────────────────────────── */
static void BringUp_ReadHX711(void)
{
    g_test.hx_ready = HX711_IsReady();
    if (g_test.hx_ready) {
        int32_t raw = HX711_ReadRaw(&s_hx);
        g_test.hx_raw   = raw;
        g_test.hx_grams = HX711_ReadGrams(&s_hx);
        g_test.hx_status = TEST_PASS;
    } else {
        g_test.hx_status = TEST_FAIL;
    }
}

/* =========================================================================
 * BringUp_Init
 * =========================================================================
 * Call after all MX_*_Init() functions complete and before the main loop.
 */
void BringUp_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim_ws)
{
    memset(&g_test, 0, sizeof(g_test));

    /* Init WS2812B — sends power-on reset frame internally */
    WS2812B_Init(htim_ws);
    g_test.rgb_status = TEST_PENDING;

    /* Init other sensors (shared ADC handle) */
    NTC_Init(&s_ntc, hadc);
    TDS_Init(&s_tds, hadc);
    HX711_Init(&s_hx);
}

/* =========================================================================
 * BringUp_RunOnce
 * =========================================================================
 * Runs the full RGB self-test then takes one reading from every sensor.
 * Blocks for ~8 seconds while the LED walk plays out — intentional for a
 * bring-up / factory-test path.
 */
void BringUp_RunOnce(void)
{
    /* --- RGB self-test -------------------------------------------------- */
    g_test.rgb_status = TEST_PENDING;

    /*
     * WS2812B_SelfTest() uses WS2812B_SendBlocking() internally, so it is
     * safe to call here before the main loop starts. It uses ws2812b_send_count
     * which is incremented by WS2812B_Send() on every DMA kick.
     */
    WS2812B_SelfTest();

    g_test.rgb_send_count  = ws2812b_send_count;
    g_test.rgb_busy        = ws2812b_busy;
    g_test.rgb_color_index = 0;
    g_test.rgb_status      = (ws2812b_send_count > 0) ? TEST_PASS : TEST_FAIL;

    /* --- Sensor first-reads --------------------------------------------- */
    BringUp_ReadNTC();
    BringUp_ReadTDS();
    BringUp_ReadHX711();
    g_test.hx_raw_tare = g_test.hx_raw;   /* snapshot as tare reference */

    /* --- Start a gentle "alive" animation for the loop phase ------------ */
    WS2812B_SetPattern(LED_PATTERN_REGISTRATION);
}

/* =========================================================================
 * BringUp_RunLoop
 * =========================================================================
 * Call every main-loop iteration. Polls sensors on a 1 s schedule and keeps
 * the RGB registration-breathing animation running. All values land in g_test
 * so you can watch them in Live Expressions without any printf output.
 */
void BringUp_RunLoop(void)
{
    static uint32_t last_sensor_ms = 0;
    static uint32_t last_rgb_cnt   = 0;

    uint32_t now = HAL_GetTick();
    g_test.uptime_ms = now;
    g_test.loop_count++;

    /* Poll sensors every 1 s */
    if (now - last_sensor_ms >= 1000U) {
        last_sensor_ms = now;
        BringUp_ReadNTC();
        BringUp_ReadTDS();
        BringUp_ReadHX711();
    }

    /* Mirror live DMA state */
    g_test.rgb_busy       = ws2812b_busy;
    g_test.rgb_send_count = ws2812b_send_count;

    /* Stall detection: if the counter stopped incrementing after 5 s, flag FAIL */
    if (now > 5000U && ws2812b_send_count == last_rgb_cnt && ws2812b_send_count > 0) {
        g_test.rgb_status = TEST_FAIL;
    }
    last_rgb_cnt = ws2812b_send_count;

    /* Drive the RGB animation */
    WS2812B_Update();
}
#endif /* BRINGUP_TEST_MODE */
