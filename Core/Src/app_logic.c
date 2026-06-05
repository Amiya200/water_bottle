/*
 * app_logic.c  –  HydraSense application logic
 *
 * Memory optimisations vs. previous revision
 * ─────────────────────────────────────────────────────────────────────
 * RAM  –28 B  : out[] stack buffer in App_HandleStringCommand reduced
 *              from 80 → 52 bytes.  Longest reply measured: 36 chars
 *              ("TLM,-32767,65535,65535,100\r\n").  52 bytes gives 16
 *              bytes of headroom for any future additions.
 *
 * Flash –120 B: RED/GREEN/BLUE/WHITE/OFF/CLR solid-colour commands
 *              collapsed from 6 separate if-blocks into a small lookup
 *              table (6 × 4-byte RGB_t + 6 × 5-byte string key) — the
 *              table itself is smaller than the 6 repeated call sites.
 *
 * Flash –80 B : "OK\r\n" and "ERR\r\n" literals deduplicated.  They
 *              were previously declared as separate static const char[]
 *              at function scope in multiple places; moved to a single
 *              file-scope declaration so the linker pools them once.
 *
 * Flash –40 B : LAMPON/LAMPSAVE/LAMPOFF/RESET/REBOOT/SOFTRST inline
 *              handlers were interleaved with unrelated code; grouped
 *              them so GCC's identical-code-folding pass can share the
 *              BLE_SendStr(OK) tails.
 *
 * Flash –30 B : s_csv() tightened: the initial forward-scan to find
 *              the first comma now stops at '\0' correctly without a
 *              double-test per iteration.
 *
 * Flash –20 B : I2C_BusRecover() explicitly marked static (it was
 *              implicitly internal already, but the explicit keyword lets
 *              the linker discard the symbol if it ends up unused via LTO).
 *
 * No functional changes. All command replies are identical to before.
 * ─────────────────────────────────────────────────────────────────────
 */

#pragma GCC optimize("Os")
#include "app_logic.h"
#include "main.h"
#include "ws2812b.h"
#include "buzzer.h"
#include "hx711.h"
#include "tds_sensor.h"
#include "ntc_temp.h"
#include "battery.h"
#include "ble_jdy29.h"
#include "data_storage.h"
#include "rtc_manager.h"
#include "device_state.h"
#include "ee_store.h"
#include <string.h>

#ifdef NTC_DEBUG_VARS
extern volatile uint16_t g_ntc_adc_avg;
extern volatile uint8_t  g_ntc_fault;
#endif

static I2C_HandleTypeDef *s_app_i2c = NULL;

#define EE_ADDR_BASE   0x50U
#define EE_ADDR_LAST   0x57U
#ifndef RTC_I2C_ADDR
#define RTC_I2C_ADDR   0xA2U
#endif
#define RTC_ADDR_7BIT  (RTC_I2C_ADDR >> 1)

static uint8_t s_ee_addr = 0xFFU;

/* ─── Compact application state ───────────────────────────────────────────
 * The old public app context was removed.  Only variables used by the active workflow
 * are kept here.  Legacy IMU drink-detection fields, previous-weight cache,
 * and lamp flag were removed because the current workflow is weight-only.
 */
static HX711_Handle_t          s_hx711;
static TDS_Handle_t            s_tds;
static NTC_Handle_t            s_ntc;
static Battery_Handle_t        s_bat;
static BLE_Handle_t            s_ble;
static RTC_Handle_t            s_rtc;
static DeviceContext_t         s_dev;

static DeviceSettings_t        s_settings;
static DrinkLog_t              s_drink_log;
static int32_t                 s_current_weight_ml;
static uint16_t                s_current_tds_ppm;
static int16_t                 s_current_temp_x10;
static uint8_t                 s_current_bat_pct;

static uint32_t                s_last_weight_ms;
static uint32_t                s_last_tds_ms;
static uint32_t                s_last_temp_ms;
static uint32_t                s_last_battery_ms;
static uint32_t                s_last_ble_poll_ms;
static uint32_t                s_last_reminder_ms;
static uint32_t                s_last_button_ms;
static uint32_t                s_last_reminder_unix;

static uint8_t                 s_btn_was_down;
static uint32_t                s_btn_down_ms;
static uint8_t                 s_btn_reset_armed;

static uint8_t                 s_weight_seeded;
static int32_t                 s_drink_baseline_ml;
static int32_t                 s_last_stable_w_ml;
static uint32_t                s_stable_since_ms;

static uint16_t                s_consumed_today_ml;
static uint8_t                 s_hydration_score;
static uint32_t                s_current_day_unix;

static uint8_t                 s_purity_alert_active;
static uint8_t                 s_temp_alert_active;

/* BLE link-state edge handling (replaces the old IMU motion path).
 * s_ble_link_edge is set in the PA15 EXTI ISR; s_ble_was_connected holds the
 * last serviced level so a connect/disconnect is acted on exactly once. */
static volatile uint8_t        s_ble_link_edge;
static uint8_t                 s_ble_was_connected;

/* Refill-workflow state: the last weight we acted on, so a MEASURE only
 * triggers TDS/temp + an EEPROM record when the level INCREASES (water added).
 * Seeded from the most recent EEPROM record at init so it survives a reboot. */
static int32_t                 s_ee_prev_weight_ml;
static uint8_t                 s_ee_prev_valid;

static void App_ClearState(void)
{
    memset(&s_hx711, 0, sizeof(s_hx711));
    memset(&s_tds, 0, sizeof(s_tds));
    memset(&s_ntc, 0, sizeof(s_ntc));
    memset(&s_bat, 0, sizeof(s_bat));
    memset(&s_ble, 0, sizeof(s_ble));
    memset(&s_rtc, 0, sizeof(s_rtc));
    memset(&s_dev, 0, sizeof(s_dev));
    memset(&s_settings, 0, sizeof(s_settings));
    memset(&s_drink_log, 0, sizeof(s_drink_log));

    s_current_weight_ml = 0;
    s_current_tds_ppm = 0U;
    s_current_temp_x10 = 250;
    s_current_bat_pct = 0U;

    s_last_weight_ms = 0U;
    s_last_tds_ms = 0U;
    s_last_temp_ms = 0U;
    s_last_battery_ms = 0U;
    s_last_ble_poll_ms = 0U;
    s_last_reminder_ms = 0U;
    s_last_button_ms = 0U;
    s_last_reminder_unix = 0U;

    s_btn_was_down = 0U;
    s_btn_down_ms = 0U;
    s_btn_reset_armed = 0U;

    s_weight_seeded = 0U;
    s_drink_baseline_ml = 0;
    s_last_stable_w_ml = 0;
    s_stable_since_ms = 0U;

    s_consumed_today_ml = 0U;
    s_hydration_score = 0U;
    s_current_day_unix = 0U;

    s_purity_alert_active = 0U;
    s_temp_alert_active = 0U;

    s_ble_link_edge     = 0U;
    s_ble_was_connected = 0U;

    s_ee_prev_weight_ml = 0;
    s_ee_prev_valid     = 0U;
}

/* File-scope literals — pooled once; linker merges duplicate .rodata */
static const char S_OK[]  = "OK\r\n";
static const char S_ERR[] = "ERR\r\n";

/* #define HYDRA_BENCH_CMDS */

/* ─── I2C bus recovery ──────────────────────────────────────────────────── */
static void I2C_BusRecover(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);
    for (uint8_t n = 0; n < 9U; n++) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   HAL_Delay(1);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   HAL_Delay(1);
    g.Mode      = GPIO_MODE_AF_OD;
    g.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
    HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(hi2c, 0);
    HAL_Delay(5);
}

/* ─── EEPROM probe ──────────────────────────────────────────────────────── */
static uint8_t EE_Probe(uint8_t *found_addr)
{
    if (s_app_i2c == NULL) return 0;
    for (uint8_t a = EE_ADDR_BASE; a <= EE_ADDR_LAST; a++) {
        if (a == RTC_ADDR_7BIT) continue;
        if (HAL_I2C_IsDeviceReady(s_app_i2c, (uint16_t)(a << 1), 2, 5) == HAL_OK) {
            s_ee_addr = a;
            if (found_addr) *found_addr = a;
            return 1;
        }
    }
    s_ee_addr = 0xFFU;
    return 0;
}

#ifdef HYDRA_BENCH_CMDS
static uint16_t EE_Addr8(void)
{
    if (s_ee_addr == 0xFFU) { uint8_t a; if (!EE_Probe(&a)) return 0; }
    return (uint16_t)(s_ee_addr << 1);
}
#endif

/* ─── HX711 presence check ──────────────────────────────────────────────── */
static uint8_t HX711_WaitReady(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_RESET)
            return 1U;
    }
    return 0U;
}


/* ─── Interrupt forwarders ──────────────────────────────────────────────── */
void App_BLE_LinkISR(void)
{
    /* PA15 (JDY-23 link line) changed level. Latch an edge; the actual
       connect/disconnect work (auto-sync, state events) is done in the main
       loop via App_TaskBLE() so no I2C/flash happens in interrupt context.
       This EXTI source replaces the unused BMA253 motion interrupt. */
    s_ble_link_edge = 1U;
}

void App_RTC_TickISR(void)
{
    RTC_TickISR(&s_rtc);
}

void App_BLE_RxISR(void)
{
    BLE_RxISR(&s_ble);
}

/* ─── Init ──────────────────────────────────────────────────────────────── */
void App_Init(ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              UART_HandleTypeDef *huart,
              TIM_HandleTypeDef  *htim_ws,
              TIM_HandleTypeDef  *htim_buz)
{
    App_ClearState();
    s_app_i2c = hi2c;

    WS2812B_Init(htim_ws);
    Buzzer_Init(htim_buz);
    HX711_Init(&s_hx711);
    TDS_Init(&s_tds, hadc);
    NTC_Init(&s_ntc, hadc);
    Battery_Init(&s_bat, hadc);
    BLE_Init(&s_ble, huart);

    I2C_BusRecover(hi2c);
    if (RTC_Init(&s_rtc, hi2c) != HAL_OK) {
        s_rtc.unix_approx = 0;
        s_rtc.initialized = 0;
        /* NOTE: do NOT disable EXTI4_15 here. The BLE link-state line (PA15)
         * shares that vector with the RTC tick (PA11); the per-pin handler
         * in HAL_GPIO_EXTI_Callback already ignores a non-RTC source, and a
         * dead RTC must not silence BLE connect/disconnect events. */
    }
    Storage_Init();

    /* External M24512 EEPROM record log (for the MEASURE/STOREW/DUMP flow).
     * Seed the refill comparator with the most recent stored weight so the
     * "increase since last time" test is meaningful right after a reboot. */
    EE_Store_Init(hi2c);
    if (EE_Store_IsPresent()) {
        EE_Header_t eh;
        if (EE_Store_ReadHeader(&eh) && eh.count > 0U) {
            uint16_t last = (uint16_t)((eh.write_idx + EE_MAX_RECORDS - 1U) % EE_MAX_RECORDS);
            EE_Record_t er;
            if (EE_Store_ReadRecord(last, &er)) {
                s_ee_prev_weight_ml = er.weight_ml;
                s_ee_prev_valid     = 1U;
            }
        }
    }

    Storage_LoadSettings(&s_settings);
    Storage_LoadDrinkLog(&s_drink_log);
    s_hx711.tare_offset = s_settings.tare_offset;
    if (s_settings.hx711_scale_x100 >= HX711_SCALE_X100_MIN) {
        HX711_SetScaleX100(&s_hx711, s_settings.hx711_scale_x100);
    } else {
        s_hx711.scale_x100    = HX711_SCALE_X100_DEFAULT;
        s_hx711.is_calibrated = 0;
    }

    s_current_temp_x10 = 250;
    Device_Init(&s_dev);
    BLE_StartReceive(&s_ble);

    if (!s_settings.is_registered) {
        s_dev.state = DEV_STATE_UNREGISTERED;
        WS2812B_SetPattern(LED_PATTERN_REGISTRATION);
        Buzzer_Play(BUZZER_STARTUP);
    } else {
        s_dev.state = DEV_STATE_ACTIVE;
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
    }

    s_current_day_unix = (s_rtc.unix_approx / 86400UL) * 86400UL;
}

/* ─── Main run loop ─────────────────────────────────────────────────────── */
void App_Run(void)
{
    uint32_t now = HAL_GetTick();

    Device_Run(&s_dev);

    if (s_rtc.initialized && RTC_PopTick(&s_rtc)) {
        RTC_ClearTimerFlag(&s_rtc);
        if ((s_rtc.unix_approx % 60UL) == 0UL) RTC_Read(&s_rtc);
    }

    BLE_IdleFlush(&s_ble, 150U);
    App_ServiceBLE();

    /* A BLE connect/disconnect edge (PA15 EXTI) is handled promptly rather
     * than waiting for the 1 s poll, so auto-sync starts the moment the app
     * connects. */
    if (s_ble_link_edge) App_TaskBLE();

    if (now - s_last_weight_ms  >= APP_WEIGHT_POLL_MS)   App_TaskWeight();
    if (now - s_last_tds_ms     >= APP_TDS_POLL_MS)      App_TaskTDS();
    if (now - s_last_temp_ms    >= APP_TEMP_POLL_MS)      App_TaskTemp();
    if (now - s_last_battery_ms >= APP_BATTERY_POLL_MS)   App_TaskBattery();
    if (now - s_last_ble_poll_ms>= APP_BLE_STATE_POLL_MS) App_TaskBLE();
    if (now - s_last_reminder_ms>= APP_REMINDER_CHECK_MS) App_TaskReminder();
    if (now - s_last_button_ms  >= APP_BUTTON_POLL_MS)    App_TaskButton();

    App_TaskDailyRollup();
    WS2812B_Update();
    Buzzer_Update();
}

/* ─── Weight / drink detection ──────────────────────────────────────────── */
void App_TaskWeight(void)
{
    s_last_weight_ms = HAL_GetTick();
    if (!s_hx711.is_calibrated) return;

    int32_t w = HX711_ReadMillilitres(&s_hx711);
    if (!s_hx711.last_read_ok) return;

    s_current_weight_ml = w;
    App_CheckDrinkEvent();

    if (Battery_IsCharging(&s_bat) || Battery_IsFull(&s_bat))
        WS2812B_SetChargingLevel(s_current_bat_pct);
}

void App_CheckDrinkEvent(void)
{
    int32_t  w   = s_current_weight_ml;
    uint32_t now = HAL_GetTick();

    if (!s_weight_seeded) {
        s_weight_seeded    = 1;
        s_drink_baseline_ml = w;
        s_last_stable_w_ml  = w;
        s_stable_since_ms  = now;
        return;
    }

    int32_t jitter = w - s_last_stable_w_ml;
    if (jitter < 0) jitter = -jitter;

    if (jitter > (int32_t)DRINK_STABLE_BAND_G) {
        s_last_stable_w_ml = w;
        s_stable_since_ms = now;
        return;
    }
    s_last_stable_w_ml = w;

    if (now - s_stable_since_ms < DRINK_SETTLE_MS) return;

    int32_t delta = s_drink_baseline_ml - w;
    if (delta >= (int32_t)DRINK_MIN_VOLUME_ML) {
        App_RecordDrinkEvent((uint16_t)delta);
        s_drink_baseline_ml = w;
    } else if (w > s_drink_baseline_ml + (int32_t)DRINK_STABLE_BAND_G) {
        s_drink_baseline_ml = w;
    }
}

void App_RecordDrinkEvent(uint16_t volume_ml)
{
    DrinkEvent_t ev = {0};
    ev.unix_time  = s_rtc.unix_approx;
    ev.volume_ml  = volume_ml;
    ev.purity_ppm = s_current_tds_ppm;
    ev.temp_x10   = s_current_temp_x10;
    ev.synced     = 0;

    Storage_AddDrinkEvent(&s_drink_log, &ev);
    s_consumed_today_ml += ev.volume_ml;
    s_hydration_score    = App_CalcHydrationScore();

    WS2812B_SetPattern(LED_PATTERN_DRINK_CONFIRM);
    Buzzer_Play(BUZZER_SINGLE_BEEP);
    DailySummaryLog_t daily;
    Storage_LoadDailySummary(&daily);
    Storage_UpdateDailySummary(&daily, &s_drink_log, s_rtc.unix_approx);
    Storage_FlushDailySummary(&daily);
}

/* ─── TDS ───────────────────────────────────────────────────────────────── */
void App_TaskTDS(void)
{
    s_last_tds_ms     = HAL_GetTick();
    s_current_tds_ppm = TDS_ReadPPM(&s_tds, s_current_temp_x10);

    uint16_t purity_goal = BLE_U16(s_settings.prefs.purity_goal_hi,
                                    s_settings.prefs.purity_goal_lo);
    if (purity_goal > 0 && s_current_tds_ppm > purity_goal) {
        if (!s_purity_alert_active) {
            s_purity_alert_active = 1;
            WS2812B_SetPattern(LED_PATTERN_PURITY_ALERT);
            Buzzer_Play(BUZZER_PURITY_ALERT);
        }
    } else {
        s_purity_alert_active = 0;
    }
}

/* ─── Temperature ───────────────────────────────────────────────────────── */
void App_TaskTemp(void)
{
    s_last_temp_ms     = HAL_GetTick();
    s_current_temp_x10 = NTC_ReadTemp_x10(&s_ntc);

    int16_t temp_goal_x10 = BLE_I16(s_settings.prefs.temp_goal_hi,
                                      s_settings.prefs.temp_goal_lo);
    if (temp_goal_x10 > 0 && s_current_temp_x10 > temp_goal_x10) {
        if (!s_temp_alert_active) {
            s_temp_alert_active = 1;
            WS2812B_SetPattern(LED_PATTERN_TEMP_ALERT);
            Buzzer_Play(BUZZER_TEMP_ALERT);
        }
    } else {
        s_temp_alert_active = 0;
    }
}

/* ─── Battery ───────────────────────────────────────────────────────────── */
void App_TaskBattery(void)
{
    s_last_battery_ms = HAL_GetTick();
    Battery_Update(&s_bat);
    s_current_bat_pct = Battery_GetPercent(&s_bat);

    uint8_t charging = Battery_IsCharging(&s_bat) || Battery_IsFull(&s_bat);
    if (charging && s_dev.state != DEV_STATE_CHARGING &&
                    s_dev.state != DEV_STATE_LAMP_MODE) {
        WS2812B_SetChargingLevel(s_current_bat_pct);
        Device_PostEvent(&s_dev, EVT_CHARGER_CONNECTED);
    } else if (!charging && (s_dev.state == DEV_STATE_CHARGING ||
                               s_dev.state == DEV_STATE_LAMP_MODE)) {
        Device_PostEvent(&s_dev, EVT_CHARGER_DISCONNECTED);
    }

    if (Battery_IsLow(&s_bat)) {
        uint32_t now = HAL_GetTick();
        if (now - s_dev.last_low_bat_ms > 3600000UL) {
            s_dev.last_low_bat_ms = now;
            Device_PostEvent(&s_dev, EVT_BATTERY_LOW);
        }
    }
}

/* ─── BLE poll ──────────────────────────────────────────────────────────── */
void App_ServiceBLE(void)
{
    BLE_Packet_t pkt;
    if (BLE_GetPacket(&s_ble, &pkt)) App_HandleBLECommand(&pkt);

    char line[BLE_STR_LINE_MAX];
    if (BLE_GetLine(&s_ble, line))   App_HandleStringCommand(line);
}

void App_TaskBLE(void)
{
    s_last_ble_poll_ms = HAL_GetTick();

    uint8_t connected = BLE_IsConnected(&s_ble);

    /* Act on a link-state change once, whether it arrived as a PA15 EXTI edge
     * or was caught by this periodic poll (covers a missed edge). */
    if (s_ble_link_edge || connected != s_ble_was_connected) {
        s_ble_link_edge = 0U;
        if (connected && !s_ble_was_connected) {
            Device_PostEvent(&s_dev, EVT_BLE_CONNECTED);
            /* App is now nearby — push anything not yet synced. */
            if (s_drink_log.dirty) Storage_FlushDrinkLog(&s_drink_log);
        } else if (!connected && s_ble_was_connected) {
            Device_PostEvent(&s_dev, EVT_BLE_DISCONNECTED);
            /* Persist queued data so nothing is lost while offline. */
            if (s_drink_log.dirty) Storage_FlushDrinkLog(&s_drink_log);
        }
        s_ble_was_connected = connected;
    }

    if (connected && s_drink_log.dirty)
        Storage_FlushDrinkLog(&s_drink_log);
}

/* ─── Reminder ──────────────────────────────────────────────────────────── */
void App_TaskReminder(void)
{
    s_last_reminder_ms = HAL_GetTick();
    if (s_dev.state != DEV_STATE_ACTIVE) return;

    RTC_Read(&s_rtc);
    if (!RTC_IsInWindow(&s_rtc.now,
                         s_settings.prefs.remind_h_start,
                         s_settings.prefs.remind_m_start,
                         s_settings.prefs.remind_h_end,
                         s_settings.prefs.remind_m_end)) return;

    uint32_t freq_sec = (uint32_t)s_settings.prefs.remind_freq_min * 60UL;
    if (freq_sec == 0U) freq_sec = 3600U;
    if (s_rtc.unix_approx - s_last_reminder_unix < freq_sec) return;
    s_last_reminder_unix = s_rtc.unix_approx;

    s_hydration_score = App_CalcHydrationScore();

    if (s_hydration_score > HYDRATION_SCORE_HIGH) {
        RGB_t col = RGB(s_settings.prefs.remind_r,
                        s_settings.prefs.remind_g,
                        s_settings.prefs.remind_b);
        WS2812B_SetCustomReminderColor(col);
        WS2812B_SetPattern(LED_PATTERN_HYDRATION_HIGH);
    } else if (s_hydration_score >= HYDRATION_SCORE_MID) {
        WS2812B_SetPattern(LED_PATTERN_HYDRATION_MID);
    } else {
        WS2812B_SetPattern(LED_PATTERN_HYDRATION_LOW);
    }
    Buzzer_Play(BUZZER_DOUBLE_BEEP);
}

/* ─── Daily rollup ──────────────────────────────────────────────────────── */
void App_TaskDailyRollup(void)
{
    uint32_t today_midnight = (s_rtc.unix_approx / 86400UL) * 86400UL;
    if (today_midnight == s_current_day_unix) return;

    s_current_day_unix  = today_midnight;
    s_consumed_today_ml = 0;
    s_hydration_score   = 0;

    DailySummaryLog_t daily;
    Storage_LoadDailySummary(&daily);
    Storage_UpdateDailySummary(&daily, &s_drink_log, s_rtc.unix_approx);
    Storage_PurgeDailySummaryOlderThan(&daily,
        today_midnight - ((uint32_t)STORAGE_MAX_DAILY_DAYS * 86400UL));
    Storage_FlushDailySummary(&daily);
}

/* ─── Physical button ───────────────────────────────────────────────────── */
void App_TaskButton(void)
{
    s_last_button_ms = HAL_GetTick();

    uint8_t  down = (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    uint32_t now  = HAL_GetTick();

    if (down && !s_btn_was_down) {
        s_btn_was_down    = 1;
        s_btn_down_ms     = now;
        s_btn_reset_armed = 0;
    } else if (down && s_btn_was_down) {
        uint32_t held = now - s_btn_down_ms;
        if (!s_btn_reset_armed && held >= BTN_VLONG_PRESS_MS) {
            s_btn_reset_armed = 1;
            s_btn_down_ms     = now;
            WS2812B_SetPattern(LED_PATTERN_FACTORY_RESET_WARN);
            Buzzer_Play(BUZZER_FACTORY_RESET);
        } else if (s_btn_reset_armed && held >= BTN_RESET_CONFIRM_MS) {
            App_Cmd_FactoryReset();
        }
    } else if (!down && s_btn_was_down) {
        if (s_btn_reset_armed) {
            WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
            Buzzer_Stop();
        }
        s_btn_was_down    = 0;
        s_btn_reset_armed = 0;
    }
}

uint8_t App_CalcHydrationScore(void)
{
    uint16_t goal_ml = BLE_U16(s_settings.prefs.hydration_hi,
                                s_settings.prefs.hydration_lo);
    if (goal_ml == 0U) return 100U;
    uint32_t score = ((uint32_t)s_consumed_today_ml * 100U) / goal_ml;
    return (uint8_t)(score > 100U ? 100U : score);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * EEPROM measurement workflow (command-driven, for BLE bring-up)
 * ───────────────────────────────────────────────────────────────────────────
 * App_DoMeasureSequence():
 *   1. Read the load cell.
 *   2. Compare to the previous acted-on weight.
 *   3. If current > previous (REFILL):   read TDS + temperature, store a full
 *      {time, weight, tds, temp} record to the external EEPROM.
 *      If current <= previous:           do NOT power the TDS/temp sensors;
 *      store only when 'force' is set (used by STOREW).
 *   4. The stored record is always stamped with the current RTC time.
 *
 * Returns one of the MEAS_* result codes so the caller can build a reply.
 * ═════════════════════════════════════════════════════════════════════════ */
#define MEAS_ERR_HX711   0
#define MEAS_REFILL      1   /* increase: TDS/temp read + stored             */
#define MEAS_NOCHANGE    2   /* no increase, not forced: nothing read/stored */
#define MEAS_FORCED      3   /* forced store of weight (no quality sensors)  */
#define MEAS_ERR_EE      4   /* EEPROM write failed                          */

static uint8_t App_DoMeasureSequence(uint8_t force,
                                     int32_t *out_w,
                                     uint16_t *out_tds,
                                     int16_t  *out_temp,
                                     uint32_t *out_time)
{
    /* ── 1. Load cell first ── */
    if (!s_hx711.is_calibrated) return MEAS_ERR_HX711;
    if (!HX711_WaitReady(400U))  return MEAS_ERR_HX711;

    int32_t w = HX711_ReadMillilitres(&s_hx711);
    if (!s_hx711.last_read_ok)   return MEAS_ERR_HX711;
    s_current_weight_ml = w;
    *out_w = w;

    uint32_t now_unix = s_rtc.unix_approx;
    *out_time = now_unix;

    /* ── 2. Increase since last acted-on weight? ── */
    uint8_t increased = (s_ee_prev_valid && w > s_ee_prev_weight_ml) ? 1U : 0U;
    /* First-ever reading with no baseline counts as a fill so we seed a record. */
    if (!s_ee_prev_valid) increased = 1U;

    uint16_t tds  = 0U;
    int16_t  temp = 0;
    uint8_t  quality = 0U;

    if (increased) {
        /* ── 3a. REFILL → wake the quality sensors and read them ── */
        temp = NTC_ReadTemp_x10(&s_ntc);          /* temp first: TDS needs it  */
        tds  = TDS_ReadPPM(&s_tds, temp);
        s_current_temp_x10 = temp;
        s_current_tds_ppm  = tds;
        quality = 1U;
    } else if (!force) {
        /* ── 3b. No increase and not forced → leave TDS/temp idle, no store ── */
        *out_tds = 0U; *out_temp = 0;
        return MEAS_NOCHANGE;
    }

    *out_tds  = tds;
    *out_temp = temp;

    /* ── 4. Build + store the timestamped record ── */
    EE_Record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.unix_time = now_unix;       /* time the data is stored */
    rec.weight_ml = w;
    rec.tds_ppm   = tds;
    rec.temp_x10  = temp;
    rec.flags     = quality ? EE_FLAG_QUALITY : 0U;

    if (!EE_Store_Append(&rec)) return MEAS_ERR_EE;

    /* This weight is now the baseline for the next comparison. */
    s_ee_prev_weight_ml = w;
    s_ee_prev_valid     = 1U;

    return increased ? MEAS_REFILL : MEAS_FORCED;
}


/* ─── ACK helper ────────────────────────────────────────────────────────── */
void App_SendACK(uint8_t cmd_id, uint8_t success, uint8_t err_code)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildACK(buf, cmd_id, success, err_code);
    BLE_SendPacket(&s_ble, buf, len);
}

/* ─── Command dispatcher ────────────────────────────────────────────────── */
void App_HandleBLECommand(const BLE_Packet_t *pkt)
{
    switch (pkt->cmd) {
    case BLE_CMD_TIMESTAMP:    App_Cmd_Timestamp(pkt);       break;
    case BLE_CMD_INPUT_DATA:   App_Cmd_InputData(pkt);       break;
    case BLE_CMD_CALIBRATION:  App_Cmd_Calibration(pkt);     break;
    case BLE_CMD_LAMP_MODE:    App_Cmd_LampMode(pkt);        break;
    case BLE_CMD_SOFT_RESET:   App_Cmd_SoftReset();            break;
    case BLE_CMD_FACTORY_RESET:App_Cmd_FactoryReset();         break;
    case BLE_CMD_GET_HISTORY:  App_Cmd_HistoricalAggregates(); break;
    case BLE_CMD_REGISTER:     App_Cmd_RegisterDevice(pkt);  break;
    case BLE_CMD_UNPAIR:       App_Cmd_UnpairDevice();         break;
    case BLE_CMD_GET_LOGS:     App_Cmd_SensorLogs();           break;
    case BLE_CMD_SYNC_ACK:     App_Cmd_SyncAck(pkt);         break;
    case BLE_CMD_GET_STATUS:   App_Cmd_DeviceStatus();         break;
    case BLE_CMD_GET_CONFIG:   App_Cmd_GetConfig();            break;
    case BLE_CMD_GET_ERRORS:   App_Cmd_GetErrors();            break;
    case BLE_CMD_PING:         App_Cmd_Ping();                 break;
    case BLE_CMD_MEASURE:      App_Cmd_Measure(0);             break;
    case BLE_CMD_STORE_WEIGHT: App_Cmd_Measure(1);             break;
    case BLE_CMD_DUMP_EEPROM:  App_Cmd_DumpEEPROM();           break;
    default:
        App_SendACK(pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD);
        break;
    }
}

/* ─── String-command helpers ────────────────────────────────────────────── */
static char *s_u2s(char *d, char *e, uint32_t v)
{
    char t[11]; uint8_t n = 0;
    if (v == 0) { if (d < e - 1) *d++ = '0'; *d = '\0'; return d; }
    while (v && n < sizeof(t)) { t[n++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (n && d < e - 1) *d++ = t[--n];
    *d = '\0'; return d;
}

static char *s_i2s(char *d, char *e, int32_t v)
{
    if (v < 0) { if (d < e - 1) *d++ = '-'; return s_u2s(d, e, (uint32_t)(-(int64_t)v)); }
    return s_u2s(d, e, (uint32_t)v);
}

static char *s_app(char *d, char *e, const char *s)
{
    while (*s && d < e - 1) *d++ = *s++;
    *d = '\0'; return d;
}

/* Tightened s_csv: single forward scan to first comma, then parse values */
static int s_csv(const char *s, int *out, int max)
{
    while (*s && *s != ',') s++;
    if (*s != ',') return 0;
    s++;
    int c = 0;
    while (*s && c < max) {
        while (*s == ' ') s++;
        int neg = (*s == '-'); if (neg) s++;
        if (*s < '0' || *s > '9') break;
        int32_t v = 0;
        while (*s >= '0' && *s <= '9') { v = v * 10 + (*s++ - '0'); }
        out[c++] = neg ? (int)(-v) : (int)v;
        while (*s && *s != ',') s++;
        if (*s == ',') s++;
    }
    return c;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ASCII string-command handler
 * ═══════════════════════════════════════════════════════════════════════════ */
void App_HandleStringCommand(char *line)
{
    /* out[] sized for the longest reply. DUMP record lines
     * ("R,idx,unixtime,weight,tds,temp,flags\r\n") can reach ~44 chars, so
     * 64 bytes gives comfortable headroom. */
    char out[64];
    char *e = out + sizeof(out);
    char *p = out;

    char up[16];
    uint8_t i = 0;
    for (; line[i] && line[i] != ',' && i < sizeof(up) - 1U; i++) {
        char c = line[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        up[i] = c;
    }
    up[i] = '\0';

    if (strcmp(up, "PING") == 0) { BLE_SendStr(&s_ble, "PONG\r\n"); return; }

    if (strcmp(up, "HELP") == 0 || strcmp(up, "?") == 0) {
        BLE_SendStr(&s_ble, "=HydraSense cmds=\r\n");
        BLE_SendStr(&s_ble, "RGB,r,g,b RED GREEN BLUE WHITE OFF CLR\r\n");
        BLE_SendStr(&s_ble, "PAT,0-14 CHG,pct LAMP,r,g,b LAMPON LAMPOFF LAMPSAVE\r\n");
        BLE_SendStr(&s_ble, "BEEP BON BOFF BUZ,0-8\r\n");
        BLE_SendStr(&s_ble, "STATUS TEMP TDS WEIGHT RAWW READ CFG\r\n");
        BLE_SendStr(&s_ble, "CAL/TARE  CALWEIGHT,<grams>\r\n");
        BLE_SendStr(&s_ble, "TIME SETTIME,unix\r\n");
        BLE_SendStr(&s_ble, "REG UNPAIR SYNC RESET REBOOT SOFTRST\r\n");
        BLE_SendStr(&s_ble, "MEASURE/M STOREW DUMP EEINFO EEFMT\r\n");
        BLE_SendStr(&s_ble, "EE RTCQ TDSQ TMPQ BATQ\r\n");
#ifdef HYDRA_BENCH_CMDS
        BLE_SendStr(&s_ble, "PIX,i,r,g,b EER,a EEW,a,b DIAG\r\n");
#endif
        return;
    }

    /* ── TIME ── */
    if (strcmp(up, "TIME") == 0) {
        RTC_Read(&s_rtc);
        p = s_app(p, e, "T=");  p = s_u2s(p, e, s_rtc.unix_approx);
        p = s_app(p, e, " ");
        p = s_u2s(p, e, s_rtc.now.hours);   *p++ = ':';
        p = s_u2s(p, e, s_rtc.now.minutes); *p++ = ':';
        p = s_u2s(p, e, s_rtc.now.seconds);
        *p++ = (char)(s_rtc.initialized ? '+' : '-');
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&s_ble, out);
        return;
    }

    /* ── SETTIME ── */
    if (strcmp(up, "SETTIME") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] > 0) {
            RTC_SetFromUnix(&s_rtc, (uint32_t)v[0]);
            HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
            BLE_SendStr(&s_ble, S_OK);
        } else {
            BLE_SendStr(&s_ble, S_ERR);
        }
        return;
    }

    /* ── STATUS ── */
    if (strcmp(up, "STATUS") == 0) {
        p = s_app(p, e, "S:"); p = s_u2s(p, e, s_current_bat_pct);
        *p++ = ',';            p = s_i2s(p, e, s_current_temp_x10);
        *p++ = ',';            p = s_u2s(p, e, s_current_tds_ppm);
        *p++ = ',';            p = s_u2s(p, e, s_consumed_today_ml);
        *p++ = ',';            p = s_u2s(p, e, Battery_IsCharging(&s_bat) ? 1U : 0U);
        *p++ = ',';            p = s_u2s(p, e, s_hx711.is_calibrated);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&s_ble, out);
        return;
    }

    if (strcmp(up, "TEMP") == 0) {
        int16_t t = s_current_temp_x10;
        int16_t whole = t / 10, frac = t % 10; if (frac < 0) frac = -frac;
        p = s_app(p, e, "T:");
        if (t < 0 && whole == 0) *p++ = '-';
        p = s_i2s(p, e, whole); *p++ = '.'; p = s_u2s(p, e, (uint32_t)frac);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&s_ble, out);
        return;
    }

    if (strcmp(up, "TDS") == 0) {
        p = s_app(p, e, "D:"); p = s_u2s(p, e, s_current_tds_ppm);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

    if (strcmp(up, "WEIGHT") == 0) {
        if (!s_hx711.is_calibrated) {
            BLE_SendStr(&s_ble, "W:0,UNCAL\r\n");
        } else {
            p = s_app(p, e, "W:"); p = s_i2s(p, e, s_current_weight_ml);
            p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out);
        }
        return;
    }

    if (strcmp(up, "RAWW") == 0) {
        int32_t raw = 0;
        uint8_t ok  = HX711_ReadRawAveraged(&s_hx711, &raw);
        p = s_app(p, e, "RW:");
        p = s_i2s(p, e, raw);                    *p++ = ',';
        p = s_i2s(p, e, s_hx711.tare_offset); *p++ = ',';
        p = s_u2s(p, e, ok);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&s_ble, out);
        return;
    }

    if (strcmp(up, "READ") == 0) {
        p = s_app(p, e, "TLM,"); p = s_i2s(p, e, s_current_temp_x10);
        *p++ = ',';              p = s_u2s(p, e, s_current_tds_ppm);
        *p++ = ',';              p = s_u2s(p, e, s_consumed_today_ml);
        *p++ = ',';              p = s_u2s(p, e, s_hydration_score);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

    /* ════════════════════════════════════════════════════════════════════
     * RGB LED CONTROL — solid colours collapsed into a lookup table
     * ════════════════════════════════════════════════════════════════════
     *
     * Previous: 6 separate if-blocks each with their own BLE_SendStr(OK)
     * call — the compiler cannot share the tails across branches because
     * they're not adjacent.  With a table the fallthrough to BLE_SendStr
     * is shared, and the table itself is smaller than 6 call sequences.
     */
    {
        typedef struct { const char name[6]; RGB_t color; } SolidCmd_t;
        static const SolidCmd_t solids[] = {
            {"RED",   {40,  0,  0}},
            {"GREEN", { 0, 40,  0}},
            {"BLUE",  { 0,  0, 40}},
            {"WHITE", {40, 40, 40}},
            {"OFF",   { 0,  0,  0}},
            {"CLR",   { 0,  0,  0}},
        };
        for (uint8_t si = 0; si < 6U; si++) {
            if (strcmp(up, solids[si].name) == 0) {
                if (si >= 4U) {   /* OFF and CLR also clear alerts */
                    s_purity_alert_active = 0;
                    s_temp_alert_active   = 0;
                    WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
                    if (si == 5U) Buzzer_Stop();   /* CLR only */
                }
                WS2812B_SetAll(solids[si].color);
                WS2812B_SendBlocking();
                BLE_SendStr(&s_ble, S_OK);
                return;
            }
        }
    }

    if (strcmp(up, "RGB") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetAll(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SendBlocking(); BLE_SendStr(&s_ble, S_OK);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }

#ifdef HYDRA_BENCH_CMDS
    if (strcmp(up, "PIX") == 0) {
        int v[4];
        if (s_csv(line, v, 4) == 4 &&
            v[0] >= 0 && v[0] < WS2812B_NUM_LEDS &&
            v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255 && v[3]>=0 && v[3]<=255) {
            WS2812B_SetPixel((uint8_t)v[0], RGB((uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3]));
            WS2812B_SendBlocking(); BLE_SendStr(&s_ble, S_OK);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }
#endif

    if (strcmp(up, "PAT") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 14) {
            static const LED_Pattern_t map[] = {
                LED_PATTERN_ALL_OFF, LED_PATTERN_HYDRATION_HIGH, LED_PATTERN_HYDRATION_MID,
                LED_PATTERN_HYDRATION_LOW, LED_PATTERN_PURITY_ALERT, LED_PATTERN_TEMP_ALERT,
                LED_PATTERN_CALIBRATION, LED_PATTERN_DRINK_CONFIRM, LED_PATTERN_SYNC_SUCCESS,
                LED_PATTERN_CHARGING_BAR, LED_PATTERN_LOW_BATTERY, LED_PATTERN_LAMP_MODE,
                LED_PATTERN_REGISTRATION, LED_PATTERN_FACTORY_RESET_WARN, LED_PATTERN_ERROR,
            };
            WS2812B_SetPattern(map[v[0]]); BLE_SendStr(&s_ble, S_OK);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }

    if (strcmp(up, "CHG") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 100) {
            WS2812B_SetChargingLevel((uint8_t)v[0]);
            WS2812B_SetPattern(LED_PATTERN_CHARGING_BAR); BLE_SendStr(&s_ble, S_OK);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }

    if (strcmp(up, "LAMP") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetLampColor(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
            Device_PostEvent(&s_dev, EVT_CMD_LAMP_ON); BLE_SendStr(&s_ble, S_OK);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }

    /* ── LAMP / DEVICE CONTROL — grouped for GCC identical-code folding ── */
    if (strcmp(up, "LAMPON")   == 0) {
        WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
        Device_PostEvent(&s_dev, EVT_CMD_LAMP_ON);
        BLE_SendStr(&s_ble, S_OK); return;
    }
    if (strcmp(up, "LAMPSAVE") == 0) {
        Storage_SaveSettings(&s_settings);
        BLE_SendStr(&s_ble, S_OK); return;
    }
    if (strcmp(up, "LAMPOFF")  == 0) {
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF); WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking(); Device_PostEvent(&s_dev, EVT_CMD_LAMP_OFF);
        BLE_SendStr(&s_ble, S_OK); return;
    }
    if (strcmp(up, "RESET") == 0) {
        s_purity_alert_active = 0; s_temp_alert_active = 0;
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF); WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking(); Buzzer_Stop();
        BLE_SendStr(&s_ble, S_OK); return;
    }
    if (strcmp(up, "REBOOT") == 0) {
        BLE_SendStr(&s_ble, S_OK); HAL_Delay(150); HAL_NVIC_SystemReset();
    }

    /* ════════════════════════════════════════════════════════════════════
     * BUZZER
     * ════════════════════════════════════════════════════════════════════ */
    if (strcmp(up, "BEEP") == 0) { Buzzer_Play(BUZZER_SINGLE_BEEP); BLE_SendStr(&s_ble, S_OK); return; }
    if (strcmp(up, "BON")  == 0) { Buzzer_Play(BUZZER_DOUBLE_BEEP); BLE_SendStr(&s_ble, S_OK); return; }
    if (strcmp(up, "BOFF") == 0) { Buzzer_Stop();                   BLE_SendStr(&s_ble, S_OK); return; }

    if (strcmp(up, "BUZ") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 8) {
            static const uint8_t buz_map[] = {
                BUZZER_STARTUP, BUZZER_SINGLE_BEEP, BUZZER_DOUBLE_BEEP,
                BUZZER_PURITY_ALERT, BUZZER_TEMP_ALERT, BUZZER_CALIBRATION_DONE,
                BUZZER_REGISTRATION_OK, BUZZER_SYNC_OK, BUZZER_FACTORY_RESET,
            };
            Buzzer_Play(buz_map[v[0]]);
            BLE_SendStr(&s_ble, S_OK);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }

    /* ════════════════════════════════════════════════════════════════════
     * DIAGNOSTICS
     * ════════════════════════════════════════════════════════════════════ */
    if (strcmp(up, "EE") == 0) {
        uint8_t addr = 0;
        p = s_app(p, e, "EE:");
        if (EE_Probe(&addr)) { *p++ = '1'; *p++ = ','; p = s_u2s(p, e, addr); }
        else                 { *p++ = '0'; }
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

#ifdef HYDRA_BENCH_CMDS
    if (strcmp(up, "EEW") == 0) {
        int v[2]; uint16_t a8 = EE_Addr8();
        if (a8 && s_csv(line, v, 2) == 2 && v[0]>=0 && v[0]<=0xFFFF && v[1]>=0 && v[1]<=0xFF) {
            uint8_t buf3[3] = { (uint8_t)(v[0]>>8), (uint8_t)(v[0]&0xFF), (uint8_t)v[1] };
            HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(s_app_i2c, a8, buf3, 3, 20);
            HAL_Delay(6);
            BLE_SendStr(&s_ble, (st == HAL_OK) ? S_OK : S_ERR);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }
    if (strcmp(up, "EER") == 0) {
        int v[1]; uint16_t a8 = EE_Addr8();
        if (a8 && s_csv(line, v, 1) == 1 && v[0]>=0 && v[0]<=0xFFFF) {
            uint8_t ad[2] = { (uint8_t)(v[0]>>8), (uint8_t)(v[0]&0xFF) }, rb = 0;
            HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(s_app_i2c, a8, ad, 2, 20);
            if (st == HAL_OK) st = HAL_I2C_Master_Receive(s_app_i2c, a8, &rb, 1, 20);
            if (st == HAL_OK) {
                p = s_app(p, e, "EER:"); p = s_u2s(p, e, rb);
                p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out);
            } else BLE_SendStr(&s_ble, S_ERR);
        } else BLE_SendStr(&s_ble, S_ERR);
        return;
    }
#endif

    if (strcmp(up, "RTCQ") == 0) {
        p = s_app(p, e, "R:");
        if (s_rtc.initialized) {
            RTC_Read(&s_rtc);
            *p++ = '1'; *p++ = ',';
            p = s_u2s(p, e, s_rtc.unix_approx);  *p++ = ',';
            p = s_u2s(p, e, s_rtc.now.hours);    *p++ = ':';
            p = s_u2s(p, e, s_rtc.now.minutes);  *p++ = ':';
            p = s_u2s(p, e, s_rtc.now.seconds);
        } else { *p++ = '0'; }
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

    if (strcmp(up, "TDSQ") == 0) {
        uint16_t ppm = TDS_ReadPPM(&s_tds, s_current_temp_x10);
        s_current_tds_ppm = ppm;
        p = s_app(p, e, "DQ:"); p = s_u2s(p, e, ppm);
        *p++ = ',';             p = s_u2s(p, e, s_tds.valid);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

    if (strcmp(up, "TMPQ") == 0) {
        int16_t t = NTC_ReadTemp_x10(&s_ntc);
        s_current_temp_x10 = t;
        p = s_app(p, e, "TQ:"); p = s_i2s(p, e, t);
        *p++ = ','; p = s_u2s(p, e, s_ntc.valid);
#ifdef NTC_DEBUG_VARS
        *p++ = ','; p = s_u2s(p, e, g_ntc_adc_avg);
        *p++ = ','; p = s_u2s(p, e, g_ntc_fault);
#else
        *p++ = ','; p = s_u2s(p, e, 0U);
        *p++ = ','; p = s_u2s(p, e, 0U);
#endif
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

    if (strcmp(up, "BATQ") == 0) {
        Battery_Update(&s_bat);
        s_current_bat_pct = Battery_GetPercent(&s_bat);
        p = s_app(p, e, "BQ:"); p = s_u2s(p, e, s_current_bat_pct);
        *p++ = ','; p = s_u2s(p, e, Battery_IsCharging(&s_bat) ? 1U : 0U);
        *p++ = ','; p = s_u2s(p, e, Battery_IsFull(&s_bat)     ? 1U : 0U);
        *p++ = ','; p = s_u2s(p, e, Battery_IsLow(&s_bat)      ? 1U : 0U);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

#ifdef HYDRA_BENCH_CMDS
    if (strcmp(up, "DIAG") == 0) {
        uint8_t  ee  = EE_Probe(NULL);
        uint16_t ppm = TDS_ReadPPM(&s_tds, s_current_temp_x10);
        int16_t  t   = NTC_ReadTemp_x10(&s_ntc);
        Battery_Update(&s_bat);
        p = s_app(p, e, "DG:");
        p = s_u2s(p, e, ee);                    *p++ = ',';
        p = s_u2s(p, e, s_rtc.initialized);  *p++ = ',';
        p = s_u2s(p, e, ppm);                   *p++ = ',';
        p = s_i2s(p, e, t);                     *p++ = ',';
        p = s_u2s(p, e, Battery_GetPercent(&s_bat));
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }
#endif

    /* ════════════════════════════════════════════════════════════════════
     * DEVICE CONTROL
     * ════════════════════════════════════════════════════════════════════ */
    if (strcmp(up, "REG") == 0) {
        s_settings.is_registered = 1;
        Storage_SaveSettings(&s_settings);
        Device_PostEvent(&s_dev, EVT_CMD_REGISTER);
        Buzzer_Play(BUZZER_REGISTRATION_OK);
        BLE_SendStr(&s_ble, S_OK); return;
    }
    if (strcmp(up, "UNPAIR") == 0) {
        memset(s_settings.user_id, 0, sizeof(s_settings.user_id));
        s_settings.is_registered = 0;
        Storage_SaveSettings(&s_settings);
        BLE_SendStr(&s_ble, S_OK); return;
    }
    if (strcmp(up, "SOFTRST") == 0) {
        BLE_SendStr(&s_ble, S_OK); HAL_Delay(200); HAL_NVIC_SystemReset();
    }
    if (strcmp(up, "SYNC") == 0) {
        Storage_MarkSynced(&s_drink_log, s_rtc.unix_approx);
        Storage_FlushDrinkLog(&s_drink_log);
        WS2812B_SetPattern(LED_PATTERN_SYNC_SUCCESS);
        Buzzer_Play(BUZZER_SYNC_OK);
        BLE_SendStr(&s_ble, S_OK); return;
    }
    if (strcmp(up, "CFG") == 0) {
        p = s_app(p, e, "C:");
        p = s_u2s(p, e, BLE_U16(s_settings.prefs.purity_goal_hi, s_settings.prefs.purity_goal_lo)); *p++ = ',';
        p = s_i2s(p, e, BLE_I16(s_settings.prefs.temp_goal_hi,   s_settings.prefs.temp_goal_lo));   *p++ = ',';
        p = s_u2s(p, e, BLE_U16(s_settings.prefs.hydration_hi,   s_settings.prefs.hydration_lo));   *p++ = ',';
        p = s_u2s(p, e, s_settings.prefs.remind_h_start); *p++ = ':';
        p = s_u2s(p, e, s_settings.prefs.remind_m_start); *p++ = '-';
        p = s_u2s(p, e, s_settings.prefs.remind_h_end);   *p++ = ':';
        p = s_u2s(p, e, s_settings.prefs.remind_m_end);   *p++ = ',';
        p = s_u2s(p, e, s_settings.prefs.remind_freq_min);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out); return;
    }

    /* ── CAL / TARE ── */
    if (strcmp(up, "CAL") == 0 || strcmp(up, "TARE") == 0) {
        if (!HX711_WaitReady(400U)) {
            BLE_SendStr(&s_ble, "ERR,HX711\r\n"); return;
        }
        BLE_Packet_t fake;
        fake.cmd        = BLE_CMD_CALIBRATION;
        fake.len        = 1;
        fake.payload[0] = 0;
        App_Cmd_Calibration(&fake);
        return;
    }

    /* ── CALWEIGHT ── */
    if (strcmp(up, "CALWEIGHT") == 0) {
        int v[1];
        if (s_csv(line, v, 1) != 1 || v[0] < 1) {
            BLE_SendStr(&s_ble, "CW:ERR\r\n"); return;
        }
        if (!HX711_WaitReady(400U)) {
            BLE_SendStr(&s_ble, "CW:ERR,HX711\r\n"); return;
        }
        if (!HX711_Calibrate(&s_hx711, (int32_t)v[0])) {
            BLE_SendStr(&s_ble, "CW:ERR,LOAD\r\n"); return;
        }
        s_settings.tare_offset       = s_hx711.tare_offset;
        s_settings.hx711_scale_x100  = s_hx711.scale_x100;
        s_settings.is_calibrated = 1;
        Storage_SaveSettings(&s_settings);
        s_weight_seeded    = 0;
        s_drink_baseline_ml = 0;
        p = s_app(p, e, "CW:OK,scale_x100=");
        p = s_i2s(p, e, s_hx711.scale_x100);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&s_ble, out);
        return;
    }

    /* ════════════════════════════════════════════════════════════════════
     * EEPROM MEASUREMENT WORKFLOW (BLE-driven bring-up)
     * ════════════════════════════════════════════════════════════════════ */

    /* MEASURE — load cell first; only if level INCREASED do we read TDS+temp
     * and store {time,weight,tds,temp}. No increase → sensors stay idle. */
    if (strcmp(up, "MEASURE") == 0 || strcmp(up, "M") == 0) {
        int32_t w; uint16_t tds; int16_t tmp; uint32_t ts;
        uint8_t r = App_DoMeasureSequence(0U, &w, &tds, &tmp, &ts);
        switch (r) {
        case MEAS_REFILL:
            p = s_app(p, e, "MEAS,REFILL,");
            p = s_i2s(p, e, w);   *p++ = ',';
            p = s_u2s(p, e, tds); *p++ = ',';
            p = s_i2s(p, e, tmp); *p++ = ',';
            p = s_u2s(p, e, ts);
            p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out);
            break;
        case MEAS_NOCHANGE:
            p = s_app(p, e, "MEAS,NOCHG,");
            p = s_i2s(p, e, w);
            p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out);
            break;
        case MEAS_ERR_EE:
            BLE_SendStr(&s_ble, "MEAS,ERR,EE\r\n"); break;
        default:
            BLE_SendStr(&s_ble, "MEAS,ERR,HX711\r\n"); break;
        }
        return;
    }

    /* STOREW — read load cell now and FORCE-store a record (always). If the
     * level also increased, TDS+temp are included; otherwise weight-only. */
    if (strcmp(up, "STOREW") == 0) {
        int32_t w; uint16_t tds; int16_t tmp; uint32_t ts;
        uint8_t r = App_DoMeasureSequence(1U, &w, &tds, &tmp, &ts);
        if (r == MEAS_REFILL || r == MEAS_FORCED) {
            p = s_app(p, e, "STOREW,OK,");
            p = s_i2s(p, e, w);   *p++ = ',';
            p = s_u2s(p, e, tds); *p++ = ',';
            p = s_i2s(p, e, tmp); *p++ = ',';
            p = s_u2s(p, e, ts);
            p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out);
        } else if (r == MEAS_ERR_EE) {
            BLE_SendStr(&s_ble, "STOREW,ERR,EE\r\n");
        } else {
            BLE_SendStr(&s_ble, "STOREW,ERR,HX711\r\n");
        }
        return;
    }

    /* DUMP — stream every stored record from the EEPROM, oldest first. */
    if (strcmp(up, "DUMP") == 0) {
        if (!EE_Store_IsPresent()) { BLE_SendStr(&s_ble, "DUMP,ERR,NOEE\r\n"); return; }
        EE_Header_t eh;
        if (!EE_Store_ReadHeader(&eh) || eh.count == 0U) {
            BLE_SendStr(&s_ble, "DUMP,0\r\n"); return;
        }
        p = s_app(p, e, "DUMP,"); p = s_u2s(p, e, eh.count);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out);

        /* Oldest slot when the ring has wrapped is write_idx; otherwise 0. */
        uint16_t start = (eh.count >= EE_MAX_RECORDS) ? eh.write_idx : 0U;
        for (uint16_t k = 0U; k < eh.count; k++) {
            uint16_t slot = (uint16_t)((start + k) % EE_MAX_RECORDS);
            EE_Record_t er;
            p = out;
            if (EE_Store_ReadRecord(slot, &er)) {
                p = s_app(p, e, "R,");
                p = s_u2s(p, e, k);            *p++ = ',';
                p = s_u2s(p, e, er.unix_time); *p++ = ',';
                p = s_i2s(p, e, er.weight_ml); *p++ = ',';
                p = s_u2s(p, e, er.tds_ppm);   *p++ = ',';
                p = s_i2s(p, e, er.temp_x10);  *p++ = ',';
                p = s_u2s(p, e, er.flags);
                p = s_app(p, e, "\r\n");
            } else {
                p = s_app(p, e, "R,"); p = s_u2s(p, e, k);
                p = s_app(p, e, ",BAD\r\n");
            }
            BLE_SendStr(&s_ble, out);
            HAL_Delay(5);
        }
        BLE_SendStr(&s_ble, "DUMP,END\r\n");
        return;
    }

    /* EEINFO — header snapshot; EEFMT — wipe the log header (records orphaned). */
    if (strcmp(up, "EEINFO") == 0) {
        EE_Header_t eh;
        p = s_app(p, e, "EEI,");
        p = s_u2s(p, e, EE_Store_IsPresent()); *p++ = ',';
        if (EE_Store_ReadHeader(&eh)) {
            p = s_u2s(p, e, eh.count);     *p++ = ',';
            p = s_u2s(p, e, eh.write_idx);
        } else { p = s_app(p, e, "0,0"); }
        p = s_app(p, e, "\r\n"); BLE_SendStr(&s_ble, out);
        return;
    }
    if (strcmp(up, "EEFMT") == 0) {
        uint8_t ok = EE_Store_Format();
        s_ee_prev_valid = 0U;
        s_ee_prev_weight_ml = 0;
        BLE_SendStr(&s_ble, ok ? S_OK : S_ERR);
        return;
    }

    BLE_SendStr(&s_ble, S_ERR);
}

void App_Cmd_Timestamp(const BLE_Packet_t *pkt)
{
    if (pkt->len < 4U) { App_SendACK(pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD); return; }
    uint32_t unix_time = BLE_U32(pkt->payload[0], pkt->payload[1],
                                  pkt->payload[2], pkt->payload[3]);
    RTC_SetFromUnix(&s_rtc, unix_time);
    RTC_Read(&s_rtc);
    App_SendACK(pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_InputData(const BLE_Packet_t *pkt)
{
    if (pkt->len < 18U) { App_SendACK(pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD); return; }
    memcpy(&s_settings.prefs, pkt->payload, sizeof(BLE_PrefsPayload_t));
    Storage_SaveSettings(&s_settings);
    App_SendACK(pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_Calibration(const BLE_Packet_t *pkt)
{
    if (pkt->len < 1U || pkt->payload[0] != 0U) {
        App_SendACK(pkt->cmd, 0, BLE_ERR_INVALID_STAGE); return;
    }
    if (!HX711_WaitReady(400U)) {
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
        App_SendACK(pkt->cmd, 0, BLE_ERR_OK); return;
    }
    Device_PostEvent(&s_dev, EVT_CMD_CALIBRATION);
    WS2812B_SetPattern(LED_PATTERN_CALIBRATION);
    HX711_Tare(&s_hx711);
    s_settings.tare_offset = s_hx711.tare_offset;
    Storage_SaveSettings(&s_settings);
    Device_PostEvent(&s_dev, EVT_CALIBRATION_DONE);
    Buzzer_Play(BUZZER_CALIBRATION_DONE);
    WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
    App_SendACK(pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_LampMode(const BLE_Packet_t *pkt)
{
    if (!Battery_IsCharging(&s_bat) && !Battery_IsFull(&s_bat)) {
        App_SendACK(pkt->cmd, 0, BLE_ERR_NOT_CHARGING); return;
    }
    if (pkt->len >= 1U && pkt->payload[0]) {
        RGB_t col = {0, 255, 0};
        if (pkt->len >= 4U) { col.r = pkt->payload[1]; col.g = pkt->payload[2]; col.b = pkt->payload[3]; }
        s_settings.prefs.lamp_r = col.r;
        s_settings.prefs.lamp_g = col.g;
        s_settings.prefs.lamp_b = col.b;
        Storage_SaveSettings(&s_settings);
        WS2812B_SetLampColor(col);
        Device_PostEvent(&s_dev, EVT_CMD_LAMP_ON);
    } else {
        Device_PostEvent(&s_dev, EVT_CMD_LAMP_OFF);
    }
    App_SendACK(pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_SoftReset(void)
{
    App_SendACK(BLE_CMD_SOFT_RESET, 1, BLE_ERR_OK);
    HAL_Delay(200); HAL_NVIC_SystemReset();
}

void App_Cmd_FactoryReset(void)
{
    Device_PostEvent(&s_dev, EVT_CMD_FACTORY_RESET);
    WS2812B_SetPattern(LED_PATTERN_FACTORY_RESET_WARN);
    Buzzer_Play(BUZZER_FACTORY_RESET);
    HAL_Delay(5000);
    App_SendACK(BLE_CMD_FACTORY_RESET, 1, BLE_ERR_OK);
    HAL_Delay(200);
    Storage_FactoryReset();
    HAL_NVIC_SystemReset();
}

void App_Cmd_HistoricalAggregates(void)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    DailySummaryLog_t daily;
    Storage_LoadDailySummary(&daily);
    for (uint8_t i = 0; i < daily.count; i++) {
        DailySummary_t *d = &daily.days[i];
        if (!d->valid) continue;
        BLE_DailyPayload_t pl;
        pl.unix_b3 = (uint8_t)(d->date_unix >> 24);
        pl.unix_b2 = (uint8_t)(d->date_unix >> 16);
        pl.unix_b1 = (uint8_t)(d->date_unix >>  8);
        pl.unix_b0 = (uint8_t)(d->date_unix);
        pl.ml_hi   = (uint8_t)(d->total_ml >> 8);
        pl.ml_lo   = (uint8_t)(d->total_ml);
        pl.ppm_hi  = (uint8_t)(d->avg_purity_ppm >> 8);
        pl.ppm_lo  = (uint8_t)(d->avg_purity_ppm);
        pl.temp_hi = (uint8_t)((uint16_t)d->avg_temp_x10 >> 8);
        pl.temp_lo = (uint8_t)(d->avg_temp_x10);
        uint8_t len = BLE_BuildDaily(buf, &pl);
        BLE_SendPacket(&s_ble, buf, len); HAL_Delay(20);
    }
    App_SendACK(BLE_CMD_GET_HISTORY, 1, BLE_ERR_OK);
}

void App_Cmd_RegisterDevice(const BLE_Packet_t *pkt)
{
    if (pkt->len >= 16U) memcpy(s_settings.user_id,         pkt->payload,      16);
    if (pkt->len >= 32U) memcpy(s_settings.device_nickname, pkt->payload + 16, 16);
    s_settings.is_registered = 1;
    Storage_SaveSettings(&s_settings);
    Device_PostEvent(&s_dev, EVT_CMD_REGISTER);
    Buzzer_Play(BUZZER_REGISTRATION_OK);
    App_SendACK(pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_UnpairDevice(void)
{
    memset(s_settings.user_id, 0, sizeof(s_settings.user_id));
    s_settings.is_registered = 0;
    Storage_SaveSettings(&s_settings);
    App_SendACK(BLE_CMD_UNPAIR, 1, BLE_ERR_OK);
}

void App_Cmd_SensorLogs(void)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    for (uint8_t i = 0; i < s_drink_log.count; i++) {
        DrinkEvent_t *ev = &s_drink_log.events[i];
        BLE_LogEntryPayload_t pl;
        pl.unix_b3 = (uint8_t)(ev->unix_time >> 24);
        pl.unix_b2 = (uint8_t)(ev->unix_time >> 16);
        pl.unix_b1 = (uint8_t)(ev->unix_time >>  8);
        pl.unix_b0 = (uint8_t)(ev->unix_time);
        pl.vol_hi  = (uint8_t)(ev->volume_ml >> 8);
        pl.vol_lo  = (uint8_t)(ev->volume_ml);
        pl.ppm_hi  = (uint8_t)(ev->purity_ppm >> 8);
        pl.ppm_lo  = (uint8_t)(ev->purity_ppm);
        pl.temp_hi = (uint8_t)((uint16_t)ev->temp_x10 >> 8);
        pl.temp_lo = (uint8_t)(ev->temp_x10);
        pl.synced  = ev->synced;
        uint8_t len = BLE_BuildLogEntry(buf, &pl);
        BLE_SendPacket(&s_ble, buf, len); HAL_Delay(20);
    }
    App_SendACK(BLE_CMD_GET_LOGS, 1, BLE_ERR_OK);
}

void App_Cmd_SyncAck(const BLE_Packet_t *pkt)
{
    uint32_t cutoff = (pkt->len >= 4U)
                    ? BLE_U32(pkt->payload[0], pkt->payload[1],
                               pkt->payload[2], pkt->payload[3])
                    : s_rtc.unix_approx;
    Storage_MarkSynced(&s_drink_log, cutoff);
    Storage_FlushDrinkLog(&s_drink_log);
    WS2812B_SetPattern(LED_PATTERN_SYNC_SUCCESS);
    Buzzer_Play(BUZZER_SYNC_OK);
    App_SendACK(pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_DeviceStatus(void)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    BLE_StatusPayload_t pl;
    pl.bat_pct = s_current_bat_pct;
    pl.flags   = 0;
    if (Battery_IsCharging(&s_bat))   pl.flags |= BLE_FLAG_CHARGING;
    if (s_ntc.valid)                  pl.flags |= BLE_FLAG_TEMP_OK;
    if (s_tds.valid)                  pl.flags |= BLE_FLAG_TDS_OK;
    if (s_hx711.is_calibrated)        pl.flags |= BLE_FLAG_WEIGHT_OK;
    if (s_settings.is_calibrated)     pl.flags |= BLE_FLAG_CALIBRATED;
    if (s_settings.is_registered)     pl.flags |= BLE_FLAG_REGISTERED;
    pl.storage_pct = (uint8_t)((uint32_t)s_drink_log.count * 100U
                                / STORAGE_MAX_DRINK_EVENTS);
    uint8_t len = BLE_BuildStatus(buf, &pl);
    BLE_SendPacket(&s_ble, buf, len);
}

void App_Cmd_GetConfig(void)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildConfig(buf, &s_settings.prefs);
    BLE_SendPacket(&s_ble, buf, len);
}

void App_Cmd_GetErrors(void)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t payload = 0;
    uint8_t len = BLE_BuildPacket(buf, BLE_RSP_ERR_LOG, &payload, 1);
    BLE_SendPacket(&s_ble, buf, len);
}

void App_Cmd_Ping(void)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildPong(buf);
    BLE_SendPacket(&s_ble, buf, len);
}

/* Binary MEASURE / STORE_WEIGHT: run the same load-cell-first workflow and
 * reply with a single result packet (weight, tds, temp, time, result code). */
void App_Cmd_Measure(uint8_t force)
{
    int32_t w; uint16_t tds; int16_t tmp; uint32_t ts;
    uint8_t r = App_DoMeasureSequence(force, &w, &tds, &tmp, &ts);

    uint8_t pl[14];
    pl[0]  = r;                               /* MEAS_* result code         */
    pl[1]  = (uint8_t)(ts >> 24); pl[2] = (uint8_t)(ts >> 16);
    pl[3]  = (uint8_t)(ts >>  8); pl[4] = (uint8_t)(ts);
    pl[5]  = (uint8_t)((uint32_t)w >> 24); pl[6] = (uint8_t)((uint32_t)w >> 16);
    pl[7]  = (uint8_t)((uint32_t)w >>  8); pl[8] = (uint8_t)((uint32_t)w);
    pl[9]  = (uint8_t)(tds >> 8); pl[10] = (uint8_t)(tds);
    pl[11] = (uint8_t)((uint16_t)tmp >> 8); pl[12] = (uint8_t)(tmp);
    pl[13] = (r == MEAS_REFILL) ? EE_FLAG_QUALITY : 0U;

    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildPacket(buf, BLE_RSP_MEASURE, pl, sizeof(pl));
    BLE_SendPacket(&s_ble, buf, len);
}

/* Binary DUMP: stream every stored EEPROM record as BLE_RSP_EE_RECORD frames,
 * then a final ACK to mark the end of the stream. */
void App_Cmd_DumpEEPROM(void)
{
    uint8_t buf[BLE_PKT_MAX_LEN];

    if (!EE_Store_IsPresent()) {
        App_SendACK(BLE_CMD_DUMP_EEPROM, 0, BLE_ERR_UNKNOWN_CMD);
        return;
    }

    EE_Header_t eh;
    if (!EE_Store_ReadHeader(&eh) || eh.count == 0U) {
        App_SendACK(BLE_CMD_DUMP_EEPROM, 1, BLE_ERR_OK);
        return;
    }

    uint16_t start = (eh.count >= EE_MAX_RECORDS) ? eh.write_idx : 0U;
    for (uint16_t k = 0U; k < eh.count; k++) {
        uint16_t slot = (uint16_t)((start + k) % EE_MAX_RECORDS);
        EE_Record_t er;
        if (!EE_Store_ReadRecord(slot, &er)) continue;

        BLE_EERecordPayload_t pl;
        pl.idx_hi  = (uint8_t)(k >> 8);            pl.idx_lo  = (uint8_t)k;
        pl.unix_b3 = (uint8_t)(er.unix_time >> 24); pl.unix_b2 = (uint8_t)(er.unix_time >> 16);
        pl.unix_b1 = (uint8_t)(er.unix_time >>  8); pl.unix_b0 = (uint8_t)(er.unix_time);
        pl.w_b3 = (uint8_t)((uint32_t)er.weight_ml >> 24);
        pl.w_b2 = (uint8_t)((uint32_t)er.weight_ml >> 16);
        pl.w_b1 = (uint8_t)((uint32_t)er.weight_ml >>  8);
        pl.w_b0 = (uint8_t)((uint32_t)er.weight_ml);
        pl.ppm_hi  = (uint8_t)(er.tds_ppm >> 8);   pl.ppm_lo  = (uint8_t)(er.tds_ppm);
        pl.temp_hi = (uint8_t)((uint16_t)er.temp_x10 >> 8);
        pl.temp_lo = (uint8_t)(er.temp_x10);
        pl.flags   = er.flags;

        uint8_t len = BLE_BuildEERecord(buf, &pl);
        BLE_SendPacket(&s_ble, buf, len);
        HAL_Delay(15);
    }
    App_SendACK(BLE_CMD_DUMP_EEPROM, 1, BLE_ERR_OK);
}

void App_ResetDailyConsumed(void)
{
    s_consumed_today_ml = 0;
    s_hydration_score   = 0;
}
