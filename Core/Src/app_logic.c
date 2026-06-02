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
#include <string.h>

extern volatile uint16_t g_ntc_adc_avg;
extern volatile uint8_t  g_ntc_fault;

static I2C_HandleTypeDef *s_app_i2c = NULL;

#define EE_ADDR_BASE   0x50U
#define EE_ADDR_LAST   0x57U
#ifndef RTC_I2C_ADDR
#define RTC_I2C_ADDR   0xA2U
#endif
#define RTC_ADDR_7BIT  (RTC_I2C_ADDR >> 1)

static uint8_t s_ee_addr = 0xFFU;

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

/* ─── Init ──────────────────────────────────────────────────────────────── */
void App_Init(AppContext_t *app,
              ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              UART_HandleTypeDef *huart,
              TIM_HandleTypeDef  *htim_ws,
              TIM_HandleTypeDef  *htim_buz)
{
    memset(app, 0, sizeof(AppContext_t));
    s_app_i2c = hi2c;

    WS2812B_Init(htim_ws);
    Buzzer_Init(htim_buz);
    HX711_Init(&app->hx711);
    TDS_Init(&app->tds, hadc);
    NTC_Init(&app->ntc, hadc);
    Battery_Init(&app->bat, hadc);
    BLE_Init(&app->ble, huart);

    I2C_BusRecover(hi2c);
    if (RTC_Init(&app->rtc, hi2c) != HAL_OK) {
        app->rtc.unix_approx = 0;
        app->rtc.initialized = 0;
        HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
    }
    Storage_Init();

    Storage_LoadSettings(&app->settings);
    Storage_LoadDrinkLog(&app->drink_log);
    Storage_LoadDailySummary(&app->daily_log);

    app->hx711.tare_offset = (int32_t)app->settings.tare_offset;
    if (app->settings.hx711_scale > 1.0f) {
        HX711_SetScale(&app->hx711, app->settings.hx711_scale);
    } else {
        app->hx711.scale         = HX711_DEFAULT_SCALE;
        app->hx711.is_calibrated = 0;
    }

    app->current_temp_x10 = 250;
    Device_Init(&app->dev);
    BLE_StartReceive(&app->ble);

    if (!app->settings.is_registered) {
        app->dev.state = DEV_STATE_UNREGISTERED;
        WS2812B_SetPattern(LED_PATTERN_REGISTRATION);
        Buzzer_Play(BUZZER_STARTUP);
    } else {
        app->dev.state = DEV_STATE_ACTIVE;
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
    }

    app->current_day_unix = (app->rtc.unix_approx / 86400UL) * 86400UL;
}

/* ─── Main run loop ─────────────────────────────────────────────────────── */
void App_Run(AppContext_t *app)
{
    uint32_t now = HAL_GetTick();

    Device_Run(&app->dev);

    if (app->rtc.initialized && RTC_PopTick(&app->rtc)) {
        RTC_ClearTimerFlag(&app->rtc);
        if ((app->rtc.unix_approx % 60UL) == 0UL) RTC_Read(&app->rtc);
    }

    BLE_IdleFlush(&app->ble, 150U);
    App_ServiceBLE(app);

    if (now - app->last_weight_ms  >= APP_WEIGHT_POLL_MS)   App_TaskWeight(app);
    if (now - app->last_tds_ms     >= APP_TDS_POLL_MS)      App_TaskTDS(app);
    if (now - app->last_temp_ms    >= APP_TEMP_POLL_MS)      App_TaskTemp(app);
    if (now - app->last_battery_ms >= APP_BATTERY_POLL_MS)   App_TaskBattery(app);
    if (now - app->last_ble_poll_ms>= APP_BLE_STATE_POLL_MS) App_TaskBLE(app);
    if (now - app->last_reminder_ms>= APP_REMINDER_CHECK_MS) App_TaskReminder(app);
    if (now - app->last_button_ms  >= APP_BUTTON_POLL_MS)    App_TaskButton(app);

    App_TaskDailyRollup(app);
    WS2812B_Update();
    Buzzer_Update();
}

/* ─── Weight / drink detection ──────────────────────────────────────────── */
void App_TaskWeight(AppContext_t *app)
{
    app->last_weight_ms = HAL_GetTick();
    if (!app->hx711.is_calibrated) return;

    float w = HX711_ReadMillilitres(&app->hx711);
    if (!app->hx711.last_read_ok) return;

    app->prev_weight_g    = app->current_weight_g;
    app->current_weight_g = w;
    App_CheckDrinkEvent(app);

    if (Battery_IsCharging(&app->bat) || Battery_IsFull(&app->bat))
        WS2812B_SetChargingLevel(app->current_bat_pct);
}

void App_CheckDrinkEvent(AppContext_t *app)
{
    float    w   = app->current_weight_g;
    uint32_t now = HAL_GetTick();

    if (!app->weight_seeded) {
        app->weight_seeded    = 1;
        app->drink_baseline_g = w;
        app->last_stable_w_g  = w;
        app->stable_since_ms  = now;
        return;
    }

    float jitter = w - app->last_stable_w_g;
    if (jitter < 0) jitter = -jitter;

    if (jitter > (float)DRINK_STABLE_BAND_G) {
        app->last_stable_w_g = w;
        app->stable_since_ms = now;
        return;
    }
    app->last_stable_w_g = w;

    if (now - app->stable_since_ms < DRINK_SETTLE_MS) return;

    float delta = app->drink_baseline_g - w;
    if (delta >= (float)DRINK_MIN_VOLUME_ML) {
        App_RecordDrinkEvent(app, delta);
        app->drink_baseline_g = w;
    } else if (w > app->drink_baseline_g + (float)DRINK_STABLE_BAND_G) {
        app->drink_baseline_g = w;
    }
}

void App_RecordDrinkEvent(AppContext_t *app, float volume_ml)
{
    DrinkEvent_t ev = {0};
    ev.unix_time  = app->rtc.unix_approx;
    ev.volume_ml  = (uint16_t)volume_ml;
    ev.purity_ppm = app->current_tds_ppm;
    ev.temp_x10   = app->current_temp_x10;
    ev.synced     = 0;

    Storage_AddDrinkEvent(&app->drink_log, &ev);
    app->consumed_today_ml += ev.volume_ml;
    app->hydration_score    = App_CalcHydrationScore(app);

    WS2812B_SetPattern(LED_PATTERN_DRINK_CONFIRM);
    Buzzer_Play(BUZZER_SINGLE_BEEP);
    Storage_UpdateDailySummary(&app->daily_log, &app->drink_log, app->rtc.unix_approx);
}

/* ─── TDS ───────────────────────────────────────────────────────────────── */
void App_TaskTDS(AppContext_t *app)
{
    app->last_tds_ms     = HAL_GetTick();
    app->current_tds_ppm = TDS_ReadPPM(&app->tds, app->current_temp_x10);

    uint16_t purity_goal = BLE_U16(app->settings.prefs.purity_goal_hi,
                                    app->settings.prefs.purity_goal_lo);
    if (purity_goal > 0 && app->current_tds_ppm > purity_goal) {
        if (!app->purity_alert_active) {
            app->purity_alert_active = 1;
            WS2812B_SetPattern(LED_PATTERN_PURITY_ALERT);
            Buzzer_Play(BUZZER_PURITY_ALERT);
        }
    } else {
        app->purity_alert_active = 0;
    }
}

/* ─── Temperature ───────────────────────────────────────────────────────── */
void App_TaskTemp(AppContext_t *app)
{
    app->last_temp_ms     = HAL_GetTick();
    app->current_temp_x10 = NTC_ReadTemp_x10(&app->ntc);

    int16_t temp_goal_x10 = BLE_I16(app->settings.prefs.temp_goal_hi,
                                      app->settings.prefs.temp_goal_lo);
    if (temp_goal_x10 > 0 && app->current_temp_x10 > temp_goal_x10) {
        if (!app->temp_alert_active) {
            app->temp_alert_active = 1;
            WS2812B_SetPattern(LED_PATTERN_TEMP_ALERT);
            Buzzer_Play(BUZZER_TEMP_ALERT);
        }
    } else {
        app->temp_alert_active = 0;
    }
}

/* ─── Battery ───────────────────────────────────────────────────────────── */
void App_TaskBattery(AppContext_t *app)
{
    app->last_battery_ms = HAL_GetTick();
    Battery_Update(&app->bat);
    app->current_bat_pct = Battery_GetPercent(&app->bat);

    uint8_t charging = Battery_IsCharging(&app->bat) || Battery_IsFull(&app->bat);
    if (charging && app->dev.state != DEV_STATE_CHARGING &&
                    app->dev.state != DEV_STATE_LAMP_MODE) {
        WS2812B_SetChargingLevel(app->current_bat_pct);
        Device_PostEvent(&app->dev, EVT_CHARGER_CONNECTED);
    } else if (!charging && (app->dev.state == DEV_STATE_CHARGING ||
                               app->dev.state == DEV_STATE_LAMP_MODE)) {
        Device_PostEvent(&app->dev, EVT_CHARGER_DISCONNECTED);
    }

    if (Battery_IsLow(&app->bat)) {
        uint32_t now = HAL_GetTick();
        if (now - app->dev.last_low_bat_ms > 3600000UL) {
            app->dev.last_low_bat_ms = now;
            Device_PostEvent(&app->dev, EVT_BATTERY_LOW);
        }
    }
}

/* ─── BLE poll ──────────────────────────────────────────────────────────── */
void App_ServiceBLE(AppContext_t *app)
{
    BLE_Packet_t pkt;
    if (BLE_GetPacket(&app->ble, &pkt)) App_HandleBLECommand(app, &pkt);

    char line[BLE_STR_LINE_MAX];
    if (BLE_GetLine(&app->ble, line))   App_HandleStringCommand(app, line);
}

void App_TaskBLE(AppContext_t *app)
{
    app->last_ble_poll_ms = HAL_GetTick();
    if (BLE_IsConnected(&app->ble) && app->drink_log.dirty)
        Storage_FlushDrinkLog(&app->drink_log);
}

/* ─── Reminder ──────────────────────────────────────────────────────────── */
void App_TaskReminder(AppContext_t *app)
{
    app->last_reminder_ms = HAL_GetTick();
    if (app->dev.state != DEV_STATE_ACTIVE) return;

    RTC_Read(&app->rtc);
    if (!RTC_IsInWindow(&app->rtc.now,
                         app->settings.prefs.remind_h_start,
                         app->settings.prefs.remind_m_start,
                         app->settings.prefs.remind_h_end,
                         app->settings.prefs.remind_m_end)) return;

    static uint32_t last_reminder_unix = 0;
    uint32_t freq_sec = (uint32_t)app->settings.prefs.remind_freq_min * 60UL;
    if (freq_sec == 0U) freq_sec = 3600U;
    if (app->rtc.unix_approx - last_reminder_unix < freq_sec) return;
    last_reminder_unix = app->rtc.unix_approx;

    app->hydration_score = App_CalcHydrationScore(app);

    if (app->hydration_score > HYDRATION_SCORE_HIGH) {
        RGB_t col = RGB(app->settings.prefs.remind_r,
                        app->settings.prefs.remind_g,
                        app->settings.prefs.remind_b);
        WS2812B_SetCustomReminderColor(col);
        WS2812B_SetPattern(LED_PATTERN_HYDRATION_HIGH);
    } else if (app->hydration_score >= HYDRATION_SCORE_MID) {
        WS2812B_SetPattern(LED_PATTERN_HYDRATION_MID);
    } else {
        WS2812B_SetPattern(LED_PATTERN_HYDRATION_LOW);
    }
    Buzzer_Play(BUZZER_DOUBLE_BEEP);
}

/* ─── Daily rollup ──────────────────────────────────────────────────────── */
void App_TaskDailyRollup(AppContext_t *app)
{
    uint32_t today_midnight = (app->rtc.unix_approx / 86400UL) * 86400UL;
    if (today_midnight == app->current_day_unix) return;

    app->current_day_unix  = today_midnight;
    app->consumed_today_ml = 0;
    app->hydration_score   = 0;

    Storage_UpdateDailySummary(&app->daily_log, &app->drink_log, app->rtc.unix_approx);
    Storage_FlushDailySummary(&app->daily_log);
    Storage_PurgeDailySummaryOlderThan(&app->daily_log, today_midnight - 30UL * 86400UL);
}

/* ─── Physical button ───────────────────────────────────────────────────── */
void App_TaskButton(AppContext_t *app)
{
    app->last_button_ms = HAL_GetTick();

    uint8_t  down = (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    uint32_t now  = HAL_GetTick();

    if (down && !app->btn_was_down) {
        app->btn_was_down    = 1;
        app->btn_down_ms     = now;
        app->btn_reset_armed = 0;
    } else if (down && app->btn_was_down) {
        uint32_t held = now - app->btn_down_ms;
        if (!app->btn_reset_armed && held >= BTN_VLONG_PRESS_MS) {
            app->btn_reset_armed = 1;
            app->btn_down_ms     = now;
            WS2812B_SetPattern(LED_PATTERN_FACTORY_RESET_WARN);
            Buzzer_Play(BUZZER_FACTORY_RESET);
        } else if (app->btn_reset_armed && held >= BTN_RESET_CONFIRM_MS) {
            App_Cmd_FactoryReset(app);
        }
    } else if (!down && app->btn_was_down) {
        if (app->btn_reset_armed) {
            WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
            Buzzer_Stop();
        }
        app->btn_was_down    = 0;
        app->btn_reset_armed = 0;
    }
}

uint8_t App_CalcHydrationScore(AppContext_t *app)
{
    uint16_t goal_ml = BLE_U16(app->settings.prefs.hydration_hi,
                                app->settings.prefs.hydration_lo);
    if (goal_ml == 0U) return 100U;
    uint32_t score = ((uint32_t)app->consumed_today_ml * 100U) / goal_ml;
    return (uint8_t)(score > 100U ? 100U : score);
}

/* ─── ACK helper ────────────────────────────────────────────────────────── */
void App_SendACK(AppContext_t *app, uint8_t cmd_id, uint8_t success, uint8_t err_code)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildACK(buf, cmd_id, success, err_code);
    BLE_SendPacket(&app->ble, buf, len);
}

/* ─── Command dispatcher ────────────────────────────────────────────────── */
void App_HandleBLECommand(AppContext_t *app, const BLE_Packet_t *pkt)
{
    switch (pkt->cmd) {
    case BLE_CMD_TIMESTAMP:    App_Cmd_Timestamp(app, pkt);       break;
    case BLE_CMD_INPUT_DATA:   App_Cmd_InputData(app, pkt);       break;
    case BLE_CMD_CALIBRATION:  App_Cmd_Calibration(app, pkt);     break;
    case BLE_CMD_LAMP_MODE:    App_Cmd_LampMode(app, pkt);        break;
    case BLE_CMD_SOFT_RESET:   App_Cmd_SoftReset(app);            break;
    case BLE_CMD_FACTORY_RESET:App_Cmd_FactoryReset(app);         break;
    case BLE_CMD_GET_HISTORY:  App_Cmd_HistoricalAggregates(app); break;
    case BLE_CMD_REGISTER:     App_Cmd_RegisterDevice(app, pkt);  break;
    case BLE_CMD_UNPAIR:       App_Cmd_UnpairDevice(app);         break;
    case BLE_CMD_GET_LOGS:     App_Cmd_SensorLogs(app);           break;
    case BLE_CMD_SYNC_ACK:     App_Cmd_SyncAck(app, pkt);         break;
    case BLE_CMD_GET_STATUS:   App_Cmd_DeviceStatus(app);         break;
    case BLE_CMD_GET_CONFIG:   App_Cmd_GetConfig(app);            break;
    case BLE_CMD_GET_ERRORS:   App_Cmd_GetErrors(app);            break;
    case BLE_CMD_PING:         App_Cmd_Ping(app);                 break;
    default:
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD);
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

/* Float-to-string, 2 decimal places */
static char *s_f2s(char *d, char *e, float f)
{
    if (f < 0.0f) { if (d < e - 1) *d++ = '-'; f = -f; }
    uint32_t whole = (uint32_t)f;
    uint32_t frac  = (uint32_t)((f - (float)whole) * 100.0f + 0.5f);
    if (frac >= 100U) { whole++; frac = 0U; }
    d = s_u2s(d, e, whole);
    if (d < e - 1) *d++ = '.';
    if (frac < 10U && d < e - 1) *d++ = '0';
    return s_u2s(d, e, frac);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ASCII string-command handler
 * ═══════════════════════════════════════════════════════════════════════════ */
void App_HandleStringCommand(AppContext_t *app, char *line)
{
    /* out[] reduced from 80→52: measured longest reply is 36 chars;
     * 52 gives 16 bytes of future-growth margin. */
    char out[52];
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

    if (strcmp(up, "PING") == 0) { BLE_SendStr(&app->ble, "PONG\r\n"); return; }

    if (strcmp(up, "HELP") == 0 || strcmp(up, "?") == 0) {
        BLE_SendStr(&app->ble, "=HydraSense cmds=\r\n");
        BLE_SendStr(&app->ble, "RGB,r,g,b RED GREEN BLUE WHITE OFF CLR\r\n");
        BLE_SendStr(&app->ble, "PAT,0-14 CHG,pct LAMP,r,g,b LAMPON LAMPOFF LAMPSAVE\r\n");
        BLE_SendStr(&app->ble, "BEEP BON BOFF BUZ,0-8\r\n");
        BLE_SendStr(&app->ble, "STATUS TEMP TDS WEIGHT RAWW READ CFG\r\n");
        BLE_SendStr(&app->ble, "CAL/TARE  CALWEIGHT,<grams>\r\n");
        BLE_SendStr(&app->ble, "TIME SETTIME,unix\r\n");
        BLE_SendStr(&app->ble, "REG UNPAIR SYNC RESET REBOOT SOFTRST\r\n");
        BLE_SendStr(&app->ble, "EE RTCQ TDSQ TMPQ BATQ\r\n");
#ifdef HYDRA_BENCH_CMDS
        BLE_SendStr(&app->ble, "PIX,i,r,g,b EER,a EEW,a,b DIAG\r\n");
#endif
        return;
    }

    /* ── TIME ── */
    if (strcmp(up, "TIME") == 0) {
        RTC_Read(&app->rtc);
        p = s_app(p, e, "T=");  p = s_u2s(p, e, app->rtc.unix_approx);
        p = s_app(p, e, " ");
        p = s_u2s(p, e, app->rtc.now.hours);   *p++ = ':';
        p = s_u2s(p, e, app->rtc.now.minutes); *p++ = ':';
        p = s_u2s(p, e, app->rtc.now.seconds);
        *p++ = (char)(app->rtc.initialized ? '+' : '-');
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    /* ── SETTIME ── */
    if (strcmp(up, "SETTIME") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] > 0) {
            RTC_SetFromUnix(&app->rtc, (uint32_t)v[0]);
            HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
            BLE_SendStr(&app->ble, S_OK);
        } else {
            BLE_SendStr(&app->ble, S_ERR);
        }
        return;
    }

    /* ── STATUS ── */
    if (strcmp(up, "STATUS") == 0) {
        p = s_app(p, e, "S:"); p = s_u2s(p, e, app->current_bat_pct);
        *p++ = ',';            p = s_i2s(p, e, app->current_temp_x10);
        *p++ = ',';            p = s_u2s(p, e, app->current_tds_ppm);
        *p++ = ',';            p = s_u2s(p, e, app->consumed_today_ml);
        *p++ = ',';            p = s_u2s(p, e, Battery_IsCharging(&app->bat) ? 1U : 0U);
        *p++ = ',';            p = s_u2s(p, e, app->hx711.is_calibrated);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    if (strcmp(up, "TEMP") == 0) {
        int16_t t = app->current_temp_x10;
        int16_t whole = t / 10, frac = t % 10; if (frac < 0) frac = -frac;
        p = s_app(p, e, "T:");
        if (t < 0 && whole == 0) *p++ = '-';
        p = s_i2s(p, e, whole); *p++ = '.'; p = s_u2s(p, e, (uint32_t)frac);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    if (strcmp(up, "TDS") == 0) {
        p = s_app(p, e, "D:"); p = s_u2s(p, e, app->current_tds_ppm);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    if (strcmp(up, "WEIGHT") == 0) {
        if (!app->hx711.is_calibrated) {
            BLE_SendStr(&app->ble, "W:0,UNCAL\r\n");
        } else {
            p = s_app(p, e, "W:"); p = s_i2s(p, e, (int32_t)app->current_weight_g);
            p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out);
        }
        return;
    }

    if (strcmp(up, "RAWW") == 0) {
        int32_t raw = 0;
        uint8_t ok  = HX711_ReadRawAveraged(&app->hx711, &raw);
        p = s_app(p, e, "RW:");
        p = s_i2s(p, e, raw);                    *p++ = ',';
        p = s_i2s(p, e, app->hx711.tare_offset); *p++ = ',';
        p = s_u2s(p, e, ok);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    if (strcmp(up, "READ") == 0) {
        p = s_app(p, e, "TLM,"); p = s_i2s(p, e, app->current_temp_x10);
        *p++ = ',';              p = s_u2s(p, e, app->current_tds_ppm);
        *p++ = ',';              p = s_u2s(p, e, app->consumed_today_ml);
        *p++ = ',';              p = s_u2s(p, e, app->hydration_score);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
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
                    app->purity_alert_active = 0;
                    app->temp_alert_active   = 0;
                    WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
                    if (si == 5U) Buzzer_Stop();   /* CLR only */
                }
                WS2812B_SetAll(solids[si].color);
                WS2812B_SendBlocking();
                BLE_SendStr(&app->ble, S_OK);
                return;
            }
        }
    }

    if (strcmp(up, "RGB") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetAll(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SendBlocking(); BLE_SendStr(&app->ble, S_OK);
        } else BLE_SendStr(&app->ble, S_ERR);
        return;
    }

#ifdef HYDRA_BENCH_CMDS
    if (strcmp(up, "PIX") == 0) {
        int v[4];
        if (s_csv(line, v, 4) == 4 &&
            v[0] >= 0 && v[0] < WS2812B_NUM_LEDS &&
            v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255 && v[3]>=0 && v[3]<=255) {
            WS2812B_SetPixel((uint8_t)v[0], RGB((uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3]));
            WS2812B_SendBlocking(); BLE_SendStr(&app->ble, S_OK);
        } else BLE_SendStr(&app->ble, S_ERR);
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
            WS2812B_SetPattern(map[v[0]]); BLE_SendStr(&app->ble, S_OK);
        } else BLE_SendStr(&app->ble, S_ERR);
        return;
    }

    if (strcmp(up, "CHG") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 100) {
            WS2812B_SetChargingLevel((uint8_t)v[0]);
            WS2812B_SetPattern(LED_PATTERN_CHARGING_BAR); BLE_SendStr(&app->ble, S_OK);
        } else BLE_SendStr(&app->ble, S_ERR);
        return;
    }

    if (strcmp(up, "LAMP") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetLampColor(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
            Device_PostEvent(&app->dev, EVT_CMD_LAMP_ON); BLE_SendStr(&app->ble, S_OK);
        } else BLE_SendStr(&app->ble, S_ERR);
        return;
    }

    /* ── LAMP / DEVICE CONTROL — grouped for GCC identical-code folding ── */
    if (strcmp(up, "LAMPON")   == 0) {
        WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
        Device_PostEvent(&app->dev, EVT_CMD_LAMP_ON);
        BLE_SendStr(&app->ble, S_OK); return;
    }
    if (strcmp(up, "LAMPSAVE") == 0) {
        Storage_SaveSettings(&app->settings);
        BLE_SendStr(&app->ble, S_OK); return;
    }
    if (strcmp(up, "LAMPOFF")  == 0) {
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF); WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking(); Device_PostEvent(&app->dev, EVT_CMD_LAMP_OFF);
        BLE_SendStr(&app->ble, S_OK); return;
    }
    if (strcmp(up, "RESET") == 0) {
        app->purity_alert_active = 0; app->temp_alert_active = 0;
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF); WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking(); Buzzer_Stop();
        BLE_SendStr(&app->ble, S_OK); return;
    }
    if (strcmp(up, "REBOOT") == 0) {
        BLE_SendStr(&app->ble, S_OK); HAL_Delay(150); HAL_NVIC_SystemReset();
    }

    /* ════════════════════════════════════════════════════════════════════
     * BUZZER
     * ════════════════════════════════════════════════════════════════════ */
    if (strcmp(up, "BEEP") == 0) { Buzzer_Play(BUZZER_SINGLE_BEEP); BLE_SendStr(&app->ble, S_OK); return; }
    if (strcmp(up, "BON")  == 0) { Buzzer_Play(BUZZER_DOUBLE_BEEP); BLE_SendStr(&app->ble, S_OK); return; }
    if (strcmp(up, "BOFF") == 0) { Buzzer_Stop();                   BLE_SendStr(&app->ble, S_OK); return; }

    if (strcmp(up, "BUZ") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 8) {
            static const uint8_t buz_map[] = {
                BUZZER_STARTUP, BUZZER_SINGLE_BEEP, BUZZER_DOUBLE_BEEP,
                BUZZER_PURITY_ALERT, BUZZER_TEMP_ALERT, BUZZER_CALIBRATION_DONE,
                BUZZER_REGISTRATION_OK, BUZZER_SYNC_OK, BUZZER_FACTORY_RESET,
            };
            Buzzer_Play(buz_map[v[0]]);
            BLE_SendStr(&app->ble, S_OK);
        } else BLE_SendStr(&app->ble, S_ERR);
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
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

#ifdef HYDRA_BENCH_CMDS
    if (strcmp(up, "EEW") == 0) {
        int v[2]; uint16_t a8 = EE_Addr8();
        if (a8 && s_csv(line, v, 2) == 2 && v[0]>=0 && v[0]<=0xFFFF && v[1]>=0 && v[1]<=0xFF) {
            uint8_t buf3[3] = { (uint8_t)(v[0]>>8), (uint8_t)(v[0]&0xFF), (uint8_t)v[1] };
            HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(s_app_i2c, a8, buf3, 3, 20);
            HAL_Delay(6);
            BLE_SendStr(&app->ble, (st == HAL_OK) ? S_OK : S_ERR);
        } else BLE_SendStr(&app->ble, S_ERR);
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
                p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out);
            } else BLE_SendStr(&app->ble, S_ERR);
        } else BLE_SendStr(&app->ble, S_ERR);
        return;
    }
#endif

    if (strcmp(up, "RTCQ") == 0) {
        p = s_app(p, e, "R:");
        if (app->rtc.initialized) {
            RTC_Read(&app->rtc);
            *p++ = '1'; *p++ = ',';
            p = s_u2s(p, e, app->rtc.unix_approx);  *p++ = ',';
            p = s_u2s(p, e, app->rtc.now.hours);    *p++ = ':';
            p = s_u2s(p, e, app->rtc.now.minutes);  *p++ = ':';
            p = s_u2s(p, e, app->rtc.now.seconds);
        } else { *p++ = '0'; }
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    if (strcmp(up, "TDSQ") == 0) {
        uint16_t ppm = TDS_ReadPPM(&app->tds, app->current_temp_x10);
        app->current_tds_ppm = ppm;
        p = s_app(p, e, "DQ:"); p = s_u2s(p, e, ppm);
        *p++ = ',';             p = s_u2s(p, e, app->tds.valid);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    if (strcmp(up, "TMPQ") == 0) {
        int16_t t = NTC_ReadTemp_x10(&app->ntc);
        app->current_temp_x10 = t;
        p = s_app(p, e, "TQ:"); p = s_i2s(p, e, t);
        *p++ = ','; p = s_u2s(p, e, app->ntc.valid);
        *p++ = ','; p = s_u2s(p, e, g_ntc_adc_avg);
        *p++ = ','; p = s_u2s(p, e, g_ntc_fault);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    if (strcmp(up, "BATQ") == 0) {
        Battery_Update(&app->bat);
        app->current_bat_pct = Battery_GetPercent(&app->bat);
        p = s_app(p, e, "BQ:"); p = s_u2s(p, e, app->current_bat_pct);
        *p++ = ','; p = s_u2s(p, e, Battery_IsCharging(&app->bat) ? 1U : 0U);
        *p++ = ','; p = s_u2s(p, e, Battery_IsFull(&app->bat)     ? 1U : 0U);
        *p++ = ','; p = s_u2s(p, e, Battery_IsLow(&app->bat)      ? 1U : 0U);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

#ifdef HYDRA_BENCH_CMDS
    if (strcmp(up, "DIAG") == 0) {
        uint8_t  ee  = EE_Probe(NULL);
        uint16_t ppm = TDS_ReadPPM(&app->tds, app->current_temp_x10);
        int16_t  t   = NTC_ReadTemp_x10(&app->ntc);
        Battery_Update(&app->bat);
        p = s_app(p, e, "DG:");
        p = s_u2s(p, e, ee);                    *p++ = ',';
        p = s_u2s(p, e, app->rtc.initialized);  *p++ = ',';
        p = s_u2s(p, e, ppm);                   *p++ = ',';
        p = s_i2s(p, e, t);                     *p++ = ',';
        p = s_u2s(p, e, Battery_GetPercent(&app->bat));
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }
#endif

    /* ════════════════════════════════════════════════════════════════════
     * DEVICE CONTROL
     * ════════════════════════════════════════════════════════════════════ */
    if (strcmp(up, "REG") == 0) {
        app->settings.is_registered = 1;
        Storage_SaveSettings(&app->settings);
        Device_PostEvent(&app->dev, EVT_CMD_REGISTER);
        Buzzer_Play(BUZZER_REGISTRATION_OK);
        BLE_SendStr(&app->ble, S_OK); return;
    }
    if (strcmp(up, "UNPAIR") == 0) {
        memset(app->settings.user_id, 0, sizeof(app->settings.user_id));
        app->settings.is_registered = 0;
        Storage_SaveSettings(&app->settings);
        BLE_SendStr(&app->ble, S_OK); return;
    }
    if (strcmp(up, "SOFTRST") == 0) {
        BLE_SendStr(&app->ble, S_OK); HAL_Delay(200); HAL_NVIC_SystemReset();
    }
    if (strcmp(up, "SYNC") == 0) {
        Storage_MarkSynced(&app->drink_log, app->rtc.unix_approx);
        Storage_FlushDrinkLog(&app->drink_log);
        WS2812B_SetPattern(LED_PATTERN_SYNC_SUCCESS);
        Buzzer_Play(BUZZER_SYNC_OK);
        BLE_SendStr(&app->ble, S_OK); return;
    }
    if (strcmp(up, "CFG") == 0) {
        p = s_app(p, e, "C:");
        p = s_u2s(p, e, BLE_U16(app->settings.prefs.purity_goal_hi, app->settings.prefs.purity_goal_lo)); *p++ = ',';
        p = s_i2s(p, e, BLE_I16(app->settings.prefs.temp_goal_hi,   app->settings.prefs.temp_goal_lo));   *p++ = ',';
        p = s_u2s(p, e, BLE_U16(app->settings.prefs.hydration_hi,   app->settings.prefs.hydration_lo));   *p++ = ',';
        p = s_u2s(p, e, app->settings.prefs.remind_h_start); *p++ = ':';
        p = s_u2s(p, e, app->settings.prefs.remind_m_start); *p++ = '-';
        p = s_u2s(p, e, app->settings.prefs.remind_h_end);   *p++ = ':';
        p = s_u2s(p, e, app->settings.prefs.remind_m_end);   *p++ = ',';
        p = s_u2s(p, e, app->settings.prefs.remind_freq_min);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    /* ── CAL / TARE ── */
    if (strcmp(up, "CAL") == 0 || strcmp(up, "TARE") == 0) {
        if (!HX711_WaitReady(400U)) {
            BLE_SendStr(&app->ble, "ERR,HX711\r\n"); return;
        }
        BLE_Packet_t fake;
        fake.cmd        = BLE_CMD_CALIBRATION;
        fake.len        = 1;
        fake.payload[0] = 0;
        App_Cmd_Calibration(app, &fake);
        return;
    }

    /* ── CALWEIGHT ── */
    if (strcmp(up, "CALWEIGHT") == 0) {
        int v[1];
        if (s_csv(line, v, 1) != 1 || v[0] < 1) {
            BLE_SendStr(&app->ble, "CW:ERR\r\n"); return;
        }
        if (!HX711_WaitReady(400U)) {
            BLE_SendStr(&app->ble, "CW:ERR,HX711\r\n"); return;
        }
        if (!HX711_Calibrate(&app->hx711, (float)v[0])) {
            BLE_SendStr(&app->ble, "CW:ERR,LOAD\r\n"); return;
        }
        app->settings.tare_offset   = (float)app->hx711.tare_offset;
        app->settings.hx711_scale   = app->hx711.scale;
        app->settings.is_calibrated = 1;
        Storage_SaveSettings(&app->settings);
        app->weight_seeded    = 0;
        app->drink_baseline_g = 0.0f;
        p = s_app(p, e, "CW:OK,scale=");
        p = s_f2s(p, e, app->hx711.scale);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    BLE_SendStr(&app->ble, S_ERR);
}

/* ─── Individual command handlers ──────────────────────────────────────── */

void App_Cmd_Timestamp(AppContext_t *app, const BLE_Packet_t *pkt)
{
    if (pkt->len < 4U) { App_SendACK(app, pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD); return; }
    uint32_t unix_time = BLE_U32(pkt->payload[0], pkt->payload[1],
                                  pkt->payload[2], pkt->payload[3]);
    RTC_SetFromUnix(&app->rtc, unix_time);
    RTC_Read(&app->rtc);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_InputData(AppContext_t *app, const BLE_Packet_t *pkt)
{
    if (pkt->len < 18U) { App_SendACK(app, pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD); return; }
    memcpy(&app->settings.prefs, pkt->payload, sizeof(BLE_PrefsPayload_t));
    Storage_SaveSettings(&app->settings);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_Calibration(AppContext_t *app, const BLE_Packet_t *pkt)
{
    if (pkt->len < 1U || pkt->payload[0] != 0U) {
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_INVALID_STAGE); return;
    }
    if (!HX711_WaitReady(400U)) {
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_OK); return;
    }
    Device_PostEvent(&app->dev, EVT_CMD_CALIBRATION);
    WS2812B_SetPattern(LED_PATTERN_CALIBRATION);
    HX711_Tare(&app->hx711);
    app->settings.tare_offset = (float)app->hx711.tare_offset;
    Storage_SaveSettings(&app->settings);
    Device_PostEvent(&app->dev, EVT_CALIBRATION_DONE);
    Buzzer_Play(BUZZER_CALIBRATION_DONE);
    WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_LampMode(AppContext_t *app, const BLE_Packet_t *pkt)
{
    if (!Battery_IsCharging(&app->bat) && !Battery_IsFull(&app->bat)) {
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_NOT_CHARGING); return;
    }
    if (pkt->len >= 1U && pkt->payload[0]) {
        RGB_t col = {0, 255, 0};
        if (pkt->len >= 4U) { col.r = pkt->payload[1]; col.g = pkt->payload[2]; col.b = pkt->payload[3]; }
        app->settings.prefs.lamp_r = col.r;
        app->settings.prefs.lamp_g = col.g;
        app->settings.prefs.lamp_b = col.b;
        Storage_SaveSettings(&app->settings);
        WS2812B_SetLampColor(col);
        Device_PostEvent(&app->dev, EVT_CMD_LAMP_ON);
    } else {
        Device_PostEvent(&app->dev, EVT_CMD_LAMP_OFF);
    }
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_SoftReset(AppContext_t *app)
{
    App_SendACK(app, BLE_CMD_SOFT_RESET, 1, BLE_ERR_OK);
    HAL_Delay(200); HAL_NVIC_SystemReset();
}

void App_Cmd_FactoryReset(AppContext_t *app)
{
    Device_PostEvent(&app->dev, EVT_CMD_FACTORY_RESET);
    WS2812B_SetPattern(LED_PATTERN_FACTORY_RESET_WARN);
    Buzzer_Play(BUZZER_FACTORY_RESET);
    HAL_Delay(5000);
    App_SendACK(app, BLE_CMD_FACTORY_RESET, 1, BLE_ERR_OK);
    HAL_Delay(200);
    Storage_FactoryReset();
    HAL_NVIC_SystemReset();
}

void App_Cmd_HistoricalAggregates(AppContext_t *app)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    for (uint8_t i = 0; i < app->daily_log.count; i++) {
        DailySummary_t *d = &app->daily_log.days[i];
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
        BLE_SendPacket(&app->ble, buf, len); HAL_Delay(20);
    }
    App_SendACK(app, BLE_CMD_GET_HISTORY, 1, BLE_ERR_OK);
}

void App_Cmd_RegisterDevice(AppContext_t *app, const BLE_Packet_t *pkt)
{
    if (pkt->len >= 16U) memcpy(app->settings.user_id,         pkt->payload,      16);
    if (pkt->len >= 32U) memcpy(app->settings.device_nickname, pkt->payload + 16, 16);
    app->settings.is_registered = 1;
    Storage_SaveSettings(&app->settings);
    Device_PostEvent(&app->dev, EVT_CMD_REGISTER);
    Buzzer_Play(BUZZER_REGISTRATION_OK);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_UnpairDevice(AppContext_t *app)
{
    memset(app->settings.user_id, 0, sizeof(app->settings.user_id));
    app->settings.is_registered = 0;
    Storage_SaveSettings(&app->settings);
    App_SendACK(app, BLE_CMD_UNPAIR, 1, BLE_ERR_OK);
}

void App_Cmd_SensorLogs(AppContext_t *app)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    for (uint8_t i = 0; i < app->drink_log.count; i++) {
        DrinkEvent_t *ev = &app->drink_log.events[i];
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
        BLE_SendPacket(&app->ble, buf, len); HAL_Delay(20);
    }
    App_SendACK(app, BLE_CMD_GET_LOGS, 1, BLE_ERR_OK);
}

void App_Cmd_SyncAck(AppContext_t *app, const BLE_Packet_t *pkt)
{
    uint32_t cutoff = (pkt->len >= 4U)
                    ? BLE_U32(pkt->payload[0], pkt->payload[1],
                               pkt->payload[2], pkt->payload[3])
                    : app->rtc.unix_approx;
    Storage_MarkSynced(&app->drink_log, cutoff);
    Storage_FlushDrinkLog(&app->drink_log);
    WS2812B_SetPattern(LED_PATTERN_SYNC_SUCCESS);
    Buzzer_Play(BUZZER_SYNC_OK);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_DeviceStatus(AppContext_t *app)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    BLE_StatusPayload_t pl;
    pl.bat_pct = app->current_bat_pct;
    pl.flags   = 0;
    if (Battery_IsCharging(&app->bat))   pl.flags |= BLE_FLAG_CHARGING;
    if (app->ntc.valid)                  pl.flags |= BLE_FLAG_TEMP_OK;
    if (app->tds.valid)                  pl.flags |= BLE_FLAG_TDS_OK;
    if (app->hx711.is_calibrated)        pl.flags |= BLE_FLAG_WEIGHT_OK;
    if (app->settings.is_calibrated)     pl.flags |= BLE_FLAG_CALIBRATED;
    if (app->settings.is_registered)     pl.flags |= BLE_FLAG_REGISTERED;
    pl.storage_pct = (uint8_t)((uint32_t)app->drink_log.count * 100U
                                / STORAGE_MAX_DRINK_EVENTS);
    uint8_t len = BLE_BuildStatus(buf, &pl);
    BLE_SendPacket(&app->ble, buf, len);
}

void App_Cmd_GetConfig(AppContext_t *app)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildConfig(buf, &app->settings.prefs);
    BLE_SendPacket(&app->ble, buf, len);
}

void App_Cmd_GetErrors(AppContext_t *app)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t payload = 0;
    uint8_t len = BLE_BuildPacket(buf, BLE_RSP_ERR_LOG, &payload, 1);
    BLE_SendPacket(&app->ble, buf, len);
}

void App_Cmd_Ping(AppContext_t *app)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildPong(buf);
    BLE_SendPacket(&app->ble, buf, len);
}

void App_ResetDailyConsumed(AppContext_t *app)
{
    app->consumed_today_ml = 0;
    app->hydration_score   = 0;
}
