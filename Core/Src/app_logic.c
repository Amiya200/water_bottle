#pragma GCC optimize("Os")
#include "app_logic.h"
#include <string.h>

/* ─── Init ──────────────────────────────────────────────────── */
void App_Init(AppContext_t *app,
              ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              UART_HandleTypeDef *huart,
              TIM_HandleTypeDef  *htim_ws,
              TIM_HandleTypeDef  *htim_buz)
{
    memset(app, 0, sizeof(AppContext_t));

    WS2812B_Init(htim_ws);
    Buzzer_Init(htim_buz);
    HX711_Init(&app->hx711);
    TDS_Init(&app->tds, hadc);
    NTC_Init(&app->ntc, hadc);
    Battery_Init(&app->bat, hadc);
//    BMA253_Init(&app->bma, hi2c);
    BLE_Init(&app->ble, huart);
    RTC_Init(&app->rtc, hi2c);
    Storage_Init();

    Storage_LoadSettings(&app->settings);
    Storage_LoadDrinkLog(&app->drink_log);
    Storage_LoadDailySummary(&app->daily_log);

    app->hx711.tare_offset   = (int32_t)app->settings.tare_offset;
    app->hx711.scale         = app->settings.hx711_scale > 0.0f
                             ? app->settings.hx711_scale : HX711_DEFAULT_SCALE;
    app->hx711.is_calibrated = app->settings.is_calibrated;
    app->current_temp_x10    = 250;   /* 25.0 °C default */

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

/* ─── Main run loop ─────────────────────────────────────────── */
void App_Run(AppContext_t *app)
{
    uint32_t now = HAL_GetTick();

    Device_Run(&app->dev);

    if (now - app->last_weight_ms  >= APP_WEIGHT_POLL_MS)  App_TaskWeight(app);
    if (now - app->last_tds_ms     >= APP_TDS_POLL_MS)     App_TaskTDS(app);
    if (now - app->last_temp_ms    >= APP_TEMP_POLL_MS)    App_TaskTemp(app);
    if (now - app->last_battery_ms >= APP_BATTERY_POLL_MS) App_TaskBattery(app);
    if (now - app->last_ble_poll_ms>= APP_BLE_STATE_POLL_MS) App_TaskBLE(app);
    if (now - app->last_reminder_ms>= APP_REMINDER_CHECK_MS) App_TaskReminder(app);

    App_TaskDailyRollup(app);
    WS2812B_Update();
    Buzzer_Update();
}

/* ─── Weight / drink detection ──────────────────────────────── */
void App_TaskWeight(AppContext_t *app)
{
    app->last_weight_ms   = HAL_GetTick();
    if (!app->hx711.is_calibrated) return;

    float w = HX711_ReadMillilitres(&app->hx711);
    app->prev_weight_g    = app->current_weight_g;
    app->current_weight_g = w;
    App_CheckDrinkEvent(app);

    if (Battery_IsCharging(&app->bat) || Battery_IsFull(&app->bat)) {
        WS2812B_SetChargingLevel(app->current_bat_pct);
    }
}

void App_CheckDrinkEvent(AppContext_t *app)
{
    if (BMA253_PopMotionFlag(&app->bma)) {
        app->motion_pending     = 1;
        app->motion_start_ms    = HAL_GetTick();
        app->weight_at_motion_g = app->current_weight_g;
    }

    if (app->motion_pending) {
        if (HAL_GetTick() - app->motion_start_ms >= DRINK_SETTLE_MS) {
            float delta = app->weight_at_motion_g - app->current_weight_g;
            if (delta >= (float)DRINK_MIN_VOLUME_ML) {
                App_RecordDrinkEvent(app, delta);
            }
            app->motion_pending = 0;
        }
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

/* ─── TDS ───────────────────────────────────────────────────── */
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

/* ─── Temperature ───────────────────────────────────────────── */
void App_TaskTemp(AppContext_t *app)
{
    app->last_temp_ms    = HAL_GetTick();
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

/* ─── Battery ───────────────────────────────────────────────── */
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

/* ─── BLE poll ──────────────────────────────────────────────── */
void App_TaskBLE(AppContext_t *app)
{
    app->last_ble_poll_ms = HAL_GetTick();

    BLE_Packet_t pkt;
    if (BLE_GetPacket(&app->ble, &pkt)) {
        App_HandleBLECommand(app, &pkt);
    }

    /* ASCII string commands (parallel to binary protocol) */
    char line[BLE_STR_LINE_MAX];
    if (BLE_GetLine(&app->ble, line)) {
        App_HandleStringCommand(app, line);
    }

    if (BLE_IsConnected(&app->ble) && app->drink_log.dirty) {
        Storage_FlushDrinkLog(&app->drink_log);
    }
}

/* ─── Reminder ──────────────────────────────────────────────── */
void App_TaskReminder(AppContext_t *app)
{
    app->last_reminder_ms = HAL_GetTick();
    if (app->dev.state != DEV_STATE_ACTIVE) return;

    RTC_Read(&app->rtc);

    /* Read reminder window h/m from prefs bytes directly — no atoi */
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
        /* Custom reminder colour stored directly as R,G,B bytes in prefs */
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

/* ─── Daily rollup ──────────────────────────────────────────── */
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

/* ─── Hydration score ───────────────────────────────────────── */
uint8_t App_CalcHydrationScore(AppContext_t *app)
{
    uint16_t goal_ml = BLE_U16(app->settings.prefs.hydration_hi,
                                app->settings.prefs.hydration_lo);
    if (goal_ml == 0U) return 100U;
    uint32_t score = ((uint32_t)app->consumed_today_ml * 100U) / goal_ml;
    return (uint8_t)(score > 100U ? 100U : score);
}

/* ─── ACK helper ────────────────────────────────────────────── */
void App_SendACK(AppContext_t *app, uint8_t cmd_id, uint8_t success, uint8_t err_code)
{
    uint8_t buf[BLE_PKT_MAX_LEN];
    uint8_t len = BLE_BuildACK(buf, cmd_id, success, err_code);
    BLE_SendPacket(&app->ble, buf, len);
}

/* ─── Command dispatcher ────────────────────────────────────── */
void App_HandleBLECommand(AppContext_t *app, const BLE_Packet_t *pkt)
{
    switch (pkt->cmd) {
    case BLE_CMD_TIMESTAMP:    App_Cmd_Timestamp(app, pkt);    break;
    case BLE_CMD_INPUT_DATA:   App_Cmd_InputData(app, pkt);    break;
    case BLE_CMD_CALIBRATION:  App_Cmd_Calibration(app, pkt);  break;
    case BLE_CMD_LAMP_MODE:    App_Cmd_LampMode(app, pkt);     break;
    case BLE_CMD_SOFT_RESET:   App_Cmd_SoftReset(app);         break;
    case BLE_CMD_FACTORY_RESET:App_Cmd_FactoryReset(app);      break;
    case BLE_CMD_GET_HISTORY:  App_Cmd_HistoricalAggregates(app); break;
    case BLE_CMD_REGISTER:     App_Cmd_RegisterDevice(app, pkt); break;
    case BLE_CMD_UNPAIR:       App_Cmd_UnpairDevice(app);      break;
    case BLE_CMD_GET_LOGS:     App_Cmd_SensorLogs(app);        break;
    case BLE_CMD_SYNC_ACK:     App_Cmd_SyncAck(app, pkt);      break;
    case BLE_CMD_GET_STATUS:   App_Cmd_DeviceStatus(app);      break;
    case BLE_CMD_GET_CONFIG:   App_Cmd_GetConfig(app);         break;
    case BLE_CMD_GET_ERRORS:   App_Cmd_GetErrors(app);         break;
    case BLE_CMD_PING:         App_Cmd_Ping(app);              break;
    default:
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD);
        break;
    }
}

/* ─── ASCII string-command handler ──────────────────────────────────────────
 * Parallel to the binary protocol. Lets a plain serial/BLE terminal control
 * the device with simple text lines. Replies are short ASCII strings.
 *
 *   PING            -> PONG
 *   STATUS          -> STATUS bat=.. tmp=.. tds=.. ml=.. chg=..
 *   TEMP            -> TEMP,<c.c>     (current water temperature)
 *   TDS             -> TDS,<ppm>      (current purity)
 *   WEIGHT          -> WEIGHT,<grams>
 *   READ            -> TLM,temp_x10,tds,ml,score
 *   RED/GREEN/BLUE/WHITE/OFF          -> solid LED colours
 *   RGB,r,g,b       -> all LEDs to r,g,b
 *   LAMP,r,g,b      -> lamp colour (also enters lamp pattern)
 *   BEEP            -> single confirm beep
 *   BON / BOFF      -> buzzer continuous on (double-beep pattern) / stop
 *   CAL             -> start empty-bottle calibration (tare)
 *   TARE            -> alias of CAL
 *
 * Uses only tiny local int->string helpers (no printf/snprintf).
 * --------------------------------------------------------------------------*/

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

/* Parse up to max comma-separated ints after the first comma. Returns count. */
static int s_csv(const char *s, int *out, int max)
{
    while (*s && *s != ',') s++;
    if (*s != ',') return 0;
    s++;
    int c = 0;
    while (*s && c < max) {
        while (*s == ' ') s++;
        int neg = 0; if (*s == '-') { neg = 1; s++; }
        if (*s < '0' || *s > '9') break;
        int32_t v = 0;
        while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
        out[c++] = neg ? -v : v;
        while (*s && *s != ',') s++;
        if (*s == ',') s++;
    }
    return c;
}

void App_HandleStringCommand(AppContext_t *app, char *line)
{
    /* Uppercase the verb up to the first comma (numbers stay intact). */
    char up[16];
    uint8_t i = 0;
    for (; line[i] && line[i] != ',' && i < sizeof(up) - 1U; i++) {
        char c = line[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        up[i] = c;
    }
    up[i] = '\0';

    char out[64];
    char *e = out + sizeof(out);
    char *p = out;

    if (strcmp(up, "PING") == 0) { BLE_SendStr(&app->ble, "PONG\r\n"); return; }

    if (strcmp(up, "STATUS") == 0) {
        p = s_app(p, e, "STATUS bat=");  p = s_u2s(p, e, app->current_bat_pct);
        p = s_app(p, e, " tmp=");        p = s_i2s(p, e, app->current_temp_x10);
        p = s_app(p, e, " tds=");        p = s_u2s(p, e, app->current_tds_ppm);
        p = s_app(p, e, " ml=");         p = s_u2s(p, e, app->consumed_today_ml);
        p = s_app(p, e, " chg=");        p = s_u2s(p, e, Battery_IsCharging(&app->bat) ? 1U : 0U);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    if (strcmp(up, "TEMP") == 0) {
        int16_t t = app->current_temp_x10;
        int16_t whole = t / 10, frac = t % 10; if (frac < 0) frac = -frac;
        p = s_app(p, e, "TEMP,");
        if (t < 0 && whole == 0) *p++ = '-';
        p = s_i2s(p, e, whole); *p++ = '.'; p = s_u2s(p, e, (uint32_t)frac);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    if (strcmp(up, "TDS") == 0) {
        p = s_app(p, e, "TDS,"); p = s_u2s(p, e, app->current_tds_ppm);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    if (strcmp(up, "WEIGHT") == 0) {
        p = s_app(p, e, "WEIGHT,"); p = s_i2s(p, e, (int32_t)app->current_weight_g);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    if (strcmp(up, "READ") == 0) {
        p = s_app(p, e, "TLM,"); p = s_i2s(p, e, app->current_temp_x10);
        *p++ = ',';             p = s_u2s(p, e, app->current_tds_ppm);
        *p++ = ',';             p = s_u2s(p, e, app->consumed_today_ml);
        *p++ = ',';             p = s_u2s(p, e, app->hydration_score);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    /* ---- LED control ---- */
    if (strcmp(up, "RED") == 0)   { WS2812B_SetAll(RGB_RED);   WS2812B_SendBlocking(); BLE_SendStr(&app->ble, "OK\r\n"); return; }
    if (strcmp(up, "GREEN") == 0) { WS2812B_SetAll(RGB_GREEN); WS2812B_SendBlocking(); BLE_SendStr(&app->ble, "OK\r\n"); return; }
    if (strcmp(up, "BLUE") == 0)  { WS2812B_SetAll(RGB_BLUE);  WS2812B_SendBlocking(); BLE_SendStr(&app->ble, "OK\r\n"); return; }
    if (strcmp(up, "WHITE") == 0) { WS2812B_SetAll(RGB(255,255,255)); WS2812B_SendBlocking(); BLE_SendStr(&app->ble, "OK\r\n"); return; }
    if (strcmp(up, "OFF") == 0)   { WS2812B_SetAll(RGB_OFF);   WS2812B_SendBlocking(); BLE_SendStr(&app->ble, "OK\r\n"); return; }

    if (strcmp(up, "RGB") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetAll(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SendBlocking();
            BLE_SendStr(&app->ble, "OK\r\n");
        } else BLE_SendStr(&app->ble, "ERR rgb\r\n");
        return;
    }

    if (strcmp(up, "LAMP") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetLampColor(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
            BLE_SendStr(&app->ble, "OK\r\n");
        } else BLE_SendStr(&app->ble, "ERR lamp\r\n");
        return;
    }

    /* ---- Buzzer ---- */
    if (strcmp(up, "BEEP") == 0) { Buzzer_Play(BUZZER_SINGLE_BEEP); BLE_SendStr(&app->ble, "OK\r\n"); return; }
    if (strcmp(up, "BON")  == 0) { Buzzer_Play(BUZZER_DOUBLE_BEEP); BLE_SendStr(&app->ble, "OK\r\n"); return; }
    if (strcmp(up, "BOFF") == 0) { Buzzer_Stop();                   BLE_SendStr(&app->ble, "OK\r\n"); return; }

    /* ---- Calibration (empty-bottle tare) ---- */
    if (strcmp(up, "CAL") == 0 || strcmp(up, "TARE") == 0) {
        BLE_Packet_t fake;          /* reuse existing calibration handler */
        fake.cmd = BLE_CMD_CALIBRATION;
        fake.len = 1;
        fake.payload[0] = 0;        /* stage 0 = empty_bottle */
        App_Cmd_Calibration(app, &fake);
        BLE_SendStr(&app->ble, "OK cal\r\n");
        return;
    }

    BLE_SendStr(&app->ble, "ERR unknown\r\n");
}

/* ─── Individual command handlers ───────────────────────────── */

void App_Cmd_Timestamp(AppContext_t *app, const BLE_Packet_t *pkt)
{
    /* Payload: 4-byte unix timestamp big-endian */
    if (pkt->len < 4U) { App_SendACK(app, pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD); return; }
    uint32_t unix_time = BLE_U32(pkt->payload[0], pkt->payload[1],
                                  pkt->payload[2], pkt->payload[3]);
    RTC_SetFromUnix(&app->rtc, unix_time);
    RTC_Read(&app->rtc);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_InputData(AppContext_t *app, const BLE_Packet_t *pkt)
{
    /* Payload: BLE_PrefsPayload_t (18 bytes) */
    if (pkt->len < 18U) { App_SendACK(app, pkt->cmd, 0, BLE_ERR_UNKNOWN_CMD); return; }
    memcpy(&app->settings.prefs, pkt->payload, sizeof(BLE_PrefsPayload_t));
    Storage_SaveSettings(&app->settings);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_Calibration(AppContext_t *app, const BLE_Packet_t *pkt)
{
    /* Payload[0]: stage — 0 = empty_bottle */
    if (pkt->len < 1U || pkt->payload[0] != 0U) {
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_INVALID_STAGE);
        return;
    }
    Device_PostEvent(&app->dev, EVT_CMD_CALIBRATION);
    WS2812B_SetPattern(LED_PATTERN_CALIBRATION);
    HX711_Tare(&app->hx711);
    app->settings.tare_offset   = (float)app->hx711.tare_offset;
    app->settings.is_calibrated = 1;
    Storage_SaveSettings(&app->settings);
    Device_PostEvent(&app->dev, EVT_CALIBRATION_DONE);
    Buzzer_Play(BUZZER_CALIBRATION_DONE);
    App_SendACK(app, pkt->cmd, 1, BLE_ERR_OK);
}

void App_Cmd_LampMode(AppContext_t *app, const BLE_Packet_t *pkt)
{
    if (!Battery_IsCharging(&app->bat) && !Battery_IsFull(&app->bat)) {
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_NOT_CHARGING);
        return;
    }
    /* Payload[0]: enable=1 / disable=0; [1][2][3]: R,G,B */
    if (pkt->len >= 1U && pkt->payload[0]) {
        RGB_t col = {0, 255, 0};   /* default green */
        if (pkt->len >= 4U) {
            col.r = pkt->payload[1];
            col.g = pkt->payload[2];
            col.b = pkt->payload[3];
        }
        /* Persist RGB in lamp_rgb bytes */
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
    HAL_Delay(200);
    HAL_NVIC_SystemReset();
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
        BLE_DailyPayload_t p;
        p.unix_b3 = (uint8_t)(d->date_unix >> 24);
        p.unix_b2 = (uint8_t)(d->date_unix >> 16);
        p.unix_b1 = (uint8_t)(d->date_unix >>  8);
        p.unix_b0 = (uint8_t)(d->date_unix);
        p.ml_hi   = (uint8_t)(d->total_ml >> 8);
        p.ml_lo   = (uint8_t)(d->total_ml);
        p.ppm_hi  = (uint8_t)(d->avg_purity_ppm >> 8);
        p.ppm_lo  = (uint8_t)(d->avg_purity_ppm);
        p.temp_hi = (uint8_t)((uint16_t)d->avg_temp_x10 >> 8);
        p.temp_lo = (uint8_t)(d->avg_temp_x10);
        uint8_t len = BLE_BuildDaily(buf, &p);
        BLE_SendPacket(&app->ble, buf, len);
        HAL_Delay(20);
    }
    App_SendACK(app, BLE_CMD_GET_HISTORY, 1, BLE_ERR_OK);
}

void App_Cmd_RegisterDevice(AppContext_t *app, const BLE_Packet_t *pkt)
{
    /* Payload: user_id[16] + nickname[16] = 32 bytes */
    if (pkt->len >= 16U)
        memcpy(app->settings.user_id, pkt->payload, 16);
    if (pkt->len >= 32U)
        memcpy(app->settings.device_nickname, pkt->payload + 16, 16);
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
        BLE_LogEntryPayload_t p;
        p.unix_b3 = (uint8_t)(ev->unix_time >> 24);
        p.unix_b2 = (uint8_t)(ev->unix_time >> 16);
        p.unix_b1 = (uint8_t)(ev->unix_time >>  8);
        p.unix_b0 = (uint8_t)(ev->unix_time);
        p.vol_hi  = (uint8_t)(ev->volume_ml >> 8);
        p.vol_lo  = (uint8_t)(ev->volume_ml);
        p.ppm_hi  = (uint8_t)(ev->purity_ppm >> 8);
        p.ppm_lo  = (uint8_t)(ev->purity_ppm);
        p.temp_hi = (uint8_t)((uint16_t)ev->temp_x10 >> 8);
        p.temp_lo = (uint8_t)(ev->temp_x10);
        p.synced  = ev->synced;
        uint8_t len = BLE_BuildLogEntry(buf, &p);
        BLE_SendPacket(&app->ble, buf, len);
        HAL_Delay(20);
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
    BLE_StatusPayload_t p;
    p.bat_pct = app->current_bat_pct;
    p.flags   = 0;
    if (Battery_IsCharging(&app->bat))    p.flags |= BLE_FLAG_CHARGING;
    if (app->ntc.valid)                   p.flags |= BLE_FLAG_TEMP_OK;
    if (app->tds.valid)                   p.flags |= BLE_FLAG_TDS_OK;
    if (app->hx711.is_calibrated)         p.flags |= BLE_FLAG_WEIGHT_OK;
    if (app->settings.is_calibrated)      p.flags |= BLE_FLAG_CALIBRATED;
    if (app->settings.is_registered)      p.flags |= BLE_FLAG_REGISTERED;
    p.storage_pct = (uint8_t)((uint32_t)app->drink_log.count * 100U
                               / STORAGE_MAX_DRINK_EVENTS);
    uint8_t len = BLE_BuildStatus(buf, &p);
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
    /* Placeholder — error count = 0 */
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
