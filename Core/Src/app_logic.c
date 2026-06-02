#pragma GCC optimize("Os")
#include "app_logic.h"
#include "main.h"
#include <string.h>

/* ─── External debug globals from ntc_temp.c ────────────────────────────────
 * These let the TMPQ / DIAG commands report the raw averaged ADC and the NTC
 * fault code over BLE. If you already export these in ntc_temp.h, you can
 * remove these externs (harmless duplicates either way). */
extern volatile uint16_t g_ntc_adc_avg;
extern volatile uint8_t  g_ntc_fault;

/* ─── Static I2C bus handle ─────────────────────────────────────────────────
 * Captured in App_Init so the EEPROM (M24512) / RTC probe commands can reach
 * the bus. The M24512, the PCF8563 RTC, and the (disabled) BMA253 all live on
 * I2C1, so any probe here uses a short timeout and never blocks indefinitely. */
static I2C_HandleTypeDef *s_app_i2c = NULL;

/* M24512 7-bit base address. A0..A2 strapping can move it within 0x50..0x57.
 * HAL APIs expect the 8-bit form, hence the (addr << 1) at each call site.
 *
 * IMPORTANT: the PCF8563 RTC sits at RTC_I2C_ADDR (0x51, 8-bit 0xA2) on the
 * SAME bus, which is INSIDE the EEPROM address window. The probe MUST skip it
 * or it will mistake the RTC for an EEPROM. We compare against the 7-bit form
 * of RTC_I2C_ADDR (>>1) during the scan. */
#define EE_ADDR_BASE   0x50U
#define EE_ADDR_LAST   0x57U
#ifndef RTC_I2C_ADDR
/* Fallback: PCF8563 8-bit write address (0x51 << 1). Define matches
 * rtc_manager.h; this guard only fires if that header isn't in scope here. */
#define RTC_I2C_ADDR   0xA2U
#endif
#define RTC_ADDR_7BIT  (RTC_I2C_ADDR >> 1)   /* 0xA2 >> 1 = 0x51 */

/* Address that the last EE_Probe() actually found, so EEW/EER target the real
 * chip rather than assuming 0x50. 0xFF = not yet probed / not present. */
static uint8_t s_ee_addr = 0xFFU;

/* ─── Bench-only commands ────────────────────────────────────────────────────
 * Define HYDRA_BENCH_CMDS (in project symbols, or here) to include the extra
 * bench-test commands: PIX, EEW, EER, TMPQ (raw/fault), DIAG. The always-on
 * presence checks (EE, RTCQ, TDSQ, BATQ) and all RGB/buzzer control remain
 * regardless. Leaving this UNDEFINED drops ~1 KB of flash for production.
 * To enable, add HYDRA_BENCH_CMDS to:
 *   Project ▸ Properties ▸ C/C++ Build ▸ Settings ▸ MCU GCC Compiler ▸
 *   Preprocessor ▸ Defined symbols (-D). */
/* #define HYDRA_BENCH_CMDS */

/* ─── I2C bus recovery ──────────────────────────────────────────────────────
 * Clocks out a stuck slave (SDA held low) before handing the bus to the RTC.
 * Called once from App_Init. Static so the compiler can discard it after use.*/
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
    /* STOP: SDA low -> SCL high -> SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   HAL_Delay(1);
    /* Restore I2C AF */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
    HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(hi2c, 0);
    HAL_Delay(5);
}

/* ─── EEPROM (M24512) probe helper ──────────────────────────────────────────
 * Scans 0x50..0x57 with a short timeout, SKIPPING the RTC's address so the
 * PCF8563 can't be mistaken for an EEPROM. Returns 1 and writes the 7-bit
 * address that acked into *found_addr (and caches it in s_ee_addr); returns 0
 * if nothing responded. */
static uint8_t EE_Probe(uint8_t *found_addr)
{
    if (s_app_i2c == NULL) return 0;
    for (uint8_t a = EE_ADDR_BASE; a <= EE_ADDR_LAST; a++) {
        if (a == RTC_ADDR_7BIT) continue;   /* never probe the RTC */
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
/* Return the 8-bit HAL address of the EEPROM, probing once if not yet known.
 * Returns 0 if no EEPROM is present. Used only by the bench EEW/EER commands. */
static uint16_t EE_Addr8(void)
{
    if (s_ee_addr == 0xFFU) {
        uint8_t a;
        if (!EE_Probe(&a)) return 0;
    }
    return (uint16_t)(s_ee_addr << 1);
}
#endif

/* ─── Init ──────────────────────────────────────────────────── */
void App_Init(AppContext_t *app,
              ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              UART_HandleTypeDef *huart,
              TIM_HandleTypeDef  *htim_ws,
              TIM_HandleTypeDef  *htim_buz)
{
    memset(app, 0, sizeof(AppContext_t));

    /* Capture the I2C handle so the BLE EEPROM/RTC probe commands can use it. */
    s_app_i2c = hi2c;

    WS2812B_Init(htim_ws);
    Buzzer_Init(htim_buz);
    HX711_Init(&app->hx711);
    TDS_Init(&app->tds, hadc);
    NTC_Init(&app->ntc, hadc);
    Battery_Init(&app->bat, hadc);
    /* BMA253_Init(&app->bma, hi2c);   // IMU disabled: not working and its
     *   blocking I2C init was stalling boot before BLE_Init, so Bluetooth
     *   never started. Drink detection falls back to weight-only (see
     *   App_CheckDrinkEvent). Re-enable once the IMU/I2C is sorted. */
    BLE_Init(&app->ble, huart);

    /* Recover I2C bus (previous BMA253 attempts may have left SDA stuck low)
     * then try to init the RTC. If the PCF8563 doesn't respond, keep going
     * with unix_approx=0 and disable the EXTI so a floating INT# doesn't
     * hammer the ISR. Time can be set later via the SETTIME BLE command. */
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

    /* Service the PCF8563 1 Hz tick — only if RTC actually initialized. */
    if (app->rtc.initialized && RTC_PopTick(&app->rtc)) {
        RTC_ClearTimerFlag(&app->rtc);
        if ((app->rtc.unix_approx % 60UL) == 0UL) {
            RTC_Read(&app->rtc);
        }
    }

    /* Process incoming BLE commands EVERY loop for fast, responsive replies
     * (the old code only polled once a second). Also finalize any ASCII line
     * that arrived without a CR/LF terminator after a short idle gap. */
    BLE_IdleFlush(&app->ble, 150U);
    App_ServiceBLE(app);

    if (now - app->last_weight_ms  >= APP_WEIGHT_POLL_MS)  App_TaskWeight(app);
    if (now - app->last_tds_ms     >= APP_TDS_POLL_MS)     App_TaskTDS(app);
    if (now - app->last_temp_ms    >= APP_TEMP_POLL_MS)    App_TaskTemp(app);
    if (now - app->last_battery_ms >= APP_BATTERY_POLL_MS) App_TaskBattery(app);
    if (now - app->last_ble_poll_ms>= APP_BLE_STATE_POLL_MS) App_TaskBLE(app);
    if (now - app->last_reminder_ms>= APP_REMINDER_CHECK_MS) App_TaskReminder(app);
    if (now - app->last_button_ms  >= APP_BUTTON_POLL_MS)    App_TaskButton(app);

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
    /* Weight-only drink detection (IMU disabled).
     *
     * Idea: hold a "baseline" of the bottle weight while it is stable. When the
     * weight drops (water removed) and then settles at a new, lower stable
     * level, the difference baseline-new is the amount consumed. A refill
     * (weight goes UP) just raises the baseline and is never counted as a drink.
     *
     * "Stable" = consecutive readings within DRINK_STABLE_BAND_G of each other
     * for DRINK_SETTLE_MS. App_TaskWeight runs every APP_WEIGHT_POLL_MS. */

    float w   = app->current_weight_g;
    uint32_t now = HAL_GetTick();

    /* First valid reading: seed the baseline and last-sample tracking. */
    if (!app->weight_seeded) {
        app->weight_seeded     = 1;
        app->drink_baseline_g  = w;
        app->last_stable_w_g   = w;
        app->stable_since_ms   = now;
        return;
    }

    /* Track stability: if this sample is close to the last one, the bottle is
     * settling; otherwise restart the settle timer at the new level. */
    float jitter = w - app->last_stable_w_g;
    if (jitter < 0) jitter = -jitter;

    if (jitter <= (float)DRINK_STABLE_BAND_G) {
        /* still close to last sample — let the settle timer keep running */
    } else {
        /* moved — reset the settle window to this new level */
        app->last_stable_w_g = w;
        app->stable_since_ms = now;
        return;
    }
    app->last_stable_w_g = w;

    /* Not yet stable long enough — wait. */
    if (now - app->stable_since_ms < DRINK_SETTLE_MS) return;

    /* We have a new stable level. Compare against baseline. */
    float delta = app->drink_baseline_g - w;   /* positive = water removed */

    if (delta >= (float)DRINK_MIN_VOLUME_ML) {
        /* A real decrease that has settled → a drink. */
        App_RecordDrinkEvent(app, delta);
        app->drink_baseline_g = w;              /* new baseline after drinking */
    } else if (w > app->drink_baseline_g + (float)DRINK_STABLE_BAND_G) {
        /* Refill (or bottle replaced heavier) — raise baseline, not a drink. */
        app->drink_baseline_g = w;
    }
    /* small noise within the band: leave baseline as-is */
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
/* Handle any pending BLE input (binary packet or ASCII line). Called every
 * loop from App_Run so replies go out promptly. */
void App_ServiceBLE(AppContext_t *app)
{
    BLE_Packet_t pkt;
    if (BLE_GetPacket(&app->ble, &pkt)) {
        App_HandleBLECommand(app, &pkt);
    }

    char line[BLE_STR_LINE_MAX];
    if (BLE_GetLine(&app->ble, line)) {
        App_HandleStringCommand(app, line);
    }
}

void App_TaskBLE(AppContext_t *app)
{
    app->last_ble_poll_ms = HAL_GetTick();

    /* Connection state + log flush only — command handling now happens every
     * loop in App_ServiceBLE(). */
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

/* ─── Physical button (PRD §8 reset spec) ───────────────────────────────────
 * Active-LOW button on PB3. Long-press handling with a confirmation window:
 *   • Press & hold ≥10 s        → arm factory-reset warning (red flash + beeps)
 *   • Keep holding ≥5 s more     → factory reset executes (data wiped)
 *   • Release during the window  → cancelled, returns to normal
 * Short presses simply wake / acknowledge (no destructive action).
 * --------------------------------------------------------------------------*/
void App_TaskButton(AppContext_t *app)
{
    app->last_button_ms = HAL_GetTick();

    uint8_t down = (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    uint32_t now = HAL_GetTick();

    if (down && !app->btn_was_down) {
        /* edge: press started */
        app->btn_was_down   = 1;
        app->btn_down_ms     = now;
        app->btn_reset_armed = 0;
    } else if (down && app->btn_was_down) {
        uint32_t held = now - app->btn_down_ms;

        if (!app->btn_reset_armed && held >= BTN_VLONG_PRESS_MS) {
            /* 10 s reached — show the warning and start the confirm window */
            app->btn_reset_armed = 1;
            app->btn_down_ms     = now;     /* reuse as confirm-window start */
            WS2812B_SetPattern(LED_PATTERN_FACTORY_RESET_WARN);
            Buzzer_Play(BUZZER_FACTORY_RESET);
        } else if (app->btn_reset_armed && held >= BTN_RESET_CONFIRM_MS) {
            /* still held through the whole confirmation window — wipe */
            App_Cmd_FactoryReset(app);      /* performs reset + system reset */
        }
    } else if (!down && app->btn_was_down) {
        /* released */
        if (app->btn_reset_armed) {
            /* cancelled before window elapsed — restore normal indication */
            WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
            Buzzer_Stop();
        }
        app->btn_was_down   = 0;
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
 *   ── Status / telemetry ──
 *   PING            -> PONG
 *   STATUS          -> S:bat,tmp,tds,ml,chg
 *   TEMP            -> T:<c.c>     (current water temperature)
 *   TDS             -> D:<ppm>     (current purity, cached)
 *   WEIGHT          -> W:<grams>
 *   READ            -> TLM,temp_x10,tds,ml,score
 *   TIME            -> T=<unix> h:m:s (+/-)
 *   SETTIME,<unix>  -> OK/ERR
 *
 *   ── RGB LED control ──
 *   RED/GREEN/BLUE/WHITE/OFF          -> solid LED colours
 *   RGB,r,g,b       -> all LEDs to r,g,b
 *   PIX,i,r,g,b     -> set single pixel i then refresh   (bench only)
 *   PAT,<0-14>      -> set any animated LED pattern (see table below)
 *   CHG,<pct>       -> set charging-bar level + show CHARGING_BAR pattern
 *   LAMP,r,g,b      -> set lamp colour + enter lamp mode (saved to settings)
 *   LAMPOFF         -> leave lamp mode, LEDs back to idle
 *
 *   ── Buzzer control ──
 *   BEEP            -> single confirm beep
 *   BON / BOFF      -> double-beep pattern / stop
 *   BUZ,<0-8>       -> play any buzzer pattern (see table below)
 *
 *   ── Device control ──
 *   REG             -> mark device registered (plays cue)
 *   UNPAIR          -> clear registration / user
 *   SYNC            -> mark logged events synced + sync animation
 *   SOFTRST         -> software restart (data preserved)
 *   CFG             -> dump stored prefs: C:purity,temp,hyd,win,freq
 *   CAL / TARE      -> start empty-bottle calibration (tare)
 *
 *   ── Diagnostics ──
 *   EE              -> EE:1,<addr> / EE:0          (M24512 EEPROM scan)
 *   RTCQ            -> R:1,<unix>,h:m:s / R:0
 *   TDSQ            -> DQ:<ppm>,<valid>
 *   TMPQ            -> TQ:<temp_x10>,<valid>,<raw_adc>,<fault>
 *   BATQ            -> BQ:<pct>,<charging>,<full>,<low>
 *   EEW,adr,byte    -> OK/ERR  (bench only — write one byte)
 *   EER,adr         -> EER:<byte> / ERR  (bench only — read one byte)
 *   DIAG            -> DG:ee,rtc,tds,temp,bat  (bench only)
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
    /* Shared response strings — defined once so the linker emits a single
     * copy in .rodata instead of one per call site (saves ~70 bytes). */
    static const char OK[]  = "OK\r\n";
    static const char ERR[] = "ERR\r\n";

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

    if (strcmp(up, "HELP") == 0 || strcmp(up, "?") == 0) {
        BLE_SendStr(&app->ble, "=HydraSense cmds=\r\n");
        BLE_SendStr(&app->ble, "RGB,r,g,b RED GREEN BLUE WHITE OFF\r\n");
        BLE_SendStr(&app->ble, "PAT,0-14 CHG,pct LAMP,r,g,b LAMPON LAMPOFF\r\n");
        BLE_SendStr(&app->ble, "BEEP BON BOFF BUZ,0-8\r\n");
        BLE_SendStr(&app->ble, "STATUS TEMP TDS WEIGHT READ CFG\r\n");
        BLE_SendStr(&app->ble, "TIME SETTIME,unix CAL/TARE\r\n");
        BLE_SendStr(&app->ble, "REG UNPAIR SYNC RESET REBOOT SOFTRST\r\n");
        BLE_SendStr(&app->ble, "EE RTCQ TDSQ TMPQ BATQ\r\n");
#ifdef HYDRA_BENCH_CMDS
        BLE_SendStr(&app->ble, "PIX,i,r,g,b EER,a EEW,a,b DIAG\r\n");
#endif
        return;
    }

    /* TIME — show unix + h:m:s + init flag */
    if (strcmp(up, "TIME") == 0) {
        RTC_Read(&app->rtc);
        char out2[48]; char *e2 = out2 + sizeof(out2); char *p2 = out2;
        p2 = s_app(p2, e2, "T=");  p2 = s_u2s(p2, e2, app->rtc.unix_approx);
        p2 = s_app(p2, e2, " ");
        p2 = s_u2s(p2, e2, app->rtc.now.hours);   *p2++ = ':';
        p2 = s_u2s(p2, e2, app->rtc.now.minutes); *p2++ = ':';
        p2 = s_u2s(p2, e2, app->rtc.now.seconds);
        *p2++ = (char)(app->rtc.initialized ? '+' : '-');  /* + = ok, - = no RTC */
        p2 = s_app(p2, e2, "\r\n");
        BLE_SendStr(&app->ble, out2);
        return;
    }

    /* SETTIME,<unix>  e.g. SETTIME,1748000000 */
    if (strcmp(up, "SETTIME") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] > 0) {
            RTC_SetFromUnix(&app->rtc, (uint32_t)v[0]);
            HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
            BLE_SendStr(&app->ble, "OK\r\n");
        } else {
            BLE_SendStr(&app->ble, "ERR\r\n");
        }
        return;
    }

    if (strcmp(up, "STATUS") == 0) {
        /* Compact: S:bat,tmp,tds,ml,chg */
        p = s_app(p, e, "S:");   p = s_u2s(p, e, app->current_bat_pct);
        *p++ = ',';              p = s_i2s(p, e, app->current_temp_x10);
        *p++ = ',';              p = s_u2s(p, e, app->current_tds_ppm);
        *p++ = ',';              p = s_u2s(p, e, app->consumed_today_ml);
        *p++ = ',';              p = s_u2s(p, e, Battery_IsCharging(&app->bat) ? 1U : 0U);
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
        p = s_app(p, e, "W:"); p = s_i2s(p, e, (int32_t)app->current_weight_g);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    if (strcmp(up, "READ") == 0) {
        p = s_app(p, e, "TLM,"); p = s_i2s(p, e, app->current_temp_x10);
        *p++ = ',';             p = s_u2s(p, e, app->current_tds_ppm);
        *p++ = ',';             p = s_u2s(p, e, app->consumed_today_ml);
        *p++ = ',';             p = s_u2s(p, e, app->hydration_score);
        p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out); return;
    }

    /* ════════════════════════════════════════════════════════════════════
     * RGB LED CONTROL
     * ════════════════════════════════════════════════════════════════════ */

    /* ---- Solid colours ---- */
    if (strcmp(up, "RED") == 0)   { WS2812B_SetAll(RGB_RED);   WS2812B_SendBlocking(); BLE_SendStr(&app->ble, OK); return; }
    if (strcmp(up, "GREEN") == 0) { WS2812B_SetAll(RGB_GREEN); WS2812B_SendBlocking(); BLE_SendStr(&app->ble, OK); return; }
    if (strcmp(up, "BLUE") == 0)  { WS2812B_SetAll(RGB_BLUE);  WS2812B_SendBlocking(); BLE_SendStr(&app->ble, OK); return; }
    if (strcmp(up, "WHITE") == 0) { WS2812B_SetAll(RGB(255,255,255)); WS2812B_SendBlocking(); BLE_SendStr(&app->ble, OK); return; }
    if (strcmp(up, "OFF") == 0)   {
        app->purity_alert_active = 0;
        app->temp_alert_active   = 0;
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF); WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking(); BLE_SendStr(&app->ble, OK); return;
    }

    /* CLR — clear any latched alert/error state and force LEDs to idle.
     * Use this to escape a purity/temp/error indication during testing.
     * Note: if the underlying condition (e.g. water still above purity goal)
     * persists, the next sensor poll may re-raise it — that is correct. */
    if (strcmp(up, "CLR") == 0) {
        app->purity_alert_active = 0;
        app->temp_alert_active   = 0;
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
        WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking();
        Buzzer_Stop();
        BLE_SendStr(&app->ble, OK);
        return;
    }

    if (strcmp(up, "RGB") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetAll(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SendBlocking();
            BLE_SendStr(&app->ble, OK);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }

#ifdef HYDRA_BENCH_CMDS
    /* PIX,<idx>,<r>,<g>,<b>  — set a single LED then push the frame. */
    if (strcmp(up, "PIX") == 0) {
        int v[4];
        if (s_csv(line, v, 4) == 4 &&
            v[0] >= 0 && v[0] < WS2812B_NUM_LEDS &&
            v[1] >= 0 && v[1] <= 255 && v[2] >= 0 && v[2] <= 255 && v[3] >= 0 && v[3] <= 255) {
            WS2812B_SetPixel((uint8_t)v[0],
                             RGB((uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3]));
            WS2812B_SendBlocking();
            BLE_SendStr(&app->ble, OK);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }
#endif /* HYDRA_BENCH_CMDS — PIX */

    /* PAT,<n>  — set any animated WS2812B pattern.
     *   0 OFF             5 TEMP_ALERT       10 LOW_BATTERY
     *   1 HYDRATION_HIGH  6 CALIBRATION      11 LAMP_MODE
     *   2 HYDRATION_MID   7 DRINK_CONFIRM    12 REGISTRATION
     *   3 HYDRATION_LOW   8 SYNC_SUCCESS     13 FACTORY_RESET_WARN
     *   4 PURITY_ALERT    9 CHARGING_BAR     14 ERROR                     */
    if (strcmp(up, "PAT") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 14) {
            static const LED_Pattern_t map[] = {
                LED_PATTERN_ALL_OFF,            /* 0  */
                LED_PATTERN_HYDRATION_HIGH,     /* 1  */
                LED_PATTERN_HYDRATION_MID,      /* 2  */
                LED_PATTERN_HYDRATION_LOW,      /* 3  */
                LED_PATTERN_PURITY_ALERT,       /* 4  */
                LED_PATTERN_TEMP_ALERT,         /* 5  */
                LED_PATTERN_CALIBRATION,        /* 6  */
                LED_PATTERN_DRINK_CONFIRM,      /* 7  */
                LED_PATTERN_SYNC_SUCCESS,       /* 8  */
                LED_PATTERN_CHARGING_BAR,       /* 9  */
                LED_PATTERN_LOW_BATTERY,        /* 10 */
                LED_PATTERN_LAMP_MODE,          /* 11 */
                LED_PATTERN_REGISTRATION,       /* 12 */
                LED_PATTERN_FACTORY_RESET_WARN, /* 13 */
                LED_PATTERN_ERROR,              /* 14 */
            };
            WS2812B_SetPattern(map[v[0]]);
            BLE_SendStr(&app->ble, OK);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }

    /* CHG,<pct>  — set the charging-bar level (0..100) and show it. */
    if (strcmp(up, "CHG") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 100) {
            WS2812B_SetChargingLevel((uint8_t)v[0]);
            WS2812B_SetPattern(LED_PATTERN_CHARGING_BAR);
            BLE_SendStr(&app->ble, OK);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }

    /* LAMP,r,g,b — set lamp colour and enter lamp mode (river animation).
     * Applies INSTANTLY — does not write flash on every call (a flash erase on
     * each colour change was slow and could stall). Use LAMPSAVE to persist
     * the current colour. The string command does NOT require the charger so
     * you can test the animation any time; the binary LAMP_MODE still does. */
    if (strcmp(up, "LAMP") == 0) {
        int v[3];
        if (s_csv(line, v, 3) == 3 &&
            v[0]>=0 && v[0]<=255 && v[1]>=0 && v[1]<=255 && v[2]>=0 && v[2]<=255) {
            WS2812B_SetLampColor(RGB((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]));
            WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
            Device_PostEvent(&app->dev, EVT_CMD_LAMP_ON);
            BLE_SendStr(&app->ble, OK);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }

    /* LAMPON — re-enter lamp mode using the last colour (no args). */
    if (strcmp(up, "LAMPON") == 0) {
        WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
        Device_PostEvent(&app->dev, EVT_CMD_LAMP_ON);
        BLE_SendStr(&app->ble, OK);
        return;
    }

    /* LAMPSAVE — persist the current lamp colour to settings (one flash write).
     * Note: we cannot read back the live colour from the driver, so the app
     * stores it from the last LAMP command via prefs only if you choose to. */
    if (strcmp(up, "LAMPSAVE") == 0) {
        Storage_SaveSettings(&app->settings);
        BLE_SendStr(&app->ble, OK);
        return;
    }

    /* LAMPOFF — leave lamp mode and return LEDs to normal/idle. */
    if (strcmp(up, "LAMPOFF") == 0) {
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
        WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking();
        Device_PostEvent(&app->dev, EVT_CMD_LAMP_OFF);
        BLE_SendStr(&app->ble, OK);
        return;
    }

    /* RESET — clear ANY stuck LED pattern (including ERROR / alerts) and return
     * to a clean idle state. Use this to get out of error mode without a reboot.
     * Also clears the latched alert flags so they can re-trigger cleanly. */
    if (strcmp(up, "RESET") == 0) {
        app->purity_alert_active = 0;
        app->temp_alert_active   = 0;
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
        WS2812B_SetAll(RGB_OFF);
        WS2812B_SendBlocking();
        Buzzer_Stop();
        BLE_SendStr(&app->ble, OK);
        return;
    }

    /* REBOOT — full hardware reset (settings/logs preserved). The ultimate
     * "get unstuck". Replies first, then resets. */
    if (strcmp(up, "REBOOT") == 0) {
        BLE_SendStr(&app->ble, OK);
        HAL_Delay(150);
        HAL_NVIC_SystemReset();
    }

    /* ════════════════════════════════════════════════════════════════════
     * BUZZER CONTROL
     * ════════════════════════════════════════════════════════════════════ */

    if (strcmp(up, "BEEP") == 0) { Buzzer_Play(BUZZER_SINGLE_BEEP); BLE_SendStr(&app->ble, OK); return; }
    if (strcmp(up, "BON")  == 0) { Buzzer_Play(BUZZER_DOUBLE_BEEP); BLE_SendStr(&app->ble, OK); return; }
    if (strcmp(up, "BOFF") == 0) { Buzzer_Stop();                   BLE_SendStr(&app->ble, OK); return; }

    /* BUZ,<n>  — play any buzzer pattern.
     *   0 STARTUP          5 CALIBRATION_DONE
     *   1 SINGLE_BEEP      6 REGISTRATION_OK
     *   2 DOUBLE_BEEP      7 SYNC_OK
     *   3 PURITY_ALERT     8 FACTORY_RESET
     *   4 TEMP_ALERT
     * A switch is used (not a typed array) so this does not depend on the
     * exact enum tag name in buzzer.h — only on the constants, which are
     * already used throughout this file. */
    if (strcmp(up, "BUZ") == 0) {
        int v[1];
        if (s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 8) {
            switch (v[0]) {
            case 0: Buzzer_Play(BUZZER_STARTUP);          break;
            case 1: Buzzer_Play(BUZZER_SINGLE_BEEP);      break;
            case 2: Buzzer_Play(BUZZER_DOUBLE_BEEP);      break;
            case 3: Buzzer_Play(BUZZER_PURITY_ALERT);     break;
            case 4: Buzzer_Play(BUZZER_TEMP_ALERT);       break;
            case 5: Buzzer_Play(BUZZER_CALIBRATION_DONE); break;
            case 6: Buzzer_Play(BUZZER_REGISTRATION_OK);  break;
            case 7: Buzzer_Play(BUZZER_SYNC_OK);          break;
            case 8: Buzzer_Play(BUZZER_FACTORY_RESET);    break;
            }
            BLE_SendStr(&app->ble, OK);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }

    /* ════════════════════════════════════════════════════════════════════
     * DIAGNOSTICS — EEPROM / RTC / TDS / TEMP / BATTERY
     * ════════════════════════════════════════════════════════════════════ */

    /* EE — scan the M24512 EEPROM on I2C1, skipping the RTC.
     *   EE:1,80  (addr in decimal)  /  EE:0  not present */
    if (strcmp(up, "EE") == 0) {
        uint8_t addr = 0;
        p = s_app(p, e, "EE:");
        if (EE_Probe(&addr)) {
            *p++ = '1'; *p++ = ',';
            p = s_u2s(p, e, addr);   /* 7-bit addr in decimal (e.g. 80=0x50) */
        } else {
            *p++ = '0';
        }
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

#ifdef HYDRA_BENCH_CMDS
    /* EEW,<addr>,<byte> — write one byte to M24512 memory address (16-bit).
     * Targets the address EE_Probe found (s_ee_addr), not a hard-coded 0x50. */
    if (strcmp(up, "EEW") == 0) {
        int v[2];
        uint16_t a8 = EE_Addr8();
        if (a8 && s_csv(line, v, 2) == 2 &&
            v[0] >= 0 && v[0] <= 0xFFFF && v[1] >= 0 && v[1] <= 0xFF) {
            uint8_t buf3[3] = { (uint8_t)(v[0] >> 8), (uint8_t)(v[0] & 0xFF),
                                (uint8_t)v[1] };
            HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
                s_app_i2c, a8, buf3, 3, 20);
            HAL_Delay(6);   /* M24512 write cycle time (max ~5 ms) */
            BLE_SendStr(&app->ble, (st == HAL_OK) ? OK : ERR);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }

    /* EER,<addr> — read one byte from M24512 memory address (16-bit).
     * Targets the address EE_Probe found (s_ee_addr), not a hard-coded 0x50. */
    if (strcmp(up, "EER") == 0) {
        int v[1];
        uint16_t a8 = EE_Addr8();
        if (a8 && s_csv(line, v, 1) == 1 && v[0] >= 0 && v[0] <= 0xFFFF) {
            uint8_t ad[2] = { (uint8_t)(v[0] >> 8), (uint8_t)(v[0] & 0xFF) };
            uint8_t rb = 0;
            HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
                s_app_i2c, a8, ad, 2, 20);
            if (st == HAL_OK) {
                st = HAL_I2C_Master_Receive(
                    s_app_i2c, a8, &rb, 1, 20);
            }
            if (st == HAL_OK) {
                p = s_app(p, e, "EER:"); p = s_u2s(p, e, rb);
                p = s_app(p, e, "\r\n"); BLE_SendStr(&app->ble, out);
            } else BLE_SendStr(&app->ble, ERR);
        } else BLE_SendStr(&app->ble, ERR);
        return;
    }
#endif /* HYDRA_BENCH_CMDS — EEW/EER */

    /* RTCQ — RTC presence + current time. */
    if (strcmp(up, "RTCQ") == 0) {
        p = s_app(p, e, "R:");
        if (app->rtc.initialized) {
            RTC_Read(&app->rtc);
            *p++ = '1'; *p++ = ',';
            p = s_u2s(p, e, app->rtc.unix_approx);  *p++ = ',';
            p = s_u2s(p, e, app->rtc.now.hours);    *p++ = ':';
            p = s_u2s(p, e, app->rtc.now.minutes);  *p++ = ':';
            p = s_u2s(p, e, app->rtc.now.seconds);
        } else {
            *p++ = '0';
        }
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    /* TDSQ — force a fresh TDS read + validity flag. */
    if (strcmp(up, "TDSQ") == 0) {
        uint16_t ppm = TDS_ReadPPM(&app->tds, app->current_temp_x10);
        app->current_tds_ppm = ppm;
        p = s_app(p, e, "DQ:"); p = s_u2s(p, e, ppm);
        *p++ = ',';             p = s_u2s(p, e, app->tds.valid);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    /* TMPQ — fresh NTC read + raw ADC + fault code. */
    if (strcmp(up, "TMPQ") == 0) {
        int16_t t = NTC_ReadTemp_x10(&app->ntc);
        app->current_temp_x10 = t;
        p = s_app(p, e, "TQ:"); p = s_i2s(p, e, t);
        *p++ = ',';             p = s_u2s(p, e, app->ntc.valid);
        *p++ = ',';             p = s_u2s(p, e, g_ntc_adc_avg);
        *p++ = ',';             p = s_u2s(p, e, g_ntc_fault);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    /* BATQ — fresh battery snapshot: pct,charging,full,low. */
    if (strcmp(up, "BATQ") == 0) {
        Battery_Update(&app->bat);
        app->current_bat_pct = Battery_GetPercent(&app->bat);
        p = s_app(p, e, "BQ:"); p = s_u2s(p, e, app->current_bat_pct);
        *p++ = ',';             p = s_u2s(p, e, Battery_IsCharging(&app->bat) ? 1U : 0U);
        *p++ = ',';             p = s_u2s(p, e, Battery_IsFull(&app->bat)     ? 1U : 0U);
        *p++ = ',';             p = s_u2s(p, e, Battery_IsLow(&app->bat)      ? 1U : 0U);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

#ifdef HYDRA_BENCH_CMDS
    /* DIAG — one compact line: EE,RTC,TDS,T,B all at once.
     * Redundant with EE/RTCQ/TDSQ/BATQ; bench convenience only. */
    if (strcmp(up, "DIAG") == 0) {
        uint8_t  ee  = EE_Probe(NULL);
        uint16_t ppm = TDS_ReadPPM(&app->tds, app->current_temp_x10);
        int16_t  t   = NTC_ReadTemp_x10(&app->ntc);
        Battery_Update(&app->bat);
        p = s_app(p, e, "DG:");
        p = s_u2s(p, e, ee);                       *p++ = ',';
        p = s_u2s(p, e, app->rtc.initialized);     *p++ = ',';
        p = s_u2s(p, e, ppm);                       *p++ = ',';
        p = s_i2s(p, e, t);                         *p++ = ',';
        p = s_u2s(p, e, Battery_GetPercent(&app->bat));
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }
#endif /* HYDRA_BENCH_CMDS — DIAG */

    /* ════════════════════════════════════════════════════════════════════
     * DEVICE CONTROL — registration / pairing / reset / sync / config
     * ════════════════════════════════════════════════════════════════════ */

    /* REG — register the device (marks it paired). Mirrors REGISTER_DEVICE.
     * No user_id needed over the test channel; flags registered + plays cue. */
    if (strcmp(up, "REG") == 0) {
        app->settings.is_registered = 1;
        Storage_SaveSettings(&app->settings);
        Device_PostEvent(&app->dev, EVT_CMD_REGISTER);
        Buzzer_Play(BUZZER_REGISTRATION_OK);
        BLE_SendStr(&app->ble, OK);
        return;
    }

    /* UNPAIR — clear registration / user, back to unregistered. */
    if (strcmp(up, "UNPAIR") == 0) {
        memset(app->settings.user_id, 0, sizeof(app->settings.user_id));
        app->settings.is_registered = 0;
        Storage_SaveSettings(&app->settings);
        BLE_SendStr(&app->ble, OK);
        return;
    }

    /* SOFTRST — software restart (data/settings preserved). */
    if (strcmp(up, "SOFTRST") == 0) {
        BLE_SendStr(&app->ble, OK);
        HAL_Delay(200);
        HAL_NVIC_SystemReset();
    }

    /* SYNC — mark all logged events up to now as synced (like SYNC_ACK). */
    if (strcmp(up, "SYNC") == 0) {
        Storage_MarkSynced(&app->drink_log, app->rtc.unix_approx);
        Storage_FlushDrinkLog(&app->drink_log);
        WS2812B_SetPattern(LED_PATTERN_SYNC_SUCCESS);
        Buzzer_Play(BUZZER_SYNC_OK);
        BLE_SendStr(&app->ble, OK);
        return;
    }

    /* CFG — dump the current stored preferences as a compact line:
     * C:purity,temp_x10,hydration,h0:m0-h1:m1,freq,score. */
    if (strcmp(up, "CFG") == 0) {
        p = s_app(p, e, "C:");
        p = s_u2s(p, e, BLE_U16(app->settings.prefs.purity_goal_hi,
                                 app->settings.prefs.purity_goal_lo));   *p++ = ',';
        p = s_i2s(p, e, BLE_I16(app->settings.prefs.temp_goal_hi,
                                 app->settings.prefs.temp_goal_lo));      *p++ = ',';
        p = s_u2s(p, e, BLE_U16(app->settings.prefs.hydration_hi,
                                 app->settings.prefs.hydration_lo));      *p++ = ',';
        p = s_u2s(p, e, app->settings.prefs.remind_h_start); *p++ = ':';
        p = s_u2s(p, e, app->settings.prefs.remind_m_start); *p++ = '-';
        p = s_u2s(p, e, app->settings.prefs.remind_h_end);   *p++ = ':';
        p = s_u2s(p, e, app->settings.prefs.remind_m_end);   *p++ = ',';
        p = s_u2s(p, e, app->settings.prefs.remind_freq_min);
        p = s_app(p, e, "\r\n");
        BLE_SendStr(&app->ble, out);
        return;
    }

    /* ---- Calibration (empty-bottle tare) ----
     * The HX711 read can BLOCK forever if the chip isn't responding (DOUT
     * stays HIGH = never data-ready). That would hang the whole firmware and
     * look like "stuck in error mode". So we first poll DOUT for a short
     * window; if it never goes LOW (data ready), the HX711 is not talking —
     * reply ERR instead of calling the blocking tare. */
    if (strcmp(up, "CAL") == 0 || strcmp(up, "TARE") == 0) {
        /* Wait up to ~400 ms for HX711 data-ready (DOUT LOW). */
        uint32_t t0 = HAL_GetTick();
        uint8_t ready = 0;
        while ((HAL_GetTick() - t0) < 400U) {
            if (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_RESET) {
                ready = 1;
                break;
            }
        }
        if (!ready) {
            /* HX711 not responding — do NOT call the blocking tare. */
            BLE_SendStr(&app->ble, "ERR,HX711\r\n");
            return;
        }
        BLE_Packet_t fake;
        fake.cmd = BLE_CMD_CALIBRATION;
        fake.len = 1;
        fake.payload[0] = 0;
        App_Cmd_Calibration(app, &fake);
        BLE_SendStr(&app->ble, OK);
        return;
    }

    BLE_SendStr(&app->ble, ERR);
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

    /* Bounded HX711 presence check: the chip pulls DOUT LOW when a sample is
     * ready. If it never does within ~400 ms it is absent/dead — do NOT call
     * the blocking HX711_Tare (which would hang the whole device forever). */
    uint32_t t0 = HAL_GetTick();
    uint8_t  ready = 0;
    while ((HAL_GetTick() - t0) < 400U) {
        if (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_RESET) {
            ready = 1;
            break;
        }
    }
    if (!ready) {
        /* Leave LEDs in a clean idle state and report the fault. */
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
        App_SendACK(app, pkt->cmd, 0, BLE_ERR_OK);   /* success=0 → failed */
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
    /* Return LEDs to idle so the device doesn't sit in the calibration wave. */
    WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
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
