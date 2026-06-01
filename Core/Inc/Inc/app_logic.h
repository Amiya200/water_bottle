#ifndef APP_LOGIC_H
#define APP_LOGIC_H

#include "stm32f0xx_hal.h"
#include "hx711.h"
#include "tds_sensor.h"
#include "ntc_temp.h"
#include "battery.h"
#include "bma253.h"
#include "ble_jdy29.h"
#include "ble_protocol.h"
#include "data_storage.h"
#include "rtc_manager.h"
#include "device_state.h"
#include <stdint.h>

/* ─── Sensor poll intervals ─────────────────────────────────── */
#define APP_WEIGHT_POLL_MS       200U
#define APP_TDS_POLL_MS         5000U
#define APP_TEMP_POLL_MS        5000U
#define APP_BATTERY_POLL_MS    10000U
#define APP_BLE_STATE_POLL_MS   1000U
#define APP_REMINDER_CHECK_MS  60000U

/* ─── Drink detection ──────────────────────────────────────── */
#define DRINK_MIN_VOLUME_ML     10U    /* ignore <10 ml changes   */
#define DRINK_SETTLE_MS         3000U
#define DRINK_WEIGHT_STABLE_MS   500U

/* ─── Hydration score thresholds ────────────────────────────── */
#define HYDRATION_SCORE_HIGH    80
#define HYDRATION_SCORE_MID     50

/* ─── App context ────────────────────────────────────────────── */
typedef struct {
    /* Handles */
    HX711_Handle_t    hx711;
    TDS_Handle_t      tds;
    NTC_Handle_t      ntc;
    Battery_Handle_t  bat;
    BMA253_Handle_t   bma;
    BLE_Handle_t      ble;
    RTC_Handle_t      rtc;
    DeviceContext_t   dev;

    /* Storage */
    DeviceSettings_t  settings;
    DrinkLog_t        drink_log;
    DailySummaryLog_t daily_log;

    /* Sensor readings — integer types, no float in context */
    float    current_weight_g;   /* HX711 still uses float internally */
    float    prev_weight_g;
    uint16_t current_tds_ppm;    /* was float — uint16 saves 2 bytes */
    int16_t  current_temp_x10;   /* °C × 10 — was float, saves 2 bytes */
    uint8_t  current_bat_pct;

    /* Timers */
    uint32_t last_weight_ms;
    uint32_t last_tds_ms;
    uint32_t last_temp_ms;
    uint32_t last_battery_ms;
    uint32_t last_ble_poll_ms;
    uint32_t last_reminder_ms;

    /* Drink event state machine */
    uint8_t  motion_pending;
    uint32_t motion_start_ms;
    float    weight_at_motion_g;

    /* Hydration */
    uint16_t consumed_today_ml;
    uint8_t  hydration_score;
    uint32_t current_day_unix;

    /* Flags */
    uint8_t  purity_alert_active;
    uint8_t  temp_alert_active;
    uint8_t  lamp_color_set;
} AppContext_t;

/* ─── API ───────────────────────────────────────────────────── */
void App_Init(AppContext_t *app,
              ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              UART_HandleTypeDef *huart,
              TIM_HandleTypeDef  *htim_ws,
              TIM_HandleTypeDef  *htim_buz);

void App_Run(AppContext_t *app);

/* Sensor tasks */
void App_TaskWeight(AppContext_t *app);
void App_TaskTDS(AppContext_t *app);
void App_TaskTemp(AppContext_t *app);
void App_TaskBattery(AppContext_t *app);
void App_TaskBLE(AppContext_t *app);
void App_TaskReminder(AppContext_t *app);
void App_TaskDailyRollup(AppContext_t *app);

/* Drink detection */
void App_CheckDrinkEvent(AppContext_t *app);
void App_RecordDrinkEvent(AppContext_t *app, float volume_ml);

/* Hydration */
uint8_t App_CalcHydrationScore(AppContext_t *app);
void    App_ResetDailyConsumed(AppContext_t *app);

/* BLE command dispatch — takes binary packet */
void App_HandleBLECommand(AppContext_t *app, const BLE_Packet_t *pkt);

/* Individual command handlers */
void App_Cmd_Timestamp(AppContext_t *app, const BLE_Packet_t *pkt);
void App_Cmd_InputData(AppContext_t *app, const BLE_Packet_t *pkt);
void App_Cmd_Calibration(AppContext_t *app, const BLE_Packet_t *pkt);
void App_Cmd_LampMode(AppContext_t *app, const BLE_Packet_t *pkt);
void App_Cmd_SoftReset(AppContext_t *app);
void App_Cmd_FactoryReset(AppContext_t *app);
void App_Cmd_HistoricalAggregates(AppContext_t *app);
void App_Cmd_RegisterDevice(AppContext_t *app, const BLE_Packet_t *pkt);
void App_Cmd_UnpairDevice(AppContext_t *app);
void App_Cmd_SensorLogs(AppContext_t *app);
void App_Cmd_SyncAck(AppContext_t *app, const BLE_Packet_t *pkt);
void App_Cmd_DeviceStatus(AppContext_t *app);
void App_Cmd_GetConfig(AppContext_t *app);
void App_Cmd_GetErrors(AppContext_t *app);
void App_Cmd_Ping(AppContext_t *app);

/* Utility */
void App_SendACK(AppContext_t *app, uint8_t cmd_id, uint8_t success, uint8_t err_code);

#endif /* APP_LOGIC_H */
