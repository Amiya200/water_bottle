#ifndef APP_LOGIC_H
#define APP_LOGIC_H

#include "stm32f0xx_hal.h"
#include "ble_protocol.h"
#include <stdint.h>

/* ─── Sensor poll intervals ─────────────────────────────────── */
#define APP_WEIGHT_POLL_MS       200U
#define APP_TDS_POLL_MS         5000U
#define APP_TEMP_POLL_MS        5000U
#define APP_BATTERY_POLL_MS    10000U
#define APP_BLE_STATE_POLL_MS   1000U
#define APP_REMINDER_CHECK_MS  60000U
#define APP_BUTTON_POLL_MS        50U

/* ─── Physical button ───────────────────────────────────────── */
/* FRD §8.3 semantics:
 * release < 3 s            → short press: wake / status / confirm
 * 3 s ≤ release < 10 s     → long press: power on/off (soft)
 * hold ≥ 10 s              → factory-reset warning (5 s, cancellable)
 * keep holding +5 s        → factory reset executes
 */
#define BTN_LONG_PRESS_MS       3000U
#define BTN_VLONG_PRESS_MS     10000U
#define BTN_RESET_CONFIRM_MS    5000U

/* App-triggered factory reset: non-blocking 5 s warning window (FRD §8.2) */
#define APP_FRESET_WARN_MS      5000U

/* Internal error log (FRD ERROR_LOG) */
#define APP_ERRLOG_DEPTH        6U
#define APP_ERR_HX711           1U
#define APP_ERR_RTC             2U
#define APP_ERR_EEPROM          3U
#define APP_ERR_TDS             4U
#define APP_ERR_NTC             5U

/* Preference validation limits (FRD ACK error example) */
#define PREF_PURITY_MAX_PPM     1500U   /* "must be between 0 and 1500" */
#define PREF_TEMP_MAX_X10       1000    /* 100.0 °C                      */
#define PREF_HYDRATION_MAX_ML   10000U

/* ─── Drink detection ───────────────────────────────────────── */
#define DRINK_MIN_VOLUME_ML     10U
#define DRINK_SETTLE_MS         3000U
#define DRINK_WEIGHT_STABLE_MS   500U
#define DRINK_STABLE_BAND_G        3U

/* ─── Hydration score thresholds ────────────────────────────── */
#define HYDRATION_SCORE_HIGH    80
#define HYDRATION_SCORE_MID     50

/*
 * RAM-optimised app API
 * ---------------------
 * The old public app-context workflow was removed.  app_logic.c now
 * owns only the state it actually uses as file-scope static variables.  main.c
 * calls App_Init() once and App_Run() forever; interrupts are forwarded through
 * the small App_*ISR wrappers below.
 */
void App_Init(ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              UART_HandleTypeDef *huart,
              TIM_HandleTypeDef  *htim_ws,
              TIM_HandleTypeDef  *htim_buz);

void App_Run(void);

/* Interrupt forwarders used by main.c callbacks. */
void App_BLE_LinkISR(void);   /* PA15 BLE link-state edge (replaces IMU motion) */
void App_RTC_TickISR(void);
void App_BLE_RxISR(void);

/* Sensor/task entry points kept public for test builds. */
void App_TaskWeight(void);
void App_TaskTDS(void);
void App_TaskTemp(void);
void App_TaskBattery(void);
void App_TaskBLE(void);
void App_ServiceBLE(void);
void App_TaskReminder(void);
void App_TaskButton(void);
void App_TaskDailyRollup(void);

void    App_CheckDrinkEvent(void);
void    App_RecordDrinkEvent(uint16_t volume_ml);
uint8_t App_CalcHydrationScore(void);
void    App_ResetDailyConsumed(void);

void App_HandleBLECommand(const BLE_Packet_t *pkt);
void App_HandleStringCommand(char *line);

void App_Cmd_Timestamp(const BLE_Packet_t *pkt);
void App_Cmd_InputData(const BLE_Packet_t *pkt);
void App_Cmd_Calibration(const BLE_Packet_t *pkt);
void App_Cmd_LampMode(const BLE_Packet_t *pkt);
void App_Cmd_SoftReset(void);
void App_Cmd_FactoryReset(void);                          /* immediate wipe (button path)    */
void App_Cmd_FactoryResetRequest(const BLE_Packet_t *pkt);/* BLE path: 5 s warn, cancellable */
void App_Cmd_HistoricalAggregates(void);
void App_Cmd_RegisterDevice(const BLE_Packet_t *pkt);
void App_Cmd_UnpairDevice(void);
void App_Cmd_SensorLogs(const BLE_Packet_t *pkt);  /* optional from/to unix filter */
void App_Cmd_SyncAck(const BLE_Packet_t *pkt);
void App_Cmd_DeviceStatus(void);
void App_Cmd_GetConfig(void);
void App_Cmd_GetErrors(void);
void App_Cmd_Ping(void);
void App_Cmd_Measure(uint8_t force);
void App_Cmd_DumpEEPROM(void);
void App_Cmd_GetInfo(void);
void App_Cmd_FirmwareUpdate(const BLE_Packet_t *pkt);

/* PRD §3: auto-transfer of unsynced data when the app connects. */
void App_PushUnsyncedLogs(void);

/* Append to the internal error ring + show the error LED cue (FRD §5.12). */
void App_LogError(uint8_t code);

void App_SendACK(uint8_t cmd_id, uint8_t success, uint8_t err_code);

#endif /* APP_LOGIC_H */
