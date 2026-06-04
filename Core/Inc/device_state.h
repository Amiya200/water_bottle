#ifndef DEVICE_STATE_H
#define DEVICE_STATE_H

#include "stm32f0xx_hal.h"
#include "ws2812b.h"
#include "buzzer.h"
#include <stdint.h>

/* ─── Device states ─────────────────────────────────────────── */
typedef enum {
    DEV_STATE_BOOT = 0,
    DEV_STATE_UNREGISTERED,
    DEV_STATE_PAIRING,
    DEV_STATE_CALIBRATING,
    DEV_STATE_ACTIVE,
    DEV_STATE_LAMP_MODE,
    DEV_STATE_CHARGING,
    DEV_STATE_LOW_BATTERY,
    DEV_STATE_SOFT_RESET_PENDING,
    DEV_STATE_FACTORY_RESET_PENDING,
    DEV_STATE_ERROR,
} DeviceState_t;

/* ─── Events ────────────────────────────────────────────────── */
typedef enum {
    EVT_NONE = 0,
    EVT_BOOT_COMPLETE,
    EVT_BLE_CONNECTED,
    EVT_BLE_DISCONNECTED,
    EVT_CMD_REGISTER,
    EVT_CMD_CALIBRATION,
    EVT_CALIBRATION_DONE,
    EVT_CMD_LAMP_ON,
    EVT_CMD_LAMP_OFF,
    EVT_CMD_SOFT_RESET,
    EVT_CMD_FACTORY_RESET,
    EVT_FACTORY_RESET_CONFIRM,
    EVT_CHARGER_CONNECTED,
    EVT_CHARGER_DISCONNECTED,
    EVT_BATTERY_LOW,
    EVT_BATTERY_OK,
    EVT_SENSOR_ERROR,
    EVT_DRINK_DETECTED,
    EVT_REMINDER_DUE,
} DeviceEvent_t;

/* ─── Context ───────────────────────────────────────────────── */
#define DEV_EVENT_QUEUE_LEN  8U

typedef struct {
    uint8_t       state;                  /* DeviceState_t stored as byte    */
    uint8_t       pending_event;          /* DeviceEvent_t stored as byte    */
    uint8_t       evt_queue[DEV_EVENT_QUEUE_LEN];
    uint8_t       evt_head;
    uint8_t       evt_tail;
    uint32_t      state_entry_tick;
    uint32_t      last_low_bat_ms;
    uint8_t       lamp_mode_active;
    uint8_t       factory_reset_confirmed;
    uint8_t       error_code;
} DeviceContext_t;

/* ─── API ───────────────────────────────────────────────────── */
void          Device_Init(DeviceContext_t *ctx);
void          Device_PostEvent(DeviceContext_t *ctx, DeviceEvent_t evt);
void          Device_Run(DeviceContext_t *ctx);
DeviceState_t Device_GetState(const DeviceContext_t *ctx);

#endif /* DEVICE_STATE_H */
