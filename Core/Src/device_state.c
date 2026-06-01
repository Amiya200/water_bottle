#pragma GCC optimize("Os")
#include "device_state.h"
#include <string.h>

static void OnEnterState(DeviceContext_t *ctx, DeviceState_t new_state)
{
    ctx->state            = new_state;
    ctx->state_entry_tick = HAL_GetTick();

    switch (new_state) {
    case DEV_STATE_UNREGISTERED:
        WS2812B_SetPattern(LED_PATTERN_REGISTRATION);
        Buzzer_Play(BUZZER_STARTUP);
        break;
    case DEV_STATE_PAIRING:
        WS2812B_SetPattern(LED_PATTERN_REGISTRATION);
        break;
    case DEV_STATE_CALIBRATING:
        WS2812B_SetPattern(LED_PATTERN_CALIBRATION);
        break;
    case DEV_STATE_ACTIVE:
        WS2812B_SetPattern(LED_PATTERN_ALL_OFF);
        Buzzer_Play(BUZZER_REGISTRATION_OK);
        break;
    case DEV_STATE_LAMP_MODE:
        WS2812B_SetPattern(LED_PATTERN_LAMP_MODE);
        break;
    case DEV_STATE_CHARGING:
        WS2812B_SetPattern(LED_PATTERN_CHARGING_BAR);
        break;
    case DEV_STATE_LOW_BATTERY:
        WS2812B_SetPattern(LED_PATTERN_LOW_BATTERY);
        Buzzer_Play(BUZZER_LOW_BATTERY);
        break;
    case DEV_STATE_FACTORY_RESET_PENDING:
        WS2812B_SetPattern(LED_PATTERN_FACTORY_RESET_WARN);
        Buzzer_Play(BUZZER_FACTORY_RESET);
        break;
    case DEV_STATE_ERROR:
        WS2812B_SetPattern(LED_PATTERN_ERROR);
        Buzzer_Play(BUZZER_ERROR);
        break;
    case DEV_STATE_SOFT_RESET_PENDING:
        HAL_NVIC_SystemReset();
        break;
    default:
        break;
    }
}

void Device_Init(DeviceContext_t *ctx)
{
    memset(ctx, 0, sizeof(DeviceContext_t));
    ctx->state         = DEV_STATE_BOOT;
    ctx->pending_event = EVT_NONE;
}

void Device_PostEvent(DeviceContext_t *ctx, DeviceEvent_t evt)
{
    ctx->pending_event = evt;
}

void Device_Run(DeviceContext_t *ctx)
{
    DeviceEvent_t evt = ctx->pending_event;
    ctx->pending_event = EVT_NONE;
    if (evt == EVT_NONE) return;

    DeviceState_t cur = ctx->state;

    switch (cur) {
    case DEV_STATE_BOOT:
        break;

    case DEV_STATE_UNREGISTERED:
        if (evt == EVT_CMD_REGISTER) OnEnterState(ctx, DEV_STATE_PAIRING);
        break;

    case DEV_STATE_PAIRING:
        if (evt == EVT_CALIBRATION_DONE) OnEnterState(ctx, DEV_STATE_ACTIVE);
        if (evt == EVT_CMD_CALIBRATION)  OnEnterState(ctx, DEV_STATE_CALIBRATING);
        break;

    case DEV_STATE_CALIBRATING:
        if (evt == EVT_CALIBRATION_DONE) OnEnterState(ctx, DEV_STATE_ACTIVE);
        break;

    case DEV_STATE_ACTIVE:
        if (evt == EVT_CHARGER_CONNECTED) {
            if (ctx->lamp_mode_active) OnEnterState(ctx, DEV_STATE_LAMP_MODE);
            else                       OnEnterState(ctx, DEV_STATE_CHARGING);
        }
        if (evt == EVT_BATTERY_LOW)       OnEnterState(ctx, DEV_STATE_LOW_BATTERY);
        if (evt == EVT_CMD_LAMP_ON)       { ctx->lamp_mode_active = 1; OnEnterState(ctx, DEV_STATE_LAMP_MODE); }
        if (evt == EVT_CMD_SOFT_RESET)    OnEnterState(ctx, DEV_STATE_SOFT_RESET_PENDING);
        if (evt == EVT_CMD_FACTORY_RESET) OnEnterState(ctx, DEV_STATE_FACTORY_RESET_PENDING);
        if (evt == EVT_SENSOR_ERROR)      OnEnterState(ctx, DEV_STATE_ERROR);
        if (evt == EVT_CMD_CALIBRATION)   OnEnterState(ctx, DEV_STATE_CALIBRATING);
        break;

    case DEV_STATE_LAMP_MODE:
        if (evt == EVT_CMD_LAMP_OFF)          { ctx->lamp_mode_active = 0; OnEnterState(ctx, DEV_STATE_CHARGING); }
        if (evt == EVT_CHARGER_DISCONNECTED)  { ctx->lamp_mode_active = 0; OnEnterState(ctx, DEV_STATE_ACTIVE);   }
        break;

    case DEV_STATE_CHARGING:
        if (evt == EVT_CHARGER_DISCONNECTED) OnEnterState(ctx, DEV_STATE_ACTIVE);
        if (evt == EVT_CMD_LAMP_ON)          { ctx->lamp_mode_active = 1; OnEnterState(ctx, DEV_STATE_LAMP_MODE); }
        break;

    case DEV_STATE_LOW_BATTERY:
        if (evt == EVT_BATTERY_OK)        OnEnterState(ctx, DEV_STATE_ACTIVE);
        if (evt == EVT_CHARGER_CONNECTED) OnEnterState(ctx, DEV_STATE_CHARGING);
        break;

    case DEV_STATE_FACTORY_RESET_PENDING:
        if (HAL_GetTick() - ctx->state_entry_tick > 5000U) OnEnterState(ctx, DEV_STATE_ACTIVE);
        break;

    case DEV_STATE_ERROR:
        if (evt == EVT_CMD_SOFT_RESET) OnEnterState(ctx, DEV_STATE_SOFT_RESET_PENDING);
        break;

    default:
        break;
    }
}

DeviceState_t Device_GetState(const DeviceContext_t *ctx)
{
    return ctx->state;
}
