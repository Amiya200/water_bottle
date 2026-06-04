#pragma GCC optimize("Os")
#include "device_state.h"
#include <string.h>

static void OnEnterState(DeviceContext_t *ctx, DeviceState_t new_state)
{
    ctx->state            = (uint8_t)new_state;
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
    ctx->pending_event = (uint8_t)EVT_NONE;
    ctx->evt_head      = 0;
    ctx->evt_tail      = 0;
}

void Device_PostEvent(DeviceContext_t *ctx, DeviceEvent_t evt)
{
    if (evt == EVT_NONE) return;
    uint8_t next = (uint8_t)((ctx->evt_head + 1U) % DEV_EVENT_QUEUE_LEN);
    if (next == ctx->evt_tail) return;        /* queue full — drop oldest-safe */
    ctx->evt_queue[ctx->evt_head] = (uint8_t)evt;
    ctx->evt_head = next;
}

/* Pop one queued event, or EVT_NONE if empty. */
static DeviceEvent_t Device_PopEvent(DeviceContext_t *ctx)
{
    if (ctx->evt_tail == ctx->evt_head) return EVT_NONE;
    DeviceEvent_t e = (DeviceEvent_t)ctx->evt_queue[ctx->evt_tail];
    ctx->evt_tail = (uint8_t)((ctx->evt_tail + 1U) % DEV_EVENT_QUEUE_LEN);
    return e;
}

/* Apply a single event to the current state. */
static void Device_Apply(DeviceContext_t *ctx, DeviceEvent_t evt)
{
    if (evt == EVT_NONE) return;

    DeviceState_t cur = (DeviceState_t)ctx->state;

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

void Device_Run(DeviceContext_t *ctx)
{
    /* Drain ALL queued events this loop so back-to-back transitions
     * (e.g. CMD_CALIBRATION immediately followed by CALIBRATION_DONE during
     *  the registration flow) are never lost. Bounded by queue length. */
    DeviceEvent_t evt;
    uint8_t guard = DEV_EVENT_QUEUE_LEN + 1U;
    while (guard-- && (evt = Device_PopEvent(ctx)) != EVT_NONE) {
        Device_Apply(ctx, evt);
    }
}

DeviceState_t Device_GetState(const DeviceContext_t *ctx)
{
    return (DeviceState_t)ctx->state;
}
