/**
 * @file app.h
 * @brief HydraSense application: device state machine, drink detection,
 *        reminder scheduling, and BLE command dispatch (FRD section 13).
 */
#ifndef APP_H
#define APP_H
#include "board.h"
#include "storage.h"

typedef enum {
    DEV_FIRST_USE = 0,   /* never registered - waiting to pair */
    DEV_PAIRING,         /* app connected, registering */
    DEV_CALIBRATING,
    DEV_RUNNING,         /* normal operation */
    DEV_LAMP,            /* lamp mode (charger only) */
    DEV_RESET_PENDING    /* factory reset countdown */
} dev_state_t;

void app_init(void);
void app_task(void);                 /* main super-loop body */

/* called from EXTI when motion / RTC interrupt fires */
void app_on_motion(void);
void app_on_rtc_tick(void);

/* command entry point (called with a received JSON line) */
void app_handle_command(const char *json);

dev_state_t app_state(void);

#endif
