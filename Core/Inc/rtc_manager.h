#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* External RTC on I2C1 — PCF8563 (0x51, per schematic U10)
 * RTC_INT → PA11 (EXTI4_15) — INT# driven by the 1 Hz countdown timer
 */

#define RTC_I2C_ADDR     (0x51U << 1)
#define RTC_TIMEOUT_MS   100U

typedef struct {
    uint8_t  seconds;   /* 0-59  */
    uint8_t  minutes;   /* 0-59  */
    uint8_t  hours;     /* 0-23  */
    uint8_t  day;       /* 1-7   */
    uint8_t  date;      /* 1-31  */
    uint8_t  month;     /* 1-12  */
    uint8_t  year;      /* 0-99 (20xx) */
} RTC_DateTime_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    RTC_DateTime_t     now;
    uint32_t           unix_approx;   /* unix epoch (seconds) */
    uint8_t            tick_flag;     /* set by EXTI ISR each second */
    uint8_t            initialized;
} RTC_Handle_t;

HAL_StatusTypeDef RTC_Init(RTC_Handle_t *hrtc, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef RTC_Read(RTC_Handle_t *hrtc);
HAL_StatusTypeDef RTC_Write(RTC_Handle_t *hrtc, const RTC_DateTime_t *dt);

/* Set time from unix timestamp (replaces RTC_SetFromISO — no atoi/string parsing) */
HAL_StatusTypeDef RTC_SetFromUnix(RTC_Handle_t *hrtc, uint32_t unix_time);

void     RTC_TickISR(RTC_Handle_t *hrtc);
uint8_t  RTC_PopTick(RTC_Handle_t *hrtc);
HAL_StatusTypeDef RTC_ClearTimerFlag(RTC_Handle_t *hrtc);
uint32_t RTC_ToUnix(const RTC_DateTime_t *dt);

/* Reminder window check — takes h/m directly, no atoi */
uint8_t  RTC_IsInWindow(const RTC_DateTime_t *dt,
                         uint8_t h_start, uint8_t m_start,
                         uint8_t h_end,   uint8_t m_end);

#endif /* RTC_MANAGER_H */
