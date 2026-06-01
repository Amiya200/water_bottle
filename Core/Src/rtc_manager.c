#pragma GCC optimize("Os")
#include "rtc_manager.h"
#include <string.h>

/* Shared days-per-month table (non-leap year) — used by both
 * RTC_SetFromUnix and RTC_ToUnix to avoid two copies in flash. */
static const uint8_t s_days_in_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

/* DS3231 register addresses */
#define DS3231_REG_SECONDS  0x00U
#define DS3231_REG_MINUTES  0x01U
#define DS3231_REG_HOURS    0x02U
#define DS3231_REG_DAY      0x03U
#define DS3231_REG_DATE     0x04U
#define DS3231_REG_MONTH    0x05U
#define DS3231_REG_YEAR     0x06U
#define DS3231_REG_CONTROL  0x0EU
#define DS3231_REG_STATUS   0x0FU

/* BCD helpers */
static uint8_t BCD2DEC(uint8_t b) { return (uint8_t)((b >> 4) * 10U + (b & 0x0FU)); }
static uint8_t DEC2BCD(uint8_t d) { return (uint8_t)(((d / 10U) << 4) | (d % 10U)); }

static HAL_StatusTypeDef DS3231_ReadRegs(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Master_Transmit(hi2c, RTC_I2C_ADDR, &reg, 1, RTC_TIMEOUT_MS);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(hi2c, RTC_I2C_ADDR, buf, len, RTC_TIMEOUT_MS);
}

static HAL_StatusTypeDef DS3231_WriteRegs(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t tx[16];
    tx[0] = reg;
    memcpy(&tx[1], buf, len);
    return HAL_I2C_Master_Transmit(hi2c, RTC_I2C_ADDR, tx, (uint16_t)(len + 1U), RTC_TIMEOUT_MS);
}

HAL_StatusTypeDef RTC_Init(RTC_Handle_t *hrtc, I2C_HandleTypeDef *hi2c)
{
    hrtc->hi2c        = hi2c;
    hrtc->tick_flag   = 0;
    hrtc->unix_approx = 0;
    hrtc->initialized = 0;
    memset(&hrtc->now, 0, sizeof(hrtc->now));

    /* 1 Hz square wave output (INTCN=0, RS[1:0]=00) */
    uint8_t ctrl = 0x00U;
    if (DS3231_WriteRegs(hi2c, DS3231_REG_CONTROL, &ctrl, 1) != HAL_OK) return HAL_ERROR;

    /* Clear OSF in status register */
    uint8_t status = 0;
    DS3231_ReadRegs(hi2c, DS3231_REG_STATUS, &status, 1);
    status &= ~0x80U;
    DS3231_WriteRegs(hi2c, DS3231_REG_STATUS, &status, 1);

    hrtc->initialized = 1;
    return RTC_Read(hrtc);
}

HAL_StatusTypeDef RTC_Read(RTC_Handle_t *hrtc)
{
    uint8_t raw[7];
    if (DS3231_ReadRegs(hrtc->hi2c, DS3231_REG_SECONDS, raw, 7) != HAL_OK) return HAL_ERROR;

    hrtc->now.seconds = BCD2DEC(raw[0] & 0x7FU);
    hrtc->now.minutes = BCD2DEC(raw[1] & 0x7FU);
    hrtc->now.hours   = BCD2DEC(raw[2] & 0x3FU);
    hrtc->now.day     = BCD2DEC(raw[3] & 0x07U);
    hrtc->now.date    = BCD2DEC(raw[4] & 0x3FU);
    hrtc->now.month   = BCD2DEC(raw[5] & 0x1FU);
    hrtc->now.year    = BCD2DEC(raw[6]);

    hrtc->unix_approx = RTC_ToUnix(&hrtc->now);
    return HAL_OK;
}

HAL_StatusTypeDef RTC_Write(RTC_Handle_t *hrtc, const RTC_DateTime_t *dt)
{
    uint8_t raw[7];
    raw[0] = DEC2BCD(dt->seconds);
    raw[1] = DEC2BCD(dt->minutes);
    raw[2] = DEC2BCD(dt->hours);
    raw[3] = DEC2BCD(dt->day);
    raw[4] = DEC2BCD(dt->date);
    raw[5] = DEC2BCD(dt->month);
    raw[6] = DEC2BCD(dt->year);
    return DS3231_WriteRegs(hrtc->hi2c, DS3231_REG_SECONDS, raw, 7);
}

/* Convert unix timestamp to RTC_DateTime_t and write to DS3231.
 * No atoi / string parsing — receives uint32_t directly from BLE packet.
 */
HAL_StatusTypeDef RTC_SetFromUnix(RTC_Handle_t *hrtc, uint32_t unix_time)
{

    uint32_t remaining = unix_time;
    uint32_t total_days = remaining / 86400UL;
    remaining -= total_days * 86400UL;

    RTC_DateTime_t dt = {0};
    dt.hours   = (uint8_t)(remaining / 3600UL);
    remaining -= (uint32_t)dt.hours * 3600UL;
    dt.minutes = (uint8_t)(remaining / 60U);
    dt.seconds = (uint8_t)(remaining % 60U);

    /* Walk years from 1970 */
    uint32_t year = 1970;
    while (1) {
        uint32_t days_this_year = ((year % 4 == 0) && (year % 100 != 0 || year % 400 == 0)) ? 366UL : 365UL;
        if (total_days < days_this_year) break;
        total_days -= days_this_year;
        year++;
    }
    dt.year = (uint8_t)(year - 2000);  /* DS3231 stores 0-99 */

    /* Walk months */
    uint8_t leap = ((year % 4 == 0) && (year % 100 != 0 || year % 400 == 0)) ? 1U : 0U;
    uint8_t month = 1;
    while (month <= 12) {
        uint8_t dim = s_days_in_month[month - 1];
        if (month == 2 && leap) dim = 29;
        if (total_days < (uint32_t)dim) break;
        total_days -= dim;
        month++;
    }
    dt.month = month;
    dt.date  = (uint8_t)(total_days + 1U);
    dt.day   = 1;  /* day-of-week: not critical for function */

    return RTC_Write(hrtc, &dt);
}

void RTC_TickISR(RTC_Handle_t *hrtc)
{
    hrtc->tick_flag = 1;
}

uint8_t RTC_PopTick(RTC_Handle_t *hrtc)
{
    if (hrtc->tick_flag) {
        hrtc->tick_flag = 0;
        return 1U;
    }
    return 0U;
}

/* Simplified unix epoch (approximation; adequate for hydration log timestamps) */
uint32_t RTC_ToUnix(const RTC_DateTime_t *dt)
{
    static const uint16_t days_in_month[] = {0,31,59,90,120,151,181,212,243,273,304,334};
    uint32_t year  = 2000UL + dt->year;
    uint32_t days  = (year - 1970UL) * 365UL
                   + (year - 1969UL) / 4UL
                   + days_in_month[dt->month - 1U]
                   + dt->date - 1UL;
    if (dt->month > 2U && (year % 4UL == 0UL)) days++;
    return days  * 86400UL
         + (uint32_t)dt->hours   * 3600UL
         + (uint32_t)dt->minutes * 60UL
         + dt->seconds;
}

/* Reminder window check using integer h/m — no atoi */
uint8_t RTC_IsInWindow(const RTC_DateTime_t *dt,
                        uint8_t h_start, uint8_t m_start,
                        uint8_t h_end,   uint8_t m_end)
{
    uint16_t now_min   = (uint16_t)dt->hours   * 60U + dt->minutes;
    uint16_t start_min = (uint16_t)h_start * 60U + m_start;
    uint16_t end_min   = (uint16_t)h_end   * 60U + m_end;
    return (now_min >= start_min && now_min <= end_min) ? 1U : 0U;
}
