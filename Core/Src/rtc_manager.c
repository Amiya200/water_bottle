#pragma GCC optimize("Os")
#include "rtc_manager.h"
#include <string.h>

/* ============================================================================
 * PCF8563 real-time clock driver  (schematic U10, I2C1 @ 0x51)
 *
 * Replaces the earlier DS3231 implementation, which targeted the wrong chip
 * (DS3231 lives at 0x68 with a different register map, so it never ACKed and
 *  every timestamp came back as zero).
 *
 * 1 Hz tick: the PCF8563 has no DS3231-style 1 Hz SQW. We use its countdown
 * timer programmed to 1 Hz / count 1 with timer-interrupt enabled, which
 * pulses INT# (wired to PA11/RTC_INT) once per second.
 * ==========================================================================*/

/* PCF8563 register addresses */
#define PCF_REG_CTRL1     0x00U   /* control/status 1                    */
#define PCF_REG_CTRL2     0x01U   /* control/status 2 (TIE/AIE/TF/AF)    */
#define PCF_REG_VL_SEC    0x02U   /* bit7 = VL (clock-integrity lost)    */
#define PCF_REG_MINUTES   0x03U
#define PCF_REG_HOURS     0x04U
#define PCF_REG_DAYS      0x05U
#define PCF_REG_WEEKDAYS  0x06U
#define PCF_REG_MONTHS    0x07U   /* bit7 = century                      */
#define PCF_REG_YEARS     0x08U
#define PCF_REG_TIMER_CTL 0x0EU   /* timer control                       */
#define PCF_REG_TIMER     0x0FU   /* timer countdown value               */

/* CTRL2 bits */
#define PCF_CTRL2_TIE     0x01U   /* timer interrupt enable              */
#define PCF_CTRL2_TF      0x04U   /* timer flag (write 0 to clear)       */

/* TIMER_CTL bits */
#define PCF_TIMER_ENABLE  0x80U
#define PCF_TIMER_1HZ     0x02U   /* source clock = 1 Hz                 */

/* Shared days-per-month table (non-leap year). */
static const uint8_t s_days_in_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

/* BCD helpers */
static uint8_t BCD2DEC(uint8_t b) { return (uint8_t)((b >> 4) * 10U + (b & 0x0FU)); }
static uint8_t DEC2BCD(uint8_t d) { return (uint8_t)(((d / 10U) << 4) | (d % 10U)); }

static HAL_StatusTypeDef PCF_ReadRegs(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_StatusTypeDef s = HAL_I2C_Master_Transmit(hi2c, RTC_I2C_ADDR, &reg, 1, RTC_TIMEOUT_MS);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(hi2c, RTC_I2C_ADDR, buf, len, RTC_TIMEOUT_MS);
}

static HAL_StatusTypeDef PCF_WriteRegs(I2C_HandleTypeDef *hi2c,
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

    /* CTRL1 = 0x00 : normal mode, oscillator running. */
    uint8_t ctrl1 = 0x00U;
    if (PCF_WriteRegs(hi2c, PCF_REG_CTRL1, &ctrl1, 1) != HAL_OK) return HAL_ERROR;

    /* CTRL2 = TIE : enable timer interrupt on INT#, clear any pending flag. */
    uint8_t ctrl2 = PCF_CTRL2_TIE;
    if (PCF_WriteRegs(hi2c, PCF_REG_CTRL2, &ctrl2, 1) != HAL_OK) return HAL_ERROR;

    /* Timer = 1 count, source 1 Hz, enabled  -> INT# pulses once per second. */
    uint8_t tval = 1U;
    PCF_WriteRegs(hi2c, PCF_REG_TIMER, &tval, 1);
    uint8_t tctl = PCF_TIMER_ENABLE | PCF_TIMER_1HZ;
    PCF_WriteRegs(hi2c, PCF_REG_TIMER_CTL, &tctl, 1);

    hrtc->initialized = 1;
    return RTC_Read(hrtc);
}

HAL_StatusTypeDef RTC_Read(RTC_Handle_t *hrtc)
{
    uint8_t raw[7];   /* regs 0x02..0x08 */
    if (PCF_ReadRegs(hrtc->hi2c, PCF_REG_VL_SEC, raw, 7) != HAL_OK) return HAL_ERROR;

    hrtc->now.seconds = BCD2DEC(raw[0] & 0x7FU);   /* mask VL bit          */
    hrtc->now.minutes = BCD2DEC(raw[1] & 0x7FU);
    hrtc->now.hours   = BCD2DEC(raw[2] & 0x3FU);
    hrtc->now.date    = BCD2DEC(raw[3] & 0x3FU);   /* days                 */
    hrtc->now.day     = BCD2DEC(raw[4] & 0x07U);   /* weekday              */
    hrtc->now.month   = BCD2DEC(raw[5] & 0x1FU);   /* months (mask century)*/
    hrtc->now.year    = BCD2DEC(raw[6]);

    hrtc->unix_approx = RTC_ToUnix(&hrtc->now);
    return HAL_OK;
}

HAL_StatusTypeDef RTC_Write(RTC_Handle_t *hrtc, const RTC_DateTime_t *dt)
{
    uint8_t raw[7];
    raw[0] = DEC2BCD(dt->seconds);   /* VL=0 -> mark clock integrity OK */
    raw[1] = DEC2BCD(dt->minutes);
    raw[2] = DEC2BCD(dt->hours);
    raw[3] = DEC2BCD(dt->date);      /* days       */
    raw[4] = DEC2BCD(dt->day);       /* weekday    */
    raw[5] = DEC2BCD(dt->month);     /* months (century bit left 0) */
    raw[6] = DEC2BCD(dt->year);
    return PCF_WriteRegs(hrtc->hi2c, PCF_REG_VL_SEC, raw, 7);
}

/* Convert unix timestamp to RTC_DateTime_t and write to the PCF8563.
 * No atoi / string parsing — receives uint32_t directly from BLE packet. */
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

    /* day-of-week: 1970-01-01 was a Thursday (=4). PCF8563 weekday 0..6. */
    dt.day = (uint8_t)((total_days + 4UL) % 7UL);

    /* Walk years from 1970 */
    uint32_t year = 1970;
    while (1) {
        uint32_t days_this_year = ((year % 4 == 0) && (year % 100 != 0 || year % 400 == 0)) ? 366UL : 365UL;
        if (total_days < days_this_year) break;
        total_days -= days_this_year;
        year++;
    }
    dt.year = (uint8_t)(year - 2000);  /* store 0-99 */

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

    HAL_StatusTypeDef s = RTC_Write(hrtc, &dt);
    /* Keep the cached unix/struct in sync immediately so callers that read
     * unix_approx right after a TIMESTAMP command get the new value without
     * waiting for the next RTC_Read. */
    hrtc->now         = dt;
    hrtc->unix_approx = unix_time;
    return s;
}

void RTC_TickISR(RTC_Handle_t *hrtc)
{
    hrtc->tick_flag = 1;
    /* Advance the cached unix clock locally so timestamps stay correct
     * between (relatively infrequent) full RTC_Read refreshes. */
    hrtc->unix_approx++;
}

uint8_t RTC_PopTick(RTC_Handle_t *hrtc)
{
    if (hrtc->tick_flag) {
        hrtc->tick_flag = 0;
        return 1U;
    }
    return 0U;
}

/* Clear the PCF8563 timer flag so INT# de-asserts and can fire again.
 * Called from the app once per tick (I2C must not be touched in the ISR). */
HAL_StatusTypeDef RTC_ClearTimerFlag(RTC_Handle_t *hrtc)
{
    uint8_t ctrl2 = PCF_CTRL2_TIE;   /* TIE set, TF cleared (write 0 to TF) */
    return PCF_WriteRegs(hrtc->hi2c, PCF_REG_CTRL2, &ctrl2, 1);
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
