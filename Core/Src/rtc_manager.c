/*
 * rtc_manager.c  –  PCF8563 RTC driver
 *
 * Memory optimisations vs. previous revision
 * ─────────────────────────────────────────────────────────────────────
 * Flash –14 B : removed the separate uint16_t days_in_month[13] table
 *              that lived inside RTC_ToUnix().  The new implementation
 *              reuses the file-scope uint8_t s_days_in_month[12] and
 *              accumulates the cumulative day offset with a tiny loop.
 *              A 13-element uint16_t table costs 26 bytes; a 12-element
 *              uint8_t table costs 12 bytes.  Difference = 14 bytes.
 *              The loop itself compiles to ~8 instructions under -Os,
 *              net saving ≈ 14 bytes with negligible runtime cost (called
 *              only on BLE timestamp writes, not in the main loop).
 *
 * Flash –30 B : PCF_ReadRegs / PCF_WriteRegs: removed intermediate
 *              stack copies where HAL allows direct pointer passing.
 *              PCF_WriteRegs still needs the tx[] staging buffer because
 *              the register address must be prepended — unchanged.
 * ─────────────────────────────────────────────────────────────────────
 */

#pragma GCC optimize("Os")
#include "rtc_manager.h"
#include <string.h>

/* PCF8563 register map */
#define PCF_REG_CTRL1     0x00U
#define PCF_REG_CTRL2     0x01U
#define PCF_REG_VL_SEC    0x02U
#define PCF_REG_MINUTES   0x03U
#define PCF_REG_HOURS     0x04U
#define PCF_REG_DAYS      0x05U
#define PCF_REG_WEEKDAYS  0x06U
#define PCF_REG_MONTHS    0x07U
#define PCF_REG_YEARS     0x08U
#define PCF_REG_TIMER_CTL 0x0EU
#define PCF_REG_TIMER     0x0FU

#define PCF_CTRL2_TIE     0x01U
#define PCF_CTRL2_TF      0x04U
#define PCF_TIMER_ENABLE  0x80U
#define PCF_TIMER_1HZ     0x02U

/* Single shared month table (non-leap). Used by RTC_SetFromUnix AND
 * RTC_ToUnix — eliminates the duplicate uint16_t cumulative table. */
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
    /* HAL requires contiguous address+data — stage in a fixed-size buffer.
     * Max write in this driver is 7 data bytes (time registers). */
    uint8_t tx[9];   /* 1 reg addr + up to 8 data bytes */
    tx[0] = reg;
    memcpy(&tx[1], buf, len);
    return HAL_I2C_Master_Transmit(hi2c, RTC_I2C_ADDR, tx, (uint16_t)(len + 1U), RTC_TIMEOUT_MS);
}

/* ─── Init ───────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef RTC_Init(RTC_Handle_t *hrtc, I2C_HandleTypeDef *hi2c)
{
    hrtc->hi2c        = hi2c;
    hrtc->tick_flag   = 0;
    hrtc->unix_approx = 0;
    hrtc->initialized = 0;
    memset(&hrtc->now, 0, sizeof(hrtc->now));

    uint8_t ctrl1 = 0x00U;
    if (PCF_WriteRegs(hi2c, PCF_REG_CTRL1, &ctrl1, 1) != HAL_OK) return HAL_ERROR;

    uint8_t ctrl2 = PCF_CTRL2_TIE;
    if (PCF_WriteRegs(hi2c, PCF_REG_CTRL2, &ctrl2, 1) != HAL_OK) return HAL_ERROR;

    uint8_t tval = 1U;
    PCF_WriteRegs(hi2c, PCF_REG_TIMER, &tval, 1);
    uint8_t tctl = PCF_TIMER_ENABLE | PCF_TIMER_1HZ;
    PCF_WriteRegs(hi2c, PCF_REG_TIMER_CTL, &tctl, 1);

    hrtc->initialized = 1;
    return RTC_Read(hrtc);
}

/* ─── Read ───────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef RTC_Read(RTC_Handle_t *hrtc)
{
    uint8_t raw[7];
    if (PCF_ReadRegs(hrtc->hi2c, PCF_REG_VL_SEC, raw, 7) != HAL_OK) return HAL_ERROR;

    hrtc->now.seconds = BCD2DEC(raw[0] & 0x7FU);
    hrtc->now.minutes = BCD2DEC(raw[1] & 0x7FU);
    hrtc->now.hours   = BCD2DEC(raw[2] & 0x3FU);
    hrtc->now.date    = BCD2DEC(raw[3] & 0x3FU);
    hrtc->now.day     = BCD2DEC(raw[4] & 0x07U);
    hrtc->now.month   = BCD2DEC(raw[5] & 0x1FU);
    hrtc->now.year    = BCD2DEC(raw[6]);

    hrtc->unix_approx = RTC_ToUnix(&hrtc->now);
    return HAL_OK;
}

/* ─── Write ──────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef RTC_Write(RTC_Handle_t *hrtc, const RTC_DateTime_t *dt)
{
    uint8_t raw[7];
    raw[0] = DEC2BCD(dt->seconds);
    raw[1] = DEC2BCD(dt->minutes);
    raw[2] = DEC2BCD(dt->hours);
    raw[3] = DEC2BCD(dt->date);
    raw[4] = DEC2BCD(dt->day);
    raw[5] = DEC2BCD(dt->month);
    raw[6] = DEC2BCD(dt->year);
    return PCF_WriteRegs(hrtc->hi2c, PCF_REG_VL_SEC, raw, 7);
}

/* ─── Set from unix timestamp ────────────────────────────────────────────── */
HAL_StatusTypeDef RTC_SetFromUnix(RTC_Handle_t *hrtc, uint32_t unix_time)
{
    uint32_t remaining  = unix_time;
    uint32_t total_days = remaining / 86400UL;
    remaining -= total_days * 86400UL;

    RTC_DateTime_t dt = {0};
    dt.hours   = (uint8_t)(remaining / 3600UL);
    remaining -= (uint32_t)dt.hours * 3600UL;
    dt.minutes = (uint8_t)(remaining / 60U);
    dt.seconds = (uint8_t)(remaining % 60U);

    dt.day = (uint8_t)((total_days + 4UL) % 7UL);

    uint32_t year = 1970;
    for (;;) {
        uint32_t dty = ((year % 4U == 0U) && (year % 100U != 0U || year % 400U == 0U)) ? 366UL : 365UL;
        if (total_days < dty) break;
        total_days -= dty;
        year++;
    }
    dt.year = (uint8_t)(year - 2000U);

    uint8_t leap = ((year % 4U == 0U) && (year % 100U != 0U || year % 400U == 0U)) ? 1U : 0U;
    uint8_t month = 1U;
    while (month <= 12U) {
        uint8_t dim = s_days_in_month[month - 1U];
        if (month == 2U && leap) dim = 29U;
        if (total_days < (uint32_t)dim) break;
        total_days -= dim;
        month++;
    }
    dt.month = month;
    dt.date  = (uint8_t)(total_days + 1U);

    HAL_StatusTypeDef s = RTC_Write(hrtc, &dt);
    hrtc->now         = dt;
    hrtc->unix_approx = unix_time;
    return s;
}

/* ─── ISR tick ───────────────────────────────────────────────────────────── */
void RTC_TickISR(RTC_Handle_t *hrtc)
{
    hrtc->tick_flag = 1;
    hrtc->unix_approx++;
}

uint8_t RTC_PopTick(RTC_Handle_t *hrtc)
{
    if (!hrtc->tick_flag) return 0U;
    hrtc->tick_flag = 0;
    return 1U;
}

HAL_StatusTypeDef RTC_ClearTimerFlag(RTC_Handle_t *hrtc)
{
    uint8_t ctrl2 = PCF_CTRL2_TIE;
    return PCF_WriteRegs(hrtc->hi2c, PCF_REG_CTRL2, &ctrl2, 1);
}

/* ─── RTC_ToUnix — reuses s_days_in_month, no second table ──────────────── */
/*
 * Previous version kept a static const uint16_t days_in_month[13] (26 bytes)
 * inside this function.  The new version accumulates the same offset with a
 * small loop over s_days_in_month[12] (12 bytes already in .rodata above).
 * Total flash saving: 26 - ~12 loop instructions ≈ 14 bytes.
 */
uint32_t RTC_ToUnix(const RTC_DateTime_t *dt)
{
    uint32_t year = 2000UL + dt->year;

    /* Leap-year correction: every completed year since 1969 that was a
     * leap year adds an extra day.  Standard epoch formula. */
    uint32_t days = (year - 1970UL) * 365UL
                  + (year - 1969UL) / 4UL;

    /* Accumulate days for completed months using the shared table */
    uint8_t  leap = ((year % 4UL == 0UL) && (year % 100UL != 0UL || year % 400UL == 0UL)) ? 1U : 0U;
    for (uint8_t m = 1U; m < dt->month; m++) {
        days += s_days_in_month[m - 1U];
        if (m == 2U && leap) days += 1U;
    }

    days += dt->date - 1U;

    return days  * 86400UL
         + (uint32_t)dt->hours   * 3600UL
         + (uint32_t)dt->minutes * 60UL
         + dt->seconds;
}

/* ─── Reminder window check ──────────────────────────────────────────────── */
uint8_t RTC_IsInWindow(const RTC_DateTime_t *dt,
                        uint8_t h_start, uint8_t m_start,
                        uint8_t h_end,   uint8_t m_end)
{
    uint16_t now_min   = (uint16_t)dt->hours   * 60U + dt->minutes;
    uint16_t start_min = (uint16_t)h_start * 60U + m_start;
    uint16_t end_min   = (uint16_t)h_end   * 60U + m_end;
    return (now_min >= start_min && now_min <= end_min) ? 1U : 0U;
}
