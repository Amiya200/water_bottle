/* HYDRA OPT: legacy LL PCF8563 driver.
 * Production uses rtc_manager.c. Define HYDRA_ENABLE_LEGACY_LL_RTC together
 * with HYDRA_ENABLE_LEGACY_LL_SENSORS only for the old LL test stack.
 */
#ifdef HYDRA_ENABLE_LEGACY_LL_RTC

/**
 * @file rtc_pcf8563.c
 */
#include "rtc_pcf8563.h"
#include "i2c_bus.h"
#include "sensors.h"   /* g_health */

#define PCF_REG_CTRL1   0x00
#define PCF_REG_CTRL2   0x01
#define PCF_REG_SECONDS 0x02

static uint8_t bcd2dec(uint8_t b) { return (uint8_t)((b >> 4) * 10 + (b & 0x0F)); }
static uint8_t dec2bcd(uint8_t d) { return (uint8_t)(((d / 10) << 4) | (d % 10)); }

bool rtc_init(void)
{
    if (!i2c_ping(I2C_ADDR_PCF8563)) { g_health.rtc_ok = false; return false; }
    /* CTRL1 = 0 (normal run), CTRL2 = 0 (no alarms/timers asserted) */
    i2c_reg_write8(I2C_ADDR_PCF8563, PCF_REG_CTRL1, 0x00);
    i2c_reg_write8(I2C_ADDR_PCF8563, PCF_REG_CTRL2, 0x00);
    g_health.rtc_ok = true;
    return true;
}

bool rtc_set(const rtc_time_t *t)
{
    uint8_t b[8];
    b[0] = PCF_REG_SECONDS;
    b[1] = dec2bcd(t->sec) & 0x7F;
    b[2] = dec2bcd(t->min) & 0x7F;
    b[3] = dec2bcd(t->hour) & 0x3F;
    b[4] = dec2bcd(t->day)  & 0x3F;
    b[5] = t->wday & 0x07;
    /* century bit (bit7 of month) = 0 for 20xx */
    b[6] = dec2bcd(t->month) & 0x1F;
    b[7] = dec2bcd((uint8_t)(t->year % 100));
    return i2c_write(I2C_ADDR_PCF8563, b, 8) == 0;
}

bool rtc_get(rtc_time_t *t)
{
    uint8_t b[7];
    if (i2c_reg_read(I2C_ADDR_PCF8563, PCF_REG_SECONDS, b, 7) != 0) {
        g_health.rtc_ok = false; return false;
    }
    /* VL bit (b[0] bit7) set => oscillator stopped / time invalid */
    if (b[0] & 0x80) { g_health.rtc_ok = false; }
    t->sec   = bcd2dec(b[0] & 0x7F);
    t->min   = bcd2dec(b[1] & 0x7F);
    t->hour  = bcd2dec(b[2] & 0x3F);
    t->day   = bcd2dec(b[3] & 0x3F);
    t->wday  = b[4] & 0x07;
    t->month = bcd2dec(b[5] & 0x1F);
    t->year  = 2000 + bcd2dec(b[6]);
    return true;
}

/* days since epoch (1970). Valid for 2000+ years used here. */
static uint32_t days_from_civil(uint16_t y, uint8_t m, uint8_t d)
{
    y -= (m <= 2);
    int32_t era = (int32_t)y / 400;
    uint32_t yoe = (uint32_t)(y - era * 400);
    uint32_t doy = (153U * ((m > 2) ? (m - 3) : (m + 9)) + 2) / 5 + d - 1;
    uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (uint32_t)(era * 146097 + (int32_t)doe - 719468);
}

uint32_t rtc_epoch(const rtc_time_t *t)
{
    uint32_t days = days_from_civil(t->year, t->month, t->day);
    return days * 86400U + t->hour * 3600U + t->min * 60U + t->sec;
}

void rtc_iso8601(const rtc_time_t *t, char *buf)
{
    /* manual format, no sprintf (saves flash) */
    static const char hx[] = "0123456789";
    char *p = buf;
    uint16_t y = t->year;
    *p++ = hx[(y/1000)%10]; *p++ = hx[(y/100)%10];
    *p++ = hx[(y/10)%10];   *p++ = hx[y%10];
    *p++ = '-';
    *p++ = hx[t->month/10]; *p++ = hx[t->month%10]; *p++ = '-';
    *p++ = hx[t->day/10];   *p++ = hx[t->day%10];   *p++ = 'T';
    *p++ = hx[t->hour/10];  *p++ = hx[t->hour%10];  *p++ = ':';
    *p++ = hx[t->min/10];   *p++ = hx[t->min%10];   *p++ = ':';
    *p++ = hx[t->sec/10];   *p++ = hx[t->sec%10];   *p++ = 'Z';
    *p   = '\0';
}


#endif /* HYDRA_ENABLE_LEGACY_LL_RTC */
