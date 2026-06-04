/**
 * @file rtc_pcf8563.h
 * @brief PCF8563 RTC: time-keep, set/get, alarm-based reminder support.
 */
#ifndef RTC_PCF8563_H
#define RTC_PCF8563_H
#include "board.h"

typedef struct {
    uint8_t  sec;    /* 0-59 */
    uint8_t  min;    /* 0-59 */
    uint8_t  hour;   /* 0-23 */
    uint8_t  day;    /* 1-31 */
    uint8_t  wday;   /* 0-6  */
    uint8_t  month;  /* 1-12 */
    uint16_t year;   /* full, e.g. 2026 */
} rtc_time_t;

bool rtc_init(void);
bool rtc_set(const rtc_time_t *t);
bool rtc_get(rtc_time_t *t);

/* Unix-ish epoch seconds (good enough for diff/ordering, not leap-perfect). */
uint32_t rtc_epoch(const rtc_time_t *t);

/* ISO8601 "YYYY-MM-DDThh:mm:ssZ" into buf (>=21 bytes). */
void rtc_iso8601(const rtc_time_t *t, char *buf);

#endif
