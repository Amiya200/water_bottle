#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include "stm32f0xx_hal.h"
#include "ble_protocol.h"    /* for BLE_PrefsPayload_t */
#include <stdint.h>

/*
 * STM32F030K6T6 — 32 KB Flash, 4 KB RAM
 *
 * Flash layout:
 *   0x0800_0000 – 0x0800_6FFF  : Application code (28 KB)
 *   0x0800_7000 – 0x0800_73FF  : Settings     page A  (1 KB)
 *   0x0800_7400 – 0x0800_77FF  : Drink log    page A  (1 KB)
 *   0x0800_7800 – 0x0800_7BFF  : Drink log    page B  (1 KB, wear-level spare)
 *   0x0800_7C00 – 0x0800_7FFF  : Daily summary page   (1 KB)
 */

#define STORAGE_SETTINGS_ADDR  0x08007000UL
#define STORAGE_LOG_A_ADDR     0x08007400UL
#define STORAGE_LOG_B_ADDR     0x08007800UL
#define STORAGE_DAILY_ADDR     0x08007C00UL

/* Log size limits — tuned to fit in 4 KB RAM */
#define STORAGE_MAX_DRINK_EVENTS  20U    /* was 64; 20 × 11 bytes = 220 bytes */
#define STORAGE_MAX_DAILY_DAYS     7U    /* one week rolling window */

/* ─── Persisted device settings (68 bytes) ─────────────────── */
typedef struct {
    uint32_t          magic;              /* 0xABCD1234 if valid */
    char              user_id[16];        /* trimmed from 32 */
    char              device_nickname[16];/* trimmed from 32 */
    BLE_PrefsPayload_t prefs;             /* 18 bytes packed */
    float             tare_offset;        /* HX711 calibration */
    float             hx711_scale;
    uint8_t           is_registered;
    uint8_t           is_calibrated;
    uint32_t          crc;
} DeviceSettings_t;                       /* ~68 bytes vs old 171 */

#define SETTINGS_MAGIC  0xABCD1234UL

/* ─── Individual drink event (11 bytes) ─────────────────────── */
typedef struct {
    uint32_t unix_time;
    uint16_t volume_ml;
    uint16_t purity_ppm;
    int16_t  temp_x10;     /* °C × 10, replaces float (4→2 bytes) */
    uint8_t  synced;
} DrinkEvent_t;             /* 11 bytes vs old 13 */

/* ─── Daily summary (11 bytes) ──────────────────────────────── */
typedef struct {
    uint32_t date_unix;
    uint16_t total_ml;
    uint16_t avg_purity_ppm;
    int16_t  avg_temp_x10;  /* replaces float */
    uint8_t  valid;
} DailySummary_t;           /* 11 bytes vs old 13 */

/* ─── RAM log buffers ───────────────────────────────────────── */
typedef struct {
    DrinkEvent_t events[STORAGE_MAX_DRINK_EVENTS];   /* 220 bytes */
    uint8_t      count;
    uint8_t      dirty;
} DrinkLog_t;                                         /* 222 bytes vs old 834 */

typedef struct {
    DailySummary_t days[STORAGE_MAX_DAILY_DAYS];      /* 77 bytes */
    uint8_t        count;
    uint8_t        dirty;
} DailySummaryLog_t;                                  /* 79 bytes vs old 392 */

/* ─── API ───────────────────────────────────────────────────── */
void Storage_Init(void);
void Storage_LoadSettings(DeviceSettings_t *out);
void Storage_SaveSettings(const DeviceSettings_t *in);
void Storage_EraseSettings(void);

void Storage_AddDrinkEvent(DrinkLog_t *log, const DrinkEvent_t *ev);
void Storage_FlushDrinkLog(DrinkLog_t *log);
void Storage_LoadDrinkLog(DrinkLog_t *log);
void Storage_MarkSynced(DrinkLog_t *log, uint32_t synced_up_to_unix);

void Storage_UpdateDailySummary(DailySummaryLog_t *daily, DrinkLog_t *log,
                                 uint32_t today_unix);
void Storage_FlushDailySummary(DailySummaryLog_t *daily);
void Storage_LoadDailySummary(DailySummaryLog_t *daily);
void Storage_PurgeDailySummaryOlderThan(DailySummaryLog_t *daily,
                                         uint32_t cutoff_unix);
void Storage_FactoryReset(void);

uint32_t Storage_CRC32(const uint8_t *data, uint16_t len);

/* Default prefs used on factory reset / first boot */
void Storage_DefaultPrefs(BLE_PrefsPayload_t *p);

#endif /* DATA_STORAGE_H */
