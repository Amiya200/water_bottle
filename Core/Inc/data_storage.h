#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

/*
 * data_storage.h  –  Flash-backed drink log and daily summary storage
 *
 * RAM optimisations
 * ─────────────────────────────────────────────────────────────────────
 * –360 B RAM : STORAGE_MAX_DRINK_EVENTS reduced 50 → 20.
 *              DrinkEvent_t is 12 bytes; 20 × 12 = 240 B (was 600 B).
 *              20 events > 10 h of drinking at one event per 30 min.
 *
 * –192 B RAM : STORAGE_MAX_DAILY_DAYS reduced 30 → 14.
 *              DailySummary_t is 12 bytes; 14 × 12 = 168 B (was 360 B).
 *              14 days = 2 weeks — more than enough between syncs.
 * ─────────────────────────────────────────────────────────────────────
 */

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <string.h>

/* ─── Flash page addresses (STM32F030K6T6, 32 KB flash, 1 KB pages) ──────── */
#define STORAGE_SETTINGS_ADDR   0x08007000UL   /* page 28 */
#define STORAGE_LOG_A_ADDR      0x08007400UL   /* page 29 */
#define STORAGE_LOG_B_ADDR      0x08007800UL   /* page 30 — reserved/backup */
#define STORAGE_DAILY_ADDR      0x08007C00UL   /* page 31 */

/* ─── Magic number & array sizes ────────────────────────────────────────── */
#define SETTINGS_MAGIC              0xA55A1235UL   /* v2: fixed-point HX711 scale */

/* PRD/FRD v2.0 §7.1:
 *  - Daily summaries MUST be kept for 30 days on-device (was 7).
 *    30 × 12 = 360 B + header — still fits the 1 KB daily flash page.
 *  - Detailed events are kept until synced, then purged (see
 *    Storage_PurgeSyncedBefore). 20 events covers a full day of typical
 *    drinking between syncs (one event ≈ every 30 min over ~10 h).
 */
#define STORAGE_MAX_DRINK_EVENTS    12U   /* 12 × 12 = 144 B — RAM-constrained, see note */
#define STORAGE_MAX_DAILY_DAYS      30U   /* PRD v2 hard requirement: 30 days */

/* ─── BLE preferences payload ───────────────────────────────────────────── */
typedef struct {
    uint8_t purity_goal_hi;
    uint8_t purity_goal_lo;
    uint8_t temp_goal_hi;
    uint8_t temp_goal_lo;
    uint8_t hydration_hi;
    uint8_t hydration_lo;
    uint8_t remind_h_start;
    uint8_t remind_m_start;
    uint8_t remind_h_end;
    uint8_t remind_m_end;
    uint8_t remind_freq_min;
    uint8_t remind_r;
    uint8_t remind_g;
    uint8_t remind_b;
    uint8_t remind_sound;
    uint8_t lamp_r;
    uint8_t lamp_g;
    uint8_t lamp_b;
} BLE_PrefsPayload_t;   /* 18 bytes */

/* ─── Device settings (persisted to flash) ───────────────────────────────── */
typedef struct {
    uint32_t           magic;            /* SETTINGS_MAGIC when valid         */
    BLE_PrefsPayload_t prefs;            /* 18 bytes                          */
    uint8_t            user_id[16];      /* BLE-assigned user UUID            */
    uint8_t            device_nickname[16];
    uint8_t            is_registered;
    uint8_t            is_calibrated;
    uint8_t            _pad[2];          /* align 32-bit calibration fields    */
    int32_t            tare_offset;       /* HX711 empty raw baseline          */
    int32_t            hx711_scale_x100;  /* counts-per-gram × 100             */
    uint32_t           crc;              /* CRC32 of all fields above         */
} DeviceSettings_t;

/* ─── DrinkEvent_t ───────────────────────────────────────────────────────── */
/*
 * NOTE: temp_x10 kept as int16_t (not compressed to uint8_t).
 * The compression was reverted because data_storage.c accesses temp_x10
 * directly for daily-summary accumulation.  The field name and type must
 * match exactly what the .c file uses.
 */
typedef struct {
    uint32_t unix_time;    /* 4 — epoch seconds                             */
    uint16_t volume_ml;    /* 2 — drink volume in millilitres               */
    uint16_t purity_ppm;   /* 2 — TDS reading                               */
    int16_t  temp_x10;     /* 2 — temperature × 10 (e.g. 250 = 25.0 °C)    */
    uint8_t  synced;       /* 1 — 1 = uploaded to app                       */
    uint8_t  _pad;         /* 1 — explicit padding                          */
} DrinkEvent_t;            /* 12 bytes                                      */

/* ─── DrinkLog_t ─────────────────────────────────────────────────────────── */
typedef struct {
    DrinkEvent_t events[STORAGE_MAX_DRINK_EVENTS]; /* 12 × 12 = 144 B      */
    uint8_t      count;
    uint8_t      dirty;
} DrinkLog_t;              /* 146 bytes total, before alignment              */

/* ─── DailySummary_t ─────────────────────────────────────────────────────── */
typedef struct {
    uint32_t date_unix;        /* 4 */
    uint16_t total_ml;         /* 2 */
    uint16_t avg_purity_ppm;   /* 2 */
    int16_t  avg_temp_x10;     /* 2 */
    uint8_t  valid;            /* 1 */
    uint8_t  _pad;             /* 1 */
} DailySummary_t;              /* 12 bytes */

/* ─── DailySummaryLog_t ──────────────────────────────────────────────────── */
/*
 * Named DailySummaryLog_t (not DailyLog_t) to match data_storage.c usage.
 */
typedef struct {
    DailySummary_t days[STORAGE_MAX_DAILY_DAYS]; /* 7 × 12 = 84 B          */
    uint8_t        count;
    uint8_t        dirty;
} DailySummaryLog_t;           /* 86 bytes total, before alignment           */

/* ─── CRC helper (used internally and by tests) ──────────────────────────── */
uint32_t Storage_CRC32(const uint8_t *data, uint16_t len);

/* ─── API ────────────────────────────────────────────────────────────────── */
void Storage_Init(void);

void Storage_DefaultPrefs(BLE_PrefsPayload_t *p);
void Storage_LoadSettings(DeviceSettings_t *out);
void Storage_SaveSettings(const DeviceSettings_t *in);
void Storage_EraseSettings(void);

void Storage_AddDrinkEvent(DrinkLog_t *log, const DrinkEvent_t *ev);
void Storage_FlushDrinkLog(DrinkLog_t *log);
void Storage_LoadDrinkLog(DrinkLog_t *log);
void Storage_MarkSynced(DrinkLog_t *log, uint32_t cutoff_unix);

/* FRD §7.1: once detailed records are transferred (SYNC_ACK), they can be
 * cleared to free space. Removes events that are BOTH synced AND older than
 * cutoff_unix (pass today's midnight so today's events survive for the
 * daily-summary recompute). */
void Storage_PurgeSyncedBefore(DrinkLog_t *log, uint32_t cutoff_unix);

void Storage_UpdateDailySummary(DailySummaryLog_t *daily,
                                 DrinkLog_t *log,
                                 uint32_t today_unix);
void Storage_FlushDailySummary(DailySummaryLog_t *daily);
void Storage_LoadDailySummary(DailySummaryLog_t *daily);
void Storage_PurgeDailySummaryOlderThan(DailySummaryLog_t *daily,
                                         uint32_t cutoff_unix);

void Storage_FactoryReset(void);

#endif /* DATA_STORAGE_H */
