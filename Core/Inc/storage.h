/**
 * @file storage.h
 * @brief M24512 EEPROM persistence: device config, user preferences,
 *        30-day daily summaries, and detailed drinking-event log.
 *
 *  M24512 = 64KByte (512Kbit), 128-byte page, 16-bit addressing.
 *  Memory map:
 *    0x0000  config block  (magic + version + prefs + calibration + CRC)
 *    0x0100  daily summary ring (30 entries)
 *    0x0400  event log ring (detailed drinking events)
 */
#ifndef STORAGE_H
#define STORAGE_H
#include "board.h"

#define CFG_MAGIC   0x48595332u   /* "HYS2" */

/* ---- user-configurable preferences (FRD section 1 & 11) ---- */
typedef struct {
    uint16_t water_purity_goal;   /* ppm, default 500 */
    uint16_t water_temp_goal;     /* deci-C threshold, default 400 = 40.0C */
    uint16_t hydration_goal_ml;   /* default 2000 */
    uint16_t rem_start_min;       /* minutes since midnight, default 480 (08:00) */
    uint16_t rem_end_min;         /* default 1320 (22:00 per JSON example) */
    uint16_t rem_freq_min;        /* default 60 */
    uint32_t hyd_color;           /* 0xRRGGBB, default green 0x00FF00 */
    uint32_t lamp_color;          /* 0xRRGGBB, default white 0xFFFFFF */
    uint8_t  reminder_sound;      /* sound_id_t */
} user_prefs_t;

/* ---- full persisted config block ---- */
typedef struct {
    uint32_t    magic;
    uint16_t    cfg_version;
    uint8_t     is_registered;
    uint8_t     is_calibrated;
    int32_t     tare_offset;      /* HX711 empty-bottle zero */
    int32_t     hx_scale_x100;    /* counts per gram × 100 */
    user_prefs_t prefs;
    char        user_id[24];
    char        nickname[24];
    int8_t      tz_offset_min_q;  /* timezone quarter-hours (e.g. +5:30 = 22) */
    uint32_t    crc;              /* CRC32 over everything above */
} device_config_t;

/* ---- daily summary (kept 30 days) ---- */
typedef struct {
    uint32_t epoch_day;     /* days since epoch; 0 = empty slot */
    uint32_t total_ml;
    uint16_t avg_purity;
    int16_t  avg_temp_x10;
    uint16_t sample_count;  /* for running average */
} daily_summary_t;

#define DAILY_COUNT  30

/* ---- detailed drinking event ---- */
typedef struct {
    uint32_t epoch;         /* timestamp */
    uint16_t volume_ml;
    uint16_t purity_ppm;
    int16_t  temp_x10;
    uint8_t  synced;        /* 0 = pending, 1 = acknowledged */
    uint8_t  _pad;
} drink_event_t;

#define EVENT_RING_LEN  64    /* on-device buffer; cleared after SYNC_ACK */

void storage_init(void);

/* config */
bool storage_load_config(device_config_t *cfg);
bool storage_save_config(const device_config_t *cfg);
void storage_default_config(device_config_t *cfg);

/* daily summaries */
void storage_summary_add_sample(uint32_t epoch_day, uint16_t ml,
                                uint16_t purity, int16_t temp_x10);
uint8_t storage_summary_get_all(daily_summary_t *out, uint8_t max);

/* events */
bool storage_event_push(const drink_event_t *e);
uint8_t storage_event_get_pending(drink_event_t *out, uint8_t max,
                                  uint32_t from_epoch, uint32_t to_epoch);
void storage_event_mark_synced(uint32_t up_to_epoch);

/* wipe everything (factory reset) */
void storage_factory_wipe(void);

/* raw EEPROM */
bool eeprom_write(uint16_t addr, const uint8_t *data, uint16_t len);
bool eeprom_read(uint16_t addr, uint8_t *data, uint16_t len);

uint32_t crc32_calc(const uint8_t *data, uint32_t len);

#endif
