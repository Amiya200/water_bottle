#ifndef EE_DATA_H
#define EE_DATA_H

/* ============================================================================
 * ee_data.h — M24512 (512 Kbit / 64 KByte) external EEPROM data store
 *
 * Holds the two persistent logs that the PRD requires to live "on the device":
 *   - 30-day daily-summary ring   (FRD §3, §7.1)
 *   - detailed drink-event ring   (FRD §7.1, §7.2 — survives a full offline day)
 *
 * This is SEPARATE from ee_store.* (the bring-up MEASURE/DUMP record log) and
 * uses a different, non-overlapping region of the same M24512 chip so both can
 * coexist.  Settings/calibration stay in internal flash (data_storage.*).
 *
 * Chip : M24512-RDW6TP, I2C 0x50, 16-bit addressing, 128-byte page,
 *        ~5 ms write cycle (ACK-polled).
 *
 * M24512 memory map (whole chip 0x0000..0xFFFF):
 *   0x0000..0x103F : ee_store measurement log (header + 256×16B records)
 *   0x2000         : daily-summary header   (EE_DataHdr_t, 16 B)
 *   0x2040..0x23FF : daily-summary records  (30 × 32 B  = 960 B)
 *   0x3000         : event-log header       (EE_DataHdr_t, 16 B)
 *   0x3040..0x363F : event records          (96 × 16 B  = 1536 B)
 *
 * All record sizes divide the 128-byte page evenly and are page-aligned, so a
 * single record write never straddles a page boundary (no wrap corruption).
 * ==========================================================================*/

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ─── I2C address (shared chip with ee_store) ───────────────────────────── */
#ifndef EE_DATA_I2C_ADDR
#define EE_DATA_I2C_ADDR     (0x50U << 1)   /* HAL 8-bit form */
#endif

/* ─── Region layout ─────────────────────────────────────────────────────── */
#define EE_DATA_DAILY_HDR    0x2000U
#define EE_DATA_DAILY_REC    0x2040U
#define EE_DATA_DAILY_MAX    30U             /* PRD: 30 days                  */
#define EE_DATA_DAILY_RECSZ  32U             /* page-aligned (32 | 128)       */

#define EE_DATA_EVT_HDR      0x3000U
#define EE_DATA_EVT_REC      0x3040U
#define EE_DATA_EVT_MAX      96U             /* > a full day of sips          */
#define EE_DATA_EVT_RECSZ    16U             /* page-aligned (16 | 128)       */

#define EE_DATA_HDR_MAGIC    0xD7U
#define EE_DATA_DAILY_MAGIC  0xDAU
#define EE_DATA_EVT_MAGIC    0xE7U

/* ─── Region header (16 bytes) ──────────────────────────────────────────── */
typedef struct {
    uint8_t  magic;        /* EE_DATA_HDR_MAGIC when initialised             */
    uint8_t  _r0;
    uint16_t count;        /* number of slots written (capped at region MAX) */
    uint16_t head;         /* next slot to write (ring)                      */
    uint8_t  _r[9];
    uint8_t  crc8;         /* CRC8 over the first 15 bytes                   */
} EE_DataHdr_t;            /* 16 bytes */

/* ─── Daily summary record (32 bytes, crc8 last) ────────────────────────── */
typedef struct {
    uint8_t  magic;          /* [0]  EE_DATA_DAILY_MAGIC                     */
    uint8_t  valid;          /* [1]  1 = in use                              */
    uint16_t sample_count;   /* [2-3]  drink events folded into the average  */
    uint32_t date_unix;      /* [4-7]  midnight epoch of the day             */
    uint32_t total_ml;       /* [8-11] total consumed that day               */
    uint32_t sum_purity;     /* [12-15] running Σ purity   (avg = Σ/count)   */
    int32_t  sum_temp_x10;   /* [16-19] running Σ temp×10  (avg = Σ/count)   */
    uint8_t  _rsv[11];       /* [20-30]                                      */
    uint8_t  crc8;           /* [31]  CRC8 over the first 31 bytes           */
} EE_DailyRec_t;             /* 32 bytes */

/* ─── Detailed drink-event record (16 bytes, crc8 last) ─────────────────── */
typedef struct {
    uint8_t  magic;          /* [0]  EE_DATA_EVT_MAGIC                       */
    uint8_t  synced;         /* [1]  0 = pending upload, 1 = synced          */
    uint16_t volume_ml;      /* [2-3]                                        */
    uint16_t purity_ppm;     /* [4-5]                                        */
    int16_t  temp_x10;       /* [6-7]                                        */
    uint32_t unix_time;      /* [8-11] time the reading was taken            */
    uint8_t  _rsv[3];        /* [12-14]                                      */
    uint8_t  crc8;           /* [15] CRC8 over the first 15 bytes            */
} EE_EvtRec_t;               /* 16 bytes */

/* ─── API ───────────────────────────────────────────────────────────────── */
void     EE_Data_Init(I2C_HandleTypeDef *hi2c);  /* formats headers if blank */
uint8_t  EE_Data_IsPresent(void);
void     EE_Data_Wipe(void);                      /* factory reset (both rings)*/

/* Daily summaries (running average maintained per day). */
uint8_t  EE_Daily_AddSample(uint32_t day_midnight, uint16_t ml,
                            uint16_t ppm, int16_t temp_x10);
uint8_t  EE_Daily_GetForDay(uint32_t day_midnight, EE_DailyRec_t *out);
void     EE_Daily_PurgeOlderThan(uint32_t cutoff_midnight);
uint16_t EE_Daily_Count(void);
uint8_t  EE_Daily_ReadHdr(EE_DataHdr_t *h);
uint8_t  EE_Daily_ReadSlot(uint16_t slot, EE_DailyRec_t *out);

/* Detailed drink events (ring; cleared/marked after SYNC_ACK). */
uint8_t  EE_Evt_Push(uint32_t unix_time, uint16_t ml,
                     uint16_t ppm, int16_t temp_x10);
void     EE_Evt_MarkSynced(uint32_t up_to_unix);
uint16_t EE_Evt_Count(void);
uint16_t EE_Evt_PendingCount(void);
uint8_t  EE_Evt_StoragePct(void);                 /* ring fill, 0..100        */
uint8_t  EE_Evt_ReadHdr(EE_DataHdr_t *h);
uint8_t  EE_Evt_ReadSlot(uint16_t slot, EE_EvtRec_t *out);

#endif /* EE_DATA_H */
