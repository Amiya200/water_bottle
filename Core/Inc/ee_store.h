#ifndef EE_STORE_H
#define EE_STORE_H

/* ============================================================================
 * ee_store.h — external M24512 (512 Kbit / 64 KByte) I2C EEPROM record log
 *
 * Schematic U6 : M24512-RDW6TP, I2C addr 0x50 (E0/E1/E2 = 0), on I2C1.
 *   - 16-bit internal addressing (2 address bytes per access)
 *   - 128-byte page write
 *   - ~5 ms internal write cycle (we poll ACK after each write)
 *
 * Purpose: an append-only ring log of measurement records, each stamped with
 * the RTC time at the moment it was stored. Used by the MEASURE / STOREW /
 * DUMP BLE commands so the whole flow can be exercised over Bluetooth while
 * the MCU's autonomous loop is still being brought up.
 *
 * Layout:
 *   addr 0x0000 : EE_Header_t          (magic, count, write_idx, crc)
 *   addr 0x0040 : record[0] ... record[N-1]   (16 bytes each, ring buffer)
 * ==========================================================================*/

#include "stm32f0xx_hal.h"
#include <stdint.h>

#define EE_I2C_ADDR_7BIT     0x50U
#define EE_I2C_ADDR          (EE_I2C_ADDR_7BIT << 1)   /* HAL 8-bit form */
#define EE_PAGE_SIZE         128U
#define EE_WRITE_CYCLE_MS    6U      /* datasheet 5 ms + margin */

#define EE_HEADER_ADDR       0x0000U
#define EE_RECORDS_ADDR      0x0040U   /* records start at byte 64 */
#define EE_RECORD_SIZE       16U
#define EE_MAX_RECORDS       256U      /* 256 * 16 = 4096 B used; plenty of room */

#define EE_HEADER_MAGIC      0xE5U
#define EE_RECORD_MAGIC      0x5EU

/* ─── On-EEPROM header (kept small, 16 bytes) ───────────────────────────── */
typedef struct {
    uint8_t  magic;        /* EE_HEADER_MAGIC when initialised      */
    uint8_t  _rsv0;
    uint16_t count;        /* number of valid records (<= EE_MAX_RECORDS) */
    uint16_t write_idx;    /* next slot to write (ring)             */
    uint8_t  _rsv[9];
    uint8_t  crc8;         /* CRC8 over the first 15 bytes          */
} EE_Header_t;             /* 16 bytes */

/* ─── One measurement record (16 bytes, fixed) ──────────────────────────── */
typedef struct {
    uint8_t  magic;        /* EE_RECORD_MAGIC                       */
    uint8_t  flags;        /* bit0 = TDS/temp present (refill), bit1 = synced */
    uint32_t unix_time;    /* RTC time the record was STORED        */
    int32_t  weight_ml;    /* load-cell reading (ml ≈ g)            */
    uint16_t tds_ppm;      /* 0 if not measured                     */
    int16_t  temp_x10;     /* 0 if not measured                     */
    uint8_t  crc8;         /* CRC8 over the first 15 bytes          */
    uint8_t  _rsv;
} EE_Record_t;             /* 16 bytes */

#define EE_FLAG_QUALITY     0x01U   /* TDS+temp were sampled (refill detected) */
#define EE_FLAG_SYNCED      0x02U

/* ─── API (all return 1 on success, 0 on failure) ───────────────────────── */
void    EE_Store_Init(I2C_HandleTypeDef *hi2c);
uint8_t EE_Store_IsPresent(void);

/* Erase/format the log header (does not need to wipe records). */
uint8_t EE_Store_Format(void);

/* Append one record (ring-buffer; overwrites oldest when full). */
uint8_t EE_Store_Append(const EE_Record_t *rec);

/* Read header / a record by physical slot index. */
uint8_t EE_Store_ReadHeader(EE_Header_t *out);
uint8_t EE_Store_ReadRecord(uint16_t slot, EE_Record_t *out);

uint16_t EE_Store_Count(void);

#endif /* EE_STORE_H */
