/*
 * ee_store.c — external M24512 I2C EEPROM record log (see ee_store.h)
 */
#pragma GCC optimize("Os")
#include "ee_store.h"
#include <string.h>

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint8_t            s_present = 0U;

/* ─── CRC8 (poly 0x07) over a byte span ─────────────────────────────────── */
static uint8_t ee_crc8(const uint8_t *d, uint8_t len)
{
    uint8_t crc = 0x00U;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= d[i];
        for (uint8_t b = 0U; b < 8U; b++)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ─── Low-level 16-bit-addressed read/write with ACK polling ────────────── */
static uint8_t ee_write(uint16_t mem_addr, const uint8_t *data, uint16_t len)
{
    if (s_hi2c == NULL) return 0U;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(s_hi2c, EE_I2C_ADDR, mem_addr,
                                             I2C_MEMADD_SIZE_16BIT,
                                             (uint8_t *)data, len, 100U);
    if (st != HAL_OK) return 0U;
    /* Poll until the internal write cycle finishes (ACK polling). */
    uint32_t t0 = HAL_GetTick();
    while (HAL_I2C_IsDeviceReady(s_hi2c, EE_I2C_ADDR, 1, 2) != HAL_OK) {
        if ((HAL_GetTick() - t0) > 20U) return 0U;
    }
    return 1U;
}

static uint8_t ee_read(uint16_t mem_addr, uint8_t *data, uint16_t len)
{
    if (s_hi2c == NULL) return 0U;
    return (HAL_I2C_Mem_Read(s_hi2c, EE_I2C_ADDR, mem_addr,
                             I2C_MEMADD_SIZE_16BIT, data, len, 100U) == HAL_OK) ? 1U : 0U;
}

/* A page write must not straddle a 128-byte page boundary. Our records are
 * 16 bytes and slots are 16-byte aligned, so a single record never crosses a
 * page — a plain ee_write() per record is safe. */

/* ─── Init / presence ───────────────────────────────────────────────────── */
void EE_Store_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_present = (HAL_I2C_IsDeviceReady(hi2c, EE_I2C_ADDR, 3, 10) == HAL_OK) ? 1U : 0U;
    if (!s_present) return;

    EE_Header_t h;
    if (!EE_Store_ReadHeader(&h) || h.magic != EE_HEADER_MAGIC) {
        EE_Store_Format();
    }
}

uint8_t EE_Store_IsPresent(void) { return s_present; }

/* ─── Header ────────────────────────────────────────────────────────────── */
uint8_t EE_Store_ReadHeader(EE_Header_t *out)
{
    if (!ee_read(EE_HEADER_ADDR, (uint8_t *)out, sizeof(EE_Header_t))) return 0U;
    if (out->crc8 != ee_crc8((const uint8_t *)out, sizeof(EE_Header_t) - 1U)) {
        return 0U;   /* caller treats as uninitialised */
    }
    return 1U;
}

static uint8_t ee_write_header(EE_Header_t *h)
{
    h->magic = EE_HEADER_MAGIC;
    h->crc8  = ee_crc8((const uint8_t *)h, sizeof(EE_Header_t) - 1U);
    return ee_write(EE_HEADER_ADDR, (const uint8_t *)h, sizeof(EE_Header_t));
}

uint8_t EE_Store_Format(void)
{
    if (!s_present) return 0U;
    EE_Header_t h;
    memset(&h, 0, sizeof(h));
    h.count     = 0U;
    h.write_idx = 0U;
    return ee_write_header(&h);
}

/* ─── Records ───────────────────────────────────────────────────────────── */
static uint16_t slot_addr(uint16_t slot)
{
    return (uint16_t)(EE_RECORDS_ADDR + (uint32_t)slot * EE_RECORD_SIZE);
}

uint8_t EE_Store_Append(const EE_Record_t *rec)
{
    if (!s_present) return 0U;

    EE_Header_t h;
    if (!EE_Store_ReadHeader(&h) || h.magic != EE_HEADER_MAGIC) {
        memset(&h, 0, sizeof(h));   /* recover: start a fresh log */
    }
    if (h.write_idx >= EE_MAX_RECORDS) h.write_idx = 0U;

    EE_Record_t r = *rec;
    r.magic = EE_RECORD_MAGIC;
    r.crc8  = ee_crc8((const uint8_t *)&r, sizeof(EE_Record_t) - 1U);

    if (!ee_write(slot_addr(h.write_idx), (const uint8_t *)&r, sizeof(r))) return 0U;

    h.write_idx = (uint16_t)((h.write_idx + 1U) % EE_MAX_RECORDS);
    if (h.count < EE_MAX_RECORDS) h.count++;

    return ee_write_header(&h);
}

uint8_t EE_Store_ReadRecord(uint16_t slot, EE_Record_t *out)
{
    if (!s_present || slot >= EE_MAX_RECORDS) return 0U;
    if (!ee_read(slot_addr(slot), (uint8_t *)out, sizeof(EE_Record_t))) return 0U;
    if (out->magic != EE_RECORD_MAGIC) return 0U;
    if (out->crc8 != ee_crc8((const uint8_t *)out, sizeof(EE_Record_t) - 1U)) return 0U;
    return 1U;
}

uint16_t EE_Store_Count(void)
{
    EE_Header_t h;
    if (!EE_Store_ReadHeader(&h) || h.magic != EE_HEADER_MAGIC) return 0U;
    return h.count;
}
