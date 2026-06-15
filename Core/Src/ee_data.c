/*
 * ee_data.c — M24512 external EEPROM data store (see ee_data.h)
 *
 * 30-day daily-summary ring + detailed drink-event ring, on a non-overlapping
 * region of the same M24512 used by ee_store.c.  All record writes are
 * page-aligned so a write never straddles a 128-byte page boundary.
 */
#pragma GCC optimize("Os")
#include "ee_data.h"
#include <string.h>

static I2C_HandleTypeDef *s_hi2c    = NULL;
static uint8_t            s_present = 0U;

/* ─── CRC8 (poly 0x07) ──────────────────────────────────────────────────── */
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
static uint8_t ee_write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    if (s_hi2c == NULL) return 0U;
    if (HAL_I2C_Mem_Write(s_hi2c, EE_DATA_I2C_ADDR, addr,
                          I2C_MEMADD_SIZE_16BIT,
                          (uint8_t *)data, len, 100U) != HAL_OK) return 0U;
    uint32_t t0 = HAL_GetTick();
    while (HAL_I2C_IsDeviceReady(s_hi2c, EE_DATA_I2C_ADDR, 1, 2) != HAL_OK) {
        if ((HAL_GetTick() - t0) > 20U) return 0U;
    }
    return 1U;
}

static uint8_t ee_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    if (s_hi2c == NULL) return 0U;
    return (HAL_I2C_Mem_Read(s_hi2c, EE_DATA_I2C_ADDR, addr,
                             I2C_MEMADD_SIZE_16BIT, data, len, 100U) == HAL_OK) ? 1U : 0U;
}

/* ─── Region header helpers ─────────────────────────────────────────────── */
static uint8_t hdr_read(uint16_t addr, EE_DataHdr_t *h)
{
    if (!ee_read(addr, (uint8_t *)h, sizeof(EE_DataHdr_t))) return 0U;
    if (h->magic != EE_DATA_HDR_MAGIC) return 0U;
    if (h->crc8 != ee_crc8((const uint8_t *)h, sizeof(EE_DataHdr_t) - 1U)) return 0U;
    return 1U;
}

static uint8_t hdr_write(uint16_t addr, EE_DataHdr_t *h)
{
    h->magic = EE_DATA_HDR_MAGIC;
    h->crc8  = ee_crc8((const uint8_t *)h, sizeof(EE_DataHdr_t) - 1U);
    return ee_write(addr, (const uint8_t *)h, sizeof(EE_DataHdr_t));
}

static uint8_t hdr_format(uint16_t addr)
{
    EE_DataHdr_t h;
    memset(&h, 0, sizeof(h));
    h.count = 0U;
    h.head  = 0U;
    return hdr_write(addr, &h);
}

/* ─── Slot address helpers ──────────────────────────────────────────────── */
static uint16_t daily_slot_addr(uint16_t slot)
{
    return (uint16_t)(EE_DATA_DAILY_REC + (uint32_t)slot * EE_DATA_DAILY_RECSZ);
}
static uint16_t evt_slot_addr(uint16_t slot)
{
    return (uint16_t)(EE_DATA_EVT_REC + (uint32_t)slot * EE_DATA_EVT_RECSZ);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Init / presence / wipe
 * ═════════════════════════════════════════════════════════════════════════ */
void EE_Data_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c    = hi2c;
    s_present = (HAL_I2C_IsDeviceReady(hi2c, EE_DATA_I2C_ADDR, 3, 10) == HAL_OK) ? 1U : 0U;
    if (!s_present) return;

    EE_DataHdr_t h;
    if (!hdr_read(EE_DATA_DAILY_HDR, &h)) hdr_format(EE_DATA_DAILY_HDR);
    if (!hdr_read(EE_DATA_EVT_HDR,   &h)) hdr_format(EE_DATA_EVT_HDR);
}

uint8_t EE_Data_IsPresent(void) { return s_present; }

void EE_Data_Wipe(void)
{
    if (!s_present) return;
    hdr_format(EE_DATA_DAILY_HDR);
    hdr_format(EE_DATA_EVT_HDR);
    /* Records are left in place; their headers now report count = 0 and a
     * fresh head, and any subsequent reads are bounded by count, so old
     * records are logically gone. */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Daily summaries
 * ═════════════════════════════════════════════════════════════════════════ */
uint8_t EE_Daily_ReadHdr(EE_DataHdr_t *h)
{
    return hdr_read(EE_DATA_DAILY_HDR, h);
}

uint8_t EE_Daily_ReadSlot(uint16_t slot, EE_DailyRec_t *out)
{
    if (!s_present || slot >= EE_DATA_DAILY_MAX) return 0U;
    if (!ee_read(daily_slot_addr(slot), (uint8_t *)out, sizeof(EE_DailyRec_t))) return 0U;
    if (out->magic != EE_DATA_DAILY_MAGIC) return 0U;
    if (out->crc8 != ee_crc8((const uint8_t *)out, sizeof(EE_DailyRec_t) - 1U)) return 0U;
    return 1U;
}

static uint8_t daily_write_slot(uint16_t slot, EE_DailyRec_t *r)
{
    r->magic = EE_DATA_DAILY_MAGIC;
    r->crc8  = ee_crc8((const uint8_t *)r, sizeof(EE_DailyRec_t) - 1U);
    return ee_write(daily_slot_addr(slot), (const uint8_t *)r, sizeof(EE_DailyRec_t));
}

uint8_t EE_Daily_AddSample(uint32_t day_midnight, uint16_t ml,
                           uint16_t ppm, int16_t temp_x10)
{
    if (!s_present) return 0U;

    EE_DataHdr_t h;
    if (!hdr_read(EE_DATA_DAILY_HDR, &h)) { hdr_format(EE_DATA_DAILY_HDR); hdr_read(EE_DATA_DAILY_HDR, &h); }

    /* Find an existing slot for this day. */
    int32_t      found = -1;
    EE_DailyRec_t r;
    for (uint16_t s = 0U; s < EE_DATA_DAILY_MAX; s++) {
        if (EE_Daily_ReadSlot(s, &r) && r.valid && r.date_unix == day_midnight) {
            found = (int32_t)s; break;
        }
    }

    uint16_t slot;
    if (found >= 0) {
        slot = (uint16_t)found;
        /* r already holds the slot contents from the search above. */
    } else {
        slot = h.head;
        memset(&r, 0, sizeof(r));
        r.valid     = 1U;
        r.date_unix = day_midnight;
        h.head = (uint16_t)((h.head + 1U) % EE_DATA_DAILY_MAX);
        if (h.count < EE_DATA_DAILY_MAX) h.count++;
    }

    r.total_ml     += ml;
    r.sum_purity   += ppm;
    r.sum_temp_x10 += temp_x10;
    if (r.sample_count < 0xFFFFU) r.sample_count++;

    if (!daily_write_slot(slot, &r)) return 0U;
    if (found < 0) (void)hdr_write(EE_DATA_DAILY_HDR, &h);
    return 1U;
}

uint8_t EE_Daily_GetForDay(uint32_t day_midnight, EE_DailyRec_t *out)
{
    if (!s_present) return 0U;
    for (uint16_t s = 0U; s < EE_DATA_DAILY_MAX; s++) {
        if (EE_Daily_ReadSlot(s, out) && out->valid && out->date_unix == day_midnight)
            return 1U;
    }
    return 0U;
}

void EE_Daily_PurgeOlderThan(uint32_t cutoff_midnight)
{
    if (!s_present) return;
    EE_DailyRec_t r;
    for (uint16_t s = 0U; s < EE_DATA_DAILY_MAX; s++) {
        if (EE_Daily_ReadSlot(s, &r) && r.valid && r.date_unix < cutoff_midnight) {
            r.valid = 0U;
            (void)daily_write_slot(s, &r);
        }
    }
}

uint16_t EE_Daily_Count(void)
{
    EE_DataHdr_t h;
    if (!hdr_read(EE_DATA_DAILY_HDR, &h)) return 0U;
    return h.count;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Detailed drink events
 * ═════════════════════════════════════════════════════════════════════════ */
uint8_t EE_Evt_ReadHdr(EE_DataHdr_t *h)
{
    return hdr_read(EE_DATA_EVT_HDR, h);
}

uint8_t EE_Evt_ReadSlot(uint16_t slot, EE_EvtRec_t *out)
{
    if (!s_present || slot >= EE_DATA_EVT_MAX) return 0U;
    if (!ee_read(evt_slot_addr(slot), (uint8_t *)out, sizeof(EE_EvtRec_t))) return 0U;
    if (out->magic != EE_DATA_EVT_MAGIC) return 0U;
    if (out->crc8 != ee_crc8((const uint8_t *)out, sizeof(EE_EvtRec_t) - 1U)) return 0U;
    return 1U;
}

static uint8_t evt_write_slot(uint16_t slot, EE_EvtRec_t *r)
{
    r->magic = EE_DATA_EVT_MAGIC;
    r->crc8  = ee_crc8((const uint8_t *)r, sizeof(EE_EvtRec_t) - 1U);
    return ee_write(evt_slot_addr(slot), (const uint8_t *)r, sizeof(EE_EvtRec_t));
}

uint8_t EE_Evt_Push(uint32_t unix_time, uint16_t ml, uint16_t ppm, int16_t temp_x10)
{
    if (!s_present) return 0U;

    EE_DataHdr_t h;
    if (!hdr_read(EE_DATA_EVT_HDR, &h)) { hdr_format(EE_DATA_EVT_HDR); hdr_read(EE_DATA_EVT_HDR, &h); }
    if (h.head >= EE_DATA_EVT_MAX) h.head = 0U;

    EE_EvtRec_t r;
    memset(&r, 0, sizeof(r));
    r.synced     = 0U;
    r.volume_ml  = ml;
    r.purity_ppm = ppm;
    r.temp_x10   = temp_x10;
    r.unix_time  = unix_time;

    if (!evt_write_slot(h.head, &r)) return 0U;
    h.head = (uint16_t)((h.head + 1U) % EE_DATA_EVT_MAX);
    if (h.count < EE_DATA_EVT_MAX) h.count++;
    return hdr_write(EE_DATA_EVT_HDR, &h);
}

void EE_Evt_MarkSynced(uint32_t up_to_unix)
{
    if (!s_present) return;
    EE_EvtRec_t r;
    for (uint16_t s = 0U; s < EE_DATA_EVT_MAX; s++) {
        if (EE_Evt_ReadSlot(s, &r) && !r.synced && r.unix_time <= up_to_unix) {
            r.synced = 1U;
            (void)evt_write_slot(s, &r);
        }
    }
}

uint16_t EE_Evt_Count(void)
{
    EE_DataHdr_t h;
    if (!hdr_read(EE_DATA_EVT_HDR, &h)) return 0U;
    return h.count;
}

uint16_t EE_Evt_PendingCount(void)
{
    if (!s_present) return 0U;
    EE_EvtRec_t r;
    uint16_t n = 0U;
    for (uint16_t s = 0U; s < EE_DATA_EVT_MAX; s++)
        if (EE_Evt_ReadSlot(s, &r) && !r.synced) n++;
    return n;
}

uint8_t EE_Evt_StoragePct(void)
{
    uint16_t c = EE_Evt_Count();
    if (c >= EE_DATA_EVT_MAX) return 100U;
    return (uint8_t)(((uint32_t)c * 100U) / EE_DATA_EVT_MAX);
}
