#pragma GCC optimize("Os")
#include "main.h"           /* HAL types, FLASH_EraseInitTypeDef, HAL_OK, etc. */
#include "data_storage.h"
#include <string.h>

/* ─── Flash page helpers ─────────────────────────────────────────────────── */
static HAL_StatusTypeDef Flash_ErasePage(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_err;
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = page_addr;
    erase.NbPages     = 1;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef s = HAL_FLASHEx_Erase(&erase, &page_err);
    HAL_FLASH_Lock();
    return s;
}

static HAL_StatusTypeDef Flash_WriteBytes(uint32_t addr,
                                           const uint8_t *data, uint32_t len)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef s = HAL_OK;
    for (uint32_t i = 0; i < len; i += 2) {
        uint16_t hw = (uint16_t)data[i];
        if (i + 1 < len) hw |= (uint16_t)((uint16_t)data[i + 1] << 8);
        s = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, hw);
        if (s != HAL_OK) break;
    }
    HAL_FLASH_Lock();
    return s;
}

static void Flash_ReadBytes(uint32_t addr, uint8_t *out, uint32_t len)
{
    memcpy(out, (const void *)addr, len);
}

/* ─── CRC32 (software, polynomial 0xEDB88320) ───────────────────────────── */
uint32_t Storage_CRC32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
    return ~crc;
}

/* ─── Default preferences ────────────────────────────────────────────────── */
void Storage_DefaultPrefs(BLE_PrefsPayload_t *p)
{
    p->purity_goal_hi  = 0x01; p->purity_goal_lo  = 0xF4;  /* 500 ppm      */
    p->temp_goal_hi    = 0x01; p->temp_goal_lo    = 0x90;  /* 40.0 °C      */
    p->hydration_hi    = 0x07; p->hydration_lo    = 0xD0;  /* 2000 ml      */
    p->remind_h_start  = 8;    p->remind_m_start  = 0;
    p->remind_h_end    = 20;   p->remind_m_end    = 0;
    p->remind_freq_min = 60;
    p->remind_r        = 0;    p->remind_g        = 255;   p->remind_b = 0;
    p->remind_sound    = 2;    /* double_beep */
    p->lamp_r          = 255;  p->lamp_g          = 255;   p->lamp_b   = 255;
}

/* ─── Init ───────────────────────────────────────────────────────────────── */
void Storage_Init(void) { /* flash always readable, nothing to do */ }

/* ─── Settings ───────────────────────────────────────────────────────────── */
void Storage_LoadSettings(DeviceSettings_t *out)
{
    Flash_ReadBytes(STORAGE_SETTINGS_ADDR, (uint8_t *)out, sizeof(DeviceSettings_t));
    if (out->magic != SETTINGS_MAGIC) {
        memset(out, 0, sizeof(DeviceSettings_t));
        Storage_DefaultPrefs(&out->prefs);
        out->hx711_scale_x100 = 44000L;
        out->tare_offset       = 0L;
        out->is_registered = 0;
        out->is_calibrated = 0;
    }
}

void Storage_SaveSettings(const DeviceSettings_t *in)
{
    DeviceSettings_t tmp;
    memcpy(&tmp, in, sizeof(DeviceSettings_t));
    tmp.magic = SETTINGS_MAGIC;
    tmp.crc   = Storage_CRC32((const uint8_t *)&tmp,
                               (uint16_t)(sizeof(DeviceSettings_t) - 4U));
    Flash_ErasePage(STORAGE_SETTINGS_ADDR);
    Flash_WriteBytes(STORAGE_SETTINGS_ADDR, (const uint8_t *)&tmp,
                     sizeof(DeviceSettings_t));
}

void Storage_EraseSettings(void)
{
    Flash_ErasePage(STORAGE_SETTINGS_ADDR);
}

/* ─── Drink log ──────────────────────────────────────────────────────────── */
void Storage_AddDrinkEvent(DrinkLog_t *log, const DrinkEvent_t *ev)
{
    if (log->count >= STORAGE_MAX_DRINK_EVENTS) {
        /* Oldest-first eviction: shift all events down by one */
        memmove(&log->events[0], &log->events[1],
                sizeof(DrinkEvent_t) * (STORAGE_MAX_DRINK_EVENTS - 1U));
        log->count = STORAGE_MAX_DRINK_EVENTS - 1U;
    }
    log->events[log->count++] = *ev;
    log->dirty = 1;
}

void Storage_FlushDrinkLog(DrinkLog_t *log)
{
    if (!log->dirty) return;
    Flash_ErasePage(STORAGE_LOG_A_ADDR);
    Flash_WriteBytes(STORAGE_LOG_A_ADDR, (const uint8_t *)log,
                     sizeof(DrinkLog_t));
    log->dirty = 0;
}

void Storage_LoadDrinkLog(DrinkLog_t *log)
{
    Flash_ReadBytes(STORAGE_LOG_A_ADDR, (uint8_t *)log, sizeof(DrinkLog_t));
    if (log->count > STORAGE_MAX_DRINK_EVENTS)
        memset(log, 0, sizeof(DrinkLog_t));
}

void Storage_MarkSynced(DrinkLog_t *log, uint32_t synced_up_to_unix)
{
    for (uint8_t i = 0; i < log->count; i++)
        if (log->events[i].unix_time <= synced_up_to_unix)
            log->events[i].synced = 1;
    log->dirty = 1;
}

/* FRD §7.1 — free space after a successful sync. Today's events are kept
 * (cutoff = today's midnight) because Storage_UpdateDailySummary recomputes
 * the current day's totals from the detailed log. */
void Storage_PurgeSyncedBefore(DrinkLog_t *log, uint32_t cutoff_unix)
{
    uint8_t kept = 0;
    for (uint8_t i = 0; i < log->count; i++) {
        if (log->events[i].synced && log->events[i].unix_time < cutoff_unix)
            continue;                      /* synced + old → drop            */
        if (kept != i) log->events[kept] = log->events[i];
        kept++;
    }
    if (kept != log->count) {
        log->count = kept;
        log->dirty = 1;
    }
}

/* ─── Daily summary ──────────────────────────────────────────────────────── */
void Storage_UpdateDailySummary(DailySummaryLog_t *daily,
                                 DrinkLog_t        *log,
                                 uint32_t           today_unix)
{
    uint32_t midnight = (today_unix / 86400UL) * 86400UL;
    int8_t   today_idx = -1;

    for (uint8_t i = 0; i < daily->count; i++) {
        if (daily->days[i].date_unix == midnight) {
            today_idx = (int8_t)i;
            break;
        }
    }

    if (today_idx < 0) {
        if (daily->count >= STORAGE_MAX_DAILY_DAYS) {
            memmove(&daily->days[0], &daily->days[1],
                    sizeof(DailySummary_t) * (STORAGE_MAX_DAILY_DAYS - 1U));
            daily->count = STORAGE_MAX_DAILY_DAYS - 1U;
        }
        today_idx = (int8_t)daily->count++;
        memset(&daily->days[today_idx], 0, sizeof(DailySummary_t));
        daily->days[today_idx].date_unix = midnight;
        daily->days[today_idx].valid     = 1;
    }

    uint32_t total_ml = 0, sum_ppm = 0;
    int32_t  sum_temp = 0;
    uint16_t cnt      = 0;

    for (uint8_t i = 0; i < log->count; i++) {
        if (log->events[i].unix_time >= midnight &&
            log->events[i].unix_time <  midnight + 86400UL) {
            total_ml += log->events[i].volume_ml;
            sum_ppm  += log->events[i].purity_ppm;
            sum_temp += log->events[i].temp_x10;   /* int16_t as defined */
            cnt++;
        }
    }

    daily->days[today_idx].total_ml       = (uint16_t)total_ml;
    daily->days[today_idx].avg_purity_ppm = cnt ? (uint16_t)(sum_ppm / cnt)  : 0U;
    daily->days[today_idx].avg_temp_x10   = cnt ? (int16_t)(sum_temp / cnt) : 0;
    daily->dirty = 1;
}

void Storage_FlushDailySummary(DailySummaryLog_t *daily)
{
    if (!daily->dirty) return;
    Flash_ErasePage(STORAGE_DAILY_ADDR);
    Flash_WriteBytes(STORAGE_DAILY_ADDR, (const uint8_t *)daily,
                     sizeof(DailySummaryLog_t));
    daily->dirty = 0;
}

void Storage_LoadDailySummary(DailySummaryLog_t *daily)
{
    Flash_ReadBytes(STORAGE_DAILY_ADDR, (uint8_t *)daily,
                    sizeof(DailySummaryLog_t));
    if (daily->count > STORAGE_MAX_DAILY_DAYS)
        memset(daily, 0, sizeof(DailySummaryLog_t));
}

void Storage_PurgeDailySummaryOlderThan(DailySummaryLog_t *daily,
                                         uint32_t           cutoff_unix)
{
    uint8_t new_count = 0;
    for (uint8_t i = 0; i < daily->count; i++)
        if (daily->days[i].date_unix >= cutoff_unix)
            daily->days[new_count++] = daily->days[i];
    daily->count = new_count;
    daily->dirty = 1;
}

void Storage_FactoryReset(void)
{
    Flash_ErasePage(STORAGE_SETTINGS_ADDR);
    Flash_ErasePage(STORAGE_LOG_A_ADDR);
    Flash_ErasePage(STORAGE_LOG_B_ADDR);
    Flash_ErasePage(STORAGE_DAILY_ADDR);
}
