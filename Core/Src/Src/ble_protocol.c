#pragma GCC optimize("Os")
#include "ble_protocol.h"
#include <string.h>

/* ─── Checksum: XOR of CMD, LEN, and all payload bytes ─────── */
static uint8_t calc_chk(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t chk = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) chk ^= payload[i];
    return chk;
}

/* ─── Parse & validate a raw framed packet ──────────────────── */
uint8_t BLE_ParsePacket(const uint8_t *raw, uint8_t raw_len, BLE_Packet_t *out)
{
    /* Minimum frame: SOF + CMD + LEN + CHK + EOF = 5 bytes */
    if (!raw || raw_len < 5) return 0;
    if (raw[0] != BLE_SOF)   return 0;

    uint8_t cmd = raw[1];
    uint8_t len = raw[2];

    /* Sanity: total frame must be raw_len */
    if ((uint8_t)(5U + len) != raw_len) return 0;
    if (len > BLE_PKT_MAX_PAYLOAD)      return 0;

    uint8_t chk = raw[3 + len];
    uint8_t eof = raw[4 + len];
    if (eof != BLE_EOF) return 0;
    if (chk != calc_chk(cmd, len, &raw[3])) return 0;

    out->cmd = cmd;
    out->len = len;
    if (len) memcpy(out->payload, &raw[3], len);
    return 1;
}

/* ─── Build a complete framed packet ────────────────────────── */
uint8_t BLE_BuildPacket(uint8_t *buf, uint8_t cmd,
                         const uint8_t *payload, uint8_t len)
{
    buf[0] = BLE_SOF;
    buf[1] = cmd;
    buf[2] = len;
    if (len && payload) memcpy(&buf[3], payload, len);
    buf[3 + len] = calc_chk(cmd, len, payload ? payload : (const uint8_t *)"");
    buf[4 + len] = BLE_EOF;
    return (uint8_t)(5U + len);
}

/* ─── Convenience builders ───────────────────────────────────── */

uint8_t BLE_BuildACK(uint8_t *buf, uint8_t in_response_to,
                      uint8_t success, uint8_t error_code)
{
    uint8_t p[3] = { in_response_to, success, error_code };
    return BLE_BuildPacket(buf, BLE_RSP_ACK, p, 3);
}

uint8_t BLE_BuildPong(uint8_t *buf)
{
    return BLE_BuildPacket(buf, BLE_RSP_PONG, NULL, 0);
}

uint8_t BLE_BuildStatus(uint8_t *buf, const BLE_StatusPayload_t *s)
{
    uint8_t p[3] = { s->bat_pct, s->flags, s->storage_pct };
    return BLE_BuildPacket(buf, BLE_RSP_STATUS, p, 3);
}

uint8_t BLE_BuildLogEntry(uint8_t *buf, const BLE_LogEntryPayload_t *e)
{
    return BLE_BuildPacket(buf, BLE_RSP_LOG_ENTRY,
                           (const uint8_t *)e,
                           (uint8_t)sizeof(BLE_LogEntryPayload_t));
}

uint8_t BLE_BuildDaily(uint8_t *buf, const BLE_DailyPayload_t *d)
{
    return BLE_BuildPacket(buf, BLE_RSP_DAILY,
                           (const uint8_t *)d,
                           (uint8_t)sizeof(BLE_DailyPayload_t));
}

uint8_t BLE_BuildConfig(uint8_t *buf, const BLE_PrefsPayload_t *p)
{
    return BLE_BuildPacket(buf, BLE_RSP_CONFIG,
                           (const uint8_t *)p,
                           (uint8_t)sizeof(BLE_PrefsPayload_t));
}
