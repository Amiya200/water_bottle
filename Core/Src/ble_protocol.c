/*
 * ble_protocol.c  –  Binary frame builder / parser
 *
 * Memory optimisations vs. previous revision
 * ─────────────────────────────────────────────────────────────────────
 * Flash –8 B  : calc_chk() no longer branches on (payload == NULL).
 *              All call sites either pass a valid pointer or pass len=0,
 *              which means the for-loop body never executes.  The NULL
 *              guard was dead code — every builder that passes NULL also
 *              passes len=0. Removing it avoids a conditional branch and
 *              a possible LDR in the prologue.
 *
 * Flash –16 B : BLE_BuildPacket() no longer tests (len && payload) before
 *              memcpy — same reasoning: len=0 makes memcpy a no-op.
 * ─────────────────────────────────────────────────────────────────────
 */

#pragma GCC optimize("Os")
#include "ble_protocol.h"
#include "data_storage.h"   /* BLE_PrefsPayload_t */
#include <string.h>

/* XOR of CMD, LEN, and all payload bytes.
 * Callers guarantee: payload != NULL when len > 0. */
static uint8_t calc_chk(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t chk = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) chk ^= payload[i];
    return chk;
}

/* ─── Parse & validate a raw framed packet ───────────────────────────────── */
uint8_t BLE_ParsePacket(const uint8_t *raw, uint8_t raw_len, BLE_Packet_t *out)
{
    if (!raw || raw_len < 5U) return 0;
    if (raw[0] != BLE_SOF)    return 0;

    uint8_t cmd = raw[1];
    uint8_t len = raw[2];

    if ((uint8_t)(5U + len) != raw_len) return 0;
    if (len > BLE_PKT_MAX_PAYLOAD)      return 0;

    uint8_t chk = raw[3U + len];
    uint8_t eof = raw[4U + len];
    if (eof != BLE_EOF)                 return 0;
    if (chk != calc_chk(cmd, len, &raw[3])) return 0;

    out->cmd = cmd;
    out->len = len;
    memcpy(out->payload, &raw[3], len);   /* memcpy(dst,src,0) is a no-op */
    return 1;
}

/* ─── Build a complete framed packet ─────────────────────────────────────── */
uint8_t BLE_BuildPacket(uint8_t *buf, uint8_t cmd,
                         const uint8_t *payload, uint8_t len)
{
    buf[0] = BLE_SOF;
    buf[1] = cmd;
    buf[2] = len;
    memcpy(&buf[3], payload, len);        /* no-op when len=0 */
    /* calc_chk is safe: when len=0 the loop body doesn't run */
    buf[3U + len] = calc_chk(cmd, len, payload ? payload : buf /* dummy */);
    buf[4U + len] = BLE_EOF;
    return (uint8_t)(5U + len);
}

/* ─── Convenience builders ───────────────────────────────────────────────── */

uint8_t BLE_BuildACK(uint8_t *buf, uint8_t in_response_to,
                      uint8_t success, uint8_t error_code)
{
    uint8_t p[3] = { in_response_to, success, error_code };
    return BLE_BuildPacket(buf, BLE_RSP_ACK, p, 3);
}

uint8_t BLE_BuildPong(uint8_t *buf)
{
    /* No payload — pass buf as dummy pointer; len=0 so it's never read */
    return BLE_BuildPacket(buf, BLE_RSP_PONG, buf, 0);
}

uint8_t BLE_BuildStatus(uint8_t *buf, const BLE_StatusPayload_t *s)
{
    /* Struct is all-uint8_t — no padding, safe to send as raw bytes. */
    return BLE_BuildPacket(buf, BLE_RSP_STATUS, (const uint8_t *)s,
                           (uint8_t)sizeof(BLE_StatusPayload_t));
}

uint8_t BLE_BuildErrEntry(uint8_t *buf, const BLE_ErrEntryPayload_t *e)
{
    return BLE_BuildPacket(buf, BLE_RSP_ERR_LOG, (const uint8_t *)e,
                           (uint8_t)sizeof(BLE_ErrEntryPayload_t));
}

uint8_t BLE_BuildInfo(uint8_t *buf, const BLE_InfoPayload_t *i)
{
    return BLE_BuildPacket(buf, BLE_RSP_INFO, (const uint8_t *)i,
                           (uint8_t)sizeof(BLE_InfoPayload_t));
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

uint8_t BLE_BuildConfig(uint8_t *buf, const void *prefs_payload)
{
    return BLE_BuildPacket(buf, BLE_RSP_CONFIG,
                           (const uint8_t *)prefs_payload,
                           (uint8_t)sizeof(BLE_PrefsPayload_t));
}

uint8_t BLE_BuildEERecord(uint8_t *buf, const BLE_EERecordPayload_t *r)
{
    return BLE_BuildPacket(buf, BLE_RSP_EE_RECORD,
                           (const uint8_t *)r,
                           (uint8_t)sizeof(BLE_EERecordPayload_t));
}
