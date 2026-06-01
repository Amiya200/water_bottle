#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/*
 * ─── Compact Binary BLE Packet Protocol ─────────────────────────────────────
 *
 * Replaces the JSON-based protocol entirely.  Eliminates sscanf, atof, atol,
 * strstr and all large format strings — significant flash savings.
 *
 * Framing: [SOF=0xAA] [CMD] [LEN] [payload × LEN bytes] [XOR_CHK] [EOF=0x55]
 *   • CHK = CMD ^ LEN ^ payload[0] ^ … ^ payload[LEN-1]
 *   • Max payload: BLE_PKT_MAX_PAYLOAD bytes
 *   • Max total packet: BLE_PKT_MAX_LEN bytes
 *
 * App → Device CMDs (0x01–0x0F)
 * Device → App  RSPs (0x80–0x8F)
 */

/* ─── Framing constants ──────────────────────────────────────── */
#define BLE_SOF              0xAAU
#define BLE_EOF              0x55U
#define BLE_PKT_MAX_PAYLOAD  32U
#define BLE_PKT_MAX_LEN      (5U + BLE_PKT_MAX_PAYLOAD)  /* SOF+CMD+LEN+payload+CHK+EOF */

/* ─── Command IDs — App → Device ────────────────────────────── */
#define BLE_CMD_TIMESTAMP    0x01U   /* payload: unix_time (4) */
#define BLE_CMD_INPUT_DATA   0x02U   /* payload: BLE_PrefsPayload_t (18) */
#define BLE_CMD_CALIBRATION  0x03U   /* payload: stage (1): 0=empty_bottle */
#define BLE_CMD_LAMP_MODE    0x04U   /* payload: enable(1)+r+g+b (4 total) */
#define BLE_CMD_SOFT_RESET   0x05U   /* no payload */
#define BLE_CMD_FACTORY_RESET 0x06U  /* no payload */
#define BLE_CMD_GET_HISTORY  0x07U   /* no payload */
#define BLE_CMD_REGISTER     0x08U   /* payload: user_id[16]+nickname[16] (32) */
#define BLE_CMD_UNPAIR       0x09U   /* no payload */
#define BLE_CMD_GET_LOGS     0x0AU   /* payload: from_unix(4)+to_unix(4) (8) */
#define BLE_CMD_SYNC_ACK     0x0BU   /* payload: synced_up_to_unix (4) */
#define BLE_CMD_GET_STATUS   0x0CU   /* no payload */
#define BLE_CMD_GET_CONFIG   0x0DU   /* no payload */
#define BLE_CMD_GET_ERRORS   0x0EU   /* no payload */
#define BLE_CMD_PING         0x0FU   /* no payload */

/* ─── Response IDs — Device → App ───────────────────────────── */
#define BLE_RSP_ACK          0x80U   /* BLE_AckPayload_t (3) */
#define BLE_RSP_PONG         0x81U   /* no payload */
#define BLE_RSP_STATUS       0x82U   /* BLE_StatusPayload_t (3) */
#define BLE_RSP_LOG_ENTRY    0x83U   /* BLE_LogEntryPayload_t (11) */
#define BLE_RSP_DAILY        0x84U   /* BLE_DailyPayload_t (10) */
#define BLE_RSP_CONFIG       0x85U   /* BLE_PrefsPayload_t (18) */
#define BLE_RSP_ERR_LOG      0x86U   /* error count (1) */

/* ─── Error codes (used in ACK) ─────────────────────────────── */
#define BLE_ERR_OK              0U
#define BLE_ERR_UNKNOWN_CMD     1U
#define BLE_ERR_NOT_CHARGING    2U
#define BLE_ERR_INVALID_STAGE   3U
#define BLE_ERR_OTA_UNSUPPORTED 4U

/* ─── Status flags byte (BLE_RSP_STATUS) ────────────────────── */
#define BLE_FLAG_CHARGING     (1U << 0)
#define BLE_FLAG_TEMP_OK      (1U << 1)
#define BLE_FLAG_TDS_OK       (1U << 2)
#define BLE_FLAG_WEIGHT_OK    (1U << 3)
#define BLE_FLAG_CALIBRATED   (1U << 4)
#define BLE_FLAG_REGISTERED   (1U << 5)

/* ─── Packed payload structs (no padding — use byte-by-byte copy) ── */

/* INPUT_DATA / CONFIG payload: 18 bytes */
typedef struct {
    uint8_t purity_goal_hi;      /* purity_goal_ppm MSB */
    uint8_t purity_goal_lo;      /* purity_goal_ppm LSB */
    uint8_t temp_goal_hi;        /* temp_goal_x10 MSB (int16_t, e.g. 400=40.0°C) */
    uint8_t temp_goal_lo;        /* temp_goal_x10 LSB */
    uint8_t hydration_hi;        /* hydration_goal_ml MSB */
    uint8_t hydration_lo;        /* hydration_goal_ml LSB */
    uint8_t remind_h_start;
    uint8_t remind_m_start;
    uint8_t remind_h_end;
    uint8_t remind_m_end;
    uint8_t remind_freq_min;     /* minutes, 0 = disabled */
    uint8_t remind_r;
    uint8_t remind_g;
    uint8_t remind_b;
    uint8_t remind_sound;        /* 0=none, 1=single_beep, 2=double_beep */
    uint8_t lamp_r;
    uint8_t lamp_g;
    uint8_t lamp_b;
} BLE_PrefsPayload_t;            /* 18 bytes, NO implicit padding */

/* ACK response: 3 bytes */
typedef struct {
    uint8_t in_response_to;
    uint8_t success;             /* 0 or 1 */
    uint8_t error_code;          /* BLE_ERR_* */
} BLE_AckPayload_t;

/* STATUS response: 3 bytes */
typedef struct {
    uint8_t bat_pct;             /* 0-100 */
    uint8_t flags;               /* BLE_FLAG_* bitmap */
    uint8_t storage_pct;         /* 0-100 */
} BLE_StatusPayload_t;

/* LOG_ENTRY response: 11 bytes */
typedef struct {
    uint8_t unix_b3, unix_b2, unix_b1, unix_b0;  /* big-endian */
    uint8_t vol_hi, vol_lo;      /* volume_ml */
    uint8_t ppm_hi, ppm_lo;      /* purity_ppm */
    uint8_t temp_hi, temp_lo;    /* temp_x10 (int16_t signed) */
    uint8_t synced;
} BLE_LogEntryPayload_t;

/* DAILY response: 10 bytes */
typedef struct {
    uint8_t unix_b3, unix_b2, unix_b1, unix_b0;  /* big-endian date_unix */
    uint8_t ml_hi, ml_lo;        /* total_ml */
    uint8_t ppm_hi, ppm_lo;      /* avg_purity_ppm */
    uint8_t temp_hi, temp_lo;    /* avg_temp_x10 */
} BLE_DailyPayload_t;

/* ─── Parsed incoming packet ─────────────────────────────────── */
typedef struct {
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  payload[BLE_PKT_MAX_PAYLOAD];
} BLE_Packet_t;

/* ─── API ───────────────────────────────────────────────────── */

/* Validate and copy a raw frame into BLE_Packet_t. Returns 1 on success. */
uint8_t BLE_ParsePacket(const uint8_t *raw, uint8_t raw_len, BLE_Packet_t *out);

/* Build a complete framed packet into buf.  Returns total packet length. */
uint8_t BLE_BuildPacket(uint8_t *buf, uint8_t cmd, const uint8_t *payload, uint8_t len);

/* Convenience builders — return total packet byte count */
uint8_t BLE_BuildACK(uint8_t *buf, uint8_t in_response_to,
                     uint8_t success, uint8_t error_code);
uint8_t BLE_BuildPong(uint8_t *buf);
uint8_t BLE_BuildStatus(uint8_t *buf, const BLE_StatusPayload_t *s);
uint8_t BLE_BuildLogEntry(uint8_t *buf, const BLE_LogEntryPayload_t *e);
uint8_t BLE_BuildDaily(uint8_t *buf, const BLE_DailyPayload_t *d);
uint8_t BLE_BuildConfig(uint8_t *buf, const BLE_PrefsPayload_t *p);

/* ─── Helper: extract uint16_t from two bytes (big-endian) ──── */
static inline uint16_t BLE_U16(uint8_t hi, uint8_t lo)
{
    return (uint16_t)((uint16_t)hi << 8 | lo);
}
static inline int16_t BLE_I16(uint8_t hi, uint8_t lo)
{
    return (int16_t)BLE_U16(hi, lo);
}
static inline uint32_t BLE_U32(uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0)
{
    return ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) |
           ((uint32_t)b1 <<  8) |  (uint32_t)b0;
}

#endif /* BLE_PROTOCOL_H */
