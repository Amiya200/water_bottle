#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

/*
 * ble_protocol.h  –  Binary frame format for HydraSense BLE protocol
 *
 * RAM optimisation
 * ─────────────────────────────────────────────────────────────────────
 * –12 B RAM : BLE_PKT_MAX_PAYLOAD reduced 27 → 15.
 *             The largest payload this device sends or receives is the
 *             BLE_PrefsPayload_t at 18 bytes — BUT that is sent as a
 *             BLE_CMD_INPUT_DATA command where the firmware only reads
 *             it, not buffers the full raw frame.  For outgoing frames
 *             the largest is BLE_BuildStatus (3 bytes) or BLE_BuildACK
 *             (3 bytes).  For incoming commands the largest payload is
 *             BLE_CMD_REGISTER at 32 bytes — HOWEVER the firmware only
 *             needs to process 32 bytes of payload for registration,
 *             and this is the sole command with a large payload.
 *
 *             Practical maximum incoming payload is BLE_CMD_INPUT_DATA
 *             at sizeof(BLE_PrefsPayload_t) = 18 bytes.  We set
 *             BLE_PKT_MAX_PAYLOAD = 20 to cover 18 + 2 bytes headroom.
 *             BLE_PKT_MAX_LEN = 5 + 20 = 25 bytes total frame.
 *
 *             This shrinks BLE_Handle_t.pkt_buf by 7 bytes and
 *             BLE_Packet_t.payload by 7 bytes wherever it appears on
 *             the stack.
 * ─────────────────────────────────────────────────────────────────────
 */

#include <stdint.h>

/* ─── Frame constants ────────────────────────────────────────────────────── */
#define BLE_SOF                 0xAAU
#define BLE_EOF                 0x55U

/*
 * BLE_PKT_MAX_PAYLOAD: largest payload in bytes.
 * BLE_PKT_MAX_LEN: total frame bytes = SOF+CMD+LEN+payload+CHK+EOF = 5+payload.
 *
 * Was: BLE_PKT_MAX_PAYLOAD = 27, BLE_PKT_MAX_LEN = 32.
 * Now: BLE_PKT_MAX_PAYLOAD = 20, BLE_PKT_MAX_LEN = 25.
 * Saving: 7 bytes in BLE_Handle_t.pkt_buf + 7 bytes in stack BLE_Packet_t.
 *
 * NOTE: BLE_PKT_MAX_LEN is derived here — do NOT redefine it in other
 * headers (ble_jdy29.h no longer defines it).
 */
/* PRD v2 FIX: BLE_CMD_REGISTER carries 16 B user_id + 16 B nickname = 32 B.
 * With MAX_PAYLOAD = 20 the parser rejected every registration frame and the
 * nickname branch in App_Cmd_RegisterDevice was dead code. Registration is a
 * mandatory first-time-setup flow (FRD §4), so the limit is 32.
 * Cost: +12 B in BLE_Handle_t.pkt_buf and +12 B per stack BLE_Packet_t. */
#define BLE_PKT_MAX_PAYLOAD     32U
#define BLE_PKT_MAX_LEN         (5U + BLE_PKT_MAX_PAYLOAD)   /* = 37 */

/* ─── Firmware identity (FRD §2.2 command metadata) ─────────────────── */
#define HYDRA_FW_VER_MAJOR      2U
#define HYDRA_FW_VER_MINOR      0U
#define HYDRA_FW_VER_PATCH      0U
#define HYDRA_MODEL_ID          0x01U   /* HydraSense-Pro-v1 */
#define HYDRA_PROTO_VER         2U      /* this command set  */

/* ─── Command IDs (host → device) ───────────────────────────────────────── */
#define BLE_CMD_TIMESTAMP       0x01U
#define BLE_CMD_INPUT_DATA      0x02U
#define BLE_CMD_CALIBRATION     0x03U
#define BLE_CMD_LAMP_MODE       0x04U
#define BLE_CMD_SOFT_RESET      0x05U
#define BLE_CMD_FACTORY_RESET   0x06U
#define BLE_CMD_GET_HISTORY     0x07U
#define BLE_CMD_REGISTER        0x08U
#define BLE_CMD_UNPAIR          0x09U
#define BLE_CMD_GET_LOGS        0x0AU
#define BLE_CMD_SYNC_ACK        0x0BU
#define BLE_CMD_GET_STATUS      0x0CU
#define BLE_CMD_GET_CONFIG      0x0DU
#define BLE_CMD_GET_ERRORS      0x0EU
#define BLE_CMD_PING            0x0FU
#define BLE_CMD_MEASURE         0x10U   /* load cell → conditional TDS/temp → store */
#define BLE_CMD_STORE_WEIGHT    0x11U   /* force-store a weight record              */
#define BLE_CMD_DUMP_EEPROM     0x12U   /* stream all EEPROM records                */
#define BLE_CMD_FW_UPDATE       0x13U   /* FRD FIRMWARE_UPDATE — OTA notification   */
#define BLE_CMD_GET_INFO        0x14U   /* fw version / model identity (FRD §2.2)   */

/* ─── Response IDs (device → host) ──────────────────────────────────────── */
#define BLE_RSP_ACK             0x80U
#define BLE_RSP_STATUS          0x81U
#define BLE_RSP_LOG_ENTRY       0x82U
#define BLE_RSP_DAILY           0x83U
#define BLE_RSP_CONFIG          0x84U
#define BLE_RSP_PONG            0x85U
#define BLE_RSP_ERR_LOG         0x86U
#define BLE_RSP_EE_RECORD       0x87U   /* one EEPROM measurement record */
#define BLE_RSP_MEASURE         0x88U   /* MEASURE/STORE_WEIGHT result   */
#define BLE_RSP_INFO            0x89U   /* reply to BLE_CMD_GET_INFO     */

/* ─── Error codes ────────────────────────────────────────────────────────── */
#define BLE_ERR_OK              0x00U
#define BLE_ERR_UNKNOWN_CMD     0x01U
#define BLE_ERR_INVALID_STAGE   0x02U
#define BLE_ERR_NOT_CHARGING    0x03U
#define BLE_ERR_UNSUPPORTED     0x04U   /* command known but not available  */
#define BLE_ERR_INVALID_VALUE   0x05U   /* FRD ACK example: range-check fail */

/* ─── Status flags ───────────────────────────────────────────────────────── */
#define BLE_FLAG_CHARGING       0x01U
#define BLE_FLAG_TEMP_OK        0x02U
#define BLE_FLAG_TDS_OK         0x04U
#define BLE_FLAG_WEIGHT_OK      0x08U
#define BLE_FLAG_CALIBRATED     0x10U
#define BLE_FLAG_REGISTERED     0x20U

/* ─── Byte-order helpers ─────────────────────────────────────────────────── */
#define BLE_U16(hi, lo)         ((uint16_t)(((uint16_t)(hi) << 8) | (lo)))
#define BLE_I16(hi, lo)         ((int16_t)BLE_U16(hi, lo))
#define BLE_U32(b3,b2,b1,b0)   (((uint32_t)(b3)<<24)|((uint32_t)(b2)<<16)|\
                                 ((uint32_t)(b1)<<8)|(uint32_t)(b0))

/* ─── Packet struct ──────────────────────────────────────────────────────── */
typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[BLE_PKT_MAX_PAYLOAD];  /* 20 bytes (was 27) */
} BLE_Packet_t;

/* ─── Payload structs ────────────────────────────────────────────────────── */
/* FRD DEVICE_STATUS response: battery, charging/sensor/calibration flags,
 * storage use, last_sync timestamp and running firmware version. */
typedef struct {
    uint8_t bat_pct;
    uint8_t flags;
    uint8_t storage_pct;
    uint8_t sync_b3, sync_b2, sync_b1, sync_b0;  /* last_sync unix; 0 = never since boot */
    uint8_t fw_major, fw_minor, fw_patch;
} BLE_StatusPayload_t;                            /* 10 bytes */

/* One internal error-log entry (FRD ERROR_LOG). */
typedef struct {
    uint8_t unix_b3, unix_b2, unix_b1, unix_b0;
    uint8_t code;                                 /* APP_ERR_* */
} BLE_ErrEntryPayload_t;                          /* 5 bytes */

/* GET_INFO reply (FRD §2.2 metadata: fw version + model for compat checks). */
typedef struct {
    uint8_t fw_major, fw_minor, fw_patch;
    uint8_t model_id;
    uint8_t proto_ver;
    uint8_t max_daily_days;                       /* 30 (PRD §7.1)           */
    uint8_t max_drink_events;
} BLE_InfoPayload_t;                              /* 7 bytes */

typedef struct {
    uint8_t unix_b3, unix_b2, unix_b1, unix_b0;
    uint8_t vol_hi,  vol_lo;
    uint8_t ppm_hi,  ppm_lo;
    uint8_t temp_hi, temp_lo;
    uint8_t synced;
} BLE_LogEntryPayload_t;

typedef struct {
    uint8_t unix_b3, unix_b2, unix_b1, unix_b0;
    uint8_t ml_hi,   ml_lo;
    uint8_t ppm_hi,  ppm_lo;
    uint8_t temp_hi, temp_lo;
} BLE_DailyPayload_t;

/* One EEPROM measurement record streamed by BLE_CMD_DUMP_EEPROM.
 * weight is signed 32-bit (ml ≈ g); temp is signed ×10. */
typedef struct {
    uint8_t idx_hi,  idx_lo;
    uint8_t unix_b3, unix_b2, unix_b1, unix_b0;
    uint8_t w_b3,    w_b2,    w_b1,    w_b0;
    uint8_t ppm_hi,  ppm_lo;
    uint8_t temp_hi, temp_lo;
    uint8_t flags;
} BLE_EERecordPayload_t;   /* 15 bytes (fits BLE_PKT_MAX_PAYLOAD) */

/* BLE_PrefsPayload_t is defined in data_storage.h to avoid circular deps */

/* ─── API ────────────────────────────────────────────────────────────────── */
uint8_t BLE_ParsePacket(const uint8_t *raw, uint8_t raw_len, BLE_Packet_t *out);
uint8_t BLE_BuildPacket(uint8_t *buf, uint8_t cmd,
                         const uint8_t *payload, uint8_t len);
uint8_t BLE_BuildACK(uint8_t *buf, uint8_t in_response_to,
                      uint8_t success, uint8_t error_code);
uint8_t BLE_BuildPong(uint8_t *buf);
uint8_t BLE_BuildStatus(uint8_t *buf, const BLE_StatusPayload_t *s);
uint8_t BLE_BuildLogEntry(uint8_t *buf, const BLE_LogEntryPayload_t *e);
uint8_t BLE_BuildDaily(uint8_t *buf, const BLE_DailyPayload_t *d);
uint8_t BLE_BuildEERecord(uint8_t *buf, const BLE_EERecordPayload_t *r);
uint8_t BLE_BuildErrEntry(uint8_t *buf, const BLE_ErrEntryPayload_t *e);
uint8_t BLE_BuildInfo(uint8_t *buf, const BLE_InfoPayload_t *i);

/* BLE_BuildConfig needs BLE_PrefsPayload_t — declared in data_storage.h */
struct BLE_PrefsPayload_t;
uint8_t BLE_BuildConfig(uint8_t *buf, const void *prefs_payload);

#endif /* BLE_PROTOCOL_H */
