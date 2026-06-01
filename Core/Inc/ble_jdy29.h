#ifndef BLE_JDY29_H
#define BLE_JDY29_H

#include "stm32f0xx_hal.h"
#include "ble_protocol.h"   /* BLE_Packet_t, BLE_PKT_MAX_LEN, BLE_SOF, BLE_EOF */
#include <stdint.h>

/* JDY-29 BLE 5.2 module
 * UART1  → PA9 (TX), PA10 (RX) @ 115200 8N1
 * BLE_STATE → PA15 (input): HIGH = connected, LOW = disconnected
 *
 * Operates as a transparent UART bridge over BLE.
 * AT commands are used only during initial configuration (baud, name).
 * All application data flows as binary framed packets (see ble_protocol.h).
 *
 * ─── RAM budget ────────────────────────────────────────────────────────────
 * Old struct (JSON era):  rx_buf[512] + json_buf[512] = 1024 bytes wasted
 * New struct (binary):    pkt_buf[37] + a few bytes   =   ~45 bytes total
 * Saving: ~980 bytes of RAM — resolves the 56-byte overflow with headroom.
 */

typedef enum {
    BLE_STATE_DISCONNECTED = 0,
    BLE_STATE_CONNECTED,
} BLE_ConnState_t;

/* Max length of an ASCII command line (e.g. "RGB,255,128,0"). */
#define BLE_STR_LINE_MAX  48U

typedef struct {
    UART_HandleTypeDef *huart;

    /* Single-byte interrupt-driven receive staging */
    uint8_t  rx_byte;

    /* Binary packet frame accumulator (replaces old 1024-byte JSON buffers) */
    uint8_t  pkt_buf[BLE_PKT_MAX_LEN];  /* max 37 bytes: SOF+CMD+LEN+32+CHK+EOF */
    uint8_t  pkt_len;                   /* bytes accumulated so far               */
    uint8_t  pkt_in_frame;              /* 1 = SOF seen, accumulating             */
    uint8_t  pkt_ready;                 /* 1 = complete frame waiting to be read  */

    /* ASCII string-command accumulator (runs in PARALLEL with binary frames).
     * Lets a plain serial terminal send "PING", "RED", "TEMP" etc. terminated
     * by CR or LF. Binary frames (starting 0xAA) are unaffected. */
    char     line_buf[BLE_STR_LINE_MAX]; /* assembled ASCII command line          */
    uint8_t  line_len;                   /* chars accumulated                     */
    uint8_t  line_ready;                 /* 1 = complete line waiting to be read  */

    BLE_ConnState_t conn_state;
} BLE_Handle_t;

/* ─── Core API ───────────────────────────────────────────────── */
void    BLE_Init(BLE_Handle_t *hble, UART_HandleTypeDef *huart);
void    BLE_StartReceive(BLE_Handle_t *hble);
void    BLE_RxISR(BLE_Handle_t *hble);     /* call from HAL_UART_RxCpltCallback */

/* Connection state */
uint8_t BLE_IsConnected(BLE_Handle_t *hble);
void    BLE_PollState(BLE_Handle_t *hble);

/* Binary packet I/O — used by app_logic.c */
uint8_t           BLE_GetPacket(BLE_Handle_t *hble, BLE_Packet_t *out);
HAL_StatusTypeDef BLE_SendPacket(BLE_Handle_t *hble,
                                  const uint8_t *buf, uint8_t len);

/* ASCII string-command I/O (parallel to binary).
 * BLE_GetLine: returns 1 and copies a complete NUL-terminated line into `out`
 *              (caller buffer >= BLE_STR_LINE_MAX). Returns 0 if none ready.
 * BLE_SendStr: transmit a NUL-terminated string (blocking). */
uint8_t           BLE_GetLine(BLE_Handle_t *hble, char *out);
HAL_StatusTypeDef BLE_SendStr(BLE_Handle_t *hble, const char *s);

/* Raw byte send (still available if needed) */
HAL_StatusTypeDef BLE_SendBytes(BLE_Handle_t *hble,
                                 const uint8_t *data, uint16_t len);

/* AT-command helpers (run at startup if module needs reconfiguring) */
HAL_StatusTypeDef BLE_AT_SetName(BLE_Handle_t *hble, const char *name);
HAL_StatusTypeDef BLE_AT_SetBaud(BLE_Handle_t *hble, uint32_t baud);

#endif /* BLE_JDY29_H */
