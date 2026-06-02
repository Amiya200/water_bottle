#ifndef BLE_JDY29_H
#define BLE_JDY29_H

/*
 * ble_jdy29.h  –  JDY-29 BLE module driver interface
 *
 * RAM optimisation
 * ─────────────────────────────────────────────────────────────────────
 * –8 B  RAM : BLE_STR_LINE_MAX reduced 32 → 24.
 *             Longest ASCII command: "CALWEIGHT,32767" = 15 chars.
 *             24 bytes gives 7 bytes headroom.
 *             Saves 8 bytes in BLE_Handle_t.line_buf AND in the stack-
 *             allocated line[] in App_ServiceBLE every App_Run() tick.
 *
 * NOTE: BLE_PKT_MAX_LEN is NO LONGER defined here.  It is defined in
 * ble_protocol.h as (5U + BLE_PKT_MAX_PAYLOAD).  Redefining it here
 * caused a -Wmacro-redefined warning that promoted to an error.
 * ─────────────────────────────────────────────────────────────────────
 */

#include "stm32f0xx_hal.h"
#include "ble_protocol.h"
#include <stdint.h>

/* ─── ASCII line buffer size ─────────────────────────────────────────────── */
/* BLE_PKT_MAX_LEN is inherited from ble_protocol.h — do NOT redefine here */
#define BLE_STR_LINE_MAX    24U   /* was 32 — saves 8 bytes in BLE_Handle_t  */

/* ─── Connection state ───────────────────────────────────────────────────── */
#define BLE_STATE_DISCONNECTED  0U
#define BLE_STATE_CONNECTED     1U

/* ─── BLE_Handle_t ───────────────────────────────────────────────────────── */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rx_byte;
    uint8_t  pkt_buf[BLE_PKT_MAX_LEN];   /* 25 bytes (ble_protocol.h) */
    uint8_t  pkt_len;
    uint8_t  pkt_in_frame;
    uint8_t  pkt_ready;
    char     line_buf[BLE_STR_LINE_MAX];  /* 24 bytes (was 32)         */
    uint8_t  line_len;
    uint8_t  line_ready;
    uint8_t  conn_state;
    uint32_t rx_last_ms;
} BLE_Handle_t;

/* ─── API ────────────────────────────────────────────────────────────────── */
void              BLE_Init(BLE_Handle_t *hble, UART_HandleTypeDef *huart);
void              BLE_StartReceive(BLE_Handle_t *hble);
void              BLE_RxISR(BLE_Handle_t *hble);

uint8_t           BLE_GetPacket(BLE_Handle_t *hble, BLE_Packet_t *out);
HAL_StatusTypeDef BLE_SendPacket(BLE_Handle_t *hble, const uint8_t *buf, uint8_t len);

uint8_t           BLE_GetLine(BLE_Handle_t *hble, char *out);
void              BLE_IdleFlush(BLE_Handle_t *hble, uint32_t idle_ms);
HAL_StatusTypeDef BLE_SendStr(BLE_Handle_t *hble, const char *s);
HAL_StatusTypeDef BLE_SendBytes(BLE_Handle_t *hble, const uint8_t *data, uint16_t len);

uint8_t           BLE_IsConnected(BLE_Handle_t *hble);

#ifdef BLE_AT_COMMANDS
HAL_StatusTypeDef BLE_AT_SetName(BLE_Handle_t *hble, const char *name);
HAL_StatusTypeDef BLE_AT_SetBaud(BLE_Handle_t *hble, uint32_t baud);
#endif

#endif /* BLE_JDY29_H */
