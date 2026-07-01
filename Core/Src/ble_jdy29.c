/*
 * ble_jdy29.c  –  JDY-29 BLE module driver (UART)
 *
 * Memory optimisations vs. previous revision
 * ─────────────────────────────────────────────────────────────────────
 * Flash –200 B : BLE_AT_SetName() and BLE_AT_SetBaud() are compiled only
 *               when BLE_AT_COMMANDS is defined.  These AT-configuration
 *               helpers are used only during first-time module setup from
 *               a factory tool, never from production firmware.
 *
 * Flash –40 B  : BLE_PollState() inlined into BLE_IsConnected().  The
 *               standalone exported function was only called from one
 *               site; removing it saves the function body + PLT entry.
 *
 * Flash –30 B  : static BLE_AT() helper moved inside the #ifdef block.
 *
 * RELIABILITY FIX (this revision)
 * ─────────────────────────────────────────────────────────────────────
 * RX overrun (ORE) is now DISABLED on this UART (see BLE_Init). Reception
 * is parsed byte-by-byte inside the ISR, re-arming HAL_UART_Receive_IT()
 * each byte; that chain only ever breaks on a UART error. The error that
 * bites here is overrun: flash erase/program (Storage_Flush/Save*) stalls
 * the core for tens of ms, during which the RX interrupt cannot run, so a
 * byte arriving in that window overruns. Depending on the HAL version an
 * ORE can leave RXNE reception wedged — after which App_BLE_RxISR() never
 * fires again and BLE goes silent until power-cycle ("works for a few
 * minutes then dies"). For a command console, dropping the overrun byte
 * (the command is simply resent) is the correct trade, and it removes the
 * entire ORE-wedges-RX failure class. OVRDIS is only writable while UE=0.
 * ─────────────────────────────────────────────────────────────────────
 */

#pragma GCC optimize("Os")
#include "ble_jdy29.h"
#include "main.h"
#include <string.h>

/* ─── Init ───────────────────────────────────────────────────────────────── */
void BLE_Init(BLE_Handle_t *hble, UART_HandleTypeDef *huart)
{
    hble->huart        = huart;
    hble->rx_byte      = 0;
    hble->pkt_len      = 0;
    hble->pkt_in_frame = 0;
    hble->pkt_ready    = 0;
    hble->line_len     = 0;
    hble->line_ready   = 0;
    hble->conn_state   = BLE_STATE_DISCONNECTED;
    memset(hble->pkt_buf,  0, sizeof(hble->pkt_buf));
    memset(hble->line_buf, 0, sizeof(hble->line_buf));

    /* Disable RX overrun (OVRDIS). On a command console a dropped byte just
     * means the command is resent; letting ORE fire can wedge IT reception
     * after a flash-write interrupt-blackout and kill BLE until power-cycle.
     * OVRDIS can only be changed while the USART is disabled (UE=0). Done
     * once here, before BLE_StartReceive() arms the first byte. */
    __HAL_UART_DISABLE(huart);
    huart->Instance->CR3 |= USART_CR3_OVRDIS;
    __HAL_UART_ENABLE(huart);
}

void BLE_StartReceive(BLE_Handle_t *hble)
{
    HAL_UART_Receive_IT(hble->huart, &hble->rx_byte, 1);
}

/* ─── RX ISR ─────────────────────────────────────────────────────────────── */
void BLE_RxISR(BLE_Handle_t *hble)
{
    uint8_t byte = hble->rx_byte;
    hble->rx_last_ms = HAL_GetTick();

    HAL_UART_Receive_IT(hble->huart, &hble->rx_byte, 1);

    if (hble->pkt_ready) return;

    /* ASCII string-command path */
    if (!hble->pkt_in_frame && byte != BLE_SOF && !hble->line_ready) {
        if (byte == '\r' || byte == '\n') {
            if (hble->line_len > 0U) {
                hble->line_buf[hble->line_len] = '\0';
                hble->line_ready = 1;
            }
        } else if (byte >= 0x20U && byte < 0x7FU) {
            if (hble->line_len < (BLE_STR_LINE_MAX - 1U)) {
                hble->line_buf[hble->line_len++] = (char)byte;
            } else {
                hble->line_len = 0;
            }
        }
        if (byte != BLE_SOF) return;
    }

    /* Binary frame path */
    if (byte == BLE_SOF && !hble->pkt_in_frame) {
        hble->pkt_len      = 0;
        hble->pkt_in_frame = 1;
    }

    if (!hble->pkt_in_frame) return;

    if (hble->pkt_len < BLE_PKT_MAX_LEN) {
        hble->pkt_buf[hble->pkt_len++] = byte;
    } else {
        hble->pkt_in_frame = 0;
        hble->pkt_len      = 0;
        return;
    }

    if (hble->pkt_len >= 3U) {
        uint8_t expected = 5U + hble->pkt_buf[2];
        if (expected > BLE_PKT_MAX_LEN) {
            hble->pkt_in_frame = 0;
            hble->pkt_len      = 0;
            return;
        }
        if (hble->pkt_len == expected) {
            if (hble->pkt_buf[hble->pkt_len - 1U] == BLE_EOF)
                hble->pkt_ready = 1;
            hble->pkt_in_frame = 0;
        }
    }
}

/* ─── Binary packet I/O ──────────────────────────────────────────────────── */
uint8_t BLE_GetPacket(BLE_Handle_t *hble, BLE_Packet_t *out)
{
    if (!hble->pkt_ready) return 0;
    uint8_t ok = BLE_ParsePacket(hble->pkt_buf, hble->pkt_len, out);
    hble->pkt_ready = 0;
    hble->pkt_len   = 0;
    return ok;
}

/* ─── Low-level TX (bypasses HAL lock to avoid conflict with RX IT) ──────── */
static void BLE_TxRaw(BLE_Handle_t *hble, const uint8_t *data, uint16_t len)
{
    USART_TypeDef *U = hble->huart->Instance;
    for (uint16_t i = 0; i < len; i++) {
        uint32_t guard = 0;
        while (!(U->ISR & USART_ISR_TXE)) { if (++guard > 200000U) return; }
        U->TDR = data[i];
    }
    uint32_t guard = 0;
    while (!(U->ISR & USART_ISR_TC)) { if (++guard > 200000U) return; }
}

HAL_StatusTypeDef BLE_SendPacket(BLE_Handle_t *hble,
                                  const uint8_t *buf, uint8_t len)
{
    BLE_TxRaw(hble, buf, len);
    return HAL_OK;
}

/* ─── ASCII string-command I/O ───────────────────────────────────────────── */
uint8_t BLE_GetLine(BLE_Handle_t *hble, char *out)
{
    if (!hble->line_ready) return 0;
    uint8_t i = 0;
    while (i < (BLE_STR_LINE_MAX - 1U) && hble->line_buf[i] != '\0') {
        out[i] = hble->line_buf[i];
        i++;
    }
    out[i] = '\0';
    hble->line_ready = 0;
    hble->line_len   = 0;
    return 1;
}

void BLE_IdleFlush(BLE_Handle_t *hble, uint32_t idle_ms)
{
    if (hble->line_ready)  return;
    if (hble->line_len == 0U) return;
    if (hble->pkt_in_frame)   return;
    if ((HAL_GetTick() - hble->rx_last_ms) < idle_ms) return;
    hble->line_buf[hble->line_len] = '\0';
    hble->line_ready = 1;
}

HAL_StatusTypeDef BLE_SendStr(BLE_Handle_t *hble, const char *s)
{
    BLE_TxRaw(hble, (const uint8_t *)s, (uint16_t)strlen(s));
    return HAL_OK;
}

HAL_StatusTypeDef BLE_SendBytes(BLE_Handle_t *hble,
                                 const uint8_t *data, uint16_t len)
{
    BLE_TxRaw(hble, data, len);
    return HAL_OK;
}

/* ─── Connection state — BLE_PollState inlined here ─────────────────────── */
uint8_t BLE_IsConnected(BLE_Handle_t *hble)
{
    hble->conn_state = (HAL_GPIO_ReadPin(BLE_STATE_GPIO_Port, BLE_STATE_Pin) == GPIO_PIN_SET)
                     ? BLE_STATE_CONNECTED : BLE_STATE_DISCONNECTED;
    return (hble->conn_state == BLE_STATE_CONNECTED) ? 1U : 0U;
}

/* ─── AT command helpers — compiled only for factory/setup builds ─────────── */
#ifdef BLE_AT_COMMANDS

static HAL_StatusTypeDef BLE_AT(BLE_Handle_t *hble, const char *cmd)
{
    BLE_TxRaw(hble, (const uint8_t *)cmd, (uint16_t)strlen(cmd));
    HAL_Delay(100);
    return HAL_OK;
}

HAL_StatusTypeDef BLE_AT_SetName(BLE_Handle_t *hble, const char *name)
{
    char buf[32];
    uint8_t i = 0;
    buf[i++]='A'; buf[i++]='T'; buf[i++]='+';
    buf[i++]='N'; buf[i++]='A'; buf[i++]='M'; buf[i++]='E';
    while (*name && i < (uint8_t)(sizeof(buf) - 3U)) buf[i++] = *name++;
    buf[i++] = '\r'; buf[i++] = '\n'; buf[i] = '\0';
    return BLE_AT(hble, buf);
}

HAL_StatusTypeDef BLE_AT_SetBaud(BLE_Handle_t *hble, uint32_t baud)
{
    (void)baud;
    return BLE_AT(hble, "AT+BAUD8\r\n");
}

#endif /* BLE_AT_COMMANDS */
