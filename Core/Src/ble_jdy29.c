#pragma GCC optimize("Os")
#include "ble_jdy29.h"
#include "main.h"
#include <string.h>

/* NOTE: <stdio.h> intentionally removed.
 * The old code used snprintf() in BLE_AT_SetName, which pulled the entire
 * printf formatting family into the binary (~2 KB even with --specs=nano.specs).
 * Replaced with a simple manual string builder — zero cost on flash.
 */

/* ─── Init ───────────────────────────────────────────────────── */
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
    memset(hble->pkt_buf, 0, sizeof(hble->pkt_buf));
    memset(hble->line_buf, 0, sizeof(hble->line_buf));
}

void BLE_StartReceive(BLE_Handle_t *hble)
{
    HAL_UART_Receive_IT(hble->huart, &hble->rx_byte, 1);
}

/* ─── RX ISR — call from HAL_UART_RxCpltCallback ────────────── *
 *
 * Accumulates a binary framed packet directly into pkt_buf.
 * Frame format: [SOF=0xAA][CMD][LEN][payload×LEN][CHK][EOF=0x55]
 *
 * Replaces the old JSON brace-counting state machine — much smaller
 * code footprint, no static-brace_depth hack, no heap use.
 */
void BLE_RxISR(BLE_Handle_t *hble)
{
    uint8_t byte = hble->rx_byte;
    hble->rx_last_ms = HAL_GetTick();   /* for idle-line detection */

    /* Restart single-byte receive immediately */
    HAL_UART_Receive_IT(hble->huart, &hble->rx_byte, 1);

    /* If a complete packet is waiting to be consumed, drop incoming bytes
     * rather than overwriting.  app_logic polls every 1 s — plenty of time. */
    if (hble->pkt_ready) return;

    /* ─── ASCII string-command path (parallel to binary) ────────────────
     * When we are NOT currently inside a binary frame, treat printable
     * bytes and CR/LF as an ASCII command line. A binary frame always
     * begins with 0xAA (BLE_SOF), which is non-printable, so the two
     * paths never collide for normal text commands.
     * ------------------------------------------------------------------ */
    if (!hble->pkt_in_frame && byte != BLE_SOF && !hble->line_ready) {
        if (byte == '\r' || byte == '\n') {
            if (hble->line_len > 0U) {
                hble->line_buf[hble->line_len] = '\0';
                hble->line_ready = 1;       /* complete line ready */
            }
            /* lone CR/LF with empty buffer: ignore */
        } else if (byte >= 0x20 && byte < 0x7F) {   /* printable ASCII */
            if (hble->line_len < (BLE_STR_LINE_MAX - 1U)) {
                hble->line_buf[hble->line_len++] = (char)byte;
            } else {
                hble->line_len = 0;   /* overflow: discard partial line */
            }
        }
        /* For ASCII path we do NOT fall through to the binary framer. */
        if (byte != BLE_SOF) return;
    }

    /* Start of frame detection */
    if (byte == BLE_SOF && !hble->pkt_in_frame) {
        hble->pkt_len      = 0;
        hble->pkt_in_frame = 1;
    }

    if (!hble->pkt_in_frame) return;

    /* Accumulate byte */
    if (hble->pkt_len < BLE_PKT_MAX_LEN) {
        hble->pkt_buf[hble->pkt_len++] = byte;
    } else {
        /* Buffer full without completing a frame — corrupted stream, reset */
        hble->pkt_in_frame = 0;
        hble->pkt_len      = 0;
        return;
    }

    /* Once we have SOF+CMD+LEN (3 bytes) we know the expected total length.
     * Total = SOF(1) + CMD(1) + LEN(1) + payload(LEN) + CHK(1) + EOF(1) = 5+LEN */
    if (hble->pkt_len >= 3U) {
        uint8_t expected = 5U + hble->pkt_buf[2];   /* pkt_buf[2] = LEN field */

        if (expected > BLE_PKT_MAX_LEN) {
            /* LEN field is out of range — garbage frame, reset */
            hble->pkt_in_frame = 0;
            hble->pkt_len      = 0;
            return;
        }

        if (hble->pkt_len == expected) {
            /* Last byte must be EOF */
            if (hble->pkt_buf[hble->pkt_len - 1U] == BLE_EOF) {
                hble->pkt_ready = 1;
            }
            /* Either way the frame attempt is over */
            hble->pkt_in_frame = 0;
        }
    }
}

/* ─── Binary packet I/O ─────────────────────────────────────── */

/* Pop a complete received packet; returns 1 if a valid packet was available. */
uint8_t BLE_GetPacket(BLE_Handle_t *hble, BLE_Packet_t *out)
{
    if (!hble->pkt_ready) return 0;
    uint8_t ok = BLE_ParsePacket(hble->pkt_buf, hble->pkt_len, out);
    hble->pkt_ready = 0;
    hble->pkt_len   = 0;
    return ok;
}

/* ─── Low-level TX ──────────────────────────────────────────────────────────
 * IMPORTANT: RX runs in interrupt mode (HAL_UART_Receive_IT armed at all
 * times). On STM32 HAL, calling the blocking HAL_UART_Transmit() while an RX
 * interrupt transfer is active can return HAL_BUSY and send NOTHING, because
 * both share the same huart lock / state. That is why the device could receive
 * commands but never reply.
 *
 * To avoid the conflict we push TX bytes straight to the USART data register,
 * polling TXE/TC. This bypasses the HAL lock entirely and is safe to interleave
 * with the armed RX interrupt. */
static void BLE_TxRaw(BLE_Handle_t *hble, const uint8_t *data, uint16_t len)
{
    USART_TypeDef *U = hble->huart->Instance;
    for (uint16_t i = 0; i < len; i++) {
        uint32_t guard = 0;
        /* wait for TX data register empty (TXE) */
        while (!(U->ISR & USART_ISR_TXE)) {
            if (++guard > 200000U) return;   /* ~ms-scale timeout, never hang */
        }
        U->TDR = data[i];
    }
    /* wait for transmission complete (TC) so the buffer can be reused safely */
    uint32_t guard = 0;
    while (!(U->ISR & USART_ISR_TC)) {
        if (++guard > 200000U) return;
    }
}

/* Transmit a pre-built binary packet (built by BLE_Build* helpers). */
HAL_StatusTypeDef BLE_SendPacket(BLE_Handle_t *hble,
                                  const uint8_t *buf, uint8_t len)
{
    BLE_TxRaw(hble, buf, len);
    return HAL_OK;
}

/* ─── ASCII string-command I/O ──────────────────────────────── */

/* Pop a complete received line; returns 1 and copies into out (NUL-term). */
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

/* Finalize a buffered ASCII line that never got a CR/LF terminator, once the
 * link has been idle for `idle_ms`. Lets terminal apps that don't append a
 * newline still work (e.g. typing "help" and hitting send). */
void BLE_IdleFlush(BLE_Handle_t *hble, uint32_t idle_ms)
{
    if (hble->line_ready) return;          /* already a line waiting          */
    if (hble->line_len == 0U) return;      /* nothing buffered                */
    if (hble->pkt_in_frame) return;        /* mid binary frame — leave it     */
    if ((HAL_GetTick() - hble->rx_last_ms) < idle_ms) return;

    hble->line_buf[hble->line_len] = '\0';
    hble->line_ready = 1;
}

/* Transmit a NUL-terminated string (does not collide with RX IT). */
HAL_StatusTypeDef BLE_SendStr(BLE_Handle_t *hble, const char *s)
{
    BLE_TxRaw(hble, (const uint8_t *)s, (uint16_t)strlen(s));
    return HAL_OK;
}

/* ─── Raw byte send ─────────────────────────────────────────── */
HAL_StatusTypeDef BLE_SendBytes(BLE_Handle_t *hble,
                                 const uint8_t *data, uint16_t len)
{
    BLE_TxRaw(hble, data, len);
    return HAL_OK;
}

/* ─── Connection state ───────────────────────────────────────── */
void BLE_PollState(BLE_Handle_t *hble)
{
    GPIO_PinState pin = HAL_GPIO_ReadPin(BLE_STATE_GPIO_Port, BLE_STATE_Pin);
    hble->conn_state  = (pin == GPIO_PIN_SET) ? BLE_STATE_CONNECTED
                                               : BLE_STATE_DISCONNECTED;
}

uint8_t BLE_IsConnected(BLE_Handle_t *hble)
{
    BLE_PollState(hble);
    return (hble->conn_state == BLE_STATE_CONNECTED) ? 1U : 0U;
}

/* ─── AT command helpers ─────────────────────────────────────── */

static HAL_StatusTypeDef BLE_AT(BLE_Handle_t *hble, const char *cmd)
{
    BLE_TxRaw(hble, (const uint8_t *)cmd, (uint16_t)strlen(cmd));
    HAL_Delay(100);
    return HAL_OK;
}

/* Build "AT+NAME<name>\r\n" without snprintf — no printf library pulled in. */
HAL_StatusTypeDef BLE_AT_SetName(BLE_Handle_t *hble, const char *name)
{
    /* "AT+NAME" = 7 chars; max module name ~20 chars; "\r\n\0" = 3 */
    char buf[32];
    uint8_t i = 0;

    buf[i++] = 'A'; buf[i++] = 'T'; buf[i++] = '+';
    buf[i++] = 'N'; buf[i++] = 'A'; buf[i++] = 'M'; buf[i++] = 'E';
    while (*name && i < (uint8_t)(sizeof(buf) - 3U)) {
        buf[i++] = *name++;
    }
    buf[i++] = '\r';
    buf[i++] = '\n';
    buf[i]   = '\0';

    return BLE_AT(hble, buf);
}

HAL_StatusTypeDef BLE_AT_SetBaud(BLE_Handle_t *hble, uint32_t baud)
{
    (void)baud;  /* map baud → index per JDY-29 datasheet if needed */
    return BLE_AT(hble, "AT+BAUD8\r\n");
}
