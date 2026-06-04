/**
 * @file ble.h
 * @brief USART1 transport for JDY-23 BLE module. Interrupt RX into a line
 *        buffer (newline-delimited JSON), polled TX.
 */
#ifndef BLE_H
#define BLE_H
#include "board.h"

#define BLE_RX_BUF  384
#define BLE_TX_BUF  512

void ble_init(void);
void ble_send(const char *str);                /* null-terminated */
void ble_send_len(const char *buf, uint16_t n);
/* returns pointer to a complete received line (without newline) or NULL.
   Buffer is valid until next call. */
char *ble_poll_line(void);
bool  ble_is_linked(void);

/* called from USART1 IRQ */
void ble_rx_irq(void);

#endif
