/**
 * @file i2c_bus.h
 * @brief Blocking I2C1 master helpers (LL). 400 kHz fast-mode.
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H
#include "board.h"

void i2c_init(void);

/* All return 0 on success, negative on error/timeout. */
int  i2c_write(uint8_t addr, const uint8_t *data, uint8_t len);
int  i2c_read(uint8_t addr, uint8_t *data, uint8_t len);
int  i2c_write_read(uint8_t addr, const uint8_t *wr, uint8_t wlen,
                    uint8_t *rd, uint8_t rlen);
int  i2c_reg_write8(uint8_t addr, uint8_t reg, uint8_t val);
int  i2c_reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
bool i2c_ping(uint8_t addr);

#endif
