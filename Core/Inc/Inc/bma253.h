#ifndef BMA253_H
#define BMA253_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* BMA253 — Bosch Sensortec 3-axis accelerometer, LGA-12
 * Interface : I2C1
 * I2C addr  : 0x18 (SDO→GND) or 0x19 (SDO→VCC) — tie SDO on PCB
 * MOTION_INT→ PA0 (EXTI0 rising) — used to wake from sleep
 */

#define BMA253_I2C_ADDR_LOW   (0x18 << 1)  /* SDO = GND */
#define BMA253_I2C_ADDR_HIGH  (0x19 << 1)  /* SDO = VCC */
#define BMA253_I2C_ADDR       BMA253_I2C_ADDR_LOW  /* update per PCB */

/* Register map (partial) */
#define BMA253_REG_CHIP_ID      0x00  /* should read 0xFA */
#define BMA253_REG_ACCD_X_LSB   0x02
#define BMA253_REG_ACCD_X_MSB   0x03
#define BMA253_REG_ACCD_Y_LSB   0x04
#define BMA253_REG_ACCD_Y_MSB   0x05
#define BMA253_REG_ACCD_Z_LSB   0x06
#define BMA253_REG_ACCD_Z_MSB   0x07
#define BMA253_REG_ACCD_TEMP    0x08
#define BMA253_REG_INT_STATUS_0 0x09
#define BMA253_REG_PMU_RANGE    0x0F  /* ±2g=0x03, ±4g=0x05, ±8g=0x08, ±16g=0x0C */
#define BMA253_REG_PMU_BW       0x10  /* bandwidth, 0x08=7.81 Hz … 0x0F=1000 Hz  */
#define BMA253_REG_PMU_LPW      0x11  /* power modes */
#define BMA253_REG_INT_EN_0     0x16  /* interrupt enables */
#define BMA253_REG_INT_EN_1     0x17
#define BMA253_REG_INT_MAP_0    0x19
#define BMA253_REG_INT_MAP_2    0x1B
#define BMA253_REG_INT_OUT_CTRL 0x20
#define BMA253_REG_SLOPE_DUR    0x27  /* any-motion duration */
#define BMA253_REG_SLOPE_THRES  0x28  /* any-motion threshold */
#define BMA253_REG_SOFTRESET    0x14  /* write 0xB6 to reset */
#define BMA253_CHIP_ID_VALUE    0xFA

typedef struct {
    I2C_HandleTypeDef *hi2c;
    int16_t  accel_x_mg;
    int16_t  accel_y_mg;
    int16_t  accel_z_mg;
    uint8_t  motion_detected;  /* set by EXTI ISR */
    uint8_t  initialized;
} BMA253_Handle_t;

HAL_StatusTypeDef BMA253_Init(BMA253_Handle_t *hbma, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMA253_ReadAccel(BMA253_Handle_t *hbma);
void              BMA253_MotionISR(BMA253_Handle_t *hbma); /* call from EXTI ISR */
uint8_t           BMA253_PopMotionFlag(BMA253_Handle_t *hbma);
HAL_StatusTypeDef BMA253_SetSuspend(BMA253_Handle_t *hbma, uint8_t suspend);
HAL_StatusTypeDef BMA253_SoftReset(BMA253_Handle_t *hbma);

#endif /* BMA253_H */
