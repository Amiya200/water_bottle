#pragma GCC optimize("Os")
#include "bma253.h"

static HAL_StatusTypeDef BMA253_WriteReg(BMA253_Handle_t *hbma,
                                          uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(hbma->hi2c, BMA253_I2C_ADDR,
                                   buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef BMA253_ReadRegs(BMA253_Handle_t *hbma,
                                          uint8_t reg, uint8_t *out, uint8_t len)
{
    HAL_StatusTypeDef s;
    s = HAL_I2C_Master_Transmit(hbma->hi2c, BMA253_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(hbma->hi2c, BMA253_I2C_ADDR, out, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BMA253_SoftReset(BMA253_Handle_t *hbma)
{
    HAL_StatusTypeDef s = BMA253_WriteReg(hbma, BMA253_REG_SOFTRESET, 0xB6);
    HAL_Delay(5);  /* >1.8 ms power-on time */
    return s;
}

HAL_StatusTypeDef BMA253_Init(BMA253_Handle_t *hbma, I2C_HandleTypeDef *hi2c)
{
    hbma->hi2c           = hi2c;
    hbma->motion_detected = 0;
    hbma->initialized    = 0;
    hbma->accel_x_mg = hbma->accel_y_mg = hbma->accel_z_mg = 0;

    /* Verify chip ID */
    uint8_t chip_id = 0;
    if (BMA253_ReadRegs(hbma, BMA253_REG_CHIP_ID, &chip_id, 1) != HAL_OK) return HAL_ERROR;
    if (chip_id != BMA253_CHIP_ID_VALUE) return HAL_ERROR;

    /* Soft reset then re-read to confirm */
    BMA253_SoftReset(hbma);
    if (BMA253_ReadRegs(hbma, BMA253_REG_CHIP_ID, &chip_id, 1) != HAL_OK) return HAL_ERROR;
    if (chip_id != BMA253_CHIP_ID_VALUE) return HAL_ERROR;

    /* Range: ±2g (0x03) */
    if (BMA253_WriteReg(hbma, BMA253_REG_PMU_RANGE, 0x03) != HAL_OK) return HAL_ERROR;

    /* Bandwidth: 62.5 Hz (0x0B) — fast enough to catch drinking motion */
    if (BMA253_WriteReg(hbma, BMA253_REG_PMU_BW, 0x0B) != HAL_OK) return HAL_ERROR;

    /* Any-motion (slope) interrupt:
     * Duration = 0x00 (1 sample), Threshold = 0x14 (~50 mg)
     * Adjust threshold to avoid false triggers from table vibrations.
     */
    if (BMA253_WriteReg(hbma, BMA253_REG_SLOPE_DUR,   0x00) != HAL_OK) return HAL_ERROR;
    if (BMA253_WriteReg(hbma, BMA253_REG_SLOPE_THRES, 0x14) != HAL_OK) return HAL_ERROR;

    /* Enable slope interrupt on all axes (bits 2:0 of INT_EN_0) */
    if (BMA253_WriteReg(hbma, BMA253_REG_INT_EN_0, 0x07) != HAL_OK) return HAL_ERROR;

    /* Map slope interrupt to INT1 pin (bit 2 of INT_MAP_0) */
    if (BMA253_WriteReg(hbma, BMA253_REG_INT_MAP_0, 0x04) != HAL_OK) return HAL_ERROR;

    /* INT1 = push-pull, active HIGH */
    if (BMA253_WriteReg(hbma, BMA253_REG_INT_OUT_CTRL, 0x01) != HAL_OK) return HAL_ERROR;

    hbma->initialized = 1;
    return HAL_OK;
}

HAL_StatusTypeDef BMA253_ReadAccel(BMA253_Handle_t *hbma)
{
    uint8_t raw[6];
    if (BMA253_ReadRegs(hbma, BMA253_REG_ACCD_X_LSB, raw, 6) != HAL_OK) return HAL_ERROR;

    /* 12-bit two's complement. The sensor places the 8 MSBs in raw[odd] and
     * the 4 LSBs in bits[7:4] of raw[even]. Pack so the 12-bit value's sign
     * bit lands in bit 15 of a signed 16-bit word, then arithmetic-shift right
     * by 4 to sign-extend.
     * (The previous code cast to int16_t before the shift, which broke sign
     *  extension and produced garbage acceleration values.) */
    int16_t x = (int16_t)(((uint16_t)raw[1] << 8) | (raw[0] & 0xF0)) >> 4;
    int16_t y = (int16_t)(((uint16_t)raw[3] << 8) | (raw[2] & 0xF0)) >> 4;
    int16_t z = (int16_t)(((uint16_t)raw[5] << 8) | (raw[4] & 0xF0)) >> 4;

    /* At ±2g range, 1 LSB = 0.98 mg ≈ 1 mg  */
    hbma->accel_x_mg = x;
    hbma->accel_y_mg = y;
    hbma->accel_z_mg = z;
    return HAL_OK;
}

void BMA253_MotionISR(BMA253_Handle_t *hbma)
{
    hbma->motion_detected = 1;
}

uint8_t BMA253_PopMotionFlag(BMA253_Handle_t *hbma)
{
    if (hbma->motion_detected) {
        hbma->motion_detected = 0;
        return 1;
    }
    return 0;
}

HAL_StatusTypeDef BMA253_SetSuspend(BMA253_Handle_t *hbma, uint8_t suspend)
{
    /* PMU_LPW bit 7 = suspend */
    uint8_t val = suspend ? 0x80 : 0x00;
    return BMA253_WriteReg(hbma, BMA253_REG_PMU_LPW, val);
}
