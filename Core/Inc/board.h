/**
 ******************************************************************************
 * @file    board.h
 * @brief   HydraSense Smart Unit - Board pin map & build configuration
 * @target  STM32F030K6T6 (LQFP32, 32KB Flash / 4KB RAM)
 * @drivers STM32 LL only (no HAL) - memory constrained part
 *
 *  Pin map taken verbatim from the CubeMX .ioc pinout:
 *    PA0  MOTION_INT   EXTI0  (BMA253 INT1)
 *    PA1  (spare)
 *    PA2  TDS_MCU      ADC_IN2  (TDS analog, post-divider)
 *    PA3  NTC_TEMP     ADC_IN3  (10k NTC thermistor)
 *    PA4  HX711_DOUT   GPIO IN  (load cell data)
 *    PA5  HX711_SCK    GPIO OUT (load cell clock)
 *    PA6  TDS_DRIVE    GPIO OUT (TDS excitation enable)
 *    PA7  BAT%         ADC_IN7  (battery voltage divider)
 *    PA8  RGB_D        TIM1_CH1 + DMA  (WS2812B via 74LVC1G125)
 *    PA9  USART1_TX    (JDY-23 BLE)
 *    PA10 USART1_RX    (JDY-23 BLE)
 *    PA11 RTC_INT      EXTI11 (PCF8563 INT)
 *    PA12 PWRC         GPIO OUT (BLE power control)
 *    PA13 SWDIO
 *    PA14 SWCLK
 *    PA15 BLE_STATE    GPIO IN  (JDY-23 link status)
 *    PB0  CHRG_STAT    GPIO IN  (TP4056 CHRG#, active low)
 *    PB1  STDBY_STAT   GPIO IN  (TP4056 STDBY#, active low)
 *    PB3  BUTTON       GPIO IN  (SW1, pulled by R30 10k to 3V3 -> active low)
 *    PB5  BUZZER       GPIO OUT (BC847 NPN -> active high)
 *    PB6  I2C1_SCL
 *    PB7  I2C1_SDA
 *
 *  I2C bus devices:
 *    ADS1110 (TDS ADC)   0x48  (A0 variant)
 *    BMA253  (accel)     0x18  (SDO low)
 *    PCF8563 (RTC)       0x51
 *    M24512  (EEPROM)    0x50  (E0/E1/E2 = 0)
 ******************************************************************************
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_iwdg.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

/* ---------- Firmware identity ---------- */
#define FW_VERSION_STR     "2.0.0"
#define BOTTLE_MODEL_STR   "HydraSense-Pro-v1"

/* ---------- GPIO port/pin definitions ---------- */
#define MOTION_INT_PORT   GPIOA
#define MOTION_INT_PIN    LL_GPIO_PIN_0

#define TDS_ADC_PORT      GPIOA
#define TDS_ADC_PIN       LL_GPIO_PIN_2          /* ADC_IN2 */
#define NTC_ADC_PORT      GPIOA
#define NTC_ADC_PIN       LL_GPIO_PIN_3          /* ADC_IN3 */
#define BAT_ADC_PORT      GPIOA
#define BAT_ADC_PIN       LL_GPIO_PIN_7          /* ADC_IN7 */

#define HX_DOUT_PORT      GPIOA
#define HX_DOUT_PIN       LL_GPIO_PIN_4
#define HX_SCK_PORT       GPIOA
#define HX_SCK_PIN        LL_GPIO_PIN_5

#define TDS_DRV_PORT      GPIOA
#define TDS_DRV_PIN       LL_GPIO_PIN_6

#define RGB_PORT          GPIOA
#define RGB_PIN           LL_GPIO_PIN_8          /* TIM1_CH1 AF2 */

#define RTC_INT_PORT      GPIOA
#define RTC_INT_PIN       LL_GPIO_PIN_11

#define PWRC_PORT         GPIOA
#define PWRC_PIN          LL_GPIO_PIN_12

#define BLE_STATE_PORT    GPIOA
#define BLE_STATE_PIN     LL_GPIO_PIN_15

#define CHRG_PORT         GPIOB
#define CHRG_PIN          LL_GPIO_PIN_0          /* active low */
#define STDBY_PORT        GPIOB
#define STDBY_PIN         LL_GPIO_PIN_1          /* active low */

#define BTN_PORT          GPIOB
#define BTN_PIN           LL_GPIO_PIN_3          /* active low (R30 pull-up) */

#define BUZ_PORT          GPIOB
#define BUZ_PIN           LL_GPIO_PIN_5          /* active high */

/* ---------- I2C addresses (7-bit, shifted by LL as needed) ---------- */
#define I2C_ADDR_ADS1110  (0x48 << 1)
#define I2C_ADDR_BMA253   (0x18 << 1)
#define I2C_ADDR_PCF8563  (0x51 << 1)
#define I2C_ADDR_M24512   (0x50 << 1)

/* ---------- Charger status helpers (active low) ---------- */
#define IS_CHARGING()  (LL_GPIO_IsInputPinSet(CHRG_PORT, CHRG_PIN) == 0)
#define IS_STANDBY()   (LL_GPIO_IsInputPinSet(STDBY_PORT, STDBY_PIN) == 0)
/* Charger present if either charging or standby (full) is asserted */
#define IS_CHARGER_PRESENT() (IS_CHARGING() || IS_STANDBY())

/* ---------- Button (active low) ---------- */
#define BTN_PRESSED()  (LL_GPIO_IsInputPinSet(BTN_PORT, BTN_PIN) == 0)

/* ---------- Buzzer ---------- */
#define BUZ_ON()   LL_GPIO_SetOutputPin(BUZ_PORT, BUZ_PIN)
#define BUZ_OFF()  LL_GPIO_ResetOutputPin(BUZ_PORT, BUZ_PIN)

/* ---------- BLE power control ---------- */
#define BLE_PWR_ON()   LL_GPIO_SetOutputPin(PWRC_PORT, PWRC_PIN)
#define BLE_PWR_OFF()  LL_GPIO_ResetOutputPin(PWRC_PORT, PWRC_PIN)
#define BLE_LINKED()   (LL_GPIO_IsInputPinSet(BLE_STATE_PORT, BLE_STATE_PIN))

/* ---------- WS2812 LED count ---------- */
#define LED_COUNT  10

/* ---------- System tick: 1 kHz ---------- */
extern volatile uint32_t g_ms_ticks;
static inline uint32_t millis(void) { return g_ms_ticks; }
void delay_ms(uint32_t ms);

/* ---------- Init ---------- */
void board_clock_init(void);
void board_gpio_init(void);

#endif /* BOARD_H */
