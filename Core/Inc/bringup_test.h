#ifndef BRINGUP_TEST_H
#define BRINGUP_TEST_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/*
 * Hardware bring-up / self-test module.
 *
 * Production builds should NOT define BRINGUP_TEST_MODE. In that case this
 * header emits tiny inline no-ops and bringup_test.c compiles to an empty file,
 * saving RAM/Flash even when the file remains inside the CubeIDE project.
 */

#ifdef BRINGUP_TEST_MODE

typedef enum {
    TEST_PENDING = 0,
    TEST_PASS    = 1,
    TEST_FAIL    = 2,
} TestStatus_t;

typedef struct {
    volatile uint8_t      rgb_color_index;
    volatile uint8_t      rgb_busy;
    volatile uint32_t     rgb_send_count;
    volatile TestStatus_t rgb_status;

    volatile uint16_t     ntc_adc_raw;
    volatile int16_t      ntc_temp_x10;
    volatile TestStatus_t ntc_status;

    volatile uint16_t     tds_adc_raw;
    volatile uint16_t     tds_ppm;
    volatile TestStatus_t tds_status;

    volatile uint8_t      hx_ready;
    volatile int32_t      hx_raw;
    volatile int32_t      hx_raw_tare;
    volatile int32_t      hx_grams;
    volatile TestStatus_t hx_status;

    volatile uint32_t     loop_count;
    volatile uint32_t     uptime_ms;
} BringUpTest_t;

extern BringUpTest_t g_test;

void BringUp_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim_ws);
void BringUp_RunOnce(void);
void BringUp_RunLoop(void);

#else

static inline void BringUp_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim_ws)
{ (void)hadc; (void)htim_ws; }
static inline void BringUp_RunOnce(void) {}
static inline void BringUp_RunLoop(void) {}

#endif /* BRINGUP_TEST_MODE */

#endif /* BRINGUP_TEST_H */
