#ifndef BUZZER_H
#define BUZZER_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* TIM3_CH2 → PB5 (BUZZER_Pin) */

typedef enum {
    BUZZER_NONE = 0,
    BUZZER_STARTUP,          /* first-time power-on jingle            */
    BUZZER_DOUBLE_BEEP,      /* default hydration reminder            */
    BUZZER_SINGLE_BEEP,      /* generic confirm                       */
    BUZZER_PURITY_ALERT,     /* water purity warning                  */
    BUZZER_TEMP_ALERT,       /* temperature warning                   */
    BUZZER_SYNC_OK,          /* data synced to app                    */
    BUZZER_LOW_BATTERY,      /* battery < 10 %                        */
    BUZZER_FACTORY_RESET,    /* 5 warning beeps before wipe           */
    BUZZER_CALIBRATION_DONE, /* short success tone after calibration  */
    BUZZER_ERROR,            /* internal error                        */
    BUZZER_REGISTRATION_OK,  /* pairing complete                      */
} Buzzer_Pattern_t;

void Buzzer_Init(TIM_HandleTypeDef *htim);
void Buzzer_Play(Buzzer_Pattern_t pattern);
void Buzzer_Update(void);   /* call every ms from SysTick or main loop */
void Buzzer_Stop(void);
uint8_t Buzzer_IsBusy(void);

#endif /* BUZZER_H */
