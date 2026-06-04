#pragma GCC optimize("Os")
#include "buzzer.h"

/* ─── TIM3_CH2 → PB5 (BUZZER_Pin)
 * PWM frequency sets tone.  At 48 MHz, PSC=47 → 1 MHz timer clock.
 * ARR determines frequency: 1 MHz / ARR = tone Hz.
 * e.g. ARR=1000 → 1 kHz beep.
 */

#define BUZZER_TIM_CLK_HZ    1000000UL   /* after PSC=47 */
#define BUZZER_DEFAULT_FREQ  2000U        /* 2 kHz pleasant beep  */
#define BUZZER_DUTY          50U          /* 50% duty cycle       */

typedef struct {
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t  repeat;
} BuzzerStep_t;

/* Patterns: array of {on_ms, off_ms, repeat}, terminated by {0,0,0} */
static const BuzzerStep_t PATTERN_STARTUP[]      = {{80,60,3},{150,0,1},{0,0,0}};
static const BuzzerStep_t PATTERN_DOUBLE_BEEP[]  = {{120,100,1},{120,0,1},{0,0,0}};
static const BuzzerStep_t PATTERN_SINGLE_BEEP[]  = {{150,0,1},{0,0,0}};
/* PURITY_ALERT and TEMP_ALERT are identical — share one array */
static const BuzzerStep_t PATTERN_PURITY_ALERT[] = {{60,50,3},{200,0,1},{0,0,0}};
#define PATTERN_TEMP_ALERT PATTERN_PURITY_ALERT
static const BuzzerStep_t PATTERN_SYNC_OK[]      = {{80,50,1},{120,0,1},{0,0,0}};
static const BuzzerStep_t PATTERN_LOW_BAT[]      = {{300,200,3},{0,0,0}};
static const BuzzerStep_t PATTERN_FACTORY_RESET[]= {{200,200,5},{0,0,0}};
static const BuzzerStep_t PATTERN_CALIB_DONE[]   = {{100,80,1},{200,0,1},{0,0,0}};
static const BuzzerStep_t PATTERN_ERROR[]        = {{500,200,3},{0,0,0}};
static const BuzzerStep_t PATTERN_REG_OK[]       = {{100,60,2},{250,0,1},{0,0,0}};

static const BuzzerStep_t * const s_patterns[] = {
    NULL,                   /* BUZZER_NONE            */
    PATTERN_STARTUP,        /* BUZZER_STARTUP         */
    PATTERN_DOUBLE_BEEP,    /* BUZZER_DOUBLE_BEEP     */
    PATTERN_SINGLE_BEEP,    /* BUZZER_SINGLE_BEEP     */
    PATTERN_PURITY_ALERT,   /* BUZZER_PURITY_ALERT    */
    PATTERN_TEMP_ALERT,     /* BUZZER_TEMP_ALERT      */
    PATTERN_SYNC_OK,        /* BUZZER_SYNC_OK         */
    PATTERN_LOW_BAT,        /* BUZZER_LOW_BATTERY     */
    PATTERN_FACTORY_RESET,  /* BUZZER_FACTORY_RESET   */
    PATTERN_CALIB_DONE,     /* BUZZER_CALIBRATION_DONE*/
    PATTERN_ERROR,          /* BUZZER_ERROR           */
    PATTERN_REG_OK,         /* BUZZER_REGISTRATION_OK */
};

/* ─── Runtime state ─────────────────────────────────────────── */
static TIM_HandleTypeDef   *s_htim    = NULL;
static const BuzzerStep_t  *s_steps   = NULL;
static uint8_t              s_step_idx = 0;
static uint8_t              s_rep_cnt  = 0;
static uint32_t             s_step_start_ms = 0;
static uint8_t              s_tone_on  = 0;
static uint8_t              s_busy     = 0;

/* ─── Low-level tone on/off ─────────────────────────────────── */
static void Buzzer_ToneOn(uint16_t freq_hz)
{
    if (!s_htim || freq_hz == 0) return;
    uint32_t arr = BUZZER_TIM_CLK_HZ / freq_hz - 1;
    s_htim->Instance->ARR = arr;
    s_htim->Instance->CCR2 = arr * BUZZER_DUTY / 100;
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_2);
    s_tone_on = 1;
}

static void Buzzer_ToneOff(void)
{
    if (!s_htim) return;
    HAL_TIM_PWM_Stop(s_htim, TIM_CHANNEL_2);
    s_tone_on = 0;
}

void Buzzer_Init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;
    /* Ensure PSC=47 so timer runs at 1 MHz */
    htim->Instance->PSC = 47;
    htim->Instance->EGR = TIM_EGR_UG;
    Buzzer_ToneOff();
}

void Buzzer_Stop(void)
{
    Buzzer_ToneOff();
    s_steps   = NULL;
    s_busy    = 0;
}

uint8_t Buzzer_IsBusy(void) { return s_busy; }

void Buzzer_Play(Buzzer_Pattern_t pattern)
{
    if ((uint8_t)pattern >= sizeof(s_patterns) / sizeof(s_patterns[0])) return;
    Buzzer_Stop();
    s_steps        = s_patterns[pattern];
    s_step_idx     = 0;
    s_rep_cnt      = 0;
    s_step_start_ms = HAL_GetTick();
    s_busy         = 1;

    if (s_steps && s_steps[0].on_ms) {
        Buzzer_ToneOn(BUZZER_DEFAULT_FREQ);
    }
}

/* ─── Call every ms (SysTick or main-loop ≤ 1 ms) ──────────── */
void Buzzer_Update(void)
{
    if (!s_busy || !s_steps) return;

    const BuzzerStep_t *step = &s_steps[s_step_idx];
    if (step->on_ms == 0 && step->off_ms == 0 && step->repeat == 0) {
        Buzzer_Stop(); return;
    }

    uint32_t elapsed = HAL_GetTick() - s_step_start_ms;

    if (s_tone_on) {
        if (elapsed >= step->on_ms) {
            Buzzer_ToneOff();
            s_step_start_ms = HAL_GetTick();
            if (step->off_ms == 0) {
                /* No gap — move to repeat / next step */
                goto next_rep;
            }
        }
    } else {
        if (elapsed >= step->off_ms) {
            next_rep:
            s_rep_cnt++;
            if (s_rep_cnt < step->repeat) {
                s_step_start_ms = HAL_GetTick();
                Buzzer_ToneOn(BUZZER_DEFAULT_FREQ);
            } else {
                s_step_idx++;
                s_rep_cnt = 0;
                const BuzzerStep_t *nxt = &s_steps[s_step_idx];
                if (nxt->on_ms == 0 && nxt->off_ms == 0) {
                    Buzzer_Stop(); return;
                }
                s_step_start_ms = HAL_GetTick();
                Buzzer_ToneOn(BUZZER_DEFAULT_FREQ);
            }
        }
    }
}
