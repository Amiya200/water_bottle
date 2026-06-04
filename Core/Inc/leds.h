/**
 * @file leds.h
 * @brief WS2812B ring (10 LEDs) on PA8 via TIM1_CH1 + DMA, plus the full
 *        LED pattern engine implementing every cue in the FRD section 5.
 */
#ifndef LEDS_H
#define LEDS_H
#include "board.h"

typedef enum {
    LED_IDLE = 0,
    LED_REGISTER_WAIT,   /* 5.10 blue breathing */
    LED_CALIBRATING,     /* 5.4  amber wave */
    LED_HYDRATION_GOOD,  /* 5.1  green/custom blink 3s   */
    LED_HYDRATION_MID,   /* 5.1  amber blink 3s          */
    LED_HYDRATION_LOW,   /* 5.1  red blink 3s            */
    LED_PURITY_ALERT,    /* 5.2  purple 2 sharp flashes  */
    LED_TEMP_ALERT,      /* 5.3  orange 2 sharp flashes  */
    LED_DRINK_OK,        /* 5.5  green soft pulse x2      */
    LED_SYNC_OK,         /* 5.6  cyan sweep once          */
    LED_CHARGING,        /* 5.7  battery progress bar     */
    LED_LOW_BATT,        /* 5.8  red 3 slow pulses        */
    LED_LAMP,            /* 5.9  flowing chosen colour    */
    LED_FACTORY_WARN,    /* 5.11 red 5 rapid flashes      */
    LED_ERROR            /* 5.12 white 5 rapid flashes     */
} led_mode_t;

void leds_init(void);

/* Set a one-shot or continuous pattern. For continuous modes (charging,
   lamp, register-wait) the engine keeps running until changed. One-shot
   modes auto-return to LED_IDLE when finished. */
void leds_set_mode(led_mode_t m);
led_mode_t leds_get_mode(void);

/* configure dynamic colours */
void leds_set_custom_hydration_color(uint8_t r, uint8_t g, uint8_t b);
void leds_set_lamp_color(uint8_t r, uint8_t g, uint8_t b);
void leds_set_battery_percent(uint8_t pct);

/* Called from main loop frequently (~ every few ms). Non-blocking. */
void leds_task(void);

/* Parse "#RRGGBB" -> rgb. Returns false on bad string. */
bool leds_parse_hex(const char *hex, uint8_t *r, uint8_t *g, uint8_t *b);

#endif
