/**
 * @file sensors.h
 * @brief Unified sensor layer: HX711 weight, ADS1110 TDS, NTC temp,
 *        BMA253 motion, battery voltage.
 */
#ifndef SENSORS_H
#define SENSORS_H
#include "board.h"

/* ---- ADC (NTC PA3 = IN3, BAT% PA7 = IN7, TDS PA2 = IN2 backup) ---- */
void     adc_init(void);
uint16_t adc_read_channel(uint32_t ll_channel);  /* raw 12-bit */
int16_t  ntc_read_celsius_x10(void);             /* deci-degrees C */
uint8_t  battery_read_percent(void);
uint16_t battery_read_mv(void);

/* ---- HX711 24-bit load cell ---- */
void     hx711_init(void);
bool     hx711_ready(void);
int32_t  hx711_read_raw(void);                   /* signed 24-bit, gain 128 */
void     hx711_tare(void);                       /* sets current as zero */
void     hx711_set_offset(int32_t off);
int32_t  hx711_get_offset(void);
/* grams = (raw - offset) / scale ; scale counts/gram. */
int32_t  hx711_read_grams(int32_t scale_x100);

/* ---- ADS1110 TDS (16-bit delta-sigma) ---- */
bool     ads1110_init(void);
int16_t  ads1110_read(void);                     /* raw signed */
uint16_t tds_read_ppm(void);                     /* drives excitation, reads, converts */

/* ---- BMA253 accelerometer ---- */
bool     bma253_init(void);                      /* sets up motion interrupt */
void     bma253_read_xyz(int16_t *x, int16_t *y, int16_t *z);

/* sensor health flags (for DEVICE_STATUS) */
typedef struct {
    bool temp_ok;
    bool tds_ok;
    bool weight_ok;
    bool accel_ok;
    bool rtc_ok;
} sensor_health_t;
extern sensor_health_t g_health;

#endif
