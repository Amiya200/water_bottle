/* HYDRA OPT: this is the older LL-based sensor stack.
 * The production app uses battery.c, ntc_temp.c, tds_sensor.c, hx711.c,
 * bma253.c and rtc_manager.c. Keep this file out of production Flash/RAM
 * unless a legacy test build explicitly defines HYDRA_ENABLE_LEGACY_LL_SENSORS.
 */
#ifdef HYDRA_ENABLE_LEGACY_LL_SENSORS

/**
 * @file sensors.c
 * @brief Sensor drivers for HydraSense (LL drivers, no HAL).
 */
#include "sensors.h"
#include "i2c_bus.h"

sensor_health_t g_health = {0};

static int32_t s_hx_offset = 0;

/* =====================================================================
 *  ADC : single-conversion polled, used for NTC, BAT%, (TDS fallback)
 * ===================================================================== */
void adc_init(void)
{
    /* Calibrate before enable (ADEN must be 0) */
    if (LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Disable(ADC1);
        while (LL_ADC_IsEnabled(ADC1)) {}
    }
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    /* >= 4 ADC clk cycles between cal end and ADEN set */
    for (volatile int i = 0; i < 64; i++) __NOP();

    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    /* enable internal vref for calibrated battery measurement */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_VREFINT);

    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {}
}

uint16_t adc_read_channel(uint32_t ll_channel)
{
    LL_ADC_REG_SetSequencerChannels(ADC1, ll_channel);
    /* small settling for high-impedance NTC source */
    for (volatile int i = 0; i < 200; i++) __NOP();
    LL_ADC_REG_StartConversion(ADC1);
    uint32_t t = 0;
    while (!LL_ADC_IsActiveFlag_EOC(ADC1)) { if (++t > 100000U) return 0; }
    return (uint16_t)LL_ADC_REG_ReadConversionData12(ADC1);
}

/* Reads VREFINT to get true VDDA, then scales BAT divider sample. */
uint16_t battery_read_mv(void)
{
    uint16_t vref_raw = adc_read_channel(LL_ADC_CHANNEL_VREFINT);
    uint16_t bat_raw  = adc_read_channel(LL_ADC_CHANNEL_7);   /* PA7 */
    if (vref_raw == 0) return 0;
    /* VDDA in mV from internal reference */
    uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, LL_ADC_RESOLUTION_12B);
    uint32_t vpin = __LL_ADC_CALC_DATA_TO_VOLTAGE(vdda, bat_raw, LL_ADC_RESOLUTION_12B);
    /* Divider: R9 100k / R10 100k -> Vbat = 2 * Vpin */
    g_health.temp_ok = g_health.temp_ok; /* no-op keep */
    return (uint16_t)(vpin * 2U);
}

uint8_t battery_read_percent(void)
{
    uint16_t mv = battery_read_mv();
    /* Li-ion discharge curve approximation 3300..4200 mV */
    if (mv >= 4200) return 100;
    if (mv <= 3300) return 0;
    /* piecewise-ish: simple linear is acceptable for a progress ring */
    return (uint8_t)(((uint32_t)(mv - 3300) * 100U) / 900U);
}

int16_t ntc_read_celsius_x10(void)
{
    uint16_t raw = adc_read_channel(LL_ADC_CHANNEL_3);   /* PA3 */
    if (raw == 0 || raw >= 4095) { g_health.temp_ok = false; return -1000; }
    g_health.temp_ok = true;
    /* NTC 10k, series R22 10k to 3V3, NTC to GND (per schematic R22 pull-up).
       Vntc = VDDA * raw/4096 ; Rntc = R * Vntc/(VDDA-Vntc) = 10k*raw/(4096-raw)
       Beta model: 1/T = 1/T0 + (1/B)*ln(R/R0), R0=10k @ 25C, B=3950 */
    float rntc = 10000.0f * ((float)raw / (float)(4096 - raw));
    float t0 = 298.15f, b = 3950.0f, r0 = 10000.0f;
    float lnv = 0.0f;
    /* tiny ln approximation via standard libm is too heavy; use a fixed
       cubic-free approach: rely on -Os libm logf (linked once). */
    extern float logf(float);
    lnv = logf(rntc / r0);
    float invT = (1.0f / t0) + (lnv / b);
    float tK = 1.0f / invT;
    float tC = tK - 273.15f;
    return (int16_t)(tC * 10.0f);
}

/* =====================================================================
 *  HX711 - bit-banged 24-bit. SCK on PA5, DOUT on PA4. Gain 128 (ch A).
 * ===================================================================== */
static inline void hx_sck(uint8_t lvl)
{
    if (lvl) LL_GPIO_SetOutputPin(HX_SCK_PORT, HX_SCK_PIN);
    else     LL_GPIO_ResetOutputPin(HX_SCK_PORT, HX_SCK_PIN);
}
static inline uint8_t hx_dout(void)
{
    return LL_GPIO_IsInputPinSet(HX_DOUT_PORT, HX_DOUT_PIN);
}
static inline void hx_us(uint32_t n) { while (n--) { __NOP(); __NOP(); __NOP(); } }

void hx711_init(void)
{
    hx_sck(0);     /* power up (SCK low) */
    delay_ms(2);
    g_health.weight_ok = true;
}

bool hx711_ready(void) { return hx_dout() == 0; }  /* DOUT low = data ready */

int32_t hx711_read_raw(void)
{
    uint32_t t = 0;
    while (!hx711_ready()) { if (++t > 200000U) { g_health.weight_ok = false; return 0; } }

    uint32_t v = 0;
    __disable_irq();                 /* timing-critical: HX711 ~50ns min */
    for (int i = 0; i < 24; i++) {
        hx_sck(1); hx_us(1);
        v = (v << 1) | hx_dout();
        hx_sck(0); hx_us(1);
    }
    /* 25th pulse -> gain 128 next read (channel A) */
    hx_sck(1); hx_us(1); hx_sck(0); hx_us(1);
    __enable_irq();

    /* sign-extend 24-bit two's complement */
    if (v & 0x800000U) v |= 0xFF000000U;
    g_health.weight_ok = true;
    return (int32_t)v;
}

void hx711_set_offset(int32_t off) { s_hx_offset = off; }
int32_t hx711_get_offset(void) { return s_hx_offset; }

void hx711_tare(void)
{
    int64_t acc = 0;
    for (int i = 0; i < 8; i++) acc += hx711_read_raw();
    s_hx_offset = (int32_t)(acc / 8);
}

int32_t hx711_read_grams(int32_t scale_x100)
{
    int64_t acc = 0;
    for (int i = 0; i < 4; i++) acc += hx711_read_raw();
    int32_t avg = (int32_t)(acc / 4);
    if (scale_x100 < 100) scale_x100 = 40000;
    int64_t g = ((int64_t)(avg - s_hx_offset) * 100LL) / (int64_t)scale_x100;
    if (g < 0) g = 0;
    return (int32_t)g;
}

/* =====================================================================
 *  ADS1110 - 16-bit delta-sigma, config byte: 15-SPS, continuous, gain 1
 * ===================================================================== */
bool ads1110_init(void)
{
    /* config: ST/DRDY=0(cont) ... 0x8C = 16-bit, 15SPS, PGA x1, continuous */
    uint8_t cfg = 0x8C;
    int r = i2c_write(I2C_ADDR_ADS1110, &cfg, 1);
    g_health.tds_ok = (r == 0);
    return g_health.tds_ok;
}

int16_t ads1110_read(void)
{
    uint8_t b[3];   /* MSB, LSB, config */
    if (i2c_read(I2C_ADDR_ADS1110, b, 3) != 0) { g_health.tds_ok = false; return 0; }
    g_health.tds_ok = true;
    return (int16_t)((b[0] << 8) | b[1]);
}

uint16_t tds_read_ppm(void)
{
    /* drive AC-ish excitation: toggle TDS_DRIVE, let it settle, read ADS */
    LL_GPIO_SetOutputPin(TDS_DRV_PORT, TDS_DRV_PIN);
    delay_ms(20);
    int32_t acc = 0;
    for (int i = 0; i < 4; i++) { acc += ads1110_read(); delay_ms(5); }
    LL_GPIO_ResetOutputPin(TDS_DRV_PORT, TDS_DRV_PIN);
    int16_t raw = (int16_t)(acc / 4);
    if (raw < 0) raw = 0;

    /* ADS1110 16-bit FS ~ +/-2.048V at PGA1 -> 15.625uV/LSB.
       Voltage at divider node -> EC -> TDS. Linear cal placeholder:
       ppm = raw * k.  k derived from calibration; default ~0.5 ppm/LSB
       (override after wet calibration). */
    float volt = raw * (2.048f / 32768.0f);
    /* simple Gravity-TDS style cubic compensation (25C reference) */
    float tds = (133.42f * volt*volt*volt
                 - 255.86f * volt*volt
                 + 857.39f * volt) * 0.5f;
    if (tds < 0) tds = 0;
    if (tds > 1500) tds = 1500;
    return (uint16_t)tds;
}

/* =====================================================================
 *  BMA253 accelerometer - sleep low-power + slope (any-motion) interrupt
 * ===================================================================== */
#define BMA_REG_CHIPID   0x00
#define BMA_REG_PMU_RANGE 0x0F
#define BMA_REG_PMU_BW    0x10
#define BMA_REG_INT_EN0   0x16
#define BMA_REG_INT_MAP0  0x19
#define BMA_REG_INT_OUT   0x20
#define BMA_REG_SLOPE_TH  0x28
#define BMA_REG_SLOPE_DUR 0x27
#define BMA_REG_ACCD_X_LSB 0x02

bool bma253_init(void)
{
    uint8_t id = 0;
    if (i2c_reg_read(I2C_ADDR_BMA253, BMA_REG_CHIPID, &id, 1) != 0) {
        g_health.accel_ok = false; return false;
    }
    /* BMA253 chip id = 0xFA */
    i2c_reg_write8(I2C_ADDR_BMA253, BMA_REG_PMU_RANGE, 0x03); /* +/-2g */
    i2c_reg_write8(I2C_ADDR_BMA253, BMA_REG_PMU_BW,    0x08); /* 7.81Hz */
    i2c_reg_write8(I2C_ADDR_BMA253, BMA_REG_SLOPE_DUR, 0x00); /* 1 sample */
    i2c_reg_write8(I2C_ADDR_BMA253, BMA_REG_SLOPE_TH,  0x14); /* threshold */
    i2c_reg_write8(I2C_ADDR_BMA253, BMA_REG_INT_EN0,   0x07); /* slope x/y/z */
    i2c_reg_write8(I2C_ADDR_BMA253, BMA_REG_INT_MAP0,  0x04); /* slope->INT1 */
    i2c_reg_write8(I2C_ADDR_BMA253, BMA_REG_INT_OUT,   0x00); /* active low, pp */
    g_health.accel_ok = true;
    return true;
}

void bma253_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t b[6];
    if (i2c_reg_read(I2C_ADDR_BMA253, BMA_REG_ACCD_X_LSB, b, 6) != 0) {
        *x=*y=*z=0; return;
    }
    *x = (int16_t)(((b[1] << 8) | (b[0] & 0xC0)) >> 4);
    *y = (int16_t)(((b[3] << 8) | (b[2] & 0xC0)) >> 4);
    *z = (int16_t)(((b[5] << 8) | (b[4] & 0xC0)) >> 4);
}


#endif /* HYDRA_ENABLE_LEGACY_LL_SENSORS */
