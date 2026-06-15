#pragma GCC optimize("Os")
#include "tds_sensor.h"
#include "main.h"

/* TDS_DRIVE (PA6): toggle AC square wave before ADC sample to avoid
 * electrolysis / electrode polarisation.
 *
 * ───────────────────────────────────────────────────────────────────────────
 * TWO-POINT CALIBRATION (this revision)
 * ───────────────────────────────────────────────────────────────────────────
 * The DFRobot cubic assumes DFRobot's probe + divider; on this hardware it
 * does not match a reference meter. A two-point linear calibration against a
 * reference TDS tester was validated on the bench (clear mV separation,
 * inverted slope handled natively).
 *
 *   - UNCALIBRATED: behaviour is IDENTICAL to the previous build (DFRobot
 *     cubic), so nothing in the existing flow changes until you calibrate.
 *   - CALIBRATED:   ppm = linear map of measured mV between the two captured
 *     points, then temperature-compensated to 25 °C.
 *
 * Calibration procedure (over the ASCII console, see app_logic.c):
 *   probe in low-ref water   ->  CALLO,<meter ppm>     e.g. CALLO,45
 *   probe in high-ref water  ->  CALHI,<meter ppm>     e.g. CALHI,891
 *   CALSTAT to inspect, TDSQ/TDS to read calibrated ppm.
 *
 * Cal points live in RAM. To persist them, store the four values returned by
 * TDS_CalGet() in DeviceSettings_t and call TDS_CalSet() after
 * Storage_LoadSettings() in App_Init() — see notes in app_logic.c.
 * ───────────────────────────────────────────────────────────────────────────
 */

/* ─── Two-point calibration state (module-private) ──────────────────────── */
#define TDS_CAL_MIN_SEP_MV  50    /* min |hi-lo| mV for a valid calibration:
                                     protects against capturing both points
                                     in the same water                       */

static int32_t  s_cal_mv_lo  = 0;
static int32_t  s_cal_mv_hi  = 0;
static uint16_t s_cal_ppm_lo = 0;
static uint16_t s_cal_ppm_hi = 0;
static uint8_t  s_cal_valid  = 0;
static uint16_t s_last_mv    = 0;  /* last measured (uncompensated) mV      */

static void TDS_CalRecompute(void)
{
    int32_t sep = s_cal_mv_hi - s_cal_mv_lo;
    if (sep < 0) sep = -sep;
    s_cal_valid = (sep >= TDS_CAL_MIN_SEP_MV &&
                   s_cal_ppm_hi > s_cal_ppm_lo) ? 1U : 0U;
}

static uint16_t TDS_ReadADC(TDS_Handle_t *htds)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    /* HAL_ADC_ConfigChannel only ORs the channel bit into CHSELR; it never
     * clears previously-selected channels. With CH2/CH3/CH7 all enabled from
     * MX_ADC_Init, the F030 forward scanner would convert the lowest selected
     * channel instead of ours. Clear CHSELR so ONLY CH2 (PA2/TDS) is read. */
    htds->hadc->Instance->CHSELR = 0U;
    sConfig.Channel      = ADC_CHANNEL_2;
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  /* long settle for the resistor-divided TDS node */
    HAL_ADC_ConfigChannel(htds->hadc, &sConfig);

    HAL_ADC_Start(htds->hadc);
    if (HAL_ADC_PollForConversion(htds->hadc, 100) != HAL_OK) {
        HAL_ADC_Stop(htds->hadc);
        return 0U;
    }
    uint16_t val = (uint16_t)HAL_ADC_GetValue(htds->hadc);
    HAL_ADC_Stop(htds->hadc);
    return val;
}

/* ─── Shared excitation + sample sequence ───────────────────────────────────
 * One function used by BOTH the ppm reader and the calibration capture, so a
 * calibration point is captured with exactly the same drive/sample timing as
 * every later measurement. Returns mV (0 = dry probe or ADC fault).        */
static uint16_t TDS_MeasureMV(TDS_Handle_t *htds)
{
    /* AC-drive: 10 toggle cycles before sample */
    for (uint8_t i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    uint16_t adc_raw = TDS_ReadADC(htds);
    HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_RESET);

    if (adc_raw == 0U) { s_last_mv = 0U; return 0U; }

    uint32_t mv = (uint32_t)adc_raw * TDS_ADC_VREF_MV / TDS_ADC_RESOLUTION;
    if (mv > 65535U) mv = 65535U;
    s_last_mv = (uint16_t)mv;
    return s_last_mv;
}

void TDS_Init(TDS_Handle_t *htds, ADC_HandleTypeDef *hadc)
{
    htds->hadc     = hadc;
    htds->last_ppm = 0;
    htds->valid    = 0;
    HAL_GPIO_WritePin(TDS_DRIVE_GPIO_Port, TDS_DRIVE_Pin, GPIO_PIN_RESET);
}

uint16_t TDS_ReadPPM(TDS_Handle_t *htds, int16_t temp_x10)
{
    uint16_t mv16 = TDS_MeasureMV(htds);

    /* A zero conversion means either a dry probe or an ADC timeout. Treat a
     * hard-zero as "no reading": keep the last good ppm and flag invalid so
     * the app does not raise a false purity alert on a 0 ppm glitch. */
    if (mv16 == 0U) {
        htds->valid = 0;
        return htds->last_ppm;
    }

    /* Temperature compensation coefficient (shared by both paths):
     * coeff_x1000 = 1000 * (1 + 0.02*(T - 25)) = 1000 + (temp_x10 - 250)*2 */
    int32_t coeff_x1000 = 1000 + ((int32_t)temp_x10 - 250) * 2;
    if (coeff_x1000 <= 200) coeff_x1000 = 200;   /* sane clamp               */

    /* ── CALIBRATED PATH: two-point linear map, then temp comp to 25 °C ──
     * Handles an inverted slope (salty water = lower mV when the probe sits
     * in the bottom divider leg) natively: the denominator is just negative.*/
    if (s_cal_valid) {
        int32_t mv  = (int32_t)mv16;
        int32_t ppm = (int32_t)s_cal_ppm_lo +
                      ((mv - s_cal_mv_lo) *
                       ((int32_t)s_cal_ppm_hi - (int32_t)s_cal_ppm_lo)) /
                      (s_cal_mv_hi - s_cal_mv_lo);
        if (ppm < 0) ppm = 0;

        ppm = (ppm * 1000) / coeff_x1000;        /* normalise to 25 °C       */
        if (ppm < 0)      ppm = 0;
        if (ppm > 65535)  ppm = 65535;

        htds->last_ppm = (uint16_t)ppm;
        htds->valid    = 1;
        return htds->last_ppm;
    }

    /* ── UNCALIBRATED PATH: previous behaviour, unchanged ────────────────── */

    /* Compensated voltage in mV × 1000 / coeff → keep in mV */
    uint32_t comp_mv = ((uint32_t)mv16 * 1000U) / (uint32_t)coeff_x1000;

    /* DFRobot empirical formula, fixed-point only:
     * ppm = (133.42*V^3 - 255.86*V^2 + 857.39*V) * 0.5
     * V = comp_mv / 1000.
     *
     * ppm_x1000 = 66710*mV^3/1e9 - 127930*mV^2/1e6 + 428695*mV/1000
     * Using int64 avoids Cortex-M0 soft-float helpers and keeps accuracy.
     */
    int64_t m  = (int64_t)comp_mv;
    int64_t m2 = m * m;
    int64_t m3 = m2 * m;

    int64_t ppm_x1000 = ((66710LL  * m3) / 1000000000LL)
                      - ((127930LL * m2) / 1000000LL)
                      + ((428695LL * m)  / 1000LL);

    if (ppm_x1000 < 0) ppm_x1000 = 0;
    if (ppm_x1000 > 65535000LL) ppm_x1000 = 65535000LL;

    htds->last_ppm = (uint16_t)((ppm_x1000 + 500LL) / 1000LL);
    htds->valid    = 1;
    return htds->last_ppm;
}

uint8_t TDS_IsAboveThreshold(TDS_Handle_t *htds, uint16_t threshold_ppm, int16_t temp_x10)
{
    return (TDS_ReadPPM(htds, temp_x10) > threshold_ppm) ? 1U : 0U;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Two-point calibration API
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Capture the LOW reference point: probe sitting in the low-ppm water.
 * Performs a fresh measurement. Returns the captured mV (0 = read failed). */
uint16_t TDS_CalCaptureLow(TDS_Handle_t *htds, uint16_t ref_ppm)
{
    uint16_t mv = TDS_MeasureMV(htds);
    if (mv == 0U) return 0U;
    s_cal_mv_lo  = (int32_t)mv;
    s_cal_ppm_lo = ref_ppm;
    TDS_CalRecompute();
    return mv;
}

/* Capture the HIGH reference point: probe sitting in the high-ppm water.
 * Returns the captured mV (0 = read failed). Check TDS_CalIsValid() after. */
uint16_t TDS_CalCaptureHigh(TDS_Handle_t *htds, uint16_t ref_ppm)
{
    uint16_t mv = TDS_MeasureMV(htds);
    if (mv == 0U) return 0U;
    s_cal_mv_hi  = (int32_t)mv;
    s_cal_ppm_hi = ref_ppm;
    TDS_CalRecompute();
    return mv;
}

/* Restore previously stored calibration (e.g. from DeviceSettings_t after
 * Storage_LoadSettings). Validity is recomputed, so garbage in storage
 * simply leaves the sensor uncalibrated on the DFRobot fallback. */
void TDS_CalSet(int32_t mv_lo, uint16_t ppm_lo, int32_t mv_hi, uint16_t ppm_hi)
{
    s_cal_mv_lo  = mv_lo;
    s_cal_ppm_lo = ppm_lo;
    s_cal_mv_hi  = mv_hi;
    s_cal_ppm_hi = ppm_hi;
    TDS_CalRecompute();
}

/* Read back the calibration for persisting. Any pointer may be NULL. */
void TDS_CalGet(int32_t *mv_lo, uint16_t *ppm_lo, int32_t *mv_hi, uint16_t *ppm_hi)
{
    if (mv_lo)  *mv_lo  = s_cal_mv_lo;
    if (ppm_lo) *ppm_lo = s_cal_ppm_lo;
    if (mv_hi)  *mv_hi  = s_cal_mv_hi;
    if (ppm_hi) *ppm_hi = s_cal_ppm_hi;
}

uint8_t TDS_CalIsValid(void)
{
    return s_cal_valid;
}

/* Last measured (uncompensated) mV — for the TDSP debug command. */
uint16_t TDS_GetLastMV(void)
{
    return s_last_mv;
}
