#pragma once
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "aerodrag_types.h"
#include "board_pins.h"
#include <string.h>

// ─── Battery ADC ──────────────────────────────────────────────────────────────
// ESP32-S3-Touch-LCD-2.8: GPIO8 → 200K + 100K divider → Vbat/3
// Formula: Vbat = 3.3 / 4096 * 3 * adc_raw

#define BAT_SAMPLES     16
#define BAT_VREF_MV    3300.0f
#define BAT_ADC_MAX    4095.0f

typedef struct {
    adc_oneshot_unit_handle_t handle;
    adc_channel_t             channel;
    float                     last_mv;
    uint8_t                   last_pct;
} battery_t;

// Li-ion discharge curve: voltage [mV] → percentage
static const struct { float mv; uint8_t pct; } BAT_CURVE[] = {
    {4200,100},{4100,95},{3980,90},{3870,80},{3780,70},
    {3700,60}, {3640,50},{3580,40},{3520,30},{3450,20},
    {3380,10}, {3300,0},
};

static uint8_t mv_to_pct(float mv)
{
    const int N = sizeof(BAT_CURVE)/sizeof(BAT_CURVE[0]);
    if (mv >= BAT_CURVE[0].mv)   return 100;
    if (mv <= BAT_CURVE[N-1].mv) return 0;
    for (int i=0; i<N-1; i++) {
        if (mv <= BAT_CURVE[i].mv && mv > BAT_CURVE[i+1].mv) {
            float t = (mv - BAT_CURVE[i+1].mv) /
                      (BAT_CURVE[i].mv - BAT_CURVE[i+1].mv);
            return (uint8_t)(BAT_CURVE[i+1].pct +
                   t * (BAT_CURVE[i].pct - BAT_CURVE[i+1].pct));
        }
    }
    return 0;
}

esp_err_t battery_init(battery_t *bat, adc_channel_t channel)
{
    bat->channel  = channel;
    bat->last_mv  = 3700.0f;
    bat->last_pct = 50;

    adc_oneshot_unit_init_cfg_t cfg = { .unit_id = ADC_UNIT_1 };
    esp_err_t ret = adc_oneshot_new_unit(&cfg, &bat->handle);
    if (ret != ESP_OK) return ret;

    adc_oneshot_chan_cfg_t ch = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_oneshot_config_channel(bat->handle, channel, &ch);
}

void battery_update(battery_t *bat)
{
    int sum = 0;
    for (int i=0; i<BAT_SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(bat->handle, bat->channel, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    float raw_avg = (float)sum / (float)BAT_SAMPLES;
    // Waveshare formula: Vbat = 3.3 / 4096 * 3 * adc_raw  → in mV × 1000
    float vbat_mv = (raw_avg / BAT_ADC_MAX) * BAT_VREF_MV * BAT_ADC_RATIO;
    if (vbat_mv < 3000.0f) vbat_mv = 3000.0f;
    if (vbat_mv > 4300.0f) vbat_mv = 4300.0f;
    // Slow EMA to avoid jitter
    bat->last_mv  = bat->last_mv * 0.9f + vbat_mv * 0.1f;
    bat->last_pct = mv_to_pct(bat->last_mv);
}

// ─── NVS calibration ──────────────────────────────────────────────────────────
#define NVS_NAMESPACE "aerodrag"
#define NVS_KEY_CAL   "calibration"

static const aerodrag_cal_t CAL_DEFAULT = {
    .pitot_offset_pa  = 0.0f,
    .imu_pitch_offset = 0.0f,
    .mass_kg          = 78.0f,
    .crr              = 0.0040f,
    .cda_target       = 0.230f,
    .wheel_circ_m     = 2.105f,   // 700c x 25mm
    .crc              = 0xDEADBEEF,
};

esp_err_t cal_load(aerodrag_cal_t *cal)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) { *cal = CAL_DEFAULT; return ret; }
    size_t len = sizeof(aerodrag_cal_t);
    ret = nvs_get_blob(h, NVS_KEY_CAL, cal, &len);
    nvs_close(h);
    if (ret != ESP_OK || cal->crc != 0xDEADBEEF) {
        *cal = CAL_DEFAULT;
        return ESP_ERR_NOT_FOUND;
    }
    // Validazione anche al LOAD da NVS (audit v0.3.1, FW-2): una NVS corrotta o
    // salvata da un firmware più vecchio non deve propagare valori fuori range
    // (o NaN) a velocità/fisica. Forma !(x >= lo && x <= hi) → respinge anche NaN.
    // Stessi range della WRITE su CONFIG 0xaa08 (contract §2).
    if (!(cal->mass_kg      >= 33.0f  && cal->mass_kg      <= 200.0f))
        cal->mass_kg      = CAL_DEFAULT.mass_kg;
    if (!(cal->crr          >= 0.001f && cal->crr          <= 0.025f))
        cal->crr          = CAL_DEFAULT.crr;
    if (!(cal->wheel_circ_m >= 1.0f   && cal->wheel_circ_m <= 2.5f))
        cal->wheel_circ_m = CAL_DEFAULT.wheel_circ_m;
    return ESP_OK;
}

esp_err_t cal_save(const aerodrag_cal_t *cal)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_blob(h, NVS_KEY_CAL, cal, sizeof(aerodrag_cal_t));
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

void cal_zero_pitot(aerodrag_cal_t *cal, float current_pa)
{
    cal->pitot_offset_pa = current_pa;
    cal_save(cal);
}
