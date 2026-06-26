#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "board_pins.h"
#include "aerodrag_types.h"
#include "version.h"

// Components
#include "../components/pitot/sdp810.h"
#include "../components/pitot/physics.h"
#include "../components/imu/qmi8658.h"
#include "../components/ble/ble_server.h"
#include "../components/ble_central/ble_sensors.h"
#include "../components/display/display.h"
#include "../components/battery/battery.h"
#include "../components/wifi/wifi_coach.h"
#include "../components/ota/ota_update.h"
#include "esp_mac.h"

static const char *TAG = "aerodrag";

// ─── Device identity ─────────────────────────────────────────────────────────────────────────────
device_identity_t g_identity = {0};

static void identity_init(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(g_identity.device_id, sizeof(g_identity.device_id),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    nvs_handle_t h;
    if (nvs_open("aerodrag", NVS_READONLY, &h) == ESP_OK) {
        size_t sz = sizeof(g_identity.athlete_name);
        if (nvs_get_str(h, "athlete_name", g_identity.athlete_name, &sz) != ESP_OK)
            strlcpy(g_identity.athlete_name, "Atleta", sizeof(g_identity.athlete_name));
        nvs_close(h);
    } else {
        strlcpy(g_identity.athlete_name, "Atleta", sizeof(g_identity.athlete_name));
    }
    ESP_LOGI(TAG, "Device ID: %s  Atleta: %s",
             g_identity.device_id, g_identity.athlete_name);
}

void identity_set_athlete_name(const char *name)
{
    strlcpy(g_identity.athlete_name, name, sizeof(g_identity.athlete_name));
    nvs_handle_t h;
    if (nvs_open("aerodrag", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "athlete_name", g_identity.athlete_name);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Athlete name aggiornato: %s", g_identity.athlete_name);
}

// ─── Global shared state ─────────────────────────────────────────────────────────────────────────────
static aerodrag_sensors_t  g_sensors = {0};
static aerodrag_physics_t  g_physics = {0};
static aerodrag_cal_t      g_cal     = {0};
static aerodrag_state_t    g_state   = STATE_INIT;

static SemaphoreHandle_t   g_sensors_mutex = NULL;
static volatile bool       g_screen_next   = false;
static SemaphoreHandle_t   g_i2c1_mutex    = NULL;

static float g_cda_smooth = 0.0f;

// Contatore giro condiviso fra task (scritto da ANT+/coach-WiFi, letto dal task
// display). L'accesso a un uint16_t allineato è atomico su ESP32 (niente torn read);
// `volatile` garantisce che il task display osservi gli aggiornamenti (audit v0.3.1, FW-3).
volatile uint16_t g_current_lap = 1;

// Fix H3: inizializzato in app_main (non a 0) per evitare che il watchdog
// deep-sleep scatti 10 minuti dopo il boot prima di qualsiasi sessione.
static int64_t g_last_activity_us  = 0;
static volatile int64_t g_session_start_us = -1;
static volatile int64_t g_lap_start_us     = -1;
#define SLEEP_TIMEOUT_US  (10LL * 60 * 1000 * 1000)

#define SENSORS_LOCK()    xSemaphoreTake(g_sensors_mutex, portMAX_DELAY)
#define SENSORS_UNLOCK()  xSemaphoreGive(g_sensors_mutex)
#define I2C1_LOCK()       xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY)
#define I2C1_UNLOCK()     xSemaphoreGive(g_i2c1_mutex)

static esp_err_t i2c_init_bus(i2c_port_t port, int sda, int scl, uint32_t hz)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda,
        .scl_io_num       = scl,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = hz,
    };
    esp_err_t ret = i2c_param_config(port, &cfg);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
}

// Scan diagnostico del bus: una riga di log con gli indirizzi che ACK-ano.
// Permette di distinguere "chip assente" (bus con altri dispositivi) da
// "pin sbagliati" (bus completamente vuoto).
static void i2c_scan_bus(i2c_port_t port, const char *name)
{
    char list[96] = {0};
    int  found    = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (a << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if (r == ESP_OK && found < 10) {
            size_t off = strlen(list);
            snprintf(list + off, sizeof(list) - off, " 0x%02X", a);
            found++;
        }
    }
    ESP_LOGI(TAG, "I2C scan %s: %d dispositivi%s%s",
             name, found, found ? ":" : "", found ? list : "");
}

// ─── Sensor drivers ─────────────────────────────────────────────────────────────────────────────
static sdp810_t  g_pitot = {0};
static qmi8658_t g_imu   = {0};
static battery_t g_bat   = {0};

static int64_t g_last_speed_us = 0;
static int64_t g_last_power_us = 0;
static int64_t g_last_hr_us    = 0;
static int64_t g_last_cad_us   = 0;
#define BLE_SPEED_STALE_US  (5LL * 1000 * 1000)

// ─── Task: Pitot + IMU @ 10 Hz ───────────────────────────────────────────────────────────────────────
// Letture I2C fallite consecutive prima di invalidare il sensore (~1 s a 10 Hz)
#define SENSOR_FAIL_LIMIT 10

// ─── ISSUE #30: auto-zero Pitot silenzioso ────────────────────────────────────
// Soglie iniziali (DA TARARE SU HW).
#define AZ_GYRO_MAX_DPS      2.0f                 // |gyro| per-asse [deg/s]
#define AZ_ACCEL_G_TOL       0.5f                 // ||a|-9.80665| <= 0.5 m/s^2
#define AZ_PITOT_STD_MAX_PA  1.0f                 // std-dev Pitot finestra dwell [Pa]
#define AZ_SPEED_MAX_MS      0.3f                 // corroborante speed [m/s]
#define AZ_T_STILL_US        (4LL * 1000 * 1000)  // dwell 4 s (tempo reale)
#define AZ_MIN_DWELL_SAMPLES 30                    // >=30 campioni Pitot validi
#define AZ_PERSIST_EPS_PA    0.2f                 // persisti solo se |nuovo-salvato| > eps
#define AZ_PERSIST_MIN_US    (5LL * 60 * 1000 * 1000) // >=5 min fra due persist NVS
#define AZ_OFFSET_SANE_PA    300.0f               // |offset| plausibile <= 300 Pa
#define AZ_GRAVITY_MS2       9.80665f

// Running-stats Welford O(1): media+varianza incrementali, niente ring-buffer.
typedef struct {
    uint32_t n;
    double   mean;
    double   m2;     // var = m2/(n-1)
} running_stats_t;

typedef enum { AZ_IDLE = 0, AZ_DWELL } az_state_t;

static az_state_t      g_az_state          = AZ_IDLE;
static running_stats_t g_az_pitot;
static int64_t         g_az_dwell_start_us = 0;

// Stato persistenza — FUORI dal blob aerodrag_cal_t (layout/CRC NVS invariati).
// Tutti gli accessi RMW vanno sotto SENSORS_LOCK (vedi az_*persist).
static float           g_last_saved_offset_pa = 0.0f;  // init da cal_load
static int64_t         g_last_persist_us      = 0;
static volatile bool   g_cal_persist_req      = false; // set da disconnect/config BLE

// Setter del flag persist, chiamato dal contesto host NimBLE (vedi ble_server.h).
void cal_request_persist(void) { g_cal_persist_req = true; }

static inline void rs_reset(running_stats_t *rs)
{
    rs->n = 0; rs->mean = 0.0; rs->m2 = 0.0;
}

static inline void rs_push(running_stats_t *rs, float x)
{
    if (!isfinite(x)) return;                 // scarta NaN/inf
    rs->n++;
    double d  = (double)x - rs->mean;
    rs->mean += d / (double)rs->n;
    rs->m2   += d * ((double)x - rs->mean);
}

static inline float rs_mean(const running_stats_t *rs)
{
    return (rs->n > 0) ? (float)rs->mean : 0.0f;
}

static inline float rs_std(const running_stats_t *rs)
{
    if (rs->n < 2) return 0.0f;
    double var = rs->m2 / (double)(rs->n - 1);
    if (var < 0.0) var = 0.0;
    return (float)sqrt(var);
}

// Gating istantaneo: true se il ciclo corrente e' 'fermo'. imu_ok = IMU letta
// CON SUCCESSO in QUESTO ciclo (chiude la finestra di grezzi stantii).
static bool az_instant_still(const aerodrag_sensors_t *s, bool imu_ok)
{
    if (!imu_ok || !g_imu.raw_valid || !s->imu_valid) return false;
    for (int i = 0; i < 3; i++) {
        float g = g_imu.gyro_dps[i];
        if (!isfinite(g) || fabsf(g) > AZ_GYRO_MAX_DPS) return false;
    }
    float ax = g_imu.accel_ms2[0], ay = g_imu.accel_ms2[1], az = g_imu.accel_ms2[2];
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) return false;
    float amag = sqrtf(ax*ax + ay*ay + az*az);   // norma invariante per rotazione
    if (fabsf(amag - AZ_GRAVITY_MS2) > AZ_ACCEL_G_TOL) return false;
    // Corroboranti vincolanti solo se la sorgente e' valida.
    if (s->gps_valid && s->speed_ms > AZ_SPEED_MAX_MS) return false;
    if (s->ant_valid && s->power_w != 0)               return false;
    return true;
}

// Guardia di varianza condivisa (auto-zero + long-press). Su true scrive *mean_pa.
static bool pitot_window_ok(const running_stats_t *rs, float *mean_pa)
{
    if (rs->n < AZ_MIN_DWELL_SAMPLES) return false;
    float std = rs_std(rs);
    if (!isfinite(std) || std > AZ_PITOT_STD_MAX_PA) return false;   // reject varianza
    float m = rs_mean(rs);
    if (!isfinite(m) || fabsf(m) > AZ_OFFSET_SANE_PA) return false;  // offset implausibile
    if (mean_pa) *mean_pa = m;
    return true;
}

// Esegue il persist NVS reale se serve. CHIAMARE SOLO da task_housekeeping (o
// shutdown). Serializza confronto+commit+aggiornamento stato sotto SENSORS_LOCK.
// force=true bypassa il rate-limit (shutdown/disconnect), rispetta sempre eps.
static void cal_do_persist(bool force)
{
    SENSORS_LOCK();
    int64_t now    = esp_timer_get_time();
    float   cur    = g_cal.pitot_offset_pa;
    bool    do_it  = (fabsf(cur - g_last_saved_offset_pa) > AZ_PERSIST_EPS_PA) &&
                     (force || (now - g_last_persist_us) >= AZ_PERSIST_MIN_US);
    aerodrag_cal_t snap;
    if (do_it) snap = g_cal;          // snapshot coerente del blob sotto lock
    SENSORS_UNLOCK();

    if (!do_it) return;
    if (cal_persist(&snap) == ESP_OK) {
        SENSORS_LOCK();
        g_last_saved_offset_pa = snap.pitot_offset_pa;
        g_last_persist_us      = now;
        SENSORS_UNLOCK();
    }
}

// FSM del rilevatore, chiamata una volta per ciclo del loop sensori.
// Solo set-RAM dell'offset (sotto lock); il commit NVS lo fa housekeeping.
static void az_step(esp_err_t pitot_ret, float pitot_raw_pa,
                    const aerodrag_sensors_t *s, bool imu_ok)
{
    bool still = az_instant_still(s, imu_ok);

    switch (g_az_state) {
    case AZ_IDLE:
        if (still) {
            rs_reset(&g_az_pitot);
            g_az_dwell_start_us = esp_timer_get_time();
            g_az_state = AZ_DWELL;
        }
        break;

    case AZ_DWELL:
        if (!still) {                       // violazione -> reset duro
            g_az_state = AZ_IDLE;
            rs_reset(&g_az_pitot);
            break;
        }
        if (pitot_ret == ESP_OK) rs_push(&g_az_pitot, pitot_raw_pa);
        if (esp_timer_get_time() - g_az_dwell_start_us >= AZ_T_STILL_US) {
            float mean_pa;
            if (pitot_window_ok(&g_az_pitot, &mean_pa)) {
                SENSORS_LOCK();
                cal_zero_pitot(&g_cal, mean_pa);   // SOLO RAM, silenzioso
                SENSORS_UNLOCK();
                g_cal_persist_req = true;          // NVS deferito a housekeeping
                ESP_LOGI(TAG, "auto-zero: offset=%.2f Pa n=%d std=%.2f",
                         mean_pa, (int)g_az_pitot.n, rs_std(&g_az_pitot));
            } // varianza alta / pochi campioni -> skip SILENZIOSO
            g_az_state = AZ_IDLE;
            rs_reset(&g_az_pitot);
        }
        break;
    }
}

static void task_pitot_imu(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint8_t env_divider = 0;
    uint8_t pitot_fail  = 0;
    uint8_t imu_fail    = 0;

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));

        {
            ble_sensor_data_t ext;
            ble_sensors_get(&ext);

            int64_t now_us = esp_timer_get_time();
            if (ext.status & BLE_SENS_SPEED) {
                g_last_speed_us = now_us;
            } else if (g_last_speed_us > 0 &&
                       (now_us - g_last_speed_us) > BLE_SPEED_STALE_US) {
                SENSORS_LOCK();
                g_sensors.gps_valid = false;
                g_sensors.speed_ms  = 0.0f;
                SENSORS_UNLOCK();
                g_last_speed_us = 0;
                ESP_LOGW("ble_sens", "Speed BLE stale — invalidato (>5s)");
            }

            if (ext.status & BLE_SENS_POWER) {
                g_last_power_us = now_us;
            } else if (g_last_power_us > 0 &&
                       (now_us - g_last_power_us) > BLE_SPEED_STALE_US) {
                SENSORS_LOCK();
                g_sensors.ant_valid = false;
                g_sensors.power_w   = 0;
                SENSORS_UNLOCK();
                g_last_power_us = 0;
                ESP_LOGW("ble_sens", "Power BLE stale — invalidato (>5s)");
            }

            if (ext.status & BLE_SENS_HR) {
                g_last_hr_us = now_us;
            } else if (g_last_hr_us > 0 &&
                       (now_us - g_last_hr_us) > BLE_SPEED_STALE_US) {
                SENSORS_LOCK();
                g_sensors.hr_bpm = 0;
                SENSORS_UNLOCK();
                g_last_hr_us = 0;
            }

            if (ext.status & BLE_SENS_CAD) {
                g_last_cad_us = now_us;
            } else if (g_last_cad_us > 0 &&
                       (now_us - g_last_cad_us) > BLE_SPEED_STALE_US) {
                SENSORS_LOCK();
                g_sensors.cadence_rpm = 0;
                SENSORS_UNLOCK();
                g_last_cad_us = 0;
            }

            SENSORS_LOCK();
            if (ext.status & BLE_SENS_POWER) {
                g_sensors.power_w   = ext.power_w;
                g_sensors.ant_valid = true;
            }
            if (ext.status & BLE_SENS_HR)    g_sensors.hr_bpm      = ext.hr_bpm;
            if (ext.status & BLE_SENS_CAD)   g_sensors.cadence_rpm = ext.cadence_rpm;
            if (ext.status & BLE_SENS_SPEED) {
                g_sensors.speed_ms   = ext.speed_cms / 100.0f;
                g_sensors.distance_m = ext.distance_m;
                g_sensors.gps_valid  = true;
            }
            if (ext.lap_event)               g_sensors.lap_event   = true;
            SENSORS_UNLOCK();
        }

        I2C1_LOCK();
        esp_err_t pitot_ret = sdp810_read(&g_pitot);
        I2C1_UNLOCK();
        ESP_LOGD("PITOT", "Pa=%.3f ret=%d", g_pitot.pressure_pa, pitot_ret);

        float imu_pitch = 0, imu_roll = 0, imu_temp = 0;
        bool  imu_ok = false;
        if (qmi8658_read(&g_imu) == ESP_OK) {
            imu_pitch = g_imu.pitch_deg;
            imu_roll  = g_imu.roll_deg;
            imu_temp  = g_imu.temp_c;
            imu_ok    = true;
        }

        bool pitot_lost = false, imu_lost = false;

        SENSORS_LOCK();
        if (pitot_ret == ESP_OK) {
            g_sensors.pitot_pa    = g_pitot.pressure_pa;
            g_sensors.static_pa   = 101325.0f;
            g_sensors.pitot_valid = true;
            g_sensors.temp_c      = g_pitot.temp_c;
            pitot_fail            = 0;
        } else if (pitot_fail < SENSOR_FAIL_LIMIT && ++pitot_fail == SENSOR_FAIL_LIMIT) {
            g_sensors.pitot_valid = false;
            pitot_lost            = true;
        }
        if (imu_ok) {
            g_sensors.pitch_deg = imu_pitch;
            g_sensors.roll_deg  = imu_roll;
            if (!g_sensors.pitot_valid) g_sensors.temp_c = imu_temp;
            g_sensors.imu_valid = true;
            imu_fail            = 0;
        } else if (imu_fail < SENSOR_FAIL_LIMIT && ++imu_fail == SENSOR_FAIL_LIMIT) {
            g_sensors.imu_valid = false;
            imu_lost            = true;
        }

        aerodrag_sensors_t sensors_copy = g_sensors;
        SENSORS_UNLOCK();

        // ISSUE #30: rilevatore immobilita' + auto-zero silenzioso. Pitot GREZZO
        // (pre-offset) del ciclo; imu_ok = IMU letta con successo in QUESTO ciclo.
        az_step(pitot_ret, g_pitot.pressure_pa, &sensors_copy, imu_ok);

        if (pitot_lost) ESP_LOGW(TAG, "Pitot: %d letture fallite — invalidato", SENSOR_FAIL_LIMIT);
        if (imu_lost)   ESP_LOGW(TAG, "IMU: %d letture fallite — invalidato",   SENSOR_FAIL_LIMIT);

        aerodrag_physics_t phy = physics_compute(&sensors_copy, &g_cal);

        SENSORS_LOCK();
        bool lap_now = sensors_copy.lap_event;
        if (lap_now) g_sensors.lap_event = false;
        uint8_t bat = g_sensors.battery_pct;
        SENSORS_UNLOCK();

        bool lap_pending_this_cycle = false;
        if (lap_now) {
            g_current_lap++;
            g_lap_start_us = esp_timer_get_time();
            lap_pending_this_cycle = true;
            ESP_LOGI("lap", "LAP event ANT+ → lap %d", g_current_lap);
        }

        if (phy.valid) {
            if (g_cda_smooth < 0.01f) g_cda_smooth = phy.CdA;
            g_cda_smooth = ema_update(g_cda_smooth, phy.CdA, EMA_ALPHA_30S);
            phy.CdA = g_cda_smooth;
            SENSORS_LOCK();
            g_physics = phy;
            SENSORS_UNLOCK();
            g_last_activity_us = esp_timer_get_time();

            if (coach_get_mode() != COACH_MODE_OFF)
                coach_send_frame(&sensors_copy, &phy, bat, lap_now);
        } else {
            SENSORS_LOCK();
            g_physics = phy;
            SENSORS_UNLOCK();
        }

        if (g_coach_lap_cmd) {
            g_coach_lap_cmd = false;
            if (!lap_pending_this_cycle) {
                g_current_lap++;
                g_lap_start_us = esp_timer_get_time();
                lap_pending_this_cycle = true;
                ESP_LOGI(TAG, "LAP da coach WiFi → lap %d", g_current_lap);
            } else {
                ESP_LOGW(TAG, "LAP coach soppresso: già fired da ANT+ stesso ciclo");
            }
        }

        if (lap_pending_this_cycle)
            ble_notify_ant(0xFFFF, 0, 0);

        if (g_coach_start_cmd) {
            g_coach_start_cmd = false;
            int64_t now = esp_timer_get_time();
            g_session_start_us = now;
            g_lap_start_us     = now;
            g_current_lap      = 1;
            g_state = STATE_RECORDING;
            display_set_screen(SCR_TIMER);
            ESP_LOGI(TAG, "Sessione avviata da coach");
        }
        if (g_coach_stop_cmd) {
            g_coach_stop_cmd = false;
            g_state = STATE_CONNECTED;
            ESP_LOGI(TAG, "Sessione fermata da coach");
        }

        ble_notify_physics(&phy);
        ble_notify_pitot(sensors_copy.pitot_pa, sensors_copy.static_pa);
        ble_notify_imu(sensors_copy.pitch_deg, sensors_copy.roll_deg);

        if (++env_divider >= 10) {
            env_divider = 0;
            ble_notify_env(sensors_copy.temp_c, sensors_copy.humidity_pct,
                           sensors_copy.altitude_m, sensors_copy.speed_ms);
        }
    }
}

// ─── Task: Display @ 5 Hz ─────────────────────────────────────────────────────────────────────────
static void task_display(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(200));

        if (g_screen_next) {
            g_screen_next = false;
            display_next_screen();
        }

        // Auto-advance from pairing screen once phone has connected via BLE
        if (display_get_screen() == SCR_PAIRING && ble_is_connected())
            display_set_screen(SCR_CDA);

        SENSORS_LOCK();
        aerodrag_sensors_t s_copy = g_sensors;
        aerodrag_physics_t p_copy = g_physics;
        SENSORS_UNLOCK();

        {
            int64_t now_us = esp_timer_get_time();
            int64_t s0 = g_session_start_us, l0 = g_lap_start_us;
            uint32_t sess_s = (s0 > 0) ? (uint32_t)((now_us - s0) / 1000000LL) : 0;
            uint32_t lap_s  = (l0 > 0) ? (uint32_t)((now_us - l0) / 1000000LL) : 0;
            display_set_timers(sess_s, lap_s, g_current_lap);
            display_set_connection_status(ble_is_connected(), coach_is_ready());
        }
        display_render(&s_copy, &p_copy);
    }
}

// ─── Power off ────────────────────────────────────────────────────────────────
// esp_deep_sleep(0) imposta un wakeup timer a 0 µs → reboot immediato.
// Lo spegnimento reale avviene rilasciando PWR_HOLD (taglia l'alimentazione);
// il deep sleep senza sorgenti di wakeup è il fallback quando si è sotto USB.
static void power_off(void)
{
    cal_do_persist(true);   // ISSUE #30: salva l'ultimo offset RAM prima dello spegnimento (force)
    gpio_set_level(PIN_PWR_HOLD, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_deep_sleep_start();
}

// ─── Task: Battery + housekeeping @ 0.1 Hz ────────────────────────────────────────────────────
static void task_housekeeping(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        battery_update(&g_bat);
        SENSORS_LOCK();
        g_sensors.battery_pct = g_bat.last_pct;
        g_sensors.battery_mv  = g_bat.last_mv;
        SENSORS_UNLOCK();

        // ISSUE #30: UNICO scrittore NVS del blob g_cal. Consuma le richieste di
        // persist (auto-zero/disconnect/config BLE) fuori dal loop sensori 10 Hz,
        // cosi' l'erase/commit (decine-centinaia di ms) non ne disturba la cadenza.
        if (g_cal_persist_req) {
            g_cal_persist_req = false;
            cal_do_persist(false);   // rate-limited (eps + >=5 min)
        }

        ble_notify_battery(g_bat.last_pct);

        if (g_bat.last_pct < 5) {
            ESP_LOGW(TAG, "Low battery: %d%% (%.0f mV) — entering sleep",
                     g_bat.last_pct, g_bat.last_mv);
            g_state = STATE_LOW_BATTERY;
        }

        int64_t now = esp_timer_get_time();
        if (ota_get_status() == OTA_RUNNING)
            g_last_activity_us = now;  // impedisci sleep durante OTA

        if (!ble_is_connected() && (now - g_last_activity_us) > SLEEP_TIMEOUT_US) {
            ESP_LOGI(TAG, "Inactivity timeout — power off");
            power_off();
        }
    }
}

// ─── Button ISR ─────────────────────────────────────────────────────────────────────────────
static volatile int64_t g_btn_press_us       = 0;
static volatile bool    g_btn_long_press      = false;
static volatile bool    g_btn_very_long_press = false;
static volatile int64_t g_btn_last_tap_us     = 0;
static volatile bool    g_btn_double_click    = false;
static volatile int64_t g_btn_last_edge_us    = 0;

static void IRAM_ATTR btn_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    /* Debounce temporale: ignora edge a <20 ms dall'ultimo edge per evitare
     * doppi g_screen_next / falsi long-press da rimbalzo meccanico. */
    if (g_btn_last_edge_us > 0 && (now - g_btn_last_edge_us) < 20000LL) return;
    g_btn_last_edge_us = now;
    if (gpio_get_level(PIN_BTN_USER) == 0) {
        g_btn_press_us = now;
    } else {
        int64_t held_ms = (now - g_btn_press_us) / 1000;
        if (held_ms >= 5000) {
            g_btn_very_long_press = true;
            g_btn_long_press      = true;
        } else if (held_ms >= 3000) {
            g_btn_long_press = true;
        } else if (held_ms >= 50) {
            g_screen_next = true;
            if (g_btn_last_tap_us > 0 && (now - g_btn_last_tap_us) < 400000LL) {
                g_btn_double_click = true;
                g_btn_last_tap_us  = 0;
            } else {
                g_btn_last_tap_us = now;
            }
        }
    }
}

static void btn_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_BTN_USER),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BTN_USER, btn_isr, NULL);
}

// ─── Calibration procedure ───────────────────────────────────────────────────────────────────────────
static void do_calibration(void)
{
    // ISSUE #30 punto 3: long-press = override esplicito, ma con la STESSA
    // guardia di varianza dell'auto-zero (silenziosa). Su varianza alta tiene
    // il vecchio offset (skip silenzioso). Feedback minimo "ZERO OK" solo a
    // successo (rimosso STATE_CALIBRATING + toast "CALIBRATING" 30 s).
    ESP_LOGI(TAG, "Manual zero: averaging Pitot over 5 seconds...");

    running_stats_t rs;
    rs_reset(&rs);
    int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < 5000000LL) {
        I2C1_LOCK();
        esp_err_t ret = sdp810_read(&g_pitot);
        float raw = g_pitot.pressure_pa;
        I2C1_UNLOCK();
        if (ret == ESP_OK) rs_push(&rs, raw);   // GREZZO (letto sotto I2C lock)
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    float mean_pa;
    if (pitot_window_ok(&rs, &mean_pa)) {
        // Set-RAM serializzato col sensor task / callback BLE (mutex condiviso).
        SENSORS_LOCK();
        cal_zero_pitot(&g_cal, mean_pa);
        SENSORS_UNLOCK();
        // Override manuale = persist immediato (force): bypassa rate-limit, eps.
        cal_do_persist(true);
        display_show_toast("ZERO OK", 2000);
        ESP_LOGI(TAG, "Manual zero: offset=%.2f Pa n=%d std=%.2f",
                 mean_pa, (int)rs.n, rs_std(&rs));
    } else {
        ESP_LOGW(TAG, "Manual zero rifiutato: std=%.2f Pa n=%d (vecchio offset tenuto)",
                 rs_std(&rs), (int)rs.n);
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────────────────────
void app_main(void)
{
    gpio_set_direction(PIN_PWR_HOLD, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PWR_HOLD, 1);

    ESP_LOGI(TAG, "AeroDrag %s — booting...", FW_VERSION_FULL);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    identity_init();
    cal_load(&g_cal);
    // ISSUE #30: stato persistenza inizializzato a cio' che e' realmente su NVS.
    // g_last_persist_us=0 -> il primo auto-zero >eps puo' persistere subito.
    g_last_saved_offset_pa = g_cal.pitot_offset_pa;
    g_last_persist_us      = 0;
    ESP_LOGI(TAG, "Calibration loaded: pitot_offset=%.4f Pa, mass=%.1f kg",
             g_cal.pitot_offset_pa, g_cal.mass_kg);

    ESP_ERROR_CHECK(i2c_init_bus(I2C_NUM_0, PIN_IMU_SDA, PIN_IMU_SCL, I2C0_SPEED_HZ));
    ESP_ERROR_CHECK(i2c_init_bus(I2C_NUM_1, PIN_PITOT_SDA, PIN_PITOT_SCL, I2C1_SPEED_HZ));
    i2c_scan_bus(I2C_NUM_0, "I2C0 (IMU/RTC, GPIO11/10)");
    i2c_scan_bus(I2C_NUM_1, "I2C1 (pitot, GPIO15/18)");

    ret = sdp810_init(&g_pitot, PITOT_I2C_PORT, PITOT_I2C_ADDR);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "SDP810 not found (%s) — pitot disabled", esp_err_to_name(ret));

    ret = qmi8658_init(&g_imu, IMU_I2C_PORT, IMU_I2C_ADDR);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "QMI8658 not found — IMU disabled");

    g_sensors.gps_fix      = 2;
    g_sensors.humidity_pct = 50.0f;

    battery_init(&g_bat, BAT_ADC_CHANNEL);

    ret = display_init();
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "Display init failed: %d", ret);
    display_set_pairing_id(g_identity.device_id);

    g_sensors_mutex = xSemaphoreCreateMutex();
    g_i2c1_mutex    = xSemaphoreCreateMutex();
    configASSERT(g_sensors_mutex);
    configASSERT(g_i2c1_mutex);

    ble_sensors_init(g_sensors_mutex);
    ble_sensors_set_wheel_circumference(g_cal.wheel_circ_m);
    /* Freeze scan before ble_server_init: on_sync fires on the NimBLE
     * host task concurrently and would start a 50%-duty-cycle scan that
     * starves the WPA2 4-way handshake under BLE/WiFi coexistence. */
    ble_sensors_set_scan_enabled(false);

    ESP_ERROR_CHECK(ble_server_init(&g_sensors, g_sensors_mutex, &g_cal));
    ESP_LOGI(TAG, "BLE Central (Power/CSC/HR) pronto");

    esp_err_t coach_ret = coach_init();
    /* For OFF and timeout: scan resumes now. For success (ESP_OK):
     * the GOT_IP handler in wifi_coach.h re-enables the scan, so this
     * is a harmless no-op. */
    ble_sensors_set_scan_enabled(true);
    if (coach_ret == ESP_OK && coach_get_mode() != COACH_MODE_OFF)
        ESP_LOGI(TAG, "Coach: %s → %s",
                 coach_get_mode() == COACH_MODE_COACH_DIRECT ? "COACH DIRECT" : "COACH CO-OP",
                 coach_get_ssid());
    else if (coach_get_mode() == COACH_MODE_OFF)
        ESP_LOGI(TAG, "Coach: OFF");

    // Fix C1: task_pitot_imu creato una sola volta.
    // Il precedente codice lo creava due volte (refactoring incompleto di task_ant).
    btn_init();
    xTaskCreatePinnedToCore(task_pitot_imu,    "pitot_imu",  4096, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(task_display,      "display",    8192, NULL, 2, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(task_housekeeping, "housekeep",  3072, NULL, 1, NULL, PRO_CPU_NUM);

    // Fix H3: il watchdog parte dal completamento del boot, non dal valore 0.
    g_last_activity_us = esp_timer_get_time();

    // OTA rollback: conferma questo firmware come valido.
    // Se l'avvio crasha prima di questa riga, il bootloader ripristina il
    // firmware precedente al prossimo riavvio (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE).
    ota_mark_valid();

    ESP_LOGI(TAG, "All tasks started — CdA measurement active");
    g_state = STATE_IDLE;

    while (1) {
        if (g_btn_double_click) {
            g_btn_double_click = false;
            int64_t now = esp_timer_get_time();
            g_session_start_us = now;
            g_lap_start_us     = now;
            g_current_lap      = 1;
            display_set_screen(SCR_TIMER);
            display_show_toast("LET'S GO!", 2000);
            ESP_LOGI(TAG, "Sessione avviata manualmente");
        }
        if (g_btn_long_press && !g_btn_very_long_press) {
            g_btn_long_press = false;
            do_calibration();
        }
        if (g_btn_very_long_press) {
            g_btn_very_long_press = false;
            g_btn_long_press      = false;
            coach_cycle_mode();
            {
                const char *labels[] = {"WIFI OFF", "COACH DIRECT", "CO-OP WIFI"};
                display_show_toast(labels[coach_get_mode()], 3000);
            }
            coach_apply_mode();
        }
        if (g_state == STATE_LOW_BATTERY) {
            ESP_LOGW(TAG, "Shutting down — battery critical");
            cal_do_persist(true);   // ISSUE #30: salva offset prima dello stop sensori (force)
            sdp810_stop(&g_pitot);
            vTaskDelay(pdMS_TO_TICKS(500));
            power_off();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
