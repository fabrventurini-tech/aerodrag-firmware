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

// Components
#include "../components/pitot/sdp810.h"
#include "../components/pitot/physics.h"
#include "../components/imu/qmi8658.h"
#include "../components/ble/ble_server.h"
#include "../components/ble_central/ble_sensors.h"  // BLE Central — Power/CSC/HR
#include "../components/display/display.h"
#include "../components/battery/battery.h"
#include "../components/wifi/wifi_coach.h"
#include "esp_mac.h"

static const char *TAG = "aerodrag";

// ─── Device identity ──────────────────────────────────────────────────────────
device_identity_t g_identity = {0};

static void identity_init(void)
{
    // Leggi MAC address (univoco per ogni ESP32 — scritto in eFuse in fabbrica)
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(g_identity.device_id, sizeof(g_identity.device_id),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Carica nome atleta da NVS (configurato dall'app al primo pairing)
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

// ─── Global shared state ──────────────────────────────────────────────────────
static aerodrag_sensors_t  g_sensors = {0};
static aerodrag_physics_t  g_physics = {0};
static aerodrag_cal_t      g_cal     = {0};
static aerodrag_state_t    g_state   = STATE_INIT;

// Bug fix 1: mutex protecting g_sensors — written by 3 tasks, read by pitot+display
static SemaphoreHandle_t   g_sensors_mutex = NULL;

// Bug fix 2: ISR-safe screen change flag — ISR cannot call display functions directly
static volatile bool       g_screen_next = false;

// Bug fix 3: I2C mutex — sdp810_read() called from task_pitot_imu AND do_calibration()
static SemaphoreHandle_t   g_i2c1_mutex = NULL;

// CdA smoothing (30-sample EMA)
static float g_cda_smooth = 0.0f;

// Contatore lap condiviso — scritto da task_ant (Garmin Controls) e da
// coach_handle_command (dashboard WebSocket). Letto da coach_send_frame.
uint8_t g_current_lap = 1;

// Last activity timestamp for deep sleep watchdog
static int64_t g_last_activity_us = 0;
#define SLEEP_TIMEOUT_US  (10LL * 60 * 1000 * 1000)  // 10 minutes

// Convenience macros for sensor mutex
#define SENSORS_LOCK()    xSemaphoreTake(g_sensors_mutex, portMAX_DELAY)
#define SENSORS_UNLOCK()  xSemaphoreGive(g_sensors_mutex)
#define I2C1_LOCK()       xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY)
#define I2C1_UNLOCK()     xSemaphoreGive(g_i2c1_mutex)

// Callback BLE lap per modalità CO_OP — definita a livello file (non annidata)
// Chiamata da wifi_coach.h quando il coach invia un LAP e la modalità è CO_OP
static void ble_lap_notify_cb(void) { ble_notify_ant(0xFFFF, 0, 0); }
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

// ─── Sensor drivers ───────────────────────────────────────────────────────────
static sdp810_t  g_pitot = {0};
static qmi8658_t g_imu   = {0};
static battery_t g_bat   = {0};

// BLE Central stale detection — condiviso con task_pitot_imu
static int64_t g_last_speed_us = 0;
#define BLE_SPEED_STALE_US  (5LL * 1000 * 1000)

// ─── Task: Pitot + IMU @ 10 Hz ───────────────────────────────────────────────
static void task_pitot_imu(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint8_t env_divider = 0;

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));  // 10 Hz

        // ── Leggi sensori BLE Central (Power, CSC, HR) ─────────────────────
        // Sostituisce la lettura UART da task_ant. I dati arrivano via notifiche
        // BLE (callback su PRO_CPU), qui li preleviamo in modo thread-safe.
        {
            ble_sensor_data_t ext;
            ble_sensors_get(&ext);

            // Stale detection: se non riceviamo speed BLE da >5s, invalidiamo
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
        ESP_LOGI("PITOT", "Pa=%.3f ret=%d", g_pitot.pressure_pa, pitot_ret);

        // Fix D: read IMU outside mutex — I2C takes ~270μs at 400kHz.
        // Take local copies of driver output, then lock only for the struct write.
        float imu_pitch = 0, imu_roll = 0, imu_temp = 0;
        bool  imu_ok = false;
        if (qmi8658_read(&g_imu) == ESP_OK) {
            imu_pitch = g_imu.pitch_deg;
            imu_roll  = g_imu.roll_deg;
            imu_temp  = g_imu.temp_c;
            imu_ok    = true;
        }

        SENSORS_LOCK();
        if (pitot_ret == ESP_OK) {
            g_sensors.pitot_pa    = g_pitot.pressure_pa;
            g_sensors.static_pa   = 101325.0f;
            g_sensors.pitot_valid = true;
            // Usa la temperatura del SDP810 (più vicina all'aria reale) invece dell'IMU
            // (che è dentro il device riscaldato dall'elettronica → +5-10°C errore su rho)
            g_sensors.temp_c      = g_pitot.temp_c;
        }
        if (imu_ok) {
            g_sensors.pitch_deg = imu_pitch;
            g_sensors.roll_deg  = imu_roll;
            // IMU temp NON usata per density — temp_c viene dal Pitot quando disponibile
            // Solo fallback se Pitot è invalido
            if (!g_sensors.pitot_valid) g_sensors.temp_c = imu_temp;
            g_sensors.imu_valid = true;
        }

        aerodrag_sensors_t sensors_copy = g_sensors;
        SENSORS_UNLOCK();

        aerodrag_physics_t phy = physics_compute(&sensors_copy, &g_cal);

        // ── Gestione LAP event — indipendente da physics.valid ───────────────
        // Il tasto LAP Garmin deve funzionare anche da fermo (physics non valida)
        SENSORS_LOCK();
        bool lap_now = sensors_copy.lap_event;
        if (lap_now) g_sensors.lap_event = false;
        uint8_t bat = g_sensors.battery_pct;
        SENSORS_UNLOCK();

        bool lap_pending_this_cycle = false;
        if (lap_now) {
            g_current_lap++;
            lap_pending_this_cycle = true;
            ESP_LOGI("lap", "LAP event ANT+ → lap %d", g_current_lap);
        }

        // (g_physics assignment moved below, after invalid handling)
        if (phy.valid) {
            if (g_cda_smooth < 0.01f) g_cda_smooth = phy.CdA;
            g_cda_smooth = ema_update(g_cda_smooth, phy.CdA, EMA_ALPHA_30S);
            phy.CdA = g_cda_smooth;
            SENSORS_LOCK();
            g_physics = phy;
            SENSORS_UNLOCK();
            g_last_activity_us = esp_timer_get_time();

            // Trasmissione diretta al Pi via WiFi (se abilitata)
            if (coach_get_mode() != COACH_MODE_OFF)
                coach_send_frame(&sensors_copy, &phy, bat, lap_now);
        } else {
            // Fix M6: aggiorna g_physics anche quando invalid — altrimenti il
            // display continua a mostrare l'ultimo CdA valido all'infinito
            SENSORS_LOCK();
            g_physics = phy;   // phy.valid = false, gli altri campi sono 0
            SENSORS_UNLOCK();
        } // end if (phy.valid)

        // Comandi coach ricevuti via WiFi (start/stop/lap sessione)
        if (g_coach_lap_cmd) {
            g_coach_lap_cmd = false;
            // Fix M2: incrementa solo se non già incrementato in questo ciclo
            // (caso raro: Garmin LAP e coach LAP nello stesso ciclo 100ms)
            if (!lap_pending_this_cycle) {
                g_current_lap++;
                lap_pending_this_cycle = true;
                ESP_LOGI(TAG, "LAP da coach WiFi → lap %d", g_current_lap);
            } else {
                ESP_LOGW(TAG, "LAP coach soppresso: già fired da ANT+ stesso ciclo");
            }
        }

        // Fix M2: una sola ble_notify_ant(0xFFFF) per ciclo anche se LAP da entrambe sorgenti
        if (lap_pending_this_cycle) {
            ble_notify_ant(0xFFFF, 0, 0);
        }
        if (g_coach_start_cmd) {
            g_coach_start_cmd = false;
            g_state = STATE_RECORDING;
            ESP_LOGI(TAG, "Sessione avviata da coach");
        }
        if (g_coach_stop_cmd) {
            g_coach_stop_cmd = false;
            g_state = STATE_CONNECTED;
            ESP_LOGI(TAG, "Sessione fermata da coach");
        }

        // BLE notify — use local copies (already have sensors_copy from above)
        ble_notify_pitot(sensors_copy.pitot_pa, sensors_copy.static_pa);
        ble_notify_imu(sensors_copy.pitch_deg, sensors_copy.roll_deg);

        if (++env_divider >= 10) {
            env_divider = 0;
            ble_notify_env(sensors_copy.temp_c, sensors_copy.humidity_pct,
                           sensors_copy.altitude_m, sensors_copy.speed_ms);
        }
    }
}

// task_gps rimossa — la velocità viene ora dal dongle ANT+ (profilo 124 Speed&Distance)
// Il modulo GPS SAM-M10Q non è più necessario nel progetto.

// ─── Task: Display @ 5 Hz ────────────────────────────────────────────────────
static void task_display(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(200));  // 5 Hz

        // Bug fix: handle screen change flag set by ISR
        if (g_screen_next) {
            g_screen_next = false;
            display_next_screen();
        }

        SENSORS_LOCK();
        aerodrag_sensors_t s_copy = g_sensors;
        aerodrag_physics_t p_copy = g_physics;
        SENSORS_UNLOCK();

        display_render(&s_copy, &p_copy);
    }
}

// ─── Task: Battery + housekeeping @ 0.1 Hz ───────────────────────────────────
static void task_housekeeping(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // every 10s

        battery_update(&g_bat);
        // Fix B: protect battery field writes — task_pitot_imu reads g_sensors concurrently
        SENSORS_LOCK();
        g_sensors.battery_pct = g_bat.last_pct;
        g_sensors.battery_mv  = g_bat.last_mv;
        SENSORS_UNLOCK();

        if (g_bat.last_pct < 5) {
            ESP_LOGW(TAG, "Low battery: %d%% (%.0f mV) — entering sleep",
                     g_bat.last_pct, g_bat.last_mv);
            g_state = STATE_LOW_BATTERY;
        }

        // Deep sleep watchdog
        int64_t now = esp_timer_get_time();
        if (!ble_is_connected() && (now - g_last_activity_us) > SLEEP_TIMEOUT_US) {
            ESP_LOGI(TAG, "Inactivity timeout — deep sleep");
            esp_deep_sleep(0);  // wake on button press only
        }
    }
}

// ─── Button ISR ───────────────────────────────────────────────────────────────
// Fix C: volatile required — g_btn_press_us written and read from ISR context
static volatile int64_t g_btn_press_us      = 0;
static volatile bool    g_btn_long_press     = false;
static volatile bool    g_btn_very_long_press = false;  // 5s → cicla modalità coach

static void IRAM_ATTR btn_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (gpio_get_level(PIN_BTN_USER) == 0) {
        g_btn_press_us = now;
    } else {
        int64_t held_ms = (now - g_btn_press_us) / 1000;
        if (held_ms >= 5000) {
            // 5 secondi: cicla modalità coach WiFi
            // g_btn_very_long_press è già definita in questo file — no extern necessario
            g_btn_very_long_press = true;
            g_btn_long_press      = true;
        } else if (held_ms >= 3000) {
            g_btn_long_press = true;
        } else if (held_ms >= 50) {
            g_screen_next = true;
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

// ─── Calibration procedure ────────────────────────────────────────────────────
static void do_calibration(void)
{
    ESP_LOGI(TAG, "Calibration: averaging Pitot over 5 seconds...");
    g_state = STATE_CALIBRATING;

    float sum = 0;
    int   n   = 0;
    int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < 5000000LL) {
        // Bug fix: take I2C1 mutex — task_pitot_imu also uses sdp810_read() on I2C1
        I2C1_LOCK();
        esp_err_t ret = sdp810_read(&g_pitot);
        I2C1_UNLOCK();
        if (ret == ESP_OK) {
            sum += g_pitot.pressure_pa;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (n > 0) {
        cal_zero_pitot(&g_cal, sum / n);
        ESP_LOGI(TAG, "Pitot zero offset set to %.4f Pa (avg of %d samples)",
                 g_cal.pitot_offset_pa, n);
    }
    g_state = STATE_CONNECTED;
}

// ─── Main ─────────────────────────────────────────────────────────────────────
void app_main(void)
{
    // Power hold — keep board powered when running on battery
    gpio_set_direction(PIN_PWR_HOLD, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PWR_HOLD, 1);

    ESP_LOGI(TAG, "AeroDrag firmware v1.0 — booting...");

    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Leggi MAC address e nome atleta — prima di tutto il resto
    identity_init();
    cal_load(&g_cal);
    ESP_LOGI(TAG, "Calibration loaded: pitot_offset=%.4f Pa, mass=%.1f kg",
             g_cal.pitot_offset_pa, g_cal.mass_kg);

    // I2C buses
    ESP_ERROR_CHECK(i2c_init_bus(I2C_NUM_0, PIN_IMU_SDA, PIN_IMU_SCL, I2C0_SPEED_HZ));
    ESP_ERROR_CHECK(i2c_init_bus(I2C_NUM_1, PIN_PITOT_SDA, PIN_PITOT_SCL, I2C1_SPEED_HZ));

    // Pitot
    ret = sdp810_init(&g_pitot, PITOT_I2C_PORT, PITOT_I2C_ADDR);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "SDP810 not found (%.2X) — pitot disabled", ret);

    // IMU (already on board)
    ret = qmi8658_init(&g_imu, IMU_I2C_PORT, IMU_I2C_ADDR);
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "QMI8658 not found — IMU disabled");

    // GPS — RIMOSSO: la velocità viene ora dal dongle ANT+ (profilo 124)
    g_sensors.gps_fix      = 2;      // segnala sempre valid (ANT speed è affidabile)
    g_sensors.humidity_pct = 50.0f; // default 50% — nessun sensore umidità
    ESP_LOGI(TAG, "GPS rimosso — velocità da ANT+ dongle. Umidità default 50%%");

    // Battery ADC — GPIO1 = ADC1_CH0 (board_pins.h: PIN_BAT_ADC = 1)
    battery_init(&g_bat, ADC_CHANNEL_0);

    // Display
    ret = display_init();
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "Display init failed: %d", ret);

    // Fix A — corrected: create mutexes BEFORE ble_server_init() which launches NimBLE task.
    // NimBLE task starts immediately inside nimble_port_freertos_init() and could call
    // on_sync() → ble_advertise() before mutexes existed in previous version.
    g_sensors_mutex = xSemaphoreCreateMutex();
    g_i2c1_mutex    = xSemaphoreCreateMutex();
    configASSERT(g_sensors_mutex);
    configASSERT(g_i2c1_mutex);

    // BLE GATT server — launched AFTER mutexes exist
    ESP_ERROR_CHECK(ble_server_init(&g_sensors, g_sensors_mutex));

    // Coach WiFi bridge (opzionale — si attiva se modalità != OFF in NVS)
    // Non bloccante: se il Pi non è disponibile prosegue in BLE-only
    esp_err_t coach_ret = coach_init();
    if (coach_ret == ESP_OK && coach_get_mode() != COACH_MODE_OFF)
        ESP_LOGI(TAG, "Coach: %s → %s",
                 coach_get_mode() == COACH_MODE_COACH_DIRECT ? "COACH DIRECT" : "COACH CO-OP",
                 coach_get_ssid());
    else if (coach_get_mode() == COACH_MODE_OFF)
        ESP_LOGI(TAG, "Coach: OFF");

    // Registra callback BLE→lap per modalità CO_OP
    coach_set_ble_lap_cb(ble_lap_notify_cb);

    // Inizializza BLE Central — legge Power/CSC/HR dai sensori BLE dell'atleta.
    // Sostituisce il dongle nRF52840 + uart_bridge. Il mutex g_sensors_mutex
    // è già creato sopra; on_sync() in ble_server.h chiamerà ble_sensors_on_sync()
    // per avviare lo scan quando NimBLE è pronto.
    ble_sensors_init(g_sensors_mutex);
    ESP_LOGI(TAG, "BLE Central (Power/CSC/HR) pronto — scan partirà con NimBLE sync");

    xTaskCreatePinnedToCore(task_pitot_imu,    "pitot_imu",  4096, NULL, 5, NULL, APP_CPU_NUM);
    // task_ant rimosso — sensori BLE letti dentro task_pitot_imu via ble_sensors_get()
    btn_init();
    xTaskCreatePinnedToCore(task_pitot_imu,    "pitot_imu",  4096, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(task_display,      "display",    8192, NULL, 2, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(task_housekeeping, "housekeep",  2048, NULL, 1, NULL, PRO_CPU_NUM);

    ESP_LOGI(TAG, "All tasks started — CdA measurement active");
    g_state = STATE_IDLE;

    // ── Main loop: handle calibration trigger from button ───────────────────
    while (1) {
        if (g_btn_long_press && !g_btn_very_long_press) {
            g_btn_long_press = false;
            do_calibration();
        }
        if (g_btn_very_long_press) {
            g_btn_very_long_press = false;
            g_btn_long_press      = false;
            coach_cycle_mode();
            // Il display mostrerà la modalità per 2s grazie a g_coach_mode_display
        }
        if (g_state == STATE_LOW_BATTERY) {
            ESP_LOGW(TAG, "Shutting down — battery critical");
            sdp810_stop(&g_pitot);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_deep_sleep(0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
