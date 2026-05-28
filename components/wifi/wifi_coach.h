#pragma once
/**
 * AeroDrag WiFi Coach Bridge
 * ─────────────────────────────────────────────────────────────────────────────
 * Connette l'ESP32-S3 direttamente all'hotspot WiFi del Raspberry Pi
 * ed invia i dati via WebSocket, eliminando il telefono come intermediario.
 *
 * Modalità operative (salvate in NVS, cambio con BOOT tenuto 5 secondi):
 *   COACH_MODE_OFF           (0) — WiFi spento, solo BLE  [default]
 *   COACH_MODE_COACH_DIRECT  (1) — WiFi diretto al Pi, BLE disabilitato
 *   COACH_MODE_CO_OP    (2) — BLE + WiFi simultanei (Coach Co-op)
 *
 * Frame JSON verso Pi — identico a quello dell'app mobile:
 * {"t":ms,"CdA":0.241,"pwr":247,"spd":38.4,"pitch":42.1,
 *  "hr":153,"cad":89,"wind":1.2,"rho":1.204,"pctAero":76,
 *  "lap":3,"lapEvent":false,"battery":78}
 *
 * Ricezione comandi dal Pi (coach → device):
 * {"type":"cmd","action":"lap"|"start"|"stop"}
 */

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_websocket_client.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "aerodrag_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

#define COACH_TAG "coach"

// ─── NVS keys ─────────────────────────────────────────────────────────────────
#define COACH_NVS_KEY_SSID  "coach_ssid"
#define COACH_NVS_KEY_PASS  "coach_pass"
#define COACH_NVS_KEY_HOST  "coach_host"
#define COACH_NVS_KEY_PORT  "coach_port"
#define COACH_NVS_KEY_MODE  "coach_mode"

// ─── Default: Pi hotspot creato da setup.sh ───────────────────────────────────
#define COACH_DEFAULT_SSID  "AeroDrag"
#define COACH_DEFAULT_PASS  "aerodrag2024"
#define COACH_DEFAULT_HOST  "192.168.8.1"   // IP Pi su wlan0 (da setup.sh)
#define COACH_DEFAULT_PORT  8080

typedef enum {
    COACH_MODE_OFF           = 0,
    COACH_MODE_COACH_DIRECT  = 1,
    COACH_MODE_CO_OP = 2,
} coach_mode_t;

typedef struct {
    char          ssid[33];
    char          pass[65];
    char          host[40];
    uint16_t      port;
    coach_mode_t  mode;
} coach_config_t;

// ─── Lap counter — condiviso con main.c ──────────────────────────────────────
extern uint8_t g_current_lap;
extern device_identity_t g_identity;

// ─── Internal state ───────────────────────────────────────────────────────────
static coach_config_t                g_cfg     = {0};
static esp_websocket_client_handle_t g_ws      = NULL;
static EventGroupHandle_t            g_wifi_eg = NULL;
static bool                          g_ws_ready = false;
// g_cur_lap rimosso — usa g_current_lap da main.c (contatore unico condiviso)

// Flags settati dall'handler comandi e letti dal main loop
volatile bool g_coach_start_cmd = false;
volatile bool g_coach_stop_cmd  = false;
volatile bool g_coach_lap_cmd   = false;   // Fix race: lap gestito nel main loop
volatile uint8_t g_coach_mode_display = 0;   // 0=nessun messaggio, 1-3=mostra modalità

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

// Fix 4: function pointer callback to avoid circular include wifi_coach.h ↔ ble_server.h
// Set by main.c after both modules are initialized: coach_set_ble_lap_cb(ble_notify_ant_lap)
static void (*g_ble_lap_cb)(void) = NULL;
void coach_set_ble_lap_cb(void (*cb)(void)) { g_ble_lap_cb = cb; }

// Forward declaration — definita sotto, chiamata da _ws_handler
static void coach_handle_command(const char *data, int len);

// Timer per reconnect WiFi — evita vTaskDelay nell'event handler di sistema
static esp_timer_handle_t g_wifi_reconnect_timer = NULL;
static void _wifi_reconnect_cb(void *arg) { esp_wifi_connect(); }

static void _wifi_handler(void *arg, esp_event_base_t base,
                           int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            g_ws_ready = false;
            ESP_LOGW(COACH_TAG, "WiFi perso — riconnessione tra 3s");
            // Fix W1: NON usare vTaskDelay nell'event handler — blocca sys_evt task.
            // Usa esp_timer per schedule asincrono della riconnessione.
            if (!g_wifi_reconnect_timer) {
                const esp_timer_create_args_t ta = {
                    .callback = _wifi_reconnect_cb,
                    .name     = "wifi_reconnect",
                };
                esp_timer_create(&ta, &g_wifi_reconnect_timer);
            }
            esp_timer_start_once(g_wifi_reconnect_timer, 3000000LL); // 3s in µs
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(COACH_TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        if (g_wifi_eg) xEventGroupSetBits(g_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

// ─── WebSocket event handler ──────────────────────────────────────────────────
static void _ws_handler(void *arg, esp_event_base_t base,
                         int32_t id, void *data)
{
    esp_websocket_event_data_t *evt = (esp_websocket_event_data_t *)data;
    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(COACH_TAG, "✓ Connesso al Pi come %s (%s)",
                 g_identity.athlete_name, g_identity.device_id);
        g_ws_ready = true;
        {
            // Invia hello con device_id e nome atleta per identificazione univoca
            char hello_buf[120];
            snprintf(hello_buf, sizeof(hello_buf),
                "{\"type\":\"hello\",\"device\":\"%s\",\"athlete\":\"%s\"}",
                g_identity.device_id, g_identity.athlete_name);
            esp_websocket_client_send_text(g_ws, hello_buf, -1, pdMS_TO_TICKS(500));
        }
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        g_ws_ready = false;
        ESP_LOGW(COACH_TAG, "WS disconnesso");
        break;
    case WEBSOCKET_EVENT_DATA:
        if (evt->data_len > 0 && evt->data_ptr)
            coach_handle_command(evt->data_ptr, evt->data_len);
        break;
    case WEBSOCKET_EVENT_ERROR:
        g_ws_ready = false;
        break;
    default: break;
    }
}

// ─── Gestione comandi ricevuti dal coach ──────────────────────────────────────
static void coach_handle_command(const char *data, int len)
{
    char buf[96] = {0};
    memcpy(buf, data, len < 95 ? len : 95);

    if (strstr(buf, "\"lap\"")) {
        // Fix race: g_current_lap++ non è atomico su LX7.
        // Usa flag — il main loop incrementa g_current_lap in sicurezza.
        g_coach_lap_cmd = true;
        ESP_LOGI(COACH_TAG, "LAP dal coach ricevuto");
        // In CO_OP la notifica BLE viene gestita dal main loop tramite g_coach_lap_cmd
    } else if (strstr(buf, "\"start\"")) {
        ESP_LOGI(COACH_TAG, "START dal coach");
        g_coach_start_cmd = true;
    } else if (strstr(buf, "\"stop\"")) {
        ESP_LOGI(COACH_TAG, "STOP dal coach");
        g_coach_stop_cmd = true;
    }
}

// ─── Invia frame dati al Pi (chiamato da task_pitot_imu a 10Hz) ───────────────
esp_err_t coach_send_frame(const aerodrag_sensors_t *s,
                            const aerodrag_physics_t *p,
                            uint8_t battery_pct,
                            bool    lap_event)
{
    if (!g_ws_ready || !g_ws || !p->valid) return ESP_ERR_INVALID_STATE;

    int64_t ts_ms = esp_timer_get_time() / 1000LL;

    // Fix N2: defense in depth — l'app già sanitizza ma il firmware fa la stessa
    // operazione su una copia locale. Difesa contro versioni vecchie dell'app
    // che potrebbero scrivere caratteri " o \ nel campo athlete_name.
    char athlete_safe[sizeof(g_identity.athlete_name)];
    strlcpy(athlete_safe, g_identity.athlete_name, sizeof(athlete_safe));
    for (size_t i = 0; i < sizeof(athlete_safe) && athlete_safe[i]; i++) {
        char c = athlete_safe[i];
        if (c == '"' || c == '\\' || (unsigned char)c < 0x20) athlete_safe[i] = '_';
    }

    char json[400];
    int n = snprintf(json, sizeof(json),
        "{\"device\":\"%s\",\"athlete\":\"%s\","
        "\"t\":%lld,\"CdA\":%.4f,\"pwr\":%d,\"spd\":%.1f,"
        "\"pitch\":%.1f,\"hr\":%d,\"cad\":%d,\"wind\":%.2f,"
        "\"rho\":%.4f,\"pctAero\":%d,\"lap\":%d,"
        "\"lapEvent\":%s,\"battery\":%d}",
        g_identity.device_id, athlete_safe,
        (long long)ts_ms, p->CdA, s->power_w, s->speed_ms * 3.6f,
        s->pitch_deg, s->hr_bpm, s->cadence_rpm, p->wind_ms,
        p->rho, p->pct_aero, g_current_lap,
        lap_event ? "true" : "false", battery_pct);

    if (n <= 0 || n >= (int)sizeof(json)) return ESP_ERR_INVALID_SIZE;

    return esp_websocket_client_send_text(g_ws, json, n, pdMS_TO_TICKS(30));
}

// ─── NVS config ───────────────────────────────────────────────────────────────
esp_err_t coach_config_load(coach_config_t *cfg)
{
    nvs_handle_t h;
    // Imposta defaults prima di leggere
    strlcpy(cfg->ssid, COACH_DEFAULT_SSID, sizeof(cfg->ssid));
    strlcpy(cfg->pass, COACH_DEFAULT_PASS, sizeof(cfg->pass));
    strlcpy(cfg->host, COACH_DEFAULT_HOST, sizeof(cfg->host));
    cfg->port = COACH_DEFAULT_PORT;
    cfg->mode = COACH_MODE_OFF;

    if (nvs_open("aerodrag", NVS_READONLY, &h) != ESP_OK) return ESP_ERR_NOT_FOUND;
    size_t sz;
    sz = sizeof(cfg->ssid); nvs_get_str(h, COACH_NVS_KEY_SSID, cfg->ssid, &sz);
    sz = sizeof(cfg->pass); nvs_get_str(h, COACH_NVS_KEY_PASS, cfg->pass, &sz);
    sz = sizeof(cfg->host); nvs_get_str(h, COACH_NVS_KEY_HOST, cfg->host, &sz);
    uint16_t port = cfg->port; nvs_get_u16(h, COACH_NVS_KEY_PORT, &port); cfg->port = port;
    uint8_t  mode = 0;        nvs_get_u8(h,  COACH_NVS_KEY_MODE, &mode);  cfg->mode = mode;
    nvs_close(h);
    return ESP_OK;
}

esp_err_t coach_config_save(const coach_config_t *cfg)
{
    nvs_handle_t h;
    if (nvs_open("aerodrag", NVS_READWRITE, &h) != ESP_OK) return ESP_FAIL;
    nvs_set_str(h, COACH_NVS_KEY_SSID, cfg->ssid);
    nvs_set_str(h, COACH_NVS_KEY_PASS, cfg->pass);
    nvs_set_str(h, COACH_NVS_KEY_HOST, cfg->host);
    nvs_set_u16(h, COACH_NVS_KEY_PORT, cfg->port);
    nvs_set_u8(h,  COACH_NVS_KEY_MODE, (uint8_t)cfg->mode);
    esp_err_t ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

// ─── Init (chiamato da app_main dopo nvs_flash_init) ─────────────────────────
esp_err_t coach_init(void)
{
    coach_config_load(&g_cfg);
    if (g_cfg.mode == COACH_MODE_OFF) {
        ESP_LOGI(COACH_TAG, "Modalità: OFF");
        return ESP_OK;
    }

    ESP_LOGI(COACH_TAG, "Modalità: %s — WiFi '%s' → %s:%d",
             g_cfg.mode == COACH_MODE_CO_OP ? "COACH CO-OP" : "COACH DIRECT",
             g_cfg.ssid, g_cfg.host, g_cfg.port);

    g_wifi_eg = xEventGroupCreate();

    // Fix W2: esp_netif_init() può essere già stato chiamato da NimBLE/BLE stack
    // su alcune versioni di ESP-IDF. La seconda chiamata restituisce
    // ESP_ERR_INVALID_STATE — non è un errore fatale, procediamo.
    esp_err_t netif_ret = esp_netif_init();
    if (netif_ret != ESP_OK && netif_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(COACH_TAG, "esp_netif_init failed: %d", netif_ret);
        return netif_ret;
    }
    esp_netif_create_default_wifi_sta();

    // Fix 3: create default event loop only if not already created by NimBLE/BLE stack
    esp_err_t loop_ret = esp_event_loop_create_default();
    if (loop_ret != ESP_OK && loop_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(COACH_TAG, "event loop create failed: %d", loop_ret);
        return loop_ret;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));
    esp_event_handler_instance_register(WIFI_EVENT,  ESP_EVENT_ANY_ID,  _wifi_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT,    IP_EVENT_STA_GOT_IP, _wifi_handler, NULL, NULL);

    wifi_config_t wc = {0};
    strlcpy((char*)wc.sta.ssid,     g_cfg.ssid, sizeof(wc.sta.ssid));
    strlcpy((char*)wc.sta.password, g_cfg.pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Attendi IP (max 15s)
    EventBits_t bits = xEventGroupWaitBits(g_wifi_eg,
        WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(COACH_TAG, "Timeout WiFi — fallback BLE, spengo radio per risparmio batteria");
        // Fix N5: senza questo cleanup, ESP32 continuerebbe a scansionare WiFi
        // in background → 60-80mA in più per nulla
        esp_wifi_stop();
        return ESP_ERR_TIMEOUT;
    }

    // WebSocket
    char url[80];
    snprintf(url, sizeof(url), "ws://%s:%d/coach", g_cfg.host, g_cfg.port);
    esp_websocket_client_config_t wsc = {
        .uri = url,
        .reconnect_timeout_ms = 3000,
        .network_timeout_ms   = 5000,
    };
    g_ws = esp_websocket_client_init(&wsc);
    esp_websocket_register_events(g_ws, WEBSOCKET_EVENT_ANY, _ws_handler, NULL);
    return esp_websocket_client_start(g_ws);
}

// ─── Cicla modalità con pressione lunga BOOT (5s) ────────────────────────────
void coach_cycle_mode(void)
{
    g_cfg.mode = (coach_mode_t)((g_cfg.mode + 1) % 3);
    coach_config_save(&g_cfg);
    const char *labels[] = {"OFF", "COACH DIRECT", "COACH CO-OP"};
    ESP_LOGI(COACH_TAG, "→ %s", labels[g_cfg.mode]);
    g_coach_mode_display = (uint8_t)g_cfg.mode + 1;  // display task lo mostra 2s
}

// Getters
uint8_t          coach_get_lap(void)    { return g_current_lap; }
bool             coach_is_ready(void)   { return g_ws_ready; }
coach_mode_t     coach_get_mode(void)   { return g_cfg.mode; }
const char      *coach_get_ssid(void)   { return g_cfg.ssid; }
