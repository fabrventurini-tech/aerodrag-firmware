#pragma once
/**
 * AeroDrag WiFi Coach Bridge
 * ─────────────────────────────────────────────────────────────────────────────
 * Connette l'ESP32-S3 all'hotspot WiFi del Raspberry Pi e invia dati via WebSocket.
 *
 * Modalità (NVS, cambio con BOOT 5s):
 *   COACH_MODE_OFF           (0) — WiFi spento, solo BLE  [default]
 *   COACH_MODE_COACH_DIRECT  (1) — WiFi diretto al Pi, BLE disabilitato
 *   COACH_MODE_CO_OP         (2) — BLE + WiFi simultanei
 *
 * Comandi dal Pi: {"type":"cmd","action":"lap"|"start"|"stop"|"ota"}
 * OTA:            {"type":"cmd","action":"ota","url":"http://192.168.8.1:8080/fw.bin"}
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
#include "version.h"
#include "ota_update.h"
#include <string.h>
#include <stdio.h>

#define COACH_TAG "coach"

#define COACH_NVS_KEY_SSID  "coach_ssid"
#define COACH_NVS_KEY_PASS  "coach_pass"
#define COACH_NVS_KEY_HOST  "coach_host"
#define COACH_NVS_KEY_PORT  "coach_port"
#define COACH_NVS_KEY_MODE  "coach_mode"

#define COACH_DEFAULT_SSID  "AeroDrag"
#define COACH_DEFAULT_PASS  "aerodrag2024"
#define COACH_DEFAULT_HOST  "192.168.8.1"
#define COACH_DEFAULT_PORT  8080

typedef enum {
    COACH_MODE_OFF           = 0,
    COACH_MODE_COACH_DIRECT  = 1,
    COACH_MODE_CO_OP         = 2,
} coach_mode_t;

typedef struct {
    char          ssid[33];
    char          pass[65];
    char          host[40];
    uint16_t      port;
    coach_mode_t  mode;
} coach_config_t;

extern uint16_t g_current_lap;
extern device_identity_t g_identity;

static coach_config_t                g_cfg     = {0};
static esp_websocket_client_handle_t g_ws      = NULL;
static EventGroupHandle_t            g_wifi_eg = NULL;
static bool                          g_ws_ready = false;
static bool                          g_wifi_hw_started = false;

volatile bool    g_coach_start_cmd    = false;
volatile bool    g_coach_stop_cmd     = false;
volatile bool    g_coach_lap_cmd      = false;
volatile uint8_t g_coach_mode_display = 0;

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static void (*g_ble_lap_cb)(void) = NULL;
void coach_set_ble_lap_cb(void (*cb)(void)) { g_ble_lap_cb = cb; }

static void coach_handle_command(const char *data, int len);

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
            if (!g_wifi_reconnect_timer) {
                const esp_timer_create_args_t ta = {
                    .callback = _wifi_reconnect_cb,
                    .name     = "wifi_reconnect",
                };
                esp_timer_create(&ta, &g_wifi_reconnect_timer);
            }
            esp_timer_start_once(g_wifi_reconnect_timer, 3000000LL);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(COACH_TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        if (g_wifi_eg) xEventGroupSetBits(g_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static void _ws_handler(void *arg, esp_event_base_t base,
                         int32_t id, void *data)
{
    esp_websocket_event_data_t *evt = (esp_websocket_event_data_t *)data;
    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(COACH_TAG, "✓ Connesso al Pi come %s (%s) fw=%s",
                 g_identity.athlete_name, g_identity.device_id, FW_VERSION_STR);
        g_ws_ready = true;
        {
            char hello_buf[160];
            snprintf(hello_buf, sizeof(hello_buf),
                "{\"type\":\"hello\",\"device\":\"%s\",\"athlete\":\"%s\",\"fw\":\"%s\"}",
                g_identity.device_id, g_identity.athlete_name, FW_VERSION_STR);
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

static void coach_handle_command(const char *data, int len)
{
    char buf[128] = {0};
    memcpy(buf, data, len < 127 ? len : 127);

    if (strstr(buf, "\"lap\"")) {
        g_coach_lap_cmd = true;
        ESP_LOGI(COACH_TAG, "LAP dal coach");
    } else if (strstr(buf, "\"start\"")) {
        g_coach_start_cmd = true;
        ESP_LOGI(COACH_TAG, "START dal coach");
    } else if (strstr(buf, "\"stop\"")) {
        g_coach_stop_cmd = true;
        ESP_LOGI(COACH_TAG, "STOP dal coach");
    } else if (strstr(buf, "\"ota\"")) {
        // Formato: {"type":"cmd","action":"ota","url":"http://..."}  
        const char *p = strstr(buf, "\"url\":\"");
        if (p) {
            p += 7;
            const char *end = strchr(p, '"');
            if (end && end > p) {
                char url[OTA_URL_MAXLEN] = {0};
                size_t ulen = (size_t)(end - p);
                if (ulen < OTA_URL_MAXLEN) {
                    memcpy(url, p, ulen);
                    url[ulen] = '\0';
                    ESP_LOGI(COACH_TAG, "OTA richiesto dal coach: %s", url);
                    ota_start(url);
                }
            }
        }
    }
}

esp_err_t coach_send_frame(const aerodrag_sensors_t *s,
                            const aerodrag_physics_t *p,
                            uint8_t battery_pct,
                            bool    lap_event)
{
    if (!g_ws_ready || !g_ws || !p->valid) return ESP_ERR_INVALID_STATE;

    int64_t ts_ms = esp_timer_get_time() / 1000LL;

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

esp_err_t coach_config_load(coach_config_t *cfg)
{
    nvs_handle_t h;
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

    esp_err_t netif_ret = esp_netif_init();
    if (netif_ret != ESP_OK && netif_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(COACH_TAG, "esp_netif_init failed: %d", netif_ret);
        return netif_ret;
    }

    esp_err_t loop_ret = esp_event_loop_create_default();
    if (loop_ret != ESP_OK && loop_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(COACH_TAG, "event loop create failed: %d", loop_ret);
        return loop_ret;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));

    if (!esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"))
        esp_netif_create_default_wifi_sta();
    esp_event_handler_instance_register(WIFI_EVENT,  ESP_EVENT_ANY_ID,    _wifi_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT,    IP_EVENT_STA_GOT_IP, _wifi_handler, NULL, NULL);

    wifi_config_t wc = {0};
    strlcpy((char*)wc.sta.ssid,     g_cfg.ssid, sizeof(wc.sta.ssid));
    strlcpy((char*)wc.sta.password, g_cfg.pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    g_wifi_hw_started = true;

    EventBits_t bits = xEventGroupWaitBits(g_wifi_eg,
        WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(COACH_TAG, "Timeout WiFi — fallback BLE");
        esp_wifi_stop();
        return ESP_ERR_TIMEOUT;
    }

    char url[80];
    snprintf(url, sizeof(url), "ws://%s:%d/device", g_cfg.host, g_cfg.port);
    esp_websocket_client_config_t wsc = {
        .uri                  = url,
        .reconnect_timeout_ms = 3000,
        .network_timeout_ms   = 5000,
    };
    g_ws = esp_websocket_client_init(&wsc);
    esp_websocket_register_events(g_ws, WEBSOCKET_EVENT_ANY, _ws_handler, NULL);
    return esp_websocket_client_start(g_ws);
}

void coach_cycle_mode(void)
{
    g_cfg.mode = (coach_mode_t)((g_cfg.mode + 1) % 3);
    coach_config_save(&g_cfg);
    const char *labels[] = {"OFF", "COACH DIRECT", "COACH CO-OP"};
    ESP_LOGI(COACH_TAG, "→ %s", labels[g_cfg.mode]);
    g_coach_mode_display = (uint8_t)g_cfg.mode + 1;
}

static void _coach_connect_task(void *arg)
{
    if (g_wifi_hw_started) {
        if (!g_ws) {
            char url[80];
            snprintf(url, sizeof(url), "ws://%s:%d/coach", g_cfg.host, g_cfg.port);
            esp_websocket_client_config_t wsc = {
                .uri                  = url,
                .reconnect_timeout_ms = 3000,
                .network_timeout_ms   = 5000,
            };
            g_ws = esp_websocket_client_init(&wsc);
            esp_websocket_register_events(g_ws, WEBSOCKET_EVENT_ANY, _ws_handler, NULL);
            esp_websocket_client_start(g_ws);
        }
        vTaskDelete(NULL);
        return;
    }

    if (!g_wifi_eg) g_wifi_eg = xEventGroupCreate();

    esp_err_t r = esp_netif_init();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(COACH_TAG, "netif_init: %d", r);
        vTaskDelete(NULL);
        return;
    }

    r = esp_event_loop_create_default();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(COACH_TAG, "event_loop: %d", r);
        vTaskDelete(NULL);
        return;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wifi_init) != ESP_OK) {
        ESP_LOGE(COACH_TAG, "esp_wifi_init failed");
        vTaskDelete(NULL);
        return;
    }

    if (!esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"))
        esp_netif_create_default_wifi_sta();
    esp_event_handler_instance_register(WIFI_EVENT,  ESP_EVENT_ANY_ID,    _wifi_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT,    IP_EVENT_STA_GOT_IP, _wifi_handler, NULL, NULL);

    wifi_config_t wc = {0};
    strlcpy((char*)wc.sta.ssid,     g_cfg.ssid, sizeof(wc.sta.ssid));
    strlcpy((char*)wc.sta.password, g_cfg.pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
    g_wifi_hw_started = true;

    EventBits_t bits = xEventGroupWaitBits(g_wifi_eg,
        WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(COACH_TAG, "Timeout WiFi — fallback BLE");
        esp_wifi_stop();
        g_wifi_hw_started = false;
        vTaskDelete(NULL);
        return;
    }

    char url[80];
    snprintf(url, sizeof(url), "ws://%s:%d/device", g_cfg.host, g_cfg.port);
    esp_websocket_client_config_t wsc = {
        .uri                  = url,
        .reconnect_timeout_ms = 3000,
        .network_timeout_ms   = 5000,
    };
    g_ws = esp_websocket_client_init(&wsc);
    esp_websocket_register_events(g_ws, WEBSOCKET_EVENT_ANY, _ws_handler, NULL);
    esp_websocket_client_start(g_ws);
    vTaskDelete(NULL);
}

esp_err_t coach_apply_mode(void)
{
    coach_config_load(&g_cfg);
    if (g_cfg.mode == COACH_MODE_OFF) {
        if (g_ws) {
            esp_websocket_client_stop(g_ws);
            esp_websocket_client_destroy(g_ws);
            g_ws = NULL;
        }
        g_ws_ready = false;
        return ESP_OK;
    }
    xTaskCreate(_coach_connect_task, "coach_conn", 4096, NULL, 3, NULL);
    return ESP_OK;
}

uint16_t         coach_get_lap(void)    { return g_current_lap; }
bool             coach_is_ready(void)   { return g_ws_ready; }
coach_mode_t     coach_get_mode(void)   { return g_cfg.mode; }
const char      *coach_get_ssid(void)   { return g_cfg.ssid; }
