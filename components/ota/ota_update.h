#pragma once
/**
 * AeroDrag OTA Firmware Update
 * ─────────────────────────────────────────────────────────────────────────────
 * Scarica e installa un nuovo firmware via HTTP (Pi locale o server HTTP).
 * Schema A/B in partitions.csv: due slot ota_0/ota_1 da 4 MB — ogni OTA
 * scrive nello slot non in esecuzione, quindi gli aggiornamenti sono
 * ripetibili indefinitamente.
 *
 * Trigger:
 *   WiFi: coach invia {"type":"cmd","action":"ota","url":"http://192.168.8.1:8080/fw.bin"}
 *   BLE:  scrittura URL in CHR_OTA_URL (0x07aa) avvia il download
 *
 * Rollback: se il nuovo firmware non chiama ota_mark_valid() entro l'avvio,
 * il bootloader ripristina automaticamente il firmware precedente.
 */

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define OTA_TAG         "ota"
#define OTA_BUF_SIZE    4096
#define OTA_URL_MAXLEN   200

typedef enum {
    OTA_IDLE    = 0,
    OTA_RUNNING = 1,
    OTA_SUCCESS = 2,
    OTA_FAILED  = 3,
} ota_status_t;

static volatile ota_status_t g_ota_status   = OTA_IDLE;
static volatile uint8_t      g_ota_progress = 0;   // 0-100 %
static char                  g_ota_pending_url[OTA_URL_MAXLEN] = {0};

static void ota_task(void *arg)
{
    const char *url = (const char *)arg;
    ESP_LOGI(OTA_TAG, "Download da: %s", url);

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        ESP_LOGE(OTA_TAG, "Nessuna partizione OTA disponibile");
        g_ota_status = OTA_FAILED;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(OTA_TAG, "Partizione target: %s @ 0x%08lx",
             update_part->label, (unsigned long)update_part->address);

    esp_http_client_config_t http_cfg = {
        .url               = url,
        .timeout_ms        = 15000,
        .buffer_size       = OTA_BUF_SIZE,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(OTA_TAG, "HTTP client init failed");
        g_ota_status = OTA_FAILED;
        vTaskDelete(NULL);
        return;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "HTTP open failed: %d", err);
        g_ota_status = OTA_FAILED;
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    int content_len = esp_http_client_fetch_headers(client);
    int http_status = esp_http_client_get_status_code(client);
    if (http_status != 200) {
        ESP_LOGE(OTA_TAG, "HTTP %d — firmware non trovato", http_status);
        g_ota_status = OTA_FAILED;
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(OTA_TAG, "HTTP 200 — dimensione: %d byte", content_len);

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "ota_begin failed: %d", err);
        g_ota_status = OTA_FAILED;
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    static uint8_t ota_buf[OTA_BUF_SIZE];  // static: BSS, non stack del task
    int total = 0;
    int chunk;
    while ((chunk = esp_http_client_read(client, (char *)ota_buf, OTA_BUF_SIZE)) > 0) {
        err = esp_ota_write(ota_handle, ota_buf, (size_t)chunk);
        if (err != ESP_OK) {
            ESP_LOGE(OTA_TAG, "ota_write failed: %d", err);
            esp_ota_abort(ota_handle);
            g_ota_status = OTA_FAILED;
            esp_http_client_cleanup(client);
            vTaskDelete(NULL);
            return;
        }
        total += chunk;
        g_ota_progress = (content_len > 0)
            ? (uint8_t)((total * 100) / content_len)
            : 0;
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "ota_end failed: %d (immagine corrotta?)", err);
        g_ota_status = OTA_FAILED;
        vTaskDelete(NULL);
        return;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "set_boot_partition failed: %d", err);
        g_ota_status = OTA_FAILED;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(OTA_TAG, "OTA completato — %d byte scritti. Riavvio tra 2s...", total);
    g_ota_status   = OTA_SUCCESS;
    g_ota_progress = 100;
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    vTaskDelete(NULL);
}

/**
 * ota_start() — avvia download e installazione in un task background (prio 3).
 * Thread-safe: ignora la chiamata se un OTA è già in corso.
 */
void ota_start(const char *url)
{
    if (g_ota_status == OTA_RUNNING) {
        ESP_LOGW(OTA_TAG, "OTA già in corso — ignorato");
        return;
    }
    if (!url || strlen(url) == 0 || strlen(url) >= OTA_URL_MAXLEN) {
        ESP_LOGE(OTA_TAG, "URL non valido");
        g_ota_status = OTA_FAILED;
        return;
    }
    strlcpy(g_ota_pending_url, url, OTA_URL_MAXLEN);
    g_ota_status   = OTA_RUNNING;
    g_ota_progress = 0;
    xTaskCreate(ota_task, "ota", 8192, g_ota_pending_url, 3, NULL);
    ESP_LOGI(OTA_TAG, "Task OTA avviato → %s", g_ota_pending_url);
}

/**
 * ota_mark_valid() — chiama dopo l'init completo per confermare il firmware.
 * Se questa funzione non viene raggiunta (crash durante boot), il bootloader
 * ripristina il firmware precedente al prossimo riavvio.
 */
void ota_mark_valid(void)
{
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK)
        ESP_LOGI(OTA_TAG, "Firmware %s confermato valido (rollback annullato)",
                 "1.0.0");  // stringa statica per evitare include circolare
}

ota_status_t ota_get_status(void)   { return g_ota_status; }
uint8_t      ota_get_progress(void) { return g_ota_progress; }
