#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "aerodrag_types.h"

// ─── SDP810-500Pa register map ────────────────────────────────────────────────
// Continuous measurement, mass flow, no averaging
#define SDP810_CMD_START_CONT_MASS  0x3603  // big-endian 2-byte command
#define SDP810_CMD_STOP             0x3FF9
#define SDP810_CMD_SOFT_RESET       0x0006
#define SDP810_CMD_READ_PRODUCT_ID1 0x367C
#define SDP810_CMD_READ_PRODUCT_ID2 0xE102

// Scale factors for SDP810-500Pa (from datasheet table 3)
#define SDP810_PRESSURE_SCALE       60.0f    // LSB/Pa
#define SDP810_TEMP_SCALE           200.0f   // LSB/°C

typedef struct {
    i2c_port_t port;
    uint8_t    addr;
    float      pressure_pa;
    float      temp_c;
    bool       started;
} sdp810_t;

// ─── CRC-8 check (polynomial 0x31, init 0xFF) ────────────────────────────────
static uint8_t sdp810_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

static esp_err_t sdp810_write_cmd(sdp810_t *dev, uint16_t cmd)
{
    uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_write_to_device(dev->port, dev->addr, buf, 2,
                                      pdMS_TO_TICKS(50));
}

esp_err_t sdp810_init(sdp810_t *dev, i2c_port_t port, uint8_t addr)
{
    dev->port    = port;
    dev->addr    = addr;
    dev->started = false;
    dev->pressure_pa = 0.0f;
    dev->temp_c      = 20.0f;

    // Soft reset
    uint8_t reset_cmd = 0x06;
    i2c_master_write_to_device(dev->port, 0x00, &reset_cmd, 1, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(25));

    // Start continuous measurement
    esp_err_t ret = sdp810_write_cmd(dev, SDP810_CMD_START_CONT_MASS);
    if (ret != ESP_OK) return ret;
    dev->started = true;
    vTaskDelay(pdMS_TO_TICKS(50));

    // ← NUOVO: verifica reale — leggi 3 byte, se il sensore non c'è fallisce qui
    uint8_t probe[3];
    ret = i2c_master_read_from_device(dev->port, dev->addr, probe, 3, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        dev->started = false;
        ESP_LOGE("SDP810", "Sensor not responding at 0x%02X (err %d)", dev->addr, ret);
        return ret;
    }

    ESP_LOGI("SDP810", "Init OK at 0x%02X — first bytes: %02X %02X %02X",
             dev->addr, probe[0], probe[1], probe[2]);
    return ESP_OK;
}

// Read one measurement (call at ≤ 100 Hz)
esp_err_t sdp810_read(sdp810_t *dev)
{
    if (!dev->started) return ESP_ERR_INVALID_STATE;

    // 9 bytes: dp_msb, dp_lsb, dp_crc, temp_msb, temp_lsb, temp_crc,
    //          scale_msb, scale_lsb, scale_crc
    uint8_t buf[9];
    esp_err_t ret = i2c_master_read_from_device(
        dev->port, dev->addr, buf, sizeof(buf), pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return ret;

    // CRC check differential pressure word
    if (sdp810_crc8(buf, 2) != buf[2]) return ESP_ERR_INVALID_CRC;
    if (sdp810_crc8(buf + 3, 2) != buf[5]) return ESP_ERR_INVALID_CRC;

    int16_t dp_raw   = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t temp_raw = (int16_t)((buf[3] << 8) | buf[4]);

    dev->pressure_pa = (float)dp_raw   / SDP810_PRESSURE_SCALE;
    dev->temp_c      = (float)temp_raw / SDP810_TEMP_SCALE;

    // Physical clamp (0-500 Pa range of sensor)
    if (dev->pressure_pa < 0.0f)   dev->pressure_pa = 0.0f;
    if (dev->pressure_pa > 500.0f) dev->pressure_pa = 500.0f;

    return ESP_OK;
}

esp_err_t sdp810_stop(sdp810_t *dev)
{
    dev->started = false;
    return sdp810_write_cmd(dev, SDP810_CMD_STOP);
}
