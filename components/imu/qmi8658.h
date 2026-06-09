#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include <math.h>

// ─── QMI8658 register map ─────────────────────────────────────────────────────
#define QMI8658_REG_WHO_AM_I    0x00   // should read 0x05
#define QMI8658_REG_CTRL1       0x02   // serial interface config
#define QMI8658_REG_CTRL2       0x03   // accelerometer
#define QMI8658_REG_CTRL3       0x04   // gyroscope
#define QMI8658_REG_CTRL7       0x08   // enable sensors
#define QMI8658_REG_STATUSINT   0x2D
#define QMI8658_REG_AX_L        0x35
#define QMI8658_REG_TEMP_L      0x33

// Accelerometer: ±8g, 125Hz ODR
#define QMI8658_ACC_RANGE_8G    0x03
#define QMI8658_ACC_ODR_125HZ   0x03
// Gyroscope: ±512dps, 125Hz ODR
#define QMI8658_GYR_RANGE_512   0x03
#define QMI8658_GYR_ODR_125HZ   0x03

#define QMI8658_ACC_SCALE       (8.0f * 9.80665f / 32768.0f)   // m/s² per LSB
#define QMI8658_GYR_SCALE       (512.0f / 32768.0f)             // dps per LSB

typedef struct {
    i2c_port_t port;
    uint8_t    addr;
    // Euler angles (simple complementary filter)
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    float temp_c;
    // Filter state
    float pitch_acc;
    float roll_acc;
    int64_t last_us;
} qmi8658_t;

static esp_err_t qmi_write(qmi8658_t *d, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(d->port, d->addr, buf, 2, pdMS_TO_TICKS(20));
}

static esp_err_t qmi_read(qmi8658_t *d, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(d->port, d->addr, &reg, 1, buf, len, pdMS_TO_TICKS(20));
}

esp_err_t qmi8658_init(qmi8658_t *dev, i2c_port_t port, uint8_t addr)
{
    dev->port  = port;
    dev->addr  = addr;
    dev->pitch_deg = 0.0f;
    dev->roll_deg  = 0.0f;
    dev->last_us   = 0;

    // WHO_AM_I check
    uint8_t id = 0;
    esp_err_t ret = qmi_read(dev, QMI8658_REG_WHO_AM_I, &id, 1);
    if (ret != ESP_OK || id != 0x05) return ESP_ERR_NOT_FOUND;
    // Disable sensors for config
    qmi_write(dev, QMI8658_REG_CTRL7, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Acc: ±8g, 125Hz, low-pass filter enabled
    qmi_write(dev, QMI8658_REG_CTRL2,
          (QMI8658_ACC_RANGE_8G << 4) | QMI8658_ACC_ODR_125HZ);
    // Gyro: ±512dps, 125Hz
    qmi_write(dev, QMI8658_REG_CTRL3,
          (QMI8658_GYR_RANGE_512 << 4) | QMI8658_GYR_ODR_125HZ);
    // Enable both sensors
    qmi_write(dev, QMI8658_REG_CTRL7, 0x03);
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

// Complementary filter: α = 0.96 (gyro weight), (1-α) = acc weight
#define CF_ALPHA 0.96f

esp_err_t qmi8658_read(qmi8658_t *dev)
{
    uint8_t buf[12];  // 6 bytes acc + 6 bytes gyro
    esp_err_t ret = qmi_read(dev, QMI8658_REG_AX_L, buf, 12);
    if (ret != ESP_OK) return ret;

    int16_t ax_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az_raw = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t gx_raw = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t gy_raw = (int16_t)((buf[9] << 8) | buf[8]);

    float ax = ax_raw * QMI8658_ACC_SCALE;
    float ay = ay_raw * QMI8658_ACC_SCALE;
    float az = az_raw * QMI8658_ACC_SCALE;
    float gx = gx_raw * QMI8658_GYR_SCALE;
    float gy = gy_raw * QMI8658_GYR_SCALE;

    // Accelerometer pitch/roll (degrees)
    float pitch_acc = atan2f(ax, sqrtf(ay*ay + az*az)) * (180.0f / 3.14159f);
    float roll_acc  = atan2f(ay, sqrtf(ax*ax + az*az)) * (180.0f / 3.14159f);

    // Delta time
    int64_t now_us = esp_timer_get_time();
    float dt = (dev->last_us > 0)
               ? (float)(now_us - dev->last_us) * 1e-6f
               : 0.008f;  // default 8ms
    dev->last_us = now_us;
    if (dt > 0.1f) dt = 0.008f;  // clamp on first call / gap

    // Complementary filter
    dev->pitch_deg = CF_ALPHA * (dev->pitch_deg + gx * dt)
                   + (1.0f - CF_ALPHA) * pitch_acc;
    dev->roll_deg  = CF_ALPHA * (dev->roll_deg  + gy * dt)
                   + (1.0f - CF_ALPHA) * roll_acc;

    // Temperature
    uint8_t tbuf[2];
    if (qmi_read(dev, QMI8658_REG_TEMP_L, tbuf, 2) == ESP_OK) {
        int16_t t_raw = (int16_t)((tbuf[1] << 8) | tbuf[0]);
        dev->temp_c = (float)t_raw / 256.0f;
    }

    return ESP_OK;
}
