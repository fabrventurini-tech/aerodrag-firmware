#pragma once
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "board_pins.h"
#include "aerodrag_types.h"
#include "qrcodegen.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// ─── ST7789T3 init sequence (Waveshare ESP32-S3-Touch-LCD-2.8) ───────────────
typedef struct { uint8_t cmd; uint8_t data[16]; uint8_t len; uint8_t delay_ms; } lcd_cmd_t;

static const lcd_cmd_t ST7789_INIT[] = {
    { 0x01, {0}, 0, 150 },   // software reset
    { 0x11, {0}, 0, 120 },   // sleep out
    { 0x3A, {0x55}, 1, 10 }, // pixel format RGB565
    { 0x36, {0x00}, 1, 0  }, // MADCTL: portrait 240x320
    { 0x2A, {0x00,0x00,0x00,0xEF}, 4, 0 }, // column address 0-239
    { 0x2B, {0x00,0x00,0x01,0x3F}, 4, 0 }, // row address 0-319
    { 0xB2, {0x0C,0x0C,0x00,0x33,0x33}, 5, 0 }, // porch control
    { 0xB7, {0x35}, 1, 0  }, // gate control
    { 0xBB, {0x19}, 1, 0  }, // VCOMS
    { 0xC0, {0x2C}, 1, 0  }, // LCM control
    { 0xC2, {0x01}, 1, 0  }, // VDV/VRH enable
    { 0xC3, {0x12}, 1, 0  }, // VRH set
    { 0xC4, {0x20}, 1, 0  }, // VDV set
    { 0xC6, {0x0F}, 1, 0  }, // FR control
    { 0xD0, {0xA4,0xA1}, 2, 0 }, // power control 1
    { 0xE0, {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23}, 14, 0 },
    { 0xE1, {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23}, 14, 0 },
    { 0x21, {0}, 0, 0  },   // display inversion on
    { 0x29, {0}, 0, 10 },   // display on
};

// ─── Framebuffer — 240×320 × 2 bytes = 153.6 KB ──────────────────────────────
#define FB_W  240
#define FB_H  320
#define FB_BYTES (FB_W * FB_H * 2)

static spi_device_handle_t g_spi  = NULL;
static uint16_t            *g_fb  = NULL;
static bool                 g_bl_on = false;

// ─── Colour helpers ───────────────────────────────────────────────────────────
static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
}
static inline uint16_t swap16(uint16_t v) { return (v << 8) | (v >> 8); }

#define COL_BG      rgb(9,  13, 20)
#define COL_SURFACE rgb(17, 25, 38)
#define COL_TEAL    rgb(0,  212,170)
#define COL_AMBER   rgb(245,166, 35)
#define COL_RED     rgb(242, 69, 96)
#define COL_MUTED   rgb(77, 96,128)
#define COL_TEXT    rgb(221,232,245)

// ─── SPI helpers ──────────────────────────────────────────────────────────────
static void lcd_cmd(uint8_t c)
{
    gpio_set_level(PIN_LCD_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &c };
    spi_device_polling_transmit(g_spi, &t);
}

static void lcd_data(const uint8_t *d, size_t len)
{
    if (!len) return;
    gpio_set_level(PIN_LCD_DC, 1);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = d };
    spi_device_polling_transmit(g_spi, &t);
}

static void lcd_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];
    lcd_cmd(0x2A);
    d[0]=x0>>8; d[1]=x0; d[2]=x1>>8; d[3]=x1; lcd_data(d,4);
    lcd_cmd(0x2B);
    d[0]=y0>>8; d[1]=y0; d[2]=y1>>8; d[3]=y1; lcd_data(d,4);
    lcd_cmd(0x2C);
}

// ─── Backlight (PWM via LEDC) ─────────────────────────────────────────────────
static void bl_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);
    ledc_channel_config_t c = {
        .gpio_num   = PIN_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 255,
        .hpoint     = 0,
    };
    ledc_channel_config(&c);
}

void display_set_brightness(uint8_t pct)  // 0-100
{
    uint32_t duty = (uint32_t)pct * 255 / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ─── Init ─────────────────────────────────────────────────────────────────────
esp_err_t display_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = PIN_LCD_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = PIN_LCD_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = FB_W * 40 * 2,
    };
    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = LCD_SPI_CLK_HZ,
        .mode           = 0,
        .spics_io_num   = PIN_LCD_CS,
        .queue_size     = 7,
    };
    ret = spi_bus_add_device(LCD_SPI_HOST, &dev, &g_spi);
    if (ret != ESP_OK) return ret;

    gpio_set_direction(PIN_LCD_DC,  GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LCD_RST, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_LCD_RST, 0); vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(PIN_LCD_RST, 1); vTaskDelay(pdMS_TO_TICKS(120));

    for (size_t i = 0; i < sizeof(ST7789_INIT)/sizeof(ST7789_INIT[0]); i++) {
        lcd_cmd(ST7789_INIT[i].cmd);
        if (ST7789_INIT[i].len)
            lcd_data(ST7789_INIT[i].data, ST7789_INIT[i].len);
        if (ST7789_INIT[i].delay_ms)
            vTaskDelay(pdMS_TO_TICKS(ST7789_INIT[i].delay_ms));
    }

    bl_init();
    display_set_brightness(90);
    g_bl_on = true;

    g_fb = (uint16_t *)heap_caps_malloc(FB_BYTES, MALLOC_CAP_SPIRAM);
    if (!g_fb)
        g_fb = (uint16_t *)heap_caps_malloc(FB_BYTES, MALLOC_CAP_INTERNAL);
    if (!g_fb) return ESP_ERR_NO_MEM;

    memset(g_fb, 0, FB_BYTES);
    return ESP_OK;
}

// ─── Flush framebuffer in 40-line strips ─────────────────────────────────────
static void display_flush(void)
{
    const int STRIP = 40;
    static uint16_t strip_buf[FB_W * 40];

    lcd_window(0, 0, FB_W-1, FB_H-1);
    gpio_set_level(PIN_LCD_DC, 1);

    for (int y = 0; y < FB_H; y += STRIP) {
        int h = (y + STRIP > FB_H) ? (FB_H - y) : STRIP;
        for (int i = 0; i < h * FB_W; i++)
            strip_buf[i] = swap16(g_fb[y * FB_W + i]);
        spi_transaction_t t = {
            .length    = h * FB_W * 16,
            .tx_buffer = strip_buf,
        };
        spi_device_polling_transmit(g_spi, &t);
    }
}

// ─── Software renderer ────────────────────────────────────────────────────────
#define PX(x,y) g_fb[(y)*FB_W+(x)]

static void fb_fill(uint16_t col)
{
    for (int i = 0; i < FB_W * FB_H; i++) g_fb[i] = col;
}

// Bring-up self-test (diagnostico): pilota la retroilluminazione nel modo più
// diretto possibile — IO5 (LCD_BL) come GPIO digitale ALTO, bypassando il LEDC —
// poi riempie lo schermo di ROSSO. Esiti:
//   ROSSO            -> tutto OK (era il PWM/LEDC);
//   illuminato/bianco -> backlight OK, problema pannello/SPI;
//   nero totale       -> IO5 non accende il backlight (HW/pin/alimentazione VLED).
void display_selftest(void)
{
    gpio_reset_pin(PIN_LCD_BL);
    gpio_set_direction(PIN_LCD_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LCD_BL, 1);   // IO5 alto = backlight ON (active-high da schematic)

    if (!g_fb) return;
    fb_fill(COL_RED);
    display_flush();
}

static void fb_circle(int cx, int cy, int r, uint16_t col)
{
    int r2 = r * r;
    for (int y = cy-r; y <= cy+r; y++)
    for (int x = cx-r; x <= cx+r; x++) {
        if (x<0||x>=FB_W||y<0||y>=FB_H) continue;
        if ((x-cx)*(x-cx)+(y-cy)*(y-cy) <= r2) PX(x,y) = col;
    }
}

static void fb_arc(int cx, int cy, int r, int w,
                   float start_deg, float end_deg, uint16_t col)
{
    if (end_deg <= start_deg) return;
    const float STEP = 0.4f;
    for (float a = start_deg; a <= end_deg; a += STEP) {
        float rad = a * 3.14159265f / 180.0f;
        float ca = cosf(rad), sa = sinf(rad);
        for (int ri = r-w; ri <= r; ri++) {
            int x = cx + (int)(ca * ri);
            int y = cy - (int)(sa * ri);
            if (x>=0 && x<FB_W && y>=0 && y<FB_H) PX(x,y) = col;
        }
    }
}

static void fb_hline(int x0, int x1, int y, uint16_t col)
{
    if (y<0||y>=FB_H) return;
    if (x0>x1) { int t=x0; x0=x1; x1=t; }
    for (int x=x0; x<=x1; x++) if (x>=0&&x<FB_W) PX(x,y)=col;
}

// ─── 5×7 pixel font — full ASCII printable (0x20–0x7E) ───────────────────────
static const uint8_t FONT5X7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x40,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x10,0x08,0x08,0x10,0x08}, // '~'
};

static void fb_char(int x, int y, char c, uint16_t col, int scale)
{
    if (c < 0x20 || c > 0x7E) return;
    int idx = c - 0x20;
    for (int row = 0; row < 7; row++)
    for (int col_b = 0; col_b < 5; col_b++) {
        if (!((FONT5X7[idx][col_b] >> row) & 1)) continue;
        for (int sy = 0; sy < scale; sy++)
        for (int sx = 0; sx < scale; sx++) {
            int px = x + col_b*scale + sx, py = y + row*scale + sy;
            if (px>=0 && px<FB_W && py>=0 && py<FB_H) PX(px,py) = col;
        }
    }
}

static void fb_str(int x, int y, const char *s, uint16_t col, int scale)
{
    while (*s) { fb_char(x, y, *s++, col, scale); x += (5+1)*scale; }
}

// ─── QR pairing state ────────────────────────────────────────────────────────
#define QR_BUF_LEN  qrcodegen_BUFFER_LEN_FOR_VERSION(5)

static char    g_pairing_id[32]      = {0};
static uint8_t g_qr_cache[QR_BUF_LEN] = {0};
static bool    g_qr_valid            = false;

void display_set_pairing_id(const char *id)
{
    strlcpy(g_pairing_id, id, sizeof(g_pairing_id));
    uint8_t tmp[QR_BUF_LEN];
    char qr_text[64];
    snprintf(qr_text, sizeof(qr_text), "AERODRAG://PAIR/%s", id);
    g_qr_valid = qrcodegen_encodeText(qr_text, tmp, g_qr_cache,
                                       qrcodegen_Ecc_LOW, 1, 5,
                                       qrcodegen_Mask_AUTO, true);
}

// ─── Session best + rider config ─────────────────────────────────────────────
static float    g_session_best_cda = 9999.0f;
static uint32_t g_best_flash_end   = 0;
static float    g_rider_mass_kg    = 75.0f;
static uint16_t g_rider_ftp_w      = 250;

void display_set_rider_config(float mass_kg, uint16_t ftp_w)
{
    g_rider_mass_kg = (mass_kg > 0.0f) ? mass_kg : 75.0f;
    g_rider_ftp_w   = (ftp_w  > 0)    ? ftp_w   : 250;
}

static const char *cda_grade(float cda)
{
    if (cda < 0.200f) return "A+";
    if (cda < 0.220f) return "A";
    if (cda < 0.250f) return "B";
    if (cda < 0.280f) return "C";
    return "D";
}

static uint16_t power_zone_col(uint16_t w, uint16_t ftp)
{
    if (!ftp || !w) return COL_MUTED;
    uint32_t pct = (uint32_t)w * 100u / ftp;
    if (pct < 55)  return COL_MUTED;
    if (pct < 75)  return COL_TEAL;
    if (pct < 90)  return rgb(80, 200, 120);
    if (pct < 105) return COL_AMBER;
    if (pct < 120) return rgb(255, 120, 20);
    return COL_RED;
}

static const char *power_zone_name(uint16_t w, uint16_t ftp)
{
    if (!ftp || !w) return "";
    uint32_t pct = (uint32_t)w * 100u / ftp;
    if (pct < 55)  return "Z1 Recovery";
    if (pct < 75)  return "Z2 Endurance";
    if (pct < 90)  return "Z3 Tempo";
    if (pct < 105) return "Z4 Threshold";
    if (pct < 120) return "Z5 VO2Max";
    return "Z6 Anaerobic";
}

// ─── Toast notification ───────────────────────────────────────────────────────
static char              g_toast_msg[40]  = {0};
static volatile uint32_t g_toast_end_ms   = 0;

// ─── Connection + timer state ─────────────────────────────────────────────────
static bool     g_ble_connected      = false;
static bool     g_wifi_ready         = false;
static uint32_t g_session_elapsed_s  = 0;
static uint32_t g_lap_elapsed_s      = 0;
static uint16_t g_lap_num_display    = 1;

// ─── Battery icon (20×10 body + 3 px cap, 4 fill bars) ───────────────────────
static void fb_battery(int x, int y, uint8_t pct, uint16_t col)
{
    fb_hline(x, x+19, y,   col);
    fb_hline(x, x+19, y+9, col);
    for (int dy = y; dy <= y+9; dy++) if (dy>=0&&dy<FB_H) { PX(x,dy)=col; PX(x+19,dy)=col; }
    fb_hline(x+20, x+22, y+3, col);
    fb_hline(x+20, x+22, y+6, col);
    for (int dy = y+3; dy <= y+6; dy++) if (dy>=0&&dy<FB_H) PX(x+22,dy) = col;
    int bars = pct * 4 / 100;
    for (int b = 0; b < 4; b++) {
        int bx = x + 2 + b * 4;
        uint16_t fc = (b < bars) ? col : COL_SURFACE;
        for (int fy = y+2; fy <= y+7; fy++)
            for (int fx = bx; fx < bx+3; fx++)
                if (fx>=0&&fx<FB_W&&fy>=0&&fy<FB_H) PX(fx,fy) = fc;
    }
}

// ─── Status dots (4×, bottom-right): WiFi BLE ANT+ Pitot ─────────────────────
static void render_status_dots(const aerodrag_sensors_t *s)
{
    struct { int x; uint16_t col; char lbl; } dots[4] = {
        { FB_W - 35, g_wifi_ready    ? COL_TEAL : COL_MUTED, 'W' },
        { FB_W - 26, g_ble_connected ? COL_TEAL : COL_MUTED, 'B' },
        { FB_W - 17, s->ant_valid    ? COL_TEAL : COL_MUTED, 'A' },
        { FB_W -  8, s->pitot_valid  ? COL_TEAL : COL_MUTED, 'P' },
    };
    for (int i = 0; i < 4; i++) {
        fb_char(dots[i].x - 2, FB_H - 19, dots[i].lbl, COL_MUTED, 1);
        fb_circle(dots[i].x,   FB_H -  8, 3, dots[i].col);
    }
}

// ─── MM:SS helper ─────────────────────────────────────────────────────────────
static void fb_time(int x, int y, uint32_t secs, uint16_t col, int scale)
{
    char buf[6];
    uint32_t mm = secs / 60;
    if (mm > 99) mm = 99;
    snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)mm, (unsigned long)(secs % 60));
    fb_str(x, y, buf, col, scale);
}

// ─── Screen state ─────────────────────────────────────────────────────────────
typedef enum { SCR_PAIRING=0, SCR_CDA, SCR_TIMER, SCR_SPEED, SCR_POWER, SCR_STATUS, SCR_COUNT } screen_t;
static screen_t g_screen = SCR_PAIRING;

screen_t display_get_screen(void) { return g_screen; }

// ─── Render ───────────────────────────────────────────────────────────────────
void display_render(const aerodrag_sensors_t *s, const aerodrag_physics_t *p)
{
    if (!g_fb) return;
    const int CX = FB_W/2, CY = FB_H/2;   // 120, 160

    fb_fill(COL_BG);

switch (g_screen) {

    // ── Pairing ───────────────────────────────────────────────────────────────
    case SCR_PAIRING: {
        if (!g_qr_valid) {
            int w = (int)strlen(g_pairing_id) * 6;
            fb_str((FB_W - w) / 2, CY - 4, g_pairing_id, COL_TEAL, 1);
            break;
        }
        int sz  = qrcodegen_getSize(g_qr_cache);
        const int SCALE = 7;
        const int PAD   = 10;
        int qr_px  = sz * SCALE;
        int rect_w = qr_px + 2 * PAD;
        int rect_h = qr_px + 2 * PAD;
        int rx = (FB_W - rect_w) / 2;
        int ry = 20;
        int ox = rx + PAD;
        int oy = ry + PAD;

        uint16_t WHITE = rgb(255,255,255);
        uint16_t BLACK = rgb(0,0,0);
        for (int y = ry; y < ry + rect_h; y++)
            for (int x = rx; x < rx + rect_w; x++)
                if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y) = WHITE;

        for (int qy = 0; qy < sz; qy++)
        for (int qx = 0; qx < sz; qx++) {
            uint16_t c = qrcodegen_getModule(g_qr_cache, qx, qy) ? BLACK : WHITE;
            for (int sy = 0; sy < SCALE; sy++)
            for (int sx = 0; sx < SCALE; sx++) {
                int px = ox + qx*SCALE + sx;
                int py = oy + qy*SCALE + sy;
                if (px>=0&&px<FB_W&&py>=0&&py<FB_H) PX(px,py) = c;
            }
        }

        {
            int id_len = (int)strlen(g_pairing_id);
            int id_w   = id_len * 6;
            int id_x   = (FB_W - id_w) / 2;
            int id_y   = ry + rect_h + 8;
            fb_str(id_x, id_y, g_pairing_id, COL_MUTED, 1);
        }

        fb_circle(CX, ry + rect_h + 30, 4, COL_RED);
        break;
    }

    // ── CdA / Performance ─────────────────────────────────────────────────────
    case SCR_CDA: {
        bool  valid = (p && p->valid);
        float cda   = valid ? p->CdA : 0.0f;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
        if (valid && cda > 0.01f && cda < g_session_best_cda) {
            g_session_best_cda = cda;
            g_best_flash_end   = now_ms + 2000;
        }

        float pct = valid ? (cda - 0.18f) / 0.20f : 0.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 1.0f) pct = 1.0f;

        uint16_t arc_col = (pct < 0.33f) ? COL_TEAL
                         : (pct < 0.67f) ? COL_AMBER : COL_RED;

        fb_arc(CX, CY, 110, 12, -225.0f, 45.0f, COL_SURFACE);
        if (valid && pct > 0.005f)
            fb_arc(CX, CY, 110, 12, -225.0f, -225.0f + pct * 270.0f, arc_col);

        // Session best — white tick on arc
        if (g_session_best_cda < 9000.0f) {
            float bp = (g_session_best_cda - 0.18f) / 0.20f;
            if (bp < 0.0f) bp = 0.0f;
            if (bp > 1.0f) bp = 1.0f;
            float ba = -225.0f + bp * 270.0f;
            fb_arc(CX, CY, 110, 12, ba - 1.5f, ba + 1.5f, rgb(255,255,255));
        }

        fb_str((FB_W - 36) / 2, 8, "CdA", COL_MUTED, 2);

        // Grade (hero)
        const char *grade     = valid ? cda_grade(cda) : "--";
        uint16_t    grade_col = valid ? arc_col : COL_MUTED;
        int gw = (int)strlen(grade) * 6 * 4;
        fb_str((FB_W - gw) / 2, CY - 40, grade, grade_col, 4);

        if (valid) {
            char buf[8];
            snprintf(buf, sizeof(buf), "0.%03d", (int)(cda * 1000) % 1000);
            int vw = (int)strlen(buf) * 12;
            fb_str((FB_W - vw) / 2, CY + 8, buf, COL_TEXT, 2);
            fb_str(CX - 12, CY + 26, "m2", COL_MUTED, 2);
        } else {
            fb_str(CX - 18, CY + 8, "---", COL_MUTED, 2);
        }

        // "NEW BEST" blink
        if (now_ms < g_best_flash_end && (now_ms / 500) % 2 == 0) {
            int nbw = (int)strlen("NEW BEST") * 12;
            fb_str((FB_W - nbw) / 2, CY + 44, "NEW BEST", COL_TEAL, 2);
        }

        // Bottom info bar: speed | wind | power
        int bby = FB_H - 34;
        if (valid && p->v_ground_ms > 0.5f) {
            char spd[6];
            snprintf(spd, sizeof(spd), "%d", (int)(p->v_ground_ms * 3.6f));
            fb_str(8, bby, spd, COL_TEXT, 2);
            fb_str(8 + (int)strlen(spd) * 12, bby + 8, "km/h", COL_MUTED, 2);
        }
        if (s->pitot_valid && s->pitot_pa > 0.3f) {
            float v_air = sqrtf(2.0f * s->pitot_pa / 1.225f) * 3.6f;
            char wa[6];
            snprintf(wa, sizeof(wa), "%d", (int)v_air);
            int wx = CX - (int)strlen(wa) * 6;
            fb_str(wx, bby, wa, COL_TEAL, 2);
            fb_str(wx + (int)strlen(wa) * 12, bby + 8, "wind", COL_MUTED, 2);
        }
        if (s->power_w > 0) {
            char pw[6];
            snprintf(pw, sizeof(pw), "%d", s->power_w);
            int px = FB_W - (int)strlen(pw) * 12 - 14;
            fb_str(px, bby, pw, COL_AMBER, 2);
            fb_str(px + (int)strlen(pw) * 12, bby + 8, "W", COL_MUTED, 2);
        }

        // Battery icon top-right
        {
            uint16_t bat_col = s->battery_pct > 25 ? COL_TEAL : COL_RED;
            char bat_str[6];
            snprintf(bat_str, sizeof(bat_str), "%d%%", s->battery_pct);
            int bpw = (int)strlen(bat_str) * 12;
            fb_str(FB_W - 24 - bpw - 3, 8, bat_str, bat_col, 2);
            fb_battery(FB_W - 24, 8, s->battery_pct, bat_col);
        }
        render_status_dots(s);
        break;
    }

    // ── Lap / Session timer ───────────────────────────────────────────────────
    case SCR_TIMER: {
        fb_str((FB_W - 36) / 2, 18, "LAP", COL_MUTED, 2);
        {
            char nbuf[6];
            snprintf(nbuf, sizeof(nbuf), "%d", g_lap_num_display);
            int nw = (int)strlen(nbuf) * 30;
            fb_str((FB_W - nw) / 2, 40, nbuf, COL_TEAL, 5);
        }
        {
            uint16_t lap_col = (g_lap_elapsed_s < 60) ? COL_TEAL : COL_AMBER;
            fb_time((FB_W - 5*24) / 2, 95, g_lap_elapsed_s, lap_col, 4);
        }
        fb_str((FB_W - 18) / 2, 130, "LAP", COL_MUTED, 1);
        fb_hline(30, FB_W - 30, 150, COL_SURFACE);
        fb_hline(30, FB_W - 30, 151, COL_SURFACE);
        fb_str((FB_W - 18) / 2, 162, "TOT", COL_MUTED, 1);
        fb_time((FB_W - 5*18) / 2, 174, g_session_elapsed_s, COL_TEXT, 3);
        render_status_dots(s);
        break;
    }

    // ── Speed ─────────────────────────────────────────────────────────────────
    case SCR_SPEED: {
        fb_str((FB_W - 60) / 2, 8, "Speed", COL_MUTED, 2);

        float spd_kmh = s->gps_valid    ? s->speed_ms * 3.6f
                      : (p && p->valid) ? p->v_ground_ms * 3.6f
                      : 0.0f;
        char spd_buf[6];
        snprintf(spd_buf, sizeof(spd_buf), "%d", (int)spd_kmh);
        int sw = (int)strlen(spd_buf) * 36;
        fb_str((FB_W - sw) / 2, CY - 22, spd_buf, COL_TEXT, 6);

        int km_w = 4 * 12;
        fb_str((FB_W - km_w) / 2, CY + 26, "km/h", COL_MUTED, 2);

        if (s->pitot_valid && s->pitot_pa > 0.3f) {
            float v_air  = sqrtf(2.0f * s->pitot_pa / 1.225f) * 3.6f;
            float delta  = spd_kmh - v_air;
            char  air[20], wind[20];
            snprintf(air,  sizeof(air),  "Air: %d km/h",  (int)v_air);
            snprintf(wind, sizeof(wind), "Wind: %+d km/h", (int)delta);
            int aw = (int)strlen(air) * 12, ww = (int)strlen(wind) * 12;
            fb_str((FB_W - aw) / 2, CY + 44, air,  COL_TEAL, 2);
            uint16_t wcol = (delta >  2.0f) ? COL_RED
                          : (delta < -2.0f) ? COL_TEAL : COL_MUTED;
            fb_str((FB_W - ww) / 2, CY + 60, wind, wcol, 2);
        }

        if (s->hr_bpm > 0) {
            char hr[6];
            snprintf(hr, sizeof(hr), "%d", s->hr_bpm);
            fb_str(FB_W - (int)strlen(hr) * 12 - 4, 8, hr, COL_RED, 2);
            fb_str(FB_W - 24, 24, "bpm", COL_MUTED, 2);
        }

        if (s->power_w > 0) {
            char pw[12];
            snprintf(pw, sizeof(pw), "%d W", s->power_w);
            int pw_w = (int)strlen(pw) * 12;
            fb_str((FB_W - pw_w) / 2, FB_H - 44,
                   pw, power_zone_col(s->power_w, g_rider_ftp_w), 2);
        }
        render_status_dots(s);
        break;
    }

    // ── Power / Energy split ──────────────────────────────────────────────────
    case SCR_POWER: {
        char buf[16];
        uint16_t    zone_col  = power_zone_col(s->power_w, g_rider_ftp_w);
        const char *zone_name = power_zone_name(s->power_w, g_rider_ftp_w);

        snprintf(buf, sizeof(buf), "%d W", s->power_w);
        int pw_w = (int)strlen(buf) * 18;
        fb_str((FB_W - pw_w) / 2, 10, buf, zone_col, 3);

        int znw = (int)strlen(zone_name) * 12;
        fb_str((FB_W - znw) / 2, 38, zone_name, zone_col, 2);

        if (g_rider_mass_kg > 0.0f && s->power_w > 0) {
            snprintf(buf, sizeof(buf), "%.1f W/kg", s->power_w / g_rider_mass_kg);
            int wkw = (int)strlen(buf) * 12;
            fb_str((FB_W - wkw) / 2, 56, buf, COL_MUTED, 2);
        }

        if (p && p->valid && s->power_w > 0) {
            uint8_t pa  = p->pct_aero;
            float   pr_f = (p->p_rolling_w / (float)s->power_w) * 100.0f;
            uint8_t pr  = (pr_f > 100.0f) ? 100 : (pr_f < 0.0f) ? 0 : (uint8_t)pr_f;
            uint8_t po  = (pa + pr < 100) ? (100 - pa - pr) : 0;

            const int BH = 100, BY = 84, BW = 30, GAP = 14;
            int bx = (FB_W - (3*BW + 2*GAP)) / 2;

            struct { uint8_t pct; uint16_t col; const char *lbl; } bars[3] = {
                {pa, COL_RED,   "Aero"},
                {pr, COL_TEAL,  "Roll"},
                {po, COL_MUTED, "Other"},
            };
            for (int b = 0; b < 3; b++) {
                int filled = BH * bars[b].pct / 100;
                for (int y = BY; y < BY + BH; y++)
                    for (int x = bx; x < bx + BW; x++)
                        if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y) = COL_SURFACE;
                for (int y = BY + BH - filled; y < BY + BH; y++)
                    for (int x = bx; x < bx + BW; x++)
                        if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y) = bars[b].col;

                int lw = (int)strlen(bars[b].lbl) * 6;
                fb_str(bx + (BW - lw) / 2, BY - 14, bars[b].lbl, bars[b].col, 1);

                snprintf(buf, sizeof(buf), "%d%%", bars[b].pct);
                int buw = (int)strlen(buf) * 6;
                fb_str(bx + (BW - buw) / 2, BY + BH + 6, buf, bars[b].col, 1);

                bx += BW + GAP;
            }
        } else {
            int dw = 3 * 12;
            fb_str((FB_W - dw) / 2, CY, "---", COL_MUTED, 2);
        }
        render_status_dots(s);
        break;
    }

    // ── Status / Vitals 2×2 ───────────────────────────────────────────────────
    case SCR_STATUS: {
        char buf[20];

        // Grid dividers
        fb_hline(0, FB_W - 1, 159, COL_SURFACE);
        fb_hline(0, FB_W - 1, 160, COL_SURFACE);
        for (int y = 0; y < FB_H; y++)
            if (y >= 0 && y < FB_H) { PX(119,y) = COL_SURFACE; PX(120,y) = COL_SURFACE; }

        // TL — CdA
        {
            bool     cda_valid = (p && p->valid);
            float    cda_val   = cda_valid ? p->CdA : 0.0f;
            float    cpct = cda_valid ? (cda_val - 0.18f) / 0.20f : 0.0f;
            if (cpct < 0.0f) cpct = 0.0f;
            if (cpct > 1.0f) cpct = 1.0f;
            uint16_t cda_col = (cpct < 0.33f) ? COL_TEAL
                             : (cpct < 0.67f) ? COL_AMBER : COL_RED;
            if (!cda_valid) cda_col = COL_MUTED;

            fb_str(10, 10, "CdA", COL_MUTED, 2);
            if (cda_valid) {
                snprintf(buf, sizeof(buf), "0.%03d", (int)(cda_val * 1000) % 1000);
                fb_str(10, 28, buf, cda_col, 3);
                fb_str(10, 72, cda_grade(cda_val), cda_col, 2);
            } else {
                fb_str(10, 28, "---", COL_MUTED, 3);
            }
        }

        // TR — Heart Rate
        fb_str(130, 10, "HR", COL_MUTED, 2);
        {
            uint16_t hr_col = s->hr_bpm > 0 ? COL_RED : COL_MUTED;
            snprintf(buf, sizeof(buf), "%d", s->hr_bpm > 0 ? s->hr_bpm : 0);
            fb_str(130, 28, buf, hr_col, 4);
            fb_str(130, 64, "bpm", COL_MUTED, 2);
            fb_circle(195, 108, 5, hr_col);
        }

        // BL — Cadence
        fb_str(10, 172, "Cadence", COL_MUTED, 2);
        {
            uint16_t cad_col = s->cadence_rpm > 0 ? COL_AMBER : COL_MUTED;
            snprintf(buf, sizeof(buf), "%d", s->cadence_rpm > 0 ? s->cadence_rpm : 0);
            fb_str(10, 190, buf, cad_col, 4);
            fb_str(10, 226, "rpm", COL_MUTED, 2);
        }

        // BR — Power
        {
            uint16_t pwr_col = power_zone_col(s->power_w, g_rider_ftp_w);
            fb_str(130, 172, "Power", COL_MUTED, 2);
            snprintf(buf, sizeof(buf), "%d", s->power_w > 0 ? s->power_w : 0);
            fb_str(130, 190, buf, pwr_col, 4);
            fb_str(130, 226, "W", COL_MUTED, 2);
            if (s->power_w > 0) {
                const char *zn = power_zone_name(s->power_w, g_rider_ftp_w);
                fb_str(130, 250, zn, pwr_col, 1);
            }
            fb_circle(195, 295, 4, s->ant_valid ? COL_TEAL : COL_MUTED);
        }
        break;
    }

    default: break;
    }

    // ── Toast overlay ─────────────────────────────────────────────────────────
    {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
        if (g_toast_end_ms > now_ms && g_toast_msg[0]) {
            int msg_w = (int)strlen(g_toast_msg) * 12;
            int box_w = msg_w + 24, box_h = 30;
            int bx = (FB_W - box_w) / 2, by = (FB_H - box_h) / 2;
            for (int yy = by - 2; yy < by + box_h + 2; yy++)
                for (int xx = bx - 2; xx < bx + box_w + 2; xx++)
                    if (xx >= 0 && xx < FB_W && yy >= 0 && yy < FB_H)
                        PX(xx, yy) = COL_TEAL;
            for (int yy = by; yy < by + box_h; yy++)
                for (int xx = bx; xx < bx + box_w; xx++)
                    if (xx >= 0 && xx < FB_W && yy >= 0 && yy < FB_H)
                        PX(xx, yy) = COL_BG;
            fb_str((FB_W - msg_w) / 2, by + 8, g_toast_msg, COL_TEAL, 2);
        }
    }
    display_flush();
}

void display_show_toast(const char *msg, uint32_t duration_ms)
{
    strlcpy(g_toast_msg, msg, sizeof(g_toast_msg));
    g_toast_end_ms = (uint32_t)(esp_timer_get_time() / 1000LL) + duration_ms;
}

void display_clear_toast(void)
{
    g_toast_msg[0] = '\0';
    g_toast_end_ms  = 0;
}

void display_set_connection_status(bool ble, bool wifi)
{
    g_ble_connected = ble;
    g_wifi_ready    = wifi;
}

void display_set_timers(uint32_t session_s, uint32_t lap_s, uint16_t lap_num)
{
    g_session_elapsed_s = session_s;
    g_lap_elapsed_s     = lap_s;
    g_lap_num_display   = lap_num;
}

void display_next_screen(void)
{
    screen_t n = (screen_t)((g_screen + 1) % SCR_COUNT);
    if (n == SCR_PAIRING) n = SCR_CDA;
    g_screen = n;
}

void display_set_screen(screen_t scr)
{
    g_screen = scr;
}
