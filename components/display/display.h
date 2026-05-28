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
        .max_transfer_sz = FB_W * 40 * 2,   // 40 lines per DMA transfer
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

static void fb_circle(int cx, int cy, int r, uint16_t col)
{
    int r2 = r * r;
    for (int y = cy-r; y <= cy+r; y++)
    for (int x = cx-r; x <= cx+r; x++) {
        if (x<0||x>=FB_W||y<0||y>=FB_H) continue;
        if ((x-cx)*(x-cx)+(y-cy)*(y-cy) <= r2) PX(x,y) = col;
    }
}

static void fb_ring(int cx, int cy, int r, int w, uint16_t col)
{
    int ro2 = r*r, ri2 = (r-w)*(r-w);
    for (int y = cy-r; y <= cy+r; y++)
    for (int x = cx-r; x <= cx+r; x++) {
        if (x<0||x>=FB_W||y<0||y>=FB_H) continue;
        int d2 = (x-cx)*(x-cx)+(y-cy)*(y-cy);
        if (d2 <= ro2 && d2 >= ri2) PX(x,y) = col;
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

static void fb_vline(int x, int y0, int y1, uint16_t col)
{
    if (x<0||x>=FB_W) return;
    if (y0>y1) { int t=y0; y0=y1; y1=t; }
    for (int y=y0; y<=y1; y++) if (y>=0&&y<FB_H) PX(x,y)=col;
}

// ─── 5×7 pixel font ──────────────────────────────────────────────────────────
static const uint8_t FONT5X7[][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'  index 0
    {0x00,0x42,0x7F,0x40,0x00}, // '1'  index 1
    {0x42,0x61,0x51,0x49,0x46}, // '2'  index 2
    {0x21,0x41,0x45,0x4B,0x31}, // '3'  index 3
    {0x18,0x14,0x12,0x7F,0x10}, // '4'  index 4
    {0x27,0x45,0x45,0x45,0x39}, // '5'  index 5
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'  index 6
    {0x01,0x71,0x09,0x05,0x03}, // '7'  index 7
    {0x36,0x49,0x49,0x49,0x36}, // '8'  index 8
    {0x06,0x49,0x49,0x29,0x1E}, // '9'  index 9
    {0x00,0x60,0x60,0x00,0x00}, // '.'  index 10
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'  index 11
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'  index 12
    {0x08,0x08,0x08,0x08,0x08}, // '-'  index 13
    {0x00,0x00,0x00,0x00,0x00}, // ' '  index 14
    {0x7F,0x08,0x14,0x22,0x00}, // 'k'  index 15
    {0x7F,0x02,0x04,0x02,0x7F}, // 'm'  index 16
    {0x10,0x08,0x04,0x02,0x01}, // '/'  index 17
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'  index 18
    {0x26,0x16,0x08,0x34,0x32}, // '%'  index 19
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'  index 20
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'  index 21
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'  index 22
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'  index 23
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'  index 24
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'  index 25
    {0x00,0x36,0x36,0x00,0x00}, // ':'  index 26
};
#define FONT_IDX_DOT   10
#define FONT_IDX_W     11
#define FONT_IDX_DASH  13
#define FONT_IDX_SPACE 14

static void fb_char(int x, int y, char c, uint16_t col, int scale)
{
    int idx = -1;
    if (c>='0' && c<='9') idx = c-'0';
    else if (c>='A' && c<='F') idx = 20 + (c - 'A');
    else if (c=='.') idx = FONT_IDX_DOT;
    else if (c=='W') idx = FONT_IDX_W;
    else if (c=='-') idx = FONT_IDX_DASH;
    else if (c==' ') idx = FONT_IDX_SPACE;
    else if (c=='k') idx = 15;
    else if (c=='m') idx = 16;
    else if (c=='/') idx = 17;
    else if (c=='h') idx = 18;
    else if (c=='%') idx = 19;
    else if (c==':') idx = 26;
    if (idx<0) return;
    for (int row=0; row<7; row++)
    for (int col_b=0; col_b<5; col_b++) {
        if (!((FONT5X7[idx][col_b]>>row)&1)) continue;
        for (int sy=0; sy<scale; sy++)
        for (int sx=0; sx<scale; sx++) {
            int px=x+col_b*scale+sx, py=y+row*scale+sy;
            if (px>=0&&px<FB_W&&py>=0&&py<FB_H) PX(px,py)=col;
        }
    }
}

static void fb_str(int x, int y, const char *s, uint16_t col, int scale)
{
    while (*s) { fb_char(x, y, *s++, col, scale); x += (5+1)*scale; }
}

// ─── QR pairing state ────────────────────────────────────────────────────────
#define QR_BUF_LEN  qrcodegen_BUFFER_LEN_FOR_VERSION(5)

static char    g_pairing_id[32]    = {0};
static uint8_t g_qr_cache[QR_BUF_LEN] = {0};
static bool    g_qr_valid          = false;

void display_set_pairing_id(const char *id)
{
    strlcpy(g_pairing_id, id, sizeof(g_pairing_id));

    uint8_t tmp[QR_BUF_LEN];
    char qr_text[64];
    // Uppercase URI: all chars are in QR alphanumeric set → smaller code (version 2)
    snprintf(qr_text, sizeof(qr_text), "AERODRAG://PAIR/%s", id);
    g_qr_valid = qrcodegen_encodeText(qr_text, tmp, g_qr_cache,
                                       qrcodegen_Ecc_LOW, 1, 5,
                                       qrcodegen_Mask_AUTO, true);
}

// ─── Screen state ─────────────────────────────────────────────────────────────
typedef enum { SCR_PAIRING=0, SCR_CDA, SCR_POWER, SCR_STATUS, SCR_COUNT } screen_t;
static screen_t g_screen = SCR_PAIRING;

screen_t display_get_screen(void)
{
    return g_screen;
}

// ─── Render ───────────────────────────────────────────────────────────────────
void display_render(const aerodrag_sensors_t *s, const aerodrag_physics_t *p)
{
    if (!g_fb) return;
    const int CX = FB_W/2, CY = FB_H/2;   // 120, 160

    fb_fill(COL_BG);

    if (g_screen != SCR_PAIRING)
        fb_ring(CX, CY, 116, 3, COL_SURFACE);

switch (g_screen) {

    case SCR_PAIRING: {
        if (!g_qr_valid) {
            // fallback: just show device ID
            int w = (int)strlen(g_pairing_id) * 6;
            fb_str((FB_W - w) / 2, CY - 4, g_pairing_id, COL_TEAL, 1);
            break;
        }
        int sz  = qrcodegen_getSize(g_qr_cache);
        const int SCALE = 7;
        const int PAD   = 10;   // white quiet-zone pixels on each side
        int qr_px  = sz * SCALE;
        int rect_w = qr_px + 2 * PAD;
        int rect_h = qr_px + 2 * PAD;
        int rx = (FB_W - rect_w) / 2;
        int ry = 20;
        int ox = rx + PAD;
        int oy = ry + PAD;

        // White background for QR
        uint16_t WHITE = rgb(255,255,255);
        uint16_t BLACK = rgb(0,0,0);
        for (int y = ry; y < ry + rect_h; y++)
            for (int x = rx; x < rx + rect_w; x++)
                if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y) = WHITE;

        // QR modules
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

        // Device ID below QR (scale=1 fits "AA:BB:CC:DD:EE:FF" = 102px)
        {
            int id_len = (int)strlen(g_pairing_id);
            int id_w   = id_len * 6;
            int id_x   = (FB_W - id_w) / 2;
            int id_y   = ry + rect_h + 8;
            fb_str(id_x, id_y, g_pairing_id, COL_MUTED, 1);
        }

        // "Not connected" indicator dot
        fb_circle(CX, ry + rect_h + 30, 4, COL_RED);
        break;
    }

    case SCR_CDA: {
        float cda = (p && p->valid) ? p->CdA : 0.0f;
        float pct = (cda - 0.20f) / 0.18f;
        if (pct < 0) pct = 0;
        if (pct > 1) pct = 1;
        uint16_t arc_col = (pct < 0.35f) ? COL_TEAL
                         : (pct < 0.65f) ? COL_AMBER
                         :                  COL_RED;
        fb_arc(CX, CY, 110, 10, -225.0f, 45.0f, COL_SURFACE);
        if (pct > 0.01f)
            fb_arc(CX, CY, 110, 10, -225.0f, -225.0f + pct*270.0f, arc_col);
        if (p && p->valid) {
            char buf[8];
            fb_str(CX - 52, CY - 26, "0.", COL_MUTED, 3);
            snprintf(buf, sizeof(buf), "%03d", (int)(cda * 1000) % 1000);
            fb_str(CX - 28, CY - 32, buf, arc_col, 4);
        } else {
            if (s->pitot_valid && s->pitot_pa > 0.3f) {
                float rho = 1.225f;
                float v_air = sqrtf(2.0f * s->pitot_pa / rho) * 3.6f;
                char buf[6];
                snprintf(buf, sizeof(buf), "%d", (int)v_air);
                fb_str(CX - 20, CY - 22, buf, COL_TEAL, 3);
                fb_str(CX - 28, CY + 12, "km/h", COL_MUTED, 2);
            } else {
                fb_str(CX - 42, CY - 22, "0.---", COL_MUTED, 3);
            }
        }
        if (p && p->valid && p->v_ground_ms > 0.5f) {
            char spd[4];
            snprintf(spd, sizeof(spd), "%d", (int)(p->v_ground_ms * 3.6f));
            int sw = (strlen(spd) * 6 * 2);
            fb_str(CX - sw/2, CY + 52, spd, COL_TEXT, 2);
        }
        if (s->power_w > 0) {
            char pw[6];
            snprintf(pw, sizeof(pw), "%d", s->power_w);
            fb_str(18, 28, pw, COL_AMBER, 2);
        }
        float bat = s->battery_pct / 100.0f;
        if (bat > 0.02f) {
            fb_arc(CX, CY, 115, 4,
                   70.0f, 70.0f + bat * 40.0f,
                   bat > 0.25f ? COL_TEAL : COL_RED);
        }
        fb_circle(CX + 48, CY - 78, 5,
                  s->pitot_valid ? COL_TEAL : COL_MUTED);
        break;
    }

    case SCR_POWER: {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", s->power_w);
        fb_str(CX - (int)strlen(buf)*9, CY - 54, buf, COL_AMBER, 3);
        fb_str(CX + 16, CY - 36, "W", COL_AMBER, 2);
        if (p && p->valid && s->power_w > 0) {
            uint8_t pa = p->pct_aero;
            float   pr_f = (p->p_rolling_w / (float)s->power_w) * 100.0f;
            uint8_t pr = (pr_f > 100.0f) ? 100 : (pr_f < 0.0f) ? 0 : (uint8_t)pr_f;
            uint8_t po = (pa + pr < 100) ? (100 - pa - pr) : 0;
            const int BH = 70, BY = CY + 10, BW = 22, GAP = 8;
            int bx = CX - BW - GAP/2 - BW/2;
            struct { uint8_t pct; uint16_t col; } bars[3] = {
                {pa, COL_RED}, {pr, COL_TEAL}, {po, COL_MUTED}
            };
            for (int b = 0; b < 3; b++) {
                int filled = BH * bars[b].pct / 100;
                for (int y=BY; y<BY+BH; y++)
                    for (int x=bx; x<bx+BW; x++)
                        if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y)=COL_SURFACE;
                for (int y=BY+BH-filled; y<BY+BH; y++)
                    for (int x=bx; x<bx+BW; x++)
                        if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y)=bars[b].col;
                bx += BW + GAP;
            }
            snprintf(buf, sizeof(buf), "%d%%", pa);
            fb_str(CX - 52, BY + BH + 6, buf, COL_RED, 1);
            snprintf(buf, sizeof(buf), "%d%%", pr);
            fb_str(CX - 12, BY + BH + 6, buf, COL_TEAL, 1);
        }
        break;
    }

    case SCR_STATUS: {
    char buf[8];

    snprintf(buf, sizeof(buf), "%d%%", s->battery_pct);
    fb_str(CX - 18, CY - 80, buf, s->battery_pct > 25 ? COL_TEAL : COL_RED, 2);

    fb_circle(CX - 40, CY, 8, COL_RED);
    snprintf(buf, sizeof(buf), "%d", s->hr_bpm > 0 ? s->hr_bpm : 0);
    fb_str(CX - 25, CY - 6, buf, COL_TEXT, 2);

    snprintf(buf, sizeof(buf), "%d", s->cadence_rpm > 0 ? s->cadence_rpm : 0);
    fb_str(CX + 30, CY - 6, buf, COL_AMBER, 2);

    uint16_t gps_col = (s->gps_fix >= 2) ? COL_TEAL
                     : (s->gps_fix == 1) ? COL_AMBER
                     :                      COL_RED;
    fb_hline(CX - 40, CX + 40, CY + 80, gps_col);
    fb_hline(CX - 40, CX + 40, CY + 81, gps_col);
    break;
}

    default: break;
    }
    display_flush();
}

void display_next_screen(void)
{
    screen_t n = (screen_t)((g_screen + 1) % SCR_COUNT);
    if (n == SCR_PAIRING) n = SCR_CDA;  // skip pairing in manual cycle
    g_screen = n;
}

void display_set_screen(screen_t scr)
{
    g_screen = scr;
}
