#pragma once
#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "board_pins.h"
#include "aerodrag_types.h"
#include "qrcodegen.h"
#include "aafont_data.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// L'init ST7789 è gestito dal driver ufficiale esp_lcd (esp_lcd_new_panel_st7789).

// ─── Framebuffer — 240×320 × 2 bytes = 153.6 KB ──────────────────────────────
#define FB_W  240
#define FB_H  320
#define FB_BYTES (FB_W * FB_H * 2)

static esp_lcd_panel_io_handle_t g_io_handle = NULL;
static esp_lcd_panel_handle_t    g_panel     = NULL;
static SemaphoreHandle_t         g_lcd_done  = NULL;
static uint16_t                 *g_fb     = NULL;   // framebuffer di rendering (PSRAM)
static bool                       g_bl_on = false;

// ─── Colour helpers ───────────────────────────────────────────────────────────
static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
}
static inline uint16_t swap16(uint16_t v) { return (v << 8) | (v >> 8); }

// ── AeroDrag palette unificata (RGB565 via rgb(), derivata dall'hex) ──────────
#define COL_ACCENT   rgb(0,   217, 163)  // #00d9a3  CdA, brand, OK, attivo
#define COL_POWER    rgb(245, 166, 35)   // #f5a623  potenza, quota AERO, warning
#define COL_SPEED    rgb(77,  159, 255)  // #4d9fff  velocità, rolling, info
#define COL_ALERT    rgb(255, 77,  106)  // #ff4d6a  SOLO FC, peak, allarmi
#define COL_POSITIVE rgb(34,  197, 94)   // #22c55e  new best, delta migliore
#define COL_TEXT     rgb(219, 230, 246)  // #dbe6f6  valori, titoli
#define COL_TEXTDIM  rgb(131, 152, 189)  // #8398bd  testo secondario
#define COL_MUTED    rgb(70,  88,  124)  // #46587c  label, unità, assi
#define COL_SURFACE  rgb(15,  20,  32)   // #0f1420  card
#define COL_TRACK    rgb(30,  40,  64)   // #1e2840  track anelli/barre
#define COL_PANEL    rgb(19,  26,  40)   // #131a28  divisori
#define COL_BG       rgb(7,   9,   15)   // #07090f  sfondo schermo

// ─── Callback fine trasferimento colore (sincronizza il flush) ────────────────
static bool lcd_color_done(esp_lcd_panel_io_handle_t io,
                           esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    BaseType_t hp = pdFALSE;
    if (g_lcd_done) xSemaphoreGiveFromISR(g_lcd_done, &hp);
    return hp == pdTRUE;
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

// ─── Init (driver ufficiale esp_lcd / ST7789) ─────────────────────────────────
esp_err_t display_init(void)
{
    g_lcd_done = xSemaphoreCreateBinary();

    spi_bus_config_t bus = {
        .mosi_io_num     = PIN_LCD_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = PIN_LCD_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = FB_W * 40 * 2 + 16,   // una strip da 40 righe
    };
    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) { ESP_LOGE("lcd", "spi_bus_initialize: %s", esp_err_to_name(ret)); return ret; }

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num         = PIN_LCD_DC,
        .cs_gpio_num         = PIN_LCD_CS,
        .pclk_hz             = LCD_SPI_CLK_HZ,
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
        .spi_mode            = 0,
        .trans_queue_depth   = 10,
        .on_color_trans_done = lcd_color_done,
    };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &g_io_handle);
    if (ret != ESP_OK) { ESP_LOGE("lcd", "panel_io_spi: %s", esp_err_to_name(ret)); return ret; }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_st7789(g_io_handle, &panel_cfg, &g_panel);
    if (ret != ESP_OK) { ESP_LOGE("lcd", "new_panel_st7789: %s", esp_err_to_name(ret)); return ret; }

    esp_lcd_panel_reset(g_panel);
    esp_lcd_panel_init(g_panel);
    esp_lcd_panel_invert_color(g_panel, true);     // ST7789: inversione colore ON
    esp_lcd_panel_disp_on_off(g_panel, true);
    ESP_LOGI("lcd", "esp_lcd ST7789 init OK @ %d MHz", LCD_SPI_CLK_HZ/1000000);

    bl_init();
    display_set_brightness(90);
    g_bl_on = true;

    g_fb = (uint16_t *)heap_caps_malloc(FB_BYTES, MALLOC_CAP_SPIRAM);
    if (!g_fb) g_fb = (uint16_t *)heap_caps_malloc(FB_BYTES, MALLOC_CAP_INTERNAL);
    if (!g_fb) return ESP_ERR_NO_MEM;

    memset(g_fb, 0, FB_BYTES);
    return ESP_OK;
}

// ─── Flush framebuffer (esp_lcd) ──────────────────────────────────────────────
static void display_flush(void)
{
    if (!g_panel || !g_fb) return;
    // Flush a STRIP da RAM INTERNA: la DMA SPI full-frame da PSRAM è instabile
    // (queue color failed). ST7789 vuole RGB565 big-endian → byte-swap di ogni
    // strip in un buffer .bss DMA-capable, una strip alla volta.
    static uint16_t strip[FB_W * 40];
    for (int y = 0; y < FB_H; y += 40) {
        int h = (y + 40 > FB_H) ? (FB_H - y) : 40;
        for (int i = 0; i < h * FB_W; i++) strip[i] = swap16(g_fb[y * FB_W + i]);
        esp_lcd_panel_draw_bitmap(g_panel, 0, y, FB_W, y + h, strip);
        if (g_lcd_done) xSemaphoreTake(g_lcd_done, pdMS_TO_TICKS(200));  // libera la strip
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

// ─── Font anti-aliased (atlanti pre-rasterizzati in aafont_data.h) ───────────
// JetBrains Mono (numeri) + Inter (label/unità). Testo in alpha-blend RGB565.
// Famiglie disponibili: AAF_HERO(46) AAF_GRADE(48,ExtraBold) AAF_VAL(34)
//                       AAF_MED(26) AAF_NUMS(14) AAF_LABEL(13) AAF_LBLS(11)

static const aaglyph_t *aaf_glyph(const aafont_t *f, uint16_t cp)
{
    for (uint16_t i = 0; i < f->n; i++) if (f->g[i].cp == cp) return &f->g[i];
    return NULL;
}

// UTF-8 minimale: ASCII + 2 byte (² ° ·). Avanza *s, ritorna il codepoint.
static uint16_t utf8_next(const char **s)
{
    const unsigned char *p = (const unsigned char *)*s;
    uint16_t cp = *p;
    if (cp < 0x80)                      { *s += 1; return cp; }
    if ((cp & 0xE0) == 0xC0 && p[1])    { cp = ((cp & 0x1F) << 6) | (p[1] & 0x3F); *s += 2; return cp; }
    *s += 1; return cp;
}

// Alpha-blend di fg su bg (RGB565 nativo, non byte-swapped).
static inline uint16_t blend565(uint16_t bg, uint16_t fg, uint8_t a)
{
    if (a == 0)   return bg;
    if (a >= 255) return fg;
    uint32_t br = (bg >> 11) & 0x1F, bgc = (bg >> 5) & 0x3F, bb = bg & 0x1F;
    uint32_t fr = (fg >> 11) & 0x1F, fgc = (fg >> 5) & 0x3F, fb = fg & 0x1F;
    uint32_t ia = 255 - a;
    uint32_t r = (fr * a + br * ia) / 255;
    uint32_t g = (fgc * a + bgc * ia) / 255;
    uint32_t b = (fb * a + bb * ia) / 255;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

static int aaf_text_w(const aafont_t *f, const char *s)
{
    int w = 0;
    while (*s) { uint16_t cp = utf8_next(&s); const aaglyph_t *g = aaf_glyph(f, cp); if (g) w += g->adv; }
    return w;
}

// estensione verticale dell'inchiostro (per il centraggio robusto dei valori)
static void aaf_ink_v(const aafont_t *f, const char *s, int *top, int *bot)
{
    int t = 999, b = -999;
    while (*s) {
        uint16_t cp = utf8_next(&s);
        const aaglyph_t *g = aaf_glyph(f, cp);
        if (!g || g->h == 0) continue;
        if (g->yoff < t)         t = g->yoff;
        if (g->yoff + g->h > b)  b = g->yoff + g->h;
    }
    if (t > 900) { t = 0; b = 0; }
    *top = t; *bot = b;
}

// y = riga ascender (top). Ritorna la x avanzata.
static int fb_text(int x, int y, const char *s, const aafont_t *f, uint16_t col)
{
    while (*s) {
        uint16_t cp = utf8_next(&s);
        const aaglyph_t *g = aaf_glyph(f, cp);
        if (!g) continue;
        const uint8_t *bm = f->bm + g->off;
        for (int gy = 0; gy < g->h; gy++) {
            int py = y + g->yoff + gy;
            if (py < 0 || py >= FB_H) continue;
            const uint8_t *row = bm + gy * g->w;
            for (int gx = 0; gx < g->w; gx++) {
                uint8_t a = row[gx];
                if (!a) continue;
                int px = x + g->xoff + gx;
                if (px < 0 || px >= FB_W) continue;
                uint16_t *d = &PX(px, py);
                *d = blend565(*d, col, a);
            }
        }
        x += g->adv;
    }
    return x;
}

static void fb_text_cx(int cx, int y, const char *s, const aafont_t *f, uint16_t col)
{ fb_text(cx - aaf_text_w(f, s) / 2, y, s, f, col); }

static void fb_text_r(int xr, int y, const char *s, const aafont_t *f, uint16_t col)
{ fb_text(xr - aaf_text_w(f, s), y, s, f, col); }

// Centra l'inchiostro della stringa su (cx,cy) — per i valori "hero".
static void fb_text_center(int cx, int cy, const char *s, const aafont_t *f, uint16_t col)
{
    int top, bot; aaf_ink_v(f, s, &top, &bot);
    fb_text(cx - aaf_text_w(f, s) / 2, cy - (top + bot) / 2, s, f, col);
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
    if (pct < 75)  return COL_ACCENT;
    if (pct < 90)  return rgb(80, 200, 120);
    if (pct < 105) return COL_POWER;
    if (pct < 120) return rgb(255, 120, 20);
    return COL_ALERT;
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

// ─── Status dots (4×, riga dedicata centrata in basso): WiFi BLE ANT+ Pitot ──
static void render_status_dots(const aerodrag_sensors_t *s)
{
    struct { uint16_t col; const char *lbl; } dots[4] = {
        { g_wifi_ready    ? COL_ACCENT : COL_MUTED, "W" },
        { g_ble_connected ? COL_ACCENT : COL_MUTED, "B" },
        { s->ant_valid    ? COL_ACCENT : COL_MUTED, "A" },
        { s->pitot_valid  ? COL_ACCENT : COL_MUTED, "P" },
    };
    const int STEP = 30;                       // passo per item (dot + lettera)
    int x0 = FB_W / 2 - (STEP * 4 - 12) / 2;    // riga centrata
    int dy = FB_H - 9;                          // centro dei pallini
    for (int i = 0; i < 4; i++) {
        int dx = x0 + i * STEP;
        fb_circle(dx, dy, 3, dots[i].col);
        fb_text(dx + 7, FB_H - 16, dots[i].lbl, &AAF_LBLS, COL_MUTED);
    }
}

// ─── MM:SS helper (centrato su cx, top y) ─────────────────────────────────────
static void fb_time_cx(int cx, int y, uint32_t secs, uint16_t col, const aafont_t *f)
{
    char buf[6];
    uint32_t mm = secs / 60;
    if (mm > 99) mm = 99;
    snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)mm, (unsigned long)(secs % 60));
    fb_text_cx(cx, y, buf, f, col);
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
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

        // Wordmark: anello accent + "AeroDrag"
        fb_circle(CX - 52, 16, 5, COL_ACCENT);
        fb_circle(CX - 52, 16, 2, COL_BG);
        fb_text(CX - 42, 9, "AeroDrag", &AAF_LABEL, COL_ACCENT);
        fb_text_cx(CX, 30, "Inquadra per associare", &AAF_LBLS, COL_MUTED);

        if (!g_qr_valid) {
            fb_text_cx(CX, CY - 8, g_pairing_id, &AAF_LABEL, COL_ACCENT);
            break;
        }
        int sz  = qrcodegen_getSize(g_qr_cache);
        const int SCALE = 6;
        const int PAD   = 10;
        int qr_px  = sz * SCALE;
        int rect_w = qr_px + 2 * PAD;
        int rect_h = qr_px + 2 * PAD;
        int rx = (FB_W - rect_w) / 2;
        int ry = 52;
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

        fb_text_cx(CX, ry + rect_h + 10, g_pairing_id, &AAF_LABEL, COL_TEXTDIM);

        // Stato "In attesa…" con dot lampeggiante
        int sy = ry + rect_h + 34;
        const char *wait = "In attesa...";
        int ww = aaf_text_w(&AAF_LBLS, wait);
        int wx = CX - (ww + 14) / 2;
        if ((now_ms / 500) % 2 == 0) fb_circle(wx, sy + 5, 4, COL_ALERT);
        fb_text(wx + 12, sy, wait, &AAF_LBLS, COL_TEXTDIM);
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

        uint16_t arc_col = (pct < 0.33f) ? COL_ACCENT
                         : (pct < 0.67f) ? COL_POWER : COL_ALERT;

        // Arco gauge ruotato 180° → apertura in basso (gauge classico)
        fb_arc(CX, CY, 110, 12, -45.0f, 225.0f, COL_TRACK);
        if (valid && pct > 0.005f)
            fb_arc(CX, CY, 110, 12, -45.0f, -45.0f + pct * 270.0f, arc_col);

        // Session best — white tick on arc
        if (g_session_best_cda < 9000.0f) {
            float bp = (g_session_best_cda - 0.18f) / 0.20f;
            if (bp < 0.0f) bp = 0.0f;
            if (bp > 1.0f) bp = 1.0f;
            float ba = -45.0f + bp * 270.0f;
            fb_arc(CX, CY, 110, 12, ba - 1.5f, ba + 1.5f, rgb(255,255,255));
        }

        // Header: "CdA" a sinistra, batteria a destra
        fb_text(12, 8, "CdA", &AAF_LABEL, COL_MUTED);
        {
            uint16_t bat_col = s->battery_pct > 25 ? COL_ACCENT : COL_ALERT;
            char bat_str[6];
            snprintf(bat_str, sizeof(bat_str), "%d%%", s->battery_pct);
            fb_battery(FB_W - 26, 9, s->battery_pct, bat_col);
            fb_text_r(FB_W - 30, 8, bat_str, &AAF_LABEL, bat_col);
        }

        // Grade (hero) + valore + unità, centrati nell'anello
        const char *grade     = valid ? cda_grade(cda) : "--";
        uint16_t    grade_col = valid ? arc_col : COL_MUTED;
        fb_text_center(CX, CY - 30, grade, &AAF_GRADE, grade_col);

        if (valid) {
            char buf[8];
            snprintf(buf, sizeof(buf), "0.%03d", (int)(cda * 1000) % 1000);
            fb_text_center(CX, CY + 18, buf, &AAF_VAL, COL_TEXT);
            fb_text_cx(CX, CY + 36, "m²", &AAF_LABEL, COL_MUTED);
        } else {
            fb_text_center(CX, CY + 18, "---", &AAF_VAL, COL_MUTED);
        }

        // Chip AERO (pctAero) sopra il divisore — NON dentro l'anello
        if (valid) {
            char num[6];
            snprintf(num, sizeof(num), "%d", p->pct_aero);
            int wl = aaf_text_w(&AAF_LBLS, "AERO ");
            int wn = aaf_text_w(&AAF_MED, num);
            int wp = aaf_text_w(&AAF_LBLS, "%");
            int x  = CX - (wl + wn + wp) / 2;
            fb_text(x, CY + 64, "AERO ", &AAF_LBLS, COL_MUTED);
            x = fb_text(x + wl, CY + 60, num, &AAF_MED, COL_POWER);
            fb_text(x, CY + 64, "%", &AAF_LBLS, COL_MUTED);
        }

        // Chip "▾ NEW BEST" lampeggiante (sopra il chip AERO)
        if (now_ms < g_best_flash_end && (now_ms / 400) % 2 == 0) {
            const char *nb = "NEW BEST";
            int nbw = aaf_text_w(&AAF_LBLS, nb);
            int nx  = CX - (nbw + 12) / 2;
            // triangolino verso il basso
            for (int dy = 0; dy < 5; dy++)
                fb_hline(nx + dy, nx + 8 - dy, CY + 46 + dy, COL_POSITIVE);
            fb_text(nx + 12, CY + 44, nb, &AAF_LBLS, COL_POSITIVE);
        }

        // Divisore
        fb_hline(24, FB_W - 24, FB_H - 56, COL_PANEL);

        // Barra inferiore: speed | wind | power  (numero JBMono + unità Inter)
        int bvy = FB_H - 50, bly = FB_H - 30;
        {
            char spd[6] = "--";
            if (valid && p->v_ground_ms > 0.5f) snprintf(spd, sizeof(spd), "%d", (int)(p->v_ground_ms * 3.6f));
            fb_text_cx(52, bvy, spd, &AAF_MED, COL_TEXT);
            fb_text_cx(52, bly, "km/h", &AAF_LBLS, COL_MUTED);
        }
        {
            char wa[6] = "--";
            if (s->pitot_valid && s->pitot_pa > 0.3f) {
                float v_air = sqrtf(2.0f * s->pitot_pa / 1.225f) * 3.6f;
                snprintf(wa, sizeof(wa), "%d", (int)v_air);
            }
            fb_text_cx(CX, bvy, wa, &AAF_MED, COL_ACCENT);
            fb_text_cx(CX, bly, "wind", &AAF_LBLS, COL_MUTED);
        }
        {
            char pw[6] = "--";
            if (s->power_w > 0) snprintf(pw, sizeof(pw), "%d", s->power_w);
            fb_text_cx(FB_W - 52, bvy, pw, &AAF_MED, COL_POWER);
            fb_text_cx(FB_W - 52, bly, "W", &AAF_LBLS, COL_MUTED);
        }
        render_status_dots(s);
        break;
    }

    // ── Lap / Session timer ───────────────────────────────────────────────────
    case SCR_TIMER: {
        fb_text_cx(CX, 20, "LAP", &AAF_LABEL, COL_MUTED);
        {
            char nbuf[6];
            snprintf(nbuf, sizeof(nbuf), "%d", g_lap_num_display);
            fb_text_center(CX, 66, nbuf, &AAF_HERO, COL_ACCENT);
        }
        {
            uint16_t lap_col = (g_lap_elapsed_s < 60) ? COL_ACCENT : COL_POWER;
            fb_time_cx(CX, 110, g_lap_elapsed_s, lap_col, &AAF_VAL);
        }
        fb_hline(30, FB_W - 30, 162, COL_PANEL);
        fb_text_cx(CX, 172, "TOT", &AAF_LBLS, COL_MUTED);
        fb_time_cx(CX, 188, g_session_elapsed_s, COL_TEXT, &AAF_MED);
        render_status_dots(s);
        break;
    }

    // ── Speed ─────────────────────────────────────────────────────────────────
    case SCR_SPEED: {
        fb_text(12, 8, "SPEED", &AAF_LABEL, COL_MUTED);

        float spd_kmh = s->gps_valid    ? s->speed_ms * 3.6f
                      : (p && p->valid) ? p->v_ground_ms * 3.6f
                      : 0.0f;
        char spd_buf[6];
        snprintf(spd_buf, sizeof(spd_buf), "%d", (int)spd_kmh);
        fb_text_center(CX, CY - 26, spd_buf, &AAF_HERO, COL_SPEED);
        fb_text_cx(CX, CY + 6, "km/h", &AAF_LABEL, COL_MUTED);

        if (s->pitot_valid && s->pitot_pa > 0.3f) {
            float v_air  = sqrtf(2.0f * s->pitot_pa / 1.225f) * 3.6f;
            float delta  = spd_kmh - v_air;
            char  air[6], wind[6];
            snprintf(air,  sizeof(air),  "%d",  (int)v_air);
            snprintf(wind, sizeof(wind), "%+d", (int)delta);
            uint16_t wcol = (delta >  2.0f) ? COL_ALERT
                          : (delta < -2.0f) ? COL_ACCENT : COL_MUTED;
            // "Air  NN km/h" (parola Inter + numero JBMono + unità Inter)
            int wa = aaf_text_w(&AAF_LBLS, "Air ") + aaf_text_w(&AAF_NUMS, air) + aaf_text_w(&AAF_LBLS, " km/h");
            int ax = CX - wa / 2;
            ax = fb_text(ax, CY + 40, "Air ", &AAF_LBLS, COL_MUTED);
            ax = fb_text(ax, CY + 38, air, &AAF_NUMS, COL_ACCENT);
            fb_text(ax, CY + 40, " km/h", &AAF_LBLS, COL_MUTED);
            // "Wind ±NN km/h"
            int ww = aaf_text_w(&AAF_LBLS, "Wind ") + aaf_text_w(&AAF_NUMS, wind) + aaf_text_w(&AAF_LBLS, " km/h");
            int wx = CX - ww / 2;
            wx = fb_text(wx, CY + 60, "Wind ", &AAF_LBLS, COL_MUTED);
            wx = fb_text(wx, CY + 58, wind, &AAF_NUMS, wcol);
            fb_text(wx, CY + 60, " km/h", &AAF_LBLS, COL_MUTED);
        }

        if (s->hr_bpm > 0) {
            char hr[6];
            snprintf(hr, sizeof(hr), "%d", s->hr_bpm);
            fb_text_r(FB_W - 10, 8, hr, &AAF_MED, COL_ALERT);
            fb_text_r(FB_W - 10, 34, "bpm", &AAF_LBLS, COL_MUTED);
        }

        if (s->power_w > 0) {
            char pw[8];
            snprintf(pw, sizeof(pw), "%d", s->power_w);
            uint16_t pcol = power_zone_col(s->power_w, g_rider_ftp_w);
            int pwn = aaf_text_w(&AAF_MED, pw), pwu = aaf_text_w(&AAF_LABEL, " W");
            int px  = CX - (pwn + pwu) / 2;
            px = fb_text(px, FB_H - 52, pw, &AAF_MED, pcol);
            fb_text(px, FB_H - 48, " W", &AAF_LABEL, COL_MUTED);
        }
        render_status_dots(s);
        break;
    }

    // ── Power / Energy split ──────────────────────────────────────────────────
    case SCR_POWER: {
        char buf[16];
        uint16_t    zone_col  = power_zone_col(s->power_w, g_rider_ftp_w);
        const char *zone_name = power_zone_name(s->power_w, g_rider_ftp_w);

        // "238 W" hero (numero JBMono + unità Inter)
        snprintf(buf, sizeof(buf), "%d", s->power_w);
        {
            int pn = aaf_text_w(&AAF_HERO, buf), pu = aaf_text_w(&AAF_LABEL, " W");
            int px = CX - (pn + pu) / 2;
            px = fb_text(px, 12, buf, &AAF_HERO, zone_col);
            fb_text(px, 30, " W", &AAF_LABEL, COL_MUTED);
        }
        fb_text_cx(CX, 64, zone_name, &AAF_LABEL, zone_col);

        if (g_rider_mass_kg > 0.0f && s->power_w > 0) {
            snprintf(buf, sizeof(buf), "%.1f", s->power_w / g_rider_mass_kg);
            int wn = aaf_text_w(&AAF_MED, buf), wu = aaf_text_w(&AAF_LBLS, " W/kg");
            int wx = CX - (wn + wu) / 2;
            wx = fb_text(wx, 84, buf, &AAF_MED, COL_TEXT);
            fb_text(wx, 90, " W/kg", &AAF_LBLS, COL_MUTED);
        }

        if (p && p->valid && s->power_w > 0) {
            uint8_t pa  = p->pct_aero;
            float   pr_f = (p->p_rolling_w / (float)s->power_w) * 100.0f;
            uint8_t pr  = (pr_f > 100.0f) ? 100 : (pr_f < 0.0f) ? 0 : (uint8_t)pr_f;
            uint8_t po  = (pa + pr < 100) ? (100 - pa - pr) : 0;

            const int BH = 96, BY = 128, BW = 34, GAP = 18;
            int bx = (FB_W - (3*BW + 2*GAP)) / 2;

            struct { uint8_t pct; uint16_t col; const char *lbl; } bars[3] = {
                {pa, COL_POWER, "Aero"},
                {pr, COL_ACCENT,  "Roll"},
                {po, COL_MUTED, "Other"},
            };
            for (int b = 0; b < 3; b++) {
                int filled = BH * bars[b].pct / 100;
                for (int y = BY; y < BY + BH; y++)
                    for (int x = bx; x < bx + BW; x++)
                        if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y) = COL_PANEL;
                for (int y = BY + BH - filled; y < BY + BH; y++)
                    for (int x = bx; x < bx + BW; x++)
                        if (x>=0&&x<FB_W&&y>=0&&y<FB_H) PX(x,y) = bars[b].col;

                fb_text_cx(bx + BW / 2, BY - 16, bars[b].lbl, &AAF_LBLS, COL_MUTED);
                snprintf(buf, sizeof(buf), "%d%%", bars[b].pct);
                fb_text_cx(bx + BW / 2, BY + BH + 6, buf, &AAF_NUMS, bars[b].col);

                bx += BW + GAP;
            }
        } else {
            fb_text_center(CX, CY + 20, "---", &AAF_VAL, COL_MUTED);
        }
        render_status_dots(s);
        break;
    }

    // ── Status / Vitals 2×2 ───────────────────────────────────────────────────
    case SCR_STATUS: {
        char buf[20];

        // Grid dividers
        fb_hline(0, FB_W - 1, 159, COL_PANEL);
        fb_hline(0, FB_W - 1, 160, COL_PANEL);
        for (int y = 0; y < FB_H; y++)
            if (y >= 0 && y < FB_H) { PX(119,y) = COL_PANEL; PX(120,y) = COL_PANEL; }

        // TL — CdA
        {
            bool     cda_valid = (p && p->valid);
            float    cda_val   = cda_valid ? p->CdA : 0.0f;
            float    cpct = cda_valid ? (cda_val - 0.18f) / 0.20f : 0.0f;
            if (cpct < 0.0f) cpct = 0.0f;
            if (cpct > 1.0f) cpct = 1.0f;
            uint16_t cda_col = (cpct < 0.33f) ? COL_ACCENT
                             : (cpct < 0.67f) ? COL_POWER : COL_ALERT;
            if (!cda_valid) cda_col = COL_MUTED;

            fb_text(12, 12, "CdA", &AAF_LBLS, COL_MUTED);
            if (cda_valid) {
                snprintf(buf, sizeof(buf), ".%03d", (int)(cda_val * 1000) % 1000);
                fb_text(12, 36, buf, &AAF_MED, cda_col);
                fb_text(12, 96, cda_grade(cda_val), &AAF_MED, cda_col);
            } else {
                fb_text(12, 36, "---", &AAF_MED, COL_MUTED);
            }
        }

        // TR — Heart Rate
        fb_text(132, 12, "HR", &AAF_LBLS, COL_MUTED);
        {
            uint16_t hr_col = s->hr_bpm > 0 ? COL_ALERT : COL_MUTED;
            snprintf(buf, sizeof(buf), "%d", s->hr_bpm > 0 ? s->hr_bpm : 0);
            fb_text(132, 36, buf, &AAF_MED, hr_col);
            fb_text(132, 96, "bpm", &AAF_LBLS, COL_MUTED);
            fb_circle(208, 100, 5, hr_col);
        }

        // BL — Cadence
        fb_text(12, 174, "Cadence", &AAF_LBLS, COL_MUTED);
        {
            uint16_t cad_col = s->cadence_rpm > 0 ? COL_POWER : COL_MUTED;
            snprintf(buf, sizeof(buf), "%d", s->cadence_rpm > 0 ? s->cadence_rpm : 0);
            fb_text(12, 198, buf, &AAF_MED, cad_col);
            fb_text(12, 258, "rpm", &AAF_LBLS, COL_MUTED);
        }

        // BR — Power
        {
            uint16_t pwr_col = power_zone_col(s->power_w, g_rider_ftp_w);
            fb_text(132, 174, "Power", &AAF_LBLS, COL_MUTED);
            snprintf(buf, sizeof(buf), "%d", s->power_w > 0 ? s->power_w : 0);
            fb_text(132, 198, buf, &AAF_MED, pwr_col);
            fb_text(132, 258, "W", &AAF_LBLS, COL_MUTED);
            if (s->power_w > 0) {
                const char *zn = power_zone_name(s->power_w, g_rider_ftp_w);
                fb_text(132, 278, zn, &AAF_LBLS, pwr_col);
            }
            fb_circle(208, 282, 5, s->ant_valid ? COL_ACCENT : COL_MUTED);
        }
        break;
    }

    default: break;
    }

    // ── Toast overlay ─────────────────────────────────────────────────────────
    {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
        if (g_toast_end_ms > now_ms && g_toast_msg[0]) {
            int msg_w = aaf_text_w(&AAF_LABEL, g_toast_msg);
            int box_w = msg_w + 24, box_h = 30;
            int bx = (FB_W - box_w) / 2, by = (FB_H - box_h) / 2;
            for (int yy = by - 2; yy < by + box_h + 2; yy++)
                for (int xx = bx - 2; xx < bx + box_w + 2; xx++)
                    if (xx >= 0 && xx < FB_W && yy >= 0 && yy < FB_H)
                        PX(xx, yy) = COL_ACCENT;
            for (int yy = by; yy < by + box_h; yy++)
                for (int xx = bx; xx < bx + box_w; xx++)
                    if (xx >= 0 && xx < FB_W && yy >= 0 && yy < FB_H)
                        PX(xx, yy) = COL_BG;
            fb_text_cx(FB_W / 2, by + 9, g_toast_msg, &AAF_LABEL, COL_ACCENT);
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
