#pragma once

// ─── Waveshare ESP32-S3-LCD-1.28 / ESP32-S3-Touch-LCD-1.28 pin map ───────────
// Source: Waveshare Wiki + schematic (GPIO table verified)
// Both variants (with/without CNC case) share the same pinout.

// ── Display (GC9A01A, 4-wire SPI) ────────────────────────────────────────────
#define PIN_LCD_SCLK     40    // SPI CLK
#define PIN_LCD_MOSI     45    // SPI MOSI
#define PIN_LCD_DC       41    // Data/Command
#define PIN_LCD_CS       42    // Chip Select
#define PIN_LCD_RST      39    // Hardware reset
#define PIN_LCD_BL        5    // Backlight PWM (GPIO2)
#define LCD_W           320    // ST7789 resolution
#define LCD_H           240
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_SPI_CLK_HZ  (40 * 1000 * 1000)   // ST7789 max 40MHz

// ── Touch (CST816S, I2C0) ────────────────────────────────────────────────────
#define PIN_TOUCH_SDA     1    // shared I2C bus with IMU
#define PIN_TOUCH_SCL     3
#define PIN_TOUCH_INT     4    // TP_INT
#define PIN_TOUCH_RST     2
#define TOUCH_I2C_PORT  I2C_NUM_0
#define TOUCH_I2C_ADDR  0x15

// ── IMU QMI8658 (I2C0, shares bus with touch) ────────────────────────────────
// INT1 = GPIO4, INT2 = GPIO3 (from wiki GPIO table)
#define PIN_IMU_SDA       11
#define PIN_IMU_SCL       10
#define PIN_IMU_INT1      13
#define PIN_IMU_INT2      12
#define IMU_I2C_PORT    I2C_NUM_0
#define IMU_I2C_ADDR    0x6B   // AD0 high on Waveshare board

// ── Pitot sensor SDP810 (I2C1 — separate bus, external wiring) ───────────────
// Use free GPIOs from the 1.27mm header (not used by on-board peripherals)
#define PIN_PITOT_SDA    15    // available on header
#define PIN_PITOT_SCL    18    // available on header
#define PITOT_I2C_PORT  I2C_NUM_1
#define PITOT_I2C_ADDR  0x25   // SDP810-500Pa default address

// ── GPS u-blox M10S — RIMOSSO ────────────────────────────────────────────────
// La velocità GPS è ora fornita dal dongle nRF52840 via ANT+ Speed (profilo 124).
// Il modulo SAM-M10Q non è più necessario — risparmio ~€40 sul BOM.
// I pin GPIO43/44 (UART1) sono ora liberi per debug seriale.

// ── ANT+ bridge (UART2) ───────────────────────────────────────────────────────
// Riceve frame v2 (12 byte) dal dongle nRF52840:
// [0xAA][0x09][pw_lo][pw_hi][cad][hr][status][spd_lo][spd_hi][dist_lo][dist_hi][xor]
#define PIN_ANT_TX       17
#define PIN_ANT_RX       18
#define ANT_UART_PORT   UART_NUM_2
#define ANT_BAUD        57600

// ── Battery ADC ───────────────────────────────────────────────────────────────
// GPIO1 → battery voltage via 200K+100K divider (ratio = 3×)
// Formula: Vbat = 3.3 / 4096 * 3 * adc_raw
#define PIN_BAT_ADC       1    // ADC1_CH0
#define BAT_ADC_RATIO   3.0f   // voltage divider multiplier
#define BAT_FULL_MV    4200
#define BAT_EMPTY_MV   3300
#define PIN_PWR_HOLD     7    // BAT control — keep HIGH to stay powered

// ── Buttons ───────────────────────────────────────────────────────────────────
#define PIN_BTN_BOOT      0    // BOOT button (GPIO0, active low)
// No dedicated user button on 1.28 board — use BOOT for calibration
#define PIN_BTN_USER      0

// ── MOSFET pads (optional — solder motor/buzzer) ─────────────────────────────
#define PIN_MOSFET1       4    // GPIO4 — around battery holder
#define PIN_MOSFET2       5    // GPIO5

// ── I2C speeds ────────────────────────────────────────────────────────────────
#define I2C0_SPEED_HZ   (400 * 1000)   // fast mode (touch + IMU)
#define I2C1_SPEED_HZ   (100 * 1000)   // standard mode (SDP810 max)
