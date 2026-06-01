#pragma once

// ─── Waveshare ESP32-S3-Touch-LCD-2.8 pin map ────────────────────────────────
// Source: Waveshare schematic (confirmed GPIO assignments)

// ── Display (ST7789T3, 4-wire SPI, 240×320 portrait) ─────────────────────────
#define PIN_LCD_SCLK     40    // SPI CLK
#define PIN_LCD_MOSI     45    // SPI MOSI
#define PIN_LCD_DC       41    // Data/Command
#define PIN_LCD_CS       42    // Chip Select
#define PIN_LCD_RST      39    // Hardware reset
#define PIN_LCD_BL        5    // Backlight PWM
#define LCD_W           240    // portrait width
#define LCD_H           320    // portrait height
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_SPI_CLK_HZ  (40 * 1000 * 1000)   // ST7789T3 max 40MHz

// ── Touch (CST816S, I2C0) ────────────────────────────────────────────────────
#define PIN_TOUCH_SDA     1    // TP_SDA (confirmed schematic: NLIO1 NLTP0SDA)
#define PIN_TOUCH_SCL     3    // TP_SCL
#define PIN_TOUCH_INT     4    // TP_INT
#define PIN_TOUCH_RST     2    // TP_RST
#define TOUCH_I2C_PORT  I2C_NUM_0
#define TOUCH_I2C_ADDR  0x15

// ── IMU QMI8658C (I2C0, shares bus with touch on GPIO1/3) ────────────────────
#define PIN_IMU_SDA       1    // shared with touch on I2C0
#define PIN_IMU_SCL       3
#define PIN_IMU_INT1      13
#define PIN_IMU_INT2      12
#define IMU_I2C_PORT    I2C_NUM_0
#define IMU_I2C_ADDR    0x6B   // AD0 high on Waveshare board

// ── Pitot sensor SDP810 (I2C1 — separate bus, external wiring) ───────────────
#define PIN_PITOT_SDA    15    // available on header
#define PIN_PITOT_SCL    18    // hardware wiring on board header
#define PITOT_I2C_PORT  I2C_NUM_1
#define PITOT_I2C_ADDR  0x25   // SDP810-500Pa default address

// ── Battery ADC ───────────────────────────────────────────────────────────────
// GPIO8 → battery voltage via 200K+100K divider (ratio = 3×)
// Confirmed from schematic: NLIO8 NLBAT0ADC
// Formula: Vbat = 3.3 / 4096 * 3 * adc_raw
#define PIN_BAT_ADC       8    // ADC1_CH7
#define BAT_ADC_CHANNEL   ADC_CHANNEL_7
#define BAT_ADC_RATIO   3.0f   // voltage divider multiplier
#define BAT_FULL_MV    4200
#define BAT_EMPTY_MV   3300
#define PIN_PWR_HOLD     7    // BAT control — keep HIGH to stay powered

// ── Buttons ───────────────────────────────────────────────────────────────────
#define PIN_BTN_BOOT      0    // BOOT button (GPIO0, active low)
#define PIN_BTN_USER      0    // use BOOT for calibration / screen cycle
// Optional external buttons (solder to header pads, internal pull-up, active low)
#define PIN_BTN_SCREEN   17    // cycle screens         (= PIN_MOSFET1 pad, GPIO17)
#define PIN_BTN_LAP      16    // new lap / start+reset (free header pin, GPIO16)

// ── MOSFET pads (optional solder points) ─────────────────────────────────────
#define PIN_MOSFET1      17    // used by PIN_BTN_SCREEN
#define PIN_MOSFET2      18    // used by PIN_PITOT_SCL

// ── I2C speeds ────────────────────────────────────────────────────────────────
#define I2C0_SPEED_HZ   (400 * 1000)   // fast mode (touch + IMU)
#define I2C1_SPEED_HZ   (100 * 1000)   // standard mode (SDP810 max)
