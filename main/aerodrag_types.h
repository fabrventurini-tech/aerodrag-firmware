#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/time.h>

// ─── Orologio oggettivo (UTC) — contract v0.3.0 ──────────────────────────────
// Sorgente unica: l'orologio di sistema (settimeofday/gettimeofday). L'app lo
// imposta alla connessione via BLE TIME (0xaa10); i frame portano `tUtc`.
// Convenzione condivisa con il contratto: tUtc == 0 ⇒ orologio NON impostato
// (epoch < 2020-01-01). Nessuno stato duplicato altrove.
#define AERODRAG_EPOCH_MIN_MS  1577836800000ULL   // 2020-01-01T00:00:00Z
// Bound superiore (audit v0.3.1, FW-1): rifiuta un clock telefono assurdo.
// time_t è 64-bit → niente overflow; è solo difesa di sanità.
#define AERODRAG_EPOCH_MAX_MS  4102444800000ULL   // 2100-01-01T00:00:00Z

static inline void aerodrag_time_set_epoch_ms(uint64_t ms)
{
    struct timeval tv;
    tv.tv_sec  = (time_t)(ms / 1000ULL);
    tv.tv_usec = (suseconds_t)((ms % 1000ULL) * 1000ULL);
    settimeofday(&tv, NULL);
}

// Epoch UTC in ms, oppure 0 se l'orologio non è ancora stato impostato.
static inline uint64_t aerodrag_time_get_epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t ms = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
    return (ms < AERODRAG_EPOCH_MIN_MS) ? 0ULL : ms;
}

// ─── Device identity ─────────────────────────────────────────────────────────
// Ogni device ha un ID univoco derivato dal MAC address dell'ESP32.
// Il nome atleta viene configurato via BLE dall'app e salvato in NVS.
#define DEVICE_ID_LEN   18   // "AA:BB:CC:DD:EE:FF\0"
#define ATHLETE_NAME_LEN 32  // nome atleta max 31 char + null

typedef struct {
    char device_id[DEVICE_ID_LEN];       // MAC address come stringa
    char athlete_name[ATHLETE_NAME_LEN]; // configurato dall'app
} device_identity_t;

// ─── Raw sensor data (filled by individual drivers) ──────────────────────────
typedef struct {
    // Pitot (SDP810)
    float   pitot_pa;
    float   static_pa;
    bool    pitot_valid;

    // IMU QMI8658
    float   pitch_deg;
    float   roll_deg;
    float   yaw_rate_dps;
    bool    imu_valid;

    // Speed & Distance — from ANT+ dongle (replaces GPS)
    float   speed_ms;        // ground speed [m/s]  (from ANT+ Speed profile 124)
    float   altitude_m;      // set to 0 — no GPS module
    float   lat;             // set to 0 — no GPS module
    float   lon;             // set to 0 — no GPS module
    uint32_t distance_m;     // cumulative distance [m]
    uint8_t gps_fix;         // always 2 (ANT+ speed considered valid)
    bool    gps_valid;       // true when ANT+ speed valid

    // ANT+ bridge (power + cadence + HR + speed + LAP)
    uint16_t power_w;
    uint8_t  cadence_rpm;
    uint8_t  hr_bpm;
    bool     ant_valid;

    // Lap event — set for ONE physics cycle when dongle sends lap_event
    bool     lap_event;

    // Environment (from IMU temperature)
    float   temp_c;
    float   humidity_pct;    // fixed default — no hygro sensor

    // Battery
    uint8_t battery_pct;
    float   battery_mv;
} aerodrag_sensors_t;

// ─── Computed physics output ──────────────────────────────────────────────────
typedef struct {
    float   CdA;
    float   v_air_ms;
    float   v_ground_ms;
    float   wind_ms;
    float   rho;
    float   p_aero_w;
    float   p_rolling_w;
    float   p_gravity_w;
    uint8_t pct_aero;
    bool    valid;
} aerodrag_physics_t;

// ─── Lap record ───────────────────────────────────────────────────────────────
#define MAX_LAPS 50

typedef struct {
    uint32_t start_ts;       // Unix epoch start
    uint32_t duration_s;
    float    avg_cda;
    float    best_cda;
    uint16_t avg_power_w;
    float    avg_speed_kmh;
    uint32_t distance_m;
    uint8_t  lap_num;
} lap_record_t;

// ─── System state ─────────────────────────────────────────────────────────────
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_SCANNING_BLE,
    STATE_CONNECTED,
    STATE_RECORDING,
    STATE_CALIBRATING,
    STATE_LOW_BATTERY,
    STATE_SLEEP,
} aerodrag_state_t;

// ─── Calibration data ────────────────────────────────────────────────────────
typedef struct {
    float   pitot_offset_pa;
    float   imu_pitch_offset;
    float   mass_kg;          // kg  — set by app via BLE 0xaa08
    float   crr;              // rolling resistance coeff — set by app via BLE 0xaa08
    float   cda_target;
    float   wheel_circ_m;     // circonferenza ruota [m] — set by app via BLE 0xaa08 (v0.2.0, app-authoritative)
    uint32_t crc;
} aerodrag_cal_t;

// ─── Session record ───────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint32_t timestamp_s;
    uint16_t power_w;
    uint16_t speed_kmh_x10;
    uint16_t cda_x1000;
    int8_t   pitch_deg;
    uint8_t  battery_pct;
    uint8_t  lap_num;         // current lap number at this point
} session_point_t;
