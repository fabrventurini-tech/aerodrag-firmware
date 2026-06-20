#pragma once
#include <math.h>
#include "aerodrag_types.h"

// ─── Physical constants ───────────────────────────────────────────────────────
#define GRAVITY_MS2     9.80665f
#define CRR             0.0040f     // rolling resistance coefficient
#define MECH_EFF        0.975f      // drivetrain efficiency
#define RHO_STD         1.225f      // kg/m³ standard air density
#define MASS_KG_DEFAULT 80.0f

// ─── Air density  ρ = f(T, RH, altitude) — ISO 2533 simplified ───────────────
static inline float physics_calc_rho(float temp_c, float humidity_pct, float alt_m)
{
    // Bug fix: clamp inputs to prevent NaN/negative from Magnus formula
    // (denominator temp_c+237.3 approaches 0 at -237°C; clamp at -40°C)
    if (temp_c      < -40.0f)  temp_c      = -40.0f;
    if (temp_c      > 60.0f)   temp_c      =  60.0f;
    if (humidity_pct < 0.0f)   humidity_pct =  0.0f;
    if (humidity_pct > 100.0f) humidity_pct = 100.0f;
    if (alt_m       < 0.0f)    alt_m        =  0.0f;
    if (alt_m       > 5000.0f) alt_m        = 5000.0f;

    const float T  = temp_c + 273.15f;
    const float p0 = 101325.0f;
    float p  = p0 * expf(-0.0289644f * GRAVITY_MS2 * alt_m / (8.31447f * T));
    float es = 610.78f * expf(17.27f * temp_c / (temp_c + 237.3f));
    float pv = (humidity_pct / 100.0f) * es;
    float rho = (p - 0.378f * pv) / (287.058f * T);
    // Final sanity — return standard density if result is out of range
    return (rho > 0.8f && rho < 1.5f) ? rho : RHO_STD;
}

// ─── CdA calculation ──────────────────────────────────────────────────────────
// Returns physics output given raw sensor data and calibration.
static inline aerodrag_physics_t physics_compute(
    const aerodrag_sensors_t *s,
    const aerodrag_cal_t     *cal)
{
    aerodrag_physics_t out = {0};

    if (!s->pitot_valid || !s->gps_valid) {
        out.valid = false;
        return out;
    }

    const float mass = cal ? cal->mass_kg : MASS_KG_DEFAULT;

    // Air density correction
    float rho = physics_calc_rho(s->temp_c, s->humidity_pct, s->altitude_m);
    if (rho < 0.8f || rho > 1.5f) rho = RHO_STD;  // sanity clamp (coerente con physics_calc_rho)

    // Pitot: v_air = sqrt(2 * ΔP / ρ)
    float dp = s->pitot_pa;
    if (cal) dp -= cal->pitot_offset_pa;
    if (dp < 0.0f) dp = 0.0f;

    float v_air = sqrtf(2.0f * dp / rho);
    float wind  = v_air - s->speed_ms;
    if (wind < 0.0f) wind = 0.0f;

    // Road gradient from GPS altitude derivative (smoothed externally)
    // Using pitch from IMU as proxy when GPS derivative not available
    float pitch_rad = (s->pitch_deg - (cal ? cal->imu_pitch_offset : 0.0f))
                      * (3.14159265f / 180.0f);
    float slope = sinf(pitch_rad);  // approximation valid for small angles

    // Power components
    float crr_val   = (cal && cal->crr > 0.0f) ? cal->crr : CRR;
    float p_rolling = crr_val * mass * GRAVITY_MS2 * s->speed_ms;
    float p_gravity = mass * GRAVITY_MS2 * slope * s->speed_ms;
    float p_mech    = (float)s->power_w * (1.0f - MECH_EFF);
    float p_aero    = (float)s->power_w - p_rolling - p_gravity - p_mech;

    if (p_aero < 0.0f) p_aero = 0.0f;

    // CdA = P_aero / (½ · ρ · v_air³)
    float v3 = v_air * v_air * v_air;
    float CdA = 0.0f;
    if (v3 > 1.0f && s->power_w > 20) {
        CdA = p_aero / (0.5f * rho * v3);
    }

    // Sanity clamp
    if (CdA < 0.10f || CdA > 0.60f) {
        out.valid = false;
        return out;
    }

    out.CdA         = CdA;
    out.v_air_ms    = v_air;
    out.v_ground_ms = s->speed_ms;
    out.wind_ms     = wind;
    out.rho         = rho;
    out.p_aero_w    = p_aero;
    out.p_rolling_w = p_rolling;
    out.p_gravity_w = p_gravity;
    float pct_f     = (s->power_w > 0) ? (p_aero / (float)s->power_w * 100.0f) : 0.0f;
    if (pct_f > 100.0f) pct_f = 100.0f;
    out.pct_aero    = (uint8_t)pct_f;
    out.valid       = true;
    return out;
}

// ─── Rolling average (exponential moving average) ────────────────────────────
// α = 1/N approximation. Call every sample, returns smoothed value.
static inline float ema_update(float prev, float new_val, float alpha)
{
    return alpha * new_val + (1.0f - alpha) * prev;
}

#define EMA_ALPHA_30S  (1.0f / 30.0f)   // 30 campioni: ~3 s a 10 Hz (task_pitot_imu)
#define EMA_ALPHA_5S   (1.0f / 5.0f)
