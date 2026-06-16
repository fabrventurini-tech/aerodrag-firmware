// ─── Forward declaration — usata da chr_access_cb ──────────────────────────────────────────
static aerodrag_sensors_t  *g_sensors_ptr = NULL;
static SemaphoreHandle_t    g_ble_sensors_mutex = NULL;
static aerodrag_cal_t      *g_cal_ptr = NULL;


#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "aerodrag_types.h"
#include "ble_sensors.h"
#include "version.h"
#include "ota_update.h"
#include <string.h>

// ─── Service and characteristic UUIDs ────────────────────────────────────────
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x00, 0xaa, 0x00, 0x00);

static const ble_uuid128_t CHR_PITOT = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x01, 0xaa, 0x00, 0x00);

static const ble_uuid128_t CHR_IMU = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x02, 0xaa, 0x00, 0x00);

static const ble_uuid128_t CHR_ENV = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x03, 0xaa, 0x00, 0x00);

static const ble_uuid128_t CHR_ANT = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x04, 0xaa, 0x00, 0x00);

static const ble_uuid128_t CHR_IDENTITY = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x05, 0xaa, 0x00, 0x00);

// VERSION (0xaa06): stringa versione firmware — READ only
static const ble_uuid128_t CHR_VERSION = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x06, 0xaa, 0x00, 0x00);

// OTA_URL (0xaa07): scrivere URL del .bin per avviare OTA — WRITE only
static const ble_uuid128_t CHR_OTA_URL = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x07, 0xaa, 0x00, 0x00);

// CONFIG (0xaa08): massKg [float32] + crr [float32] — WRITE WITH RESPONSE
static const ble_uuid128_t CHR_CONFIG = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x08, 0xaa, 0x00, 0x00);

// PHYSICS (0xaa09): output physics_compute, 28 bytes, NOTIFY 10 Hz
static const ble_uuid128_t CHR_PHYSICS = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x09, 0xaa, 0x00, 0x00);

// BATTERY (0xaa0a): percentuale batteria [uint8 0-100] — NOTIFY only, ~0.1 Hz
static const ble_uuid128_t CHR_BATTERY = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x0a, 0xaa, 0x00, 0x00);

static uint16_t g_chr_identity_h = 0;
static uint16_t g_chr_version_h  = 0;
static uint16_t g_chr_ota_h      = 0;
static uint16_t g_chr_config_h   = 0;
static uint16_t g_chr_physics_h  = 0;

// Notify slot — 32 bytes to accommodate physics payload (28 B)
typedef struct { uint8_t data[32]; uint8_t len; bool pending; } notify_slot_t;

static notify_slot_t g_slot_pitot   = {0};
static notify_slot_t g_slot_imu     = {0};
static notify_slot_t g_slot_env     = {0};
static notify_slot_t g_slot_ant     = {0};
static notify_slot_t g_slot_bat     = {0};
static notify_slot_t g_slot_physics = {0};

static struct ble_npl_callout g_callout_pitot;
static struct ble_npl_callout g_callout_imu;
static struct ble_npl_callout g_callout_env;
static struct ble_npl_callout g_callout_ant;
static struct ble_npl_callout g_callout_bat;
static struct ble_npl_callout g_callout_physics;
static struct ble_npl_mutex   g_notify_mutex;

static void callout_pitot  (struct ble_npl_event *ev);
static void callout_imu    (struct ble_npl_event *ev);
static void callout_env    (struct ble_npl_event *ev);
static void callout_ant    (struct ble_npl_event *ev);
static void callout_bat    (struct ble_npl_event *ev);
static void callout_physics(struct ble_npl_event *ev);

// ─── BLE state ────────────────────────────────────────────────────────────────
static uint16_t g_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_chr_pitot_h    = 0;
static uint16_t g_chr_imu_h      = 0;
static uint16_t g_chr_env_h      = 0;
static uint16_t g_chr_ant_h      = 0;
static uint16_t g_chr_battery_h  = 0;
static bool     g_notify_pitot   = false;
static bool     g_notify_imu     = false;
static bool     g_notify_env     = false;
static bool     g_notify_ant     = false;
static bool     g_notify_battery = false;
static bool     g_notify_physics = false;

static const char *DEVICE_NAME = "AeroDrag Pro";

static int ble_gap_event(struct ble_gap_event *event, void *arg);

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields = {0};
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name                  = (uint8_t *)DEVICE_NAME;
    fields.name_len              = strlen(DEVICE_NAME);
    fields.name_is_complete      = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;

    ble_gap_adv_set_fields(&fields);
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, ble_gap_event, NULL);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            struct ble_gap_upd_params params = {
                .itvl_min            = 16,
                .itvl_max            = 24,
                .latency             = 0,
                .supervision_timeout = 400,
            };
            ble_gap_update_params(g_conn_handle, &params);
        } else {
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        g_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
        g_notify_pitot = g_notify_imu = g_notify_env = g_notify_ant =
        g_notify_battery = g_notify_physics = false;
        ble_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == g_chr_pitot_h)
            g_notify_pitot = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle == g_chr_imu_h)
            g_notify_imu = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle == g_chr_env_h)
            g_notify_env = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle == g_chr_ant_h)
            g_notify_ant = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle == g_chr_battery_h)
            g_notify_battery = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle == g_chr_physics_h)
            g_notify_physics = event->subscribe.cur_notify;
        break;
    default:
        break;
    }
    return 0;
}

#define BLE_SENSORS_LOCK()   if(g_ble_sensors_mutex) xSemaphoreTake(g_ble_sensors_mutex, portMAX_DELAY)
#define BLE_SENSORS_UNLOCK() if(g_ble_sensors_mutex) xSemaphoreGive(g_ble_sensors_mutex)

static int chr_access_cb(uint16_t conn_h, uint16_t attr_h,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;
    if (!g_sensors_ptr) return BLE_ATT_ERR_UNLIKELY;

    BLE_SENSORS_LOCK();
    aerodrag_sensors_t s = *g_sensors_ptr;
    BLE_SENSORS_UNLOCK();

    if (attr_h == g_chr_pitot_h) {
        float vals[2] = { s.pitot_pa, s.static_pa };
        os_mbuf_append(ctxt->om, vals, sizeof(vals));
    } else if (attr_h == g_chr_imu_h) {
        float vals[2] = { s.pitch_deg, s.roll_deg };
        os_mbuf_append(ctxt->om, vals, sizeof(vals));
    } else if (attr_h == g_chr_env_h) {
        float vals[4] = { s.temp_c, s.humidity_pct, s.altitude_m, s.speed_ms };
        os_mbuf_append(ctxt->om, vals, sizeof(vals));
    } else if (attr_h == g_chr_ant_h) {
        uint8_t vals[4];
        memcpy(vals, &s.power_w, 2);
        vals[2] = s.cadence_rpm;
        vals[3] = s.hr_bpm;
        os_mbuf_append(ctxt->om, vals, sizeof(vals));
    }
    return 0;
}

extern device_identity_t g_identity;
extern void identity_set_athlete_name(const char *name);

static int identity_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t buf[DEVICE_ID_LEN + ATHLETE_NAME_LEN] = {0};
        memcpy(buf, g_identity.device_id, DEVICE_ID_LEN);
        memcpy(buf + DEVICE_ID_LEN, g_identity.athlete_name, ATHLETE_NAME_LEN);
        os_mbuf_append(ctxt->om, buf, sizeof(buf));
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > 0 && len < ATHLETE_NAME_LEN) {
            char name[ATHLETE_NAME_LEN] = {0};
            os_mbuf_copydata(ctxt->om, 0, len, name);
            name[len] = '\0';
            for (uint16_t i = 0; i < len; i++) {
                if (name[i] == '"' || name[i] == '\\' || (unsigned char)name[i] < 0x20)
                    name[i] = '_';
            }
            identity_set_athlete_name(name);
        }
    }
    return 0;
}

// ─── VERSION + OTA characteristic callbacks ───────────────────────────────────
static int version_ota_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)arg;
    if (attr_handle == g_chr_version_h) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            const char *ver = FW_VERSION_FULL;
            os_mbuf_append(ctxt->om, ver, strlen(ver) + 1);
        }
    } else if (attr_handle == g_chr_ota_h) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len > 0 && len < OTA_URL_MAXLEN) {
                char url[OTA_URL_MAXLEN] = {0};
                os_mbuf_copydata(ctxt->om, 0, len, url);
                url[len] = '\0';
                ESP_LOGI("ble_ota", "OTA URL via BLE: %s", url);
                ota_start(url);
            }
        }
    }
    return 0;
}

// ─── CONFIG (0xaa08) callback — READ + WRITE WITH RESPONSE ───────────────────
// Contract v0.1.0 — Payload 12 bytes little-endian:
//   float32 massKg [0..3] + float32 crr [4..7] + float32 wheelCircM [8..11]
// Backward compatible: a 8-byte WRITE updates mass+crr and leaves wheelCircM.
extern esp_err_t cal_save(const aerodrag_cal_t *cal);

static int config_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (!g_cal_ptr) return BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // READ: device is source of truth for wheel circumference
        float vals[3] = { g_cal_ptr->mass_kg, g_cal_ptr->crr, g_cal_ptr->wheel_circ_m };
        os_mbuf_append(ctxt->om, vals, sizeof(vals));
        return 0;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 8) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t buf[12] = {0};
    uint16_t copy = len < 12 ? len : 12;
    os_mbuf_copydata(ctxt->om, 0, copy, buf);

    float mass_kg, crr;
    memcpy(&mass_kg, buf + 0, 4);
    memcpy(&crr,     buf + 4, 4);

    if (mass_kg < 33.0f || mass_kg > 200.0f) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    if (crr < 0.001f    || crr > 0.025f)     return BLE_ATT_ERR_VALUE_NOT_ALLOWED;

    g_cal_ptr->mass_kg = mass_kg;
    g_cal_ptr->crr     = crr;

    // wheel_circ_m optional (only when client sends the full 12-byte payload)
    if (len >= 12) {
        float wheel;
        memcpy(&wheel, buf + 8, 4);
        if (wheel < 1.0f || wheel > 2.5f) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        g_cal_ptr->wheel_circ_m = wheel;
    }

    cal_save(g_cal_ptr);

    ESP_LOGI("ble_cfg", "Config updated: mass=%.1f kg  crr=%.4f  wheel=%.3f m",
             mass_kg, crr, g_cal_ptr->wheel_circ_m);
    return 0;
}

// ─── GATT service table ───────────────────────────────────────────────────────
static const struct ble_gatt_svc_def GATT_SERVICES[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &CHR_PITOT.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_pitot_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid       = &CHR_IMU.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_imu_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid       = &CHR_ENV.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_env_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid       = &CHR_ANT.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_ant_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid       = &CHR_IDENTITY.u,
                .access_cb  = identity_access_cb,
                .val_handle = &g_chr_identity_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid       = &CHR_VERSION.u,
                .access_cb  = version_ota_access_cb,
                .val_handle = &g_chr_version_h,
                .flags      = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid       = &CHR_OTA_URL.u,
                .access_cb  = version_ota_access_cb,
                .val_handle = &g_chr_ota_h,
                .flags      = BLE_GATT_CHR_F_WRITE,
            },
            {
                // Config: massKg + crr + wheelCircM (3×float32, 12 B) — READ + WRITE WITH RESPONSE
                .uuid       = &CHR_CONFIG.u,
                .access_cb  = config_access_cb,
                .val_handle = &g_chr_config_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Physics output: 28 bytes, NOTIFY 10 Hz
                .uuid       = &CHR_PHYSICS.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_physics_h,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Battery moved to 0xaa0a
                .uuid       = &CHR_BATTERY.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_battery_h,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        }
    },
    { 0 }
};

static void on_sync(void)
{
    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_advertise();
    ble_sensors_on_sync();
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_server_init(aerodrag_sensors_t *sensors, SemaphoreHandle_t mutex,
                           aerodrag_cal_t *cal)
{
    g_sensors_ptr       = sensors;
    g_ble_sensors_mutex = mutex;
    g_cal_ptr           = cal;
    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    esp_err_t ret = ble_gatts_count_cfg(GATT_SERVICES);
    if (ret != ESP_OK) return ret;
    ret = ble_gatts_add_svcs(GATT_SERVICES);
    if (ret != ESP_OK) return ret;

    ble_hs_cfg.sync_cb = on_sync;

    ble_npl_mutex_init(&g_notify_mutex);
    struct ble_npl_eventq *eq = nimble_port_get_dflt_eventq();
    ble_npl_callout_init(&g_callout_pitot,   eq, callout_pitot,   NULL);
    ble_npl_callout_init(&g_callout_imu,     eq, callout_imu,     NULL);
    ble_npl_callout_init(&g_callout_env,     eq, callout_env,     NULL);
    ble_npl_callout_init(&g_callout_ant,     eq, callout_ant,     NULL);
    ble_npl_callout_init(&g_callout_bat,     eq, callout_bat,     NULL);
    ble_npl_callout_init(&g_callout_physics, eq, callout_physics, NULL);

    nimble_port_freertos_init(nimble_host_task);
    return ESP_OK;
}

#define MAKE_CALLOUT_FN(name, slot, handle_var, notify_flag)            \
static void callout_##name(struct ble_npl_event *ev) {                   \
    (void)ev;                                                             \
    if (!notify_flag || g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;\
    uint8_t _buf[32]; uint8_t _len;                                      \
    ble_npl_mutex_pend(&g_notify_mutex, BLE_NPL_TIME_FOREVER);           \
    _len = slot.len;                                                      \
    if (_len > 32) _len = 32;                                            \
    memcpy(_buf, slot.data, _len);                                        \
    ble_npl_mutex_release(&g_notify_mutex);                               \
    struct os_mbuf *om = ble_hs_mbuf_from_flat(_buf, _len);              \
    if (om) ble_gattc_notify_custom(g_conn_handle, handle_var, om);      \
}

MAKE_CALLOUT_FN(pitot,   g_slot_pitot,   g_chr_pitot_h,   g_notify_pitot)
MAKE_CALLOUT_FN(imu,     g_slot_imu,     g_chr_imu_h,     g_notify_imu)
MAKE_CALLOUT_FN(env,     g_slot_env,     g_chr_env_h,     g_notify_env)
MAKE_CALLOUT_FN(ant,     g_slot_ant,     g_chr_ant_h,     g_notify_ant)
MAKE_CALLOUT_FN(bat,     g_slot_bat,     g_chr_battery_h, g_notify_battery)
MAKE_CALLOUT_FN(physics, g_slot_physics, g_chr_physics_h, g_notify_physics)

static void fill_and_schedule(notify_slot_t *slot,
                               struct ble_npl_callout *callout,
                               const void *data, uint8_t len)
{
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    if (len > sizeof(slot->data)) len = sizeof(slot->data);
    ble_npl_mutex_pend(&g_notify_mutex, BLE_NPL_TIME_FOREVER);
    memcpy(slot->data, data, len);
    slot->len = len;
    ble_npl_mutex_release(&g_notify_mutex);
    ble_npl_callout_reset(callout, 0);
}

void ble_notify_pitot(float pitot_pa, float static_pa)
{
    if (!g_notify_pitot) return;
    float v[2] = { pitot_pa, static_pa };
    fill_and_schedule(&g_slot_pitot, &g_callout_pitot, v, sizeof(v));
}

void ble_notify_imu(float pitch, float roll)
{
    if (!g_notify_imu) return;
    float v[2] = { pitch, roll };
    fill_and_schedule(&g_slot_imu, &g_callout_imu, v, sizeof(v));
}

void ble_notify_env(float temp, float humidity, float alt, float speed_ms)
{
    if (!g_notify_env) return;
    float v[4] = { temp, humidity, alt, speed_ms };
    fill_and_schedule(&g_slot_env, &g_callout_env, v, sizeof(v));
}

void ble_notify_ant(uint16_t power, uint8_t cad, uint8_t hr)
{
    if (!g_notify_ant) return;
    uint8_t v[4]; memcpy(v, &power, 2); v[2] = cad; v[3] = hr;
    fill_and_schedule(&g_slot_ant, &g_callout_ant, v, sizeof(v));
}

void ble_notify_battery(uint8_t pct)
{
    if (!g_notify_battery) return;
    fill_and_schedule(&g_slot_bat, &g_callout_bat, &pct, sizeof(pct));
}

// ─── Physics output notify (0xaa09) — 28 bytes, 10 Hz ────────────────────────
// Layout: cda(4) vAirMs(4) rhoKgM3(4) pctAero(4) pAeroW(4) pRollingW(4) pGravityW(4)
// Contract v0.1.0 — pctAero is a percentage 0-100 (same scale as WiFi/Pi/app).
void ble_notify_physics(const aerodrag_physics_t *p)
{
    if (!g_notify_physics) return;
    float v[7];
    if (p && p->valid && p->v_air_ms >= 0.5f) {
        v[0] = p->CdA;
        v[1] = p->v_air_ms;
        v[2] = p->rho;
        v[3] = (float)p->pct_aero;     // percent 0-100 (was 0-1 before contract v0.1.0)
        v[4] = p->p_aero_w;
        v[5] = p->p_rolling_w;
        v[6] = p->p_gravity_w;
    } else {
        memset(v, 0, sizeof(v));
    }
    fill_and_schedule(&g_slot_physics, &g_callout_physics, v, sizeof(v));
}

bool ble_is_connected(void)
{
    return g_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}
