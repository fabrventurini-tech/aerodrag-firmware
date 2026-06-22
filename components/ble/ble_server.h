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

// CONFIG (0xaa08): massKg + crr + wheelCircM [3× float32] — READ + WRITE
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

// SENSOR_WHITELIST (0xaa0b): elenco sensori autorizzati — READ + WRITE (v0.2.0)
static const ble_uuid128_t CHR_SENSOR_WL = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x0b, 0xaa, 0x00, 0x00);

// WHEEL_STREAM (0xaa0c): relay stream sensore ruota Crr — NOTIFY 16 B (v0.2.0)
static const ble_uuid128_t CHR_WHEEL_STREAM = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x0c, 0xaa, 0x00, 0x00);

// WHEEL_CMD (0xaa0d): comando coast-down → sensore ruota — WRITE 1 B (v0.2.0)
static const ble_uuid128_t CHR_WHEEL_CMD = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x0d, 0xaa, 0x00, 0x00);

// SENSOR_SCAN (0xaa0e): discovery sensori — WRITE start/stop + NOTIFY entry (v0.2.2)
static const ble_uuid128_t CHR_SENSOR_SCAN = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x0e, 0xaa, 0x00, 0x00);

// COACH_LINK (0xaa0f): relay coach→app — NOTIFY 2 B `uint8 type + uint8 arg` (v0.3.0)
static const ble_uuid128_t CHR_COACH_LINK = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x0f, 0xaa, 0x00, 0x00);

// TIME (0xaa10): orologio oggettivo UTC — READ + WRITE `uint64 epochMs` LE (v0.3.0)
static const ble_uuid128_t CHR_TIME = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x10, 0xaa, 0x00, 0x00);

static uint16_t g_chr_identity_h    = 0;
static uint16_t g_chr_version_h     = 0;
static uint16_t g_chr_ota_h         = 0;
static uint16_t g_chr_config_h      = 0;
static uint16_t g_chr_physics_h     = 0;
static uint16_t g_chr_sensorwl_h    = 0;
static uint16_t g_chr_wheelstream_h = 0;
static uint16_t g_chr_wheelcmd_h    = 0;
static uint16_t g_chr_sensorscan_h  = 0;
static uint16_t g_chr_coachlink_h   = 0;
static uint16_t g_chr_time_h        = 0;

// Notify slot — 32 bytes to accommodate physics payload (28 B)
typedef struct { uint8_t data[32]; uint8_t len; bool pending; } notify_slot_t;

static notify_slot_t g_slot_pitot   = {0};
static notify_slot_t g_slot_imu     = {0};
static notify_slot_t g_slot_env     = {0};
static notify_slot_t g_slot_ant     = {0};
static notify_slot_t g_slot_bat     = {0};
static notify_slot_t g_slot_physics = {0};
static notify_slot_t g_slot_wheel   = {0};
static notify_slot_t g_slot_scan    = {0};
static notify_slot_t g_slot_coach   = {0};

static struct ble_npl_callout g_callout_pitot;
static struct ble_npl_callout g_callout_imu;
static struct ble_npl_callout g_callout_env;
static struct ble_npl_callout g_callout_ant;
static struct ble_npl_callout g_callout_bat;
static struct ble_npl_callout g_callout_physics;
static struct ble_npl_callout g_callout_wheel;
static struct ble_npl_callout g_callout_scan;
static struct ble_npl_callout g_callout_coach;
static struct ble_npl_mutex   g_notify_mutex;

static void callout_pitot  (struct ble_npl_event *ev);
static void callout_imu    (struct ble_npl_event *ev);
static void callout_env    (struct ble_npl_event *ev);
static void callout_ant    (struct ble_npl_event *ev);
static void callout_bat    (struct ble_npl_event *ev);
static void callout_physics(struct ble_npl_event *ev);
static void callout_wheel  (struct ble_npl_event *ev);
static void callout_scan   (struct ble_npl_event *ev);
static void callout_coach  (struct ble_npl_event *ev);

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
static bool     g_notify_wheel   = false;
static bool     g_notify_scan    = false;
static bool     g_notify_coach   = false;

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
        g_notify_battery = g_notify_physics = g_notify_wheel =
        g_notify_scan = g_notify_coach = false;
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
        else if (event->subscribe.attr_handle == g_chr_wheelstream_h)
            g_notify_wheel = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle == g_chr_sensorscan_h)
            g_notify_scan = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle == g_chr_coachlink_h)
            g_notify_coach = event->subscribe.cur_notify;
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
    } else {
        // Char NOTIFY-only (physics/battery/wheel/coach/scan): nessuna READ ammessa.
        // I flag GATT già lo impediscono; difesa esplicita contro READ inattese.
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
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
// Write da 8 byte (solo mass+crr) ancora accettata: la circonferenza resta invariata.
extern esp_err_t cal_save(const aerodrag_cal_t *cal);

static int config_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (!g_cal_ptr) return BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        float vals[3] = { g_cal_ptr->mass_kg, g_cal_ptr->crr,
                          g_cal_ptr->wheel_circ_m };
        os_mbuf_append(ctxt->om, vals, sizeof(vals));
        return 0;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 8) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t buf[12];
    os_mbuf_copydata(ctxt->om, 0, (len < 12) ? len : 12, buf);

    float mass_kg, crr;
    memcpy(&mass_kg, buf + 0, 4);
    memcpy(&crr,     buf + 4, 4);

    // Forma !(x >= lo && x <= hi): respinge anche NaN, che con
    // (x < lo || x > hi) passerebbe la validazione
    if (!(mass_kg >= 33.0f && mass_kg <= 200.0f)) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    if (!(crr >= 0.001f && crr <= 0.025f))        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;

    float wheel_m = g_cal_ptr->wheel_circ_m;
    if (len >= 12) {
        memcpy(&wheel_m, buf + 8, 4);
        if (!(wheel_m >= 1.0f && wheel_m <= 2.5f)) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }

    /* Write identica alla config corrente: ACK senza toccare la NVS.
     * L'app riscrive la config a ogni connessione e a ogni tap +/-
     * nelle Impostazioni — evita cicli di erase/write inutili. */
    if (mass_kg == g_cal_ptr->mass_kg && crr == g_cal_ptr->crr &&
        wheel_m == g_cal_ptr->wheel_circ_m)
        return 0;

    g_cal_ptr->mass_kg      = mass_kg;
    g_cal_ptr->crr          = crr;
    g_cal_ptr->wheel_circ_m = wheel_m;
    cal_save(g_cal_ptr);
    ble_sensors_set_wheel_circumference(wheel_m);
    ble_sensors_set_rider_mass(mass_kg);   /* propaga al sensore ruota (0xBB04) */

    ESP_LOGI("ble_cfg", "Config updated: mass=%.1f kg  crr=%.4f  wheel=%.3f m",
             mass_kg, crr, wheel_m);
    return 0;
}

// ─── SENSOR_WHITELIST (0xaa0b) callback — READ + WRITE (contract v0.2.0) ──────
// WRITE payload: uint8 count + count×(uint8 type + uint8 mac[6]).
// type: 1=power, 2=csc, 3=hr, 4=wheel. mac[6] in display order (mac[0]=primo ottetto, v0.2.3).
static int sensor_wl_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        sensor_wl_entry_t wl[SENSOR_WL_MAX];
        uint8_t n = ble_sensors_get_whitelist(wl, SENSOR_WL_MAX);
        uint8_t out[1 + SENSOR_WL_MAX * 7];
        out[0] = n;
        for (uint8_t i = 0; i < n; i++) {
            out[1 + i*7] = wl[i].type;
            memcpy(&out[2 + i*7], wl[i].mac, 6);
        }
        os_mbuf_append(ctxt->om, out, (uint16_t)(1 + n * 7));
        return 0;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t buf[1 + SENSOR_WL_MAX * 7];
    uint16_t copy = (len < sizeof(buf)) ? len : sizeof(buf);
    os_mbuf_copydata(ctxt->om, 0, copy, buf);

    uint8_t count = buf[0];
    if (count > SENSOR_WL_MAX) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    if ((uint16_t)(1 + count * 7) > copy) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    sensor_wl_entry_t wl[SENSOR_WL_MAX] = {0};
    for (uint8_t i = 0; i < count; i++) {
        wl[i].type = buf[1 + i*7];
        memcpy(wl[i].mac, &buf[2 + i*7], 6);
    }
    ble_sensors_set_whitelist(wl, count);
    return 0;
}

// ─── WHEEL_CMD (0xaa0d) callback — WRITE (contract v0.2.0) ────────────────────
// Inoltra il comando coast-down (1 byte) al sensore ruota via 0xBB03.
static int wheelcmd_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    uint8_t cmd = 0;
    os_mbuf_copydata(ctxt->om, 0, 1, &cmd);
    ble_sensors_wheel_command(cmd);
    return 0;
}

// ─── SENSOR_SCAN (0xaa0e) callback — WRITE start/stop discovery (v0.2.2) ──────
static int sensor_scan_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    if (OS_MBUF_PKTLEN(ctxt->om) < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    uint8_t v = 0;
    os_mbuf_copydata(ctxt->om, 0, 1, &v);
    ble_sensors_set_discovery(v == 0x01);
    return 0;
}

// ─── TIME (0xaa10) callback — orologio oggettivo UTC (contract v0.3.0) ────────
// READ  → uint64 epochMs LE (0 se l'orologio non è impostato, anno < 2020).
// WRITE → uint64 epochMs LE (8 byte ESATTI): imposta l'orologio di sistema.
static int time_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint64_t ms = aerodrag_time_get_epoch_ms();   // 0 se non impostato
        os_mbuf_append(ctxt->om, &ms, sizeof(ms));     // little-endian (ESP32)
        return 0;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    if (OS_MBUF_PKTLEN(ctxt->om) != 8) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    uint64_t ms = 0;
    os_mbuf_copydata(ctxt->om, 0, 8, &ms);
    if (ms < AERODRAG_EPOCH_MIN_MS) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;  // epoch < 2020
    aerodrag_time_set_epoch_ms(ms);
    ESP_LOGI("ble_time", "Orologio impostato: %llu ms UTC", (unsigned long long)ms);
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
                // Config: massKg + crr + wheelCircM (3× float32, 12 B) — READ + WRITE
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
            {
                // Sensor whitelist (v0.2.0): l'app scrive i sensori autorizzati
                .uuid       = &CHR_SENSOR_WL.u,
                .access_cb  = sensor_wl_access_cb,
                .val_handle = &g_chr_sensorwl_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Wheel stream relay (v0.2.0): NOTIFY 16 B
                .uuid       = &CHR_WHEEL_STREAM.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_wheelstream_h,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Wheel coast-down command (v0.2.0): WRITE 1 B
                .uuid       = &CHR_WHEEL_CMD.u,
                .access_cb  = wheelcmd_access_cb,
                .val_handle = &g_chr_wheelcmd_h,
                .flags      = BLE_GATT_CHR_F_WRITE,
            },
            {
                // Sensor discovery (v0.2.2): WRITE start/stop + NOTIFY entries
                .uuid       = &CHR_SENSOR_SCAN.u,
                .access_cb  = sensor_scan_access_cb,
                .val_handle = &g_chr_sensorscan_h,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Coach link relay (v0.3.0): NOTIFY 2 B (cmd/stato coach → app)
                .uuid       = &CHR_COACH_LINK.u,
                .access_cb  = chr_access_cb,
                .val_handle = &g_chr_coachlink_h,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Orologio oggettivo UTC (v0.3.0): READ + WRITE uint64 epochMs LE
                .uuid       = &CHR_TIME.u,
                .access_cb  = time_access_cb,
                .val_handle = &g_chr_time_h,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
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
    ble_npl_callout_init(&g_callout_wheel,   eq, callout_wheel,   NULL);
    ble_npl_callout_init(&g_callout_scan,    eq, callout_scan,    NULL);
    ble_npl_callout_init(&g_callout_coach,   eq, callout_coach,   NULL);

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
MAKE_CALLOUT_FN(bat,     g_slot_bat,     g_chr_battery_h,     g_notify_battery)
MAKE_CALLOUT_FN(physics, g_slot_physics, g_chr_physics_h,     g_notify_physics)
MAKE_CALLOUT_FN(wheel,   g_slot_wheel,   g_chr_wheelstream_h, g_notify_wheel)
MAKE_CALLOUT_FN(scan,    g_slot_scan,    g_chr_sensorscan_h,  g_notify_scan)
MAKE_CALLOUT_FN(coach,   g_slot_coach,   g_chr_coachlink_h,   g_notify_coach)

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

// ─── Wheel stream relay notify (0xaa0c) — contract v0.2.0 ────────────────────
// Relay grezzo dello stream del sensore ruota Crr (16 B: speedMs, accelMs2,
// tempC, vibRMS). Chiamato dal central (ble_sensors.c) sui dati di 0xBB01.
void ble_notify_wheel_stream(const void *data, uint8_t len)
{
    if (!g_notify_wheel) return;
    fill_and_schedule(&g_slot_wheel, &g_callout_wheel, data, len);
}

// ─── Sensor discovery notify (0xaa0e) — contract v0.2.2 ──────────────────────
// Una entry per sensore scoperto: type(1) + mac[6] + rssi(1) + nameLen(1) + name.
void ble_notify_sensor_scan(uint8_t type, const uint8_t *mac_be, int8_t rssi,
                            const char *name, uint8_t name_len)
{
    if (!g_notify_scan) return;
    uint8_t buf[32];
    if (name_len > 22) name_len = 22;            /* 9 byte header + name ≤ 32 */
    buf[0] = type;
    memcpy(&buf[1], mac_be, 6);
    buf[7] = (uint8_t)rssi;
    buf[8] = name_len;
    if (name_len && name) memcpy(&buf[9], name, name_len);
    fill_and_schedule(&g_slot_scan, &g_callout_scan, buf, (uint8_t)(9 + name_len));
}

// ─── Coach link relay notify (0xaa0f) — contract v0.3.0 ──────────────────────
// 2 byte: `uint8 type` + `uint8 arg`. Il firmware inoltra all'app i comandi/stato
// coach ricevuti sul proprio /device (start/stop/lap, uplink su/giù). Vedi CONTRACT §2.
void ble_notify_coach_link(uint8_t type, uint8_t arg)
{
    if (!g_notify_coach) return;
    uint8_t v[2] = { type, arg };
    fill_and_schedule(&g_slot_coach, &g_callout_coach, v, sizeof(v));
}

bool ble_is_connected(void)
{
    return g_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}
