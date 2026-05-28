// ─── Forward declaration — usata da chr_access_cb ──────────────────────────────────────────
static aerodrag_sensors_t *g_sensors_ptr = NULL;
static SemaphoreHandle_t    g_ble_sensors_mutex = NULL;


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

// ─── Service and characteristic UUIDs ─────────────────────────────────────────────────────────────────
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

// VERSION (0x06aa): stringa versione firmware — READ only
static const ble_uuid128_t CHR_VERSION = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x06, 0xaa, 0x00, 0x00);

// OTA_URL (0x07aa): scrivere URL del .bin per avviare OTA — WRITE only
static const ble_uuid128_t CHR_OTA_URL = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x07, 0xaa, 0x00, 0x00);

static uint16_t g_chr_identity_h = 0;
static uint16_t g_chr_version_h  = 0;
static uint16_t g_chr_ota_h      = 0;

typedef struct { uint8_t data[16]; uint8_t len; bool pending; } notify_slot_t;

static notify_slot_t g_slot_pitot = {0};
static notify_slot_t g_slot_imu   = {0};
static notify_slot_t g_slot_env   = {0};
static notify_slot_t g_slot_ant   = {0};

static struct ble_npl_callout g_callout_pitot;
static struct ble_npl_callout g_callout_imu;
static struct ble_npl_callout g_callout_env;
static struct ble_npl_callout g_callout_ant;
static struct ble_npl_mutex   g_notify_mutex;

static void callout_pitot(struct ble_npl_event *ev);
static void callout_imu  (struct ble_npl_event *ev);
static void callout_env  (struct ble_npl_event *ev);
static void callout_ant  (struct ble_npl_event *ev);

// ─── BLE state ─────────────────────────────────────────────────────────────────────────────────────
static uint16_t g_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_chr_pitot_h  = 0;
static uint16_t g_chr_imu_h    = 0;
static uint16_t g_chr_env_h    = 0;
static uint16_t g_chr_ant_h    = 0;
static bool     g_notify_pitot = false;
static bool     g_notify_imu   = false;
static bool     g_notify_env   = false;
static bool     g_notify_ant   = false;

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
        g_notify_pitot = g_notify_imu = g_notify_env = g_notify_ant = false;
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

// ─── VERSION + OTA characteristic callbacks ────────────────────────────────────────────────────────────
// VERSION (0x06aa): READ → stringa "1.0.0 (May 28 2026 12:00:00)"
// OTA_URL (0x07aa): WRITE → avvia OTA con l'URL del .bin fornito dall'app
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

// ─── GATT service table ─────────────────────────────────────────────────────────────────────────────────────
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
                // Versione firmware: es. "1.0.0 (May 28 2026 12:00:00)"
                .uuid       = &CHR_VERSION.u,
                .access_cb  = version_ota_access_cb,
                .val_handle = &g_chr_version_h,
                .flags      = BLE_GATT_CHR_F_READ,
            },
            {
                // OTA trigger: scrivere URL HTTP del .bin (max 199 char)
                .uuid       = &CHR_OTA_URL.u,
                .access_cb  = version_ota_access_cb,
                .val_handle = &g_chr_ota_h,
                .flags      = BLE_GATT_CHR_F_WRITE,
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

esp_err_t ble_server_init(aerodrag_sensors_t *sensors, SemaphoreHandle_t mutex)
{
    g_sensors_ptr       = sensors;
    g_ble_sensors_mutex = mutex;
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
    ble_npl_callout_init(&g_callout_pitot, eq, callout_pitot, NULL);
    ble_npl_callout_init(&g_callout_imu,   eq, callout_imu,   NULL);
    ble_npl_callout_init(&g_callout_env,   eq, callout_env,   NULL);
    ble_npl_callout_init(&g_callout_ant,   eq, callout_ant,   NULL);

    nimble_port_freertos_init(nimble_host_task);
    return ESP_OK;
}

#define MAKE_CALLOUT_FN(name, slot, handle_var, notify_flag)          \
static void callout_##name(struct ble_npl_event *ev) {                 \
    (void)ev;                                                           \
    if (!notify_flag || g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return; \
    uint8_t _buf[16]; uint8_t _len;                                    \
    ble_npl_mutex_pend(&g_notify_mutex, BLE_NPL_TIME_FOREVER);         \
    _len = slot.len;                                                    \
    if (_len > 16) _len = 16;                                          \
    memcpy(_buf, slot.data, _len);                                      \
    ble_npl_mutex_release(&g_notify_mutex);                             \
    struct os_mbuf *om = ble_hs_mbuf_from_flat(_buf, _len);            \
    if (om) ble_gattc_notify_custom(g_conn_handle, handle_var, om);    \
}

MAKE_CALLOUT_FN(pitot, g_slot_pitot, g_chr_pitot_h, g_notify_pitot)
MAKE_CALLOUT_FN(imu,   g_slot_imu,   g_chr_imu_h,   g_notify_imu)
MAKE_CALLOUT_FN(env,   g_slot_env,   g_chr_env_h,   g_notify_env)
MAKE_CALLOUT_FN(ant,   g_slot_ant,   g_chr_ant_h,   g_notify_ant)

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

bool ble_is_connected(void)
{
    return g_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}
