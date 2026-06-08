/*
 * ble_sensors.c — BLE Central AeroDrag v1.0
 *
 * Gestisce fino a 3 connessioni simultanee:
 *   Slot 0 → Power Meter   (0x1818)
 *   Slot 1 → CSC Sensor    (0x1816)
 *   Slot 2 → HR Monitor    (0x180D)
 *
 * Flusso per ogni slot:
 *   SCAN → CONNECT → DISC_SVC → DISC_CHR → SUBSCRIBE → NOTIFY_RX
 *
 * In caso di disconnessione il slot torna IDLE e il scan riparte
 * automaticamente per ritrovare il sensore.
 *
 * NimBLE gira su PRO_CPU. Tutte le scritture a g_sensors usano il mutex
 * passato da main.c per evitare race condition con APP_CPU tasks.
 */

#include "ble_sensors.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"

#include "host/ble_uuid.h"
#include "os/os_mbuf.h"

static const char *TAG = "ble_sens";

/* ─────────────────────────────────────────────────────────────────────────── */
/*  COSTANTI GATT                                                               */
/* ─────────────────────────────────────────────────────────────────────────── */

#define UUID_SVC_POWER      0x1818u     /* Cycling Power Service               */
#define UUID_SVC_CSC        0x1816u     /* Cycling Speed & Cadence Service      */
#define UUID_SVC_HR         0x180Du     /* Heart Rate Service                   */

#define UUID_CHR_POWER_MEAS 0x2A63u     /* Cycling Power Measurement           */
#define UUID_CHR_CSC_MEAS   0x2A5Bu     /* CSC Measurement                     */
#define UUID_CHR_HR_MEAS    0x2A37u     /* Heart Rate Measurement              */

/* UUID dei 3 servizi target — usati per filtrare l'advertising */
static const uint16_t TARGET_SVCS[] = {
    UUID_SVC_POWER, UUID_SVC_CSC, UUID_SVC_HR
};
#define N_TARGET_SVCS  (sizeof(TARGET_SVCS) / sizeof(TARGET_SVCS[0]))

/* ─────────────────────────────────────────────────────────────────────────── */
/*  STATO MACCHINA A STATI PER OGNI SLOT                                        */
/* ─────────────────────────────────────────────────────────────────────────── */

typedef enum {
    SLOT_IDLE        = 0,
    SLOT_CONNECTING  = 1,
    SLOT_DISC_SVC    = 2,
    SLOT_DISC_CHR    = 3,
    SLOT_SUBSCRIBING = 4,
    SLOT_READY       = 5,
} slot_state_t;

typedef struct {
    ble_addr_t   addr;
    uint16_t     conn_handle;
    uint16_t     svc_uuid;          /* 0x1818 / 0x1816 / 0x180D            */
    uint16_t     svc_start_handle;
    uint16_t     svc_end_handle;
    uint16_t     chr_val_handle;    /* handle valore caratteristica         */
    slot_state_t state;
} sensor_slot_t;

#define N_SLOTS 3
static sensor_slot_t s_slots[N_SLOTS];

/* ─────────────────────────────────────────────────────────────────────────── */
/*  DATI SENSORI CONDIVISI                                                       */
/* ─────────────────────────────────────────────────────────────────────────── */

static ble_sensor_data_t s_data;
static SemaphoreHandle_t s_mutex;   /* passato da main.c = g_sensors_mutex  */

/* ─────────────────────────────────────────────────────────────────────────── */
/*  STATO CALCOLO VELOCITÀ / CADENZA (delta tra notifiche successive)           */
/* ─────────────────────────────────────────────────────────────────────────── */

static float    s_wheel_circ_m   = 2.105f;  /* 700c x 25mm default */
static uint32_t s_wheel_revs_prev;
static uint16_t s_wheel_time_prev;
static bool     s_wheel_init;

static uint16_t s_crank_revs_prev;
static uint16_t s_crank_time_prev;
static bool     s_crank_init;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  HELPER: ricerca slot                                                         */
/* ─────────────────────────────────────────────────────────────────────────── */

static sensor_slot_t *slot_by_conn(uint16_t conn_h) {
    for (int i = 0; i < N_SLOTS; i++)
        if (s_slots[i].state != SLOT_IDLE &&
            s_slots[i].conn_handle == conn_h)
            return &s_slots[i];
    return NULL;
}

static sensor_slot_t *slot_idle(void) {
    for (int i = 0; i < N_SLOTS; i++)
        if (s_slots[i].state == SLOT_IDLE)
            return &s_slots[i];
    return NULL;
}

static bool already_connected(const ble_addr_t *addr) {
    for (int i = 0; i < N_SLOTS; i++)
        if (s_slots[i].state != SLOT_IDLE &&
            memcmp(s_slots[i].addr.val, addr->val, 6) == 0)
            return true;
    return false;
}

/* UUID chr corretta per ogni servizio */
static uint16_t svc_to_chr_uuid(uint16_t svc) {
    switch (svc) {
        case UUID_SVC_POWER: return UUID_CHR_POWER_MEAS;
        case UUID_SVC_CSC:   return UUID_CHR_CSC_MEAS;
        case UUID_SVC_HR:    return UUID_CHR_HR_MEAS;
        default:             return 0;
    }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  PARSING GATT — Cycling Power Measurement (0x2A63)                           */
/*                                                                               */
/*  Byte 0-1: Flags                                                              */
/*    bit 0  = Pedal Power Balance Present   (+1 byte)                          */
/*    bit 2  = Accumulated Torque Present    (+2 byte)                          */
/*    bit 4  = Wheel Revolution Data Present (+6 byte)                          */
/*    bit 5  = Crank Revolution Data Present (+4 byte) ← usiamo per cadenza    */
/*  Byte 2-3: Instantaneous Power [int16, W]                                     */
/* ─────────────────────────────────────────────────────────────────────────── */

static void parse_power_meas(const uint8_t *d, uint16_t len) {
    if (len < 4) return;
    uint16_t flags = (uint16_t)(d[0] | (d[1] << 8));
    int16_t  pw    = (int16_t) (d[2] | (d[3] << 8));

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    s_data.power_w  = (pw > 0) ? (uint16_t)pw : 0;
    s_data.status  |= BLE_SENS_POWER;

    /* Calcola offset per Crank Revolution Data */
    uint8_t off = 4;
    if (flags & (1u << 0)) off += 1;   /* Pedal Power Balance */
    if (flags & (1u << 2)) off += 2;   /* Accumulated Torque  */
    if (flags & (1u << 4)) off += 6;   /* Wheel Revolution    */

    if ((flags & (1u << 5)) && (uint16_t)(off + 4) <= len) {
        uint16_t cr = (uint16_t)(d[off+0] | (d[off+1] << 8));
        uint16_t ct = (uint16_t)(d[off+2] | (d[off+3] << 8));

        if (s_crank_init) {
            uint16_t dr = (uint16_t)(cr - s_crank_revs_prev);
            uint16_t dt = (uint16_t)(ct - s_crank_time_prev); /* 1/1024 s */
            if (dr > 0 && dt > 0) {
                float rpm = (float)dr * 1024.0f * 60.0f / (float)dt;
                if (rpm > 20.0f && rpm < 240.0f) {
                    s_data.cadence_rpm = (uint8_t)rpm;
                    s_data.status     |= BLE_SENS_CAD;
                }
            }
        }
        s_crank_revs_prev = cr;
        s_crank_time_prev = ct;
        s_crank_init      = true;
    }

    xSemaphoreGive(s_mutex);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  PARSING GATT — CSC Measurement (0x2A5B)                                     */
/*                                                                               */
/*  Byte 0: Flags                                                                */
/*    bit 0 = Wheel Revolution Data: uint32 cumulRevs + uint16 lastEventTime    */
/*    bit 1 = Crank Revolution Data: uint16 cumulRevs + uint16 lastEventTime    */
/*  lastEventTime in 1/1024 secondi — wrappa a 65535/1024 = 63.99s              */
/* ─────────────────────────────────────────────────────────────────────────── */

static void parse_csc_meas(const uint8_t *d, uint16_t len) {
    if (len < 1) return;
    uint8_t flags = d[0];
    uint8_t off   = 1;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* ── Wheel Revolution Data ── */
    if ((flags & 0x01u) && (uint16_t)(off + 6) <= len) {
        uint32_t wr = (uint32_t)(d[off+0] | ((uint32_t)d[off+1] << 8) |
                                 ((uint32_t)d[off+2] << 16) | ((uint32_t)d[off+3] << 24));
        uint16_t wt = (uint16_t)(d[off+4] | (d[off+5] << 8));
        off += 6;

        if (s_wheel_init) {
            /* uint32 e uint16 wrappano — la sottrazione funziona comunque */
            uint32_t dr = wr - s_wheel_revs_prev;
            uint16_t dt = (uint16_t)(wt - s_wheel_time_prev);
            if (dr > 0 && dt > 0) {
                /* velocità [m/s] = (revs * circ_m) / (ticks / 1024) */
                float speed_ms = (float)dr * s_wheel_circ_m * 1024.0f / (float)dt;
                if (speed_ms > 0.5f && speed_ms < 30.0f) {  /* 1.8 km/h – 108 km/h */
                    s_data.speed_cms  = (uint16_t)(speed_ms * 100.0f);
                    /* distanza cumulativa — wrappa a ~65535m = 65km */
                    s_data.distance_m = (uint16_t)((float)wr * s_wheel_circ_m);
                    s_data.status    |= BLE_SENS_SPEED;
                }
            }
        }
        s_wheel_revs_prev = wr;
        s_wheel_time_prev = wt;
        s_wheel_init      = true;
    }

    /* ── Crank Revolution Data ── */
    if ((flags & 0x02u) && (uint16_t)(off + 4) <= len) {
        uint16_t cr = (uint16_t)(d[off+0] | (d[off+1] << 8));
        uint16_t ct = (uint16_t)(d[off+2] | (d[off+3] << 8));

        if (s_crank_init) {
            uint16_t dr = (uint16_t)(cr - s_crank_revs_prev);
            uint16_t dt = (uint16_t)(ct - s_crank_time_prev);
            if (dr > 0 && dt > 0) {
                float rpm = (float)dr * 1024.0f * 60.0f / (float)dt;
                if (rpm > 20.0f && rpm < 240.0f) {
                    s_data.cadence_rpm = (uint8_t)rpm;
                    s_data.status     |= BLE_SENS_CAD;
                }
            }
        }
        s_crank_revs_prev = cr;
        s_crank_time_prev = ct;
        s_crank_init      = true;
    }

    xSemaphoreGive(s_mutex);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  PARSING GATT — Heart Rate Measurement (0x2A37)                              */
/*                                                                               */
/*  Byte 0: Flags                                                                */
/*    bit 0 = Heart Rate Value Format: 0=uint8, 1=uint16                        */
/*  Byte 1 (o 1-2): Heart Rate Value                                            */
/* ─────────────────────────────────────────────────────────────────────────── */

static void parse_hr_meas(const uint8_t *d, uint16_t len) {
    if (len < 2) return;
    uint8_t hr;
    if (d[0] & 0x01u) {
        if (len < 3) return;
        uint16_t hr16 = (uint16_t)(d[1] | (d[2] << 8));
        hr = (hr16 > 255u) ? 255u : (uint8_t)hr16;
    } else {
        hr = d[1];
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_data.hr_bpm  = hr;
    s_data.status |= BLE_SENS_HR;
    xSemaphoreGive(s_mutex);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  GATT DISCOVERY CALLBACKS                                                     */
/* ─────────────────────────────────────────────────────────────────────────── */

/* 3. CCCD write completato → slot pronto */
static int on_write_cccd(uint16_t conn_h,
                          const struct ble_gatt_error *err,
                          struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    sensor_slot_t *slot = (sensor_slot_t *)arg;
    if (!slot) return 0;

    if (err->status == 0) {
        slot->state = SLOT_READY;
        ESP_LOGI(TAG, "Slot[%td] PRONTO  svc=0x%04X  val_h=%d",
                 slot - s_slots, slot->svc_uuid, slot->chr_val_handle);
    } else {
        ESP_LOGW(TAG, "Slot[%td] CCCD write errore %d, disconnetto",
                 slot - s_slots, err->status);
        ble_gap_terminate(conn_h, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

/* 2. Caratteristica trovata → abilita notifiche via CCCD */
static int on_disc_chr(uint16_t conn_h,
                        const struct ble_gatt_error *err,
                        const struct ble_gatt_chr *chr, void *arg)
{
    sensor_slot_t *slot = (sensor_slot_t *)arg;
    if (!slot) return 0;

    if (err->status != 0 && err->status != BLE_HS_EDONE) {
        ble_gap_terminate(conn_h, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    if (err->status == BLE_HS_EDONE) {
        if (slot->chr_val_handle == 0) {
            ESP_LOGW(TAG, "Slot[%td] caratteristica 0x%04X non trovata",
                     slot - s_slots, svc_to_chr_uuid(slot->svc_uuid));
            ble_gap_terminate(conn_h, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    if (!chr) return 0;

    uint16_t want = svc_to_chr_uuid(slot->svc_uuid);
    uint16_t got  = ble_uuid_u16(&chr->uuid.u);

    if (got == want) {
        slot->chr_val_handle = chr->val_handle;
        slot->state          = SLOT_SUBSCRIBING;

        /*
         * Il CCCD si trova quasi sempre a val_handle + 1 per i sensori
         * di ciclismo. Usiamo questo approccio diretto anziché lanciare
         * una discovery degli attribute descriptor per semplicità.
         * Se un sensore edge-case non segue questo layout, aggiungere
         * ble_gattc_disc_dsc_by_uuid() qui.
         */
        uint16_t cccd_h    = chr->val_handle + 1;
        uint8_t  enable[2] = { 0x01, 0x00 };   /* Notifications ON */

        ESP_LOGI(TAG, "Slot[%td] chr=0x%04X val_h=%d cccd_h=%d — abilito notify",
                 slot - s_slots, got, chr->val_handle, cccd_h);

        ble_gattc_write_flat(conn_h, cccd_h,
                             enable, sizeof(enable),
                             on_write_cccd, slot);
    }
    return 0;
}

/* 1. Servizio trovato → discovery caratteristica */
static int on_disc_svc(uint16_t conn_h,
                        const struct ble_gatt_error *err,
                        const struct ble_gatt_svc *svc, void *arg)
{
    sensor_slot_t *slot = (sensor_slot_t *)arg;
    if (!slot) return 0;

    if (err->status != 0 && err->status != BLE_HS_EDONE) {
        ble_gap_terminate(conn_h, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    if (err->status == BLE_HS_EDONE) {
        if (slot->svc_start_handle == 0) {
            ESP_LOGW(TAG, "Slot[%td] servizio 0x%04X non trovato",
                     slot - s_slots, slot->svc_uuid);
            ble_gap_terminate(conn_h, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    if (!svc) return 0;

    /* Prima istanza del servizio — salva range e avvia chr discovery */
    if (slot->svc_start_handle == 0) {
        slot->svc_start_handle = svc->start_handle;
        slot->svc_end_handle   = svc->end_handle;
        slot->state            = SLOT_DISC_CHR;

        ble_uuid16_t chr_uuid = BLE_UUID16_INIT(svc_to_chr_uuid(slot->svc_uuid));
        ble_gattc_disc_chrs_by_uuid(conn_h,
                                     svc->start_handle, svc->end_handle,
                                     &chr_uuid.u,
                                     on_disc_chr, slot);
    }
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  SCAN HELPER                                                                  */
/* ─────────────────────────────────────────────────────────────────────────── */

static int central_gap_event(struct ble_gap_event *event, void *arg);

static esp_timer_handle_t s_scan_restart_timer = NULL;
static void _scan_restart_cb(void *arg) { start_scan(); }

/* Riavvia il scan dopo 2 s — lascia tempo a WiFi tra una sessione e l'altra */
static void start_scan_delayed(void) {
    if (!s_scan_restart_timer) {
        const esp_timer_create_args_t ta = {
            .callback = _scan_restart_cb,
            .name     = "ble_scan_restart",
        };
        esp_timer_create(&ta, &s_scan_restart_timer);
    }
    esp_timer_start_once(s_scan_restart_timer, 2000000ULL); /* 2 s */
}

static void start_scan(void) {
    if (!slot_idle())          return;  /* tutti gli slot occupati */
    if (ble_gap_disc_active()) return;  /* scan già in corso       */

    struct ble_gap_disc_params p = {
        .itvl              = 0x0060,    /* 60 ms                    */
        .window            = 0x0030,    /* 30 ms                    */
        .filter_policy     = BLE_HCI_SCAN_FILT_NO_WL,
        .limited           = 0,
        .passive           = 0,         /* active: legge scan rsp   */
        .filter_duplicates = 1,
    };

    ESP_LOGD(TAG, "Avvio scan (slot liberi: %d)",
             (int)(slot_idle() - s_slots + 1));

    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 6000 /* ms */, &p,
                 central_gap_event, NULL);
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  HELPER: cerca UUID 16-bit nell'advertising data                              */
/* ─────────────────────────────────────────────────────────────────────────── */

static uint16_t adv_find_target_svc(const uint8_t *data, uint8_t dlen) {
    for (uint8_t i = 0; i < dlen; ) {
        if (i + 1 >= dlen) break;
        uint8_t alen = data[i];
        uint8_t type = data[i + 1];
        if (alen == 0) break;

        /* 0x02 = Incomplete list 16-bit UUIDs, 0x03 = Complete list */
        if ((type == 0x02 || type == 0x03) && alen >= 3) {
            for (uint8_t j = 2; j < alen && i + j + 1 < dlen; j += 2) {
                uint16_t uuid = (uint16_t)(data[i+j] | (data[i+j+1] << 8));
                for (unsigned k = 0; k < N_TARGET_SVCS; k++)
                    if (uuid == TARGET_SVCS[k]) return uuid;
            }
        }
        i += alen + 1;
    }
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  GAP EVENT HANDLER CENTRALE                                                   */
/* ─────────────────────────────────────────────────────────────────────────── */

static int central_gap_event(struct ble_gap_event *event, void *arg) {
    (void)arg;

    switch (event->type) {

    /* ── Dispositivo scoperto durante scan ─────────────────────────────── */
    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc *disc = &event->disc;
        uint16_t svc = adv_find_target_svc(disc->data, disc->length_data);
        if (!svc) break;
        if (already_connected(&disc->addr)) break;

        /* Controlla se abbiamo già uno slot per questo tipo di servizio */
        bool svc_already = false;
        for (int i = 0; i < N_SLOTS; i++)
            if (s_slots[i].state != SLOT_IDLE && s_slots[i].svc_uuid == svc)
                { svc_already = true; break; }
        if (svc_already) break;

        sensor_slot_t *slot = slot_idle();
        if (!slot) break;

        ESP_LOGI(TAG, "Trovato svc=0x%04X addr=%02X:%02X:%02X:%02X:%02X:%02X",
                 svc,
                 disc->addr.val[5], disc->addr.val[4], disc->addr.val[3],
                 disc->addr.val[2], disc->addr.val[1], disc->addr.val[0]);

        slot->addr             = disc->addr;
        slot->svc_uuid         = svc;
        slot->state            = SLOT_CONNECTING;
        slot->conn_handle      = BLE_HS_CONN_HANDLE_NONE;
        slot->svc_start_handle = 0;
        slot->chr_val_handle   = 0;

        ble_gap_disc_cancel();

        int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &disc->addr,
                                 10000 /* ms timeout */,
                                 NULL  /* default conn params */,
                                 central_gap_event, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_connect errore %d", rc);
            memset(slot, 0, sizeof(*slot));
            start_scan();
        }
        break;
    }

    /* ── Connessione stabilita ─────────────────────────────────────────── */
    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Connessione fallita: %d", event->connect.status);
            for (int i = 0; i < N_SLOTS; i++)
                if (s_slots[i].state == SLOT_CONNECTING)
                    memset(&s_slots[i], 0, sizeof(s_slots[i]));
            start_scan();
            break;
        }
        uint16_t conn_h = event->connect.conn_handle;
        for (int i = 0; i < N_SLOTS; i++) {
            if (s_slots[i].state == SLOT_CONNECTING) {
                s_slots[i].conn_handle = conn_h;
                s_slots[i].state       = SLOT_DISC_SVC;

                ESP_LOGI(TAG, "Slot[%d] connesso conn_h=%d svc=0x%04X — avvio discovery",
                         i, conn_h, s_slots[i].svc_uuid);

                ble_uuid16_t svc_uuid = BLE_UUID16_INIT(s_slots[i].svc_uuid);
                ble_gattc_disc_svc_by_uuid(conn_h, &svc_uuid.u,
                                           on_disc_svc, &s_slots[i]);
                break;
            }
        }
        /* Se ci sono ancora slot liberi, riprende il scan */
        if (slot_idle()) start_scan();
        break;
    }

    /* ── Disconnessione ────────────────────────────────────────────────── */
    case BLE_GAP_EVENT_DISCONNECT: {
        uint16_t conn_h = event->disconnect.conn.conn_handle;
        sensor_slot_t *slot = slot_by_conn(conn_h);
        if (slot) {
            ESP_LOGI(TAG, "Slot[%td] disconnesso svc=0x%04X reason=%d — rescan",
                     slot - s_slots, slot->svc_uuid,
                     event->disconnect.reason);
            /* Resetta stato calcolo delta per CSC */
            if (slot->svc_uuid == UUID_SVC_CSC) {
                s_wheel_init = false;
                s_crank_init = false;
            }
            if (slot->svc_uuid == UUID_SVC_POWER) {
                s_crank_init = false;
            }
            memset(slot, 0, sizeof(*slot));
        }
        start_scan_delayed();
        break;
    }

    /* ── Notifica ricevuta dal sensore ─────────────────────────────────── */
    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t conn_h = event->notify_rx.conn_handle;
        uint16_t val_h  = event->notify_rx.attr_handle;

        sensor_slot_t *slot = slot_by_conn(conn_h);
        if (!slot || slot->chr_val_handle != val_h) break;

        /* Copia i dati dall'mbuf in un buffer locale */
        uint16_t pkt_len = OS_MBUF_PKTLEN(event->notify_rx.om);
        uint8_t  buf[40];
        uint16_t copy_len = (pkt_len < sizeof(buf)) ? pkt_len : sizeof(buf);
        os_mbuf_copydata(event->notify_rx.om, 0, copy_len, buf);

        switch (slot->svc_uuid) {
            case UUID_SVC_POWER: parse_power_meas(buf, copy_len); break;
            case UUID_SVC_CSC:   parse_csc_meas(buf, copy_len);   break;
            case UUID_SVC_HR:    parse_hr_meas(buf, copy_len);    break;
        }
        break;
    }

    /* ── Scan completato senza target (timeout) ────────────────────────── */
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGD(TAG, "Scan completato, riavvio tra 2s");
        start_scan_delayed();
        break;

    default:
        break;
    }
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  API PUBBLICA                                                                 */
/* ─────────────────────────────────────────────────────────────────────────── */

void ble_sensors_init(SemaphoreHandle_t sensors_mutex) {
    s_mutex = sensors_mutex;
    memset(s_slots, 0, sizeof(s_slots));
    memset(&s_data, 0, sizeof(s_data));
    s_wheel_init = false;
    s_crank_init = false;
    /* Il scan parte in ble_sensors_on_sync(), non qui */
}

void ble_sensors_on_sync(void) {
    /*
     * Chiamato da on_sync() in ble_server.h, dopo ble_advertise().
     * A questo punto NimBLE è sincronizzato — il scan è sicuro.
     */
    ESP_LOGI(TAG, "BLE Central pronto — avvio scan sensori");
    start_scan();
}

void ble_sensors_get(ble_sensor_data_t *out) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_data;
    /* Reset flag one-shot dopo la lettura */
    s_data.lap_event = false;
    s_data.status    = 0;
    xSemaphoreGive(s_mutex);
}

void ble_sensors_trigger_lap(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_data.lap_event = true;
    s_data.status   |= BLE_SENS_LAP;
    xSemaphoreGive(s_mutex);
}

void ble_sensors_set_wheel_circumference(float meters) {
    s_wheel_circ_m = meters;
}
