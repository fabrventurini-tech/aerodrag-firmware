/*
 * AeroDrag Wheel — firmware sensore ruota Crr (nRF52840 + ICM-42688-P)
 * SKELETON Zephyr / nRF Connect SDK.
 *
 * Espone il servizio BLE 0xBB00 (contract: ../docs/CONTRACT.md, confine
 * firmware↔wheel). Da v0.2.0 il consumer è l'ESP32 (BLE central), che relaya
 * lo stream all'app e inoltra comandi/config.
 *
 * Caratteristiche:
 *   0xBB01 STREAM  NOTIFY 10 Hz  16B: float speedMs, accelMs2, tempC, vibRMS
 *   0xBB02 RESULT  NOTIFY        6B:  float crr + uint8 quality + uint8 runIdx (legacy)
 *   0xBB03 CMD     WRITE         1B:  0x01 indoor / 0x02 out-A / 0x03 out-B / 0xFF cancel
 *   0xBB04 CONFIG  READ+WRITE    8B:  float tireCircM + float massKg
 *
 * TODO (non implementato in questo skeleton):
 *   - driver ICM-42688-P: lettura accel/gyro → speedMs (da encoder/giroscopio),
 *     accelMs2, vibRMS; tempC dall'IMU.
 *   - macchina a stati coast-down (indoor / outdoor A/B) pilotata da 0xBB03.
 *   - (opzionale) calcolo Crr locale → 0xBB02. In v0.2.0 il Crr lo calcola l'app.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(aerodrag_wheel, LOG_LEVEL_INF);

/* ── UUID servizio/caratteristiche 0xBB00 (base BT SIG 16-bit) ─────────────── */
#define BT_UUID_WHEEL_SVC     BT_UUID_DECLARE_16(0xBB00)
#define BT_UUID_WHEEL_STREAM  BT_UUID_DECLARE_16(0xBB01)
#define BT_UUID_WHEEL_RESULT  BT_UUID_DECLARE_16(0xBB02)
#define BT_UUID_WHEEL_CMD     BT_UUID_DECLARE_16(0xBB03)
#define BT_UUID_WHEEL_CONFIG  BT_UUID_DECLARE_16(0xBB04)

/* ── Stato config (scritta dall'ESP32 via 0xBB04) ──────────────────────────── */
struct wheel_config {
	float tire_circ_m;   /* circonferenza ruota [m] */
	float mass_kg;       /* rider+bici [kg]         */
} __packed;

static struct wheel_config g_cfg = { .tire_circ_m = 2.105f, .mass_kg = 78.0f };

/* ── Stato coast-down (pilotato da 0xBB03) ─────────────────────────────────── */
enum coast_mode { COAST_IDLE = 0, COAST_INDOOR = 1, COAST_OUT_A = 2, COAST_OUT_B = 3 };
static enum coast_mode g_mode = COAST_IDLE;

/* ── GATT callbacks ────────────────────────────────────────────────────────── */

static ssize_t cfg_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &g_cfg, sizeof(g_cfg));
}

static ssize_t cfg_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(g_cfg)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(&g_cfg, buf, sizeof(g_cfg));
	LOG_INF("config: circ=%.3f m mass=%.1f kg", (double)g_cfg.tire_circ_m,
		(double)g_cfg.mass_kg);
	return len;
}

static ssize_t cmd_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	uint8_t cmd = ((const uint8_t *)buf)[0];
	switch (cmd) {
	case 0x01: g_mode = COAST_INDOOR; break;
	case 0x02: g_mode = COAST_OUT_A;  break;
	case 0x03: g_mode = COAST_OUT_B;  break;
	case 0xFF: g_mode = COAST_IDLE;   break;
	default: return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	LOG_INF("coast-down cmd=0x%02X mode=%d", cmd, g_mode);
	/* TODO: avvia/annulla la macchina a stati coast-down */
	return len;
}

/* Servizio 0xBB00: STREAM(notify) + RESULT(notify) + CMD(write) + CONFIG(rw) */
BT_GATT_SERVICE_DEFINE(wheel_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_WHEEL_SVC),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_STREAM, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_RESULT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_CMD, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, cmd_write, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_CONFIG,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       cfg_read, cfg_write, NULL),
);

/* Indici attributo per le notify (STREAM = attr[1], RESULT = attr[4]) */
#define ATTR_STREAM (&wheel_svc.attrs[1])
#define ATTR_RESULT (&wheel_svc.attrs[4])

/* ── Stream a 10 Hz ────────────────────────────────────────────────────────── */
static void stream_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(stream_work, stream_work_fn);

static void stream_work_fn(struct k_work *work)
{
	float frame[4];

	/* TODO: leggere ICM-42688-P + encoder ruota.
	 * speedMs   = revs/s * tire_circ_m
	 * accelMs2  = derivata della velocità (o accel longitudinale IMU)
	 * tempC     = temperatura IMU
	 * vibRMS    = RMS vibrazione (qualità superficie)               */
	frame[0] = 0.0f; /* speedMs  */
	frame[1] = 0.0f; /* accelMs2 */
	frame[2] = 0.0f; /* tempC    */
	frame[3] = 0.0f; /* vibRMS   */

	bt_gatt_notify(NULL, ATTR_STREAM, frame, sizeof(frame));

	k_work_reschedule(&stream_work, K_MSEC(100)); /* 10 Hz */
}

/* ── Advertising ───────────────────────────────────────────────────────────── */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x00, 0xBB), /* 0xBB00 (LE) */
};

static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("BLE init failed (%d)", err);
		return;
	}
	bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	LOG_INF("AeroDrag Wheel advertising (svc 0xBB00)");
	k_work_schedule(&stream_work, K_MSEC(100));
}

int main(void)
{
	int err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
	}
	return 0;
}
