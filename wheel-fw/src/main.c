/*
 * AeroDrag Wheel — firmware sensore ruota Crr (nRF52840 + ICM-42688-P)
 * Zephyr / nRF Connect SDK.
 *
 * Espone il servizio BLE 0xBB00 (contract: ../docs/CONTRACT.md, confine
 * firmware↔wheel). Da v0.2.0 il consumer è l'ESP32 (BLE central), che relaya
 * lo stream all'app (0xaa0c) e inoltra comandi/config.
 *
 *   0xBB01 STREAM  NOTIFY 10 Hz  16B: float speedMs, accelMs2, tempC, vibRMS
 *   0xBB02 RESULT  NOTIFY        6B:  float crr + uint8 quality + uint8 runIdx (legacy)
 *   0xBB03 CMD     WRITE         1B:  0x01 indoor / 0x02 out-A / 0x03 out-B / 0xFF cancel
 *   0xBB04 CONFIG  READ+WRITE    8B:  float tireCircM + float massKg
 *
 * ── MODELLO FISICO (ASSUNZIONI HW — da validare su hardware) ──────────────────
 * Il sensore è montato sul MOZZO della ruota: l'asse di rotazione della ruota
 * coincide con l'asse Z del giroscopio. Quindi:
 *   omega   = |gyro_z|                      [rad/s]   (velocità angolare ruota)
 *   radius  = tireCircM / (2·pi)            [m]
 *   speedMs = omega · radius                [m/s]
 *   accelMs2 = d(speedMs)/dt (low-pass)     [m/s²]    (decelerazione coast-down)
 *   vibRMS  = RMS della componente AC del modulo accelerometrico [m/s²]
 *             (deviazione dalla media mobile → rugosità superficie)
 *   tempC   = temperatura die IMU           [°C]
 * Le costanti di montaggio/segno vanno verificate; se l'asse di spin non è Z,
 * cambiare GYRO_SPIN_CHAN sotto.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(aerodrag_wheel, LOG_LEVEL_INF);

#define PI_F            3.14159265f
#define SAMPLE_HZ       100                 /* campionamento IMU              */
#define STREAM_DIV      10                  /* notify a 100/10 = 10 Hz        */
#define SAMPLE_PERIOD   K_MSEC(1000 / SAMPLE_HZ)
#define DT_S            (1.0f / SAMPLE_HZ)
#define ACCEL_LP_ALPHA  0.2f                /* low-pass su accelMs2           */
#define VIB_MEAN_ALPHA  0.05f               /* media mobile modulo accel      */

/* ── UUID servizio/caratteristiche 0xBB00 ──────────────────────────────────── */
#define BT_UUID_WHEEL_SVC     BT_UUID_DECLARE_16(0xBB00)
#define BT_UUID_WHEEL_STREAM  BT_UUID_DECLARE_16(0xBB01)
#define BT_UUID_WHEEL_RESULT  BT_UUID_DECLARE_16(0xBB02)
#define BT_UUID_WHEEL_CMD     BT_UUID_DECLARE_16(0xBB03)
#define BT_UUID_WHEEL_CONFIG  BT_UUID_DECLARE_16(0xBB04)

/* ── Config (scritta dall'ESP32 via 0xBB04) ────────────────────────────────── */
struct wheel_config {
	float tire_circ_m;
	float mass_kg;
} __packed;
static struct wheel_config g_cfg = { .tire_circ_m = 2.105f, .mass_kg = 78.0f };

/* ── Stato coast-down (da 0xBB03) ──────────────────────────────────────────── */
enum coast_mode { COAST_IDLE = 0, COAST_INDOOR = 1, COAST_OUT_A = 2, COAST_OUT_B = 3 };
static enum coast_mode g_mode = COAST_IDLE;

/* ── Stato calcolo ─────────────────────────────────────────────────────────── */
static const struct device *g_imu;
static float g_speed_ms;      /* ultima velocità                              */
static float g_accel_lp;      /* accelMs2 filtrata                            */
static float g_am_mean;       /* media mobile |accel|                         */
static float g_vib_acc;       /* accumulatore dev² nella finestra 10 Hz       */
static uint16_t g_vib_n;      /* campioni nella finestra                      */
static float g_temp_c = 20.0f;
static uint8_t g_div;         /* divisore per il notify 10 Hz                 */

/* Notify subscription flag su STREAM */
static volatile bool g_stream_subscribed;

/* ── GATT ──────────────────────────────────────────────────────────────────── */

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
	return len;
}

static void stream_ccc_cfg(const struct bt_gatt_attr *attr, uint16_t value)
{
	g_stream_subscribed = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(wheel_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_WHEEL_SVC),

	BT_GATT_CHARACTERISTIC(BT_UUID_WHEEL_STREAM, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(stream_ccc_cfg, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

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

#define ATTR_STREAM (&wheel_svc.attrs[1])

/* ── Sampling + calcolo + notify ───────────────────────────────────────────── */

/* Asse del giroscopio allineato con l'asse di spin della ruota (vedi assunzioni) */
#define GYRO_SPIN_CHAN  SENSOR_CHAN_GYRO_Z

static void process_sample(void)
{
	struct sensor_value gyro[3], accel[3], temp;

	if (sensor_sample_fetch(g_imu) != 0) {
		return;
	}

	/* Velocità angolare ruota → velocità lineare */
	float omega = 0.0f;
	if (sensor_channel_get(g_imu, GYRO_SPIN_CHAN, &gyro[2]) == 0) {
		omega = fabsf((float)sensor_value_to_double(&gyro[2]));  /* rad/s */
	}
	float radius = g_cfg.tire_circ_m / (2.0f * PI_F);
	float speed = omega * radius;
	if (speed < 0.05f) speed = 0.0f;

	/* Accelerazione (decelerazione) longitudinale = d(speed)/dt, low-pass */
	float a = (speed - g_speed_ms) / DT_S;
	g_accel_lp += ACCEL_LP_ALPHA * (a - g_accel_lp);
	g_speed_ms = speed;

	/* Vibrazione: deviazione del modulo accelerometrico dalla media mobile */
	if (sensor_channel_get(g_imu, SENSOR_CHAN_ACCEL_XYZ, accel) == 0) {
		float ax = (float)sensor_value_to_double(&accel[0]);
		float ay = (float)sensor_value_to_double(&accel[1]);
		float az = (float)sensor_value_to_double(&accel[2]);
		float am = sqrtf(ax*ax + ay*ay + az*az);
		g_am_mean += VIB_MEAN_ALPHA * (am - g_am_mean);
		float dev = am - g_am_mean;
		g_vib_acc += dev * dev;
		g_vib_n++;
	}

	if (sensor_channel_get(g_imu, SENSOR_CHAN_DIE_TEMP, &temp) == 0) {
		g_temp_c = (float)sensor_value_to_double(&temp);
	}

	/* Notify STREAM a 10 Hz */
	if (++g_div >= STREAM_DIV) {
		g_div = 0;
		float vib_rms = (g_vib_n > 0) ? sqrtf(g_vib_acc / (float)g_vib_n) : 0.0f;
		g_vib_acc = 0.0f;
		g_vib_n   = 0;

		float frame[4] = { g_speed_ms, g_accel_lp, g_temp_c, vib_rms };
		if (g_stream_subscribed) {
			bt_gatt_notify(NULL, ATTR_STREAM, frame, sizeof(frame));
		}
	}
}

static void sample_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(sample_work, sample_work_fn);

static void sample_work_fn(struct k_work *work)
{
	if (g_imu && device_is_ready(g_imu)) {
		process_sample();
	}
	k_work_reschedule(&sample_work, SAMPLE_PERIOD);
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
	k_work_schedule(&sample_work, SAMPLE_PERIOD);
}

int main(void)
{
	g_imu = DEVICE_DT_GET_ONE(invensense_icm42688);
	if (!device_is_ready(g_imu)) {
		LOG_ERR("ICM-42688 non pronto — stream a zero finché non risponde");
	} else {
		LOG_INF("ICM-42688 pronto");
	}

	int err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
	}
	return 0;
}
