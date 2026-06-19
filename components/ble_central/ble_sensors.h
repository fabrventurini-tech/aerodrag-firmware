/*
 * ble_sensors.h — BLE Central per AeroDrag
 *
 * Sostituisce COMPLETAMENTE:
 *   - aerodrag-dongle/ (firmware nRF52840 + ANT+)
 *   - components/ant/uart_bridge.h (parser frame UART)
 *   - task_ant() in main.c
 *
 * Legge via GATT BLE:
 *   Power Meter  →  Cycling Power Service 0x1818 / chr 0x2A63
 *   CSC Sensor   →  Cycling Speed & Cadence 0x1816 / chr 0x2A5B
 *   HR Monitor   →  Heart Rate Service 0x180D / chr 0x2A37
 *
 * API pubblica identica all'interfaccia uart_bridge lato ESP32:
 *   stessa struttura di bit nel campo `status`
 *   stesse unità di misura (speed_cms, distance_m, ecc.)
 *   stessa semantica di lap_event (one-shot, resettato dopo la lettura)
 *
 * Requisiti build (CMakeLists.txt del componente):
 *   REQUIRES bt
 *   CONFIG_BT_NIMBLE_ENABLED=y
 *   CONFIG_BT_NIMBLE_ROLE_CENTRAL=y
 *   CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y   ← già attivo per ble_server
 *   CONFIG_BT_NIMBLE_MAX_CONNECTIONS=4   ← 3 sensori + 1 app
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ── Bit del campo status — identici ai bit del frame UART precedente ───── */
#define BLE_SENS_POWER  (1u << 0)   /* power_w      aggiornato          */
#define BLE_SENS_HR     (1u << 1)   /* hr_bpm       aggiornato          */
#define BLE_SENS_CAD    (1u << 2)   /* cadence_rpm  aggiornato          */
#define BLE_SENS_SPEED  (1u << 3)   /* speed_cms    aggiornato          */
#define BLE_SENS_LAP    (1u << 4)   /* evento LAP (one-shot)            */

/* ── Struttura dati condivisa (stessa semantica di g_sensors in main.c) ─── */
typedef struct {
    uint16_t power_w;       /* potenza [W]                              */
    uint8_t  cadence_rpm;   /* cadenza [rpm]                            */
    uint8_t  hr_bpm;        /* freq. cardiaca [bpm]                     */
    uint16_t speed_cms;     /* velocità [cm/s], da CSC wheel revs       */
    uint32_t distance_m;    /* distanza cumulativa [m]                  */
    uint8_t  status;        /* maschera di bit BLE_SENS_*               */
    bool     lap_event;     /* true = LAP ricevuto, resettato dopo read  */
} ble_sensor_data_t;

/*
 * ble_sensors_init()
 * -----------------
 * Registra il modulo centrale. Chiama DOPO ble_server_init() e PRIMA
 * che il NimBLE host task parta (cioè prima di nimble_port_freertos_init).
 * Non avvia il scan — lo fa ble_sensors_on_sync() quando NimBLE è pronto.
 *
 * Parametri:
 *   sensors_mutex  — mutex di g_sensors già creato in main.c
 *                    (usato per proteggere le scritture dai callback BLE)
 */
void ble_sensors_init(SemaphoreHandle_t sensors_mutex);

/*
 * ble_sensors_on_sync()
 * ---------------------
 * Chiama dall'interno di on_sync() in ble_server.h, DOPO le chiamate
 * esistenti. Avvia la scansione BLE e il loop di connessione ai sensori.
 *
 *   static void on_sync(void) {
 *       ble_hs_id_infer_auto(0, NULL);
 *       ble_svc_gap_device_name_set(DEVICE_NAME);
 *       ble_advertise();
 *       ble_sensors_on_sync();   // ← AGGIUNGERE QUESTA RIGA
 *   }
 */
void ble_sensors_on_sync(void);

/*
 * ble_sensors_get()
 * -----------------
 * Copia l'ultimo stato dei sensori in *out.
 * Thread-safe (usa il mutex passato a ble_sensors_init).
 * Dopo la lettura resetta: lap_event = false, status = 0.
 * Chiamare al posto di uart_bridge_read() nel task pitot/imu o in
 * qualsiasi task che legge g_sensors dai canali esterni.
 */
void ble_sensors_get(ble_sensor_data_t *out);

/*
 * ble_sensors_trigger_lap()
 * -------------------------
 * Inietta un evento LAP manuale.
 * Equivalente al LAP via ANT+ Controls (profilo 16, Garmin LAP button).
 * Chiamare dal gestore del pulsante BOOT o dalla callback WebSocket
 * del coach dashboard (dove prima si inviava il comando lap via UART).
 */
void ble_sensors_trigger_lap(void);

/*
 * ble_sensors_set_wheel_circumference()
 * --------------------------------------
 * Imposta la circonferenza ruota per il calcolo velocità dal sensore CSC.
 * Default: 2.105 m (700c x 25mm).
 * Chiamare prima di ble_sensors_on_sync() se necessario.
 */
void ble_sensors_set_wheel_circumference(float meters);

/* ── Sensor whitelist (contract v0.2.0 §2) ───────────────────────────────────
 * I sensori esterni si bondano SOLO al firmware e SOLO se il loro MAC è nella
 * whitelist scritta dall'app via BLE 0xaa0b. Anti cross-talk dalle bici vicine.
 * type: 1=power(0x1818), 2=csc(0x1816), 3=hr(0x180D), 4=wheel-Crr(0xBB00).
 * mac[6] big-endian (mac[0]=AA per "AA:BB:CC:DD:EE:FF").
 */
#define SENSOR_WL_MAX 5

#define SENSOR_TYPE_POWER 1
#define SENSOR_TYPE_CSC   2
#define SENSOR_TYPE_HR    3
#define SENSOR_TYPE_WHEEL 4

typedef struct {
    uint8_t type;
    uint8_t mac[6];   /* big-endian, come la stringa "AA:BB:CC:DD:EE:FF" */
} sensor_wl_entry_t;

/* Imposta la whitelist (sostituisce quella corrente), la persiste in NVS,
 * disconnette gli slot non più ammessi e riavvia lo scan. */
void ble_sensors_set_whitelist(const sensor_wl_entry_t *entries, uint8_t count);

/* Copia la whitelist corrente in out (max entries). Ritorna il numero scritto. */
uint8_t ble_sensors_get_whitelist(sensor_wl_entry_t *out, uint8_t max);

/* Carica la whitelist da NVS. Chiamare in ble_sensors_init(). */
void ble_sensors_load_whitelist(void);

/* Inoltra un comando coast-down (0x01/0x02/0x03/0xFF) al sensore ruota Crr
 * tramite la sua caratteristica 0xBB03. Chiamato dalla WRITE su 0xaa0d. */
void ble_sensors_wheel_command(uint8_t cmd);

/* Massa rider+bici [kg] inoltrata al sensore ruota (0xBB04) per il coast-down.
 * Impostata dall'app via CONFIG 0xaa08; il firmware la propaga al sensore. */
void ble_sensors_set_rider_mass(float kg);

/*
 * ble_sensors_set_scan_enabled()
 * ------------------------------
 * Abilita/disabilita il scan dei sensori. Quando disabilitato il scan
 * attivo viene annullato e non riparte finché non si riabilita.
 * Usato dal modulo coach per liberare la radio durante l'handshake WiFi
 * iniziale (la coesistenza BLE/WiFi fa fallire l'auth se il scan è attivo).
 */
void ble_sensors_set_scan_enabled(bool enabled);
