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

/*
 * ble_sensors_set_scan_enabled()
 * ------------------------------
 * Abilita/disabilita il scan dei sensori. Quando disabilitato il scan
 * attivo viene annullato e non riparte finché non si riabilita.
 * Usato dal modulo coach per liberare la radio durante l'handshake WiFi
 * iniziale (la coesistenza BLE/WiFi fa fallire l'auth se il scan è attivo).
 */
void ble_sensors_set_scan_enabled(bool enabled);
