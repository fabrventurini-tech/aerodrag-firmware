# AeroDrag Wheel — firmware sensore ruota Crr

Firmware del **sensore ruota** per la calibrazione del **Crr** (Coefficient of
Rolling Resistance) tramite coast-down.

- **MCU:** nRF52840 (Nordic) · **IMU:** ICM-42688-P
- **Toolchain:** nRF Connect SDK / Zephyr (BLE peripheral)
- **Ruolo:** dispositivo BLE che espone il servizio proprietario **`0xBB00`**.

## Gestione (repo figlia del contratto)

Questo è un **componente gestito** dal contratto della repo madre:
[`../docs/CONTRACT.md`](../docs/CONTRACT.md). Vive come sottocartella di
`aerodrag-firmware` ma è un firmware **separato** (MCU e toolchain diversi):
NON fa parte del build ESP-IDF (cartella top-level, non `components/`).

Da contract **v0.2.0**: il sensore ruota si **bonda solo all'ESP32** (BLE
central). L'app non si connette più direttamente. L'ESP32:
- legge lo **stream** `0xBB01` e lo relaya all'app (`0xaa0c`);
- inoltra i **comandi** coast-down al `0xBB03` (da `0xaa0d`);
- scrive la **config** (circonferenza+massa) sul `0xBB04` (da `CONFIG 0xaa08`).
Il `crr` viene calcolato **dall'app** sullo stream — il `0xBB02` (risultato Crr
del sensore) resta esposto ma **non è usato** nel flusso v0.2.0.

## Contratto BLE — servizio `0xBB00` (UUID base `0000bbXX-0000-1000-8000-00805f9b34fb`)

| CHR | UUID | Flags | Bytes | Payload |
|-----|------|-------|-------|---------|
| STREAM | `0xBB01` | NOTIFY 10 Hz | 16 | `float32 speedMs, accelMs2, tempC, vibRMS` (LE) |
| RESULT | `0xBB02` | NOTIFY on-complete | 6 | `float32 crr + uint8 quality + uint8 runIdx` (legacy, non usato in v0.2.0) |
| CMD | `0xBB03` | WRITE | 1 | `uint8`: 0x01 indoor, 0x02 outdoor-A, 0x03 outdoor-B, 0xFF cancel |
| CONFIG | `0xBB04` | READ+WRITE | 8 | `float32 tireCircM + float32 massKg` |

Governance: nessuna modifica al servizio `0xBB00` in autonomia — si propone nella
seam `firmware↔wheel`, si concorda, poi il capofila ratifica nel contratto.

## Modello fisico (assunzioni HW — da validare)

Sensore montato sul **mozzo**: l'asse di rotazione ruota = asse **Z** del giroscopio.
- `omega = |gyro_z|` [rad/s] → `speedMs = omega · tireCircM/(2π)`
- `accelMs2 = d(speedMs)/dt` (low-pass) — decelerazione coast-down
- `vibRMS` = RMS della componente AC del modulo accelerometrico (rugosità)
- `tempC` = temperatura die IMU

Se l'asse di spin non è Z, cambiare `GYRO_SPIN_CHAN` in `src/main.c`. I segni e
le costanti di montaggio vanno verificati sul prototipo.

## Stato

🟡 **Oltre lo skeleton** — implementati: servizio GATT `0xBB00`, integrazione IMU
ICM-42688 via Zephyr Sensor API, calcolo `speedMs/accelMs2/tempC/vibRMS`, stream
NOTIFY a 10 Hz (sampling 100 Hz), handler `CMD`/`CONFIG`, advertising, overlay
devicetree SPI (`app.overlay`, da adattare alla board).

⚠️ **Non ancora compilato/flashato** (manca toolchain nRF Connect SDK in questo
ambiente) e il **modello di montaggio è un'assunzione**: serve bring-up + taratura
su hardware reale. `RESULT 0xBB02` resta legacy (il Crr lo calcola l'app).

## Build (nRF Connect SDK)

```sh
west build -b aerodrag_wheel_nrf52840   # board da definire (o nrf52840dk_nrf52840)
west flash
```
