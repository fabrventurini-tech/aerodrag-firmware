# AeroDrag Wheel — firmware sensore ruota Crr

Firmware del **sensore ruota** per la calibrazione del **Crr** (Coefficient of
Rolling Resistance) tramite coast-down.

- **Board:** Seeed **XIAO BLE Sense** (nRF52840 Sense) · **IMU onboard:** **LSM6DS3TR-C** (I2C)
- **Batteria:** LiPo 3.7 V 100 mAh (caricatore onboard sul XIAO)
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

🟡 **Oltre lo skeleton** — implementati: servizio GATT `0xBB00`, integrazione
IMU onboard via **Zephyr Sensor API** (driver-agnostica), calcolo
`speedMs/accelMs2/tempC/vibRMS`, stream NOTIFY a 10 Hz (sampling 100 Hz), handler
`CMD`/`CONFIG`, advertising. L'IMU è risolta dall'alias `imu` o dal primo
`st,lsm6dsl`/`st,lsm6dso` della DTS (la board `xiao_ble//sense` dichiara l'IMU).

⚠️ **Non ancora compilato/flashato** (manca toolchain nRF Connect SDK in questo
ambiente). Da verificare in bring-up:
- che la board `xiao_ble/nrf52840/sense` esponga l'IMU LSM6DS3TR-C nella DTS con
  il **driver/Kconfig** giusto per la tua versione di NCS (qui `CONFIG_LSM6DSL`;
  se la tua DTS usa un altro compatible, adegua `prj.conf`);
- il **modello di montaggio** (asse spin = `GYRO_SPIN_CHAN` = Z) e le costanti;
- consumi/autonomia con LiPo 100 mAh (valuta sampling più basso + sleep).

`RESULT 0xBB02` resta legacy (il Crr lo calcola l'app).

## Build (nRF Connect SDK)

```sh
west build -b xiao_ble/nrf52840/sense
west flash            # XIAO: doppio reset → bootloader UF2, oppure J-Link
```

> TODO bring-up: monitoraggio batteria (tensione su pin analogico del XIAO +
> partitore) e gestione carica; gating dello stream allo stato coast-down.
