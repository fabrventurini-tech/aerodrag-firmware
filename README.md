# AeroDrag Pro — Firmware ESP32-S3

Firmware ESP-IDF 5.2 per la board **Waveshare ESP32-S3-Touch-LCD-2.8**
(display ST7789T3 240×320, IMU QMI8658C, touch CST328). Pin map autorevole in
[`main/board_pins.h`](main/board_pins.h).

## Prerequisiti

```bash
# Installa ESP-IDF 5.2
git clone --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
cd ~/esp/esp-idf && git checkout v5.2.0
./install.sh esp32s3
source export.sh   # da aggiungere al .bashrc / .zshrc
```

## Build e flash

```bash
cd aerodrag-firmware

# Configura target
idf.py set-target esp32s3

# Build
idf.py build

# Flash (sostituisci /dev/ttyUSB0 con la tua porta)
idf.py -p /dev/ttyUSB0 flash monitor
```

## Schema di collegamento

La Waveshare ESP32-S3-Touch-LCD-2.8 ha già integrati:
- ✅ Display **ST7789T3** 240×320 (4-wire SPI: SCLK40/MOSI45/DC41/CS42/RST39/BL5)
- ✅ Touch **CST328** (I2C dedicato GPIO1/3, 0x1A) — non usato dal firmware
- ✅ IMU **QMI8658C** (I2C0 GPIO11/10, 0x6B)
- ✅ Batteria LiPo con gestione carica (ADC GPIO8, partitore ×3)

**Devi collegare esternamente:**

```
┌────────────────────────────────────────────────────────┐
│ SENSORE              │ PIN BOARD   │ Note               │
├────────────────────────────────────────────────────────┤
│ SDP810 SDA           │ GPIO 15     │ I2C1 (addr 0x25)   │
│ SDP810 SCL           │ GPIO 18     │ I2C1 (100 kHz)     │
│ SDP810 VCC           │ 3.3V        │                    │
│ SDP810 GND           │ GND         │                    │
└────────────────────────────────────────────────────────┘
```

La velocità a terra arriva dai **sensori esterni BLE** (CSC / sensore ruota)
gestiti dal central del firmware (`components/ble_central`), non da un GPS
dedicato. Il pulsante di calibrazione / cambio schermata è il **BOOT (GPIO0)**.

## Architettura task FreeRTOS

```
Core 0 (APP_CPU):
  task_pitot_imu   [prio 5, 100ms] — legge SDP810 + QMI8658, calcola CdA, notifica BLE Pitot+IMU
  task_gps         [prio 4,  50ms] — parsing NMEA, aggiorna velocità/quota
  task_ant         [prio 3, 300ms] — bridge ANT+ seriale, notifica BLE ANT

Core 1 (PRO_CPU):
  task_display     [prio 2, 200ms] — rendering UI sul display rotondo
  task_housekeeping[prio 1,  10s]  — batteria ADC, watchdog deep sleep
  nimble_host_task [NimBLE]        — stack BLE (lanciato da ble_server_init)
```

## Uso del pulsante fisico (BOOT, GPIO 0)

| Pressione | Azione |
|---|---|
| Breve (< 3s) | Cambia schermata display (CdA → Potenza → Status) |
| Lunga (≥ 3s) | **Calibrazione zero Pitot**: mantieni il device fermo per 5 secondi |

## Schermate display

| Schermata | Contenuto |
|---|---|
| **CdA** | Valore CdA grande, arco progressivo colorato, velocità, % batteria |
| **Potenza** | Watt totali, breakdown aero/rolling come barre verticali |
| **Status** | Batteria %, FC, cadenza, stato BLE |

## Calibrazione Pitot

La calibrazione zero del Pitot è necessaria ad ogni accensione in condizioni di vento diverso.

**Procedura:**
1. Posizionare la bici ferma all'aperto, fuori dal vento diretto
2. Tenere premuto il pulsante utente per ≥ 3 secondi
3. Attendere 5 secondi (il device sta mediando la lettura zero)
4. L'offset viene salvato in NVS (persiste dopo il riavvio)

## Struttura file

```
aerodrag-firmware/
├── CMakeLists.txt
├── sdkconfig.defaults          # Config ESP-IDF ottimizzata
├── partitions.csv              # 16MB: factory 4MB + sessions 8MB
├── main/
│   ├── CMakeLists.txt
│   ├── board_pins.h            # Pin map Waveshare Touch-LCD-2.8
│   ├── aerodrag_types.h        # Tipi condivisi tra componenti
│   └── main.c                  # Entry point, task launcher
└── components/
    ├── pitot/
    │   ├── sdp810.h            # Driver I2C Sensirion SDP810
    │   └── physics.h           # Motore fisico CdA in C
    ├── imu/
    │   └── qmi8658.h           # Driver QMI8658 + filtro complementare
    ├── ble/
    │   └── ble_server.h        # Server GATT NimBLE (servizio 0xAA00)
    ├── ble_central/
    │   └── ble_sensors.c       # Central: sensori esterni BLE + sensore ruota 0xBB00
    ├── wifi/
    │   └── wifi_coach.h        # Client WiFi/WebSocket verso il Pi (/device)
    ├── ota/
    │   └── ota_update.h        # OTA via URL (.bin)
    ├── display/
    │   └── display.h           # Driver ST7789T3 + renderer UI
    └── battery/
        └── battery.h           # ADC batteria + NVS calibrazione
```

## UUID BLE (devono coincidere con l'app React Native)

Contratto: **v0.2.3** — sorgente di verità in [`docs/CONTRACT.md`](docs/CONTRACT.md).

| UUID | Flags | Dati | Frequenza |
|---|---|---|---|
| `0000aa01-...-aa00` | R+N | float32[2]: pitotPa, staticPa | 10 Hz |
| `0000aa02-...-aa00` | R+N | float32[2]: pitchDeg, rollDeg | 10 Hz |
| `0000aa03-...-aa00` | R+N | float32[4]: tempC, humidity, altM, speedMs | 1 Hz |
| `0000aa04-...-aa00` | R+N | uint16+uint8+uint8: power, cad, hr (N=sentinella lap 0xFFFF) | poll 1 Hz |
| `0000aa05-...-aa00` | R+W | char[18] deviceId + char[32] athleteName (50 B) | on-connect |
| `0000aa06-...-aa00` | R | stringa versione FW NUL-terminated | once |
| `0000aa07-...-aa00` | W | URL OTA (.bin) | on-demand |
| `0000aa08-...-aa00` | R+W | float32[3]: massKg, crr, wheelCircM (12 B) | on-connect |
| `0000aa09-...-aa00` | N | float32[7]: cda, vAirMs, rho, pctAero(0-100), pAeroW, pRollingW, pGravityW (28 B) | 10 Hz |
| `0000aa0a-...-aa00` | N | uint8: battery % | 0.1 Hz |
| `0000aa0b-...-aa00` | R+W | sensor whitelist: count + N×(type + mac[6]) | on-pair |
| `0000aa0c-...-aa00` | N | float32[4]: speedMs, accelMs2, tempC, vibRMS (relay ruota Crr) | 10 Hz |
| `0000aa0d-...-aa00` | W | uint8: comando coast-down → sensore ruota | on-demand |
| `0000aa0e-...-aa00` | W+N | discovery sensori: W 0x01/0x00; N type+mac[6]+rssi+nameLen+name | on-pair |

## Troubleshooting

**SDP810 non trovato (I2C error)**
→ Verifica cablaggio GPIO 15/18 (I2C1). L'indirizzo default è 0x25 — modifica `PITOT_I2C_ADDR` in `board_pins.h` se necessario.

**QMI8658 non trovato**
→ Sulla Waveshare il pin AD0 è collegato a VDD quindi l'indirizzo è 0x6B. Alcune revisioni usano 0x6A — prova entrambi.

**Display bianco o nero**
→ Verifica il reset hardware (GPIO 39). Il ST7789T3 richiede reset >10ms.

**BLE non visibile sul telefono**
→ Controlla nei log che NimBLE sia partito (`sync_cb` chiamato). Il device si chiama `"AeroDrag Pro"`.

**Batteria sempre 0%**
→ ADC1_CH7 è GPIO 8 (partitore ×3). Se non è collegato la lettura è 0. Con batteria LiPo collegata al connettore MX1.25 funziona automaticamente.
