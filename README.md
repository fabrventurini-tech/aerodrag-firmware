# AeroDrag Pro — Firmware ESP32-S3

Firmware ESP-IDF 5.3 per la board **Waveshare ESP32-S3-Touch-LCD-2.8**.

## Prerequisiti

```bash
# Installa ESP-IDF 5.3 (la CI usa 5.3.2)
git clone --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
cd ~/esp/esp-idf && git checkout v5.3.2
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

## Hardware integrato sulla board

La Waveshare ESP32-S3-Touch-LCD-2.8 integra:
- Display **ST7789T3** 240×320 (SPI: SCLK GPIO40, MOSI GPIO45, DC GPIO41, CS GPIO42, **RST GPIO39**, **BL GPIO5**)
- Touch **CST328** (I2C 0x1A su **GPIO1 SDA / GPIO3 SCL**) — **non usato dal firmware** (i due controller I2C dell'S3 sono entrambi occupati da IMU e pitot)
- IMU **QMI8658C** (I2C0 su **GPIO11 SDA / GPIO10 SCL**, addr **0x6B**, probe automatico anche su 0x6A)
- Batteria LiPo con gestione carica + lettura ADC

## Collegamenti esterni

```
┌──────────────────────────────────────────────────────────────┐
│ SENSORE / SEGNALE     │ PIN BOARD          │ Note             │
├──────────────────────────────────────────────────────────────┤
│ SDP810 SDA            │ GPIO 15            │ I2C1 (0x25)      │
│ SDP810 SCL            │ GPIO 18            │ I2C1 (0x25)      │
│ SDP810 VCC            │ 3.3V               │                  │
│ SDP810 GND            │ GND                │                  │
├──────────────────────────────────────────────────────────────┤
│ Batteria (Vbat)       │ ADC GPIO 8         │ ADC1_CH7, ×3     │
│ Pulsante (BOOT)       │ GPIO 0             │ active low       │
└──────────────────────────────────────────────────────────────┘
```

La **velocità a terra** NON arriva da un GPS: proviene dai **sensori esterni BLE**
(CSC / sensore ruota) gestiti da `components/ble_central`. Non esiste alcun
componente GPS/u-blox/NMEA nel firmware.

## Architettura task FreeRTOS

```
task_pitot_imu   — legge SDP810 + QMI8658, calcola CdA, notifica BLE Pitot+IMU+Physics
ble_central      — scan/connessione ai sensori esterni BLE (CSC/power/HR/ruota)
task_display     — rendering UI sul display ST7789T3
task_housekeeping— batteria ADC, watchdog/sleep
nimble_host_task — stack BLE (server GATT + central)
```

## Uso del pulsante fisico (BOOT, GPIO 0)

| Pressione | Azione |
|---|---|
| Breve | Cambia schermata display |
| Lunga (≥ 3s) | **Calibrazione zero Pitot** (device fermo per ~5 s) |
| Molto lunga (≥ 5s) | Funzioni avanzate / cambio modalità |

## Calibrazione Pitot

1. Posizionare la bici ferma all'aperto, fuori dal vento diretto.
2. Tenere premuto il pulsante per ≥ 3 secondi.
3. Attendere ~5 secondi (media della lettura zero).
4. L'offset viene salvato in NVS (persiste dopo il riavvio).

## Struttura file

```
aerodrag-firmware/
├── CMakeLists.txt
├── sdkconfig.defaults
├── partitions.csv
├── main/
│   ├── board_pins.h            # Pin map Waveshare 2.8
│   ├── aerodrag_types.h        # Tipi condivisi
│   └── main.c                  # Entry point, task launcher
└── components/
    ├── pitot/
    │   ├── sdp810.h            # Driver I2C Sensirion SDP810
    │   └── physics.h           # Motore fisico CdA in C
    ├── imu/
    │   └── qmi8658.h           # Driver QMI8658C + filtro complementare
    ├── ble/
    │   └── ble_server.h        # Server GATT NimBLE (caratteristiche app)
    ├── ble_central/
    │   └── ble_sensors.c/.h    # Central: sensori esterni BLE (velocità ruota)
    ├── wifi/
    │   └── wifi_coach.h        # Bridge WiFi/WebSocket al coach (Pi)
    ├── ota/
    │   └── ota_update.h        # OTA via HTTP (slot A/B)
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
| `0000aa09-...-aa00` | N | float32[7]: cda, vAirMs, rho, pctAero, pAeroW, pRollingW, pGravityW (28 B) | 10 Hz |
| `0000aa0a-...-aa00` | N | uint8: battery % | 0.1 Hz |
| `0000aa0b-...-aa00` | R+W | sensor whitelist: count + N×(type + mac[6]) | on-pair |
| `0000aa0c-...-aa00` | N | float32[4]: speedMs, accelMs2, tempC, vibRMS (relay ruota Crr) | 10 Hz |
| `0000aa0d-...-aa00` | W | uint8: comando coast-down → sensore ruota | on-demand |
| `0000aa0e-...-aa00` | W+N | **SENSOR_SCAN**: W 1B (0x01 start / 0x00 stop discovery); N per sensore scoperto: uint8 type + uint8 mac[6] + int8 rssi + uint8 nameLen + char name[nameLen] | on-pair |

## Troubleshooting

**SDP810 non trovato (I2C error)**
→ Verifica cablaggio **GPIO 15 (SDA) / GPIO 18 (SCL)** su I2C1. Indirizzo default 0x25 (con ADDR a VDD diventa 0x26 — modifica `PITOT_I2C_ADDR` in `board_pins.h`).

**QMI8658 non trovato**
→ IMU su I2C0 **GPIO 11 (SDA) / GPIO 10 (SCL)**. Indirizzo 0x6B; alcune revisioni usano 0x6A (il driver prova entrambi).

**Display bianco o nero**
→ Verifica il reset hardware **GPIO 39** (il pannello richiede reset >10 ms) e il backlight su GPIO 5.

**BLE non visibile sul telefono**
→ Controlla nei log che NimBLE sia partito (`sync_cb` chiamato).

**Batteria sempre 0%**
→ La lettura batteria usa l'**ADC su GPIO 8 (ADC1_CH7)** con partitore ×3. Se la LiPo non è collegata la lettura è 0.
