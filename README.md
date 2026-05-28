# AeroDrag Pro — Firmware ESP32-S3

Firmware ESP-IDF 5.2 per la board **Waveshare ESP32-S3-Touch-LCD-1.46**.

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

La Waveshare ESP32-S3-Touch-LCD-1.46 ha già integrati:
- ✅ Display GC9D01N (SPI)
- ✅ Touch CST816D (I2C0)
- ✅ IMU QMI8658 (I2C0)
- ✅ Batteria LiPo con gestione carica

**Devi collegare esternamente:**

```
┌────────────────────────────────────────────────────────┐
│ SENSORE              │ PIN BOARD   │ Note               │
├────────────────────────────────────────────────────────┤
│ SDP810 SDA           │ GPIO 41     │ I2C1               │
│ SDP810 SCL           │ GPIO 42     │ I2C1               │
│ SDP810 VCC           │ 3.3V        │                    │
│ SDP810 GND           │ GND         │                    │
├────────────────────────────────────────────────────────┤
│ GPS u-blox TX        │ GPIO 44     │ UART1 RX           │
│ GPS u-blox RX        │ GPIO 43     │ UART1 TX           │
│ GPS VCC              │ 3.3V        │                    │
│ GPS GND              │ GND         │                    │
├────────────────────────────────────────────────────────┤
│ ANT+ bridge TX       │ GPIO 18     │ UART2 RX (opt.)    │
│ ANT+ bridge RX       │ GPIO 17     │ UART2 TX (opt.)    │
├────────────────────────────────────────────────────────┤
│ Pulsante calibr.     │ GPIO 1      │ pull-up interno    │
└────────────────────────────────────────────────────────┘
```

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

## Uso del pulsante fisico (GPIO 1)

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
│   ├── board_pins.h            # Pin map Waveshare 1.46
│   ├── aerodrag_types.h        # Tipi condivisi tra componenti
│   └── main.c                  # Entry point, task launcher
└── components/
    ├── pitot/
    │   ├── sdp810.h            # Driver I2C Sensirion SDP810
    │   └── physics.h           # Motore fisico CdA in C
    ├── imu/
    │   └── qmi8658.h           # Driver QMI8658 + filtro complementare
    ├── gps/
    │   └── gps_nmea.h          # Parser NMEA + configurazione u-blox
    ├── ble/
    │   └── ble_server.h        # Server GATT NimBLE con 4 caratteristiche
    ├── display/
    │   └── display.h           # Driver GC9D01N + renderer UI
    └── battery/
        └── battery.h           # ADC batteria + NVS calibrazione
```

## UUID BLE (devono coincidere con l'app React Native)

| UUID | Dati | Frequenza |
|---|---|---|
| `0000aa01-...-aa00` | float32[2]: pitotPa, staticPa | 10 Hz |
| `0000aa02-...-aa00` | float32[2]: pitchDeg, rollDeg | 10 Hz |
| `0000aa03-...-aa00` | float32[3]: tempC, humidity, altM | 1 Hz |
| `0000aa04-...-aa00` | uint16+uint8+uint8: power, cad, hr | 4 Hz |

## Troubleshooting

**SDP810 non trovato (I2C error)**
→ Verifica cablaggio GPIO 41/42. L'indirizzo default è 0x25. Con ADDR pin a GND diventa 0x25, a VDD diventa 0x26 — modifica `PITOT_I2C_ADDR` in `board_pins.h`.

**QMI8658 non trovato**
→ Sulla Waveshare il pin AD0 è collegato a VDD quindi l'indirizzo è 0x6B. Alcune revisioni usano 0x6A — prova entrambi.

**Display bianco o nero**
→ Verifica il reset hardware (GPIO 14). Il GC9D01N richiede reset >10ms.

**BLE non visibile sul telefono**
→ Controlla nei log che NimBLE sia partito (`sync_cb` chiamato). Il device si chiama `"AeroDrag Pro"`.

**Batteria sempre 0%**
→ ADC1_CH3 è GPIO 4. Se non è collegato la lettura è 0. Con batteria LiPo collegata al connettore MX1.25 funziona automaticamente.
