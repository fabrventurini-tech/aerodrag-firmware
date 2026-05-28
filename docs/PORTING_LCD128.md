# Porting per Waveshare ESP32-S3-LCD-1.28

## Modifiche rispetto alla versione 1.46

### 1. Display (cambio principale)

| Parametro | LCD-1.46 | LCD-1.28 |
|---|---|---|
| Driver chip | GC9D01N | **GC9A01A** |
| Risoluzione | 412×412 | **240×240** |
| Backlight pin | GPIO15 | **GPIO2** |
| Init sequence | 9 comandi | **43 comandi** (GC9A01A più elaborato) |
| SPI clock max | 40 MHz | **80 MHz** |

### 2. Pin I2C (cambio critico)

| Bus | LCD-1.46 | LCD-1.28 |
|---|---|---|
| I2C0 (Touch+IMU) | GPIO39/40 | **GPIO6/7** |
| I2C1 (Pitot) | GPIO41/42 | **GPIO15/16** |

### 3. Battery ADC

| Parametro | LCD-1.46 | LCD-1.28 |
|---|---|---|
| GPIO | GPIO4 (divider /2) | **GPIO1 (divider /3)** |
| ADC channel | ADC1_CH3 | **ADC1_CH0** |
| Formula Vbat | adc × 2 × (3.3/4096) | **adc × 3 × (3.3/4096)** |

### 4. PSRAM

| | LCD-1.46 | LCD-1.28 |
|---|---|---|
| PSRAM | 8MB OPI | **2MB OPI** |
| Framebuffer | 340KB (PSRAM) | **115KB (internal RAM o PSRAM)** |

---

## Schema di collegamento aggiornato

```
┌──────────────────────────────────────────────────────────┐
│ SENSORE              │ PIN BOARD   │ Note                 │
├──────────────────────────────────────────────────────────┤
│ SDP810 SDA           │ GPIO 15     │ I2C1 (extern)        │
│ SDP810 SCL           │ GPIO 16     │ I2C1 (extern)        │
│ SDP810 VCC           │ 3.3V        │                      │
│ SDP810 GND           │ GND         │                      │
├──────────────────────────────────────────────────────────┤
│ GPS TX               │ GPIO 44     │ UART1 RX             │
│ GPS RX               │ GPIO 43     │ UART1 TX             │
│ GPS VCC              │ 3.3V        │                      │
│ GPS GND              │ GND         │                      │
├──────────────────────────────────────────────────────────┤
│ ANT+ bridge TX       │ GPIO 18     │ UART2 RX (opz.)      │
│ ANT+ bridge RX       │ GPIO 17     │ UART2 TX (opz.)      │
├──────────────────────────────────────────────────────────┤
│ Pulsante calibraz.   │ GPIO 0      │ BOOT button on board │
└──────────────────────────────────────────────────────────┘

On-board già collegato (nessun cablaggio necessario):
  ✅ Display GC9A01A  → GPIO 8(DC), 9(CS), 10(CLK), 11(MOSI), 14(RST), 2(BL)
  ✅ Touch CST816S    → GPIO 6(SDA), 7(SCL), 5(INT), 13(RST)
  ✅ IMU QMI8658      → GPIO 6(SDA), 7(SCL), 4(INT1), 3(INT2)
  ✅ Battery ADC      → GPIO 1 (200K+100K divider, Vbat/3)
  ✅ Battery charger  → ETA6096, MX1.25 connector
```

---

## File modificati

| File | Modifica |
|---|---|
| `main/board_pins.h` | GPIO I2C0 6/7, BL GPIO2, ADC GPIO1, Pitot I2C1 su 15/16 |
| `components/display/display.h` | Init GC9A01A (43 cmd), FB 240×240, UI riscalata |
| `components/battery/battery.h` | Formula Vbat × 3 (divider /3), canal ADC0 |
| `main/main.c` | ADC_CHANNEL_0, I2C0 su PIN_IMU_SDA/SCL |
| `sdkconfig.defaults` | PSRAM size 2MB |

---

## Schermate display (240×240)

### Screen 0 — CdA (principale)
- Arco progressivo 270° colorato (verde/ambra/rosso) su r=110px
- Valore CdA "0.XXX" al centro, scala adattata a 240px
- Velocità in basso, potenza in alto-sinistra
- Batteria arc sottile in cima, dot BLE in alto-destra

### Screen 1 — Potenza
- Watt totali grande in alto
- 3 barre verticali: Aero (rosso) | Rolling (teal) | Altro (grigio)
- Percentuale sotto ogni barra

### Screen 2 — Status
- % batteria, FC, cadenza
- Barra GPS (verde=3D fix, ambra=2D, rosso=no fix)
- Angolo busto IMU

---

## Calibrazione (invariata)

Tenere premuto il **pulsante BOOT** (GPIO0) per ≥ 3 secondi con il device fermo.
Il firmware media il Pitot per 5 secondi e salva l'offset in NVS.
