# AeroDrag — Interface Contract

```
contract: v0.1.1
owner:    aerodrag-firmware (questa repo è la fonte di verità unica)
status:   ratified
date:     2026-06-16
```

Questo documento è l'**unica fonte di verità** per le interfacce fra i quattro
componenti AeroDrag. Le repo figlie **implementano** questo contratto, non lo
modificano. Ogni cambiamento d'interfaccia passa dalle *seam issue*, viene
concordato fra le due figlie coinvolte e infine **ratificato qui** con un bump
SemVer.

SemVer del contratto:
- **MAJOR** — cambiamento incompatibile di un payload/significato di campo.
- **MINOR** — aggiunta retro-compatibile (nuovo campo opzionale, nuova caratteristica).
- **PATCH** — chiarimenti, vincoli, documentazione senza impatto sul wire.

---

## 1. Architettura e confini

```
   FIRMWARE (ESP32-S3, C)
   sensori Pitot+IMU+ANT/BLE central, calcola CdA (verità del CdA)
        │                                   │
   (A) BLE GATT 0xAA00                  (B) WiFi WebSocket /device
   binario LE, 10 Hz                    JSON, 2 Hz
        ▼                                   ▼
   NEW (app TS, Expo)  ──(C) WS /coach──►  PI (gateway JS)
   engine.ts = solo       JSON 2 Hz ↑      server.js :8080
   fallback/sim           cmd ↓            - /device (da firmware)
        ▲                                  - /coach  (frames+eventi+cmd)
        │ (F) schema condiviso             - /api/sessions, /status (REST)
        │ (frame + sessione)               - /fw.bin (OTA)
        ▼                                       │
   COACH (Electron JS)  ◄──(E) HTTP POST /receive :8081── (sessione JSON)
   pc-receiver :8081, dashboard
```

| # | Confine | Trasporto | Sezione |
|---|---------|-----------|---------|
| A | firmware → new | BLE GATT (svc `0xAA00`) | §2 |
| B | firmware → pi | WebSocket `/device` (JSON) | §3 |
| C | new → pi | WebSocket `/coach` (JSON) | §3 |
| D | pi → new / pi → coach | WebSocket `/coach` (cmd + eventi) | §4 |
| E | pi → coach | HTTP `/receive` :8081 (sessione) | §5 |
| F | coach ↔ new | schema dati condiviso | §5 |

Tutti i multi-byte sono **little-endian**; i float sono **IEEE-754 32 bit**.
Le unità sono SI salvo dove indicato (velocità di rete in km/h).

---

## 2. Confine A — firmware → new (BLE GATT)

Servizio primario: `0000aa00-0000-1000-8000-00805f9b34fb` (device name `AeroDrag Pro`).
MTU richiesto ≥ 53 (si negozia 185): PHYSICS=28 B, READ IDENTITY=50 B.

| CHR | UUID (suffisso) | Flags | Freq | Bytes | Payload |
|-----|-----------------|-------|------|-------|---------|
| PITOT    | `aa01` | R+N | 10 Hz | 8  | `float pitotPa, staticPa` [Pa] |
| IMU      | `aa02` | R+N | 10 Hz | 8  | `float pitchDeg, rollDeg` [°] |
| ENV      | `aa03` | R+N | 1 Hz  | 16 | `float tempC[°C], humidityPct[0-100], altM[m], speedMs[m/s]` |
| ANT      | `aa04` | R+N | poll 1 Hz | 4 | `uint16 powerW, uint8 cadRpm, uint8 hrBpm`. NOTIFY = **sentinella lap** (`powerW=0xFFFF`); i dati reali via READ |
| IDENTITY | `aa05` | R+W | on-connect | 50 | `char deviceId[18]` ("AA:BB:..\0") + `char athleteName[32]` (≤31 + NUL) |
| VERSION  | `aa06` | R   | once | var | stringa FW NUL-terminated, es. `"1.0.0 (a972c56)"` |
| OTA_URL  | `aa07` | W   | on-demand | ≤256 | URL `http` del `.bin` → avvia OTA |
| CONFIG   | `aa08` | R+W | on-connect | 12 | `float massKg, crr, wheelCircM` |
| PHYSICS  | `aa09` | N   | 10 Hz | 28 | `float cda[m²], vAirMs[m/s], rho[kg/m³], pctAero[%0-100], pAeroW, pRollingW, pGravityW` |
| BATTERY  | `aa0a` | N   | 0.1 Hz | 1 | `uint8 batteryPct [0-100]` |

### Vincoli `CONFIG` (0xaa08)
- **12 byte**: `massKg`∈[33,200], `crr`∈[0.001,0.025], `wheelCircM`∈[1.0,2.5].
- Il device è **proprietario** della circonferenza ruota (`wheelCircM`): l'app la
  **legge** in connect; massa e Crr sono guidati dall'app (profili atleta).
- Fuori range → errore ATT, **nessun campo** scritto (clampare lato app).
- Retro-compatibilità: una WRITE di **8 byte** aggiorna solo `massKg`+`crr`.

### Vincoli `PHYSICS` (0xaa09)
- `pctAero` è una **percentuale 0–100** (stessa scala di WiFi/Pi/app).
- Tutti i campi 0 se la misura non è valida (`cda>0.01 && vAirMs>0.5` lato app).
- Il **CdA del firmware è la verità**. `engine.ts` nell'app resta solo per
  *sim mode* / firmware legacy e DEVE replicare le formule di §6.

---

## 3. Confini B/C — frame di telemetria (firmware/app → pi, WebSocket)

Endpoint device→pi: `ws://192.168.8.1:8080/device`.
Endpoint app→pi (coach): `ws://<pi>:8080/coach`.

**Handshake** (alla connessione):
```json
{ "type": "hello", "device": "AA:BB:CC:DD:EE:FF", "athlete": "Mario Rossi", "fw": "1.0.0" }
```
(`fw` inviato dal firmware; opzionale dall'app.)

**Frame dati** — **2 Hz**, inviato solo se la fisica è valida:
```json
{
  "t": 1716892800000,
  "device": "AA:BB:CC:DD:EE:FF",
  "athlete": "Mario Rossi",
  "lap": 2,
  "CdA": 0.2451,
  "pwr": 207,
  "spd": 37.8,
  "hr": 145,
  "cad": 90,
  "wind": 1.8,
  "battery": 85,
  "pctAero": 87.0,
  "pitch": 2.3,
  "rho": 1.225,
  "lapEvent": false
}
```

| Campo | Tipo | Unità | Note |
|-------|------|-------|------|
| `t` | number | ms | timestamp sorgente (ms da boot per il firmware) |
| `device` | string | — | MAC BLE; chiave di sessione |
| `athlete` | string | — | nome atleta (sanificato: niente `"` `\` o ctrl) |
| `lap` | number | — | giro corrente, ≥1 |
| `CdA` | number | m² | valido in **[0.05, 1.0]**; fuori → scartato dal Pi |
| `pwr` | number | W | intero |
| `spd` | number | **km/h** | 1 decimale |
| `hr` | number | bpm | 0 se assente |
| `cad` | number | rpm | 0 se assente |
| `wind` | number | m/s | `max(0, vAir − vGround)` |
| `battery` | number | % | 0–100 |
| `pctAero` | number | **% 0–100** | 1 decimale |
| `pitch` | number | ° | inclinazione bici |
| `rho` | number | kg/m³ | densità aria |
| `lapEvent` | boolean | — | `true` una sola volta al cambio giro |

Vincoli: frame > 127 byte rifiutati lato firmware per i comandi; sessione
salvata solo con ≥ 20 frame; il Pi campiona 1 punto ogni 5 frame (≤ 2000 pt/giro).

---

## 4. Confine D — pi → new / pi → coach (eventi e comandi, WS `/coach`)

**Comandi** (coach/app → device, via Pi), payload ≤ 127 byte:
```json
{ "type": "cmd", "action": "start" | "stop" | "lap", "deviceId"?: "AA:.." }
{ "type": "cmd", "action": "ota", "deviceId": "AA:..", "url": "http://192.168.8.1:8080/fw.bin" }
{ "type": "sync_now" }
{ "type": "lap_note", "deviceId": "AA:..", "lapNum": 1, "text": "..." }
```
(`deviceId` assente = broadcast. L'app gestisce solo `start`/`stop`/`lap`.)

**Eventi** (pi → coach/app), principali:
`history`, `athletes`, `device_connected`, `device_disconnected`,
`athlete_update`, `lap_event`, `session_saved`, `session_synced`,
`sync_status`, `sync_progress`, `sync_complete`, `cmd_echo`, `cmd_error`,
più il **frame live** (oggetto §3 senza `type`, con `serverTs` aggiunto dal Pi).

**REST** (coach dashboard): `GET /status`, `GET /api/sessions`,
`GET /api/sessions/{id}`, `GET /fw.bin`.

---

## 5. Confini E/F — sessione persistita (pi → coach, schema condiviso con new)

Il Pi invia la sessione conclusa: `POST http://192.168.7.2:8081/receive?filename=...`
con nome `session_{ts}_{deviceIdHex}.json`. Schema:

```json
{
  "ts": 1716892800000,
  "deviceId": "AA:BB:CC:DD:EE:FF",
  "athleteName": "Mario Rossi",
  "laps": [
    {
      "lapNum": 1,
      "startTs": 1716892800000,
      "durationS": 120,
      "avgCdA": 0.2451,
      "bestCdA": 0.2340,
      "avgPowerW": 210,
      "avgSpeedKmh": 38.5,
      "avgHr": 145,
      "avgCad": 90,
      "avgWindMs": 1.80,
      "notes": "",
      "pts": [
        { "s": 0, "CdA": 0.2451, "pwr": 207, "spd": 37.8,
          "hr": 145, "wind": 1.80, "cad": 90, "pitch": 2.3, "rho": 1.225 }
      ]
    }
  ]
}
```

Questo schema è **condiviso** fra Pi (produttore), coach (sink/visualizzatore) e
app (tipi TS in `aerodrag-new/src/store`). Modifiche → seam `coach↔new`.

**Vincolo nome file (v0.1.1):** il `deviceId` è **obbligatorio e identificabile**.
Il Pi **NON DEVE** emettere sessioni con `deviceId` vuoto o non valido: tali
sessioni vanno **scartate a monte**, non inviate al receiver. Il nome file è
**sempre** `session_{ts}_{deviceIdHex}.json` con `deviceIdHex` = cifre esadecimali
non vuote (MAC senza `:`). Nessun token di fallback (`unknown`) è ammesso: il
`/receive` del coach valida con `^session_\d+_[A-Fa-f0-9]+\.json$` e rifiuta il resto.

---

## 6. Modello fisico canonico (riferimento per le doppie implementazioni)

L'implementazione C in `aerodrag-firmware/components/pitot/physics.h` è
**canonica**. Qualsiasi reimplementazione (es. `aerodrag-new/src/physics/engine.ts`,
usata solo in sim/fallback) deve corrispondere:

```
rho     = f(tempC, humidityPct, altM)          # ISO 2533 + Magnus (umidità)
v_air   = sqrt(2 · max(0, pitot − offset) / rho)
p_roll  = crr · mass · g · v_ground
p_grav  = mass · g · sin(pitch) · v_ground
p_mech  = power · (1 − MECH_EFF)               # MECH_EFF = 0.975
p_aero  = max(0, power − p_roll − p_grav − p_mech)
CdA     = p_aero / (0.5 · rho · v_air³)         # se v_air³>1 e power>20 W
pctAero = clamp(p_aero / power · 100, 0, 100)   # PERCENTUALE 0-100
```
Costanti: `g = 9.80665`, `RHO_STD = 1.225`, CdA valido in `[0.10, 0.60]` (device).

---

## 7. Versioni componenti (al contratto v0.1.0)

| Componente | Versione |
|-----------|----------|
| firmware (`FW_VERSION_STR`) | 1.0.0 |
| pi server | 1.0.0 (v4 multi-atleta) |
| pi pc-receiver | 1.0.0 |
| coach (Electron) | 1.0.0 |
| coach pc-receiver | 1.0.0 |
| new (app) | 1.0.0 (Expo ~56) |

---

## 8. Changelog

### v0.1.1 — 2026-06-17
Chiarimento ratificato dalla seam `pi↔coach` (coach#3 / pi#6), nessuna rottura:
- **§5 nome file sessione**: `deviceId` obbligatorio/identificabile. Il Pi scarta a
  monte le sessioni con `deviceId` vuoto/non valido e **non** usa il fallback
  `unknown` (fuori contratto). Filename sempre `session_{ts}_{deviceIdHex}.json`.

### v0.1.0 — 2026-06-16
Primo contratto ratificato. Risolte 5 divergenze fra implementazioni:
1. **CONFIG 0xaa08** portata a **12 B** (`massKg, crr, wheelCircM`) con READ
   abilitata sul firmware; WRITE 8 B retro-compatibile.
2. **pctAero** unificato a **0–100** su tutti i confini (BLE incluso: prima 0–1).
3. **Frame di rete** fissato a **2 Hz** (BLE notify resta 10 Hz); throttle nel firmware.
4. **pitch/rho/lapEvent** aggiunti al frame `new → pi` (già presenti nel firmware,
   registrati dal Pi).
5. **physics.h (C)** dichiarato canonico; `engine.ts` solo fallback/sim.
