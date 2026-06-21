# AeroDrag — Interface Contract

```
contract: v0.3.0
owner:    aerodrag-firmware (questa repo è la fonte di verità unica)
status:   ratified
date:     2026-06-21
```

> **v0.3.0 — Fase 1: recisione del live app↔Pi.** Il telefono resta **solo-BLE** verso
> il proprio ESP32 (così non lascia la rete internet); l'**ESP32 è l'uplink WiFi
> primario** verso il Pi. **Lato firmware (madre) è già implementato** (`COACH_LINK
> 0xaa0f`, `TIME 0xaa10`, `tUtc`). Le figlie si adeguano come consumer: `aerodrag-new`
> (issue dedicata), `aerodrag-pi`/`aerodrag-coach` invariati sul wire (vedi §3/§4).
> Confine **H** (cloud, §7) resta **riservato** per la Fase 2. Changelog in §9.

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
   FIRMWARE (ESP32-S3, C) — gateway/uplink di ogni atleta
   sensori Pitot+IMU+ANT/BLE central, calcola CdA (verità del CdA)
        │                                        │
   (A) BLE GATT 0xAA00                       (B) WiFi WS /device  ◄─ UPLINK PRIMARIO
   binario LE, 10 Hz                         JSON 2 Hz ↑ ; cmd coach ↓
   + COACH_LINK 0xaa0f (cmd coach → app)           │ (relay → app via 0xaa0f)
        ▼                                          ▼
   NEW (app TS, Expo)                          PI (gateway JS) server.js :8080
   SOLO BLE verso il proprio ESP32             - /device (firmware: telemetria + cmd)
   (mantiene la rete internet del telefono)    - /coach  (app: DEPRECATO → §3)
        ╎                                       - /api/sessions, /status (REST)
        ╎ (C) WS /coach — DEPRECATO             - /fw.bin (OTA)
        ╎ fallback legacy, non più primario          │
                                                      │
   COACH (Electron JS)  ◄──(E) HTTP POST /receive :8081── (sessione JSON)
   pc-receiver :8081, dashboard

   (H) client ↔ cloud — RISERVATO (Fase 2, §7): archivio canonico. NON implementato.
```

| # | Confine | Trasporto | Sezione |
|---|---------|-----------|---------|
| A | firmware → new | BLE GATT (svc `0xAA00`) — **app solo-BLE** | §2 |
| B | firmware → pi | WebSocket `/device` (JSON) — **uplink primario** | §3 |
| C | ~~new → pi~~ | ~~WebSocket `/coach`~~ — **DEPRECATO** (fallback legacy) | §3 |
| D | pi → new / pi → coach | cmd a new **via firmware** (`0xaa0f`); eventi a coach su `/coach` | §4 |
| E | pi → coach | HTTP `/receive` :8081 (sessione) | §5 |
| F | coach ↔ new | schema dati condiviso | §5 |
| G | firmware ↔ wheel | BLE GATT (svc `0xBB00`) | §2 |
| H | client ↔ cloud | REST autenticato — **RISERVATO**, non implementato | §7 |

Componente gestito aggiuntivo: **`wheel-fw/`** (firmware sensore ruota Crr,
nRF52840) — repo figlia che vive nella repo madre, vedi confine G.

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
| SENSOR_WHITELIST | `aa0b` | R+W | on-pair | var | `uint8 count` + count×(`uint8 type` + `uint8 mac[6]`). type: 1=power,2=csc,3=hr,4=wheel. Firmware central connette **solo** a questi MAC |
| WHEEL_STREAM | `aa0c` | N | 10 Hz | 16 | `float speedMs, accelMs2, tempC, vibRMS` — relay dei dati grezzi del sensore ruota durante la calibrazione Crr |
| WHEEL_CMD | `aa0d` | W | on-demand | 1 | `uint8` comando coast-down: 0x01 indoor, 0x02 outdoor-A, 0x03 outdoor-B, 0xFF cancel → inoltrato dal firmware al sensore ruota |
| SENSOR_SCAN | `aa0e` | W+N | on-pair | var | **W** 1B: 0x01 start / 0x00 stop discovery. **N** (1 entry per sensore scoperto): `uint8 type` + `uint8 mac[6]` + `int8 rssi` + `uint8 nameLen` + `char name[nameLen]` |
| COACH_LINK | `aa0f` | N | event-driven | 2 | **(v0.3.0)** relay coach→app: `uint8 type` + `uint8 arg`. Il firmware inoltra all'app i **comandi coach** (`start`/`stop`/`lap`) ricevuti sul suo `/device`. Vedi sotto |
| TIME | `aa10` | R+W | on-connect | 8 | **(v0.3.0)** `uint64 epochMs` (UTC, ms, **little-endian**, 8 byte esatti): orologio oggettivo del device. **W** lo imposta (orologio di sistema), **R** lo legge (`0` se non impostato). Vedi "Timebase oggettivo" |

### Vincoli `CONFIG` (0xaa08)
- **12 byte**: `massKg`∈[33,200], `crr`∈[0.001,0.025], `wheelCircM`∈[1.0,2.5].
- **L'app è autorevole** per **tutti e tre** i parametri (`massKg`, `crr`,
  `wheelCircM`): li calcola/imposta e li **scrive** (massa+Crr dai profili atleta;
  `crr` dalla calibrazione coast-down; `wheelCircM` dall'impostazione utente). Il
  firmware li **persiste in NVS e li integra solo nella fisica** (e usa
  `wheelCircM` per la velocità da CSC) — non li calcola né li possiede. La READ è
  solo informativa/echo (es. valore di default al primo avvio).
- Fuori range → errore ATT, **nessun campo** scritto (clampare lato app).
- Retro-compatibilità: una WRITE di **8 byte** aggiorna solo `massKg`+`crr`.

### Vincoli `PHYSICS` (0xaa09)
- `pctAero` è una **percentuale 0–100** (stessa scala di WiFi/Pi/app).
- Tutti i campi 0 se la misura non è valida (`cda>0.01 && vAirMs>0.5` lato app).
- Il **CdA del firmware è la verità**. `engine.ts` nell'app resta solo per
  *sim mode* / firmware legacy e DEVE replicare le formule di §6.

### Identità & pairing `IDENTITY` (0xaa05) — (v0.1.4)
- Il `device_id` esposto in READ su `0xaa05` (MAC `"AA:BB:CC:DD:EE:FF"`) è
  l'**identità canonica** del device e l'**unica** sorgente del campo `device`
  dei frame telemetria (§3). Il consumer (app) **DEVE leggerlo alla connessione**
  e usarlo come `device`; coincide con quello che il firmware usa in WiFi diretto.
- **Disaccoppiare identità e trasporto BLE**: l'identificatore di connessione BLE
  è platform-specific (su iOS è un UUID CoreBluetooth, **non** il MAC), quindi
  **NON** va usato come identità coach né confrontato col MAC. L'identità arriva
  sempre da `0xaa05`.
- **Pairing — QR come fonte primaria (v0.2.0)**: l'accoppiamento parte dalla
  scansione del QR `AERODRAG://PAIR/<MAC>`, che fornisce il **MAC autorevole**.
  Poiché su iOS non ci si può connettere per MAC, l'app: scansiona `0xAA00` →
  si connette a un candidato → **legge `device_id` da `0xaa05`** → tiene la
  connessione **solo se `== MAC del QR`** (su Android vale anche la fast-path
  `device.id == MAC`). La selezione manuale dalla lista BLE è un fallback.

### Sensori esterni: whitelist & relay (v0.2.0)
Principio: i sensori esterni (power `0x1818`, CSC `0x1816`, HR `0x180D`, sensore
ruota Crr `0xBB00`) si **bondano SOLO al firmware**. L'app è **broker di pairing
+ specchio**: non apre connessioni dati ai sensori (elimina il cross-talk dalle
bici vicine; il firmware resta l'unica fonte di verità).
- **Pairing**: l'app scansiona per far **scegliere** il sensore all'utente, ne
  ottiene il MAC e scrive l'elenco autorizzato in **`SENSOR_WHITELIST` (0xaa0b)**.
  Il central del firmware si connette **esclusivamente** ai MAC in whitelist
  (non più "il primo trovato").
- **Discovery dal firmware (v0.2.2) — fix iOS**: l'app **non può ricavare il MAC**
  dei sensori dal proprio stack BLE (su iOS `device.id` è un UUID CoreBluetooth,
  non il MAC). Quindi è il **firmware** (che vede i MAC reali) a fare la scoperta:
  l'app scrive `0x01` su **`SENSOR_SCAN` (0xaa0e)** → il firmware scansiona (senza
  connettersi) e **notifica una entry per sensore scoperto** `{type, mac[6], rssi,
  name}`; l'app mostra la lista, l'utente sceglie, l'app scrive i MAC in `0xaa0b`.
  Discovery con auto-stop (~15 s) o stop esplicito (`0x00`). Funziona su iOS e
  Android (il MAC arriva dal firmware, non dal BLE dell'app).
- **Crr — calibrazione nell'app, dati via firmware**: il sensore ruota si bonda
  al firmware; durante il coast-down il firmware **relaya** lo stream grezzo su
  **`WHEEL_STREAM` (0xaa0c)** e inoltra i comandi dell'app da **`WHEEL_CMD`
  (0xaa0d)** al sensore. L'app esegue il fit e **calcola il `crr`**, poi lo
  **scrive in `CONFIG 0xaa08`** (vedi sopra). Il `wheelCircM` è impostato
  dall'app e scritto in `CONFIG`. Né `crr` né `wheelCircM` transitano da
  `WHEEL_STREAM`: lì passano **solo** i dati grezzi.

### Coach link `COACH_LINK` (0xaa0f) — relay coach→app (v0.3.0)
Con la **recisione del live app↔Pi** (§3) il telefono non è più connesso al `/coach`
del Pi: non riceverebbe quindi i comandi del coach. Il firmware — che è già connesso al
Pi su `/device` e **riceve già** i comandi `start`/`stop`/`lap` (§4) — li **inoltra
all'app** via NOTIFY su `0xaa0f`. Payload **2 byte esatti**: `uint8 type` + `uint8 arg`.

**Set di eventi (esaustivo — il firmware emette esattamente questi):**

| `type` | significato | `arg` |
|--------|-------------|-------|
| `0x01` | cmd **start** sessione | `0` |
| `0x02` | cmd **stop** sessione  | `0` |
| `0x03` | cmd **lap**            | `0` (lapNum non noto dal comando del coach) |

- L'app **deve ignorare** ogni `type` non elencato (i valori restano **riservati** per
  estensioni future → forward-compat).
- Lo **stato "REC"** dell'app si **deriva** da `start`/`stop` (non c'è un evento REC
  separato): `start` → REC on, `stop` → REC off.
- L'app usa questi eventi per segmentare la **propria** registrazione locale (che poi
  verrà caricata sul cloud, §7). La sessione registrata dal **coach** resta comunque
  generata dal **Pi** dai frame `/device` (fonte di verità lato coach).
- L'evento è **edge-only** (emesso quando arriva il comando), quindi è significativo
  mentre l'app è connessa e **sottoscritta** a `0xaa0f`: non c'è stato "ritardato" da
  leggere alla sottoscrizione.
- L'app **non** invia comandi al coach via BLE (NOTIFY-only): le azioni atleta passano
  dal firmware, che è il device comandato.

### Timebase oggettivo `TIME` (0xaa10) + `tUtc` (v0.3.0)
I frame di rete portano oggi `t` = **ms-da-boot** del firmware: monotòno e buono per
ordinare/rilevare deriva **dentro** una sessione, ma **soggettivo** (non confrontabile
fra device, inutile per il cloud e per la fusione aero+biomeccanica). Si introduce un
**timestamp oggettivo UTC**.

- Il device mantiene l'ora sull'**orologio di sistema** (`settimeofday`/`gettimeofday`).
  Il chip **RTC PCF85063** è presente sulla board ma **non ancora usato** (persistenza
  dell'ora attraverso il power-off → miglioria futura; oggi l'ora si perde al riavvio
  e va re-impostata alla connessione).
- **Sincronizzazione**: alla connessione il client con clock affidabile (in pratica
  l'**app**, dal clock del telefono) **scrive** in `TIME 0xaa10` l'epoch UTC corrente
  come `uint64 epochMs` **little-endian, esattamente 8 byte**. Regole della WRITE
  (firmware): lunghezza ≠ 8 → errore ATT `INVALID_ATTR_VALUE_LEN`; valore
  `< 2020-01-01` (`1577836800000`) → errore ATT `VALUE_NOT_ALLOWED`. La **READ**
  restituisce l'ora corrente del device come `uint64` LE, oppure **`0`** se mai impostata.
- **Nei frame** (§3) compare il campo **`tUtc`** = epoch ms UTC. `t` resta il monotòno di
  sorgente. Se l'orologio non è impostato, **`tUtc = 0`** (soglia: epoch `< 2020`) e il
  consumer usa il fallback (`serverTs` del Pi).
- **Priorità delle sorgenti di tempo**: (1) `tUtc` del device (impostato dall'app); (2)
  `serverTs` del Pi (può essere offline/senza NTP → meno affidabile); il cloud (§7)
  ordina/deduplica per `tUtc` quando `> 0`.
- **Fase 3**: `tUtc` è il **riferimento temporale comune** per allineare aerodinamica e
  biomeccanica fra sensori/device diversi.

#### Ordine byte del MAC in `0xaa0b`/`0xaa0e` (seam firmware↔new) — (v0.2.3)
In `SENSOR_WHITELIST` (0xaa0b) e nelle notifiche `SENSOR_SCAN` (0xaa0e) il campo
`uint8 mac[6]` è in **ordine di visualizzazione** (display order): `mac[0]` = primo
ottetto del MAC (la `AA` di `AA:BB:CC:DD:EE:FF`), `mac[5]` = ultimo ottetto (`FF`).
NON è l'ordine little-endian dello stack BLE (`ble_addr_t.val`). Firmware e app
DEVONO usare questo ordine; il firmware converte internamente verso `ble_addr_t`.

#### Confine firmware ↔ wheel — servizio `0xBB00` (lato sensore, v0.2.1)
Il firmware del sensore ruota Crr (**Seeed XIAO BLE Sense (nRF52840) + IMU
onboard LSM6DS3TR-C**) è un **componente
gestito** della repo madre: vive in [`wheel-fw/`](../wheel-fw/) (firmware
separato, **non** parte del build ESP-IDF). Espone il servizio `0xBB00` che
l'ESP32 (central) consuma:

| CHR | UUID | Flags | Bytes | Payload |
|-----|------|-------|-------|---------|
| STREAM | `0xBB01` | NOTIFY 10 Hz | 16 | `float speedMs, accelMs2, tempC, vibRMS` → relay su `0xaa0c` |
| RESULT | `0xBB02` | NOTIFY | 6 | `float crr + uint8 quality + uint8 runIdx` (legacy, non usato: il Crr lo calcola l'app) |
| CMD | `0xBB03` | WRITE | 1 | `uint8` coast-down (0x01/0x02/0x03/0xFF) ← inoltrato da `0xaa0d` |
| CONFIG | `0xBB04` | R+W | 8 | `float tireCircM + float massKg` ← scritto dall'ESP32 da `CONFIG 0xaa08` |

Governance: modifiche a `0xBB00` passano dalla seam `firmware↔wheel` (le due parti
sono nello stesso repo madre, ma restano firmware distinti) e dalla ratifica qui.

---

## 3. Confini B/C — frame di telemetria (firmware/app → pi, WebSocket)

Endpoint device→pi: `ws://192.168.8.1:8080/device` — **uplink primario**.
Endpoint app→pi (coach): `ws://<pi>:8080/coach` — **DEPRECATO** (v0.3.0).

> **Recisione del live app↔Pi (v0.3.0).** Il produttore primario di telemetria è
> l'**ESP32 su `/device`** (confine B). L'app (`aerodrag-new`) **non** si connette
> più al `/coach` del Pi per lo streaming live: resta **solo-BLE** verso il proprio
> ESP32 (così il telefono non deve unirsi all'AP del Pi e conserva la sua rete
> internet). Il confine **C** (`new → pi /coach`) è **deprecato**: il Pi può
> continuare ad accettarlo come **fallback legacy** (es. app in sim senza ESP32),
> ma non è più il percorso atteso e sarà rimosso in un MAJOR. I comandi del coach
> raggiungono l'app **via firmware** (`COACH_LINK 0xaa0f`, §2/§4), non più sul
> `/coach`.
>
> **Forward-compat — dato grezzo (Fase 3, biomeccanica/AI).** La **sorgente del
> dato ad alta frequenza è l'ESP32**. Il frame di rete a 2 Hz e il campionamento
> del Pi (1 pt ogni 5 frame) sono una **vista decimata per la dashboard**: NON
> sono da considerare l'unica forma archiviabile. L'archivio canonico (cloud, §7)
> dovrà poter preservare il dato grezzo/timestampato per l'analisi futura; il
> contratto **non** vincola l'archivio alla forma decimata.

**Handshake** (alla connessione):
```json
{ "type": "hello", "device": "AA:BB:CC:DD:EE:FF", "athlete": "Mario Rossi", "fw": "1.0.0" }
```
(`fw` inviato dal firmware; opzionale dall'app.)

**Frame dati** — **2 Hz**, inviato solo se la fisica è valida:
```json
{
  "t": 1716892800000,
  "tUtc": 1716892800000,
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
| `t` | number | ms | timestamp **soggettivo** monotòno (ms da boot per il firmware); per ordinare/deriva intra-sessione |
| `tUtc` | number | ms | **(v0.3.0)** timestamp **oggettivo** epoch UTC dall'orologio del device; `0` se non impostato (epoch `< 2020`) → fallback `serverTs`. Vedi §2 "Timebase oggettivo" |
| `device` | string | — | MAC BLE `AA:BB:CC:DD:EE:FF`; **obbligatorio**, chiave di sessione |
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

**Obbligo produttore — identità (v0.1.3):** ogni produttore (firmware **e app**)
**NON DEVE** inviare `hello`/frame senza un `device` MAC valido
(`^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$`). Se l'identità non è disponibile —
es. **app in sim mode o non ancora accoppiata** — il produttore **non apre lo
stream/non trasmette** al coach. È la regola speculare alla validazione che il
Pi applica all'ingestione (§5): garantisce il *principio di verità dei dati* —
nessuna sessione anonima o simulata viene registrata come reale.

**Identità alla sorgente (v0.1.2):** il campo `device` è **obbligatorio** e DEVE
essere un MAC valido (6 ottetti esadecimali separati da `:`). Il Pi **DEVE
rifiutare all'ingestione** ogni frame con `device` assente o non valido — come già
fa per il `CdA` fuori range — così una sessione priva di identità **non si forma
mai**. Conseguenza: nessuna sessione anonima e nessun fallback (`unknown` o forma
senza suffisso): vedi §5.

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

> **Instradamento comandi all'app (v0.3.0).** Con l'app non più su `/coach`, i
> comandi diretti all'**atleta** (`start`/`stop`/`lap`) raggiungono l'app **via
> firmware**: il Pi li invia al device su `/device` (come già fa), e il firmware li
> **inoltra in BLE** su `COACH_LINK 0xaa0f` (§2). Gli **eventi** verso il **coach**
> (dashboard) restano su `/coach` invariati. `ota` resta diretto al device. Se è
> attivo il fallback legacy `/coach` per l'app (sim), su quella connessione valgono
> ancora le regole pre-v0.3.0.

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

**Timestamp oggettivo (v0.3.0):** il `ts` della sessione DOVREBBE derivare dal `tUtc`
del device (epoch UTC, §2/§3) quando disponibile, così le sessioni sono ordinabili e
deduplicabili in modo assoluto fra device diversi (chiave cloud, §7). Fallback al
`serverTs` del Pi se `tUtc = 0`.

**Vincolo nome file (v0.1.1, raffinato in v0.1.2):** poiché il Pi rifiuta a monte
i frame senza `device` valido (§3), ogni sessione persistita ha un `deviceId`
valido. Il nome file è **sempre** `session_{ts}_{deviceIdHex}.json` con
`deviceIdHex` = MAC senza `:` (esadecimale non vuoto). **Non** sono ammessi né il
token `unknown` né la forma anonima senza suffisso `session_{ts}.json`. Il
`/receive` del coach valida con `^session_\d+_[A-Fa-f0-9]+\.json$` (suffisso
**obbligatorio**) e rifiuta il resto.

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

## 7. Confine H — client ↔ cloud (RISERVATO, non implementato) — Fase 2

> **Stato: RISERVATO.** Confine **documentato ma non implementato**. Nessun client lo
> usa ancora; è qui per fissare la direzione ed evitare scelte che lo precludano. La
> spec di dettaglio (auth, schema, endpoint) sarà ratificata quando si costruisce il
> backend (Fase 2). Niente codice finché non è promosso da `riservato` a `ratified`.

**Scopo.** Il **cloud è l'archivio canonico** delle sessioni (verità di lungo
periodo). Con il cloud, il backfill "scarica i dati mancanti da ovunque" è
**client↔cloud**, non più un legame diretto app↔Pi sulla LAN: ogni client (app,
coach) sincronizza col cloud quando ha internet. Questo **sostituisce** il fallback
LAN app↔Pi che altrimenti sarebbe servito.

**Principi (vincolanti per il disegno futuro):**
- **Trasporto**: REST/HTTPS, **autenticato** (token derivato dall'identità/pairing).
  Non WebSocket: è riconciliazione asincrona, non streaming.
- **Sessioni immutabili**, **upsert idempotente per `sessionId`** (`session_{ts}_{deviceHex}`):
  chi non ha una sessione la prende; chi ce l'ha non la sovrascrive → niente conflitti,
  niente merge a metà sessione.
- **Pull a delta** (`since` timestamp); upload bidirezionale (app **e** coach possono
  caricare; dedup per id).
- **Identità atleta-centrica**: la chiave longitudinale è l'**atleta** (persona nel
  tempo, su più device/sport); il MAC del device è **una** delle sorgenti mappate
  all'atleta, non la chiave primaria. (Richiesto dall'analisi AI, Fase 3.)
- **Schema a canali versionato ed estensibile** (non colonne fisse): l'aggiunta di
  modalità nuove (es. **biomeccanica**: IMU multipli, forza ai pedali, angoli
  articolari) = nuovi canali, **senza** rompere il contratto. Aero e biomeccanica
  condividono un **timebase allineato**.
- **Privacy/consenso**: dati biomeccanici/health-adjacent → ownership e consenso
  previsti **dall'inizio** (GDPR; possibili dati sensibili).
- **Naming**: per non collidere col "sync" Pi→PC esistente (§4), il cloud usa termini
  distinti (es. *cloud push/pull*, *archive*).

**Fase 3 (oltre H).** Il cloud è la base per l'**AI coach** (aerodinamica **e**
biomeccanica), che è un **consumatore** dell'archivio canonico: richiede il dato
grezzo ad alta frequenza (vedi forward-compat in §3) e l'identità atleta-centrica.

---

## 8. Versioni componenti (al contratto v0.1.0)

| Componente | Versione |
|-----------|----------|
| firmware (`FW_VERSION_STR`) | 1.0.0 |
| pi server | 1.0.0 (v4 multi-atleta) |
| pi pc-receiver | 1.0.0 |
| coach (Electron) | 1.0.0 |
| coach pc-receiver | 1.0.0 |
| new (app) | 1.0.0 (Expo ~56) |
| wheel-fw (sensore ruota Crr) | 0.2.1 (XIAO BLE Sense + LSM6DS3TR-C, da buildare) |

---

## 9. Changelog

### v0.3.0 — 2026-06-21
**Recisione del live app↔Pi + uplink ESP32 primario** (cambiamento di ruolo/architettura →
MINOR con deprecazione). Decisione dell'owner del progetto. **Firmware (madre) implementato.**
- **App solo-BLE**: `aerodrag-new` non si connette più al `/coach` del Pi per il live;
  resta in BLE verso il proprio ESP32 e **conserva la rete internet** del telefono.
- **ESP32 = uplink primario** verso il Pi (`/device`, confine B). Già implementato lato
  firmware/Pi: cambia il *ruolo*, non il payload dei frame (§3 invariato).
- **Confine C (`new → pi /coach`) DEPRECATO** → fallback legacy (app in sim senza ESP32);
  rimozione in un MAJOR futuro.
- **Nuova `COACH_LINK 0xaa0f`** (§2): il firmware inoltra all'app **solo** i comandi
  coach `start`/`stop`/`lap` (NOTIFY 2 B `type`+`arg`) ricevuti sul `/device`; l'app
  deriva la REC da start/stop e ignora i `type` non elencati. I comandi all'atleta non
  passano più dal `/coach` dell'app (§4).
- **Timestamp oggettivo (`TIME 0xaa10` + `tUtc`)** (§2/§3): il device tiene l'ora
  sull'**orologio di sistema** (impostato dall'app alla connessione, `uint64 epochMs` LE
  8 B; l'RTC PCF85063 non è ancora usato) e marca i frame con `tUtc` (epoch UTC, `0` se
  non impostato). `t` resta il monotòno soggettivo. Abilita ordinamento/dedup assoluti
  (cloud, §7) e l'allineamento temporale aero↔biomeccanica (Fase 3). Il `ts` di sessione
  (§5) deriva da `tUtc` quando `> 0`.
- **Confine H riservato** (§7): cloud come **archivio canonico** (Fase 2), che sostituisce
  ogni fallback LAN app↔Pi. Note di **forward-compat** (§3): la sorgente del dato grezzo
  ad alta frequenza è l'ESP32; l'archivio non è vincolato alla vista decimata (Fase 3:
  biomeccanica + AI coach).
- Nessuna modifica al payload dei frame (§3), allo schema sessione (§5) o al modello
  fisico (§6).

### v0.2.3 — 2026-06-20
Chiarimenti (PATCH, nessuna modifica al wire) dall'audit bug cross-repo.
- **Seam firmware↔new**: specificato l'ordine byte di `mac[6]` in `0xaa0b`/`0xaa0e`
  (display order, `mac[0]`=primo ottetto). Risolve l'ambiguità "big-endian".
- **Sensore ruota** (confine G): hardware → Seeed XIAO BLE Sense + LSM6DS3TR-C
  (0xBB00 invariato).
- Allineamenti firmware al contratto già esistente: `OTA_URL` ora ≤256 (era 200),
  MTU preferito 185 (era 100). Nessun impatto sui consumatori.

### v0.2.2 — 2026-06-19
Fix pairing sensori su iOS (seam `firmware↔new`). Su iOS l'app non conosce il
MAC dei sensori (l'id BLE è un UUID), quindi non può popolare `0xaa0b`.
- Nuova **`SENSOR_SCAN` (0xaa0e)**: il **firmware** fa la discovery (vede i MAC
  reali) e notifica all'app i candidati `{type, mac, rssi, name}`; l'app sceglie
  e scrive i MAC in `SENSOR_WHITELIST 0xaa0b`. Funziona iOS+Android.

### v0.2.1 — 2026-06-17
Il firmware del **sensore ruota Crr** entra nel contratto come componente gestito:
- Nuovo confine **G** (`firmware ↔ wheel`, BLE `0xBB00`) con la spec lato sensore
  (STREAM/RESULT/CMD/CONFIG).
- Il firmware vive in **`wheel-fw/`** nella repo madre (separato dal build
  ESP-IDF; toolchain nRF Connect SDK/Zephyr). Skeleton iniziale `0.1.0`.

### v0.2.0 — 2026-06-17
Nuovo modello pairing & sensori (cambiamento comportamentale → MINOR). Decisioni
dell'owner del progetto, propagate sulla seam firmware↔new.
- **Pairing device — QR primario** (supera v0.1.4 "opzionale"): il QR fornisce il
  MAC autorevole; la connessione conferma l'identità leggendo `0xaa05` (iOS-safe).
- **Sensori bondati SOLO al firmware**: nuova `SENSOR_WHITELIST` (0xaa0b); il
  central del firmware connette solo ai MAC scelti dall'app (basta cambiare il
  filtro in `ble_sensors.c`, oggi "primo trovato" → anti cross-talk). L'app non
  apre più connessioni dati ai sensori (era uno specchio errato).
- **Crr — calibrazione app, dati via firmware**: nuove `WHEEL_STREAM` (0xaa0c,
  relay grezzo) e `WHEEL_CMD` (0xaa0d, comandi coast-down). L'app calcola `crr` e
  lo scrive in `CONFIG`.
- **`CONFIG 0xaa08` — app autorevole per tutti e tre i parametri** (`massKg`,
  `crr`, `wheelCircM`); il firmware li integra solo nella fisica. (Prima
  `wheelCircM` era "device-owned".)
- Rende **vestigiale** la whitelist sensori lato-app: sostituita da `0xaa0b`.

### v0.1.4 — 2026-06-17
Audit pairing. Chiarimento §2, nessuna rottura di wire:
- **IDENTITY 0xaa05** è l'identità canonica: l'app DEVE leggere il `device_id`
  (MAC) alla connessione e usarlo come `device` dei frame (§3). Risolve l'uso
  errato dell'id BLE come identità.
- **Disaccoppiamento identità/trasporto BLE**: l'id di connessione BLE è
  platform-specific (UUID su iOS) → usato solo come filtro di connessione, mai
  come identità né confrontato col MAC. Corregge il bug iOS (device QR-MAC mai
  uguale all'UUID iOS → nessuna connessione) e il caso `device:'unknown'`.
- QR = whitelist opzionale, non fonte dell'identità runtime.

### v0.1.3 — 2026-06-17
Audit dataflow cross-repo. Chiarimento + fix, nessuna rottura di wire:
- **§3 obbligo produttore**: firmware/app non inviano `hello`/frame senza `device`
  MAC valido (sim/non-accoppiato → niente stream). Regola speculare alla
  validazione del Pi (§5). Risolve la perdita silenziosa di frame app con
  `device:'unknown'` introdotta dal filtro MAC del Pi (v0.1.2).
- **Firmware (`coach_send_frame`)**: i frame con `lapEvent=true` **bypassano** il
  throttle 2 Hz, così il marker di giro non viene perso quando l'edge cade in un
  ciclo throttlato.

### v0.1.2 — 2026-06-17
Risoluzione capofila della seam `pi↔coach` (PR pi#7 divergente da v0.1.1).
Adottata la soluzione **più corretta**: validazione dell'identità **alla sorgente**.
- **§3**: `device` obbligatorio e MAC-valido; il Pi **rifiuta all'ingestione** i
  frame senza `device` valido (la sessione non si forma) — niente perdita di dati
  legittimi, che hanno sempre un MAC.
- **§5**: confermato filename `session_{ts}_{deviceIdHex}.json`; vietati sia
  `unknown` sia la forma anonima `session_{ts}.json`. Regex coach a suffisso
  obbligatorio.
- Supera la variante "no-data-loss" della PR pi#7 (forma anonima), che non è più conforme.

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
