# Schermate riviste — FIRMWARE DISPLAY

**Repo di destinazione:** `aerodrag-firmware`
**Hardware:** Waveshare **ESP32-S3-Touch-LCD-2.8** · **240×320 px portrait** · controller **ST7789T3** · RGB565.
**File principale:** `components/display/display.h` (renderer software: font bitmap 5×7, archi, rettangoli).

> Tutte le modifiche sono **solo** nel repo `aerodrag-firmware`. Nessuna toccata a dashboard/app/pi.
> ⚠️ Il README del repo cita un LCD-1.46 **rotondo** → è un **refuso di codice vecchio**. Il display reale è **rettangolare 2,8"**.
> 🛑 **NOTA IMPLEMENTAZIONE (dalla revisione):** la colonna RGB565 della tabella §1 contiene **3 valori errati**. **Ricalcola SEMPRE lRGB565 dallhex** con `((R>>3)<<11)|((G>>2)<<5)|(B>>3)` — lhex è la fonte di verità. Correzioni note: `COL_BG #07090f → 0x0041` (non 0x0841) · `COL_TEXTDIM #8398bd → 0x84D7` (non 0x8417) · `COL_TEXT #dbe6f6 → 0xDF3E` (non 0xDF1E).

> ⚠️ **Contratto dati INVARIATO (v0.2.3)** — questo redesign è puramente visivo: nessun nuovo campo sul wire.

---

## 1. Design tokens — palette unificata (fonte di verità unica)

Identica su firmware / app / dashboard. Sul firmware = stessi colori in **RGB565**.

| Token | Hex | RGB565 | Ruolo |
|---|---|---|---|
| accent | `#00d9a3` | `0x06D4` | CdA, brand, OK, attivo |
| power | `#f5a623` | `0xF524` | potenza, quota aero, warning |
| speed | `#4d9fff` | `0x4CFF` | velocità, rolling, info |
| alert | `#ff4d6a` | `0xFA6D` | **SOLO** FC, peak, allarmi |
| positive | `#22c55e` | `0x262B` | new best, delta migliore |
| text | `#dbe6f6` | `0xDF1E` | valori, titoli |
| textDim | `#8398bd` | `0x8417` | testo secondario |
| muted | `#46587c` | `0x42CF` | label, unità, assi |
| surface | `#0f1420` | `0x08A4` | card |
| track | `#1e2840` | `0x1948` | sfondo barre/anelli |
| panel | `#131a28` | `0x10C5` | divisori, sfondo barre |
| bg | `#07090f` | `0x0841` | sfondo schermo |

```c
// ── AeroDrag palette unificata (RGB565) — display.h ──────────
#define COL_ACCENT    0x06D4  // #00d9a3  CdA, brand, OK
#define COL_POWER     0xF524  // #f5a623  potenza, AERO, warning
#define COL_SPEED     0x4CFF  // #4d9fff  velocità, rolling
#define COL_ALERT     0xFA6D  // #ff4d6a  SOLO FC, peak, allarmi
#define COL_POSITIVE  0x262B  // #22c55e  new best, delta
#define COL_TEXT      0xDF1E  // #dbe6f6  valori, titoli
#define COL_TEXTDIM   0x8417  // #8398bd  testo secondario
#define COL_MUTED     0x42CF  // #46587c  label, unità, assi
#define COL_SURFACE   0x08A4  // #0f1420  card
#define COL_TRACK     0x1948  // #1e2840  track anelli/barre
#define COL_PANEL     0x10C5  // #131a28  divisori
#define COL_BG        0x0841  // #07090f  sfondo schermo
```

---

## 2. MIGRAZIONE COLORI — tabella esplicita (CRITICO, anti-regressione)

I vecchi nomi sono usati **38 volte**: `COL_TEAL`×20, `COL_AMBER`×7, `COL_RED`×11. Applicare così:

| Vecchio | Usi | → Nuovo | Regola |
|---|---|---|---|
| `COL_TEAL` | 20 | `COL_ACCENT` | **tutti** i 20 usi, sostituzione diretta |
| `COL_AMBER` | 7 | `COL_POWER` | **tutti** i 7 usi, sostituzione diretta |
| `COL_RED` | 11 | `COL_ALERT` **o** `COL_POWER` | **caso per caso** (vedi sotto) |
| `COL_MUTED` | — | `COL_MUTED` | **mantieni il nome**, aggiorna solo il valore → `0x42CF` |
| `COL_TEXT` | — | `COL_TEXT` | **mantieni il nome**, aggiorna valore → `0xDF1E` |
| `COL_SURFACE` | — | `COL_SURFACE` | **mantieni il nome**, aggiorna valore → `0x08A4` |
| `COL_BG` | — | `COL_BG` | **mantieni il nome**, aggiorna valore → `0x0841` |
| *(nuovi)* | — | `COL_SPEED`, `COL_POSITIVE`, `COL_PANEL`, `COL_TRACK`, `COL_TEXTDIM` | aggiungere |

### Disambiguazione dei 11 `COL_RED`
- FC, valore di **peak** (CdA/potenza), stato **pairing-wait** lampeggiante → **`COL_ALERT`**.
- Barra **"Aero"** / quota aerodinamica nello `SCR_POWER` → **`COL_POWER`** (era un falso allarme).
- In dubbio: se rappresenta un *valore normale* (non un'anomalia) → `COL_POWER`; se è *allarme/limite* → `COL_ALERT`.

---

## 3. Driver display — NON toccare

Il display ora usa il driver **ufficiale `esp_lcd_panel_st7789`** (riscritto in bring-up).
**Il redesign tocca SOLO:** i `#define COL_*` e i **layout dentro `display_render()`** per ogni `SCR_*`.
**NON toccare:** `display_init` / `display_flush` / lo stack `esp_lcd` / la sequenza di init del pannello.

---

## 4. Schermate reali = 6 (enum completo)

`SCR_PAIRING` · `SCR_CDA` · `SCR_TIMER` · `SCR_SPEED` · `SCR_POWER` · `SCR_STATUS`.
Tipografia: font bitmap **5×7** scalato (1–6). Numeri = dato dominante (scale grandi); label piccole `COL_MUTED`.

### SCR_PAIRING (QR)
Wordmark `◉ AeroDrag` (accent) + "Inquadra per associare" (muted). QR su fondo bianco, codice `AD-7F3K` (textDim), "In attesa…" con dot `COL_ALERT` lampeggiante.

### SCR_CDA (performance)
Header: `CdA` (muted) · `88% AERO` (accent) · batteria. **Arco gauge** (track `COL_TRACK`, progress `COL_ACCENT`, pallino bianco). Centro: grade `A` (accent) + `0.214 m²` (text). `▼ NEW BEST` lampeggiante `COL_POSITIVE`. Riga inferiore: 38 km/h (text) · 40 wind (accent) · 238 W (power). Status dots W/B/A/P.

### SCR_TIMER (lap)
`LAP` (muted) + numero lap grande (accent). Tempo lap `02:14` mono (accent), `TOT 12:30` (text) sotto divisore `COL_PANEL`. Chip distanza (text) + pot. media (power).

### SCR_SPEED (velocità) — **AGGIUNTA (mancava)**
- Numero **km/h grande** mono centrale in **`COL_SPEED`** (coerente con il ruolo "velocità").
- Label `km/h` (muted) sotto al valore.
- Riga secondaria: **media** (`AVG 41.2`, textDim) ed eventuale **max** (`MAX 52.0`, muted).
- Opzionale: piccola v aria / vento (`wind 40`, accent) se già nel frame.
- Status dots W/B/A/P in basso a dx, coerenti con le altre SCR.

### SCR_POWER (split) — con fix barra Aero
Header: `238 W` (power) + zona `Z4` + W/kg. 3 barre verticali: **Aero `COL_POWER`** / Roll `COL_ACCENT` / Other `COL_MUTED`, con percentuali.

### SCR_STATUS (vitals 2×2)
Quadranti divisi da `COL_PANEL`: CdA (accent) · HR (alert) · Cadence (power) · Power (power). Dot di stato sui quadranti HR/Power.

---

## 5. Note implementative
- Solo `#define` colore + ritocchi layout in `display_render()`. **Nessun nuovo carico** sul renderer (stesse primitive).
- Coordinate sul framebuffer **240×320 reale** (i mockup sono a 2× = 480×640).
- Framebuffer RGB565 = 240×320×2 = 153.6 KB in PSRAM (invariato).

---

## 6. NOTE ANTI-REGRESSIONE
- **NON** modificare `display_init` / `display_flush` / `esp_lcd_panel_st7789` / init pannello.
- **NON** introdurre nuovi campi dati: contratto **v0.2.3 invariato**.
- Mantieni i nomi `COL_MUTED/COL_TEXT/COL_SURFACE/COL_BG` (cambia solo il valore) per non rompere i 30+ riferimenti esistenti.
- Disambigua i `COL_RED` uno per uno: un solo errore su "Aero" reintroduce il falso allarme.

## 7. CHECKLIST DI ACCETTAZIONE
- [ ] `idf.py build` compila senza errori.
- [ ] `grep COL_TEAL` e `grep COL_AMBER` → **0 residui**.
- [ ] `COL_RED` non esiste più; `COL_ALERT` usato **solo** su FC / peak / pairing-wait.
- [ ] Barra "Aero" in `COL_POWER`.
- [ ] Tutte e **6** le `SCR_*` renderizzano correttamente (incl. la nuova `SCR_SPEED`).
- [ ] Coordinate verificate a 240×320 (non 480×640).
- [ ] Valori RGB565 corretti a video (no overflow/endianness sul bus SPI).
- [ ] Riferimento LCD-1.46 rotondo rimosso dai commenti/README.
