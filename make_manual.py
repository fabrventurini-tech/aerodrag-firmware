#!/usr/bin/env python3
"""Generate AeroDrag firmware user manual as PDF."""

from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
                                 TableStyle, HRFlowable, PageBreak)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY

OUTPUT = "AeroDrag_UserManual.pdf"

W, H = A4

# ── Palette ───────────────────────────────────────────────────────────────────
DARK   = colors.HexColor("#09080C")   # near-black
TEAL   = colors.HexColor("#00D4AA")
AMBER  = colors.HexColor("#F5A623")
RED    = colors.HexColor("#F24560")
MUTED  = colors.HexColor("#4D6080")
LIGHT  = colors.HexColor("#DDE8F5")
WHITE  = colors.white
BG     = colors.HexColor("#111925")

def build_styles():
    s = getSampleStyleSheet()

    title = ParagraphStyle("ManTitle",
        fontName="Helvetica-Bold", fontSize=28, textColor=TEAL,
        spaceAfter=4*mm, alignment=TA_CENTER)

    subtitle = ParagraphStyle("ManSub",
        fontName="Helvetica", fontSize=13, textColor=MUTED,
        spaceAfter=8*mm, alignment=TA_CENTER)

    h1 = ParagraphStyle("H1",
        fontName="Helvetica-Bold", fontSize=16, textColor=TEAL,
        spaceBefore=8*mm, spaceAfter=3*mm)

    h2 = ParagraphStyle("H2",
        fontName="Helvetica-Bold", fontSize=12, textColor=AMBER,
        spaceBefore=5*mm, spaceAfter=2*mm)

    body = ParagraphStyle("Body",
        fontName="Helvetica", fontSize=10, textColor=LIGHT,
        spaceAfter=2*mm, leading=15, alignment=TA_JUSTIFY)

    mono = ParagraphStyle("Mono",
        fontName="Courier", fontSize=9, textColor=TEAL,
        spaceAfter=1*mm, leading=14)

    note = ParagraphStyle("Note",
        fontName="Helvetica-Oblique", fontSize=9, textColor=MUTED,
        spaceAfter=2*mm, leading=13)

    return dict(title=title, subtitle=subtitle, h1=h1, h2=h2,
                body=body, mono=mono, note=note)


def hr():
    return HRFlowable(width="100%", thickness=0.5,
                      color=MUTED, spaceAfter=4*mm, spaceBefore=2*mm)


def table(data, col_widths, header=True):
    t = Table(data, colWidths=col_widths)
    style = [
        ("BACKGROUND", (0,0), (-1,0 if header else -1), BG),
        ("TEXTCOLOR",  (0,0), (-1,0), TEAL if header else LIGHT),
        ("FONTNAME",   (0,0), (-1,0), "Helvetica-Bold"),
        ("FONTSIZE",   (0,0), (-1,-1), 9),
        ("FONTNAME",   (0,1), (-1,-1), "Helvetica"),
        ("TEXTCOLOR",  (0,1), (-1,-1), LIGHT),
        ("ROWBACKGROUNDS", (0,1), (-1,-1),
             [colors.HexColor("#0D1520"), colors.HexColor("#111925")]),
        ("GRID",       (0,0), (-1,-1), 0.3, MUTED),
        ("LEFTPADDING",(0,0), (-1,-1), 6),
        ("RIGHTPADDING",(0,0),(-1,-1), 6),
        ("TOPPADDING", (0,0), (-1,-1), 4),
        ("BOTTOMPADDING",(0,0),(-1,-1), 4),
        ("VALIGN",     (0,0), (-1,-1), "MIDDLE"),
        ("WORDWRAP",   (0,0), (-1,-1), True),
    ]
    t.setStyle(TableStyle(style))
    return t


def on_page(canvas, doc):
    """Dark background + header/footer on every page."""
    canvas.saveState()
    canvas.setFillColor(colors.HexColor("#090D14"))
    canvas.rect(0, 0, W, H, fill=1, stroke=0)

    # Top bar
    canvas.setFillColor(BG)
    canvas.rect(0, H - 18*mm, W, 18*mm, fill=1, stroke=0)
    canvas.setFont("Helvetica-Bold", 9)
    canvas.setFillColor(TEAL)
    canvas.drawString(15*mm, H - 11*mm, "AeroDrag Firmware")
    canvas.setFillColor(MUTED)
    canvas.drawRightString(W - 15*mm, H - 11*mm, "Manuale Utente  -  v1.0")

    # Bottom bar
    canvas.setFillColor(BG)
    canvas.rect(0, 0, W, 12*mm, fill=1, stroke=0)
    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(MUTED)
    canvas.drawCentredString(W/2, 5*mm, f"Pagina {doc.page}")

    canvas.restoreState()


def build():
    doc = SimpleDocTemplate(
        OUTPUT,
        pagesize=A4,
        leftMargin=18*mm, rightMargin=18*mm,
        topMargin=24*mm, bottomMargin=18*mm,
        title="AeroDrag Firmware — Manuale Utente",
        author="AeroDrag",
    )

    st = build_styles()
    P  = lambda txt, style="body": Paragraph(txt, st[style])
    SP = lambda h=4: Spacer(1, h*mm)

    story = []

    # ── Cover ─────────────────────────────────────────────────────────────────
    story += [SP(20),
              P("AeroDrag", "title"),
              P("Manuale Utente del Firmware", "subtitle"),
              P("Waveshare ESP32-S3-Touch-LCD-2.8  -  IDF v5.3.2", "subtitle"),
              SP(6), hr(), SP(4)]

    story += [P("""
AeroDrag è un dispositivo per la misurazione del coefficiente aerodinamico (CdA)
in tempo reale durante sessioni di ciclismo. Il firmware gestisce il display,
la sensoristica (pitot SDP810, IMU QMI8658C, BLE ANT+), la connettività
WiFi/BLE e la trasmissione dati al coach.
""", "body"), SP(2)]

    # ── 1. Primo avvio ────────────────────────────────────────────────────────
    story += [P("1. Primo Avvio e Pairing", "h1"), hr()]

    story += [P("""
All'accensione il dispositivo mostra il <b>QR Code di pairing</b>.
Inquadralo con l'app AeroDrag per associare il dispositivo. Il punto rosso
in basso indica che nessun telefono è collegato. Una volta connesso via BLE,
il display avanza automaticamente alla schermata CdA.
""", "body")]

    story += [P("URI del QR Code:", "h2"),
              P("AERODRAG://PAIR/&lt;MAC_ADDRESS&gt;", "mono"),
              P("Esempio: AERODRAG://PAIR/AA:BB:CC:DD:EE:FF", "mono"), SP(2)]

    # ── 2. Schermate ──────────────────────────────────────────────────────────
    story += [P("2. Schermate — Ciclo con Pulsante BOOT", "h1"), hr()]

    story += [P("""
Premi brevemente il <b>pulsante BOOT</b> (GPIO0) per avanzare alla schermata successiva.
Il ciclo è: <b>CdA → Timer → Velocità → Potenza → Vitals → CdA</b>.
""", "body"), SP(2)]

    screens = [
        ["Schermata", "Cosa mostra", "Note"],
        ["SCR_CDA\n(CdA / Performance)",
         "Arco CdA con grade A+/A/B/C/D\nValore 0.xxx m²\nBattery icon top-right\nVelocità · Vento · Potenza (barra inferiore)\nTick bianco del best di sessione",
         "Schermata principale\ndi misura"],
        ["SCR_TIMER\n(Lap Timer)",
         "Numero giro (grande)\nTempo giro MM:SS\nTempo sessione totale MM:SS",
         "Avvia con doppio click\nBOOT"],
        ["SCR_SPEED\n(Velocità)",
         "Velocità GPS/ANT+ in km/h (grande)\nAirspeed dal pitot\nDelta vento (±km/h)\nFC e Potenza in overlay",
         "Richiede pitot collegato\nper airspeed"],
        ["SCR_POWER\n(Potenza)",
         "Potenza in Watt\nNome zona (Z1-Z6)\nW/kg\nBarre Aero / Roll / Other (%)",
         "Richiede sensore\nANT+ potenza"],
        ["SCR_STATUS\n(Vitals 2×2)",
         "TL: CdA con grade\nTR: FC (bpm)\nBL: Cadenza (rpm)\nBR: Potenza (W) + zona",
         "Vista compatta\ndi tutti i parametri"],
    ]
    story += [table(screens,
                    [38*mm, 90*mm, 42*mm]), SP(4)]

    # ── 3. Pulsante BOOT ──────────────────────────────────────────────────────
    story += [P("3. Funzioni del Pulsante BOOT (GPIO0)", "h1"), hr()]

    btn_data = [
        ["Pressione", "Durata", "Azione"],
        ["Click singolo",   "50 ms – 3 s",  "Avanza alla schermata successiva"],
        ["Doppio click",    "< 400 ms tra i due click",
         "Avvia sessione: azzera timer, imposta Lap 1, toast LET'S GO!"],
        ["Long press",      "≥ 3 s",
         "Avvia calibrazione pitot (5 secondi, toast CALIBRATING)"],
        ["Very long press", "≥ 5 s",
         "Cicla modalità WiFi Coach:\nWIFI OFF → COACH DIRECT → CO-OP WIFI"],
    ]
    story += [table(btn_data, [35*mm, 45*mm, 90*mm]), SP(4)]

    # ── 4. Indicatori di stato ────────────────────────────────────────────────
    story += [P("4. Barra di Stato (4 dot, angolo in basso a destra)", "h1"), hr()]

    dots = [
        ["Dot", "Lettera", "Significato", "Verde = OK"],
        ["1", "W", "WiFi / Coach",    "Coach connesso e pronto"],
        ["2", "B", "BLE",             "Telefono connesso via BLE"],
        ["3", "A", "ANT+",            "Sensore ANT+ attivo (potenza/cadenza/FC)"],
        ["4", "P", "Pitot",           "SDP810 risponde e legge pressione"],
    ]
    story += [table(dots, [12*mm, 20*mm, 55*mm, 83*mm]), SP(2)]
    story += [P("I dot sono visibili su tutte le schermate tranne il pairing e il vitals 2×2.", "note")]

    # ── 5. Calibrazione pitot ─────────────────────────────────────────────────
    story += [P("5. Calibrazione del Sensore Pitot", "h1"), hr()]

    story += [P("""
<b>Quando calibrare:</b> a ogni sessione, prima di partire, con il dispositivo
fermo e la sonda pitot libera da vento.
""", "body"),
              P("""
<b>Procedura:</b><br/>
1. Tieni il dispositivo fermo all'aria ferma.<br/>
2. Tieni premuto BOOT per ≥ 3 secondi. Appare il toast <b>CALIBRATING</b>.<br/>
3. Aspetta 5 secondi senza muovere il dispositivo.<br/>
4. Il toast scompare: la calibrazione è completata.
Il valore di zero offset è salvato in flash (NVS).
""", "body"), SP(2)]

    # ── 6. Sessione di misura ─────────────────────────────────────────────────
    story += [P("6. Sessione di Misura", "h1"), hr()]

    story += [P("""
<b>Avvio manuale:</b> doppio click su BOOT → il display passa a SCR_TIMER,
il lap 1 inizia, appare il toast <b>LET'S GO!</b>.
""", "body"),
              P("""
<b>Avvio da coach:</b> il server WiFi manda il comando START.
Il dispositivo inizia la sessione e comincia a inviare frame di dati.
""", "body"),
              P("""
<b>Cambio giro:</b> automatico da evento ANT+ (computer di bordo),
oppure comando LAP dal coach WiFi.
""", "body"),
              P("""
<b>Best di sessione (SCR_CDA):</b> il tick bianco sull'arco segna il CdA
minimo raggiunto durante la sessione. Per 2 secondi lampeggia <b>NEW BEST</b>.
""", "body"), SP(2)]

    # ── 7. WiFi Coach ─────────────────────────────────────────────────────────
    story += [P("7. Modalità WiFi Coach", "h1"), hr()]

    modes = [
        ["Modalità", "Descrizione"],
        ["WIFI OFF",
         "WiFi disabilitato. Solo BLE. Nessuna connessione al coach."],
        ["COACH DIRECT",
         "Il dispositivo si connette al WiFi configurato in NVS\n"
         "e contatta direttamente il server coach."],
        ["CO-OP WIFI",
         "Modalità cooperativa: il coach manda comandi START/STOP/LAP\n"
         "e riceve i frame di dati in tempo reale."],
    ]
    story += [table(modes, [42*mm, 128*mm]), SP(2)]

    story += [P("""
Per cambiare modalità: Very long press su BOOT (≥ 5 s).
La modalità è salvata in NVS e persiste tra i riavvii.
""", "body")]

    story += [PageBreak()]

    # ── 8. Connettività BLE ───────────────────────────────────────────────────
    story += [P("8. Connettività BLE", "h1"), hr()]

    story += [P("""
Il dispositivo espone un server GATT BLE con le seguenti notifiche:
""", "body")]

    ble_data = [
        ["Caratteristica", "Dati trasmessi", "Frequenza"],
        ["Pitot",    "Pressione differenziale (Pa) + pressione statica",   "10 Hz"],
        ["IMU",      "Pitch e roll (gradi)",                                "10 Hz"],
        ["ANT+",     "Potenza (W), cadenza (rpm), FC (bpm), velocità",      "su evento"],
        ["Ambiente", "Temperatura (°C), umidità (%), altitudine (m)",       "1 Hz"],
        ["Batteria", "Percentuale carica batteria",                          "ogni 10 s"],
        ["LAP",      "Notifica cambio giro",                                 "su evento"],
    ]
    story += [table(ble_data, [40*mm, 90*mm, 40*mm]), SP(2)]

    story += [P("""
Il BLE Central integrato si collega automaticamente a sensori ANT+ Bridge
(potenza, CSC, FC) nelle vicinanze. Il pairing avviene tramite QR Code all'avvio.
""", "body")]

    # ── 9. OTA ────────────────────────────────────────────────────────────────
    story += [P("9. Aggiornamento Firmware OTA", "h1"), hr()]

    story += [P("""
Il firmware supporta aggiornamenti Over-The-Air (OTA) tramite HTTPS.
L'aggiornamento viene avviato dal coach WiFi. Se il nuovo firmware va in
crash prima di completare il boot, il bootloader ripristina automaticamente
la versione precedente (rollback).
""", "body"),
              P("Durante l'OTA il dispositivo non va in deep sleep (l'attività è mantenuta attiva).", "note")]

    # ── 10. Batteria ──────────────────────────────────────────────────────────
    story += [P("10. Gestione Batteria", "h1"), hr()]

    bat_data = [
        ["Parametro",   "Valore"],
        ["Piena",       "4200 mV"],
        ["Scarica",     "3300 mV"],
        ["Divisore ADC","GPIO8, rapporto 3×"],
        ["Soglia sleep","< 5% → deep sleep automatico"],
        ["Timeout inattività", "10 minuti senza dati → deep sleep"],
    ]
    story += [table(bat_data, [60*mm, 110*mm]), SP(2)]

    story += [P("""
La <b>battery icon</b> in alto a destra nella schermata CdA mostra 4 barre
(ognuna = 25%) e la percentuale numerica. Diventa rossa sotto il 25%.
""", "body")]

    # ── 11. Pin e hardware ────────────────────────────────────────────────────
    story += [P("11. Pin Hardware", "h1"), hr()]

    pins = [
        ["Funzione",        "GPIO", "Protocollo / Note"],
        ["Display SCLK",    "40",   "SPI CLK (ST7789T3)"],
        ["Display MOSI",    "45",   "SPI MOSI"],
        ["Display DC",      "41",   "Data/Command"],
        ["Display CS",      "42",   "Chip Select"],
        ["Display RST",     "39",   "Hardware reset"],
        ["Display BL",      "5",    "Backlight PWM"],
        ["Touch SDA",       "1",    "Bus I2C dedicato (CST328, 0x1A)"],
        ["Touch SCL",       "3",    "Bus touch GPIO1/3 (non usato dal fw)"],
        ["Touch INT",       "4",    "Interrupt"],
        ["Touch RST",       "2",    "Reset"],
        ["IMU SDA",         "11",   "I2C0 (QMI8658C, 0x6B)"],
        ["IMU SCL",         "10",   "I2C0"],
        ["IMU INT1",        "13",   "Interrupt 1"],
        ["IMU INT2",        "12",   "Interrupt 2"],
        ["Pitot SDA",       "15",   "I2C1 (SDP810-500Pa)"],
        ["Pitot SCL",       "18",   "I2C1"],
        ["Batteria ADC",    "8",    "ADC1_CH7, divisore 3×"],
        ["PWR HOLD",        "7",    "Tenere HIGH per restare acceso"],
        ["BOOT button",     "0",    "GPIO0, active low"],
    ]
    story += [table(pins, [50*mm, 20*mm, 100*mm]), SP(2)]

    # ── 12. Specifiche tecniche ────────────────────────────────────────────────
    story += [P("12. Specifiche Tecniche", "h1"), hr()]

    specs = [
        ["Parametro",              "Valore"],
        ["MCU",                    "ESP32-S3 (dual-core Xtensa LX7, 240 MHz)"],
        ["Framework",              "ESP-IDF v5.3.2"],
        ["Display",                "ST7789T3 240×320 SPI, 40 MHz"],
        ["Frequenza rendering",    "5 Hz"],
        ["Sensore pitot",          "SDP810-500Pa (I2C1, 0x25), 10 Hz"],
        ["IMU",                    "QMI8658C (I2C0, 0x6B), 10 Hz"],
        ["Touch",                  "CST328 (bus GPIO1/3, 0x1A) - non usato"],
        ["WiFi",                   "802.11 b/g/n 2.4 GHz"],
        ["BLE",                    "BLE 5.0 (server GATT + central)"],
        ["Flash",                  "Partizioni OTA dual-bank"],
        ["RAM framebuffer",        "153.6 KB (240×320×2 byte, PSRAM preferito)"],
        ["Formula CdA",            "CdA = 2·P_aero / (ρ·v_aria²·v_terra)"],
        ["EMA smoothing",          "α = 1/30 s (30 campioni @ 1 Hz)"],
    ]
    story += [table(specs, [60*mm, 110*mm])]

    # ── Footer note ───────────────────────────────────────────────────────────
    story += [SP(6), hr(),
              P("Documento generato automaticamente dal firmware AeroDrag. "
                "Per supporto: repository fabrventurini-tech/aerodrag-firmware.", "note")]

    doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
    print(f"PDF generato: {OUTPUT}")


if __name__ == "__main__":
    build()
