#!/usr/bin/env python3
# Genera components/display/aafont_data.h: atlanti di glifi anti-aliased (alpha 8-bit)
# pre-rasterizzati dai TTF reali. Nessun FreeType a runtime sul MCU: si renderizza
# il bitmap alpha in alpha-blend (fb_text in display.h).
#
# Coppia font (coerente con app + dashboard AeroDrag):
#   numeri/valori -> JetBrains Mono (Bold 700, ExtraBold 800 per il grade) — OFL
#   label/unità   -> Inter SemiBold (600)                                   — OFL
#
# Uso: python3 tools/genfont.py   (richiede Pillow e i TTF in tools/fonts/)
import os
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FONTS = os.path.join(HERE, "fonts")
OUT = os.path.join(HERE, "..", "components", "display", "aafont_data.h")

JB_B = os.path.join(FONTS, "JetBrainsMono-Bold.ttf")       # 700
JB_XB = os.path.join(FONTS, "JBExtraBold.ttf")             # 800 (JetBrainsMono-ExtraBold)
INTER = os.path.join(FONTS, "Inter-SemiBold.ttf")          # 600

NUM = "0123456789.,:%+-/ ABCD"   # ABCD = lettere grade in JetBrains Mono
GRADE = "ABCDEF+-0123456789 "
ASCII = "".join(chr(c) for c in range(0x20, 0x7F))
LABEL = ASCII + "²°·"   # +  ²  °  ·

# (nome, ttf, px, charset)  — px = dimensione reale a 240x320, NON scalata
ATLASES = [
    ("AAF_HERO",  JB_B,  46, NUM),    # hero numerico
    ("AAF_GRADE", JB_XB, 48, GRADE),  # grade "A" (ExtraBold)
    ("AAF_VAL",   JB_B,  34, NUM),    # valore primario (0.214)
    ("AAF_MED",   JB_B,  26, NUM),    # valore medio (barra inf, tempi)
    ("AAF_NUMS",  JB_B,  14, NUM),    # numeri piccoli (wind)
    ("AAF_LABEL", INTER, 13, LABEL),  # label/unità
    ("AAF_LBLS",  INTER, 11, LABEL),  # label minime (min assoluto 11px)
]


def main():
    out = []
    out.append("// AUTO-GENERATED da tools/genfont.py — NON modificare a mano.")
    out.append("// Atlanti glifi anti-aliased (alpha 8-bit). JetBrains Mono + Inter (OFL).")
    out.append("#pragma once")
    out.append("#include <stdint.h>")
    out.append("")
    out.append("typedef struct { uint16_t cp; uint8_t w,h; int8_t xoff,yoff; uint8_t adv; uint32_t off; } aaglyph_t;")
    out.append("typedef struct { const aaglyph_t *g; uint16_t n; const uint8_t *bm; uint8_t ascent; uint8_t px; } aafont_t;")
    out.append("")

    total = 0
    for name, path, px, chars in ATLASES:
        font = ImageFont.truetype(path, px)
        ascent, _ = font.getmetrics()
        PAD = px
        glyphs = []
        blob = bytearray()
        for ch in chars:
            cp = ord(ch)
            adv = round(font.getlength(ch))
            img = Image.new("L", (px * 3, px * 3), 0)
            ImageDraw.Draw(img).text((PAD, PAD), ch, font=font, fill=255, anchor="la")
            bbox = img.getbbox()
            if bbox is None:                       # spazio / glifo vuoto
                glyphs.append((cp, 0, 0, 0, 0, adv, len(blob)))
                continue
            x0, y0, x1, y1 = bbox
            glyphs.append((cp, x1 - x0, y1 - y0, x0 - PAD, y0 - PAD, adv, len(blob)))
            blob += img.crop(bbox).tobytes()

        bm = name + "_BM"
        out.append("static const uint8_t %s[%d] = {" % (bm, len(blob)))
        line = "  "
        for b in blob:
            line += "%d," % b
            if len(line) > 110:
                out.append(line); line = "  "
        if line.strip():
            out.append(line)
        out.append("};")

        g = name + "_G"
        out.append("static const aaglyph_t %s[%d] = {" % (g, len(glyphs)))
        for (cp, w, h, xo, yo, adv, off) in glyphs:
            out.append("  {%d,%d,%d,%d,%d,%d,%d}," % (cp, w, h, xo, yo, adv, off))
        out.append("};")
        out.append("static const aafont_t %s = { %s, %d, %s, %d, %d };" %
                   (name, g, len(glyphs), bm, ascent, px))
        out.append("")
        total += len(blob)
        print("%-10s px=%2d glyphs=%2d  bitmap=%d B" % (name, px, len(glyphs), len(blob)))

    print("TOTAL atlas bitmap bytes:", total)
    with open(OUT, "w") as f:
        f.write("\n".join(out))
    print("written", OUT)


if __name__ == "__main__":
    main()
