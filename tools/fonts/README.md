# Font del display (atlanti AA)

`tools/genfont.py` pre-rasterizza questi TTF in `components/display/aafont_data.h`
(atlanti di glifi anti-aliased, alpha 8-bit). Nessun FreeType a runtime sul MCU:
il firmware fa solo alpha-blend dei glifi già rasterizzati.

| Ruolo            | Font                         | Peso        | Licenza |
|------------------|------------------------------|-------------|---------|
| Numeri / valori  | JetBrains Mono               | Bold (700)  | OFL 1.1 |
| Grade "A"        | JetBrains Mono ExtraBold     | 800         | OFL 1.1 |
| Label / unità    | Inter                        | SemiBold 600| OFL 1.1 |

Coerente con la coppia font di app (`aerodrag-new`) e dashboard (`aerodrag-pi`).

## Rigenerare gli atlanti
```
pip install pillow
python3 tools/genfont.py
```
Modifica `ATLASES` / `NUM` / `LABEL` in `genfont.py` per cambiare dimensioni px
o subset di glifi (riduce la flash). Le dimensioni sono in **px reali** a 240×320,
non vanno scalate ×2.

## Provenienza
- `JetBrainsMono-Bold.ttf`, `JBExtraBold.ttf` — https://github.com/JetBrains/JetBrainsMono (OFL)
- `Inter-SemiBold.ttf` — Inter di Rasmus Andersson, https://github.com/rsms/inter (OFL)

Entrambi i progetti sono rilasciati sotto SIL Open Font License 1.1.
