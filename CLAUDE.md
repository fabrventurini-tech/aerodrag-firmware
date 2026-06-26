# AeroDrag — istruzioni di repo (firmware ESP32-S3)

Questo repo è una **figlia** del progetto AeroDrag (firmware ESP32-S3, ESP-IDF) e
**ospita** il contratto d'interfaccia in `docs/CONTRACT.md`.

## Contratto — fonte di verità unica (OSPITATO qui, editing riservato a MOTHER)
`docs/CONTRACT.md` su `main` è l'UNICA fonte autorevole delle interfacce fra tutti i
componenti. Questo repo **lo ospita ma non ne possiede l'editing**: lo modifica e lo
**ratifica solo MOTHER** — l'orchestratore che sta **sopra tutti i repo** (ruolo/team
`@.../mother`). La chat firmware **implementa** §2/§3/§5/§6 per l'ESP32 e **non cambia il
contratto**: per un cambiamento d'interfaccia **proponi via seam issue a Mother**, non
editare il file. Copie del contratto altrove = "copia di lavoro, NON autorevole".
