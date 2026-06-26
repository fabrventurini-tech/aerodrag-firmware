# AeroDrag — istruzioni di repo (firmware ESP32-S3)

Questo repo è una **chat figlia** del progetto AeroDrag (firmware ESP32-S3, ESP-IDF) e
**ospita** il contratto d'interfaccia in `docs/CONTRACT.md`. Il coordinamento fra tutti i
repo è gestito da **MOTHER** (ruolo/team `@.../mother`), che sta sopra tutti i repo.

## Contratto — fonte di verità unica (OSPITATO qui, editing riservato a MOTHER)
`docs/CONTRACT.md` su `main` è l'UNICA fonte autorevole delle interfacce fra tutti i
componenti. Questo repo **lo ospita ma non ne possiede l'editing**: lo modifica e lo
**ratifica solo MOTHER**. La chat firmware **implementa** §2/§3/§5/§6 per l'ESP32 e **non
cambia il contratto**.

Vale solo la versione **RATIFICATA** su `main`. Eventuali bump **in lavorazione** (es.
seam issue aperte o draft della versione successiva) **NON sono autorevoli** finché MOTHER
non li **ratifica** su `main`: fino ad allora resta valida l'ultima versione ratificata.

Per un cambiamento d'interfaccia **proponi via seam issue a MOTHER**, non editare il file.
Copie del contratto altrove = "copia di lavoro, NON autorevole".
