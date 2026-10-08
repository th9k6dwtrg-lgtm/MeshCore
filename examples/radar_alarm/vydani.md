Radarový hlídač a zahradní světla pro MeshCore (Seeed XIAO nRF52840 + Wio-SX1262, radar HLK-LD2410S). Základ MeshCore v1.17.1.

**Soubory**
- `Xiao_nrf52_radar-zahrada-1.0.uf2`: firmware radarového uzlu
- `Xiao_nrf52_light-zahrada-1.0.uf2`: firmware zahradního světla
- `radarovy-hlidac-meshcore-navod.pdf`: návod od součástek po příkazy v kanálu
- `ld2410s.py`: nastavení radaru z počítače přes USB-UART

Nahrání: 2× reset na XIAO, na disk, který se objeví, zkopírovat `.uf2`. Klíč kanálu, jméno, heslo a nastavení zůstávají uložené. `ver` v USB konzoli vypíše `zahrada-1.0`.

**Ve verzi 1.0**
- Ovládání ze soukromého kanálu: `RADAR ON/OFF`, `SVETLO ON/OFF`, `RON`, `STATUS`, `STATUS RADAR`, `LON`, tahák `RADAR PRIKAZY` / `LIGHT PRIKAZY`
- Zprávy `POHYB!` s místním časem (CET/CEST), `cas?` dokud hodiny nesrovná první zpráva v kanálu
- Kalibrace a prahy radaru přes mesh: `RADAR KALIBRACE [20m|STOP]`, `RADAR PRAHY [H|VYCHOZI]`
- Hlídání, že radar funguje (UART každých 6 h, trvalá přítomnost 30 min), watchdog 120 s, upozornění na slabou baterii
- Ochrana proti přehrání pro až 16 odesílatelů, platí i po restartu
- Výchozí ustálení radaru po startu 300 s (`WARMUP`)
