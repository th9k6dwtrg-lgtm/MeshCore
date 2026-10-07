# Radar: hlídací čidlo s HLK-LD2410S

Firmware pro **Seeed XIAO nRF52840 + Wio-SX1262** s mmWave radarem **HLK-LD2410S**. Postavený stejně jako zahradní světlo (`examples/zahrada_light`): MeshCore Sensor, ovládání a upozornění přes soukromý kanál, watchdog, hlídání baterie, bez I2C (`XIAO_NO_I2C`). PlatformIO env: **`Xiao_nrf52_radar`**, hotový `.uf2` je v GitHub Actions (workflow „Zahrada - build XIAO“, artefakt `Xiao_nrf52_radar`).

## Zapojení

```
 18650 (+) ── BMS / ochrana ──────► BAT+   (pad na spodní straně XIAO)
 18650 (−) ───────────────────────► BAT−

 XIAO 3V3 ────────────────────────► LD2410S 3V3   (jen 3,3 V, 5 V radar zničí)
 XIAO GND ────────────────────────► LD2410S GND
 XIAO D7  ◄──────────────────────── LD2410S OT2   (na desce „OT2“, HIGH = přítomnost)
                                    LD2410S OT1 (= UART TX), RX: nezapojeno

 XIAO D6 ── 330 Ω ──►|── GND        testovací LED (anoda k rezistoru)
                                    později MOSFET stejně jako u zahradních světel
```

- **Nabíjení:** XIAO má vlastní nabíječku Li-Ion z USB-C (výchozí 50 mA). Pro solár jde použít stejné řešení jako u světel (CN3065 + BMS) nebo TP4056 s ochranou; nenechávat dvě nabíječky trvale na jednom článku.
- **Měření napětí:** dělič je na desce XIAO (1 MΩ / 510 kΩ na P0.31), firmware ho čte přes `board.getBattMilliVolts()`. Externí dělič není potřeba. ADC je nekalibrované (u světel ukazovalo asi o 0,15 V méně).
- **Piny:** D1–D5 a D8–D10 rádio, D0 tlačítko, D6 světlo, D7 radar.
- **Odběr (odhad, neměřeno):** jako světlo bez MT3608, tj. asi 8–12 mA (rádio musí stále poslouchat) + LD2410S. Jeden článek 3400 mAh tedy zhruba 2 týdny bez dobíjení.

## Příkazy

Velikost písmen nevadí. V soukromém kanálu (stejný mechanismus a klíč jako u světel) i přes CLI (USB nebo přihlášený admin).

| Příkaz | Co udělá | Odpověď |
|---|---|---|
| `RADAR ON` / `RADAR OFF` | zapne / vypne hlídání | `dum-radar: RADAR ON` |
| `SVETLO ON` / `SVETLO OFF` | zapne / vypne rozsvícení na 3 s při pohybu | `dum-radar: SVETLO ON` |
| `SVETLO TEST` | rozsvítí na 3 s | `dum-radar: SVETLO TEST 3s` |
| `STATUS RADAR` | stav radaru | `dum-radar: RADAR ON SVETLO OFF ot2=0/7 pohyb=3 (5m) bat=3.95V rssi=-60 snr=9.5 up=0d02h15m` |
| `STATUS` | odpoví všechny uzly v kanálu, radar až po světlech 1–4 (6,6 s) | jako výše |

- `ot2` = okamžitý stav výstupu radaru (1 = přítomnost) / počet jeho sepnutí od startu (počítá i při RADAR OFF, slouží k testu radaru). `pohyb` = počet pohybů od `RADAR ON`, v závorce kdy byl poslední. `rssi/snr` = poslední přijatý paket (tj. tento příkaz, od nejbližšího souseda).
- Více radarů: číslo na konci jména (`dum-radar-2`), pak `RADAR OFF 2`, `STATUS 2`. Bez čísla platí pro všechny radary.
- Zahradní světla na `STATUS RADAR`, `RADAR …` ani `SVETLO …` nereagují; radar nereaguje na `LON/LOFF`.
- Stav RADAR/SVETLO se ukládá (`/radar_cfg`) a po restartu zůstává. Klíč kanálu je v `/radar_ch`.

**Poplach:** při zapnutém hlídání a novém pohybu (náběžná hrana OT2) pošle hned `dum-radar: POHYB! bat=3.95V`. Další zpráva nejdřív za 60 s (`ALARM_COOLDOWN_SECS`), se souhrnem `POHYB! 3x za 60s, …`. Je-li zapnuté světlo, rozsvítí se na 3 s (`LIGHT_PULSE_SECS`) při každém pohybu. Prvních 30 s po startu (`RADAR_STARTUP_SECS`, ve finální verzi 300) se pohyb nevyhodnocuje (radar se ustaluje) a přítomnost, která trvá už při startu, se za pohyb nepovažuje. Na konci ustalování pošle do kanálu `radar pripraven (RADAR ON, SVETLO OFF)`, a je-li ustalování delší než 60 s, ještě 60 s předem `radar pripraven za 60 s (RADAR ON)`. Hodiny uzlu jsou po restartu nastavené až první zprávou v kanálu, takže tyto zprávy mohou mít v aplikaci staré datum. Zpráva jde jednou, floodem a bez potvrzení (stejně jako upozornění světel), takže přes slabé spojení nemusí dorazit; pro alarm se vyplatí vlastní repeater.

**Baterie:** stejně jako světla, slabá < 3,50 V, kritická < 3,35 V, jedna zpráva do kanálu.

**Jen přes USB:** `ALERTTEST` (zkušební poplach do kanálu), `WDTTEST` (restart watchdogem).

## Nastavení nového uzlu (USB konzole)
Stejně jako světlo: `ver`, `get radio`, `set name dum-radar`, `password …`, `set path.hash.mode 1`, `get advert.interval` (případně `set advert.interval 0`), `chan <klíč>`, `advert.zerohop`. Pak `SVETLO TEST`, `RADAR ON`, projít před radarem a zkontrolovat zprávu v kanálu.

Heslo ani klíč kanálu nepatří do kódu (repozitář je veřejný).

## Nastavení radaru
Dosah, citlivost a doba držení OT2 se nastavují přes UART radaru (115200 Bd) programem HLK-LD2410S_TOOL na PC. Radar odpojit od XIAO a připojit na USB-UART převodník přepnutý na **3,3 V** (např. LaskaKit CH9102): VCC → 3V3, GND → GND, TX převodníku → RX radaru, RX převodníku → OT1 radaru. Delší doba držení = méně opakovaných pohybů. Pro první zkoušku stačí výchozí nastavení.

## Testy
`./radar_test/run.sh`: logika příkazů, kanálu, poplachu a baterie na PC (napodoba MeshCore ze `zahrada_test/mocks.h`), běží v Actions před kompilací.
