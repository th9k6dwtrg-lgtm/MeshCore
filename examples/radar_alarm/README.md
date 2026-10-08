# Radar: hlídací čidlo s HLK-LD2410S

Firmware pro **Seeed XIAO nRF52840 + Wio-SX1262** s mmWave radarem **HLK-LD2410S**. Postavený stejně jako zahradní světlo (`examples/zahrada_light`): MeshCore Sensor, ovládání a upozornění přes soukromý kanál, watchdog, hlídání baterie, bez I2C (`XIAO_NO_I2C`). PlatformIO env: **`Xiao_nrf52_radar`**, hotový `.uf2` je v GitHub Actions (workflow „Zahrada - build XIAO“, artefakt `Xiao_nrf52_radar`).

Verze **zahrada-1.0** (vypíše ji příkaz `ver` v USB konzoli, stejně u světla), vydání je na GitHubu pod štítkem `zahrada-v1.0`.

## Zapojení

```
 18650 (+) ── BMS / ochrana ──────► BAT+   (pad na spodní straně XIAO)
 18650 (−) ───────────────────────► BAT−

 XIAO 3V3 ────────────────────────► LD2410S 3V3   (jen 3,3 V, 5 V radar zničí)
 XIAO GND ────────────────────────► LD2410S GND
 XIAO D7  ◄──────────────────────── LD2410S OT2   (na desce „OT2“, HIGH = přítomnost)
 XIAO NFC1 (P0.09) ◄─────────────── LD2410S OT1   (= UART TX radaru)    jen pro kalibraci
 XIAO NFC2 (P0.10) ────────────────► LD2410S RX                          a čtení prahů

 XIAO D6 ── 330 Ω ──►|── GND        testovací LED (anoda k rezistoru)
                                    později MOSFET stejně jako u zahradních světel
```

- **Nabíjení:** XIAO má vlastní nabíječku Li-Ion z USB-C (výchozí 50 mA). Pro solár jde použít stejné řešení jako u světel (CN3065 + BMS) nebo TP4056 s ochranou; nenechávat dvě nabíječky trvale na jednom článku.
- **Měření napětí:** dělič je na desce XIAO (1 MΩ / 510 kΩ na P0.31), firmware ho čte přes `board.getBattMilliVolts()`. Externí dělič není potřeba. ADC je nekalibrované (u světel ukazovalo asi o 0,15 V méně).
- **Piny:** D1–D5 a D8–D10 rádio, D0 tlačítko, D6 světlo, D7 radar. UART radaru je na **NFC padech** na spodní straně XIAO (NFC1 a NFC2, vedle padů baterie). Firmware je přepne na obyčejné piny (`CONFIG_NFCT_PINS_AS_GPIOS`; při prvním startu se to jednou zapíše do čipu a XIAO se samo restartuje). Bez zapojení UART vše funguje dál, jen `RADAR KALIBRACE` a `RADAR PRAHY` odpoví chybou.
- **Odběr (odhad, neměřeno):** jako světlo bez MT3608, tj. asi 8–12 mA (rádio musí stále poslouchat) + LD2410S. Jeden článek 3400 mAh tedy zhruba 2 týdny bez dobíjení.

## Příkazy

Velikost písmen nevadí. V soukromém kanálu (stejný mechanismus a klíč jako u světel) i přes CLI (USB nebo přihlášený admin).

| Příkaz | Co udělá | Odpověď |
|---|---|---|
| `RADAR ON` / `RADAR OFF` | zapne / vypne hlídání | `RADAR 1: RADAR ON` |
| `SVETLO ON` / `SVETLO OFF` | zapne / vypne rozsvícení na 3 s při pohybu | `RADAR 1: SVETLO ON` |
| `RON` | hned rozsvítí na 3 s (obdoba `LON`), i při `SVETLO OFF`; `c.` = pořadové číslo rozsvícení od zapnutí | `RADAR 1: ON 3s c.2 bat=3.95V` |
| `STATUS RADAR` | stav radaru | `RADAR 1: RADAR ON SVETLO OFF ot2=0/7 pohyb=3 (5 min) bat=3.95V rssi=-60 snr=9.5 up=0d02h15m` |
| `RADAR KALIBRACE` | za 60 s spustí automatické prahy radaru, sken 15 min (`RADAR KALIBRACE 20m` = 20 min, 2–60) | `RADAR 1: KALIBRACE za 60s, sken 15 min - odejdi z dosahu` |
| `RADAR KALIBRACE STOP` | zruší kalibraci: během prvních 60 s hned (radar beze změny), během skenu radar sken dokončí (zastavit ho neumí) a uzel pak vrátí prahy a parametry z doby před kalibrací | `RADAR 1: KALIBRACE STOP: radar dokonci sken (asi 12 min), pak vratim puvodni prahy`, po skenu `kalibrace zrusena, puvodni prahy vraceny` |
| `RADAR PRAHY` | prahy sepnutí bran 0–15 s hodnocením proti výchozím (viz níže) | `RADAR 1: sepnuti citlivejsi: 44-4 42 36 …` |
| `RADAR PRAHY H` | totéž pro prahy udržení | `RADAR 1: udrzeni jako vychozi: 45 42 …` |
| `RADAR PRAHY VYCHOZI` | zapíše do radaru výchozí prahy sepnutí i udržení a ověří je (také `výchozí` nebo `reset`) | `RADAR 1: VYCHOZI zapsany, sepnuti jako vychozi: 48 42 …` |
| `STATUS` | odpoví všechny uzly v kanálu, radar až po světlech 1–4 (6,6 s) | jako výše |
| `RADAR PRIKAZY` | tahák všech příkazů do kanálu, každý na svém řádku, ve 4 zprávách po 3 s; bez čísla odpoví jen radar 1 (nebo radar bez čísla), `RADAR PRIKAZY 2` radar 2 (také `příkazy`) | `RADAR 1: prikazy 1/4:` a pod tím řádky `RADAR ON - hlidat`, `RADAR OFF - nehlidat` … |

- `ot2` = okamžitý stav výstupu radaru (1 = přítomnost) / počet jeho sepnutí od startu (počítá i při RADAR OFF, slouží k testu radaru). `pohyb` = počet pohybů od `RADAR ON`, v závorce kdy byl poslední. `rssi/snr` = poslední přijatý paket (tj. tento příkaz, od nejbližšího souseda).
- Více radarů: číslo na konci jména (`RADAR 2`, světla `LIGHT 2`), pak `RADAR OFF 2`, `RON 2`, `STATUS 2`. Číslo jde přidat ke každému příkazu radaru (i víc čísel, `RON 1 3`). Bez čísla platí pro všechny radary.
- Zprávy od uzlů, jejichž jméno obsahuje „radar“ (`RADAR 2`, `dum-radar-2`), radar ignoruje. Jejich odpovědi jako `RADAR ON` by jinak ostatní radary braly jako příkaz a odpovídaly by si navzájem dokola. Člověk v kanálu proto nesmí mít „radar“ ve jménu.
- **Rozestupy odpovědí** jako u světel: radar 1 odpovídá za 0,6 s, radar 2 o 1,5 s později, …, radar bez čísla jako 5. (8 oken, radar 9 = okno radaru 1). Na `STATUS` a `STATUS 2` (odpovídají i světla) se radary posunou až za světla (+6,6 s), aby se radar 2 nesrazil se světlem 2; na `STATUS RADAR` odpovídají hned. Stejně se rozkládají i zprávy „radar pripraven“, baterie a výsledky kalibrace. `POHYB!` jde vždy hned.
- Zahradní světla na `STATUS RADAR`, `RADAR …` ani `SVETLO …` nereagují; radar nereaguje na `LON` ani `LIGHT PRIKAZY`. Světla od verze 1.0 nemají `LOFF`, zhasnou sama po 5 s.
- Světla mají vlastní tahák `LIGHT PRIKAZY` (odpoví světlo 1, `LIGHT PRIKAZY 2` světlo 2), jedna zpráva: `LIGHT 1: prikazy:` a řádky `LON - svetlo na 5 s`, `STATUS - stav vsech uzlu`, `LIGHT PRIKAZY - tento seznam`, `LON 2 = jen svetlo 2`.
- Tahák začíná slovem `prikazy`, které není příkaz, takže žádný uzel na něj nereaguje. Zprávy taháku jsou dlouhé až 139 znaků; s dlouhým jménem uzlu se rozdělí do více zpráv.
- Po každém zapnutí nebo restartu začíná radar s `RADAR ON` a `SVETLO OFF` (`RADAR_BOOT_ON`, `SVETLO_BOOT_ON`), změna příkazem platí jen do vypnutí. Klíč kanálu je v `/radar_ch`.

**Poplach:** při zapnutém hlídání a novém pohybu (náběžná hrana OT2) pošle hned `RADAR 1: POHYB! c.1 12:05:31 bat=3.95V SVETLO ON` (pořadové číslo zprávy o pohybu od zapnutí uzlu, místní čas zachycení, napětí a jestli se při pohybu rozsvítilo světlo). Další zpráva nejdřív za 60 s (výchozí `ALARM_COOLDOWN_SECS`, mění se příkazem `PAUZA`), se souhrnem `POHYB! c.2 12:05:40 3x za 60s bat=3.95V SVETLO ON`, kde čas je první pohyb ze souhrnu. Pořadové číslo je jen v RAM, po vypnutí a zapnutí začíná od 1. Čas je středoevropský včetně letního, dokud hodiny uzlu nesrovná první zpráva v kanálu, je místo něj `cas?`. Je-li zapnuté světlo, rozsvítí se na 3 s (`LIGHT_PULSE_SECS`) při každém pohybu. Prvních 300 s po startu (výchozí `RADAR_STARTUP_SECS`, mění se příkazem `WARMUP`; radar, který už má uloženou jinou hodnotu, si ji ponechá) se pohyb nevyhodnocuje (radar se ustaluje) a přítomnost, která trvá už při startu, se za pohyb nepovažuje. Na konci ustalování pošle do kanálu `radar pripraven (RADAR ON, SVETLO OFF)`, a je-li ustalování delší než 60 s, ještě 60 s předem `radar pripraven za 60 s (RADAR ON)`. Hodiny uzlu jsou po restartu nastavené až první zprávou v kanálu, takže tyto zprávy mohou mít v aplikaci staré datum. Zpráva jde jednou, floodem a bez potvrzení (stejně jako upozornění světel), takže přes slabé spojení nemusí dorazit; pro alarm se vyplatí vlastní repeater.

**Hlídání, že radar funguje:** zamrzlý radar nebo přerušený OT2 by jinak tiše přestal hlásit pohyb. Proto:
- V polovině ustalování po startu se uzel zeptá radaru přes UART. Když neodpoví, zpráva po startu je `radar pripraven (RADAR ON, SVETLO OFF), POZOR UART neodpovida` a `STATUS` ukazuje `uart=chyba` (bez zapojeného UART je to normální, jen nejde kalibrace).
- Pak každých 6 h (`RADAR_CHECK_HOURS`) znovu. Když radar, který už odpovídal, 3× po minutě neodpoví, pošle jednou `POZOR radar neodpovida (UART), asi nehlida. Vypni a zapni napajeni uzlu` (restart uzlu radar nerestartuje, ten jde z 3V3). Až zase odpoví, pošle `radar zase odpovida (UART)`. Po každé kontrole se 5 s nevyhodnocuje pohyb (OT2 může přeblikout).
- Když při `RADAR ON` drží OT2 přítomnost 30 min v kuse (`RADAR_STUCK_MIN`), pošle jednou `POZOR radar hlasi pritomnost uz 30 min, novy pohyb nepozna`. Dokud OT2 nespadne, další pohyb radar nepozná (zamrzlý radar, moc citlivé prahy, něco se hýbe v záběru).

**Baterie:** stejně jako světla, slabá < 3,50 V, kritická < 3,35 V, jedna zpráva do kanálu.

**Jen přes USB:** `ALERTTEST` (zkušební poplach do kanálu), `WDTTEST` (restart watchdogem).

**Nastavení přes CLI** (USB nebo přihlášený admin, ne z kanálu). Ukládá se do `/radar_cfg` a po restartu zůstává, takže všem radarům stačí jeden `.uf2`:

| Příkaz | Co udělá | Odpověď |
|---|---|---|
| `WARMUP 300` | ustalování po startu 10–900 s (bez čísla zobrazí), po skončení ustalování platí nová hodnota od dalšího startu | `OK warmup=300s` |
| `PAUZA 120` | nejkratší odstup zpráv o pohybu 10–3600 s | `OK pauza=120s` |
| `BATKAL 4.12` | korekce měření baterie: zadej napětí naměřené multimetrem na článku (2,5–4,5 V, čárka i tečka, korekce max. ±20 %) | `batkal=1.026 bat=4.12V` |
| `BATKAL OFF` | bez korekce | `batkal=1.000 bat=…` |
| `NASTAVENI` | vše najednou | `warmup=300s pauza=60s batkal=1.000 bat=3.95V` |

Korekce `BATKAL` platí pro napětí ve zprávách, `STATUS` i hlídání slabé baterie (telemetrie v aplikaci zůstává bez korekce). `BATKAL` a `CHAN` umí stejně i zahradní světlo, ukládá se do `/batkal`.

## Společný kód se světly
Kanál (`CHAN`, ochrana proti přehrání, odesílání zpráv), hlídání baterie s `BATKAL`, rozestupy podle čísla uzlu, watchdog a doba běhu jsou v `examples/zahrada_common/ZahradaNode.h`, ze kterého vychází světlo (`examples/zahrada_light`) i radar. Pořád jsou to dva samostatné firmwary: světlo umí jen `LON`/`STATUS`/`LIGHT PRIKAZY` a spíná D6. Ochrana proti přehrání si pamatuje poslední časové razítko až 16 odesílatelů a ukládá ho do flash (`/chan_ts`, nejvýš 1× za minutu), takže stará nahraná zpráva neprojde ani po restartu; `CHAN <klíč>` tabulku vymaže. Po změně společného souboru je potřeba projít testy obou (`./zahrada_test/run.sh` a `./radar_test/run.sh`, v Actions běží automaticky).

## Nastavení nového uzlu (USB konzole)
Stejně jako světlo: `ver`, `get radio`, `set name RADAR 1`, `password …`, `set path.hash.mode 1`, `get advert.interval` (případně `set advert.interval 0`), `chan <klíč>`, `advert.zerohop`. Pak `RON`, `RADAR ON`, projít před radarem a zkontrolovat zprávu v kanálu.

Heslo ani klíč kanálu nepatří do kódu (repozitář je veřejný).

## Kalibrace přes mesh
`RADAR KALIBRACE` (v kanálu i přes CLI) zapne UART radaru a ověří, že odpovídá. Pak má člověk 60 s (`RADAR_KAL_DELAY_SECS`) na odchod z dosahu, radar dostane příkaz automatických prahů (0x0009, faktory 2 a 1 jako nástroj Hi-Link, doba skenu v sekundách) a sám si změří prázdný prostor. Firmware počká, až radar nahlásí 100 %, a pak ještě 20 s. Radar po skenu asi 10 s ukládá prahy a neodpovídá; dotaz hned po 100 % ho jednou zasekl až do vypnutí napájení (LD2410S, 8. 10. 2026). Sken trvá o pár procent déle než zadaná doba (120 s → 124 s). Když 100 % nepřijde, firmware čeká dobu skenu + 25 %. Pak přečte nové prahy (když radar neodpoví, zkusí to znovu každých 10 s, nejvýš 6×), pošle do kanálu srozumitelné shrnutí v metrech, např. `kalibrace OK: sepnuti citlivejsi o 1-5 (12 bran), mene citlive o 3 (0,7-1,4 m)` a `kalibrace OK: udrzeni citlivejsi o 1-5 (10 bran), mene citlive o 3 (0,7-1,4 m). Cisla: RADAR PRAHY` (brána je asi 0,7 m, rozsah v metrech se uvede, když jsou změněné brány souvislé; při odchylce 10 a víc `kalibrace POZOR: … Zopakuj nebo RADAR PRAHY VYCHOZI`; když se vejde do jedné zprávy, přijde jedna). Čísla po branách vrátí `RADAR PRAHY` a UART zase vypne (šetří baterii). Obecné parametry (dosah, doba držení…) si firmware přečte před skenem a když je radar během kalibrace změní (viděno: doba držení 10 → 40 s), vrátí je a pošle třetí zprávu `kalibrace: radar zmenil sve nastaveni, vraceno zpet`. Když radar neodpoví ani na 6. pokus, přijde `kalibrace CHYBA: radar neodpovida, vypni a zapni uzel`. Během čekání i skenu se pohyb nehlásí; `STATUS` ukazuje `kal=start` a pak průběh hlášený radarem. Během skenu nesmí nikdo projít ani projet a výsledek je dobré zkontrolovat (`RADAR PRAHY`, průchod před radarem, `ot2` ve `STATUS`).

## Jak číst prahy (je kalibrace v pořádku?)
Práh je energie odrazu, kterou musí pohyb v dané bráně (vzdálenostním pásmu, brána 0 nejblíž) překročit. **Vyšší číslo = méně citlivé.** Sepnutí rozhoduje, kdy radar ohlásí příchod; udržení, jak dlouho přítomnost drží.

Výpis porovnává každou bránu s **výchozími prahy** (hodnoty z nového modulu: sepnutí 48 42 36 34 32 31 …, udržení 45 42 33 32 28 …; protokol tovární hodnoty neuvádí). U změněné brány je za hodnotou rozdíl: `44-4` = o 4 citlivější, `55+7` = o 7 méně citlivá. Na začátku je souhrn:

| Souhrn | Význam |
|---|---|
| `jako vychozi` / `skoro jako vychozi` | žádná brána se neliší o víc než 3 |
| `citlivejsi` / `mene citlive` / `citlivejsi i mene citlive` | některé brány se liší o 4–9, kalibrace prostor upravila, to je normální |
| `POZOR prilis citlive` | některá brána je o 10 a víc citlivější, hrozí plané poplachy |
| `POZOR malo citlive` | některá brána je o 10 a víc méně citlivá, radar nemusí člověka zachytit (typicky když se při skenu někdo pohyboval) |

Hranice 3 a 10 jsou odhad, ne údaj výrobce. Hodnotí se jen brány do nejvzdálenější použité brány (parametr far, výchozí 12); vzdálenější radar nepoužívá. Při `POZOR` kalibraci zopakuj s prázdným prostorem, nebo vrať `RADAR PRAHY VYCHOZI`. Nejjistější kontrola je vždy průchod před radarem a `ot2` ve `STATUS`.

## Nastavení radaru
Dosah, citlivost a doba držení OT2 se nastavují přes UART radaru (115200 Bd) programem HLK-LD2410S_TOOL na PC. Radar odpojit od XIAO a připojit na USB-UART převodník přepnutý na **3,3 V** (např. LaskaKit CH9102): VCC → 3V3, GND → GND, TX převodníku → RX radaru, RX převodníku → OT1 radaru. Delší doba držení = méně opakovaných pohybů. Pro první zkoušku stačí výchozí nastavení.

## Testy
`./radar_test/run.sh`: logika příkazů, kanálu, poplachu a baterie na PC (napodoba MeshCore ze `zahrada_test/mocks.h`), běží v Actions před kompilací.
