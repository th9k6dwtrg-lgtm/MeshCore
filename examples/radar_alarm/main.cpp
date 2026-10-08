#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

// Hlidaci cidlo s mmWave radarem HLK-LD2410S na XIAO nRF52840 + Wio-SX1262.
// Zapojeni: LD2410S 3V3/GND na 3V3/GND XIAO, LD2410S OT2 -> D7 (HIGH = pritomnost),
//           volitelne pro kalibraci: OT1 (TX radaru) -> NFC1 pad (P0.09), NFC2 pad (P0.10) -> RX radaru,
//           D6 -> svetlo (pro test LED pres 330 R na GND, pozdeji MOSFET jako u zahradnich svetel).
// Ovladani a upozorneni jdou stejne jako u zahradnich svetel pres soukromy kanal MeshCore.

#ifndef LIGHT_PULSE_SECS
  #define LIGHT_PULSE_SECS  3     // jak dlouho svetlo sviti po zachyceni pohybu (sekundy)
#endif
#define LIGHT_PULSE_MS  ((uint32_t)LIGHT_PULSE_SECS * 1000UL)

#ifndef ALARM_COOLDOWN_SECS
  #define ALARM_COOLDOWN_SECS  60 // dalsi zprava o pohybu nejdriv po teto dobe (sit je pro lidi, ne pro spam)
#endif
#define ALARM_COOLDOWN_MS  ((uint32_t)ALARM_COOLDOWN_SECS * 1000UL)

#ifndef RADAR_STARTUP_SECS
  #define RADAR_STARTUP_SECS  30  // po startu uzlu (a radaru) se pohyb nevyhodnocuje: radar se ustaluje
#endif
#define RADAR_STARTUP_MS  ((uint32_t)RADAR_STARTUP_SECS * 1000UL)
#define RADAR_READY_WARN_SECS  60 // zprava "pripraven za 60 s" (jen kdyz je ustalovani delsi nez 60 s)

#ifndef REPLY_ALL_DELAY_MS
  #define REPLY_ALL_DELAY_MS  6600  // odpoved na prikaz pro vsechny (STATUS): az po zahradnich svetlech 1-4
#endif

static uint32_t light_on_since = 0;      // millis() okamziku rozsviceni
static bool light_timer_armed = false;   // odpocet do zhasnuti bezi

static void lightTimerStart() {
  light_on_since = millis();
  light_timer_armed = true;
}

// Hlida dobu svitu: plati pro pohyb, SVETLO TEST i pro oficialni 'io s 1'.
static void lightTimerLoop() {
  if ((board.getGpio() & 1) == 0) { light_timer_armed = false; return; }
  if (!light_timer_armed) lightTimerStart();     // rozsviceno jinak (napr. 'io s 1')
  if ((uint32_t)(millis() - light_on_since) >= LIGHT_PULSE_MS) {
    board.setGpio(board.getGpio() & ~1u);        // automaticke zhasnuti
    light_timer_armed = false;
  }
}

// ---------- hlidaci obvod (watchdog) nRF52 ----------
// Stejne jako u zahradnich svetel: kdyz hlavni smycka nebezi WDT_TIMEOUT_SECS, cip se restartuje.
#ifndef WDT_TIMEOUT_SECS
  #define WDT_TIMEOUT_SECS  120
#endif
#ifdef NRF52_PLATFORM
static void wdtStart() {
  if ((NRF_WDT->RUNSTATUS & 1) == 0) {     // jeste nebezi (po vypnuti napajeni / resetu pinem)
    NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
    NRF_WDT->CRV = (uint32_t)WDT_TIMEOUT_SECS * 32768UL;
    NRF_WDT->RREN = WDT_RREN_RR0_Msk;
    NRF_WDT->TASKS_START = 1;
  }
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
static inline void wdtFeed() { NRF_WDT->RR[0] = WDT_RR_RR_Reload; }
#else
static void wdtStart() { }
static inline void wdtFeed() { }
#endif

// ---------- doba behu od startu (64bit, nepretece po 49 dnech) ----------
static uint64_t uptime_ms = 0;
static uint32_t uptime_last = 0;
static void uptimeLoop() {
  uint32_t now = millis();
  uptime_ms += (uint32_t)(now - uptime_last);
  uptime_last = now;
}

// ---------- UART radaru LD2410S: kalibrace (automaticke prahy) a cteni prahu ----------
// Protokol "HLK-LD2410S serial communication protocol V1.00": 115200 Bd, prikaz FD FC FB FA | delka | slovo | data | 04 03 02 01,
// datove ramce F4 F3 F2 F1 | delka | data | F8 F7 F6 F5. Zapojeni: OT1 (TX radaru) -> PIN_RADAR_UART_RX,
// PIN_RADAR_UART_TX -> RX radaru (NFC pady XIAO, D6/D7 zustavaji pro svetlo a OT2).
// UART bezi jen behem kalibrace / cteni prahu (setri baterii).
#ifndef RADAR_KAL_MINUTES
  #define RADAR_KAL_MINUTES     15   // vychozi doba skenovani prostoru
#endif
#ifndef RADAR_KAL_DELAY_SECS
  #define RADAR_KAL_DELAY_SECS  60   // po prikazu cas odejit z dosahu radaru, pak teprve sken
#endif
#define RADAR_KAL_GRACE_MS      20000UL   // po uplynuti skenu jeste pockat na radar, pak precist prahy
#define RADAR_KAL_TRIGGER_FACTOR  2       // parametry prikazu 0x0009 jako v nastroji Hi-Link
#define RADAR_KAL_HOLD_FACTOR     1
#define LD_GATES  16

#ifdef PIN_RADAR_UART_RX
#ifndef RADAR_UART
  #define RADAR_UART  Serial1
#endif

// Vychozi prahy = hodnoty prectene z noveho modulu LD2410S (8. 10. 2026, far=12); protokol tovarni hodnoty neuvadi.
// Prah je energie, kterou musi pohyb v dane brane prekrocit: vyssi cislo = mene citlive.
static const uint8_t LD_DEF_TRIGGER[LD_GATES] = {48, 42, 36, 34, 32, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31};
static const uint8_t LD_DEF_HOLD[LD_GATES]    = {45, 42, 33, 32, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28};
#define LD_SAME_DIFF   3   // odchylka do +-3 od vychozi hodnoty = "skoro jako vychozi"
#define LD_WARN_DIFF  10   // od +-10 "POZOR" (odhad z praxe, ne udaj vyrobce)

// radek prahu pro cloveka, napr. "sepnuti citlivejsi: 44-4 42 36 ...": u zmenenych bran rozdil proti vychozi
// hodnote (minus = citlivejsi, plus = mene citlive). Hodnoceni jen pro brany 0..far_gate, ktere radar pouziva.
static void thresholdLine(char* dest, size_t max, const char* label, const uint8_t* v, const uint8_t* def, int far_gate) {
  int dmin = 0, dmax = 0;
  bool changed = false;
  for (int g = 0; g <= far_gate && g < LD_GATES; g++) {
    int d = (int)v[g] - def[g];
    if (d < dmin) dmin = d;
    if (d > dmax) dmax = d;
    if (d != 0) changed = true;
  }
  const char* verdict;
  if (dmin <= -LD_WARN_DIFF) verdict = "POZOR prilis citlive";
  else if (dmax >= LD_WARN_DIFF) verdict = "POZOR malo citlive";
  else if (dmin < -LD_SAME_DIFF && dmax > LD_SAME_DIFF) verdict = "citlivejsi i mene citlive";
  else if (dmin < -LD_SAME_DIFF) verdict = "citlivejsi";
  else if (dmax > LD_SAME_DIFF) verdict = "mene citlive";
  else verdict = changed ? "skoro jako vychozi" : "jako vychozi";
  int p = snprintf(dest, max, "%s %s:", label, verdict);
  for (int g = 0; g < LD_GATES && p > 0 && p < (int)max; g++) {
    int d = (int)v[g] - def[g];
    if (d != 0) p += snprintf(dest + p, max - p, " %u%+d", v[g], d);
    else p += snprintf(dest + p, max - p, " %u", v[g]);
  }
}

class LD2410S {
  uint8_t buf[96];
  int n = 0;

  // vraci delku dat ramce (bez hlavicky, delky a konce) nebo -1, kdyz zatim neni cely ramec
  int rx(uint8_t* p, int max, bool& is_ack) {
    static const uint8_t A[4] = {0xFD, 0xFC, 0xFB, 0xFA}, D[4] = {0xF4, 0xF3, 0xF2, 0xF1};
    while (RADAR_UART.available()) {
      uint8_t c = RADAR_UART.read();
      if (n < 4) {   // hledani hlavicky (minimalni ramce 6E .. 62 se tim preskoci)
        const uint8_t* h = (n > 0 && buf[0] == 0xF4) ? D : A;
        if (n == 0 && c == 0xF4) h = D;
        if (c == h[n]) buf[n++] = c;
        else { n = 0; if (c == 0xFD || c == 0xF4) buf[n++] = c; }
        continue;
      }
      buf[n++] = c;
      if (n >= 6) {
        int len = buf[4] | (buf[5] << 8);
        if (len > (int)sizeof(buf) - 10) { n = 0; continue; }
        if (n == 6 + len + 4) {
          n = 0;
          is_ack = buf[0] == 0xFD;
          int m = len < max ? len : max;
          memcpy(p, &buf[6], m);
          return m;
        }
      }
    }
    return -1;
  }

  void dataFrame(const uint8_t* p, int l) {
    // prubeh automatickych prahu: typ 0x03, posledni 2 bajty = prubeh
    if (l >= 3 && p[0] == 0x03) progress = p[l - 2] | (p[l - 1] << 8);
  }

public:
  int progress = -1;   // posledni hlaseny prubeh automatickych prahu (-1 = zatim nic)

  void begin() {
    RADAR_UART.setPins(PIN_RADAR_UART_RX, PIN_RADAR_UART_TX);
    RADAR_UART.begin(115200);
    while (RADAR_UART.available()) RADAR_UART.read();
    n = 0;
    progress = -1;
  }
  void end() { RADAR_UART.end(); }

  // posle prikaz a pocka na potvrzeni (3 pokusy po 300 ms); data odpovedi za stavem do out
  bool command(uint16_t cmd, const uint8_t* data, int dlen, uint8_t* out = NULL, int out_max = 0) {
    uint8_t f[4 + 2 + 2 + 6 * LD_GATES + 4];
    int fl = 0;
    memcpy(f, "\xFD\xFC\xFB\xFA", 4); fl = 4;
    f[fl++] = (uint8_t)(dlen + 2); f[fl++] = 0;
    f[fl++] = cmd & 0xFF; f[fl++] = cmd >> 8;
    memcpy(&f[fl], data, dlen); fl += dlen;
    memcpy(&f[fl], "\x04\x03\x02\x01", 4); fl += 4;
    for (int attempt = 0; attempt < 3; attempt++) {
      RADAR_UART.write(f, fl);
      uint32_t t0 = millis();
      while ((uint32_t)(millis() - t0) < 300) {
        uint8_t p[80];
        bool ack = false;
        int l = rx(p, sizeof(p), ack);
        if (l < 0) continue;
        if (!ack) { dataFrame(p, l); continue; }
        if (l < 4 || (p[0] | (p[1] << 8)) != (cmd | 0x0100)) continue;
        if (p[2] | p[3]) return false;   // radar prikaz odmitl
        if (out) memcpy(out, &p[4], (l - 4) < out_max ? (l - 4) : out_max);
        return true;
      }
    }
    return false;
  }

  bool configOn()  { static const uint8_t d[2] = {0x01, 0x00}; return command(0x00FF, d, 2); }
  bool configOff() { return command(0x00FE, NULL, 0); }

  // prahy pro vsech 16 bran: 0x0073 = trigger (sepnuti), 0x0077 = hold (udrzeni)
  bool readThresholds(uint16_t cmd, uint8_t vals[LD_GATES]) {
    uint8_t d[2 * LD_GATES], out[4 * LD_GATES];
    for (int g = 0; g < LD_GATES; g++) { d[2 * g] = g; d[2 * g + 1] = 0; }
    if (!command(cmd, d, sizeof(d), out, sizeof(out))) return false;
    for (int g = 0; g < LD_GATES; g++) vals[g] = out[4 * g];
    return true;
  }

  bool startAuto(uint16_t scan_secs) {
    uint8_t d[6] = {RADAR_KAL_TRIGGER_FACTOR, 0, RADAR_KAL_HOLD_FACTOR, 0,
                    (uint8_t)(scan_secs & 0xFF), (uint8_t)(scan_secs >> 8)};
    if (!configOn()) return false;
    bool ok = command(0x0009, d, sizeof(d));
    configOff();   // prubeh radar posila az mimo konfiguracni rezim
    return ok;
  }

  // 0x0072 = trigger (sepnuti), 0x0076 = hold (udrzeni)
  bool writeThresholds(uint16_t cmd, const uint8_t vals[LD_GATES]) {
    uint8_t d[6 * LD_GATES];
    for (int g = 0; g < LD_GATES; g++) {
      uint8_t* e = &d[6 * g];
      e[0] = g; e[1] = 0; e[2] = vals[g]; e[3] = 0; e[4] = 0; e[5] = 0;
    }
    return command(cmd, d, sizeof(d));
  }

  // nejvzdalenejsi pouzita brana (obecny parametr 0x05), brany za ni radar nevyhodnocuje
  int readFarGate() {
    static const uint8_t d[2] = {0x05, 0x00};
    uint8_t out[4];
    if (!command(0x0071, d, 2, out, 4)) return LD_GATES - 1;
    return (out[0] >= 1 && out[0] < LD_GATES) ? out[0] : LD_GATES - 1;
  }

  // precte prahy sepnuti i udrzeni a nejvzdalenejsi branu
  bool readAll(uint8_t t[LD_GATES], uint8_t h[LD_GATES], int& far_gate) {
    if (!configOn()) return false;
    far_gate = readFarGate();
    bool ok = readThresholds(0x0073, t) && readThresholds(0x0077, h);
    configOff();
    return ok;
  }

  bool writeDefaults() {
    if (!configOn()) return false;
    bool ok = writeThresholds(0x0072, LD_DEF_TRIGGER) && writeThresholds(0x0076, LD_DEF_HOLD);
    configOff();
    return ok;
  }

  // zpracuje prichozi datove ramce (behem kalibrace)
  void poll() {
    uint8_t p[80];
    bool ack;
    int l;
    while ((l = rx(p, sizeof(p), ack)) >= 0) if (!ack) dataFrame(p, l);
  }
};
#endif

// ---------- upozorneni na slabou baterii ----------
#ifndef BATT_LOW_MV
  #define BATT_LOW_MV     3500   // "baterie slaba"
#endif
#ifndef BATT_CRIT_MV
  #define BATT_CRIT_MV    3350   // "baterie kriticka" (pod 3,3 V uz uzel po restartu nenabehne)
#endif
#define BATT_HYST_MV       100   // zruseni upozorneni az po vzrustu o 0,1 V nad prah
#define BATT_DEBOUNCE        3   // prah musi byt podkrocen 3x po sobe (mereni 1x za minutu)

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables)
  {
  }

protected:
  /* ========================== custom logic here ========================== */
  // Radar. Prikazy (velikost pismen nehraje roli):
  //   RADAR ON / RADAR OFF    -> zapne / vypne hlidani (zpravy o pohybu)
  //   SVETLO ON / SVETLO OFF  -> zapne / vypne rozsviceni svetla na LIGHT_PULSE_SECS pri pohybu
  //   SVETLO TEST             -> rozsviti svetlo na LIGHT_PULSE_SECS
  //   STATUS                  -> stav + napeti baterie + sila signalu posledniho paketu + doba behu
  //   WDTTEST / ALERTTEST     -> (jen USB) test watchdogu / zkusebni poplach do kanalu
  //
  // Dve cesty, jak prikaz poslat (stejne jako u zahradnich svetel):
  //  1) CLI (USB, nebo LoRa od prihlaseneho admina) - jen tomuto uzlu.
  //  2) Soukromy kanal MeshCore:
  //       "STATUS RADAR"  -> jen radary (zahradni svetla na to nereaguji)
  //       "STATUS"        -> vsechny uzly v kanalu vcetne radaru
  //       "RADAR ON", "SVETLO OFF" ...  -> vsechny radary; "RADAR ON 2" -> jen radar s cislem 2
  //     Radar odpovi do kanalu "<jmeno>: <stav>". Pri pohybu posle "<jmeno>: POHYB! ...".
  //  Klic kanalu se nastavi pres CLI:  CHAN <32 nebo 64 hex znaku>,  CHAN = stav,  CHAN OFF = smazat.
  //  Stav RADAR/SVETLO ON/OFF se uklada do pameti uzlu a po restartu zustava.

  bool radar_on = false;          // hlidani zapnuto
  bool light_on = false;          // rozsviceni pri pohybu zapnuto
  bool motion_prev = false;       // posledni stav OT2 radaru (1 = pritomnost)
  uint32_t ot2_edges = 0;         // nabezne hrany OT2 od startu, i pri RADAR OFF (pro test radaru)
  bool radar_seen = false;        // OT2 uz byl aspon jednou precten
  uint32_t radar_start = 0;       // millis() prvniho cteni OT2 (zacatek ustalovani)
  bool ready_warn_sent = false;   // zprava "pripraven za 60 s" uz odesla
  bool ready_sent = false;        // zprava "pripraven" uz odesla
  uint32_t motion_count = 0;      // pocet pohybu od zapnuti hlidani
  uint32_t motion_last = 0;       // millis() posledniho pohybu
  bool alarm_sent_once = false;
  uint32_t alarm_last = 0;        // millis() posledni zpravy o pohybu
  uint32_t alarm_pending = 0;     // pohyby od posledni odeslane zpravy

  // nastaveni pres CLI (uklada se do /radar_cfg): ustalovani po startu, pauza mezi zpravami o pohybu, korekce ADC
  uint16_t startup_secs = RADAR_STARTUP_SECS;
  uint16_t cooldown_secs = ALARM_COOLDOWN_SECS;
  uint16_t batkal = 1000;         // napeti baterie x batkal/1000 (BATKAL podle multimetru)

  // kalibrace (automaticke prahy radaru pres UART)
  enum { KAL_IDLE, KAL_WAIT, KAL_SCAN };
  uint8_t kal_state = KAL_IDLE;
  uint32_t kal_t0 = 0;            // millis() prikazu (KAL_WAIT) / zacatku skenu (KAL_SCAN)
  uint16_t kal_minutes = RADAR_KAL_MINUTES;
#ifdef PIN_RADAR_UART_RX
  LD2410S ld;
#endif

  mesh::GroupChannel light_chan;
  bool light_chan_ok = false;

  // Ochrana proti prehrani v kanalu: od kazdeho odesilatele (jmeno v zasifrovane zprave) jen novejsi
  // casove razitko. Zvlast pro kazdeho, aby nevadil rozdil hodin mezi telefony.
  #define CHAN_MAX_SENDERS  16
  struct ChanSender { uint32_t name_hash; uint32_t last_ts; };
  ChanSender chan_senders[CHAN_MAX_SENDERS];
  uint8_t chan_senders_n = 0, chan_senders_next = 0;

  static uint32_t nameHash(const char* s, size_t n) {   // FNV-1a
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= (uint8_t)s[i]; h *= 16777619u; }
    return h;
  }
  bool chanReplayOk(uint32_t name_hash, uint32_t ts) {
    for (int i = 0; i < chan_senders_n; i++) {
      if (chan_senders[i].name_hash == name_hash) {
        if (ts <= chan_senders[i].last_ts) return false;   // stejna nebo starsi zprava = prehrani
        chan_senders[i].last_ts = ts;
        return true;
      }
    }
    int idx;   // novy odesilatel
    if (chan_senders_n < CHAN_MAX_SENDERS) idx = chan_senders_n++;
    else { idx = chan_senders_next; chan_senders_next = (chan_senders_next + 1) % CHAN_MAX_SENDERS; }
    chan_senders[idx].name_hash = name_hash;
    chan_senders[idx].last_ts = ts;
    return true;
  }

  // Upozorneni na slabou baterii: 1 zprava do soukromeho kanalu (stejne jako u zahradnich svetel).
  bool low_active = false, crit_active = false;
  uint8_t low_cnt = 0, crit_cnt = 0;

  void battLevel(bool& active, uint8_t cnt, uint32_t mv, uint32_t thr_mv, const char* what) {
    bool now = cnt >= BATT_DEBOUNCE || (active && mv < thr_mv + BATT_HYST_MV);
    if (now && !active) {
      char body[48];
      snprintf(body, sizeof(body), "baterie %s %.2f V", what, mv / 1000.0f);
      active = sendChannelText(body, slotDelay());
    } else {
      active = now;
    }
  }

  void onSensorDataRead() override {   // vola SensorMesh 1x za minutu
#ifdef NRF52_POWER_MANAGEMENT
    if (board.isExternalPowered()) { low_cnt = crit_cnt = 0; return; }   // na USB napeti baterie nevypovida
#endif
    uint32_t mv = battMilliVolts();

    low_cnt  = (mv < BATT_LOW_MV)  ? (low_cnt  < 255 ? low_cnt  + 1 : 255) : 0;
    crit_cnt = (mv < BATT_CRIT_MV) ? (crit_cnt < 255 ? crit_cnt + 1 : 255) : 0;

    battLevel(low_active,  low_cnt,  mv, BATT_LOW_MV,  "slaba");
    battLevel(crit_active, crit_cnt, mv, BATT_CRIT_MV, "KRITICKA");
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    return 0;
  }

  // cislo radaru = cislice na konci jmena uzlu (napr. "dum-radar-2" -> 2), 0 = bez cisla
  int nodeNumber() {
    const char* name = getNodePrefs()->node_name;
    int len = strlen(name), i = len;
    while (i > 0 && isdigit((unsigned char)name[i - 1])) i--;
    return (i < len) ? atoi(&name[i]) : 0;
  }

  // napeti baterie: prumer 4 mereni (jedno mereni ADC kolisa), s korekci BATKAL
  uint32_t battRawMilliVolts() {
    uint32_t mv = 0;
    for (int i = 0; i < 4; i++) mv += board.getBattMilliVolts();
    return mv / 4;
  }
  uint32_t battMilliVolts() { return battRawMilliVolts() * batkal / 1000; }
  float battVolts() { return battMilliVolts() / 1000.0f; }

  // rozestup zprav podle cisla radaru (stejne jako zahradni svetla): radar 1 hned, 2 o 1,5 s pozdeji ...,
  // radar bez cisla jako 5., at se odpovedi a hlaseni vice radaru v kanalu nesrazi
  uint32_t slotDelay() {
    int me = nodeNumber();
    return (uint32_t)((me > 0 ? me - 1 : 4) % 8) * 1500UL;
  }

  void lightPulse() {
    lightTimerStart();                  // (znovu) spustit odpocet
    board.setGpio(board.getGpio() | 1);
  }

public:
  // ---------- radar: volat v kazdem pruchodu loop() s aktualnim stavem OUT ----------
  void radarLoop(bool motion) {
    if (!radar_seen) {   // prvni cteni po startu: pritomnost, ktera uz trva, neni novy pohyb
      radar_seen = true;
      radar_start = millis();
      motion_prev = motion;
      return;
    }
    bool rising = motion && !motion_prev;    // novy pohyb = nabezna hrana OT2
    motion_prev = motion;
    if (rising) ot2_edges++;
    uint32_t since_start = (uint32_t)(millis() - radar_start);
    uint32_t startup_ms = (uint32_t)startup_secs * 1000UL;
    bool settling = since_start < startup_ms;
    bool kal = kal_state != KAL_IDLE;   // behem kalibrace se pohyb (odchod z dosahu) nehlasi

    // po startu do kanalu: 60 s pred koncem ustalovani a pri jeho konci (jednou, at je jasne, kdy radar hlida)
    if (!ready_warn_sent && startup_secs > RADAR_READY_WARN_SECS &&
        since_start >= startup_ms - RADAR_READY_WARN_SECS * 1000UL) {
      char body[48];
      snprintf(body, sizeof(body), "radar pripraven za %d s (RADAR %s)", RADAR_READY_WARN_SECS, radar_on ? "ON" : "OFF");
      sendChannelText(body, slotDelay());
      ready_warn_sent = true;
    }
    if (!ready_sent && !settling) {
      char body[48];
      snprintf(body, sizeof(body), "radar pripraven (RADAR %s, SVETLO %s)", radar_on ? "ON" : "OFF", light_on ? "ON" : "OFF");
      sendChannelText(body, slotDelay());
      ready_warn_sent = ready_sent = true;
    }

    if (rising && radar_on && !settling && !kal) {
      motion_count++;
      motion_last = millis();
      alarm_pending++;
      if (light_on) lightPulse();
    }
    // zprava hned pri prvnim pohybu, dalsi az po pauze (PAUZA, vychozi ALARM_COOLDOWN_SECS) s poctem pohybu mezi tim
    if (radar_on && alarm_pending > 0 &&
        (!alarm_sent_once || (uint32_t)(millis() - alarm_last) >= (uint32_t)cooldown_secs * 1000UL)) {
      char body[64];
      if (alarm_pending > 1) {
        snprintf(body, sizeof(body), "POHYB! %ux za %us, bat=%.2fV", (unsigned)alarm_pending,
                 (unsigned)cooldown_secs, battVolts());
      } else {
        snprintf(body, sizeof(body), "POHYB! bat=%.2fV", battVolts());
      }
      sendChannelText(body, 0);   // bez kanalu se zprava zahodi (nehromadit stare poplachy)
      alarm_sent_once = true;
      alarm_last = millis();
      alarm_pending = 0;
    }
  }

  // ---------- kalibrace: volat v kazdem pruchodu loop() ----------
  void radarKalLoop() {
#ifdef PIN_RADAR_UART_RX
    if (kal_state == KAL_IDLE) return;
    uint32_t el = (uint32_t)(millis() - kal_t0);
    if (kal_state == KAL_WAIT) {
      if (el < RADAR_KAL_DELAY_SECS * 1000UL) return;
      if (!ld.startAuto(kal_minutes * 60)) {
        ld.end();
        kal_state = KAL_IDLE;
        sendChannelText("kalibrace CHYBA: radar neodpovida", slotDelay());
        return;
      }
      kal_state = KAL_SCAN;
      kal_t0 = millis();
      return;
    }
    ld.poll();   // prubeh skenu (jen pro STATUS; jednotku protokol jasne neuvadi, konec se ridi casem)
    if (el < kal_minutes * 60000UL + RADAR_KAL_GRACE_MS) return;
    uint8_t t[LD_GATES], h[LD_GATES];
    int far_gate = LD_GATES - 1;
    bool ok = ld.readAll(t, h, far_gate);
    ld.end();
    kal_state = KAL_IDLE;
    if (!ok) { sendChannelText("kalibrace CHYBA: prahy nejdou precist", slotDelay()); return; }
    char body[160];
    strcpy(body, "kalibrace: ");
    thresholdLine(&body[strlen(body)], sizeof(body) - strlen(body), "sepnuti", t, LD_DEF_TRIGGER, far_gate);
    sendChannelText(body, slotDelay());
    thresholdLine(body, sizeof(body), "udrzeni", h, LD_DEF_HOLD, far_gate);
    sendChannelText(body, slotDelay() + 1000);
#endif
  }

protected:
  bool execKalibrace(const char* cmd, char* reply) {
#ifdef PIN_RADAR_UART_RX
    if (strncmp(cmd, "radar kalibrace", 15) == 0) {
      if (kal_state != KAL_IDLE) { strcpy(reply, "KALIBRACE uz bezi"); return true; }
      int mins = RADAR_KAL_MINUTES;
      if (cmd[15] == ' ') {   // volitelne "20m"
        mins = atoi(&cmd[16]);
        if (mins < 2 || mins > 60) { strcpy(reply, "Err - doba 2m az 60m"); return true; }
      } else if (cmd[15] != 0) {
        return false;
      }
      ld.begin();
      bool ok = ld.configOn() && ld.configOff();   // radar odpovida?
      if (!ok) {
        ld.end();
        strcpy(reply, "Err - radar neodpovida (UART)");
        return true;
      }
      kal_minutes = mins;
      kal_state = KAL_WAIT;
      kal_t0 = millis();
      sprintf(reply, "KALIBRACE za %ds, sken %d min - odejdi z dosahu", RADAR_KAL_DELAY_SECS, mins);
      return true;
    }
    // RADAR PRAHY = sepnuti, RADAR PRAHY H = udrzeni, RADAR PRAHY VYCHOZI = zapsat vychozi prahy
    if (strncmp(cmd, "radar prahy", 11) == 0) {
      const char* arg = cmd[11] == ' ' ? &cmd[12] : &cmd[11];
      bool hold = strcmp(arg, "h") == 0;
      bool defaults = strcmp(arg, "vychozi") == 0 || strcmp(arg, "výchozí") == 0 || strcmp(arg, "reset") == 0;
      if (*arg && !hold && !defaults) return false;
      if (kal_state != KAL_IDLE) { strcpy(reply, "KALIBRACE bezi, prahy az po ni"); return true; }
      ld.begin();
      bool wrote = !defaults || ld.writeDefaults();
      uint8_t t[LD_GATES], h[LD_GATES];
      int far_gate = LD_GATES - 1;
      bool ok = wrote && ld.readAll(t, h, far_gate);
      ld.end();
      if (!ok) { strcpy(reply, "Err - radar neodpovida (UART)"); return true; }
      int p = defaults ? sprintf(reply, "VYCHOZI zapsany, ") : 0;
      thresholdLine(&reply[p], 150 - p, hold ? "udrzeni" : "sepnuti", hold ? h : t,
                    hold ? LD_DEF_HOLD : LD_DEF_TRIGGER, far_gate);
      return true;
    }
#else
    if (strncmp(cmd, "radar kalibrace", 15) == 0 || strncmp(cmd, "radar prahy", 11) == 0) {
      strcpy(reply, "Err - build bez UART radaru");
      return true;
    }
#endif
    return false;
  }

  // provede prikaz radaru; cmd uz je malymi pismeny, bez cisel uzlu
  bool execRadar(const char* cmd, char* reply) {
    if (execKalibrace(cmd, reply)) return true;
    if (strcmp(cmd, "radar on") == 0) {
      if (!radar_on) { motion_count = 0; alarm_pending = 0; alarm_sent_once = false; }
      radar_on = true;
      saveRadarConfig();
      strcpy(reply, "RADAR ON");
      return true;
    }
    if (strcmp(cmd, "radar off") == 0) {
      radar_on = false;
      alarm_pending = 0;
      saveRadarConfig();
      strcpy(reply, "RADAR OFF");
      return true;
    }
    if (strcmp(cmd, "svetlo on") == 0) {
      light_on = true;
      saveRadarConfig();
      strcpy(reply, "SVETLO ON");
      return true;
    }
    if (strcmp(cmd, "svetlo off") == 0) {
      light_on = false;
      board.setGpio(board.getGpio() & ~1u);
      saveRadarConfig();
      strcpy(reply, "SVETLO OFF");
      return true;
    }
    if (strcmp(cmd, "svetlo test") == 0) {
      lightPulse();
      sprintf(reply, "SVETLO TEST %ds", LIGHT_PULSE_SECS);
      return true;
    }
    if (strcmp(cmd, "status") == 0 || strcmp(cmd, "status radar") == 0) {
      char last[24];
      if (motion_count == 0) strcpy(last, "-");
      else {
        uint32_t m = (uint32_t)(millis() - motion_last) / 60000UL;
        if (m < 60) sprintf(last, "%um", (unsigned)m);
        else sprintf(last, "%uh%02um", (unsigned)(m / 60), (unsigned)(m % 60));
      }
      uint32_t up_min = (uint32_t)(uptime_ms / 60000ULL);
      // ot2 = okamzity stav vystupu radaru / pocet jeho nabeznych hran od startu (i pri RADAR OFF)
      sprintf(reply, "RADAR %s SVETLO %s ot2=%d/%u pohyb=%u (%s) bat=%.2fV rssi=%d snr=%.1f up=%ud%02uh%02um",
              radar_on ? "ON" : "OFF", light_on ? "ON" : "OFF",
              motion_prev ? 1 : 0, (unsigned)ot2_edges,
              (unsigned)motion_count, last,
              battVolts(),
              (int)radio_driver.getLastRSSI(),
              radio_driver.getLastSNR(),
              (unsigned)(up_min / 1440), (unsigned)((up_min / 60) % 24), (unsigned)(up_min % 60));
#ifdef PIN_RADAR_UART_RX
      if (kal_state == KAL_WAIT) strcat(reply, " kal=start");
      else if (kal_state == KAL_SCAN) {
        if (ld.progress >= 0) sprintf(&reply[strlen(reply)], " kal=%d", ld.progress);
        else strcat(reply, " kal=sken");
      }
#endif
      return true;
    }
    return false;
  }

  // ---------- ulozeni stavu a klice kanalu v pameti uzlu ----------
  #define RADAR_CFG_FILE   "/radar_cfg"
  #define WARMUP_MIN   10
  #define WARMUP_MAX   900
  #define PAUZA_MIN    10
  #define PAUZA_MAX    3600
  #define BATKAL_MIN   800    // korekce ADC 0,800 az 1,250
  #define BATKAL_MAX   1250
  #define RADAR_CHAN_FILE  "/radar_ch"

  void applyChannelSecret(const uint8_t* secret32) {
    static const uint8_t zeroes[16] = {0};
    memcpy(light_chan.secret, secret32, PUB_KEY_SIZE);
    // stejne jako aplikace/companion: 128bit klic (druha polovina nulova) nebo 256bit klic
    int klen = (memcmp(&secret32[16], zeroes, 16) == 0) ? 16 : 32;
    mesh::Utils::sha256(light_chan.hash, sizeof(light_chan.hash), light_chan.secret, klen);
    light_chan_ok = true;
    chan_senders_n = chan_senders_next = 0;
  }

public:
  void loadRadarState() {
#if defined(NRF52_PLATFORM)
    File f = InternalFS.open(RADAR_CHAN_FILE, FILE_O_READ);
    if (f) {
      uint8_t buf[PUB_KEY_SIZE];
      if (f.read(buf, sizeof(buf)) == (int)sizeof(buf)) applyChannelSecret(buf);
      f.close();
    }
    File c = InternalFS.open(RADAR_CFG_FILE, FILE_O_READ);
    if (c) {
      uint8_t b[8];
      int n = c.read(b, sizeof(b));
      if (n >= 2) { radar_on = b[0] != 0; light_on = b[1] != 0; }
      if (n >= 8) {   // novejsi soubor: nastaveni z CLI
        uint16_t w = b[2] | (b[3] << 8), pz = b[4] | (b[5] << 8), k = b[6] | (b[7] << 8);
        if (w >= WARMUP_MIN && w <= WARMUP_MAX) startup_secs = w;
        if (pz >= PAUZA_MIN && pz <= PAUZA_MAX) cooldown_secs = pz;
        if (k >= BATKAL_MIN && k <= BATKAL_MAX) batkal = k;
      }
      c.close();
    }
#endif
  }

protected:
  void saveRadarConfig() {
#if defined(NRF52_PLATFORM)
    InternalFS.remove(RADAR_CFG_FILE);
    File f = InternalFS.open(RADAR_CFG_FILE, FILE_O_WRITE);
    if (f) {
      uint8_t b[8] = { (uint8_t)(radar_on ? 1 : 0), (uint8_t)(light_on ? 1 : 0),
                       (uint8_t)(startup_secs & 0xFF), (uint8_t)(startup_secs >> 8),
                       (uint8_t)(cooldown_secs & 0xFF), (uint8_t)(cooldown_secs >> 8),
                       (uint8_t)(batkal & 0xFF), (uint8_t)(batkal >> 8) };
      f.write(b, sizeof(b));
      f.close();
    }
#endif
  }

  bool saveRadarChannel(const uint8_t* secret32) {
#if defined(NRF52_PLATFORM)
    InternalFS.remove(RADAR_CHAN_FILE);
    if (secret32 == NULL) return true;
    File f = InternalFS.open(RADAR_CHAN_FILE, FILE_O_WRITE);
    if (!f) return false;
    bool ok = f.write(secret32, PUB_KEY_SIZE) == PUB_KEY_SIZE;
    f.close();
    return ok;
#else
    return false;
#endif
  }

  // ---------- nastaveni jen pres CLI (USB nebo prihlaseny admin), kazdy radar zvlast ----------
  //   WARMUP [s]       ustalovani radaru po startu (10-900 s), bez cisla = zobrazit
  //   PAUZA [s]        nejkratsi odstup zprav o pohybu (10-3600 s)
  //   BATKAL [V|OFF]   korekce mereni baterie: zadat napeti namerene multimetrem, OFF = bez korekce
  //   NASTAVENI        vse najednou
  bool execSettings(const char* cmd, char* reply) {
    if (strcmp(cmd, "nastaveni") == 0) {
      sprintf(reply, "warmup=%us pauza=%us batkal=%u.%03u bat=%.2fV", startup_secs, cooldown_secs,
              batkal / 1000, batkal % 1000, battVolts());
      return true;
    }
    if (strcmp(cmd, "warmup") == 0) { sprintf(reply, "warmup=%us", startup_secs); return true; }
    if (strncmp(cmd, "warmup ", 7) == 0) {
      int v = atoi(&cmd[7]);
      if (v < WARMUP_MIN || v > WARMUP_MAX) { sprintf(reply, "Err - warmup %d az %d s", WARMUP_MIN, WARMUP_MAX); return true; }
      startup_secs = v;
      saveRadarConfig();
      sprintf(reply, "OK warmup=%us", startup_secs);
      return true;
    }
    if (strcmp(cmd, "pauza") == 0) { sprintf(reply, "pauza=%us", cooldown_secs); return true; }
    if (strncmp(cmd, "pauza ", 6) == 0) {
      int v = atoi(&cmd[6]);
      if (v < PAUZA_MIN || v > PAUZA_MAX) { sprintf(reply, "Err - pauza %d az %d s", PAUZA_MIN, PAUZA_MAX); return true; }
      cooldown_secs = v;
      saveRadarConfig();
      sprintf(reply, "OK pauza=%us", cooldown_secs);
      return true;
    }
    if (strcmp(cmd, "batkal") == 0 || strcmp(cmd, "batkal off") == 0 || strncmp(cmd, "batkal ", 7) == 0) {
      if (strcmp(cmd, "batkal off") == 0) {
        batkal = 1000;
        saveRadarConfig();
      } else if (cmd[6] == ' ') {
        char num[12];                // napeti z multimetru, napr. 4.12 (carka i tecka)
        strncpy(num, &cmd[7], sizeof(num) - 1);
        num[sizeof(num) - 1] = 0;
        char* comma = strchr(num, ',');
        if (comma) *comma = '.';
        float v = atof(num);
        uint32_t raw = battRawMilliVolts();
        uint32_t k = raw > 0 ? (uint32_t)(v * 1000.0f * 1000.0f / raw + 0.5f) : 0;
        if (v < 2.5f || v > 4.5f || k < BATKAL_MIN || k > BATKAL_MAX) {
          sprintf(reply, "Err - BATKAL 2.5 az 4.5 V a nejvys o 20 %% od mereni (%.2fV)", raw / 1000.0f);
          return true;
        }
        batkal = k;
        saveRadarConfig();
      }
      sprintf(reply, "batkal=%u.%03u bat=%.2fV", batkal / 1000, batkal % 1000, battVolts());
      return true;
    }
    return false;
  }

  // ---------- CLI: USB nebo LoRa od prihlaseneho admina ----------
  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    char cmd[80];
    int n = 0;
    while (command[n] && n < (int)sizeof(cmd) - 1) {
      cmd[n] = tolower((unsigned char)command[n]);
      n++;
    }
    cmd[n] = 0;
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\r' || cmd[n - 1] == '\n')) cmd[--n] = 0;

    // hodiny uzlu po startu nejdou spravne -> srovnat podle prikazu od admina (jen dopredu)
    if (sender_timestamp > getRTCClock()->getCurrentTime()) getRTCClock()->setCurrentTime(sender_timestamp);

    if (execRadar(cmd, reply)) return true;
    if (execSettings(cmd, reply)) return true;

    // test poplachu: posle zkusebni zpravu do kanalu (jen pres USB)
    if (sender_timestamp == 0 && strcmp(cmd, "alerttest") == 0) {
      char body[48];
      snprintf(body, sizeof(body), "test poplachu, bat %.2f V", battVolts());
      strcpy(reply, sendChannelText(body, 0) ? "OK - test poplachu odeslan do kanalu" : "Err - kanal neni nastaven");
      return true;
    }

    // test hlidaciho obvodu: zamerne zasekne firmware, do WDT_TIMEOUT_SECS se uzel sam restartuje (jen USB)
    if (sender_timestamp == 0 && strcmp(cmd, "wdttest") == 0) {
      board.setGpio(0);
      Serial.println("  -> WDT test: firmware zaseknut, cekam na restart...");
      Serial.flush();
      while (1) { }
    }

    if (strcmp(cmd, "chan") == 0) {
      if (light_chan_ok) sprintf(reply, "chan ON hash=%02X", light_chan.hash[0]);
      else strcpy(reply, "chan OFF");
      return true;
    }
    if (strcmp(cmd, "chan off") == 0) {
      light_chan_ok = false;
      saveRadarChannel(NULL);
      strcpy(reply, "OK chan OFF");
      return true;
    }
    if (memcmp(cmd, "chan ", 5) == 0) {
      const char* hex = &cmd[5];
      uint8_t secret[PUB_KEY_SIZE];
      memset(secret, 0, sizeof(secret));
      int hl = strlen(hex);
      if ((hl == 32 && mesh::Utils::fromHex(secret, 16, hex)) ||
          (hl == 64 && mesh::Utils::fromHex(secret, 32, hex))) {
        applyChannelSecret(secret);
        bool saved = saveRadarChannel(secret);
        sprintf(reply, "OK chan ON hash=%02X%s", light_chan.hash[0], saved ? "" : " (NEULOZENO!)");
      } else {
        strcpy(reply, "Err - klic musi mit 32 nebo 64 hex znaku");
      }
      return true;
    }
    return false;  // ostatni prikazy zpracuje standardni CLI MeshCore
  }

  // ---------- soukromy kanal ----------
  int searchChannelsByHash(const uint8_t* hash, mesh::GroupChannel channels[], int max_matches) override {
    if (light_chan_ok && max_matches > 0 && memcmp(hash, light_chan.hash, sizeof(light_chan.hash)) == 0) {
      channels[0] = light_chan;
      return 1;
    }
    return 0;
  }

  void onGroupDataRecv(mesh::Packet* packet, uint8_t type, const mesh::GroupChannel& channel, uint8_t* data, size_t len) override {
    if (type != PAYLOAD_TYPE_GRP_TXT || len < 6 || len > 5 + 120) return;
    if ((data[4] >> 2) != 0) return;            // jen obycejny text (TXT_TYPE_PLAIN)

    uint32_t ts;
    memcpy(&ts, data, 4);

    // hodiny uzlu srovnat podle kazde overene zpravy v kanalu (jen dopredu)
    if (ts > getRTCClock()->getCurrentTime()) getRTCClock()->setCurrentTime(ts);

    // text zpravy ve tvaru "<odesilatel>: <text>" -> vzit cast za ": ", prevest na mala pismena
    char text[128];
    size_t tl = len - 5;
    memcpy(text, &data[5], tl);
    text[tl] = 0;
    const char* sep = strstr(text, ": ");
    size_t sender_len = sep ? (size_t)(sep - text) : 0;
    const char* body = sep ? sep + 2 : text;
    char cmd[64];
    int n = 0;
    while (body[n] && n < (int)sizeof(cmd) - 1) { cmd[n] = tolower((unsigned char)body[n]); n++; }
    cmd[n] = 0;
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\r' || cmd[n - 1] == '\n')) cmd[--n] = 0;

    // "radar on 2" -> prikaz "radar on", cile "2";  "status radar" -> prikaz "status", cil "radar"
    char* save = NULL;
    char* word = strtok_r(cmd, " ", &save);
    if (word == NULL) return;
    char action[32];
    char* t = NULL;   // prvni token za prikazem (cile)
    if (strcmp(word, "radar") == 0 && (t = strtok_r(NULL, " ,", &save)) != NULL &&
        (strcmp(t, "kalibrace") == 0 || strcmp(t, "prahy") == 0)) {
      // "RADAR KALIBRACE [20m] [cile]", "RADAR PRAHY [cile]"
      snprintf(action, sizeof(action), "radar %s", t);
      t = strtok_r(NULL, " ,", &save);
      int tl = t ? strlen(t) : 0;
      bool kal_time = tl >= 2 && t[tl - 1] == 'm' && isdigit((unsigned char)t[0]) && strcmp(action, "radar kalibrace") == 0;
      bool prahy_arg = t && strcmp(action, "radar prahy") == 0 &&
                       (strcmp(t, "h") == 0 || strcmp(t, "vychozi") == 0 || strcmp(t, "výchozí") == 0 || strcmp(t, "reset") == 0);
      if (kal_time || prahy_arg) {
        snprintf(&action[strlen(action)], sizeof(action) - strlen(action), " %s", t);
        t = strtok_r(NULL, " ,", &save);
      }
    } else if (strcmp(word, "radar") == 0 || strcmp(word, "svetlo") == 0) {
      char* sub = (strcmp(word, "radar") == 0) ? t : strtok_r(NULL, " ,", &save);
      if (sub == NULL) return;
      if (strcmp(sub, "on") && strcmp(sub, "off") && strcmp(sub, "test")) return;
      if (strcmp(word, "radar") == 0 && strcmp(sub, "test") == 0) return;
      snprintf(action, sizeof(action), "%s %s", word, sub);
      t = strtok_r(NULL, " ,", &save);
    } else if (strcmp(word, "status") == 0) {
      strcpy(action, "status");
      t = strtok_r(NULL, " ,", &save);
    } else {
      return;   // neni pro nas (LON/LOFF svetel, odpovedi ostatnich uzlu, bezne zpravy)
    }

    int me = nodeNumber();
    bool for_me = true, for_all = true;
    if (t != NULL) {
      for_me = false; for_all = false;
      for (; t != NULL; t = strtok_r(NULL, " ,", &save)) {
        if (strcmp(t, "all") == 0) { for_me = true; for_all = true; }
        else if (strcmp(t, "radar") == 0 || (me > 0 && atoi(t) == me)) for_me = true;
      }
    }
    if (!for_me) return;

    // ochrana proti prehrani (zvlast pro kazdeho odesilatele)
    if (!chanReplayOk(nameHash(text, sender_len), ts)) return;

    char result[160];
    if (!execRadar(action, result)) return;

    // prikaz pro vsechny uzly -> odpovedet az po zahradnich svetlech, at se zpravy nesrazi
    // rozestup podle cisla radaru; prikaz pro vsechny uzly -> az po zahradnich svetlech
    sendChannelText(result, (for_all && strcmp(action, "status") == 0 ? REPLY_ALL_DELAY_MS : 600) + slotDelay());
  }

  // zprava do soukromeho kanalu ve tvaru "<jmeno>: <text>" (stejny format jako zprava z aplikace)
  bool sendChannelText(const char* text, uint32_t delay_ms) {
    if (!light_chan_ok) return false;
    uint8_t out[5 + 140];
    uint32_t now = getRTCClock()->getCurrentTimeUnique();
    memcpy(out, &now, 4);
    out[4] = 0;   // TXT_TYPE_PLAIN
    int ol = snprintf((char*)&out[5], sizeof(out) - 5, "%s: %s", getNodePrefs()->node_name, text);
    if (ol < 0) return false;
    if (ol > (int)sizeof(out) - 6) ol = sizeof(out) - 6;
    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, light_chan, out, 5 + ol);
    if (!pkt) return false;
    sendFlood(pkt, delay_ms + getRNG()->nextInt(0, 400), getNodePrefs()->path_hash_mode + 1);
    return true;
  }
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

void setup() {
#ifdef PIN_BOARD_LIGHT
  board.setGpio(0);   // svetlo VYPNUTE ihned po startu
#endif
  pinMode(PIN_RADAR_OUT, INPUT_PULLDOWN);   // OUT radaru (bez radaru = LOW = zadny pohyb)
  Serial.begin(115200);
  delay(1000);

  board.begin();
  wdtStart();          // hlidaci obvod: od ted musi hlavni smycka bezet

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.begin();
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Radar ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);
  the_mesh.loadRadarState();   // klic kanalu a stav RADAR/SVETLO ON/OFF

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif
}

void loop() {
  wdtFeed();
  uptimeLoop();
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
  the_mesh.radarLoop(digitalRead(PIN_RADAR_OUT) == HIGH);
  the_mesh.radarKalLoop();
  lightTimerLoop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.loop();
#endif
}
