#include "SensorMesh.h"
#include "../zahrada_common/ZahradaNode.h"   // kanal, baterie, watchdog (spolecne se svetly)

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

// Hlida dobu svitu: plati pro pohyb, SVETLO RADAR i pro oficialni 'io s 1'.
static void lightTimerLoop() {
  if ((board.getGpio() & 1) == 0) { light_timer_armed = false; return; }
  if (!light_timer_armed) lightTimerStart();     // rozsviceno jinak (napr. 'io s 1')
  if ((uint32_t)(millis() - light_on_since) >= LIGHT_PULSE_MS) {
    board.setGpio(board.getGpio() & ~1u);        // automaticke zhasnuti
    light_timer_armed = false;
  }
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
// Po 100 % radar jeste ~10 s uklada prahy a na prikazy neodpovida; dotaz hned po 100 % ho jednou zasekl
// az do vypnuti napajeni (videno 8. 10. 2026). Proto se ceka na 100 % a jeste RADAR_KAL_GRACE_MS.
// Sken trva o neco dele nez zadana doba (120 s -> 124 s), bez hlaseni 100 % se ceka doba + 25 %.
#define RADAR_KAL_GRACE_MS      20000UL
#define RADAR_UART_QUIET_MS     5000UL    // po prikazech pres UART 5 s nehlasit pohyb (OT2 muze preblikout)
#define RADAR_KAL_RETRY_MS      10000UL   // kdyz radar po skenu neodpovi, zkusit znovu za 10 s
#define RADAR_KAL_TRIES         6         // ... nejvyse 6x
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
// obecne parametry: nejvzdalenejsi a nejblizsi brana, doba drzeni, frekvence hlaseni stavu a vzdalenosti, rychlost
#define LD_NPARAMS  6
static const uint8_t LD_PARAM_IDS[LD_NPARAMS] = {0x05, 0x0A, 0x06, 0x02, 0x0C, 0x0B};
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
  bool command(uint16_t cmd, const uint8_t* data, int dlen, uint8_t* out = NULL, int out_max = 0, int* out_len = NULL) {
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
        if (out_len) *out_len = l - 4;
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
    saved_ok = readParams(saved);
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

  // obecne parametry (far, near, doba drzeni, frekvence hlaseni, rychlost); kalibrace umi prepsat dobu drzeni
  bool readParams(uint32_t v[LD_NPARAMS]) {
    uint8_t d[2 * LD_NPARAMS], out[4 * LD_NPARAMS];
    for (int i = 0; i < LD_NPARAMS; i++) { d[2 * i] = LD_PARAM_IDS[i]; d[2 * i + 1] = 0; }
    int len = 0;
    if (!command(0x0071, d, sizeof(d), out, sizeof(out), &len) || len < (int)sizeof(out)) return false;
    for (int i = 0; i < LD_NPARAMS; i++)
      v[i] = out[4 * i] | (out[4 * i + 1] << 8) | ((uint32_t)out[4 * i + 2] << 16) | ((uint32_t)out[4 * i + 3] << 24);
    return true;
  }
  uint32_t saved[LD_NPARAMS];   // obecne parametry pred kalibraci
  bool saved_ok = false;

  // po kalibraci vrati obecne parametry, ktere radar zmenil; vraci pocet vracenych, -1 = chyba
  int restoreParams() {
    if (!saved_ok) return 0;
    uint32_t now[LD_NPARAMS];
    if (!configOn()) return -1;
    int n = readParams(now) ? 0 : -1;
    for (int i = 0; i < LD_NPARAMS && n >= 0; i++) {
      if (now[i] == saved[i]) continue;
      uint32_t v = saved[i];
      uint8_t d[6] = {LD_PARAM_IDS[i], 0, (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
      n = command(0x0070, d, sizeof(d)) ? n + 1 : -1;
    }
    configOff();
    return n;
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

class MyMesh : public ZahradaNode {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : ZahradaNode(board, radio, ms, rng, rtc, tables, "/radar_ch")
  {
  }

protected:
  /* ========================== custom logic here ========================== */
  // Radar. Prikazy (velikost pismen nehraje roli):
  //   RADAR ON / RADAR OFF    -> zapne / vypne hlidani (zpravy o pohybu)
  //   SVETLO ON / SVETLO OFF  -> zapne / vypne rozsviceni svetla na LIGHT_PULSE_SECS pri pohybu
  //   SVETLO RADAR            -> hned rozsviti svetlo na LIGHT_PULSE_SECS (obdoba LON u zahradnich svetel)
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
  //  Klic kanalu (CHAN) a korekce baterie (BATKAL) jsou spolecne se svetly, viz ZahradaNode.h.
  //  Po kazdem zapnuti zacina s RADAR ON a SVETLO OFF (RADAR_BOOT_ON, SVETLO_BOOT_ON), prikaz plati do vypnuti.

  bool radar_on = false;          // hlidani zapnuto
  bool light_on = false;          // rozsviceni pri pohybu zapnuto
  bool motion_prev = false;       // posledni stav OT2 radaru (1 = pritomnost)
  uint32_t ot2_edges = 0;         // nabezne hrany OT2 od startu, i pri RADAR OFF (pro test radaru)
  bool radar_seen = false;        // OT2 uz byl aspon jednou precten
  uint32_t radar_start = 0;       // millis() prvniho cteni OT2 (zacatek ustalovani)
  bool ready_warn_sent = false;   // zprava "pripraven za 60 s" uz odesla
  bool ready_sent = false;        // zprava "pripraven" uz odesla
  uint32_t motion_count = 0;      // pocet pohybu od zapnuti hlidani
  uint32_t alarm_seq = 0;         // poradove cislo zpravy POHYB! od zapnuti uzlu (jen v RAM)
  uint32_t alarm_first_ts = 0;    // UTC prvniho pohybu, ktery jeste neni ve zprave
  uint32_t motion_last = 0;       // millis() posledniho pohybu
  bool alarm_sent_once = false;
  uint32_t alarm_last = 0;        // millis() posledni zpravy o pohybu
  uint32_t alarm_pending = 0;     // pohyby od posledni odeslane zpravy

  // nastaveni pres CLI (uklada se do /radar_cfg): ustalovani po startu, pauza mezi zpravami o pohybu
  uint16_t startup_secs = RADAR_STARTUP_SECS;
  uint16_t cooldown_secs = ALARM_COOLDOWN_SECS;

  // kalibrace (automaticke prahy radaru pres UART)
  enum { KAL_IDLE, KAL_WAIT, KAL_SCAN };
  uint8_t kal_state = KAL_IDLE;
  uint32_t kal_t0 = 0;            // millis() prikazu (KAL_WAIT) / zacatku skenu (KAL_SCAN)
  uint16_t kal_minutes = RADAR_KAL_MINUTES;
  uint32_t kal_done = 0;          // millis() hlaseni 100 % (0 = zatim ne)
  uint8_t kal_tries = 0;          // neuspesne pokusy o cteni prahu po skenu
  uint32_t kal_retry_t = 0;       // millis() posledniho neuspesneho pokusu
  bool uart_quiet = false;        // chvili po UART komunikaci s radarem se OT2 nevyhodnocuje
  uint32_t uart_quiet_t = 0;
  void uartDone() { uart_quiet = true; uart_quiet_t = millis(); }
#ifdef PIN_RADAR_UART_RX
  LD2410S ld;
#endif

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
    // po prikazech pres UART (konfiguracni rezim radaru) muze OT2 kratce spadnout a znovu sepnout: neni to pohyb
    if (uart_quiet && (uint32_t)(millis() - uart_quiet_t) >= RADAR_UART_QUIET_MS) uart_quiet = false;
    if (uart_quiet) kal = true;

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
      if (alarm_pending == 0) alarm_first_ts = getRTCClock()->getCurrentTime();
      alarm_pending++;
      if (light_on) lightPulse();
    }
    // zprava hned pri prvnim pohybu, dalsi az po pauze (PAUZA, vychozi ALARM_COOLDOWN_SECS) s poctem pohybu mezi tim
    if (radar_on && alarm_pending > 0 &&
        (!alarm_sent_once || (uint32_t)(millis() - alarm_last) >= (uint32_t)cooldown_secs * 1000UL)) {
      // "POHYB! c.3 12:05:31 bat=3.95V", souhrn "POHYB! c.4 12:06:40 3x za 60s bat=3.95V" (cas = prvni pohyb)
      char body[72], when[12];
      localTimeStr(alarm_first_ts, when);
      alarm_seq++;
      if (alarm_pending > 1) {
        snprintf(body, sizeof(body), "POHYB! c.%u %s %ux za %us bat=%.2fV", (unsigned)alarm_seq, when,
                 (unsigned)alarm_pending, (unsigned)cooldown_secs, battVolts());
      } else {
        snprintf(body, sizeof(body), "POHYB! c.%u %s bat=%.2fV", (unsigned)alarm_seq, when, battVolts());
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
        ld.end(); uartDone();
        kal_state = KAL_IDLE;
        sendChannelText("kalibrace CHYBA: radar neodpovida", slotDelay());
        return;
      }
      kal_state = KAL_SCAN;
      kal_t0 = millis();
      kal_done = 0;
      kal_tries = 0;
      return;
    }
    ld.poll();   // prubeh skenu v % (pro STATUS a konec skenu)
    if (kal_done == 0 && ld.progress >= 100) { kal_done = millis(); if (kal_done == 0) kal_done = 1; }
    if (kal_done) {
      if ((uint32_t)(millis() - kal_done) < RADAR_KAL_GRACE_MS) return;
    } else {   // 100 % neprislo (rusene RX?): doba skenu + 25 %
      uint32_t scan_ms = kal_minutes * 60000UL;
      if (el < scan_ms + scan_ms / 4 + RADAR_KAL_GRACE_MS) return;
    }
    if (kal_tries > 0 && (uint32_t)(millis() - kal_retry_t) < RADAR_KAL_RETRY_MS) return;
    uint8_t t[LD_GATES], h[LD_GATES];
    int far_gate = LD_GATES - 1;
    bool ok = ld.readAll(t, h, far_gate);
    if (!ok && ++kal_tries < RADAR_KAL_TRIES) { kal_retry_t = millis(); return; }   // dalsi pokus za 10 s
    int restored = ok ? ld.restoreParams() : 0;
    ld.end(); uartDone();
    kal_state = KAL_IDLE;
    if (!ok) { sendChannelText("kalibrace CHYBA: radar neodpovida, vypni a zapni uzel", slotDelay()); return; }
    char body[160];
    strcpy(body, "kalibrace: ");
    thresholdLine(&body[strlen(body)], sizeof(body) - strlen(body), "sepnuti", t, LD_DEF_TRIGGER, far_gate);
    sendChannelText(body, slotDelay());
    thresholdLine(body, sizeof(body), "udrzeni", h, LD_DEF_HOLD, far_gate);
    sendChannelText(body, slotDelay() + 1000);
    if (restored > 0) sendChannelText("kalibrace: radar zmenil sve nastaveni, vraceno zpet", slotDelay() + 2000);
    else if (restored < 0) sendChannelText("kalibrace: nastaveni radaru nejde overit", slotDelay() + 2000);
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
        ld.end(); uartDone();
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
      ld.end(); uartDone();
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
    if (strcmp(cmd, "svetlo radar") == 0) {
      lightPulse();
      sprintf(reply, "SVETLO RADAR %ds", LIGHT_PULSE_SECS);
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

  // ---------- ulozeni stavu v pameti uzlu ----------
  #define RADAR_CFG_FILE   "/radar_cfg"
  #define WARMUP_MIN   10
  #define WARMUP_MAX   900
#ifndef RADAR_BOOT_ON
  #define RADAR_BOOT_ON   true    // stav po zapnuti: RADAR ON
#endif
#ifndef SVETLO_BOOT_ON
  #define SVETLO_BOOT_ON  false   // stav po zapnuti: SVETLO OFF
#endif
  #define PAUZA_MIN    10
  #define PAUZA_MAX    3600

public:
  void loadRadarState() {
    loadNodeState();   // klic kanalu a BATKAL
    // po kazdem zapnuti RADAR ON a SVETLO OFF, ulozeny stav RADAR/SVETLO ON/OFF se nepouziva
    radar_on = RADAR_BOOT_ON;
    light_on = SVETLO_BOOT_ON;
#if defined(NRF52_PLATFORM)
    File c = InternalFS.open(RADAR_CFG_FILE, FILE_O_READ);
    if (c) {
      uint8_t b[6];
      int n = c.read(b, sizeof(b));
      if (n >= 6) {   // novejsi soubor: nastaveni z CLI
        uint16_t w = b[2] | (b[3] << 8), pz = b[4] | (b[5] << 8);
        if (w >= WARMUP_MIN && w <= WARMUP_MAX) startup_secs = w;
        if (pz >= PAUZA_MIN && pz <= PAUZA_MAX) cooldown_secs = pz;
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
      uint8_t b[6] = { (uint8_t)(radar_on ? 1 : 0), (uint8_t)(light_on ? 1 : 0),
                       (uint8_t)(startup_secs & 0xFF), (uint8_t)(startup_secs >> 8),
                       (uint8_t)(cooldown_secs & 0xFF), (uint8_t)(cooldown_secs >> 8) };
      f.write(b, sizeof(b));
      f.close();
    }
#endif
  }

  // ---------- nastaveni jen pres CLI (USB nebo prihlaseny admin), kazdy radar zvlast ----------
  //   WARMUP [s]       ustalovani radaru po startu (10-900 s), bez cisla = zobrazit
  //   PAUZA [s]        nejkratsi odstup zprav o pohybu (10-3600 s)
  //   NASTAVENI        vse najednou (vcetne BATKAL, ktery je spolecny se svetly)
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
    return false;
  }

  // ---------- CLI: USB nebo LoRa od prihlaseneho admina ----------
  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    char cmd[80];
    cliPrepare(sender_timestamp, command, cmd, sizeof(cmd));

    if (execRadar(cmd, reply)) return true;
    if (execSettings(cmd, reply)) return true;

    // test poplachu: posle zkusebni zpravu do kanalu (jen pres USB)
    if (sender_timestamp == 0 && strcmp(cmd, "alerttest") == 0) {
      char body[48];
      snprintf(body, sizeof(body), "test poplachu, bat %.2f V", battVolts());
      strcpy(reply, sendChannelText(body, 0) ? "OK - test poplachu odeslan do kanalu" : "Err - kanal neni nastaven");
      return true;
    }

    if (execCommon(sender_timestamp, cmd, reply)) return true;   // CHAN, BATKAL, WDTTEST
    return false;  // ostatni prikazy zpracuje standardni CLI MeshCore
  }

  // ---------- soukromy kanal ----------
  void onGroupDataRecv(mesh::Packet* packet, uint8_t type, const mesh::GroupChannel& channel, uint8_t* data, size_t len) override {
    char cmd[64];
    uint32_t ts, sender;
    if (!chanCommand(type, data, len, cmd, sizeof(cmd), ts, sender)) return;

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
      if (strcmp(sub, "on") && strcmp(sub, "off") && strcmp(sub, "radar")) return;
      if (strcmp(word, "radar") == 0 && strcmp(sub, "radar") == 0) return;
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
    if (!chanReplayOk(sender, ts)) return;

    char result[160];
    if (!execRadar(action, result)) return;

    // rozestup podle cisla radaru; prikaz pro vsechny uzly -> az po zahradnich svetlech, at se zpravy nesrazi
    sendChannelText(result, (for_all && strcmp(action, "status") == 0 ? REPLY_ALL_DELAY_MS : REPLY_BASE_MS) + slotDelay());
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
  the_mesh.loadRadarState();   // klic kanalu, BATKAL a stav RADAR/SVETLO ON/OFF

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
