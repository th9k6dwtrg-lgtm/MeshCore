#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

// Hlidaci cidlo s mmWave radarem HLK-LD2410S na XIAO nRF52840 + Wio-SX1262.
// Zapojeni: LD2410S 3V3/GND na 3V3/GND XIAO, LD2410S OT2 -> D7 (HIGH = pritomnost), OT1 (TX) a RX nezapojene,
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
  uint32_t motion_count = 0;      // pocet pohybu od zapnuti hlidani
  uint32_t motion_last = 0;       // millis() posledniho pohybu
  bool alarm_sent_once = false;
  uint32_t alarm_last = 0;        // millis() posledni zpravy o pohybu
  uint32_t alarm_pending = 0;     // pohyby od posledni odeslane zpravy

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
      active = sendChannelText(body, 0);
    } else {
      active = now;
    }
  }

  void onSensorDataRead() override {   // vola SensorMesh 1x za minutu
#ifdef NRF52_POWER_MANAGEMENT
    if (board.isExternalPowered()) { low_cnt = crit_cnt = 0; return; }   // na USB napeti baterie nevypovida
#endif
    uint32_t mv = 0;
    for (int i = 0; i < 4; i++) mv += board.getBattMilliVolts();
    mv /= 4;

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
    bool settling = (uint32_t)(millis() - radar_start) < RADAR_STARTUP_MS;

    if (rising && radar_on && !settling) {
      motion_count++;
      motion_last = millis();
      alarm_pending++;
      if (light_on) lightPulse();
    }
    // zprava hned pri prvnim pohybu, dalsi az po ALARM_COOLDOWN_SECS (s poctem pohybu mezi tim)
    if (radar_on && alarm_pending > 0 &&
        (!alarm_sent_once || (uint32_t)(millis() - alarm_last) >= ALARM_COOLDOWN_MS)) {
      char body[64];
      if (alarm_pending > 1) {
        snprintf(body, sizeof(body), "POHYB! %ux za %us, bat=%.2fV", (unsigned)alarm_pending,
                 (unsigned)ALARM_COOLDOWN_SECS, board.getBattMilliVolts() / 1000.0f);
      } else {
        snprintf(body, sizeof(body), "POHYB! bat=%.2fV", board.getBattMilliVolts() / 1000.0f);
      }
      sendChannelText(body, 0);   // bez kanalu se zprava zahodi (nehromadit stare poplachy)
      alarm_sent_once = true;
      alarm_last = millis();
      alarm_pending = 0;
    }
  }

protected:
  // provede prikaz radaru; cmd uz je malymi pismeny, bez cisel uzlu
  bool execRadar(const char* cmd, char* reply) {
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
              board.getBattMilliVolts() / 1000.0f,
              (int)radio_driver.getLastRSSI(),
              radio_driver.getLastSNR(),
              (unsigned)(up_min / 1440), (unsigned)((up_min / 60) % 24), (unsigned)(up_min % 60));
      return true;
    }
    return false;
  }

  // ---------- ulozeni stavu a klice kanalu v pameti uzlu ----------
  #define RADAR_CFG_FILE   "/radar_cfg"
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
      uint8_t b[2];
      if (c.read(b, 2) == 2) { radar_on = b[0] != 0; light_on = b[1] != 0; }
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
      uint8_t b[2] = { (uint8_t)(radar_on ? 1 : 0), (uint8_t)(light_on ? 1 : 0) };
      f.write(b, 2);
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

    // test poplachu: posle zkusebni zpravu do kanalu (jen pres USB)
    if (sender_timestamp == 0 && strcmp(cmd, "alerttest") == 0) {
      char body[48];
      snprintf(body, sizeof(body), "test poplachu, bat %.2f V", board.getBattMilliVolts() / 1000.0f);
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
    char action[24];
    if (strcmp(word, "radar") == 0 || strcmp(word, "svetlo") == 0) {
      char* sub = strtok_r(NULL, " ,", &save);
      if (sub == NULL) return;
      if (strcmp(sub, "on") && strcmp(sub, "off") && strcmp(sub, "test")) return;
      if (strcmp(word, "radar") == 0 && strcmp(sub, "test") == 0) return;
      snprintf(action, sizeof(action), "%s %s", word, sub);
    } else if (strcmp(word, "status") == 0) {
      strcpy(action, "status");
    } else {
      return;   // neni pro nas (LON/LOFF svetel, odpovedi ostatnich uzlu, bezne zpravy)
    }

    int me = nodeNumber();
    bool for_me = true, for_all = true;
    char* t = strtok_r(NULL, " ,", &save);
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

    char result[120];
    if (!execRadar(action, result)) return;

    // prikaz pro vsechny uzly -> odpovedet az po zahradnich svetlech, at se zpravy nesrazi
    sendChannelText(result, for_all && strcmp(action, "status") == 0 ? REPLY_ALL_DELAY_MS : 600);
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
