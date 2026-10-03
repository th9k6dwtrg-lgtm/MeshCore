#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#ifndef LIGHT_ON_SECS
  #define LIGHT_ON_SECS  5      // jak dlouho svetlo po LON sviti (sekundy)
#endif
#define LIGHT_ON_MS  ((uint32_t)LIGHT_ON_SECS * 1000UL)

static uint32_t light_on_since = 0;      // millis() okamziku rozsviceni
static bool light_timer_armed = false;   // odpocet do zhasnuti bezi

static void lightTimerStart() {
  light_on_since = millis();
  light_timer_armed = true;
}

// Hlida maximalni dobu svitu: plati pro LON i pro oficialni 'io s 1'.
static void lightTimerLoop() {
  if ((board.getGpio() & 1) == 0) { light_timer_armed = false; return; }
  if (!light_timer_armed) lightTimerStart();     // rozsviceno jinak nez LON (napr. 'io s 1')
  if ((uint32_t)(millis() - light_on_since) >= LIGHT_ON_MS) {
    board.setGpio(board.getGpio() & ~1u);        // automaticke zhasnuti
    light_timer_armed = false;
  }
}

// ---------- hlidaci obvod (watchdog) nRF52 ----------
// Kdyz se firmware zasekne a hlavni smycka ho neobnovi do WDT_TIMEOUT_SECS, cip se sam restartuje.
// Po restartu je svetlo vzdy zhasnute (D6 = LOW hned po startu).
// Pozn.: jednou spusteny WDT nejde zastavit a bezi i po softwarovem resetu (napr. do zavadece).
// Zavadec Adafruit ho neobnovuje, proto dlouha doba (prijde-li reset v zavadeci, staci zopakovat).
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
  // Zahradni svetlo. Prikazy (velikost pismen nehraje roli):
  //   LON     -> svetlo sviti LIGHT_ON_SECS sekund (vychozi 5 s), pak samo zhasne
  //   LOFF    -> svetlo hned zhasne
  //   STATUS  -> stav + napeti baterie + sila signalu posledniho prijateho paketu + doba behu od startu
  //   WDTTEST -> (jen USB) zamerne zasekne firmware -> overeni, ze se uzel sam restartuje
  //   ALERTTEST -> (jen USB) posle zkusebni upozorneni adminum (overeni doruceni)
  //
  // Dve cesty, jak prikaz poslat:
  //  1) CLI (USB, nebo LoRa od prihlaseneho admina) - jen tomuto uzlu.
  //  2) Soukromy kanal MeshCore (stejny tajny klic v aplikaci i v uzlu):
  //       "LON"      -> vsechna svetla
  //       "LON 2"    -> jen svetlo s cislem 2 (cislo = cislice na konci jmena uzlu)
  //       "LON 1 3"  -> svetla 1 a 3
  //     Kazdy uzel odpovi do kanalu "<jmeno>: <stav>".
  //  Klic kanalu se nastavi pres CLI:  CHAN <32 nebo 64 hex znaku>,  CHAN = stav,  CHAN OFF = smazat.

  mesh::GroupChannel light_chan;
  bool light_chan_ok = false;

  // Ochrana proti prehrani v kanalu: od kazdeho odesilatele (jmeno v zasifrovane zprave) jen novejsi
  // casove razitko. Zvlast pro kazdeho, aby nevadil rozdil hodin mezi telefony (napr. 2 lide v kanalu).
  #define CHAN_MAX_SENDERS  8
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

  // Upozorneni na slabou baterii:
  //  - oficialni mechanismus SensorMesh (alertIf): primo adminum v ACL, s potvrzenim a opakovanim
  //  - navic 1 zprava do soukromeho kanalu (vidi vsichni v kanalu; bez potvrzeni doruceni)
  // Jedna zprava pri prekroceni prahu, dalsi az po nabiti nad prah + 0,1 V a novem poklesu.
  Trigger batt_low, batt_crit, test_alert;
  uint8_t low_cnt = 0, crit_cnt = 0;

  void onSensorDataRead() override {   // vola SensorMesh 1x za minutu
#ifdef NRF52_POWER_MANAGEMENT
    if (board.isExternalPowered()) { low_cnt = crit_cnt = 0; return; }   // na USB napeti baterie nevypovida
#endif
    uint32_t mv = 0;
    for (int i = 0; i < 4; i++) mv += board.getBattMilliVolts();
    mv /= 4;

    low_cnt  = (mv < BATT_LOW_MV)  ? (low_cnt  < 255 ? low_cnt  + 1 : 255) : 0;
    crit_cnt = (mv < BATT_CRIT_MV) ? (crit_cnt < 255 ? crit_cnt + 1 : 255) : 0;

    char body[48], text[96];
    bool was_low = batt_low.isTriggered(), was_crit = batt_crit.isTriggered();

    snprintf(body, sizeof(body), "baterie slaba %.2f V", mv / 1000.0f);
    snprintf(text, sizeof(text), "%s: %s", getNodePrefs()->node_name, body);
    alertIf(low_cnt >= BATT_DEBOUNCE || (batt_low.isTriggered() && mv < BATT_LOW_MV + BATT_HYST_MV),
            batt_low, HIGH_PRI_ALERT, text);
    if (!was_low && batt_low.isTriggered()) sendChannelText(body);     // 1x do kanalu pri vzniku

    snprintf(body, sizeof(body), "baterie KRITICKA %.2f V", mv / 1000.0f);
    snprintf(text, sizeof(text), "%s: %s", getNodePrefs()->node_name, body);
    alertIf(crit_cnt >= BATT_DEBOUNCE || (batt_crit.isTriggered() && mv < BATT_CRIT_MV + BATT_HYST_MV),
            batt_crit, HIGH_PRI_ALERT, text);
    if (!was_crit && batt_crit.isTriggered()) sendChannelText(body);
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    return 0;
  }

  // cislo svetla = cislice na konci jmena uzlu (napr. "zahrada-svetlo-2" -> 2), 0 = bez cisla
  int lightNumber() {
    const char* name = getNodePrefs()->node_name;
    int len = strlen(name), i = len;
    while (i > 0 && isdigit((unsigned char)name[i - 1])) i--;
    return (i < len) ? atoi(&name[i]) : 0;
  }

  // provede LON / LOFF / STATUS; cmd uz je malymi pismeny a bez parametru
  bool execLight(const char* cmd, char* reply) {
    if (strcmp(cmd, "lon") == 0) {
      lightTimerStart();                  // (znovu) spustit odpocet
      board.setGpio(board.getGpio() | 1);
      sprintf(reply, "ON %ds", LIGHT_ON_SECS);
      return true;
    }
    if (strcmp(cmd, "loff") == 0) {
      board.setGpio(board.getGpio() & ~1u);
      strcpy(reply, "OFF");
      return true;
    }
    if (strcmp(cmd, "status") == 0) {
      char state[16];
      if (board.getGpio() & 1) {
        uint32_t elapsed = millis() - light_on_since;
        uint32_t left = elapsed < LIGHT_ON_MS ? (LIGHT_ON_MS - elapsed + 999) / 1000 : 0;
        sprintf(state, "ON %us", (unsigned)left);
      } else {
        strcpy(state, "OFF");
      }
      uint32_t up_min = (uint32_t)(uptime_ms / 60000ULL);
      sprintf(reply, "%s bat=%.2fV rssi=%d snr=%.1f up=%ud%02uh%02um",
              state,
              board.getBattMilliVolts() / 1000.0f,
              (int)radio_driver.getLastRSSI(),
              radio_driver.getLastSNR(),
              (unsigned)(up_min / 1440), (unsigned)((up_min / 60) % 24), (unsigned)(up_min % 60));
      return true;
    }
    return false;
  }

  // ---------- klic soukromeho kanalu: ulozeni v pameti uzlu ----------
  #define LIGHT_CHAN_FILE  "/zahrada_ch"

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
  void loadLightChannel() {
#if defined(NRF52_PLATFORM)
    File f = InternalFS.open(LIGHT_CHAN_FILE, FILE_O_READ);
    if (f) {
      uint8_t buf[PUB_KEY_SIZE];
      if (f.read(buf, sizeof(buf)) == (int)sizeof(buf)) applyChannelSecret(buf);
      f.close();
    }
#endif
  }

protected:
  bool saveLightChannel(const uint8_t* secret32) {
#if defined(NRF52_PLATFORM)
    InternalFS.remove(LIGHT_CHAN_FILE);
    if (secret32 == NULL) return true;
    File f = InternalFS.open(LIGHT_CHAN_FILE, FILE_O_WRITE);
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
    // prevod na mala pismena a oriznuti mezer na konci (do pomocneho bufferu)
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

    if (execLight(cmd, reply)) return true;

    // test doruceni upozorneni: posle zkusebni upozorneni stejnou cestou jako slaba baterie (jen pres USB)
    if (sender_timestamp == 0 && strcmp(cmd, "alerttest") == 0) {
      char body[48], text[96];
      snprintf(body, sizeof(body), "test upozorneni, bat %.2f V", board.getBattMilliVolts() / 1000.0f);
      snprintf(text, sizeof(text), "%s: %s", getNodePrefs()->node_name, body);
      alertIf(false, test_alert, HIGH_PRI_ALERT, text);   // zrusit predchozi test
      alertIf(true,  test_alert, HIGH_PRI_ALERT, text);
      bool ch = sendChannelText(body);
      sprintf(reply, "OK - test: adminum v ACL%s", ch ? " + do kanalu" : " (kanal neni nastaven)");
      return true;
    }

    // test hlidaciho obvodu: zamerne zasekne firmware, do WDT_TIMEOUT_SECS se uzel sam restartuje
    // (jen pres USB, ne na dalku)
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
      saveLightChannel(NULL);
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
        bool saved = saveLightChannel(secret);
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

    // rozdelit: prvni slovo = prikaz, dalsi slova = cisla svetel (zadna = vsechna)
    char* save = NULL;
    char* word = strtok_r(cmd, " ", &save);
    if (word == NULL) return;
    if (strcmp(word, "lon") && strcmp(word, "loff") && strcmp(word, "status")) return;  // neni pro nas (napr. odpovedi ostatnich svetel)

    int me = lightNumber();
    bool for_me = true;
    char* t = strtok_r(NULL, " ,", &save);
    if (t != NULL) {
      for_me = false;
      for (; t != NULL; t = strtok_r(NULL, " ,", &save)) {
        if (strcmp(t, "all") == 0 || (me > 0 && atoi(t) == me)) for_me = true;
      }
    }
    if (!for_me) return;

    // ochrana proti prehrani (zvlast pro kazdeho odesilatele)
    if (!chanReplayOk(nameHash(text, sender_len), ts)) return;

    // hodiny uzlu po startu nejdou spravne -> srovnat podle overene zpravy z telefonu (jen dopredu)
    if (ts > getRTCClock()->getCurrentTime()) getRTCClock()->setCurrentTime(ts);

    char result[96];
    if (!execLight(word, result)) return;

    sendChannelText(result);
  }

  // zprava do soukromeho kanalu ve tvaru "<jmeno>: <text>" (stejny format jako zprava z aplikace);
  // kazde svetlo se zpozdenim podle sveho cisla, at se zpravy vice svetel nesrazi
  bool sendChannelText(const char* text) {
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
    int me = lightNumber();
    uint32_t delay = 600 + (uint32_t)((me > 0 ? me - 1 : 4) % 8) * 1500 + getRNG()->nextInt(0, 400);
    sendFlood(pkt, delay, getNodePrefs()->path_hash_mode + 1);
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
  board.setGpio(0);   // svetlo VYPNUTE ihned po startu, jeste pred cekanim na seriovou linku
#endif
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

  Serial.print("Zahrada svetlo ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);
  the_mesh.loadLightChannel();   // klic soukromeho kanalu (pokud byl nastaven)

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
