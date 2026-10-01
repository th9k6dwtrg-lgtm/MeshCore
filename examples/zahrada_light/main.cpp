#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#ifndef LIGHT_ON_SECS
  #define LIGHT_ON_SECS  5      // jak dlouho svetlo po LON sviti (sekundy)
#endif
#define LIGHT_ON_MS  ((uint32_t)LIGHT_ON_SECS * 1000UL)

static uint32_t light_on_since = 0;   // millis() okamziku rozsviceni

// Hlida maximalni dobu svitu: plati pro LON i pro oficialni 'io s 1'.
static void lightTimerLoop() {
  static bool was_on = false;
  bool on = board.getGpio() & 1;
  if (on && !was_on && (millis() - light_on_since) > 100) {
    light_on_since = millis();          // rozsviceno jinak nez LON (napr. 'io s 1')
  }
  if (on && (millis() - light_on_since) >= LIGHT_ON_MS) {
    board.setGpio(board.getGpio() & ~1u);   // automaticke zhasnuti
    on = false;
  }
  was_on = on;
}

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
  //   STATUS  -> stav + napeti baterie + sila signalu posledniho prijateho paketu
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
  uint32_t light_chan_last_ts = 0;   // ochrana proti prehrani: prijmout jen novejsi casove razitko

  void onSensorDataRead() override {
    // zatim nic: zadne automaticke zpravy do site
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
      light_on_since = millis();          // (znovu) spustit odpocet
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
      sprintf(reply, "%s bat=%.2fV rssi=%d snr=%.1f",
              state,
              board.getBattMilliVolts() / 1000.0f,
              (int)radio_driver.getLastRSSI(),
              radio_driver.getLastSNR());
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
    light_chan_last_ts = 0;
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

    if (execLight(cmd, reply)) return true;

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
    const char* body = strstr(text, ": ");
    body = body ? body + 2 : text;
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

    // ochrana proti prehrani: kazdy dalsi prikaz musi mit novejsi cas nez predchozi
    if (ts <= light_chan_last_ts) return;
    light_chan_last_ts = ts;

    // hodiny uzlu po startu nejdou spravne -> srovnat podle overene zpravy z telefonu (jen dopredu)
    if (ts > getRTCClock()->getCurrentTime()) getRTCClock()->setCurrentTime(ts);

    char result[96];
    if (!execLight(word, result)) return;

    // odpoved do kanalu "<jmeno>: <vysledek>", kazde svetlo se zpozdenim podle sveho cisla (at se nesrazi)
    uint8_t out[5 + 140];
    uint32_t now = getRTCClock()->getCurrentTimeUnique();
    memcpy(out, &now, 4);
    out[4] = 0;   // TXT_TYPE_PLAIN
    int ol = snprintf((char*)&out[5], sizeof(out) - 5, "%s: %s", getNodePrefs()->node_name, result);
    if (ol < 0) return;
    if (ol > (int)sizeof(out) - 6) ol = sizeof(out) - 6;
    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, out, 5 + ol);
    if (pkt) {
      uint32_t delay = 600 + (uint32_t)((me > 0 ? me - 1 : 4) % 8) * 1500 + getRNG()->nextInt(0, 400);
      sendFlood(pkt, delay, getNodePrefs()->path_hash_mode + 1);
    }
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
