#include "SensorMesh.h"
#include "../zahrada_common/ZahradaNode.h"   // kanal, baterie, watchdog (spolecne s radarem)

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

class MyMesh : public ZahradaNode {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : ZahradaNode(board, radio, ms, rng, rtc, tables, "/zahrada_ch")
  {
  }

protected:
  uint32_t light_seq = 0;   // pocet rozsviceni po LON od zapnuti uzlu
  /* ========================== custom logic here ========================== */
  // Zahradni svetlo. Prikazy (velikost pismen nehraje roli):
  //   LON     -> svetlo sviti LIGHT_ON_SECS sekund (vychozi 5 s), pak samo zhasne; odpoved "ON 5s c.3 bat=3.95V"
  //              (c. = poradove cislo rozsviceni od zapnuti uzlu)
  //   STATUS  -> stav + napeti baterie + sila signalu posledniho prijateho paketu + doba behu od startu
  //   LIGHT PRIKAZY -> (jen v kanalu) tahak prikazu, kazdy na svem radku; odpovi svetlo 1 (LIGHT PRIKAZY 2 = svetlo 2)
  //   WDTTEST -> (jen USB) zamerne zasekne firmware -> overeni, ze se uzel sam restartuje
  //   ALERTTEST -> (jen USB) posle zkusebni upozorneni do kanalu
  //   CHAN, BATKAL -> klic kanalu a korekce baterie (spolecne s radarem, viz ZahradaNode.h)
  //
  // Dve cesty, jak prikaz poslat:
  //  1) CLI (USB, nebo LoRa od prihlaseneho admina) - jen tomuto uzlu.
  //  2) Soukromy kanal MeshCore (stejny tajny klic v aplikaci i v uzlu):
  //       "LON"      -> vsechna svetla
  //       "LON 2"    -> jen svetlo s cislem 2 (cislo = cislice na konci jmena uzlu)
  //       "LON 1 3"  -> svetla 1 a 3
  //     Kazdy uzel odpovi do kanalu "<jmeno>: <stav>", s rozestupem podle sveho cisla.

  // provede LON / STATUS; cmd uz je malymi pismeny a bez parametru (LOFF uz neni, svetlo zhasne samo)
  bool execLight(const char* cmd, char* reply) {
    if (strcmp(cmd, "lon") == 0) {
      lightTimerStart();                  // (znovu) spustit odpocet
      board.setGpio(board.getGpio() | 1);
      light_seq++;                        // poradove cislo rozsviceni od zapnuti uzlu (jen v RAM)
      sprintf(reply, "ON %ds c.%u bat=%.2fV", LIGHT_ON_SECS, (unsigned)light_seq, battVolts());
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
              battVolts(),
              (int)radio_driver.getLastRSSI(),
              radio_driver.getLastSNR(),
              (unsigned)(up_min / 1440), (unsigned)((up_min / 60) % 24), (unsigned)(up_min % 60));
      return true;
    }
    return false;
  }

  // ---------- CLI: USB nebo LoRa od prihlaseneho admina ----------
  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    char cmd[80];
    cliPrepare(sender_timestamp, command, cmd, sizeof(cmd));

    if (execLight(cmd, reply)) return true;

    // test upozorneni: posle zkusebni zpravu do kanalu stejnou cestou jako slaba baterie (jen pres USB)
    if (sender_timestamp == 0 && strcmp(cmd, "alerttest") == 0) {
      char body[48];
      snprintf(body, sizeof(body), "test upozorneni, bat %.2f V", battVolts());
      strcpy(reply, sendChannelText(body, REPLY_BASE_MS + slotDelay()) ? "OK - test upozorneni odeslan do kanalu" : "Err - kanal neni nastaven");
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

    // rozdelit: prvni slovo = prikaz, dalsi slova = cisla svetel (zadna = vsechna)
    char* save = NULL;
    char* word = strtok_r(cmd, " ", &save);
    if (word == NULL) return;
    char* t = strtok_r(NULL, " ,", &save);
    // "LIGHT PRIKAZY" = tahak prikazu; bez cisla odpovi jen svetlo 1 (nebo svetlo bez cisla), "LIGHT PRIKAZY 2" svetlo 2
    bool prikazy = strcmp(word, "light") == 0 && t != NULL && isPrikazy(t);
    if (prikazy) t = strtok_r(NULL, " ,", &save);
    else if (strcmp(word, "lon") && strcmp(word, "status")) return;  // neni pro nas (napr. odpovedi ostatnich svetel)

    int me = nodeNumber();
    bool for_me = !prikazy || me <= 1;
    if (t != NULL) {
      for_me = false;
      for (; t != NULL; t = strtok_r(NULL, " ,", &save)) {
        if (strcmp(t, "radar") == 0) return;   // "STATUS RADAR", "STATUS RADAR 2" je jen pro radary
        if (strcmp(t, "all") == 0 || (me > 0 && atoi(t) == me)) for_me = true;
      }
    }
    if (!for_me) return;

    // ochrana proti prehrani (zvlast pro kazdeho odesilatele)
    if (!chanReplayOk(sender, ts)) return;

    if (prikazy) {
      static const char* const lines[] = {
        "LON - svetlo na " ZSTR(LIGHT_ON_SECS) " s",
        "STATUS - stav vsech uzlu",
        "LIGHT PRIKAZY - tento seznam",
        "LON 2 = jen svetlo 2",
      };
      sendLines(lines, sizeof(lines) / sizeof(lines[0]), REPLY_BASE_MS + slotDelay());
      return;
    }

    char result[96];
    if (!execLight(word, result)) return;

    // kazde svetlo se zpozdenim podle sveho cisla, at se zpravy vice svetel nesrazi
    sendChannelText(result, REPLY_BASE_MS + slotDelay());
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
  the_mesh.loadNodeState();   // klic soukromeho kanalu a BATKAL (pokud byly nastaveny)

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
