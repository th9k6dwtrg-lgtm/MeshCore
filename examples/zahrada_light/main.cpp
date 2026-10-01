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
  // Zahradni svetlo: prikazy prijate pres CLI (USB, nebo LoRa od prihlaseneho admina).
  //   LON     -> svetlo sviti LIGHT_ON_SECS sekund (vychozi 5 s), pak samo zhasne
  //              (LON pri rozsvicenem svetle odpocet znovu spusti od zacatku)
  //   LOFF    -> svetlo hned zhasne
  //   STATUS  -> stav + napeti baterie + sila signalu posledniho prijateho paketu
  // Velikost pismen nehraje roli. Odpoved v 'reply' posle MeshCore automaticky zpet = potvrzeni.

  void onSensorDataRead() override {
    // zatim nic: zadne automaticke zpravy do site
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    return 0;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    // prevod na mala pismena a oriznuti mezer na konci (do pomocneho bufferu)
    char cmd[24];
    int n = 0;
    while (command[n] && n < (int)sizeof(cmd) - 1) {
      cmd[n] = tolower((unsigned char)command[n]);
      n++;
    }
    cmd[n] = 0;
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\r' || cmd[n - 1] == '\n')) cmd[--n] = 0;

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
    return false;  // ostatni prikazy zpracuje standardni CLI MeshCore
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
