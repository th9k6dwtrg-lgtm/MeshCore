// Spolecny zaklad uzlu na zahrade a v dome: zahradni svetlo (examples/zahrada_light) a radar (examples/radar_alarm).
// Soukromy kanal MeshCore, hlidani baterie s korekci BATKAL, rozestupy zprav podle cisla uzlu, watchdog a doba behu.
// Kazdy uzel ma dal vlastni firmware (.uf2); tady je jen to, co maji oba stejne, at se oprava dela jednou.
// Vklada se v main.cpp hned za "SensorMesh.h". Logicke testy (zahrada_test, radar_test) ho prekladaji spolu s main.cpp.
#pragma once

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

// ---------- mistni cas (CET/CEST) pro zpravy ----------
// Hodiny uzlu jsou UTC a srovnaji se podle casu v prijatych zpravach. Letni cas plati
// od posledni nedele v breznu 01:00 UTC do posledni nedele v rijnu 01:00 UTC (pravidlo EU).
static int32_t daysFromCivil(int y, int m, int d) {   // dny od 1. 1. 1970
  y -= m <= 2;
  int era = y / 400;
  int yoe = y - era * 400;
  int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}
static uint32_t lastSundayUtc(int y, int m) {   // posledni nedele mesice (brezen, rijen) v 01:00 UTC
  int32_t ld = daysFromCivil(y, m, 31);
  return (uint32_t)(ld - (ld + 4) % 7) * 86400UL + 3600UL;
}
// "HH:MM:SS" mistniho casu, "cas?" dokud hodiny nejsou srovnane
static void localTimeStr(uint32_t utc, char* out) {
  if (utc < 1704067200UL) { strcpy(out, "cas?"); return; }   // pred 1. 1. 2024 = hodiny nesrovnane
  int y = 1970 + (int)(utc / 31556952UL);                     // rok (pripadne o 1 vic, opravi se nize)
  if ((uint32_t)daysFromCivil(y, 1, 1) * 86400UL > utc) y--;
  bool dst = utc >= lastSundayUtc(y, 3) && utc < lastSundayUtc(y, 10);
  uint32_t t = (utc + (dst ? 7200UL : 3600UL)) % 86400UL;
  sprintf(out, "%02u:%02u:%02u", (unsigned)(t / 3600), (unsigned)(t / 60 % 60), (unsigned)(t % 60));
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
#define BATKAL_MIN         800   // korekce mereni baterie 0,800 az 1,250
#define BATKAL_MAX        1250
#define BATKAL_FILE  "/batkal"

#define SLOT_MS           1500   // rozestup zprav sousednich cisel uzlu
#define REPLY_BASE_MS      600   // zakladni zpozdeni odpovedi na prikaz

class ZahradaNode : public SensorMesh {
public:
  ZahradaNode(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng,
              mesh::RTCClock& rtc, mesh::MeshTables& tables, const char* chan_file)
     : SensorMesh(board, radio, ms, rng, rtc, tables), chan_file(chan_file)
  {
  }

  // klic kanalu a BATKAL z pameti uzlu (volat po the_mesh.begin())
  void loadNodeState() {
#if defined(NRF52_PLATFORM)
    File f = InternalFS.open(chan_file, FILE_O_READ);
    if (f) {
      uint8_t buf[PUB_KEY_SIZE];
      if (f.read(buf, sizeof(buf)) == (int)sizeof(buf)) applyChannelSecret(buf);
      f.close();
    }
    File k = InternalFS.open(BATKAL_FILE, FILE_O_READ);
    if (k) {
      uint8_t b[2];
      if (k.read(b, 2) == 2) {
        uint16_t v = b[0] | (b[1] << 8);
        if (v >= BATKAL_MIN && v <= BATKAL_MAX) batkal = v;
      }
      k.close();
    }
#endif
  }

protected:
  //  Klic kanalu se nastavi pres CLI:  CHAN <32 nebo 64 hex znaku>,  CHAN = stav,  CHAN OFF = smazat.
  //  Korekce baterie:  BATKAL <napeti z multimetru>,  BATKAL = stav,  BATKAL OFF = bez korekce.
  const char* chan_file;          // soubor s klicem kanalu (kazdy typ uzlu svuj)
  mesh::GroupChannel light_chan;
  bool light_chan_ok = false;
  uint16_t batkal = 1000;         // napeti baterie x batkal/1000 (BATKAL podle multimetru)

  // Ochrana proti prehrani v kanalu: od kazdeho odesilatele (jmeno v zasifrovane zprave) jen novejsi
  // casove razitko. Zvlast pro kazdeho, aby nevadil rozdil hodin mezi telefony (napr. 2 lide v kanalu).
  #define CHAN_MAX_SENDERS  16   // az 16 ruznych lidi (jmen) v kanalu
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

  // cislo uzlu = cislice na konci jmena (napr. "LIGHT 2" nebo "RADAR 2" -> 2), 0 = bez cisla
  int nodeNumber() {
    const char* name = getNodePrefs()->node_name;
    int len = strlen(name), i = len;
    while (i > 0 && isdigit((unsigned char)name[i - 1])) i--;
    return (i < len) ? atoi(&name[i]) : 0;
  }

  // rozestup zprav podle cisla uzlu: 1 hned, 2 o 1,5 s pozdeji ..., bez cisla jako 5. (8 oken),
  // at se odpovedi vice svetel nebo radaru v kanalu nesrazi
  uint32_t slotDelay() {
    int me = nodeNumber();
    return (uint32_t)((me > 0 ? me - 1 : 4) % 8) * SLOT_MS;
  }

  // napeti baterie: prumer 4 mereni (jedno mereni ADC kolisa), s korekci BATKAL
  uint32_t battRawMilliVolts() {
    uint32_t mv = 0;
    for (int i = 0; i < 4; i++) mv += board.getBattMilliVolts();
    return mv / 4;
  }
  uint32_t battMilliVolts() { return battRawMilliVolts() * batkal / 1000; }
  float battVolts() { return battMilliVolts() / 1000.0f; }

  // Upozorneni na slabou baterii: 1 zprava do soukromeho kanalu (vidi vsichni clenove kanalu).
  // (Prime zpravy adminum aplikace u kontaktu typu Sensor nezobrazuje, proto se nepouzivaji.)
  // Jedna zprava pri prekroceni prahu, dalsi az po nabiti nad prah + 0,1 V a novem poklesu.
  bool low_active = false, crit_active = false;
  uint8_t low_cnt = 0, crit_cnt = 0;

  // vyhodnoceni jednoho prahu; zprava se odesle pri vzniku (kdyz se nepodari, zkusi se za minutu znovu)
  void battLevel(bool& active, uint8_t cnt, uint32_t mv, uint32_t thr_mv, const char* what) {
    bool now = cnt >= BATT_DEBOUNCE || (active && mv < thr_mv + BATT_HYST_MV);
    if (now && !active) {
      char body[48];
      snprintf(body, sizeof(body), "baterie %s %.2f V", what, mv / 1000.0f);
      active = sendChannelText(body, REPLY_BASE_MS + slotDelay());
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

  // ---------- klic kanalu a BATKAL: ulozeni v pameti uzlu ----------
  void applyChannelSecret(const uint8_t* secret32) {
    static const uint8_t zeroes[16] = {0};
    memcpy(light_chan.secret, secret32, PUB_KEY_SIZE);
    // stejne jako aplikace/companion: 128bit klic (druha polovina nulova) nebo 256bit klic
    int klen = (memcmp(&secret32[16], zeroes, 16) == 0) ? 16 : 32;
    mesh::Utils::sha256(light_chan.hash, sizeof(light_chan.hash), light_chan.secret, klen);
    light_chan_ok = true;
    chan_senders_n = chan_senders_next = 0;
  }

  bool saveChannel(const uint8_t* secret32) {
#if defined(NRF52_PLATFORM)
    InternalFS.remove(chan_file);
    if (secret32 == NULL) return true;
    File f = InternalFS.open(chan_file, FILE_O_WRITE);
    if (!f) return false;
    bool ok = f.write(secret32, PUB_KEY_SIZE) == PUB_KEY_SIZE;
    f.close();
    return ok;
#else
    return false;
#endif
  }

  void saveBatkal() {
#if defined(NRF52_PLATFORM)
    InternalFS.remove(BATKAL_FILE);
    File f = InternalFS.open(BATKAL_FILE, FILE_O_WRITE);
    if (f) {
      uint8_t b[2] = { (uint8_t)(batkal & 0xFF), (uint8_t)(batkal >> 8) };
      f.write(b, 2);
      f.close();
    }
#endif
  }

  // ---------- CLI: USB nebo LoRa od prihlaseneho admina ----------
  // prevod na mala pismena a oriznuti mezer na konci (do pomocneho bufferu); hodiny uzlu po startu
  // nejdou spravne -> srovnat podle prikazu od admina (jen dopredu)
  void cliPrepare(uint32_t sender_timestamp, const char* command, char* cmd, size_t max) {
    int n = 0;
    while (command[n] && n < (int)max - 1) {
      cmd[n] = tolower((unsigned char)command[n]);
      n++;
    }
    cmd[n] = 0;
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\r' || cmd[n - 1] == '\n')) cmd[--n] = 0;
    if (sender_timestamp > getRTCClock()->getCurrentTime()) getRTCClock()->setCurrentTime(sender_timestamp);
  }

  // spolecne prikazy CLI: CHAN, BATKAL, WDTTEST (jen USB); cmd uz je z cliPrepare
  bool execCommon(uint32_t sender_timestamp, const char* cmd, char* reply) {
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
      saveChannel(NULL);
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
        bool saved = saveChannel(secret);
        sprintf(reply, "OK chan ON hash=%02X%s", light_chan.hash[0], saved ? "" : " (NEULOZENO!)");
      } else {
        strcpy(reply, "Err - klic musi mit 32 nebo 64 hex znaku");
      }
      return true;
    }

    // BATKAL 4.12 = napeti namerene multimetrem na clanku (carka i tecka), BATKAL OFF = bez korekce
    if (strcmp(cmd, "batkal") == 0 || strncmp(cmd, "batkal ", 7) == 0) {
      if (strcmp(cmd, "batkal off") == 0) {
        batkal = 1000;
        saveBatkal();
      } else if (cmd[6] == ' ') {
        char num[12];
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
        saveBatkal();
      }
      sprintf(reply, "batkal=%u.%03u bat=%.2fV", batkal / 1000, batkal % 1000, battVolts());
      return true;
    }
    return false;
  }

  // ---------- soukromy kanal ----------
  int searchChannelsByHash(const uint8_t* hash, mesh::GroupChannel channels[], int max_matches) override {
    if (light_chan_ok && max_matches > 0 && memcmp(hash, light_chan.hash, sizeof(light_chan.hash)) == 0) {
      channels[0] = light_chan;
      return 1;
    }
    return 0;
  }

  char chan_sender[32] = "";   // jmeno odesilatele posledni zpravy z kanalu, malymi pismeny (z chanCommand)

  // Rozbali textovou zpravu z kanalu "<odesilatel>: <text>": do cmd da text malymi pismeny bez mezer na konci,
  // vrati casove razitko a hash odesilatele (pro chanReplayOk). Hodiny uzlu srovna podle kazde overene zpravy
  // v kanalu (jen dopredu), at maji upozorneni spravne datum co nejdriv po restartu.
  bool chanCommand(uint8_t type, const uint8_t* data, size_t len, char* cmd, size_t max, uint32_t& ts, uint32_t& sender_hash) {
    if (type != PAYLOAD_TYPE_GRP_TXT || len < 6 || len > 5 + 120) return false;
    if ((data[4] >> 2) != 0) return false;      // jen obycejny text (TXT_TYPE_PLAIN)

    memcpy(&ts, data, 4);
    if (ts > getRTCClock()->getCurrentTime()) getRTCClock()->setCurrentTime(ts);

    char text[128];
    size_t tl = len - 5;
    memcpy(text, &data[5], tl);
    text[tl] = 0;
    const char* sep = strstr(text, ": ");
    size_t sender_len = sep ? (size_t)(sep - text) : 0;
    size_t sl = sender_len < sizeof(chan_sender) - 1 ? sender_len : sizeof(chan_sender) - 1;
    for (size_t i = 0; i < sl; i++) chan_sender[i] = tolower((unsigned char)text[i]);
    chan_sender[sl] = 0;
    sender_hash = nameHash(text, sender_len);
    const char* body = sep ? sep + 2 : text;
    int n = 0;
    while (body[n] && n < (int)max - 1) { cmd[n] = tolower((unsigned char)body[n]); n++; }
    cmd[n] = 0;
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\r' || cmd[n - 1] == '\n')) cmd[--n] = 0;
    return true;
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
};
