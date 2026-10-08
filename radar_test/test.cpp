#include "mocks.h"
#include <deque>

// napodoba LD2410S na UART: odpovida na prikazy podle protokolu, posila minimalni ramce (6E .. 62) jako sum
struct UartMock {
  std::deque<uint8_t> in;
  bool alive = true, open = false;
  int rx_pin = -1, tx_pin = -1;
  uint8_t trig[16] = {48,42,36,34,32,31,31,31,31,31,31,31,31,31,31,31};
  uint8_t hold[16] = {45,42,33,32,28,28,28,28,28,28,28,28,28,28,28,28};
  bool config = false;
  int auto_scan = -1;
  std::vector<uint16_t> cmds;
  void setPins(int rx, int tx) { rx_pin = rx; tx_pin = tx; }
  void begin(uint32_t) { open = true; noise(); }
  void end() { open = false; }
  void noise() { for (uint8_t b : {0x6E, 0x02, 0x96, 0x00, 0x62}) in.push_back(b); }
  int available() { if (in.empty()) g_millis++; return (int)in.size(); }   // bez dat ubiha cas (timeout)
  int read() { uint8_t c = in.front(); in.pop_front(); return c; }
  void frame(bool ack, const std::vector<uint8_t>& d) {
    const uint8_t* h = ack ? (const uint8_t*)"\xFD\xFC\xFB\xFA" : (const uint8_t*)"\xF4\xF3\xF2\xF1";
    const uint8_t* t = ack ? (const uint8_t*)"\x04\x03\x02\x01" : (const uint8_t*)"\xF8\xF7\xF6\xF5";
    noise();
    for (int i = 0; i < 4; i++) in.push_back(h[i]);
    in.push_back(d.size() & 0xFF); in.push_back(d.size() >> 8);
    for (uint8_t b : d) in.push_back(b);
    for (int i = 0; i < 4; i++) in.push_back(t[i]);
  }
  void ack(uint16_t cmd, std::vector<uint8_t> extra = {}) {
    std::vector<uint8_t> d = {(uint8_t)(cmd & 0xFF), (uint8_t)((cmd >> 8) | 1), 0, 0};
    d.insert(d.end(), extra.begin(), extra.end());
    frame(true, d);
  }
  void progress(uint16_t p) { frame(false, {0x03, 0x00, (uint8_t)(p & 0xFF), (uint8_t)(p >> 8)}); }
  size_t write(const uint8_t* f, size_t n) {
    if (!open || !alive || n < 12 || memcmp(f, "\xFD\xFC\xFB\xFA", 4) != 0) return n;
    uint16_t cmd = f[6] | (f[7] << 8);
    const uint8_t* d = &f[8];
    cmds.push_back(cmd);
    if (cmd == 0x00FF) { config = true; ack(cmd, {3, 0, 0x80, 0}); }
    else if (cmd == 0x00FE) { config = false; ack(cmd); }
    else if (!config) { }   // mimo konfiguracni rezim radar neodpovi
    else if (cmd == 0x0009) { auto_scan = d[4] | (d[5] << 8); ack(cmd); }
    else if (cmd == 0x0073 || cmd == 0x0077) {
      std::vector<uint8_t> v;
      for (int g = 0; g < 16; g++) { v.push_back(cmd == 0x0073 ? trig[g] : hold[g]); v.push_back(0); v.push_back(0); v.push_back(0); }
      ack(cmd, v);
    }
    return n;
  }
} radar_uart;
#define RADAR_UART  radar_uart
#define PIN_RADAR_UART_RX  30
#define PIN_RADAR_UART_TX  31

#include "extracted.h"
static int fails = 0, checks = 0;
#define CHECK(c, msg) do { checks++; if (!(c)) { fails++; printf("FAIL: %s  (line %d)\n", msg, __LINE__); } } while(0)

mesh::MainBoard mb; mesh::Radio mr; mesh::MillisecondClock mc; mesh::RNG rg; mesh::RTCClock rc; mesh::MeshTables mt;

static void chanMsg(MyMesh& m, uint32_t ts, const char* txt) {
  uint8_t data[200]; memset(data, 0, sizeof(data));
  memcpy(data, &ts, 4); data[4] = 0;
  size_t tl = strlen(txt); memcpy(&data[5], txt, tl);
  size_t len = 5 + tl; len = (len + 15) / 16 * 16;   // jako po desifrovani (zarovnani na bloky)
  mesh::Packet p; m.onGroupDataRecv(&p, PAYLOAD_TYPE_GRP_TXT, m.light_chan, data, len);
}
static std::string cli(MyMesh& m, uint32_t ts, const char* c) {
  char cmd[160]; strcpy(cmd, c); char reply[200]; reply[0] = 0;
  bool h = m.handleCustomCommand(ts, cmd, reply);
  return h ? std::string(reply) : std::string("<NEZPRACOVANO>");
}
// pohyb: OUT radaru na HIGH (jedna smycka), pak zpet LOW
static void pulse(MyMesh& m) { m.radarLoop(true); lightTimerLoop(); m.radarLoop(false); }

int main() {
  MyMesh m(mb, mr, mc, rg, rc, mt);
  strcpy(m.prefs.node_name, "dum-radar");
  g_millis = 1000;

  // --- start uzlu s ulozenym RADAR ON + SVETLO ON a clovekem pred radarem ---
  {
    const uint32_t W = RADAR_STARTUP_MS;           // doba ustalovani (testy bezi pro 30 s i 300 s)
    MyMesh b(mb, mr, mc, rg, rc, mt);
    strcpy(b.prefs.node_name, "dum-radar");
    b.radar_on = true; b.light_on = true;          // jako po loadRadarState()
    b.applyChannelSecret((const uint8_t*)"0123456789abcdef0123456789abcdef");
    g_sent.clear(); board.setGpio(0);
    g_millis = 500; b.radarLoop(true); lightTimerLoop();
    CHECK(g_sent.empty() && board.light_state == 0, "start s OT2=HIGH: zadny poplach ani svetlo");
    g_millis = 5000; b.radarLoop(false); b.radarLoop(true); lightTimerLoop();
    CHECK(g_sent.empty() && board.light_state == 0, "behem ustalovani se pohyb nevyhodnocuje");
    CHECK(b.ot2_edges == 1, "hrany OT2 se pocitaji i behem ustalovani");
    b.radarLoop(false);
    if (W > 60000) {
      g_millis = 500 + W - 60001; b.radarLoop(false);
      CHECK(g_sent.empty(), "pred 'pripraven za 60 s' nic");
      g_millis = 500 + W - 60000; b.radarLoop(false);
      CHECK(g_sent.size() == 1 && g_sent[0].text == "dum-radar: radar pripraven za 60 s (RADAR ON)", "60 s pred koncem ustalovani zprava");
      b.radarLoop(true); b.radarLoop(false);
      CHECK(g_sent.size() == 1 && board.light_state == 0, "zprava 'za 60 s' jen jednou, pohyb porad ignorovan");
      g_sent.clear();
    }
    g_millis = 500 + W - 1; b.radarLoop(false);
    CHECK(g_sent.empty(), "tesne pred koncem ustalovani nic");
    g_millis = 500 + W; b.radarLoop(false);
    CHECK(g_sent.size() == 1 && g_sent[0].text == "dum-radar: radar pripraven (RADAR ON, SVETLO ON)", "konec ustalovani -> zprava pripraven");
    b.radarLoop(false);
    CHECK(g_sent.size() == 1, "zprava 'pripraven' jen jednou");
    g_millis = 500 + W + 500; b.radarLoop(true); lightTimerLoop();
    CHECK(g_sent.size() == 2 && g_sent[1].text.rfind("dum-radar: POHYB!", 0) == 0 && board.light_state == 1, "po ustaleni pohyb rozsviti a posle zpravu");
    board.setGpio(0); g_sent.clear();
  }
  g_millis = 1000;
  m.radarLoop(false);                    // prvni cteni OT2 (zacatek ustalovani) ...
  m.radar_start = g_millis - RADAR_STARTUP_MS - 100000;   // ... ustaleni uz probehlo
  m.ready_warn_sent = m.ready_sent = true;                // a zpravy o pripravenosti odesly

  // --- CLI ---
  CHECK(cli(m, 0, "chan") == "chan OFF", "chan bez klice");
  CHECK(cli(m, 0, "chan 00112233445566778899aabbccddeeff").rfind("OK chan ON hash=", 0) == 0, "chan 32 hex");
  std::string st = cli(m, 0, "STATUS RADAR");
  CHECK(st.rfind("RADAR OFF SVETLO OFF ot2=0/0 pohyb=0 (-) bat=3.90V rssi=-61 snr=9.", 0) == 0, "STATUS RADAR format");
  CHECK(st.find(" up=0d00h00m") != std::string::npos, "STATUS up");
  CHECK(cli(m, 0, "Status") == st, "STATUS = STATUS RADAR");
  CHECK(cli(m, 0, "radar on ") == "RADAR ON", "RADAR ON");
  CHECK(cli(m, 0, "SVETLO ON") == "SVETLO ON", "SVETLO ON");
  CHECK(cli(m, 0, "svetlo off") == "SVETLO OFF", "SVETLO OFF");
  CHECK(cli(m, 0, "radar off") == "RADAR OFF", "RADAR OFF");
  CHECK(cli(m, 0, "lon") == "<NEZPRACOVANO>", "LON neni prikaz radaru");
  CHECK(cli(m, 1800000000, "alerttest") == "<NEZPRACOVANO>", "ALERTTEST na dalku nejde");
  CHECK(cli(m, 1800000000, "wdttest") == "<NEZPRACOVANO>", "WDTTEST na dalku nejde");

  // --- hlidani vypnute: pohyb nic nedela ---
  g_sent.clear(); board.setGpio(0);
  cli(m, 0, "svetlo on");
  pulse(m);
  CHECK(g_sent.empty(), "RADAR OFF: pohyb bez zpravy");
  CHECK(board.light_state == 0, "RADAR OFF: svetlo se nerozsviti ani pri SVETLO ON");

  // --- hlidani zapnute, svetlo vypnute ---
  cli(m, 0, "svetlo off"); cli(m, 0, "radar on");
  g_millis = 10000; pulse(m);
  CHECK(g_sent.size() == 1 && g_sent[0].text == "dum-radar: POHYB! bat=3.90V", "prvni pohyb -> zprava hned");
  CHECK(g_sent[0].delay < 1000, "poplach bez zbytecneho zpozdeni");
  CHECK(board.light_state == 0, "SVETLO OFF: nesviti");
  m.radarLoop(true); m.radarLoop(true);   // 2. pohyb (OUT znovu HIGH)
  CHECK(g_sent.size() == 1, "trvajici pritomnost (OUT stale HIGH) neni novy pohyb");
  m.radarLoop(false);

  // --- cooldown 60 s ---
  g_millis = 20000; pulse(m);
  g_millis = 30000; pulse(m);
  CHECK(g_sent.size() == 1, "behem 60 s zadna dalsi zprava");
  g_millis = 69999; m.radarLoop(false);
  CHECK(g_sent.size() == 1, "v 59,999 s porad nic");
  g_millis = 70000; m.radarLoop(false);
  CHECK(g_sent.size() == 2 && g_sent[1].text == "dum-radar: POHYB! 3x za 60s, bat=3.90V", "po 60 s souhrn pohybu");
  g_millis = 200000; m.radarLoop(false);
  CHECK(g_sent.size() == 2, "bez noveho pohybu uz nic");
  g_millis = 200001; pulse(m);
  CHECK(g_sent.size() == 3 && g_sent[2].text == "dum-radar: POHYB! bat=3.90V", "pohyb po klidu -> hned zprava");

  // --- svetlo pri pohybu na 3 s ---
  cli(m, 0, "svetlo on");
  g_millis = 300000; m.radarLoop(true); lightTimerLoop();
  CHECK(board.light_state == 1, "SVETLO ON: pohyb rozsviti");
  g_millis = 302999; lightTimerLoop(); CHECK(board.light_state == 1, "sviti v 2,999 s");
  g_millis = 303000; lightTimerLoop(); CHECK(board.light_state == 0, "zhasne ve 3 s");
  m.radarLoop(false);
  g_millis = 310000; m.radarLoop(true); lightTimerLoop(); m.radarLoop(false);
  g_millis = 312000; m.radarLoop(true); lightTimerLoop(); m.radarLoop(false);   // dalsi pohyb obnovi odpocet
  g_millis = 314500; lightTimerLoop(); CHECK(board.light_state == 1, "dalsi pohyb prodlouzi svit");
  g_millis = 315000; lightTimerLoop(); CHECK(board.light_state == 0, "pak zhasne");
  CHECK(cli(m, 0, "svetlo test") == "SVETLO TEST 3s" && board.light_state == 1, "SVETLO TEST");
  cli(m, 0, "svetlo off");
  CHECK(board.light_state == 0, "SVETLO OFF hned zhasne");

  // --- RADAR OFF zahodi cekajici pohyby ---
  g_sent.clear();
  g_millis = 400000; pulse(m);              // zprava hned
  g_millis = 410000; pulse(m);              // ceka na cooldown
  cli(m, 0, "radar off");
  g_millis = 500000; m.radarLoop(false);
  CHECK(g_sent.size() == 1, "po RADAR OFF se cekajici souhrn neposle");
  std::string st2 = cli(m, 0, "status");
  CHECK(st2.find("pohyb=10 (1m)") != std::string::npos, "STATUS: pocet a cas posledniho pohybu");
  cli(m, 0, "radar on");
  CHECK(cli(m, 0, "status").find("pohyb=0 (-)") != std::string::npos, "RADAR ON nuluje pocitadlo");

  // --- kanal ---
  g_sent.clear();
  uint32_t T = 1800001000;
  chanMsg(m, T, "Jirka: RADAR OFF");
  CHECK(g_sent.size() == 1 && g_sent[0].text == "dum-radar: RADAR OFF" && g_sent[0].delay == 600, "kanal RADAR OFF");
  CHECK(m.rtc.t >= T, "hodiny srovnany podle kanalu");
  chanMsg(m, T, "Jirka: RADAR ON");
  CHECK(g_sent.size() == 1, "prehrani (stejne razitko) odmitnuto");
  chanMsg(m, T + 1, "Jirka: radar on");
  CHECK(g_sent.size() == 2 && g_sent[1].text == "dum-radar: RADAR ON", "kanal radar on malymi");
  chanMsg(m, T + 2, "Jirka: STATUS RADAR");
  CHECK(g_sent.size() == 3 && g_sent[2].text.rfind("dum-radar: RADAR ON SVETLO OFF", 0) == 0, "kanal STATUS RADAR");
  CHECK(g_sent[2].delay == 600, "STATUS RADAR odpovida hned");
  chanMsg(m, T + 3, "Jirka: STATUS");
  CHECK(g_sent.size() == 4 && g_sent[3].delay == REPLY_ALL_DELAY_MS, "STATUS pro vsechny: odpoved az po svetlech");
  chanMsg(m, T + 4, "Jirka: STATUS 2");
  CHECK(g_sent.size() == 4, "STATUS 2 (svetlo 2) neni pro radar bez cisla");
  chanMsg(m, T + 5, "Jirka: STATUS all");
  CHECK(g_sent.size() == 5, "STATUS all");
  chanMsg(m, T + 6, "Jirka: SVETLO ON");
  CHECK(g_sent.size() == 6 && g_sent[5].text == "dum-radar: SVETLO ON", "kanal SVETLO ON");
  chanMsg(m, T + 7, "Jirka: svetlo test");
  CHECK(board.light_state == 1, "kanal SVETLO TEST");
  board.setGpio(0);
  size_t before = g_sent.size();
  chanMsg(m, T + 8, "Jirka: LON");                  CHECK(g_sent.size() == before, "LON svetel radar ignoruje");
  chanMsg(m, T + 9, "Jirka: radar");                CHECK(g_sent.size() == before, "neuplny prikaz ignorovan");
  chanMsg(m, T + 10, "Jirka: radar test");          CHECK(g_sent.size() == before, "RADAR TEST neexistuje");
  chanMsg(m, T + 11, "dum-radar: RADAR ON");        CHECK(g_sent.size() == before + 1, "(vlastni ozvena se do kanalu nevraci, ale kdyby ano, zpracuje se jako prikaz)");
  chanMsg(m, T + 12, "svetlo-1: OFF bat=3.80V rssi=-60 snr=9.5 up=0d01h00m"); CHECK(g_sent.size() == before + 1, "odpoved svetla ignorovana");
  chanMsg(m, T + 13, "Jirka: ahoj");                CHECK(g_sent.size() == before + 1, "bezna zprava ignorovana");
  // cislovane radary
  strcpy(m.prefs.node_name, "dum-radar-2");
  chanMsg(m, T + 14, "Jirka: RADAR OFF 1");         CHECK(m.radar_on, "RADAR OFF 1 neni pro radar 2");
  chanMsg(m, T + 15, "Jirka: RADAR OFF 2");         CHECK(!m.radar_on, "RADAR OFF 2 je pro radar 2");
  chanMsg(m, T + 16, "Jirka: STATUS 2");            CHECK(g_sent.back().text.rfind("dum-radar-2: RADAR OFF", 0) == 0, "STATUS 2 pro radar 2");
  strcpy(m.prefs.node_name, "dum-radar");

  // --- poplach bez kanalu se nehromadi ---
  cli(m, 0, "radar on"); cli(m, 0, "chan off"); g_sent.clear();
  g_millis = 600000; pulse(m);
  CHECK(g_sent.empty(), "bez kanalu nic neodeslano");
  cli(m, 0, "chan 00112233445566778899aabbccddeeff");
  g_millis = 700000; m.radarLoop(false);
  CHECK(g_sent.empty(), "stary poplach se po nastaveni kanalu neposle");

  // --- test radaru: ot2 a pocet hran i pri RADAR OFF ---
  cli(m, 0, "radar off");
  uint32_t e0 = m.ot2_edges;
  m.radarLoop(true);
  CHECK(cli(m, 0, "status").find(" ot2=1/") != std::string::npos, "STATUS ukazuje ot2=1 pri pritomnosti");
  m.radarLoop(false);
  CHECK(m.ot2_edges == e0 + 1, "hrany OT2 se pocitaji i pri RADAR OFF");
  char exp[24]; snprintf(exp, sizeof(exp), " ot2=0/%u ", (unsigned)(e0 + 1));
  CHECK(cli(m, 0, "status").find(exp) != std::string::npos, "STATUS ukazuje ot2=0/pocet hran");
  // nejdelsi mozna odpoved se vejde do zpravy v kanalu (~120 B)
  {
    MyMesh w(mb, mr, mc, rg, rc, mt);
    strcpy(w.prefs.node_name, "dum-radar-2");
    w.ot2_edges = 99999; w.motion_count = 99999; w.motion_last = 0; g_millis = 23*3600000u + 59*60000u;
    uint64_t sv = uptime_ms; uptime_ms = (uint64_t)(99*1440 + 23*60 + 59) * 60000ULL; board.mv = 4199;
    std::string longest = std::string(w.prefs.node_name) + ": " + cli(w, 0, "status");
    uptime_ms = sv; board.mv = 3900;
    CHECK(longest.size() <= 120, "nejdelsi STATUS do 120 znaku");
  }
  // --- baterie (stejna logika jako svetla) ---
  g_sent.clear();
  auto minute = [&](uint16_t mv){ board.mv = mv; m.onSensorDataRead(); };
  minute(3490); minute(3490);
  CHECK(g_sent.empty(), "2 mereni pod prahem jeste nic");
  minute(3490);
  CHECK(g_sent.size() == 1 && g_sent[0].text == "dum-radar: baterie slaba 3.49 V", "baterie slaba do kanalu");
  board.ext = true; minute(3000); minute(3000); minute(3000); board.ext = false;
  CHECK(g_sent.size() == 1, "na USB se baterie nehlida");
  board.mv = 3900;
  CHECK(cli(m, 0, "alerttest") == "OK - test poplachu odeslan do kanalu", "ALERTTEST");
  CHECK(g_sent.back().text == "dum-radar: test poplachu, bat 3.90 V", "ALERTTEST zprava");

  // --- kalibrace pres UART (automaticke prahy LD2410S) ---
  {
    MyMesh k(mb, mr, mc, rg, rc, mt);
    strcpy(k.prefs.node_name, "dum-radar");
    k.applyChannelSecret((const uint8_t*)"0123456789abcdef0123456789abcdef");
    g_millis = 1000000;
    k.radarLoop(false);
    k.radar_start = g_millis - RADAR_STARTUP_MS - 1000; k.ready_warn_sent = k.ready_sent = true;
    k.radar_on = true;

    radar_uart.alive = false;
    CHECK(cli(k, 0, "radar kalibrace") == "Err - radar neodpovida (UART)", "kalibrace bez radaru: chyba");
    CHECK(!radar_uart.open && k.kal_state == MyMesh::KAL_IDLE, "UART zase vypnuty");
    CHECK(cli(k, 0, "radar prahy") == "Err - radar neodpovida (UART)", "prahy bez radaru: chyba");
    radar_uart.alive = true;

    CHECK(cli(k, 0, "radar prahy") == "prahy T 48 42 36 34 32 31 31 31 31 31 31 31 31 31 31 31 H 45 42 33 32 28 28 28 28 28 28 28 28 28 28 28 28",
          "RADAR PRAHY cte oba radky prahu");
    CHECK(!radar_uart.open && !radar_uart.config, "po cteni prahu UART vypnuty, radar mimo konfiguraci");
    CHECK(radar_uart.rx_pin == 30 && radar_uart.tx_pin == 31, "UART na pinech NFC");
    CHECK(cli(k, 0, "radar kalibrace 1m") == "Err - doba 2m az 60m", "kratka doba odmitnuta");
    CHECK(cli(k, 0, "radar kalibracex") == "<NEZPRACOVANO>", "preklep neni prikaz");

    g_sent.clear();
    uint32_t t0 = g_millis;
    CHECK(cli(k, 0, "RADAR KALIBRACE") == "KALIBRACE za 60s, sken 15 min - odejdi z dosahu", "RADAR KALIBRACE potvrdi");
    t0 = k.kal_t0;   // napodoba UART posouva cas pri cekani na data
    CHECK(cli(k, 0, "radar kalibrace") == "KALIBRACE uz bezi", "druha kalibrace ne");
    CHECK(cli(k, 0, "status").find(" kal=start") != std::string::npos, "STATUS ukazuje cekani na kalibraci");
    g_millis = t0 + 5000; pulse(k);
    CHECK(g_sent.empty(), "odchod z dosahu behem kalibrace nehlasi poplach");
    g_millis = t0 + 59000; k.radarKalLoop();
    CHECK(radar_uart.auto_scan == -1, "sken jeste nezacal");
    g_millis = t0 + 60000; k.radarKalLoop();
    CHECK(radar_uart.auto_scan == 900 && !radar_uart.config, "po 60 s sken 900 s, radar mimo konfiguraci");
    radar_uart.progress(50); k.radarKalLoop();
    CHECK(cli(k, 0, "status").find(" kal=50") != std::string::npos, "STATUS ukazuje prubeh");
    CHECK(cli(k, 0, "radar prahy") == "KALIBRACE bezi, prahy az po ni", "prahy behem kalibrace ne");
    radar_uart.trig[0] = 55; radar_uart.hold[15] = 20;   // radar si nastavil nove prahy
    uint32_t ts = k.kal_t0;
    g_millis = ts + 900000 + RADAR_KAL_GRACE_MS - 1; k.radarKalLoop();
    CHECK(g_sent.empty(), "pred koncem skenu nic");
    g_millis = ts + 900000 + RADAR_KAL_GRACE_MS; k.radarKalLoop();
    CHECK(g_sent.size() == 1 && g_sent[0].text ==
          "dum-radar: kalibrace hotova T 55 42 36 34 32 31 31 31 31 31 31 31 31 31 31 31 H 45 42 33 32 28 28 28 28 28 28 28 28 28 28 28 20",
          "po skenu nove prahy do kanalu");
    CHECK(!radar_uart.open && k.kal_state == MyMesh::KAL_IDLE, "po kalibraci UART vypnuty");
    CHECK(cli(k, 0, "status").find(" kal=") == std::string::npos, "STATUS bez kal po kalibraci");
    g_millis += 1000; pulse(k);
    CHECK(g_sent.size() == 2 && g_sent[1].text.rfind("dum-radar: POHYB!", 0) == 0, "po kalibraci zase hlida");

    // kanal: doba a cile
    g_sent.clear(); radar_uart.auto_scan = -1;
    uint32_t T2 = 1900000000;
    chanMsg(k, T2, "Jirka: RADAR KALIBRACE 2");
    CHECK(g_sent.empty() && k.kal_state == MyMesh::KAL_IDLE, "RADAR KALIBRACE 2 neni pro radar bez cisla");
    chanMsg(k, T2 + 1, "Jirka: RADAR KALIBRACE 20m");
    CHECK(g_sent.size() == 1 && g_sent[0].text == "dum-radar: KALIBRACE za 60s, sken 20 min - odejdi z dosahu", "kanal RADAR KALIBRACE 20m");
    g_millis = k.kal_t0 + 60000; k.radarKalLoop();
    CHECK(radar_uart.auto_scan == 1200, "sken 20 min");
    radar_uart.alive = false;   // radar mezitim prestal odpovidat
    g_millis = k.kal_t0 + 1200000 + RADAR_KAL_GRACE_MS; k.radarKalLoop();
    CHECK(g_sent.back().text == "dum-radar: kalibrace CHYBA: prahy nejdou precist", "chyba cteni prahu hlasena");
    radar_uart.alive = true;
    chanMsg(k, T2 + 2, "Jirka: RADAR PRAHY");
    CHECK(g_sent.back().text.rfind("dum-radar: prahy T 55 ", 0) == 0, "kanal RADAR PRAHY");
    chanMsg(k, T2 + 3, "Jirka: RADAR OFF");
    CHECK(!k.radar_on, "RADAR OFF po zmene parseru funguje");
    size_t nb = g_sent.size();
    chanMsg(k, T2 + 4, "Jirka: RADAR KALIBRACE 20m 3");
    CHECK(g_sent.size() == nb && k.kal_state == MyMesh::KAL_IDLE, "KALIBRACE 20m 3 neni pro radar bez cisla");
  }

  printf("%d kontrol, %d chyb\n", checks, fails);
  return fails ? 1 : 0;
}
