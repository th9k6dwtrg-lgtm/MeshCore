#include "mocks.h"
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

  printf("%d kontrol, %d chyb\n", checks, fails);
  return fails ? 1 : 0;
}
