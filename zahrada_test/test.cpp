#include "mocks.h"
#include "extracted.h"
#include <cassert>
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

int main() {
  MyMesh m(mb, mr, mc, rg, rc, mt);

  // --- cislo svetla ---
  strcpy(m.prefs.node_name, "svetlo-1");  CHECK(m.nodeNumber() == 1, "svetlo-1 -> 1");
  strcpy(m.prefs.node_name, "svetlo12");  CHECK(m.nodeNumber() == 12, "svetlo12 -> 12");
  strcpy(m.prefs.node_name, "svetlo-1-x");CHECK(m.nodeNumber() == 0, "bez cisla na konci -> 0");
  strcpy(m.prefs.node_name, "3");         CHECK(m.nodeNumber() == 3, "3 -> 3");
  strcpy(m.prefs.node_name, "svetlo-1");

  // --- CLI ---
  CHECK(cli(m, 0, "chan") == "chan OFF", "chan bez klice");
  CHECK(cli(m, 0, "CHAN 00112233445566778899aabbccddeeff") .rfind("OK chan ON hash=", 0) == 0, "chan 32 hex");
  CHECK(cli(m, 0, "chan 0011") .rfind("Err", 0) == 0, "kratky klic odmitnut");
  CHECK(cli(m, 0, "chan") .rfind("chan ON", 0) == 0, "chan stav ON");
  CHECK(cli(m, 0, "LON  ").rfind("ON 5s c.1 bat=", 0) == 0, "LON velkymi + mezery, poradove cislo a napeti");
  CHECK(board.light_state == 1, "LON rozsvitil");
  CHECK(cli(m, 0, "loff") == "OFF", "loff");
  CHECK(board.light_state == 0, "LOFF zhasl");
  std::string st = cli(m, 0, "Status");
  CHECK(st.rfind("OFF bat=3.90V rssi=-61 snr=9.2", 0) == 0 || st.rfind("OFF bat=3.90V rssi=-61 snr=9.3", 0) == 0, "STATUS format");
  CHECK(st.find(" up=0d00h00m") != std::string::npos, "STATUS up");
  CHECK(cli(m, 0, "lights on") == "<NEZPRACOVANO>", "preklep jde do standardniho CLI");
  CHECK(cli(m, 1800000000, "alerttest") == "<NEZPRACOVANO>", "ALERTTEST na dalku nejde");
  CHECK(cli(m, 1800000000, "wdttest") == "<NEZPRACOVANO>", "WDTTEST na dalku nejde");
  // srovnani hodin podle CLI od admina (jen dopredu)
  cli(m, 1800000000, "status"); CHECK(m.rtc.t == 1800000000, "hodiny srovnany podle CLI");
  cli(m, 1700000000, "status"); CHECK(m.rtc.t == 1800000000, "hodiny nejdou dozadu");

  // --- casovac 5 s ---
  g_millis = 10000; cli(m, 0, "lon"); m.rtc.t++;
  g_millis = 14999; lightTimerLoop(); CHECK(board.light_state == 1, "sviti v 4,999 s");
  g_millis = 15000; lightTimerLoop(); CHECK(board.light_state == 0, "zhasne v 5,000 s");
  g_millis = 20000; cli(m, 0, "lon"); g_millis = 23000; lightTimerLoop();
  cli(m, 0, "lon");   // obnoveni odpoctu
  g_millis = 27000; lightTimerLoop(); CHECK(board.light_state == 1, "opakovany LON obnovi odpocet");
  g_millis = 28000; lightTimerLoop(); CHECK(board.light_state == 0, "po obnoveni zhasne v 5 s");
  // rozsviceni jinou cestou (io s 1)
  g_millis = 40000; board.setGpio(1); lightTimerLoop(); g_millis = 44999; lightTimerLoop();
  CHECK(board.light_state == 1, "io s 1 sviti");
  g_millis = 45001; lightTimerLoop(); CHECK(board.light_state == 0, "io s 1 zhasne po 5 s");
  // preteceni millis()
  g_millis = 0xFFFFF000u; cli(m, 0, "lon"); lightTimerLoop(); g_millis = 0xFFFFF000u + 4000; lightTimerLoop();
  CHECK(board.light_state == 1, "kolem preteceni millis sviti");
  g_millis = 0xFFFFF000u + 5000; lightTimerLoop(); CHECK(board.light_state == 0, "kolem preteceni millis zhasne");
  g_millis = 50000;

  // --- kanal ---
  g_sent.clear();
  uint32_t T = 1800001000;
  chanMsg(m, T, "Jirka: LON");             CHECK(board.light_state == 1, "kanal LON pro vsechny");
  CHECK(g_sent.size() == 1 && g_sent[0].text == "svetlo-1: ON 5s c." + std::to_string(m.light_seq) + " bat=3.90V", "odpoved do kanalu s cislem rozsviceni a napetim");
  { uint32_t n = m.light_seq; cli(m, 0, "lon"); CHECK(m.light_seq == n + 1, "kazde LON zvysi poradove cislo o 1"); cli(m, 0, "loff"); }
  CHECK(g_sent[0].delay == 600, "zpozdeni svetla 1");
  CHECK(m.rtc.t >= T, "hodiny srovnany podle kanalu");
  board.setGpio(0);
  chanMsg(m, T, "Jirka: LON");             CHECK(board.light_state == 0, "prehrani stejne zpravy odmitnuto");
  chanMsg(m, T - 5, "Jirka: LON");         CHECK(board.light_state == 0, "starsi zprava stejneho odesilatele odmitnuta");
  chanMsg(m, T - 30, "Jana: LON");         CHECK(board.light_state == 1, "manzelka s o 30 s pozadu jdoucimi hodinami prijata");
  board.setGpio(0);
  chanMsg(m, T + 10, "Jirka: LON 2");      CHECK(board.light_state == 0, "LON 2 neni pro svetlo 1");
  chanMsg(m, T + 11, "Jirka: LON 1 3");    CHECK(board.light_state == 1, "LON 1 3 je pro svetlo 1");
  board.setGpio(0);
  chanMsg(m, T + 12, "Jirka: lon 2,1");    CHECK(board.light_state == 1, "lon 2,1 (carka)");
  board.setGpio(0);
  chanMsg(m, T + 13, "Jirka: LON all");    CHECK(board.light_state == 1, "LON all");
  chanMsg(m, T + 14, "Jirka: LOFF");       CHECK(board.light_state == 0, "LOFF v kanalu");
  size_t before = g_sent.size();
  chanMsg(m, T + 15, "svetlo-2: ON 5s");   CHECK(g_sent.size() == before, "odpovedi jinych svetel ignorovany");
  chanMsg(m, T + 16, "svetlo-2: OFF bat=3.80V rssi=-60 snr=9.5 up=0d01h00m"); CHECK(g_sent.size() == before, "STATUS odpoved jineho svetla ignorovana");
  chanMsg(m, T + 17, "svetlo-2: baterie slaba 3.45 V"); CHECK(g_sent.size() == before, "upozorneni jineho svetla ignorovano");
  chanMsg(m, T + 18, "Jirka: ahoj");       CHECK(g_sent.size() == before, "bezna zprava ignorovana");
  chanMsg(m, T + 19, "Jirka: STATUS");     CHECK(g_sent.size() == before + 1 && g_sent.back().text.rfind("svetlo-1: OFF bat=", 0) == 0, "STATUS v kanalu");
  strcpy(m.prefs.node_name, "svetlo-4");
  chanMsg(m, T + 20, "Jirka: STATUS");     CHECK(g_sent.back().delay == 600 + 3*1500, "zpozdeni svetla 4");
  strcpy(m.prefs.node_name, "svetlo-1");

  // --- upozorneni na baterii (jen do kanalu) ---
  cli(m, 0, "chan 00112233445566778899aabbccddeeff");
  g_sent.clear(); g_alerts.clear();
  auto minute = [&](uint16_t mv){ board.mv = mv; m.onSensorDataRead(); };
  minute(3600); minute(3490); minute(3490);
  CHECK(g_sent.empty(), "2 mereni pod prahem jeste nic");
  minute(3490);
  CHECK(g_sent.size() == 1 && g_sent[0].text == "svetlo-1: baterie slaba 3.49 V", "3. mereni -> 1 zprava do kanalu");
  CHECK(g_alerts.empty(), "zadne prime zpravy (alertIf se nepouziva)");
  minute(3480); minute(3550); minute(3590); minute(3480);
  CHECK(g_sent.size() == 1, "kolisani pod prahem+0,1 V neopakuje upozorneni");
  minute(3600);   // nabito nad prah + 0,1 V -> zruseno
  CHECK(!m.low_active, "zruseni po vzrustu o 0,1 V");
  minute(3490); minute(3490); minute(3490);
  CHECK(g_sent.size() == 2, "novy pokles -> nove upozorneni");
  minute(3340); minute(3340); minute(3340);
  CHECK(g_sent.size() == 3 && g_sent.back().text == "svetlo-1: baterie KRITICKA 3.34 V", "kriticke upozorneni");
  board.ext = true; minute(3000); minute(3000); minute(3000); board.ext = false;
  CHECK(g_sent.size() == 3, "na USB se upozorneni nevyhodnocuje");
  // bez klice kanalu se upozorneni neztrati: odesle se, jakmile je kanal nastaven
  minute(3700); minute(3700);   // vse zruseno
  cli(m, 0, "chan off"); g_sent.clear();
  minute(3490); minute(3490); minute(3490); minute(3490);
  CHECK(g_sent.empty() && !m.low_active, "bez kanalu nic neodeslano, upozorneni ceka");
  cli(m, 0, "chan 00112233445566778899aabbccddeeff");
  minute(3490);
  CHECK(g_sent.size() == 1 && g_sent[0].text == "svetlo-1: baterie slaba 3.49 V", "po nastaveni kanalu se upozorneni odesle");
  // ALERTTEST
  g_sent.clear(); board.mv = 3900;
  CHECK(cli(m, 0, "alerttest") == "OK - test upozorneni odeslan do kanalu", "ALERTTEST odpoved");
  CHECK(g_sent.size() == 1 && g_sent[0].text == "svetlo-1: test upozorneni, bat 3.90 V", "ALERTTEST zprava do kanalu");
  cli(m, 0, "chan off");
  CHECK(cli(m, 0, "alerttest") == "Err - kanal neni nastaven", "ALERTTEST bez kanalu");

  // --- 5 lidi v kanalu s ruzne jdoucimi hodinami ---
  cli(m, 0, "chan 00112233445566778899aabbccddeeff");
  {
    const char* lide[5] = {"Jirka", "Jana", "Petr", "Eva", "Tomas"};
    int offs[5] = {0, -40, +25, -90, +5};      // rozdil hodin telefonu (s)
    uint32_t B = 1800100000;
    int ok = 0;
    for (int kolo = 0; kolo < 3; kolo++) {
      for (int p = 0; p < 5; p++) {
        board.setGpio(0);
        char msg[48]; snprintf(msg, sizeof(msg), "%s: LON", lide[p]);
        chanMsg(m, B + kolo * 10 + p + offs[p], msg);
        if (board.light_state == 1) ok++;
      }
    }
    CHECK(ok == 15, "5 lidi s ruznymi hodinami: vsech 15 prikazu provedeno");
    board.setGpio(0);
    chanMsg(m, B + 20 + 1 + offs[1], "Jana: LON");   // presne prehrani posledni Janiny zpravy
    CHECK(board.light_state == 0, "prehrani zpravy jednoho z 5 lidi odmitnuto");
  }
  // hodiny se srovnaji i podle beznych zprav a prikazu pro jina svetla
  m.rtc.t = 1715770351;                       // jako po restartu (rok 2024)
  chanMsg(m, 1800200000, "Jana: ahoj, jdu na zahradu");
  CHECK(m.rtc.t == 1800200000, "hodiny srovnany podle bezne zpravy v kanalu");
  chanMsg(m, 1800200100, "Petr: LON 3");
  CHECK(m.rtc.t == 1800200100, "hodiny srovnany podle prikazu pro jine svetlo");
  chanMsg(m, 1800000000, "Eva: ahoj");
  CHECK(m.rtc.t == 1800200100, "hodiny nejdou dozadu");

  // --- doplnkove okrajove pripady ---
  strcpy(m.prefs.node_name, "svetlo-1"); board.setGpio(0); light_timer_armed = false;
  g_millis = 100000; cli(m, 0, "lon");
  g_millis = 100500; lightTimerLoop();               // smycka zdrzena 500 ms po LON
  g_millis = 104999; lightTimerLoop(); CHECK(board.light_state == 1, "zdrzena smycka: jeste sviti");
  g_millis = 105000; lightTimerLoop(); CHECK(board.light_state == 0, "zdrzena smycka: zhasne presne po 5 s od LON");
  g_millis = 200000; cli(m, 0, "lon"); g_millis = 202000; lightTimerLoop();
  CHECK(cli(m, 0, "status").rfind("ON 3s bat=", 0) == 0, "STATUS ukazuje zbyvajici cas");
  cli(m, 0, "loff"); lightTimerLoop(); CHECK(!light_timer_armed, "LOFF zrusi odpocet");
  CHECK(cli(m, 0, "chan 00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff").rfind("OK chan ON", 0) == 0, "chan 64 hex");
  uint8_t h1 = m.light_chan.hash[0];
  mesh::GroupChannel out[4];
  CHECK(m.searchChannelsByHash(&h1, out, 4) == 1, "kanal nalezen podle hashe");
  uint8_t h2 = h1 ^ 0xFF;
  CHECK(m.searchChannelsByHash(&h2, out, 4) == 0, "cizi kanal ignorovan");
  cli(m, 0, "chan off");
  CHECK(m.searchChannelsByHash(&h1, out, 4) == 0, "po CHAN OFF kanal neposloucha");

  // --- doba behu ---
  uptime_ms = 0; uptime_last = 0;
  g_millis = 0; uptimeLoop(); g_millis = (uint32_t)((2*1440 + 3*60 + 4) * 60000UL); uptimeLoop();
  CHECK(cli(m, 0, "status").find("up=2d03h04m") != std::string::npos, "up format");

  // --- BATKAL (spolecne s radarem) ---
  board.mv = 3900;
  CHECK(cli(m, 0, "batkal") == "batkal=1.000 bat=3.90V", "BATKAL vychozi");
  CHECK(cli(m, 0, "BATKAL 4,00") == "batkal=1.026 bat=4.00V", "BATKAL podle multimetru");
  CHECK(cli(m, 0, "status").find("bat=4.00V") != std::string::npos, "STATUS s korekci");
  CHECK(cli(m, 0, "batkal 3.0").rfind("Err - BATKAL", 0) == 0, "BATKAL o vic nez 20 % odmitnut");
  CHECK(cli(m, 1800300000, "batkal off") == "batkal=1.000 bat=3.90V", "BATKAL OFF i na dalku (admin)");

  printf("%d kontrol, %d chyb\n", checks, fails);
  return fails ? 1 : 0;
}
