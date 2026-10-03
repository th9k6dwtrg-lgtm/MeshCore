// Minimalni napodoba prostredi MeshCore/Arduino pro test logiky main.cpp na PC
#pragma once
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <string>
#include <vector>
#define NRF52_POWER_MANAGEMENT 1
#define PUB_KEY_SIZE 32
#define PAYLOAD_TYPE_GRP_TXT 0x05
#define MAX_PACKET_PAYLOAD 184
#define HIGH 1
#define LOW 0
#define MAX_CONCURRENT_ALERTS 4

static uint32_t g_millis = 0;
static inline uint32_t millis() { return g_millis; }
static inline void delay(uint32_t) {}
struct SerialMock { void println(const char*){} void flush(){} } Serial;

namespace mesh {
  struct MainBoard {}; struct Radio {}; struct MillisecondClock {}; struct RNG { uint32_t nextInt(uint32_t a, uint32_t b){ return a; } };
  struct RTCClock { uint32_t t = 1715770351; uint32_t last=0; uint32_t getCurrentTime(){return t;} void setCurrentTime(uint32_t x){t=x;}
    uint32_t getCurrentTimeUnique(){ uint32_t x=getCurrentTime(); if (x<=last) return ++last; return last=x; } };
  struct MeshTables {};
  struct Packet { std::string text; };
  struct GroupChannel { uint8_t hash[1]; uint8_t secret[PUB_KEY_SIZE]; };
  struct Utils {
    static void sha256(uint8_t* hash, size_t hl, const uint8_t* m, int ml){ uint8_t h=0; for(int i=0;i<ml;i++) h=h*31+m[i]; hash[0]=h; }
    static uint8_t hexVal(char c){ if(c>='A'&&c<='F')return c-'A'+10; if(c>='a'&&c<='f')return c-'a'+10; if(c>='0'&&c<='9')return c-'0'; return 0; }
    static bool fromHex(uint8_t* d, int ds, const char* s){ int l=strlen(s); if(l!=ds*2) return false; for(int i=0;i<ds;i++) d[i]=(hexVal(s[2*i])<<4)|hexVal(s[2*i+1]); return true; }
  };
}

struct BoardMock {
  uint32_t light_state = 0; uint16_t mv = 3900; bool ext = false; int writes = 0;
  void setGpio(uint32_t v){ light_state = v & 1; writes++; }
  uint32_t getGpio(){ return light_state; }
  uint16_t getBattMilliVolts(){ return mv; }
  bool isExternalPowered(){ return ext; }
} board;
struct RadioMock { float getLastRSSI(){return -61;} float getLastSNR(){return 9.25f;} } radio_driver;

struct NodePrefs { char node_name[32]; uint8_t path_hash_mode = 1; };
struct MinMaxAvg {};

struct Sent { std::string text; uint32_t delay; uint32_t ts; };
static std::vector<Sent> g_sent;         // zpravy do kanalu
static std::vector<std::string> g_alerts; // zarazena prima upozorneni

class SensorMesh {
public:
  enum AlertPriority { LOW_PRI_ALERT, HIGH_PRI_ALERT };
  struct Trigger { char text[MAX_PACKET_PAYLOAD]; Trigger(){text[0]=0;} bool isTriggered() const { return text[0]!=0; } };
  NodePrefs prefs; mesh::RTCClock rtc; mesh::RNG rng;
  Trigger* queue[MAX_CONCURRENT_ALERTS]; int nq = 0;
  SensorMesh(mesh::MainBoard&, mesh::Radio&, mesh::MillisecondClock&, mesh::RNG&, mesh::RTCClock&, mesh::MeshTables&) { strcpy(prefs.node_name, "zahrada-svetlo"); }
  NodePrefs* getNodePrefs(){ return &prefs; }
  mesh::RTCClock* getRTCClock(){ return &rtc; }
  mesh::RNG* getRNG(){ return &rng; }
  // stejna logika jako SensorMesh::alertIf (zarazeni / zruseni)
  void alertIf(bool cond, Trigger& t, AlertPriority, const char* text) {
    if (cond) { if (!t.isTriggered() && nq < MAX_CONCURRENT_ALERTS) { strncpy(t.text, text, sizeof(t.text)-1); t.text[sizeof(t.text)-1]=0; queue[nq++]=&t; g_alerts.push_back(text); } }
    else if (t.isTriggered()) { t.text[0]=0; int i=0; while(i<nq && queue[i]!=&t) i++; if(i<nq){ nq--; for(;i<nq;i++) queue[i]=queue[i+1]; } }
  }
  void alertsDone(){ nq = 0; }   // simulace: fronta odeslana
  mesh::Packet* createGroupDatagram(uint8_t, const mesh::GroupChannel&, const uint8_t* d, size_t l){ auto p = new mesh::Packet(); uint32_t ts; memcpy(&ts,d,4); p->text = std::string((const char*)d+5, l-5) + "|" + std::to_string(ts); return p; }
  void sendFlood(mesh::Packet* p, uint32_t delay, uint8_t){ size_t k=p->text.rfind('|'); g_sent.push_back({p->text.substr(0,k), delay, (uint32_t)std::stoul(p->text.substr(k+1))}); delete p; }
  virtual ~SensorMesh(){}
protected:
  virtual void onSensorDataRead() = 0;
  virtual int querySeriesData(uint32_t, uint32_t, MinMaxAvg[], int) = 0;
  virtual bool handleCustomCommand(uint32_t, char*, char*) { return false; }
  virtual int searchChannelsByHash(const uint8_t*, mesh::GroupChannel[], int) { return 0; }
  virtual void onGroupDataRecv(mesh::Packet*, uint8_t, const mesh::GroupChannel&, uint8_t*, size_t) {}
};
