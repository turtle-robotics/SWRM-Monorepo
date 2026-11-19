#pragma once
#include "Client.h"
#include <functional>
constexpr unsigned WIFI_STA=1, WL_CONNECTED=3, ARDUINO_EVENT_WIFI_STA_DISCONNECTED=5;
using WiFiEvent_t=unsigned;
struct WiFiEventInfo_t {};
struct HostWiFi {
    unsigned state=0, begins=0;
    const char* ssid=nullptr; const char* password=nullptr;
    std::function<void(WiFiEvent_t,WiFiEventInfo_t)> event;
    unsigned status() const { return state; }
    void mode(unsigned) {}
    void begin(const char* name,const char* pass) { ++begins; ssid=name; password=pass; }
    void onEvent(std::function<void(WiFiEvent_t,WiFiEventInfo_t)> callback) { event=callback; }
    void loseConnection() { state=0; if (event) event(ARDUINO_EVENT_WIFI_STA_DISCONNECTED,{}); }
};
inline HostWiFi WiFi;
class WiFiClient : public Client {
public:
    bool online=false, canConnect=true;
    unsigned connections=0;
    const char* host=nullptr; uint16_t port=0;
    std::deque<uint8_t> incoming;
    std::vector<std::vector<uint8_t>> packets;
    int connect(const char* name,uint16_t number) override {
        ++connections; host=name; port=number; online=canConnect;
        if (online) incoming={0x20,2,0,0}; // MQTT CONNACK, clean session, accepted.
        return int(online);
    }
    int connect(IPAddress,uint16_t number) override { return connect("ip",number); }
    uint8_t connected() override { return online; }
    int available() override { return int(incoming.size()); }
    int read() override { if (incoming.empty()) return -1; const int value=incoming.front(); incoming.pop_front(); return value; }
    std::size_t write(uint8_t value) override { packets.push_back({value}); return 1; }
    std::size_t write(const uint8_t* data,std::size_t len) override {
        packets.emplace_back(data,data+len);
        if (len==2 && data[0]==0xc0) { incoming.push_back(0xd0); incoming.push_back(0); } // Broker PINGRESP.
        return len;
    }
    void stop() override { online=false; incoming.clear(); }
    void flush() override {}
};
