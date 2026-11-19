#define SWRM_ENABLE_MQTT 1
#include "../../legacy-motor-test/src/main.cpp"
#include "../../legacy-motor-test/src/mqtt_controller.cpp"
#include <cassert>
#include <iostream>

std::string lastPublished() {
    const auto& bytes=wifiClient.packets.back();
    assert((bytes[0]&0xf0)==0x30);
    unsigned p=1; while (bytes[p++]&0x80) {}
    const unsigned len=unsigned(bytes[p])*256+bytes[p+1]; p+=2;
    assert(std::string(bytes.begin()+p,bytes.begin()+p+len)==robot::selected.statusTopic);
    return std::string(bytes.begin()+p+len,bytes.end());
}
void receive(const std::string& payload,bool retained=false,const char* topic=robot::selected.commandTopic) {
    // Genuine MQTT packet through REAL PubSubClient v2.8; callback payload is
    // intentionally nonterminated and metadata comes from its actual header.
    const std::string name=topic;
    unsigned remaining=2+unsigned(name.size()+payload.size());
    wifiClient.incoming.push_back(retained ? 0x31 : 0x30);
    do {
        unsigned digit=remaining%128; remaining/=128;
        wifiClient.incoming.push_back(uint8_t(digit|(remaining ? 0x80 : 0)));
    } while (remaining);
    wifiClient.incoming.push_back(uint8_t(name.size()>>8));
    wifiClient.incoming.push_back(uint8_t(name.size()));
    for (char c : name+payload) wifiClient.incoming.push_back(uint8_t(c));
    for (unsigned tick=0;tick<8 && !wifiClient.incoming.empty();++tick) serviceNetwork();
    assert(wifiClient.incoming.empty());
}
void commandResult() { updateRobotMovement(); serviceNetwork(); }
void noResume() {
    assert(!running && !enabled && !squarePattern.active && !spinActive);
    // Drain a queued disconnect stop before checking future inactivity.
    updateRobotMovement();
    const auto writes=Wire.writes.size(); hostTime+=10000; updateRobotMovement();
    assert(Wire.writes.size()==writes);
}
int main() {
    setup(); assert(hostTaskCreates==1 && hostCreatedTask==networkTask && Wire.writes.empty());
    beginRobotNetwork(); assert(hostTaskCreates==1); // One client owner/task.
    initializeNetworkTransport();
    assert(hostLed==0 && hostLedPin==2 && !running);
    serviceNetwork(); assert(WiFi.begins==1 && !running && Wire.writes.empty());
    hostTime+=9999; serviceNetwork(); assert(WiFi.begins==1);
    assert(!std::strcmp(WiFi.ssid,"TAMU_IoT") && !std::strcmp(WiFi.password,""));
    WiFi.state=WL_CONNECTED; serviceNetwork();
    assert(brokerOnline.load() && wifiClient.connections==1 && lastPublished()=="online");
    assert(!std::strcmp(wifiClient.host,"10.251.20.11") && wifiClient.port==1883);
    // Original connect(ID) is still a clean session. Decode CONNECT packet ID.
    const auto& connect=wifiClient.packets.front();
    assert(connect[0]==0x10 && (connect[9]&2));
    assert(std::string(connect.begin()+14,connect.end())==robot::selected.mqttClientId);
    const auto bootWrites=Wire.writes.size();
    receive("square"); assert(lastPublished()=="ack:square:QUEUED");
    assert(Wire.writes.size()==bootWrites && !running);
    commandResult(); assert(lastPublished().find("successful init required")!=std::string::npos);
    assert(Wire.writes.size()==bootWrites); noResume();
    assert(initializeRobotMovement());

    // LED compatibility and exact routing, including binary/NUL/whitespace.
    auto writes=Wire.writes.size();
    receive("on"); assert(lastPublished()=="ack:on" && hostLed==1);
    receive("off"); assert(lastPublished()=="ack:off" && hostLed==0);
    for (const auto& bad : {std::string(""),std::string("Square"),std::string("spin "),
         std::string("square\0",7),std::string("on\0off",6),std::string("stop square")}) {
        receive(bad); assert(lastPublished().find("err:unknown")==0);
        updateRobotMovement(); assert(!running && hostLed==0 && Wire.writes.size()==writes);
    }
    receive("square",false,"robots/other/cmd"); assert(lastPublished()=="err:topic" && !running);
    // The real library drops oversized complete packets; never dispatch a prefix.
    const auto published=wifiClient.packets.size(); receive(std::string(300,'x'));
    assert(wifiClient.packets.size()==published && !running && Wire.writes.size()==writes);
    // Retained flag accessor is checked at callback entry before ACK publishing.
    receive("square",true); assert(lastPublished()=="err:retained-start");
    receive("spin",true); assert(lastPublished()=="err:retained-start");
    updateRobotMovement(); assert(!running && Wire.writes.size()==writes);
    receive("square"); assert(!running && Wire.writes.size()==writes);
    commandResult(); assert(running && squarePattern.active && lastPublished()=="result:square:STARTED");
    receive("spin"); commandResult(); assert(lastPublished().find("ERR: busy")!=std::string::npos && !spinActive);
    receive("stop",true); commandResult(); noResume();
    assert(lastPublished()=="result:stop:STOPPED; disarmed");
    receive("square"); receive("stop"); commandResult(); noResume();
    receive("spin"); commandResult(); assert(spinActive && running);

    // Simulate Wi-Fi disconnect event and MQTT disconnect independently. These
    // callbacks must queue stops without writing motors in the network context.
    writes=Wire.writes.size(); WiFi.loseConnection();
    assert(Wire.writes.size()==writes && running && !brokerOnline.load());
    updateRobotMovement(); noResume();
    WiFi.state=WL_CONNECTED; wifiClient.stop(); hostTime+=2000; serviceNetwork();
    assert(brokerOnline.load()); noResume();
    receive("square",true); commandResult(); noResume();
    receive("square"); commandResult(); assert(squarePattern.active);
    writes=Wire.writes.size(); wifiClient.stop(); serviceNetwork();
    assert(Wire.writes.size()==writes); updateRobotMovement(); noResume();
    hostTime+=2000; serviceNetwork(); assert(brokerOnline.load()); noResume();

    // Failed connections retry once per deadline, no reconnect while-loop.
    wifiClient.stop(); wifiClient.canConnect=false; hostTime+=2000; serviceNetwork();
    auto attempts=wifiClient.connections; serviceNetwork(); assert(wifiClient.connections==attempts);
    hostTime+=1999; serviceNetwork(); assert(wifiClient.connections==attempts);
    hostTime+=1; serviceNetwork(); assert(wifiClient.connections==attempts+1);
    wifiClient.canConnect=true; hostTime+=2000; serviceNetwork(); noResume();

    // Simulate the two-task schedule during a real PubSubClient partial-packet
    // socket wait: the motor loop can still consume serial stop before it ends.
    receive("spin"); commandResult(); assert(spinActive);
    const auto waitBegan=hostTime;
    hostNetworkWaitHook=[] {
        if (spinActive) { for (char c : std::string("stop\n")) Serial.input.push_back(c); }
        loop();
    };
    wifiClient.incoming={0x30}; serviceNetwork(); hostNetworkWaitHook=nullptr;
    assert(hostTime-waitBegan>=1000); noResume();
    // Latching a movement fault cannot be cleared by commands or reconnect.
    Wire.ack=false; receive("square"); commandResult(); assert(broken && !running);
    Wire.ack=true; hostTime=lastRetry+legacy::retryIntervalMs; updateRobotMovement();
    receive("spin"); commandResult(); assert(broken && !running && lastPublished().find("FAULT: latched")!=std::string::npos);
    std::cout << "PASS: MQTT robot " << robot::selected.identity << " real PubSubClient packets/callback, retained-bit patch, LED on/off, exact payload/topic checks, oversize rejection, init/fault guards, queued results, stop priority, disconnect/reconnect and scheduler independence during socket waits.\n";
}
