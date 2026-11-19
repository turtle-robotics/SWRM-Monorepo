// Adapted from agfoti04/SWRM_Leds mqtt_test src/main.cpp, commit decd230.
// Keep the lead's WiFiClient + PubSubClient setup and topic/LED convention.
// PubSubClient's synchronous socket waits run in this task, never the motor loop.
#ifdef SWRM_ENABLE_MQTT
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <cstdio>
#include "../include/mqtt_controller.h"
#include "../include/network_config.h"
#include "../include/robot_settings.h"

namespace {
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
std::atomic<bool> brokerOnline{false};
std::atomic<unsigned> latestResult{0}; // Best-effort status, never a motion command.
bool taskStarted=false, wifiAttempted=false, mqttAttempted=false;
uint32_t lastWifiAttempt=0, lastMqttAttempt=0;

void connectionLost() {
    if (brokerOnline.exchange(false)) {
        latestResult.store(0);
        onRobotConnectionLost(); // No I2C from this thread/event callback.
    }
}
const char* commandName(robot::Command cmd) {
    return cmd==robot::Command::square ? "square" : cmd==robot::Command::spin ? "spin" : "stop";
}
void acknowledge(const char* message) {
    mqtt.publish(robot::selected.statusTopic,message); // All PubSubClient calls on this task.
}
}

// EXACT callback signature from the lead's PubSubClient example. PubSubClient
// delivers a complete packet, not AsyncMqttClient-style payload fragments.
void onMessage(char* topic, byte* payload, unsigned int len) {
    const bool retained=mqtt.messageRetained(); // Read BEFORE publish reuses its buffer.
    if (!topic || std::strcmp(topic,robot::selected.commandTopic)) { acknowledge("err:topic"); return; }
    if (!payload) { acknowledge("err:unknown"); return; }
    if (len==2 && !std::memcmp(payload,"on",2)) {
        digitalWrite(network::ledPin,HIGH); acknowledge("ack:on"); return;
    }
    if (len==3 && !std::memcmp(payload,"off",3)) {
        digitalWrite(network::ledPin,LOW); acknowledge("ack:off"); return;
    }
    const auto cmd=robot::parseCommand(reinterpret_cast<const char*>(payload),len);
    if (cmd==robot::Command::none) { acknowledge("err:unknown; expected on,off,square,stop,spin"); return; }
    if (retained && cmd!=robot::Command::stop) { acknowledge("err:retained-start"); return; }
    if (!brokerOnline.load()) { acknowledge("err:connection-lost"); return; }
    const auto result=queueRobotCommand(reinterpret_cast<const char*>(payload),len);
    char response[128];
    std::snprintf(response,sizeof(response),"ack:%s:%s",commandName(cmd),robot::resultName(result));
    acknowledge(response); // QUEUED is distinct from the later guarded result.
}

namespace {
void serviceNetwork() {
    const uint32_t now=millis();
    if (WiFi.status()!=WL_CONNECTED) {
        connectionLost();
        if (!wifiAttempted || now-lastWifiAttempt>=network::wifiRetryMs) {
            wifiAttempted=true; lastWifiAttempt=now;
            WiFi.begin(network::wifiSsid,network::wifiPassword);
        }
        return;
    }
    if (!mqtt.connected()) {
        connectionLost();
        if (!mqttAttempted || now-lastMqttAttempt>=network::mqttRetryMs) {
            mqttAttempted=true; lastMqttAttempt=now;
            // Same clean-session, no-auth mqtt.connect(ROBOT_ID) as the lead.
            if (mqtt.connect(robot::selected.mqttClientId)) {
                if (!mqtt.subscribe(robot::selected.commandTopic,0)) {
                    mqtt.disconnect(); return;
                }
                // A Wi-Fi event can race the blocking connect. Check before
                // enabling delivery; a reconnect never queues any start.
                if (WiFi.status()!=WL_CONNECTED) { mqtt.disconnect(); return; }
                brokerOnline.store(true);
                acknowledge("online");
            }
        }
        return;
    }
    if (!mqtt.loop()) { connectionLost(); return; }
    const unsigned status=latestResult.exchange(0);
    if (status) {
        const auto cmd=static_cast<robot::Command>(status>>8);
        const auto result=static_cast<robot::Result>((status&0xff)-1);
        char response[192];
        std::snprintf(response,sizeof(response),"result:%s:%s",commandName(cmd),robot::resultName(result));
        acknowledge(response);
    }
}
void initializeNetworkTransport() {
    pinMode(network::ledPin,OUTPUT); digitalWrite(network::ledPin,LOW);
    WiFi.mode(WIFI_STA);
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t) {
        if (event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED) connectionLost();
    });
    mqtt.setServer(network::mqttHost,network::mqttPort);
    mqtt.setCallback(onMessage);
    mqtt.setSocketTimeout(network::socketTimeoutSeconds);
    mqtt.setKeepAlive(network::keepAliveSeconds);
}
void networkTask(void*) {
    initializeNetworkTransport();
    for (;;) {
        serviceNetwork();
        vTaskDelay(pdMS_TO_TICKS(network::taskPeriodMs));
    }
}
}

void beginRobotNetwork() {
    if (taskStarted) return;
    // Core 0 runs networking; the normal ESP32 Arduino loop services motors on
    // core 1. Only queueRobotCommand/onRobotConnectionLost cross into movement.
    taskStarted=xTaskCreatePinnedToCore(networkTask,"robot-mqtt",network::taskStackBytes,
        nullptr,1,nullptr,0)==pdPASS;
    if (!taskStarted) Serial.println("ERR: MQTT task creation failed; networking unavailable, no motion started.");
}
void reportRobotCommandResult(robot::Command cmd, robot::Result result) {
    if (brokerOnline.load()) latestResult.store((static_cast<unsigned>(cmd)<<8)|(static_cast<unsigned>(result)+1));
}
#endif
