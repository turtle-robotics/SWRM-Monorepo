// Adapted from agfoti04/SWRM_Leds mqtt_test src/main.cpp, commit decd230.
// Identical example moved to sample_code/sameple_MQTT.cpp at 4ea371a.
// Keep the lead's WiFiClient + PubSubClient setup and topic/LED convention.
// PubSubClient's synchronous socket waits run in this task, never the motor
// loop.
#ifdef SWRM_ENABLE_MQTT
#include "../include/mqtt_controller.h"
#include "../include/network_config.h"
#include "../include/robot_settings.h"
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <atomic>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace {
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
std::atomic<bool> brokerOnline{false};
std::atomic<unsigned> latestResult{
    0}; // Best-effort command result, never a motion command.
QueueHandle_t stateQueue = nullptr, faultQueue = nullptr;
robot::Status cachedState, cachedFault;
bool stateDirty = false, faultDirty = false;
// Only the movement loop calls reportRobotState; only the network task owns
// the caches/client. Queues copy snapshots without cross-task motor-state
// reads.
bool faultWasReported = false;
bool taskStarted = false, wifiAttempted = false, mqttAttempted = false;
uint32_t lastWifiAttempt = 0, lastMqttAttempt = 0;

void connectionLost() {
  if (brokerOnline.exchange(false)) {
    latestResult.store(0);
    onRobotConnectionLost(); // No I2C from this thread/event callback.
  }
}
void acknowledge(const char *message) {
  mqtt.publish(robot::selected.statusTopic,
               message); // All PubSubClient calls on this task.
}
const char *stateName(const robot::Status &s) {
  if (s.fault)
    return "fault";
  if (s.running)
    return s.spin ? "spin" : "moving";
  if (s.paused)
    return "paused";
  if (s.armed)
    return "armed";
  return s.ready ? "ready" : "not-ready";
}
bool publishState(const char *prefix, const robot::Status &s) {
  char
      response[224]; // Fits the 256-byte MQTT packet with the configured topic.
  const int size = std::snprintf(
      response, sizeof(response),
      "%s:%s ready=%d configured=%d armed=%d running=%d fault=%d retries=%u "
      "err=%u reg=%u firstErr=%u firstReg=%u leg=%u square=%d pause=%d spin=%d "
      "cycles=%lu seq=%lu",
      prefix, stateName(s), s.ready, s.configured, s.armed, s.running, s.fault,
      s.retries, unsigned(s.error), unsigned(s.reg), unsigned(s.firstError),
      unsigned(s.firstReg), s.faultLeg, s.square, s.paused, s.spin,
      (unsigned long)s.cycles, (unsigned long)s.sequence);
  if (size < 0 || unsigned(size) >= sizeof(response))
    return mqtt.publish(robot::selected.statusTopic, "err:status-too-long");
  return mqtt.publish(robot::selected.statusTopic, response);
}
} // namespace

// EXACT callback signature from the lead's PubSubClient example. PubSubClient
// delivers a complete packet, not AsyncMqttClient-style payload fragments.
void onMessage(char *topic, byte *payload, unsigned int len) {
  const bool retained =
      mqtt.messageRetained(); // Read BEFORE publish reuses its buffer.
  if (!topic || std::strcmp(topic, robot::selected.commandTopic)) {
    acknowledge("err:topic");
    return;
  }
  if (!payload) {
    acknowledge("err:unknown");
    return;
  }
  if (len == 2 && !std::memcmp(payload, "on", 2)) {
    digitalWrite(network::ledPin, HIGH);
    acknowledge("ack:on");
    return;
  }
  if (len == 3 && !std::memcmp(payload, "off", 3)) {
    digitalWrite(network::ledPin, LOW);
    acknowledge("ack:off");
    return;
  }
  const auto cmd =
      robot::parseCommand(reinterpret_cast<const char *>(payload), len);
  if (cmd == robot::Command::none) {
    acknowledge("err:unknown; expected on,off,square,stop,spin,init,status");
    return;
  }
  if (retained && cmd == robot::Command::init) {
    acknowledge("err:retained-init");
    return;
  }
  if (retained &&
      (cmd == robot::Command::square || cmd == robot::Command::spin)) {
    acknowledge("err:retained-start");
    return;
  }
  if (!brokerOnline.load()) {
    acknowledge("err:connection-lost");
    return;
  }
  const auto result =
      queueRobotCommand(reinterpret_cast<const char *>(payload), len);
  char response[128];
  std::snprintf(response, sizeof(response), "ack:%s:%s",
                robot::commandName(cmd), robot::resultName(result));
  acknowledge(response); // QUEUED is distinct from the later guarded result.
}

namespace {
// Same connection calls as the lead's connectWiFi/connectMQTT. Each call
// makes at most one paced attempt; no retry loop can hold up motor stopping.
bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED)
    return true;
  const uint32_t now = millis();
  connectionLost();
  if (!wifiAttempted || now - lastWifiAttempt >= network::wifiRetryMs) {
    wifiAttempted = true;
    lastWifiAttempt = now;
    Serial.printf("Connecting to WiFi: %s\n", network::wifiSsid);
    WiFi.begin(network::wifiSsid, network::wifiPassword);
  }
  return false;
}
bool connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    connectionLost();
    return false;
  }
  const uint32_t now = millis();
  // A Wi-Fi event can invalidate the session before the TCP client notices.
  // Close that stale transport on its owning task and establish a fresh session.
  if (!brokerOnline.load() && mqtt.connected())
    mqtt.disconnect();
  if (!mqtt.connected()) {
    connectionLost();
    if (!mqttAttempted || now - lastMqttAttempt >= network::mqttRetryMs) {
      mqttAttempted = true;
      lastMqttAttempt = now;
      Serial.printf("Connecting to MQTT: %s:%u client=%s\n", network::mqttHost,
                    network::mqttPort, robot::selected.mqttClientId);
      // Same clean-session, no-auth mqtt.connect(ROBOT_ID) as the lead.
      if (mqtt.connect(robot::selected.mqttClientId)) {
        if (!mqtt.subscribe(robot::selected.commandTopic, 0)) {
          mqtt.disconnect();
          Serial.println("MQTT subscription failed; will reconnect.");
          return false;
        }
        // A Wi-Fi event can race the blocking connect. Check before
        // enabling delivery; a reconnect never queues any start.
        if (WiFi.status() != WL_CONNECTED) {
          mqtt.disconnect();
          return false;
        }
        brokerOnline.store(true);
        Serial.println("MQTT connected; awaiting explicit commands.");
        acknowledge("online");
        // Request a fresh post-disconnect loop snapshot. Never expose
        // a cached pre-stop "running/ready" state as the current state.
        stateDirty = false;
        queueRobotCommand("status", 6);
      } else
        Serial.printf("MQTT connect failed, state=%d; will retry.\n", mqtt.state());
    }
  }
  return brokerOnline.load() && mqtt.connected();
}
void serviceNetwork() {
  if (xQueueReceive(stateQueue, &cachedState, 0) == pdPASS)
    stateDirty = true;
  if (xQueueReceive(faultQueue, &cachedFault, 0) == pdPASS)
    faultDirty = true;
  if (!connectWiFi() || !connectMQTT())
    return;
  if (!mqtt.loop()) {
    connectionLost();
    return;
  }
  if (faultDirty && publishState("event", cachedFault))
    faultDirty = false;
  if (stateDirty && publishState("state", cachedState))
    stateDirty = false;
  const unsigned status = latestResult.exchange(0);
  if (status) {
    const auto cmd = static_cast<robot::Command>(status >> 8);
    const auto result = static_cast<robot::Result>((status & 0xff) - 1);
    char response[192];
    std::snprintf(response, sizeof(response), "result:%s:%s",
                  robot::commandName(cmd), robot::resultName(result));
    acknowledge(response);
  }
}
void initializeNetworkTransport() {
  pinMode(network::ledPin, OUTPUT);
  digitalWrite(network::ledPin, LOW);
  WiFi.mode(WIFI_STA);
  // Equivalent to the lead's setup: identity/topics, MAC, station mode,
  // server and exact callback. Connection attempts run on this task below.
  Serial.printf("MQTT setup: robot=%s MAC=%s WiFi=%s broker=%s:%u\n",
                robot::selected.mqttClientId, WiFi.macAddress().c_str(),
                network::wifiSsid, network::mqttHost, network::mqttPort);
  Serial.printf("MQTT topics: cmd=%s status=%s\n", robot::selected.commandTopic,
                robot::selected.statusTopic);
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
      connectionLost();
  });
  mqtt.setServer(network::mqttHost, network::mqttPort);
  mqtt.setCallback(onMessage);
  mqtt.setSocketTimeout(network::socketTimeoutSeconds);
  mqtt.setKeepAlive(network::keepAliveSeconds);
}
void networkTask(void *) {
  initializeNetworkTransport();
  for (;;) {
    serviceNetwork();
    vTaskDelay(pdMS_TO_TICKS(network::taskPeriodMs));
  }
}
} // namespace

void beginRobotNetwork() {
  if (taskStarted)
    return;
  if (!stateQueue)
    stateQueue = xQueueCreate(1, sizeof(robot::Status));
  if (!faultQueue)
    faultQueue = xQueueCreate(1, sizeof(robot::Status));
  if (!stateQueue || !faultQueue) {
    if (stateQueue)
      vQueueDelete(stateQueue);
    if (faultQueue)
      vQueueDelete(faultQueue);
    stateQueue = faultQueue = nullptr;
    Serial.println(
        "ERR: MQTT status queue allocation failed; networking unavailable.");
    return;
  }
  // Core 0 runs networking; the normal ESP32 Arduino loop services motors on
  // core 1. Only queueRobotCommand/onRobotConnectionLost cross into movement.
  taskStarted = xTaskCreatePinnedToCore(networkTask, "robot-mqtt",
                                        network::taskStackBytes, nullptr, 1,
                                        nullptr, 0) == pdPASS;
  if (!taskStarted)
    Serial.println("ERR: MQTT task creation failed; networking unavailable, no "
                   "motion started.");
}
void reportRobotCommandResult(robot::Command cmd, robot::Result result) {
  if (brokerOnline.load())
    latestResult.store((static_cast<unsigned>(cmd) << 8) |
                       (static_cast<unsigned>(result) + 1));
}

// Called only by the movement loop. Keep a fault snapshot independently so a
// successful init cannot overwrite its evidence before the network publishes.
void reportRobotState(const robot::Status &status) {
  // Boot initialization can report before networking/queues exist. Its final
  // snapshot is sent after beginRobotNetwork; do not consume the fault edge yet.
  if (!stateQueue || !faultQueue)
    return;
  xQueueOverwrite(stateQueue, &status);
  if (status.fault && !faultWasReported && faultQueue)
    xQueueOverwrite(faultQueue, &status);
  faultWasReported = status.fault;
}
#endif
