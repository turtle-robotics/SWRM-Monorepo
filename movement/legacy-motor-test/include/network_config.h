#pragma once

// Connection values from agfoti04/SWRM_Leds mqtt_test at decd230.
// The lead owns this file and src/mqtt_controller.cpp; no motor settings here.
namespace network {
constexpr const char* wifiSsid = "TAMU_IoT";
constexpr const char* wifiPassword = "";
constexpr const char* mqttHost = "10.251.169.194"; // Lead's laptop; update when its IP changes.
constexpr unsigned mqttPort = 1883;
constexpr unsigned ledPin = 2; // Original on/off onboard LED, not the HUB75 panel.
constexpr unsigned wifiRetryMs = 10000, mqttRetryMs = 2000;
constexpr unsigned socketTimeoutSeconds = 1, keepAliveSeconds = 5;
constexpr unsigned taskStackBytes = 6144, taskPeriodMs = 2;
}
