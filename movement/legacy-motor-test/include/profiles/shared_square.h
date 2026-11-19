#pragma once

namespace robot {
struct LegTuning {
    float timeScale = 1;
    float trim[3] = {1,1,1};
};
struct SquareProfile {
    const char* revision;
    unsigned sideMs;
    int speed;
    unsigned pauseMs;
    LegTuning legs[4]; // Body-relative forward, right, backward, left.
};
// Robot A, USB-only, November 17. Five completed cycles; ~1cm accumulating
// right drift/cycle accepted for now. B inherits this WITHOUT physical validation.
constexpr SquareProfile sharedSquare = {
    "robot-a-usb-v14-20261117-p1", 1500, 20, 1000,
    {{1,{1.04f,1,1.10f}}, {1,{0.90f,0.95f,0.90f}},
     {1,{1,1,1}}, {1,{0.90f,0.95f,1}}}
};
struct Settings {
    const char* identity;
    const char* mqttClientId; // Lead's connection code may use this unique ID.
    const char* commandTopic;
    const char* statusTopic;
    bool inheritsUnvalidated;
    SquareProfile square; // Value copy: each robot can override its own profile.
    int spinSpeed; // Pulses/10ms reference; provisional, NOT physically tested.
};
}
