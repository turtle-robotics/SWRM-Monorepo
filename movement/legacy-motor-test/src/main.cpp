/*
V5: relative timed movement using the bench-verified speed interface.
Current boot defaults: motor type 1, encoder polarity 0, block speed writes.
The user verified independent M1/M2/M3, reverse M2 and M2 at speed 30 with
these settings. Geometry below remains provisional. Historical context follows:
Motor comparison V4: original speed interface plus refreshed open-loop PWM
==================================================
The original demo starts motion automatically and repeats it. This program keeps
its important driver values, but requires init -> arm -> a test command and runs
each test only once. Speed mode uses register 51; explicit PWM mode uses
register 31 with repeated full snapshots. Neither mode sends periodic idle
writes.

EXACT VALUES FROM THE ORIGINAL SOURCE:
  device 0x34; SDA=21; SCL=22; 100kHz; motor-type register 20 receives 1;
  encoder-polarity register 21 receives 1; speed register 51 gets four bytes;
  driveRobot(0,1,0) produces about +58,-58,0,0, for 3000ms, then speed zeros.

The original calls type 1 "N20"; current Hiwonder docs call N20 type 2. Here 1
is an explicit COMPATIBILITY TEST, not a claim that the docs are wrong for all
boards. Speed values are pulses per 10ms, not percentages. +58 can be fast.
Start with a single wheel at +10 and keep every wheel raised for this bench
experiment.

Power-cycle the USB-only setup before uploading; this program cannot guarantee a
stop from a previously powered driver if communication is broken. It does not
automatically start motors on boot, init, help or status. `!` sends a software
stop, but physical power removal is the fallback if that write fails.
*/
#include <Arduino.h>
#include <Wire.h>
#include <cmath> // Dot-product wheel mixing uses sin/cos; inputs are checked for NaN/Inf.
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <initializer_list> // Enables iterating over the three candidate motor values.

namespace legacy {
constexpr uint8_t address = 0x34; // Original Hiwonder device address.
constexpr int sda = 21, scl = 22; // Original ESP32 GPIO signal pins.
constexpr unsigned busHz = 100000, baud = 115200;
constexpr uint8_t typeReg = 20, polarityReg = 21, speedReg = 51;
constexpr uint8_t type = 1,
                  polarity = 0; // Bench-verified on this robot: independent
                                // motion and positive counts.
constexpr int maxCommand =
    58; // Original active movement's largest wheel command.
constexpr unsigned maxDurationMs =
    3000; // Original three-second movement interval.
constexpr unsigned transactionTimeoutMs =
    50; // Arduino Wire default; original had no override.
constexpr unsigned settingDelayMs =
    10; // Original pause after each configuration write.
constexpr unsigned retryIntervalMs =
    250; // Our bounded attempts to deliver a failed stop.
constexpr unsigned maxStopRetries =
    4; // Then remain faulted/quiet; do not flood the bus forever.
constexpr int maxPwmDuty =
    50; // Chosen raised-wheel bench cap; protocol allows +/-100.
constexpr unsigned pwmRefreshMs =
    20; // Chosen 50Hz refresh for continuous PWM control.
} // namespace legacy

// State starts explicitly disabled. Configuration and arming are separate
// steps.
bool configured = false, enabled = false, running = false, broken = false;
// V3: pending, operator-selected comparisons. Edits are allowed only while idle
// and disarmed, and invalidate initialization. No edit automatically runs
// motors.
uint8_t selectedType = legacy::type, selectedPolarity = legacy::polarity;
bool singleWrites =
    false;            // Full four-channel snapshots, verified on this robot.
bool pwmMode = false; // V4 alternative: register 31 open-loop duty, not encoder
                      // speed regulation.
int activeOutputs[3] =
    {}; // Snapshot for PWM refresh, reset to zeros on every stop.
uint32_t lastPwmRefresh = 0;
unsigned selectedClock = legacy::busHz;
unsigned writeGapMs =
    0; // Optional settling pause AFTER a transaction, not between its bytes.
uint32_t began = 0, testMs = 0, lastReport = 0, lastRetry = 0;
unsigned stopRetries = 0; // Zero means no failed-stop retry is pending.
uint8_t lastError = 0, lastReg = 0; // Last failed write, retained for status.
char buffer[96] = {}; // Fixed command storage; no heap allocation needed.
unsigned used = 0;
bool discard =
    false; // Reject overlong/unsupported input through the next newline.

// Provisional rolling-axis directions, NOT measured wheel locations. For a
// symmetric kiwi chassis these axes are 120 degrees apart. A positive command
// may roll a mounted wheel along or against its assumed axis, hence wheelSign.
// `wheel CHANNEL ANGLE SIGN` edits these assumptions without changing wiring.
float wheelAngle[3] = {90, 210, 330};
int wheelSign[3] = {1, 1, 1};

// Project desired robot-relative translation onto each wheel's rolling axis:
// wheel = cos(angle)*X + sin(angle)*Y. X is forward, Y is left. Defaults give
// M1=Y, M2=-0.866X-0.5Y, M3=0.866X-0.5Y. Those numbers come from 120-degree
// geometry, not tuned constants. Scale oversized diagonals together to retain
// the ratios. Output units are encoder pulses/10ms, as in the working motor
// test.
bool mixDirection(float x, float y, int speed, int (&out)[3]) {
  for (int &value : out)
    value = 0;
  if (!std::isfinite(x) || !std::isfinite(y) || std::fabs(x) > 1 ||
      std::fabs(y) > 1 || speed < 1 || speed > 30)
    return false; // 30 is the successfully tested speed, not a motor rating.
  float mix[3] = {};
  float peak = 1; // Scale down only; small inputs remain small.
  constexpr float radiansPerDegree = 3.14159265358979323846f / 180;
  for (unsigned i = 0; i < 3; ++i) {
    const float angle = wheelAngle[i] * radiansPerDegree;
    mix[i] = (std::cos(angle) * x + std::sin(angle) * y) * wheelSign[i];
    if (std::fabs(mix[i]) > peak)
      peak = std::fabs(mix[i]);
  }
  for (unsigned i = 0; i < 3; ++i)
    out[i] = int(std::lround(speed * mix[i] / peak));
  return true;
}

// Same register-plus-byte-array transaction as the original, but report every
// write result. Numeric result 0 means acknowledgment, not verified wheel
// motion.
bool writeRegister(uint8_t reg, const uint8_t *data, unsigned length,
                   bool verbose = true) {
  Wire.beginTransmission(legacy::address);
  Wire.write(reg);
  for (unsigned i = 0; i < length; ++i)
    Wire.write(data[i]);
  const uint8_t result = Wire.endTransmission();
  // Print the actual data too, not just the caller's intended wheel values.
  if (verbose || result) {
    Serial.printf("WRITE register=%u bytes=%u result=%u data=", unsigned(reg),
                  length, unsigned(result));
    for (unsigned i = 0; i < length; ++i)
      Serial.printf("%02X ", unsigned(data[i]));
    Serial.println("");
  }
  if (writeGapMs)
    delay(writeGapMs);
  if (result) {
    lastError = result;
    lastReg = reg;
  }
  return result == 0;
}

// Same four logical channel requests as the original setMotorSpeeds(), now
// transmitted separately. Only channels 1..3 can receive nonzero test values.
bool sendSpeeds(int a = 0, int b = 0, int c = 0, bool quietRefresh = false) {
  b *= -1; // Account for polarity of m2 in the motor driver

  if (pwmMode) {
    // Open-loop PWM uses an entirely new full four-channel snapshot each
    // time. No speed-register write follows it, so we do not switch back
    // into speed regulation at a stop. No encoder counts are used here.
    const uint8_t data[4] = {uint8_t(int8_t(a)), uint8_t(int8_t(b)),
                             uint8_t(int8_t(c)), 0};
    return writeRegister(31, data, 4, !quietRefresh);
  }
  if (!singleWrites) {
    const uint8_t data[4] = {uint8_t(int8_t(a)), uint8_t(int8_t(b)),
                             uint8_t(int8_t(c)), 0};
    return writeRegister(legacy::speedReg, data, 4);
  }
  // V2 channel-isolation test: Hiwonder's official Pi example addresses one
  // motor with register 50+motor_id and a ONE-byte payload. Thus 51=M1,
  // 52=M2, 53=M3, 54=M4. These are register numbers, not I2C device addresses.
  // https://wiki.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/4_Raspberry%20Pi_formatted.html
  // The previous four-byte block was acknowledged but physically activated
  // extra motors. Its cause is not established. This bypasses its packing /
  // auto-increment assumptions rather than claiming the board is now verified.
  const int values[4] = {a, b, c, 0};
  bool cleared = true;
  // Clear every unselected channel FIRST. Always try every zero write even
  // if an earlier zero fails, and never energize a channel after such a
  // failure.
  for (unsigned i = 0; i < 4; ++i)
    if (values[i] == 0) {
      const uint8_t zero = 0;
      if (!writeRegister(uint8_t(legacy::speedReg + i), &zero, 1))
        cleared = false;
    }
  if (!cleared)
    return false;
  for (unsigned i = 0; i < 4; ++i)
    if (values[i] != 0) {
      const uint8_t value = uint8_t(int8_t(values[i]));
      if (!writeRegister(uint8_t(legacy::speedReg + i), &value, 1))
        return false;
    }
  return true;
}

// A normal stop always disables the next motion. Failure latches a fault and
// schedules a few zero-only retries; it never permits continuing a motor test.
void stopTest(const char *reason) {
  running = enabled = false;
  for (int &value : activeOutputs)
    value = 0;
  if (!sendSpeeds()) {
    broken = true;
    stopRetries = legacy::maxStopRetries;
    lastRetry = millis();
    Serial.println("FAULT: output-zero write failed. Use physical power "
                   "removal if needed.");
  } else
    stopRetries = 0;
  Serial.println(reason);
}

// Test the original numeric settings step by step. Stop at the first failure,
// preserving which register failed. This is not an address-only connectivity
// check.
void initialize() {
  configured = enabled = running = false;
  broken = false;
  stopRetries = 0;
  lastError = lastReg = 0;
  Serial.printf("INIT: type=%u polarity=%u transport=%s clock=%uHz gap=%ums\n",
                unsigned(selectedType), unsigned(selectedPolarity),
                singleWrites ? "single" : "block", selectedClock, writeGapMs);
  Wire.setClock(selectedClock);
  // In PWM mode first request zero duty before editing motor settings. This
  // requests leaving speed control; it is not a documented PID-state reset.
  if (pwmMode && !sendSpeeds()) {
    broken = true;
    stopTest("FAULT: initial PWM zero.");
    return;
  }
  const uint8_t type = selectedType, polarity = selectedPolarity;
  if (!writeRegister(legacy::typeReg, &type, 1)) {
    broken = true;
    stopTest("FAULT: original motor-type write failed.");
    return;
  }
  delay(legacy::settingDelayMs);
  if (!writeRegister(legacy::polarityReg, &polarity, 1)) {
    broken = true;
    stopTest("FAULT: original polarity write failed.");
    return;
  }
  delay(legacy::settingDelayMs);
  if (!sendSpeeds()) {
    broken = true;
    stopRetries = legacy::maxStopRetries;
    lastRetry = millis();
    return;
  }
  configured = true;
  Serial.println(
      "READY: selected settings acknowledged; DISARMED. No motion verified.");
}

// Read-only diagnostics: selecting a register writes only its address, never a
// motor value. Require idle/disarmed so these blocking reads cannot delay a
// stop. Register layout follows Hiwonder's register table and PX4's HiwonderEMM
// driver: reg 0 = two little-endian supply millivolt bytes; reg 60 = four
// signed counts.
// https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/hiwonder_emm/HiwonderEMM.cpp
bool readDiagnostic(uint8_t reg, uint8_t *data, uint8_t length) {
  if (enabled || running || stopRetries) {
    Serial.println("ERR: diagnostics require idle/disarmed; stop first.");
    return false;
  }
  Wire.beginTransmission(legacy::address);
  Wire.write(reg);
  const uint8_t result =
      Wire.endTransmission(false); // Repeated START before reading.
  if (result) {
    Serial.printf("READ register=%u select error=%u; no value available.\n",
                  unsigned(reg), unsigned(result));
    return false;
  }
  const unsigned received = Wire.requestFrom(legacy::address, length);
  if (received != length || Wire.available() != length) {
    Serial.printf(
        "READ register=%u short response=%u expected=%u; no value available.\n",
        unsigned(reg), received, unsigned(length));
    while (Wire.available())
      Wire.read();
    return false;
  }
  for (unsigned i = 0; i < length; ++i)
    data[i] = uint8_t(Wire.read());
  return true;
}

void showVoltage() {
  uint8_t data[2] = {}; // Never print these initial zeros if the read fails.
  if (!readDiagnostic(0, data, sizeof(data)))
    return;
  const unsigned millivolts = unsigned(data[0]) | (unsigned(data[1]) << 8);
  Serial.printf("SUPPLY reported=%u mV (%.3f V); raw=%02X %02X. Board reading, "
                "not a meter measurement.\n",
                millivolts, millivolts / 1000.0, unsigned(data[0]),
                unsigned(data[1]));
}

void showEncoders() {
  uint8_t data[16] =
      {}; // Four channels, four bytes each; no reset/write to counters.
  if (!readDiagnostic(60, data, sizeof(data)))
    return;
  Serial.printf("%s", "ENC cumulative:");
  for (unsigned i = 0; i < 4; ++i) {
    const unsigned offset = 4 * i;
    const uint32_t raw =
        uint32_t(data[offset]) | (uint32_t(data[offset + 1]) << 8) |
        (uint32_t(data[offset + 2]) << 16) | (uint32_t(data[offset + 3]) << 24);
    // Explicit sign conversion avoids overflowing signed shifts or relying
    // on implementation-defined unsigned-to-signed conversion for negatives.
    const int64_t count =
        raw <= INT32_MAX ? int64_t(raw) : int64_t(raw) - 4294967296LL;
    Serial.printf(" M%u=%ld", i + 1, long(count));
  }
  Serial.println(" (counts; wheel distance/calibration not inferred)");
}

void showHelp() {
  Serial.println("KIWI SPEED DRIVE v5: verified type=1 polarity=0 block "
                 "defaults; boots DISARMED.");
  Serial.println("init | status | arm | stop | ! | help");
  Serial.println("voltage | enc (idle/disarmed only; read driver "
                 "supply/counters; no motor command)");
  Serial.println("drive X Y [MS [SPEED]] (speed mode only; X/Y -1..1; defaults "
                 "1000ms,20 pulses/10ms; max 3000ms,30)");
  Serial.println("wheel CHANNEL ANGLE_DEG SIGN (idle/disarmed; sign +/-1; "
                 "provisional defaults 90/210/330,+1)");
  Serial.println("drive uses provisional geometry: X forward, Y left; NOT "
                 "centimeters. Verify physical directions.");
  Serial.println("profile TYPE POLARITY (TYPE 1 or 2; POLARITY 0 or 1)");
  Serial.println("transport block|single | bus 25000|50000|100000 | gap 0..10");
  Serial.println("mode speed|pwm (disarmed only; init afterward; PWM always "
                 "uses a full block)");
  Serial.println("Change settings only while disarmed; init required after "
                 "edits. Settings reset at boot.");
  Serial.println("motor CHANNEL SPEED MS  (channel 1..3; signed speed 1..58; "
                 "time 50..3000)");
  Serial.println(
      "original  (after arm: original pattern +58,-58,0 for 3000ms ONCE)");
  Serial.println(
      "Speed units are pulses/10ms, NOT percent. Start: motor 1 10 1000");
  Serial.println("In PWM mode motor CHANNEL DUTY MS uses percent, +/-1..50; "
                 "refreshed every 20ms.");
}

void showStatus() {
  Serial.printf("configured=%d armed=%d running=%d fault=%d lastError=%u "
                "lastRegister=%u\n",
                configured, enabled, running, broken, unsigned(lastError),
                unsigned(lastReg));
  Serial.printf("type=%u polarity=%u speedTransport=%s clock=%uHz gap=%ums\n",
                unsigned(selectedType), unsigned(selectedPolarity),
                singleWrites ? "single" : "block", selectedClock, writeGapMs);
  Serial.printf("MODE=%s; output register=%u; units=%s\n",
                pwmMode ? "PWM" : "SPEED", pwmMode ? 31u : 51u,
                pwmMode ? "percent duty" : "pulses/10ms");
  for (unsigned i = 0; i < 3; ++i)
    Serial.printf("M%u rollingAngle=%.1f sign=%d (provisional; RAM only)\n",
                  i + 1, double(wheelAngle[i]), wheelSign[i]);
}

// Validate again at this boundary: even C++ callers cannot bypass
// arm/fault/limits.
bool startTest(int a, int b, int c, unsigned milliseconds) {
  if (!configured || !enabled || broken || running || milliseconds < 50 ||
      milliseconds > legacy::maxDurationMs)
    return false;
  const int limit = pwmMode ? legacy::maxPwmDuty : legacy::maxCommand;
  for (int value : {a, b, c})
    if (value < -limit || value > limit)
      return false;
  if (!a && !b && !c) {
    stopTest("Zero pattern; stopped/disarmed.");
    return true;
  }
  began = lastReport = lastPwmRefresh = millis();
  testMs = milliseconds;
  Serial.printf("REQUEST: M1=%d M2=%d M3=%d %s, %ums\n", a, b, c,
                pwmMode ? "percent PWM" : "pulses/10ms", milliseconds);
  if (!sendSpeeds(a, b, c)) {
    // Attempt zero even though command acknowledgment failed. The packet
    // might have partly reached the driver; a fault must not mean "keep going".
    broken = true;
    stopTest("FAULT: test command failed; disarmed.");
    return false;
  }
  running = true;
  activeOutputs[0] = a;
  activeOutputs[1] = b;
  activeOutputs[2] = c;
  Serial.println(
      "STARTED: command acknowledged. Observe wheel; ! stops early.");
  return true;
}

// Whole base-10 numbers only, with bounds checked before any smaller-type cast.
bool parseInt(const char *text, long &value) {
  char *end = nullptr;
  value = std::strtol(text, &end, 10);
  return end != text && *end == '\0' && value >= -1000000 && value <= 1000000;
}

bool parseFloat(const char *text, float &value) {
  char *end = nullptr;
  value = std::strtof(text, &end);
  return end != text && *end == '\0' && std::isfinite(value);
}

void command(char *text) {
  char *words[6] = {};
  unsigned count = 0;
  char *saved = nullptr;
  if (text[0])
    Serial.printf("Command received: %s\n", text);
  for (char *w = strtok_r(text, " \t", &saved); w;
       w = strtok_r(nullptr, " \t", &saved)) {
    if (count == 6) {
      Serial.println("ERR: too many arguments.");
      return;
    }
    words[count++] = w;
  }
  if (!count)
    return;
  const char *cmd = words[0];
  if (count == 1 && !strcmp(cmd, "stop")) {
    stopTest("STOPPED; disarmed.");
    return;
  }
  if (count == 1 && !strcmp(cmd, "status")) {
    showStatus();
    return;
  }
  if (count == 1 && !strcmp(cmd, "help")) {
    showHelp();
    return;
  }
  if (count == 1 && !strcmp(cmd, "voltage")) {
    showVoltage();
    return;
  }
  if (count == 1 && !strcmp(cmd, "enc")) {
    showEncoders();
    return;
  }
  if (running) {
    Serial.println("ERR: busy; stop or wait first.");
    return;
  }
  if (!strcmp(cmd, "wheel") && count == 4) {
    long channel = 0, sign = 0;
    float angle = 0;
    if (enabled || stopRetries || !parseInt(words[1], channel) || channel < 1 ||
        channel > 3 || !parseFloat(words[2], angle) || std::fabs(angle) > 360 ||
        !parseInt(words[3], sign) || (sign != 1 && sign != -1)) {
      Serial.println("ERR: wheel needs idle/disarmed, channel 1..3, angle "
                     "-360..360, sign +/-1.");
      return;
    }
    wheelAngle[channel - 1] = angle;
    wheelSign[channel - 1] = int(sign);
    Serial.println(
        "Wheel geometry stored in RAM; no driver write. Reset loses edits.");
    return;
  }
  if (!strcmp(cmd, "drive") && count >= 3 && count <= 5) {
    float x = 0, y = 0;
    long milliseconds = 1000, speed = 20;
    int outputs[3] =
        {}; // Fresh all-zero candidate; never reuse a previous move.
    if (pwmMode || !parseFloat(words[1], x) || !parseFloat(words[2], y) ||
        (count >= 4 && !parseInt(words[3], milliseconds)) ||
        (count == 5 && !parseInt(words[4], speed)) || milliseconds < 50 ||
        milliseconds > 3000 || !mixDirection(x, y, int(speed), outputs)) {
      Serial.println("ERR: drive requires speed mode, X/Y -1..1, time "
                     "50..3000ms, speed 1..30.");
      return;
    }
    if (!startTest(outputs[0], outputs[1], outputs[2], unsigned(milliseconds)))
      Serial.println(
          "ERR: drive rejected; successful init and separate arm required.");
    return;
  }
  // Compare only documented type/polarity choices and standard bus rates.
  // These edits store preferences; init is the separate register-write step.
  const bool settings = !strcmp(cmd, "profile") || !strcmp(cmd, "transport") ||
                        !strcmp(cmd, "bus") || !strcmp(cmd, "gap") ||
                        !strcmp(cmd, "mode");
  if (settings) {
    if (enabled || stopRetries) {
      Serial.println(
          "ERR: stop/disarm and finish stop retries before settings edits.");
      return;
    }
    long first = 0, second = 0;
    bool valid = false;
    if (!strcmp(cmd, "mode") && count == 2 &&
        (!strcmp(words[1], "pwm") || !strcmp(words[1], "speed"))) {
      pwmMode = !strcmp(words[1], "pwm");
      valid = true;
    } else if (!strcmp(cmd, "profile") && count == 3 &&
               parseInt(words[1], first) && parseInt(words[2], second) &&
               (first == 1 || first == 2) && (second == 0 || second == 1)) {
      selectedType = uint8_t(first);
      selectedPolarity = uint8_t(second);
      valid = true;
    } else if (!strcmp(cmd, "transport") && count == 2 &&
               (!strcmp(words[1], "block") || !strcmp(words[1], "single"))) {
      singleWrites = !strcmp(words[1], "single");
      valid = true;
    } else if (!strcmp(cmd, "bus") && count == 2 && parseInt(words[1], first) &&
               (first == 25000 || first == 50000 || first == 100000)) {
      selectedClock = unsigned(first);
      valid = true;
    } else if (!strcmp(cmd, "gap") && count == 2 && parseInt(words[1], first) &&
               first >= 0 && first <= 10) {
      writeGapMs = unsigned(first);
      valid = true;
    }
    if (!valid) {
      Serial.println("ERR: invalid settings command; type help.");
      return;
    }
    configured = false;
    Serial.println(
        "Settings stored; no driver write yet. Enter init before arm.");
    return;
  }
  if (count == 1 && !strcmp(cmd, "init")) {
    initialize();
    return;
  }
  if (count == 1 && !strcmp(cmd, "arm")) {
    if (!configured || broken || stopRetries) {
      Serial.println("ERR: successful init required.");
      return;
    }
    if (!sendSpeeds()) {
      broken = true;
      stopTest("FAULT: arm zero write.");
      return;
    }
    enabled = true;
    Serial.println("ARMED; no motion yet.");
    return;
  }
  if (count == 1 && !strcmp(cmd, "original")) {
    if (pwmMode) {
      Serial.println("ERR: original is a speed-mode test only.");
      return;
    }
    if (!startTest(58, -58, 0, 3000))
      Serial.println("ERR: original test rejected.");
    return;
  }
  if (count == 4 && !strcmp(cmd, "motor")) {
    long channel = 0, speed = 0, milliseconds = 0;
    const int limit = pwmMode ? legacy::maxPwmDuty : legacy::maxCommand;
    if (!parseInt(words[1], channel) || !parseInt(words[2], speed) ||
        !parseInt(words[3], milliseconds) || channel < 1 || channel > 3 ||
        speed < -limit || speed > limit || speed == 0 || milliseconds < 50 ||
        milliseconds > 3000) {
      Serial.println(
          "ERR: check channel, mode's magnitude limit and time 50..3000ms.");
      return;
    }
    int values[3] = {};
    values[channel - 1] = int(speed);
    if (!startTest(values[0], values[1], values[2], unsigned(milliseconds)))
      Serial.println("ERR: test rejected; check status/init/arm.");
    return;
  }
  Serial.println("ERR: unknown command. Type help.");
}

// Short input updates keep the deadline active even if serial input is flooded.
void pollInput() {
  for (unsigned budget = 0; budget < 64 && Serial.available(); ++budget) {
    const uint8_t b = uint8_t(Serial.read());
    const char c = char(b);
    if (c == '!') {
      stopTest("STOPPED by !; disarmed.");
      used = 0;
      discard = true;
    } else if (c == '\n' || c == '\r') {
      if (!discard) {
        buffer[used] = 0;
        command(buffer);
      }
      used = 0;
      discard = false;
    } else if (!discard) {
      if (b == 8 || b == 127) {
        if (used)
          --used;
      } else if (b == 21)
        used = 0;
      else if ((b < 32 && c != '\t') || b > 126 || used == sizeof(buffer) - 1) {
        used = 0;
        discard = true;
        Serial.println("ERR: unsupported/overlong input; Enter, then retype.");
      } else
        buffer[used++] = c;
    }
  }
}

// Speed mode holds the original one-write request until the deadline. PWM mode
// instead refreshes a full snapshot at 50Hz, with stop/fault checks on every
// loop.
void updateTest() {
  const uint32_t now = millis();
  if (running) {
    if (now - began >= testMs) {
      stopTest("DONE; disarmed.");
      return;
    }
    if (pwmMode && now - lastPwmRefresh >= legacy::pwmRefreshMs) {
      lastPwmRefresh = now;
      if (!sendSpeeds(activeOutputs[0], activeOutputs[1], activeOutputs[2],
                      true)) {
        broken = true;
        stopTest("FAULT: PWM refresh; disarmed.");
        return;
      }
    }
    if (now - lastReport >= 1000) {
      lastReport = now;
      Serial.printf("ACTIVE: %ums remaining.\n",
                    unsigned(testMs - (now - began)));
    }
  } else if (stopRetries && now - lastRetry >= legacy::retryIntervalMs) {
    lastRetry = now;
    --stopRetries;
    if (sendSpeeds()) {
      stopRetries = 0;
      Serial.println("Stop retry acknowledged; remains disarmed.");
    } else if (!stopRetries)
      Serial.println(
          "FAULT: stop retries exhausted; use physical power removal.");
  }
}

void setup() {
  Serial.begin(legacy::baud);
  Wire.begin(legacy::sda, legacy::scl);
  Wire.setClock(legacy::busHz);
  Wire.setTimeOut(legacy::transactionTimeoutMs);
  delay(100); // Original setup's initial I2C pause.
  showHelp(); // No driver writes or automatic movement at boot.
  Serial.println("DISARMED. Enter init to test original driver settings.");
}

void loop() {
  updateTest();
  pollInput();
  delay(1);
}
