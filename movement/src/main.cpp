/*
SWRM: first-day robot controls
============================
Start with: arm <Enter>, then drive 1 0 <Enter>.
This sends a defined three-motor PWM pattern for 500ms and stops. It needs NO
wheel diameter, channel-position measurement or encoder calibration. The actual
chassis direction is something you observe and label; unknown signs can cause
rotation as well as translation. `motors` lets you try an exact motor pattern.

Later, `wheel` edits direction assumptions and `cal` supplies measured scale.
Only then use `move X Y` to request centimeters. `drive` never means
centimeters.

Origins of the code:
- SWRM_Leds-master/src/main.cpp supplies the ESP32/I2C baseline and demonstrates
  sending three motor commands plus a zero fourth channel.
- Hiwonder's register protocol is used to communicate with the motor driver.
- kiwi_math.h derives an ideal 120-degree kiwi mix, rather than claiming the
  original mix has been verified on this robot.
- Operating limits/timing are chosen initial settings in robot_config.h.

Read setup() first, then loop(), then the helper called by the command you use.
No uploaded firmware or physical robot motion has been verified here.
*/
#include "robot_config.h" // Named hardware assumptions, timing choices, wheel defaults.
#include <Arduino.h> // setup/loop framework, Serial text, millis() and delay().
#include <Wire.h> // I2C communication to the Hiwonder board (NOT direct motor power).
#include <cstdio>  // C-style formatted text support.
#include <cstdlib> // strtof() converts typed numbers; integer absolute values.
#include <cstring> // strcmp() compares command words; strtok_r() splits a line.

// Short local names for the documented register numbers; all provenance and
// editable constants are explained in robot_config.h. PWM and speed are
// different units.
constexpr uint8_t pwmReg = config::pwmReg;
constexpr uint8_t speedReg = config::speedReg;
constexpr uint8_t encoderReg = config::encoderReg;

// PROGRAM STATE: these variables remember what's happening between loop calls.
// They are initialized explicitly so startup never depends on leftover memory.
kiwi::Wheel wheels[3] =
    {};             // Space for M1/M2/M3 geometry; setup() fills the defaults.
bool ready = false; // True only after driver initialization acknowledges.
bool armed = false; // Operator has enabled the NEXT motion; boot is disarmed.
bool fault =
    false; // Latched communication/distance error; `init` can clear it.
uint8_t lastI2cError =
    0; // Last nonzero Wire write result, retained for troubleshooting.
uint8_t lastI2cErrorReg =
    0; // Register involved in that failure; 0 means none recorded yet.

// A state machine: one motion kind at a time, with no blocking "wait until
// done". This lets loop() keep reading stop commands while wheels are turning.
enum class Motion { idle, timedPwm, distance, jog, pulse };
Motion motion = Motion::idle; // At startup we have requested no movement.

// uint32_t is an unsigned 32-bit integer. millis() uses it for elapsed
// milliseconds. Initial zeros mean "no activity yet"; each motion sets its
// actual starting time.
uint32_t started = 0;      // Time at which the current motion was requested.
uint32_t lastTick = 0;     // Last periodic distance/PWM update.
uint32_t lastIdleStop = 0; // Last retry of zero output while idle.
uint32_t durationMs = 0;   // Length of the current timed command, not distance.
uint32_t lastTestReport =
    0; // Last single-wheel countdown message; reset for each test.
uint32_t lastProgress[3] =
    {}; // Last progress timestamp per wheel, distance mode only.

// Distance-mode encoder snapshots start at zero but are overwritten before use.
// We subtract starting counts instead of resetting the hardware counters.
int32_t origin[3] = {};   // Counts at start of a calibrated move.
int32_t progress[3] = {}; // Counts at last significant progress check.
int32_t previous[3] = {}; // Counts at previous 20ms sample, for jump detection.
float target[3] = {}; // Desired count CHANGES, not absolute room coordinates.
int8_t applied[3] =
    {}; // Last commands: PWM duty or pulses/10ms depending on state.

// Serial input arrives byte by byte, so we accumulate a line until Enter.
char line[config::serialLineBytes] =
    {};                // Fixed-size storage avoids growing allocations.
unsigned lineSize = 0; // Number of characters stored so far (initially empty).
bool discardLine =
    false; // True after overflow/!; ignore remainder until newline.

// Lowest-level write helper. reg selects the driver's internal register; bytes
// points to len outgoing bytes. A pointer is a memory address, not the motor.
// Return true only if the driver's I2C transaction was acknowledged. This does
// not prove the motor physically obeyed. Like the original wireWriteBlock(),
// but it returns an error status instead of silently ignoring a failed write.
bool writeBlock(uint8_t reg, const uint8_t *bytes, uint8_t len) {
  Wire.beginTransmission(config::driverAddress);
  Wire.write(reg);
  Wire.write(bytes, len);
  const uint8_t result = Wire.endTransmission();
  if (result != 0) {
    lastI2cError = result;
    lastI2cErrorReg = reg;
  }
  return result == 0;
}

// Package one signed command per motor. int8_t stores -128..127. Casting to
// uint8_t preserves the transmitted byte pattern, including negative commands.
// The fourth byte is always zero because this robot uses only three channels.
// Default arguments =0 make send(reg) a convenient all-motors stop request.
bool send(uint8_t reg, int8_t a = 0, int8_t b = 0, int8_t c = 0) {
  b *= -1; // Account for the polarity of motor b
  const uint8_t bytes[4] = {uint8_t(a), uint8_t(b), uint8_t(c), 0};
  return writeBlock(reg, bytes, sizeof(bytes));
}

// Clear both driver modes. Both writes must be attempted even if one fails.
// This is a software request, not a motor-power cut or hardware emergency stop.
bool zeroOutputs() {
  const bool s = send(speedReg);
  // Leave open-loop PWM zero as the last mode request. In particular, ending
  // a raw pulse must not engage an as-yet unverified encoder speed loop.
  const bool p = send(pwmReg);
  return p && s;
}

// End the active state, ask the driver to stop, and print a human-readable
// reason. By default the operator must type arm again before another motion. If
// the stop cannot be delivered, latch a fault and keep retrying zeros in idle
// ticks.
void stopMotion(const char *reason, bool disarm = true) {
  motion = Motion::idle;
  if (disarm)
    armed = false;
  if (!zeroOutputs()) {
    fault = true;
    armed = false;
    Serial.println("FAULT: stop write failed. Cut motor power if moving.");
  }
  for (int8_t &c : applied)
    c = 0;
  Serial.println(reason);
}

// A fault differs from normal completion: it prevents rearming until `init`.
void fail(const char *reason) {
  fault = true;
  stopMotion(reason);
}

// Read the three signed counters together. Each counter takes four bytes, so
// three channels need 3*4=12 bytes, starting at register 60. (&out)[3] means
// "a reference to the caller's three-item array", so results go into that
// array. In contrast to the original read helper, reject partial reads as
// failures. Only `enc` / `move` use this; timed driving works without encoder
// data.
bool readEncoders(int32_t (&out)[3]) {
  Wire.beginTransmission(config::driverAddress);
  Wire.write(encoderReg);
  if (Wire.endTransmission(false) != 0)
    return false; // false keeps the bus for a repeated-start read.
  const uint8_t wanted = 3 * 4; // Three 32-bit (four-byte) counters.
  if (Wire.requestFrom(config::driverAddress, wanted) != wanted) {
    while (Wire.available())
      Wire.read();
    return false;
  }
  uint8_t bytes[12] = {}; // All 12 bytes must actually arrive before decoding.
  for (int i = 0; i < 12; ++i) {
    if (!Wire.available())
      return false;
    bytes[i] = uint8_t(Wire.read());
  }
  for (int i = 0; i < 3; ++i)
    out[i] = kiwi::decode(bytes + 4 * i);
  return true;
}

// Configure communication with the driver, adapted from the original setup().
// This only sets type/polarity and requests zeros. It does not read encoders or
// check geometry. Thus missing distance calibration never blocks timed driving.
// Also used by the `init` command after fixing a communication problem.
void initDriver() {
  armed = false;
  motion = Motion::idle;
  ready = false;
  lastI2cError = lastI2cErrorReg =
      0; // Start a new initialization attempt's error history.
  const bool stopped = zeroOutputs();
  if (!stopped) {
    fail("FAULT: no driver/stop acknowledgement at 0x34.");
    return;
  }
  const uint8_t type = config::motorType, polarity = config::encoderPolarity;
  // &type means "address of this byte"; the helper expects a byte-array
  // pointer.
  if (!writeBlock(config::motorTypeReg, &type, 1)) {
    fail("FAULT: motor type write.");
    return;
  }
  delay(config::settingWaitMs);
  if (!writeBlock(config::encoderPolarityReg, &polarity, 1)) {
    fail("FAULT: encoder polarity write.");
    return;
  }
  delay(config::settingWaitMs);
  if (!zeroOutputs()) {
    fail("FAULT: final stop write.");
    return;
  }
  ready = true;
  fault = false;
  Serial.println("Driver acknowledged. DISARMED. No physical motion verified.");
}

// Print the command menu. No motor command is sent by printing help.
void help() {
  Serial.println(
      "help | status | enc | init | arm | stop | !\n"
      "drive X Y [MS [PWM]] (X/Y -1..1; defaults 500ms, 20%; max 2000ms, 25%)\n"
      "motors M1 M2 M3 [MS] (raw PWM -25..25; default 500ms, max 2000ms)\n"
      "Timed drive needs NO wheel measurements; physical direction is "
      "provisional.\n"
      "jog MOTOR SPEED MS   (1..3, +/-1..3 pulses/10ms, 50..5000ms)\n"
      "pulse MOTOR PWM MS   (1..3, +/-1..25 percent, 50..5000ms; no feedback)\n"
      "cal MOTOR DIAM_CM COUNTS_PER_REV (MOTOR 0 = all; measured values)\n"
      "wheel MOTOR ANGLE_DEG MOTOR_SIGN ENCODER_SIGN (signs +/-1)\n"
      "move FORWARD_CM LEFT_CM (max vector length 50cm, timeout 30s)\n"
      "+X forward, +Y left, relative to robot at command start.\n"
      "Settings live in RAM; reset loses cal/wheel edits. See README.");
}

// Show current software state and assumptions. ready means an I2C
// acknowledgment, not a guarantee of correct wiring or motion. Zero
// diameter/CPR is fine for drive.
void status() {
  Serial.printf(
      "ready=%d armed=%d fault=%d busy=%d speedLimit=%d pulses/10ms\n", ready,
      armed, fault, motion != Motion::idle, config::maxSpeed);
  Serial.printf("Last I2C write error=%u register=%u (0=no error since init). "
                "Motor supply not measured.\n",
                unsigned(lastI2cError), unsigned(lastI2cErrorReg));
  Serial.printf("Timed defaults: %ums, %d%% PWM; limit %ums, %d%% PWM. No "
                "calibration needed.\n",
                config::defaultTimedMs, config::defaultPwm, config::maxTimedMs,
                config::maxPwm);
  for (int i = 0; i < 3; ++i) {
    const auto &w = wheels[i];
    Serial.printf(
        "M%d angle=%.1f motorSign=%d encoderSign=%d diameter=%.3fcm CPR=%.1f\n",
        i + 1, w.angleDeg, w.motorSign, w.encoderSign, w.diameterCm,
        w.countsPerRev);
  }
}

// Start an exact THREE-MOTOR timed PWM pattern. This is the common
// implementation for `drive` (calculated pattern) and `motors` (operator's
// explicit pattern). A float array makes validation happen BEFORE narrowing
// values into signed bytes. Returns immediately: loop() is responsible for
// refreshing and ending the motion.
bool startTimedPwm(const float (&duty)[3], unsigned milliseconds) {
  if (!armed || !ready || fault || motion != Motion::idle ||
      milliseconds < config::minTimedMs || milliseconds > config::maxTimedMs)
    return false;
  int8_t candidate[3] =
      {}; // No active output until every input has passed validation.
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(duty[i]) || std::floor(duty[i]) != duty[i] ||
        std::fabs(duty[i]) > config::maxPwm)
      return false;
    candidate[i] =
        int8_t(duty[i]); // Conversion now safe: verified whole -25..25.
  }
  if (!candidate[0] && !candidate[1] && !candidate[2]) {
    stopMotion("Zero PWM pattern; stopped/disarmed.");
    return true;
  }
  started = lastTick =
      millis(); // Start deadline BEFORE I2C write, so its time is included.
  durationMs = milliseconds;
  if (!send(pwmReg, candidate[0], candidate[1], candidate[2])) {
    fail("FAULT: timed PWM write.");
    return false;
  }
  for (int i = 0; i < 3; ++i)
    applied[i] = candidate[i];
  motion = Motion::timedPwm;
  Serial.printf("TIMED PWM: M1=%d M2=%d M3=%d percent, %ums. "
                "Direction/distance unmeasured.\n",
                int(applied[0]), int(applied[1]), int(applied[2]),
                milliseconds);
  return true;
}

// Simple C++ entry point for the first-day mode. x/y are direction weights,
// not distances. Defaults allow driveRelative(1,0) with only two arguments.
// Uses provisional geometry but NO diameter, count calibration or encoder
// reads.
bool driveRelative(float forward, float left,
                   unsigned milliseconds = config::defaultTimedMs,
                   int power = config::defaultPwm) {
  if (power < 1 || power > config::maxPwm)
    return false;
  int8_t candidate[3] = {};
  if (!kiwi::directionPwm(forward, left, wheels, power, candidate))
    return false;
  const float duty[3] = {float(candidate[0]), float(candidate[1]),
                         float(candidate[2])};
  return startTimedPwm(duty, milliseconds);
}

// C++ entry point for relative travel. Returns immediately; loop() drives the
// move to completion. Returns false if busy, disarmed, uncalibrated, or
// invalid.
bool moveRelative(float forwardCm, float leftCm) {
  if (!armed || !ready || fault || motion != Motion::idle)
    return false;
  float planned[3] =
      {}; // Candidate target counts; not applied until all checks pass.
  if (!kiwi::targets(forwardCm, leftCm, wheels, planned))
    return false;
  if (!readEncoders(origin)) {
    fail("FAULT: encoder read before move.");
    return false;
  }
  started = lastTick = millis();
  for (int i = 0; i < 3; ++i) {
    target[i] = planned[i];
    progress[i] = previous[i] =
        origin[i]; // Current count is the new local reference.
    lastProgress[i] = started;
    applied[i] = 0;
  }
  motion = Motion::distance;
  Serial.printf("Move accepted: forward=%.2fcm left=%.2fcm; count deltas %.1f "
                "%.1f %.1f\n",
                forwardCm, leftCm, target[0], target[1], target[2]);
  return true;
}

// Strict, finite, complete numeric parsing: no partial numbers or silent zeros.
// s is a null-terminated typed token; &value is the caller's output variable.
// end points to the first unparsed character; requiring '\0' rejects e.g.
// "1oops".
bool number(const char *s, float &value) {
  if (!s || !*s)
    return false;
  char *end;
  value = std::strtof(s, &end);
  return end != s && *end == '\0' && std::isfinite(value);
}
// True for whole numbers such as 1 or -20; false for a fractional
// duration/channel.
bool integer(float v) { return std::floor(v) == v; }

// Handle ONE completed command line. We split it into words, validate argument
// counts and numbers, then call a motion helper. This parser does not queue
// moves. Example: "drive 1 0" -> tokens[0]="drive", tokens[1]="1",
// tokens[2]="0".
void command(char *text) {
  // Log exactly the edited line the ESP32 received. This makes hidden typing
  // differences easier to diagnose and includes the input in monitor log files.
  if (text[0])
    Serial.printf("Command received: %s\n", text);
  char *tokens[7] =
      {};         // Room for the command plus up to six numeric arguments.
  unsigned n = 0; // Number of words found so far; initially none.
  char *save =
      nullptr; // strtok_r's remembered parsing position, initially nowhere.
  for (char *t = strtok_r(text, " \t", &save); t;
       t = strtok_r(nullptr, " \t", &save)) {
    if (n == 7) {
      Serial.println("ERR: too many arguments.");
      return;
    }
    tokens[n++] = t;
  }
  if (!n)
    return;
  const char *cmd =
      tokens[0]; // First word chooses the action; strcmp returns 0 on equality.
  // Handle stop/help/status BEFORE the busy check: they work during movement.
  if (n == 1 && !strcmp(cmd, "stop")) {
    stopMotion("STOPPED; disarmed.");
    return;
  }
  if (n == 1 && !strcmp(cmd, "help")) {
    help();
    return;
  }
  if (n == 1 && !strcmp(cmd, "status")) {
    status();
    return;
  }
  if (motion != Motion::idle) {
    Serial.println("ERR: busy; stop first. No queue.");
    return;
  }
  if (n == 1 && !strcmp(cmd, "init")) {
    initDriver();
    return;
  }
  if (n == 1 && !strcmp(cmd, "enc")) {
    int32_t e[3] =
        {}; // Temporary result; printed only if a complete read succeeds.
    if (!readEncoders(e)) {
      fail("FAULT: encoder read.");
      return;
    }
    Serial.printf("Encoder counts: %ld %ld %ld\n", long(e[0]), long(e[1]),
                  long(e[2]));
    return;
  }
  if (n == 1 && !strcmp(cmd, "arm")) {
    if (!ready || fault) {
      Serial.println("ERR: fix hardware, then init first.");
      return;
    }
    if (!zeroOutputs()) {
      fail("FAULT: arm stop write.");
      return;
    }
    armed = true;
    Serial.println("ARMED. Motion requires a separate command.");
    return;
  }
  float a[6] = {}; // Parsed numeric arguments; a[0] is the FIRST number, not
                   // the command word.
  for (unsigned i = 1; i < n; ++i)
    if (!number(tokens[i], a[i - 1])) {
      Serial.println("ERR: invalid number; no motion.");
      return;
    }
  // Two arguments are enough. Optional third/fourth numbers override time and
  // duty. Check ranges in floating-point BEFORE converting to unsigned time/int
  // duty.
  if (n >= 3 && n <= 5 && !strcmp(cmd, "drive")) {
    const float ms = n >= 4 ? a[2] : float(config::defaultTimedMs);
    const float power = n == 5 ? a[3] : float(config::defaultPwm);
    if (!integer(ms) || ms < config::minTimedMs || ms > config::maxTimedMs ||
        !integer(power) || power < 1 || power > config::maxPwm ||
        !driveRelative(a[0], a[1], unsigned(ms), int(power)))
      Serial.println(
          "ERR: drive rejected; arm first, X/Y -1..1, MS 50..2000, PWM 1..25.");
    return;
  }
  if ((n == 4 || n == 5) && !strcmp(cmd, "motors")) {
    const float ms = n == 5 ? a[3] : float(config::defaultTimedMs);
    const float duty[3] = {a[0], a[1],
                           a[2]}; // Driver-channel order M1, M2, M3.
    if (!integer(ms) || ms < config::minTimedMs || ms > config::maxTimedMs ||
        !startTimedPwm(duty, unsigned(ms)))
      Serial.println(
          "ERR: motors rejected; arm first, whole PWM -25..25, MS 50..2000.");
    return;
  }
  if (n == 4 && !strcmp(cmd, "cal")) {
    if (armed) {
      Serial.println("ERR: stop to disarm before calibration.");
      return;
    }
    if (!integer(a[0]) || a[0] < 0 || a[0] > 3 || a[1] < 1 || a[1] > 50 ||
        a[2] < 10 || a[2] > 100000) {
      Serial.println("ERR: cal MOTOR(0..3) DIAM(1..50cm) CPR(10..100000).");
      return;
    }
    for (int i = 0; i < 3; ++i)
      if (a[0] == 0 || a[0] == i + 1) {
        wheels[i].diameterCm = a[1];
        wheels[i].countsPerRev = a[2];
      }
    Serial.println("Calibration set in RAM. Use status to check.");
    return;
  }
  if (n == 5 && !strcmp(cmd, "wheel")) {
    if (armed) {
      Serial.println("ERR: stop to disarm before geometry edits.");
      return;
    }
    if (!integer(a[0]) || a[0] < 1 || a[0] > 3) {
      Serial.println("ERR: wheel MOTOR must be 1..3.");
      return;
    }
    auto w = wheels[int(a[0]) - 1];
    // Check BEFORE float-to-integer conversion.
    if (std::fabs(a[1]) > 360 || std::fabs(a[2]) != 1 || std::fabs(a[3]) != 1) {
      Serial.println("ERR: angle -360..360, both signs +1 or -1.");
      return;
    }
    w.angleDeg = a[1];
    w.motorSign = int(a[2]);
    w.encoderSign = int(a[3]);
    wheels[int(a[0]) - 1] = w;
    Serial.println("Wheel geometry set in RAM.");
    return;
  }
  if (n == 3 && !strcmp(cmd, "move")) {
    if (!moveRelative(a[0], a[1]))
      Serial.println(
          "ERR: move rejected; check arm, cal, and <=50cm distance.");
    return;
  }
  const bool pulse = !strcmp(cmd, "pulse");
  if (n == 4 && (pulse || !strcmp(cmd, "jog"))) {
    const int limit = pulse ? config::maxPwm : config::maxSpeed;
    const unsigned maxMs = pulse ? config::maxPulseMs : config::maxJogMs;
    if (!armed || !ready || fault) {
      Serial.println("ERR: init/arm first.");
      return;
    }
    if (!integer(a[0]) || a[0] < 1 || a[0] > 3 || !integer(a[1]) ||
        std::fabs(a[1]) < 1 || std::fabs(a[1]) > limit || !integer(a[2]) ||
        a[2] < config::minTimedMs || a[2] > maxMs) {
      Serial.println(
          "ERR: jog/pulse motor, speed or duration outside help limits.");
      return;
    }
    int8_t c[3] =
        {}; // Start every channel at zero; turn on only the chosen wheel.
    c[int(a[0]) - 1] = int8_t(a[1]); // Human motor 1 becomes array index 0.
    started = lastTick = lastTestReport =
        millis(); // Include the write in the test's deadline.
    durationMs = uint32_t(a[2]);
    if (!send(pulse ? pwmReg : speedReg, c[0], c[1], c[2])) {
      fail("FAULT: test command write.");
      return;
    }
    for (int i = 0; i < 3; ++i)
      applied[i] = c[i]; // Keep the exact pattern for periodic refreshes.
    motion = pulse ? Motion::pulse : Motion::jog;
    Serial.printf("Wheel test started: M%d=%d %s; register=%u; duration=%ums. "
                  "Others=0.\n",
                  int(a[0]), int(a[1]), pulse ? "percent PWM" : "pulses/10ms",
                  unsigned(pulse ? pwmReg : speedReg), unsigned(durationMs));
    Serial.println("Automatic stop at deadline; type ! or stop to end early.");
    return;
  }
  Serial.println("ERR: unknown command/argument count. Type help.");
}

// Receive part/all of a typed line without waiting for more input. Newline or
// carriage return submits it. '!' is recognized as soon as received, even
// mid-line. The PC terminal's send-on-enter setting may buffer that character
// until Enter.
void pollSerial() {
  // Bound work per loop so a stream of serial bytes cannot starve the timeout.
  for (unsigned budget = 0;
       budget < config::serialBytesPerLoop && Serial.available(); ++budget) {
    const uint8_t byte =
        uint8_t(Serial.read()); // Unsigned so bytes >=128 are identifiable.
    const char c = char(byte);
    if (c == '!') {
      stopMotion("STOPPED by !; disarmed.");
      lineSize = 0;
      discardLine = true;
    } else if (c == '\n' || c == '\r') {
      if (!discardLine) {
        line[lineSize] = '\0';
        command(line);
      } // '\0' terminates a C string.
      lineSize = 0;
      discardLine = false;
    } else if (!discardLine) {
      // Direct serial terminals send editing keys as bytes. Previously
      // Backspace/DEL became part of the command, so a visually correct
      // "arm" could fail parsing. Remove one buffered character instead.
      if (byte == 8 || byte == 127) {
        if (lineSize)
          --lineSize;
      } else if (byte == 21) {
        lineSize = 0; // Ctrl+U clears an unfinished line; does not disarm.
      } else if ((byte < 32 && c != '\t') || byte > 126) {
        // Reject arrow-key escapes, embedded NUL and non-ASCII input.
        // Never silently drop unknown bytes and turn a malformed line
        // into an accidentally valid motion command. ! still stops.
        lineSize = 0;
        discardLine = true;
        Serial.printf(
            "ERR: unsupported input byte 0x%02X; press Enter and retype.\n",
            unsigned(byte));
      } else if (lineSize < sizeof(line) - 1)
        line[lineSize++] = c;
      else {
        lineSize = 0;
        discardLine = true;
        Serial.println("ERR: line too long; discarded through newline.");
      }
    }
  }
}

// Advance the active motion by ONE update, then return so input stays
// responsive. State decides which work is needed: idle retries zeros; timed PWM
// checks a deadline and refreshes duty; calibrated distance reads encoders and
// corrects error.
void tickMotion() {
  const uint32_t now = millis(); // One timestamp for this whole update.
  // Unsigned subtraction (now-started) still works if millis() wraps around
  // after ~49.7 days, provided these short intervals stay far below one wrap.
  if (motion == Motion::idle) {
    if (now - lastIdleStop >= config::idleStopPeriodMs) {
      lastIdleStop = now;
      if (!zeroOutputs()) {
        if (!fault)
          Serial.println("FAULT: idle stop write; fix bus and init.");
        fault = true;
        armed = false;
      }
    }
    return;
  }
  if (motion == Motion::timedPwm) {
    if (now - started >= durationMs) {
      stopMotion("Timed pattern done; disarmed.");
      return;
    }
    // No encoder read and no scale check here. Re-sending the same pattern
    // keeps output definite and notices an I2C write failure during the test.
    if (now - lastTick >= config::timedRefreshMs) {
      lastTick = now;
      if (!send(pwmReg, applied[0], applied[1], applied[2]))
        fail("FAULT: timed PWM refresh write.");
    }
    return;
  }
  if (motion == Motion::jog || motion == Motion::pulse) {
    if (now - started >= durationMs) {
      stopMotion("Wheel test done; disarmed.");
      return;
    }
    // A longer test should also detect lost I2C contact DURING the test.
    // Re-send the same signed output every 100ms, never increasing it.
    if (now - lastTick >= config::timedRefreshMs) {
      lastTick = now;
      const uint8_t reg = motion == Motion::pulse ? pwmReg : speedReg;
      if (!send(reg, applied[0], applied[1], applied[2])) {
        fail("FAULT: wheel test refresh write.");
        return;
      }
    }
    if (now - lastTestReport >= config::testReportPeriodMs) {
      lastTestReport = now;
      Serial.printf(
          "Wheel test active: %ums remaining. ! or stop ends early.\n",
          unsigned(durationMs - (now - started)));
    }
    return;
  }
  if (now - started >= config::moveTimeoutMs) {
    fail("FAULT: move timeout.");
    return;
  }
  if (now - lastTick < config::controlPeriodMs)
    return;
  lastTick = now;
  int32_t e[3] = {}; // Latest counts; a failed read ends the move before use.
  if (!readEncoders(e)) {
    fail("FAULT: incomplete/failed encoder read.");
    return;
  }
  float commandSpeed[3] = {}; // Start this update requesting no wheel speed.
  bool done =
      true; // Assume completion until ANY wheel is found outside tolerance.
  for (int i = 0; i < 3; ++i) {
    const int64_t step =
        kiwi::delta(e[i], previous[i]); // Change since last feedback sample.
    previous[i] = e[i];
    // Impossible jump at this speed; catches resets and corrupt full reads.
    if (std::abs(step) > config::maxEncoderStep) {
      fail("FAULT: implausible encoder jump.");
      return;
    }
    // Desired count change minus actual count change = remaining wheel error.
    const float error = target[i] - float(kiwi::delta(e[i], origin[i]));
    // Convert our chosen 0.1cm wheel tolerance to counts, but accept at least
    // 3 counts so the controller doesn't chase the encoder's last transition.
    const float tolerance =
        fmaxf(config::minimumToleranceCounts,
              wheels[i].countsPerRev * config::wheelToleranceCm /
                  (kiwi::pi * wheels[i].diameterCm));
    if (std::fabs(error) <= tolerance) {
      lastProgress[i] = now;
      progress[i] = e[i];
      continue;
    }
    done = false;
    if (std::abs(kiwi::delta(e[i], progress[i])) >= config::progressCounts) {
      const int64_t advance = kiwi::delta(e[i], progress[i]);
      if (applied[i] && advance * applied[i] * wheels[i].encoderSign < 0) {
        fail("FAULT: encoder direction disagrees with command; check wheel "
             "encoderSign.");
        return;
      }
      progress[i] = e[i];
      lastProgress[i] = now;
    }
    if (now - lastProgress[i] > config::stallTimeoutMs) {
      fail("FAULT: wheel stalled/no encoder progress.");
      return;
    }
    // Position gain 2/s, then convert counts/s to pulses/10ms. Driver closes
    // the inner speed loop; this program closes the outer distance loop.
    // The old literal 0.02 was 2/s * 0.01s, now written as named constants.
    commandSpeed[i] = error * config::positionGainPerSec *
                      config::speedUnitSeconds * wheels[i].encoderSign;
  }
  if (done) {
    stopMotion("DONE: encoder targets reached; disarmed.");
    return;
  }
  kiwi::scale(commandSpeed, config::maxSpeed);
  for (int i = 0; i < 3; ++i) {
    applied[i] = int8_t(std::lround(commandSpeed[i]));
    // Driver speed resolution is 1 pulse/10ms. Avoid rounding a needed
    // correction to zero. This may cause small final path/heading errors.
    if (!applied[i] && commandSpeed[i] != 0)
      applied[i] = commandSpeed[i] > 0 ? 1 : -1;
  }
  if (!send(speedReg, applied[0], applied[1], applied[2]))
    fail("FAULT: speed write.");
}

// Arduino calls setup() ONCE after power-up/reset. Establish text
// communication, choose provisional geometry, configure I2C, and request a
// stationary driver. Unlike the old demo loop, this firmware never
// automatically starts movement.
void setup() {
  Serial.begin(config::serialBaud);
  config::defaults(wheels); // Known assumptions, not measured calibration.
  Wire.begin(config::sda, config::scl);
  Wire.setClock(config::i2cClockHz);
  Wire.setTimeOut(config::i2cTimeoutMs);
  delay(config::startupWaitMs);
  Serial.println("SWRM relative kiwi drive. Boots DISARMED; never auto-moves.");
  initDriver();
  help();
}

// Arduino repeats loop() indefinitely. These calls are intentionally short:
// do one motion update, handle available text, then yield for 1ms and repeat.
// A long delay(500) during movement would prevent prompt serial stop handling.
void loop() {
  tickMotion(); // Stop a finished/timed-out motion before processing more
                // input.
  pollSerial(); // May start a NEW motion or stop the current one.
  delay(config::loopPauseMs);
}
