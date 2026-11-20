/*
V16 Kiwi movement: +X right, +Y forward, +W clockwise; M2 inversion at I2C.
init leaves disarmed. Explicit square/spin auto-arm through the existing guard;
stop cancels all motion/queued starts. Settings and networking are separate
files. See ../README.md for setup, calibration defaults, diagnostics and test
limits. HUB75 startup below is preserved from origin/main; disable its build
flag for motor-only lab comparisons. No heading/floor-position feedback is
implemented.
*/
#include <Arduino.h>
#include <Wire.h>
#if defined(SWRM_ENABLE_LED_PANEL) && SWRM_ENABLE_LED_PANEL
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#endif
#include "../include/mqtt_controller.h"
#include "../include/robot_commands.h"
#include "../include/robot_settings.h"
#include <atomic>
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
// Speed snapshots encode each wheel as a signed byte. Keep a symmetric range
// so reversing M2 cannot turn -128 into an unrepresentable +128. This is an
// encoding ceiling, NOT a measured driver/motor capability or a target to
// reach.
constexpr int maxCommand = 127;
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
#if defined(SWRM_ENABLE_LED_PANEL) && SWRM_ENABLE_LED_PANEL
// RGB channel pins
constexpr unsigned r1 = 25;
constexpr unsigned g1 = 26;
constexpr unsigned b1 = 27;
constexpr unsigned r2 = 14;
constexpr unsigned g2 = 12;
constexpr unsigned b2 = 13;

// Other pins
constexpr unsigned a = 23; // yes
constexpr unsigned b = 19; // yes
constexpr unsigned c = 5;  // yes
constexpr unsigned d = 18; // yes
constexpr unsigned e = 17; // yes

// issue with these pins
constexpr unsigned lat = 4;  // moved
constexpr unsigned oe = 15;  // moved
constexpr unsigned clk = 16; // moved

constexpr unsigned res_x = 64;
constexpr unsigned res_y = 64;
constexpr unsigned panel_chain = 2;
#endif
} // namespace legacy

// Operator-selected defaults and experiment bounds, NOT motor specifications.
// Keep these together so a future limit change reaches planner AND parser.
namespace motion {
constexpr unsigned minDurationMs = 50, defaultDriveMs = 1000;
constexpr unsigned defaultSquareMs = 1200, defaultPauseMs = 1000,
                   maxPauseMs = 2000;
constexpr int defaultSpeed = 20, maxSpeed = legacy::maxCommand;
} // namespace motion
static_assert(
    legacy::maxCommand <= INT8_MAX && motion::maxSpeed <= legacy::maxCommand,
    "Wheel speed and M2 sign reversal must fit the signed-byte payload.");

// State starts explicitly disabled. Configuration and arming are separate
// steps.
bool configured = false, enabled = false, running = false, broken = false;
bool spinActive =
    false; // Continuous speed hold; cancelled by the common stop/fault path.
std::atomic<robot::Command> pendingRobotCommand{robot::Command::none};
std::atomic<bool> pendingStatusRequest{false}, maintenanceBusy{false};
// Preserve a concurrently queued stop while cancelling older starts/init.
static void clearPendingAction() {
  auto pending = pendingRobotCommand.load();
  if (pending != robot::Command::stop)
    pendingRobotCommand.compare_exchange_strong(pending, robot::Command::none);
}
static void publishRobotState(bool force = false);
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
uint32_t lastWriteEnded =
    0;                // I2C completion BEFORE logging/optional post-write gap.
char buffer[96] = {}; // Fixed command storage; no heap allocation needed.
unsigned used = 0;
bool discard =
    false; // Reject overlong/unsupported input through the next newline.

// One arm authorizes the selected one-shot, bounded or endless square run.
// Cache the complete plan at its start;
// normal commands cannot replace a leg or change settings during corner pauses.
struct SquarePattern {
  bool active = false, pausing = false;
  unsigned leg = 0, sideMs = 0, pauseMs = 0;
  unsigned durations[4] = {};
  uint32_t cycles = 0, cycleLimit = 1; // Limit 0 means explicit endless mode.
  bool attemptOpen = false; // A completed cycle's pause is not a new attempt.
  uint32_t pauseBegan = 0;
  int outputs[4][3] = {};
};
SquarePattern squarePattern;
struct SquareStats {
  uint32_t attempts = 0, completed = 0, faults = 0, cancelled = 0;
  uint32_t streak = 0, best = 0;
};
SquareStats squareStats; // RAM; init preserves counts, reboot clears them.
struct WriteFault {
  bool valid = false;
  uint32_t at = 0, elapsed = 0;
  unsigned leg = 0, length = 0, clock = 0;
  uint8_t reg = 0, error = 0, data[4] = {};
};
WriteFault firstWriteFault; // First failure since init; zero retries cannot
                            // replace it.

const char *wireResultName(uint8_t result) {
  switch (result) {
  case 0:
    return "ok";
  case 1:
    return "buffer-too-long";
  case 2:
    return "nack";
  case 3:
    return "data-nack";
  case 4:
    return "other-error";
  case 5:
    return "timeout";
  default:
    return "unknown";
  }
}
void showSquareStats() {
  Serial.printf(
      "SQUARE STATS attempts=%lu completed=%lu faults=%lu cancelled=%lu "
      "streak=%lu best=%lu target=10 (RAM; init preserves counts)\n",
      (unsigned long)squareStats.attempts, (unsigned long)squareStats.completed,
      (unsigned long)squareStats.faults, (unsigned long)squareStats.cancelled,
      (unsigned long)squareStats.streak, (unsigned long)squareStats.best);
}
void showWriteFault() {
  if (!firstWriteFault.valid) {
    Serial.println("FIRST WRITE FAULT: none since init.");
    return;
  }
  const auto &f = firstWriteFault;
  Serial.printf("FIRST WRITE FAULT t=%lums leg=%u/4 register=%u error=%u (%s) "
                "tx=%lums clock=%uHz bytes=%u data=",
                (unsigned long)f.at, f.leg, unsigned(f.reg), unsigned(f.error),
                wireResultName(f.error), (unsigned long)f.elapsed, f.clock,
                f.length);
  for (unsigned i = 0; i < f.length && i < sizeof(f.data); ++i)
    Serial.printf("%02X ", unsigned(f.data[i]));
  Serial.println(
      "; leg=0 means outside square; error 4 alone does not identify cause.");
}
constexpr float squareX[4] = {0, 1, 0, -1};
constexpr float squareV9Y[4] = {-1, 0, 1, 0};
const char *const squareNames[4] = {"forward", "right", "backward", "left"};

// Compiled boot defaults survive reboot. Serial tuning remains RAM-only;
// explicit square-leg reset still selects unity for diagnostic comparisons.
using SquareLegTuning = robot::LegTuning;
SquareLegTuning squareTuning[4] = {
    robot::selected.square.legs[0], robot::selected.square.legs[1],
    robot::selected.square.legs[2], robot::selected.square.legs[3]};
void showSquareTuning() {
  for (unsigned leg = 0; leg < 4; ++leg)
    Serial.printf("SQUARE LEG %s timeScale=%.3f trims=%.3f,%.3f,%.3f (RAM; "
                  "multiply global trims)\n",
                  squareNames[leg], double(squareTuning[leg].timeScale),
                  double(squareTuning[leg].trim[0]),
                  double(squareTuning[leg].trim[1]),
                  double(squareTuning[leg].trim[2]));
}
int squareLegIndex(const char *name) {
  for (unsigned leg = 0; leg < 4; ++leg)
    if (!strcmp(name, squareNames[leg]))
      return int(leg);
  return -1;
}

enum class Mixer { legacy, original, geometry };
Mixer selectedMixer = Mixer::legacy; // Start with the teammate's working model.
const char *mixerName() {
  if (selectedMixer == Mixer::legacy)
    return "legacy";
  return selectedMixer == Mixer::original ? "original" : "geometry";
}

enum class DriveOrientation { lab, v9 };
DriveOrientation selectedOrientation = DriveOrientation::lab;
const char *orientationName() {
  return selectedOrientation == DriveOrientation::lab ? "lab" : "v9";
}
bool setOrientation(DriveOrientation orientation) {
  if (enabled || running || squarePattern.active || stopRetries ||
      (orientation != DriveOrientation::lab &&
       orientation != DriveOrientation::v9))
    return false;
  selectedOrientation = orientation;
  return true;
}
void showOrientation() {
  if (selectedOrientation == DriveOrientation::lab)
    Serial.println(
        "Orientation=lab: +X intended RIGHT, +Y intended FORWARD; M1 "
        "rear-left, M2 front, M3 rear-right; no heading correction.");
  else
    Serial.println(
        "Orientation=v9: previous X/Y inputs restored (formerly labeled +X "
        "right/+Y backward; physical directions differ).");
}
float squareY(unsigned leg) {
  return selectedOrientation == DriveOrientation::lab ? -squareV9Y[leg]
                                                      : squareV9Y[leg];
}

// Preserve the teammate's physical channel correction ONCE at the I2C boundary.
// These are hardware output signs, separate from geometry wheelSign and
// positive motorTrim. Raw motor commands also pass through this adapter, as in
// GitHub V5.
constexpr int hardwareSign[3] = {1, -1, 1};
int driverCommand(unsigned channelIndex, int logical) {
  return logical * hardwareSign[channelIndex];
}
void showHardwareSigns() {
  Serial.println("Hardware output signs: M1=+1 M2=-1 M3=+1 (applied once; raw "
                 "encoders retain driver signs)");
}

// GEOMETRY MODE ONLY: reference R1 upper-left, R2 upper-right, R3 rear,
// assuming those are M1/M2/M3. This channel mapping is UNKNOWN on the physical
// robot. Angles are the CCW tangent directions from FORWARD toward LEFT (not
// +public X). Each tangent belongs to a wheel on an equal-radius circle about
// chassis center; changing an angle assumes the associated wheel position
// rotates with it. Positive CCW chassis rotation needs all three positive
// tangent speeds. Use wheelSign=-1 if positive driver speed propels against
// that tangent. `wheel CHANNEL ANGLE SIGN` affects only geometry mode, never
// driver polarity.
float wheelAngle[3] = {150, 30, 270};
int wheelSign[3] = {1, 1, 1};

// Empirical SPEED-request multipliers, not mass or power settings. Unity keeps
// prior behavior. Applied only to calculated drive/twist/square outputs, never
// raw motor/PWM tests or fixed `original` packets. Runtime edits need no init.
float motorTrim[3] = {1, 1, 1};
constexpr float minMotorTrim = 0.5f,
                maxMotorTrim = 1.5f; // Chosen experiment bounds.
constexpr int maxDriveCommand =
    motion::maxSpeed; // Shared wheel ceiling after trims.

bool setMotorTrim(unsigned channel, float multiplier) {
  if (enabled || running || squarePattern.active || stopRetries ||
      channel < 1 || channel > 3 || !std::isfinite(multiplier) ||
      multiplier < minMotorTrim || multiplier > maxMotorTrim)
    return false;
  motorTrim[channel - 1] = multiplier;
  return true;
}

bool resetMotorTrims() {
  if (enabled || running || squarePattern.active || stopRetries)
    return false;
  for (float &trim : motorTrim)
    trim = 1;
  return true;
}

void showMotorTrims() {
  Serial.printf("Speed trims: M1=%.3f M2=%.3f M3=%.3f (drive/twist/square "
                "only; RAM; wheel commands capped at %d)\n",
                double(motorTrim[0]), double(motorTrim[1]),
                double(motorTrim[2]), maxDriveCommand);
}

// Pure calculation: NO I2C writes, initialization, arming, or motion.
// Lab: x=right, y=forward; v9: previous input axes. w=clockwise in both.
// Normalized weights, NOT distances/radians. Orientation changes the public
// frame ONLY; all existing mixer equations below retain their V9 model frame.
// speed is a reference pulses/10ms scale. Common limiting preserves ratios
// before rounding. Original mode uses the old coefficients; at the same output
// scale, rounding differs from old truncation by at most one count command.
bool computeWheelCommands(float x, float y, float w, int speed, int (&out)[3],
                          const float *legTrim = nullptr) {
  for (int &value : out)
    value = 0;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) ||
      std::fabs(x) > 1 || std::fabs(y) > 1 || std::fabs(w) > 1 || speed < 1 ||
      speed > motion::maxSpeed)
    return false; // Experiment cap, not a motor rating.
  // Actual channel positions: M1 rear-left, M2 front, M3 rear-right. Rotate
  // the existing model frame instead of changing its mixer or motor signs.
  // For legacy this yields M1=.5*x-.866*y, M2=-x, M3=.5*x+.866*y (W=0).
  // Therefore new +Y forward leaves FRONT M2 stopped, as ideal kinematics
  // requires. This assumes symmetric 120-degree geometry and needs floor QA.
  constexpr float root3Over2 = 0.8660254037844386f;
  const float modelX = selectedOrientation == DriveOrientation::lab
                           ? -0.5f * x + root3Over2 * y
                           : x;
  const float modelY = selectedOrientation == DriveOrientation::lab
                           ? -root3Over2 * x - 0.5f * y
                           : y;
  float mix[3] = {};
  float peak = 1; // Scale down only; small inputs remain small.
  if (selectedMixer == Mixer::original) {
    mix[0] = 0.33f * modelX + 0.58f * modelY - 0.33f * w;
    mix[1] = 0.33f * modelX - 0.58f * modelY - 0.33f * w;
    mix[2] = 0.67f * modelX + 0.33f * w;
  } else {
    const float forward = -modelY, left = -modelX;
    constexpr float radiansPerDegree = 3.14159265358979323846f / 180;
    for (unsigned i = 0; i < 3; ++i) {
      const bool legacyModel = selectedMixer == Mixer::legacy;
      const float angleDegrees =
          legacyModel ? 90.0f + 120.0f * i : wheelAngle[i];
      const int sign = legacyModel ? 1 : wheelSign[i];
      if (!std::isfinite(angleDegrees) || (sign != 1 && sign != -1))
        return false;
      const float angle = angleDegrees * radiansPerDegree;
      // w is an L*omega-equivalent normalized weight, CW hence minus.
      mix[i] = (std::cos(angle) * forward + std::sin(angle) * left - w) * sign;
    }
  }
  for (float value : mix)
    if (std::fabs(value) > peak)
      peak = std::fabs(value);
  float requests[3] = {};
  float trimmedPeak = 0;
  for (unsigned i = 0; i < 3; ++i) {
    if (!std::isfinite(motorTrim[i]) || motorTrim[i] < minMotorTrim ||
        motorTrim[i] > maxMotorTrim)
      return false;
    const float extraTrim = legTrim ? legTrim[i] : 1;
    if (!std::isfinite(extraTrim) || extraTrim < minMotorTrim ||
        extraTrim > maxMotorTrim)
      return false;
    requests[i] = speed * mix[i] / peak * motorTrim[i] * extraTrim;
    if (std::fabs(requests[i]) > trimmedPeak)
      trimmedPeak = std::fabs(requests[i]);
  }
  // If a trim pushes a wheel above the encoding ceiling, scale ALL requests
  // down together to preserve the trimmed ratios. Never clip wheels
  // independently or energize a wheel whose original request was zero. Rounding
  // has finite resolution.
  const float capScale =
      trimmedPeak > maxDriveCommand ? maxDriveCommand / trimmedPeak : 1;
  for (unsigned i = 0; i < 3; ++i)
    out[i] = int(std::lround(requests[i] * capScale));
  return true;
}

// Pure square planner: all sides must be valid/nonzero before any motor write.
// Equal elapsed times do not guarantee equal measured distances or closure.
bool planSquare(unsigned sideMs, int speed, unsigned pauseMs,
                int (&outputs)[4][3], unsigned *durations = nullptr) {
  if (sideMs < motion::minDurationMs || sideMs > legacy::maxDurationMs ||
      pauseMs > motion::maxPauseMs)
    return false;
  for (unsigned i = 0; i < 4; ++i) {
    const float scale = squareTuning[i].timeScale;
    if (!std::isfinite(scale) || scale < 0.5f || scale > 1.5f)
      return false;
    const long duration = std::lround(sideMs * scale);
    if (duration < motion::minDurationMs || duration > legacy::maxDurationMs)
      return false;
    if (!computeWheelCommands(squareX[i], squareY(i), 0, speed, outputs[i],
                              squareTuning[i].trim) ||
        (!outputs[i][0] && !outputs[i][1] && !outputs[i][2]))
      return false;
    if (durations)
      durations[i] = unsigned(duration);
  }
  return true;
}

// Same register-plus-byte-array transaction as the original, but report every
// write result. Numeric result 0 means acknowledgment, not verified wheel
// motion.
bool writeRegister(uint8_t reg, const uint8_t *data, unsigned length,
                   bool verbose = true) {
  const uint32_t txBegan = millis();
  Wire.beginTransmission(legacy::address);
  Wire.write(reg);
  for (unsigned i = 0; i < length; ++i)
    Wire.write(data[i]);
  const uint8_t result = Wire.endTransmission();
  lastWriteEnded = millis();
  const uint32_t elapsed = lastWriteEnded - txBegan;
  if (result && !firstWriteFault.valid) {
    firstWriteFault.valid = true;
    firstWriteFault.at = lastWriteEnded;
    firstWriteFault.elapsed = elapsed;
    firstWriteFault.leg = squarePattern.active ? squarePattern.leg + 1 : 0;
    firstWriteFault.reg = reg;
    firstWriteFault.error = result;
    firstWriteFault.length = length;
    firstWriteFault.clock = selectedClock;
    for (unsigned i = 0; i < length && i < sizeof(firstWriteFault.data); ++i)
      firstWriteFault.data[i] = data[i];
  }
  // Print the actual data too, not just the caller's intended wheel values.
  if (verbose || result) {
    Serial.printf("WRITE register=%u bytes=%u result=%u data=", unsigned(reg),
                  length, unsigned(result));
    for (unsigned i = 0; i < length; ++i)
      Serial.printf("%02X ", unsigned(data[i]));
    Serial.printf("t=%lums tx=%lums leg=%u/4 (%s)\n",
                  (unsigned long)lastWriteEnded, (unsigned long)elapsed,
                  squarePattern.active ? squarePattern.leg + 1 : 0,
                  wireResultName(result));
  }
  if (writeGapMs)
    delay(writeGapMs);
  if (result) {
    lastError = result;
    lastReg = reg;
  }
  return result == 0;
}

// Same four logical channel requests as original setMotorSpeeds(). Default is a
// four-byte block; individual writes remain a diagnostic option. M4 stays zero.
bool sendSpeeds(int a = 0, int b = 0, int c = 0, bool quietRefresh = false) {
  // Validate before both the sign adapter and int8_t conversion. A caller
  // cannot wrap a too-large speed into reverse, or a PWM request beyond its
  // separate duty limit, by bypassing the command parser.
  const int limit = pwmMode ? legacy::maxPwmDuty : legacy::maxCommand;
  for (int value : {a, b, c})
    if (value < -limit || value > limit)
      return false;
  a = driverCommand(0, a);
  b = driverCommand(1, b);
  c = driverCommand(2, c);
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
    return writeRegister(legacy::speedReg, data, 4, !quietRefresh);
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

// Record only a full cycle with an acknowledged final zero write.
void recordSquareCompletion() {
  ++squareStats.completed;
  ++squareStats.streak;
  ++squarePattern.cycles;
  if (squareStats.streak > squareStats.best)
    squareStats.best = squareStats.streak;
  squarePattern.attemptOpen = false;
}
// A normal stop always disables the next motion. Failure latches a fault and
// schedules a few zero-only retries; it never permits continuing a motor test.
void stopTest(const char *reason, bool squareFinished = false) {
  pendingRobotCommand.store(robot::Command::none);
  spinActive = false;
  const bool wasSquare = squarePattern.active;
  const bool attemptOpen = wasSquare && squarePattern.attemptOpen;
  // Keep square context through the zero write so a final/corner stop fault
  // records its leg. Cancel the pattern before returning or any retry tick.
  squarePattern.pausing = false;
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
  squarePattern.active = false;
  if (attemptOpen) {
    if (broken) {
      ++squareStats.faults;
      squareStats.streak = 0;
    } else if (squareFinished) {
      recordSquareCompletion();
    } else {
      ++squareStats.cancelled;
      squareStats.streak = 0;
    }
  } else if (wasSquare && broken) {
    // A failed stop between completed cycles still breaks reliability.
    squareStats.streak = 0;
  }
  squarePattern.attemptOpen = false;
  if (wasSquare)
    showSquareStats();
  Serial.println(reason);
  if (broken)
    showWriteFault();
}

// Test the original numeric settings step by step. Stop at the first failure,
// preserving which register failed. This is not an address-only connectivity
// check.
void initialize() {
  clearPendingAction();
  if (spinActive)
    stopTest("Spin cancelled by initialization.");
  if (squarePattern.active)
    stopTest("Square cancelled by initialization.");
  configured = enabled = running = false;
  broken = false;
  stopRetries = 0;
  lastError = lastReg = 0;
  firstWriteFault = WriteFault{};
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
// Bounded diagnostics retain the idle-only guard below; a short read never
// becomes a valid zero count.
static bool readRegisterBytes(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(legacy::address);
  Wire.write(reg);
  const uint8_t result =
      Wire.endTransmission(false); // Repeated START before reading.
  if (result) {
    lastError = result;
    lastReg = reg;
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

bool readDiagnostic(uint8_t reg, uint8_t *data, uint8_t length) {
  if (enabled || running || stopRetries) {
    Serial.println("ERR: diagnostics require idle/disarmed; stop first.");
    return false;
  }
  return readRegisterBytes(reg, data, length);
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
  Serial.println(
      "KIWI MONOREPO LAB v16: battery boot initialization; square/stop/spin; "
      "orientation=lab (+Y forward); M2 reversed; mixer=legacy; DISARMED.");
  Serial.println("init | status | arm | stop | ! | help");
  Serial.println("voltage | enc (idle/disarmed only; read driver "
                 "supply/counters; no motor command)");
  Serial.printf("drive X Y [MS [SPEED]] (speed mode only; X/Y -1..1; defaults "
                "%ums,%d pulses/10ms; max %ums,%d)\n",
                motion::defaultDriveMs, motion::defaultSpeed,
                legacy::maxDurationMs, motion::maxSpeed);
  Serial.println(
      "twist X Y W [MS [SPEED]] (same limits; W -1..1, positive clockwise)");
  Serial.println("preview X Y [W [SPEED]] (idle/disarmed; calculate only, NO "
                 "driver writes)");
  Serial.println("square | spin | stop (one-word starts auto-arm after init; "
                 "square repeats, spin holds; stop disarms)");
  Serial.println("square SIDE_MS [SPEED [PAUSE_MS]] (diagnostic ONE cycle "
                 "after separate arm; defaults 1200,20,1000)");
  Serial.println("preview-square [SIDE_MS [SPEED [PAUSE_MS]]] (disarmed; all "
                 "four legs, NO driver writes)");
  Serial.println("square-endless [SIDE_MS [SPEED [PAUSE_MS]]] | square-repeat "
                 "COUNT [SIDE_MS [SPEED [PAUSE_MS]]]");
  Serial.println("Repeats pause after the fourth side too; stop/! cancels now, "
                 "faults cancel the run. No controller-loss heartbeat yet.");
  Serial.println("square-leg [reset] | square-leg DIRECTION TIME_SCALE M1_TRIM "
                 "M2_TRIM M3_TRIM (idle/disarmed; all 0.5..1.5; RAM)");
  Serial.println("square-test DIRECTION [SIDE_MS [SPEED]] (one tuned leg after "
                 "arm; direction=forward|right|backward|left)");
  Serial.println("Square leg factors multiply global trims; use trim reset for "
                 "tuning. drive/twist do not use square leg factors.");
  Serial.printf("Square: side %u..%ums, speed 1..%d, pause 0..%ums; timed "
                "pattern, NOT measured distance.\n",
                motion::minDurationMs, legacy::maxDurationMs, motion::maxSpeed,
                motion::maxPauseMs);
  Serial.println("square-stats | square-stats reset (counts "
                 "completions/faults/cancels/streak; reset requires "
                 "idle/disarmed; RAM only)");
  Serial.println("mixer legacy|original|geometry (idle/disarmed; RAM only; no "
                 "init needed)");
  Serial.println("orientation lab|v9 (idle/disarmed; RAM only; lab = +X "
                 "right/+Y forward; v9 restores old commands)");
  Serial.println("trim CHANNEL MULTIPLIER | trim reset (idle/disarmed; "
                 "0.5..1.5; defaults 1; no init needed)");
  Serial.printf("Trims affect drive/twist/square and previews; raw "
                "motor/PWM/original tests bypass them. Shared wheel cap=%d.\n",
                maxDriveCommand);
  Serial.println("wheel CHANNEL ANGLE_DEG SIGN (geometry only; tangent angles "
                 "from FORWARD toward LEFT; sign +/-1)");
  showOrientation();
  Serial.println("drive/twist X/Y/W are weights. Changing "
                 "orientation does not fix heading drift.");
  Serial.println("Geometry defaults 150/30/270,+1 assume reference "
                 "R1/R2/R3=M1/M2/M3; mapping UNKNOWN.");
  Serial.println("profile TYPE POLARITY (TYPE 1 or 2; POLARITY 0 or 1)");
  Serial.println("transport block|single | bus 25000|50000|100000 | gap 0..10");
  Serial.println("mode speed|pwm (disarmed only; init afterward; PWM always "
                 "uses a full block)");
  Serial.println("Change settings only while disarmed; init required after "
                 "edits. Settings reset at boot.");
  Serial.printf("motor CHANNEL SPEED MS (channel 1..3; signed speed magnitude "
                "1..%d; time %u..%ums)\n",
                legacy::maxCommand, motion::minDurationMs,
                legacy::maxDurationMs);
  Serial.println("127 is a signed-byte encoding limit, NOT a measured "
                 "achievable speed. Defaults remain 20; increase gradually.");
  Serial.println("original  (after arm: old LOGICAL pattern +58,-58,0; wire "
                 "+58,+58,0; 3000ms ONCE)");
  Serial.println(
      "Speed units are pulses/10ms, NOT percent. Start: motor 1 10 1000");
  Serial.println("In PWM mode motor CHANNEL DUTY MS uses percent, +/-1..50; "
                 "refreshed every 20ms.");
}

void showRobotProfile() {
  Serial.printf(
      "ROBOT=%s clientId=%s commandTopic=%s bootProfile=%s "
      "B-inherits-unvalidated=%d spinSpeed=%d (UNTESTED) spinActive=%d\n",
      robot::selected.identity, robot::selected.mqttClientId,
      robot::selected.commandTopic, robot::selected.square.revision,
      robot::selected.inheritsUnvalidated, robot::selected.spinSpeed,
      spinActive);
}
void showStatus() {
  showRobotProfile();
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
  Serial.printf("Speed limits: drive/twist/square=1..%d; raw motor=-%d..%d; "
                "PWM duty=-%d..%d (encoding/test limits, not "
                "motor ratings).\n",
                motion::maxSpeed, legacy::maxCommand, legacy::maxCommand,
                legacy::maxPwmDuty, legacy::maxPwmDuty);
  Serial.printf("mixer=%s; orientation=%s; weights, NOT distances\n",
                mixerName(), orientationName());
  showOrientation();
  showHardwareSigns();
  showMotorTrims();
  Serial.printf(
      "squareActive=%d leg=%u/4 pausing=%d sideMs=%u pauseMs=%u\n",
      squarePattern.active, squarePattern.active ? squarePattern.leg + 1 : 0,
      squarePattern.pausing, squarePattern.sideMs, squarePattern.pauseMs);
  Serial.printf("squareCycles=%lu cycleLimit=%lu (0=endless); no "
                "controller-loss heartbeat\n",
                (unsigned long)squarePattern.cycles,
                (unsigned long)squarePattern.cycleLimit);
  showSquareTuning();
  showSquareStats();
  showWriteFault();
  for (unsigned i = 0; i < 3; ++i)
    Serial.printf("M%u tangentAngle=%.1f sign=%d (geometry only; from forward "
                  "toward left; RAM)\n",
                  i + 1, double(wheelAngle[i]), wheelSign[i]);
}

// Validate again at this boundary: even C++ callers cannot bypass
// arm/fault/limits. Internal primitive also used by the pattern scheduler.
// Public startTest and driveRobot reject an active square, including a corner's
// zero-output pause.
static bool beginTimedOutput(int a, int b, int c, unsigned milliseconds) {
  if (!configured || !enabled || broken || running ||
      milliseconds < motion::minDurationMs || stopRetries ||
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
  Serial.printf("REQUEST: M1=%d M2=%d M3=%d %s, %ums\n", a, b, c,
                pwmMode ? "percent PWM" : "pulses/10ms", milliseconds);
  Serial.printf("DRIVER: M1=%d M2=%d M3=%d (hardware signs applied)\n",
                driverCommand(0, a), driverCommand(1, b), driverCommand(2, c));
  if (!sendSpeeds(a, b, c)) {
    // Attempt zero even though command acknowledgment failed. The packet
    // might have partly reached the driver; a fault must not mean "keep going".
    broken = true;
    stopTest("FAULT: test command failed; disarmed.");
    return false;
  }
  // Start at the acknowledged transaction, not before REQUEST/DRIVER logging
  // or the I2C call. Include post-write logging/gap in elapsed running time.
  began = lastWriteEnded;
  lastReport = lastPwmRefresh = began;
  testMs = milliseconds;
  running = true;
  activeOutputs[0] = a;
  activeOutputs[1] = b;
  activeOutputs[2] = c;
  Serial.println(
      "STARTED: command acknowledged. Observe wheel; ! stops early.");
  return true;
}

bool startTest(int a, int b, int c, unsigned milliseconds) {
  if (squarePattern.active)
    return false;
  return beginTimedOutput(a, b, c, milliseconds);
}

// Public C++ entry point, sharing the serial commands' limits and state checks.
// Uses the selected orientation; lab has +X right/+Y forward. Starts ONE test.
// true means accepted, not measured physical motion. Keep loop() running; do
// not call every iteration or follow with delay(). init + arm are still
// required.
bool driveRobot(float x, float y, float w = 0,
                unsigned milliseconds = motion::defaultDriveMs,
                int speed = motion::defaultSpeed) {
  int outputs[3] = {};
  if (pwmMode || !computeWheelCommands(x, y, w, speed, outputs))
    return false;
  return startTest(outputs[0], outputs[1], outputs[2], milliseconds);
}

// Starts timed squares: forward -> right -> backward -> left, W=0 each leg.
// cycleLimit defaults to one; zero explicitly selects endless operation.
// Call once after init + arm, then keep loop() running. No blocking delay and
// no arm is needed between legs. Uses the selected mixer; no physical units
// assumed. 1000ms pauses allow visible stopped corners; they have ALSO faulted
// in the lab. This is an experimental schedule, not an established
// communication fix.
bool driveSquare(unsigned sideMs = motion::defaultSquareMs,
                 int speed = motion::defaultSpeed,
                 unsigned pauseMs = motion::defaultPauseMs,
                 uint32_t cycleLimit = 1) {
  if (pwmMode || squarePattern.active || !configured || !enabled || broken ||
      running || stopRetries)
    return false;
  int planned[4][3] = {};
  unsigned durations[4] = {};
  if (cycleLimit > 1000000 ||
      !planSquare(sideMs, speed, pauseMs, planned, durations))
    return false;
  squarePattern = SquarePattern{};
  squarePattern.active = true;
  squarePattern.sideMs = sideMs;
  squarePattern.pauseMs = pauseMs;
  squarePattern.cycleLimit = cycleLimit;
  squarePattern.attemptOpen = true;
  for (unsigned i = 0; i < 4; ++i)
    squarePattern.durations[i] = durations[i];
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 3; ++j)
      squarePattern.outputs[i][j] = planned[i][j];
  ++squareStats.attempts;
  // Establish context BEFORE leg 1, including failures of its first write.
  if (!beginTimedOutput(planned[0][0], planned[0][1], planned[0][2],
                        durations[0])) {
    if (squarePattern.active)
      stopTest("Square start rejected; cancelled.");
    return false;
  }
  Serial.printf(
      "SQUARE mixer=%s: leg 1/4 forward; base=%ums, actual legs=%u/%u/%u/%ums, "
      "pause=%ums, cycleLimit=%lu (0=endless). Timed, not measured.\n",
      mixerName(), sideMs, durations[0], durations[1], durations[2],
      durations[3], pauseMs, (unsigned long)cycleLimit);
  return true;
}

// The same guarded arm path serves diagnostics and explicit one-word starts.
static bool armMovement() {
  if (!configured || broken || stopRetries || running || squarePattern.active ||
      spinActive)
    return false;
  if (!sendSpeeds()) {
    broken = true;
    stopTest("FAULT: arm zero write.");
    return false;
  }
  enabled = true;
  Serial.println("ARMED; no motion yet.");
  return true;
}

const char *robot::resultName(robot::Result result) {
  switch (result) {
  case Result::started:
    return "STARTED";
  case Result::stopped:
    return "STOPPED; disarmed";
  case Result::queued:
    return "QUEUED";
  case Result::unknown:
    return "ERR: unknown command; expected square, stop, spin, init, status";
  case Result::notReady:
    return "ERR: successful init required";
  case Result::busy:
    return "ERR: busy; stop first";
  case Result::invalidSettings:
    return "ERR: invalid settings; use lab/legacy, calibrated driver settings "
           "and unity global trims";
  case Result::ready:
    return "READY; disarmed";
  case Result::reported:
    return "STATUS";
  case Result::fault:
    return "FAULT: latched; inspect status; explicit init required, no "
           "automatic recovery";
  }
  return "ERR";
}

bool initializeRobotMovement() {
  if (enabled || running || squarePattern.active || spinActive || stopRetries ||
      pendingRobotCommand.load() == robot::Command::stop ||
      maintenanceBusy.exchange(true))
    return false;
  clearPendingAction();
  // Preserve the old fault snapshot for MQTT before explicit init clears it.
  publishRobotState(true);
  bool success = false;
  if (!sendSpeeds()) {
    configured = false;
    broken = true;
    stopTest("FAULT: init preflight zero failed; configuration not attempted.");
  } else if (pendingRobotCommand.load() == robot::Command::stop) {
    stopTest("Initialization cancelled by stop; remains disarmed.");
  } else {
    initialize();
    success = configured && !broken;
    if (pendingRobotCommand.load() == robot::Command::stop) {
      stopTest("Stop received during initialization; remains disarmed.");
      success = false;
    }
  }
  clearPendingAction(); // No start received before/during recovery may run
                        // later.
  maintenanceBusy.store(false);
  publishRobotState(true);
  return success;
}
void stopRobotMovement() { stopTest("STOPPED; disarmed."); }

robot::Result queueRobotCommand(const char *bytes, std::size_t length) {
  const auto cmd = robot::parseCommand(bytes, length);
  if (cmd == robot::Command::none)
    return robot::Result::unknown;
  if (cmd == robot::Command::status) {
    pendingStatusRequest.store(true);
    return robot::Result::queued;
  }
  if (cmd == robot::Command::stop) {
    pendingRobotCommand.store(
        cmd); // Replace every queued start; never wait for space.
    return robot::Result::queued;
  }
  if (maintenanceBusy.load())
    return robot::Result::busy;
  auto empty = robot::Command::none;
  return pendingRobotCommand.compare_exchange_strong(empty, cmd)
             ? robot::Result::queued
             : robot::Result::busy;
}
void onRobotConnectionLost() {
  pendingRobotCommand.store(robot::Command::stop);
}

robot::Result handleRobotCommand(const char *bytes, std::size_t length) {
  using robot::Result;
  const auto cmd = robot::parseCommand(bytes, length);
  if (cmd == robot::Command::none)
    return Result::unknown;
  if (cmd == robot::Command::status) {
    publishRobotState(true);
    return Result::reported;
  }
  if (cmd == robot::Command::stop) {
    stopRobotMovement();
    return broken ? Result::fault : Result::stopped;
  }
  if (cmd == robot::Command::init) {
    if (enabled || running || squarePattern.active || spinActive ||
        stopRetries || maintenanceBusy.load() ||
        pendingRobotCommand.load() == robot::Command::stop)
      return Result::busy;
    const bool success = initializeRobotMovement();
    return success ? Result::ready : (broken ? Result::fault : Result::busy);
  }
  if (broken || stopRetries)
    return Result::fault;
  if (!configured)
    return Result::notReady;
  if (running || squarePattern.active || spinActive || maintenanceBusy.load() ||
      pendingRobotCommand.load() == robot::Command::stop)
    return Result::busy;
  if (pwmMode || singleWrites || selectedMixer != Mixer::legacy ||
      selectedOrientation != DriveOrientation::lab ||
      selectedType != legacy::type || selectedPolarity != legacy::polarity ||
      selectedClock != legacy::busHz || writeGapMs)
    return Result::invalidSettings;
  for (float trim : motorTrim)
    if (trim != 1)
      return Result::invalidSettings;
  // Validate the entire plan BEFORE arming or writing any nonzero output.
  int planned[4][3] = {};
  unsigned durations[4] = {};
  int spin[3] = {};
  const auto &profile = robot::selected.square;
  if (cmd == robot::Command::square) {
    if (!planSquare(profile.sideMs, profile.speed, profile.pauseMs, planned,
                    durations))
      return Result::invalidSettings;
  } else if (robot::selected.spinSpeed < 1 || robot::selected.spinSpeed > 20 ||
             !computeWheelCommands(0, 0, 1, robot::selected.spinSpeed, spin))
    return Result::invalidSettings;
  if (!armMovement())
    return broken ? Result::fault : Result::notReady;
  if (pendingRobotCommand.load() == robot::Command::stop) {
    stopRobotMovement();
    return broken ? Result::fault : Result::stopped;
  }
  const bool started =
      cmd == robot::Command::square
          ? driveSquare(profile.sideMs, profile.speed, profile.pauseMs, 0)
          : startTest(spin[0], spin[1], spin[2], legacy::maxDurationMs);
  if (!started) {
    if (!broken)
      stopRobotMovement();
    return broken ? Result::fault : Result::invalidSettings;
  }
  spinActive = cmd == robot::Command::spin;
  return Result::started;
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
  if (count == 1 && (!strcmp(cmd, "square") || !strcmp(cmd, "spin") ||
                     !strcmp(cmd, "stop") || !strcmp(cmd, "init"))) {
    Serial.println(robot::resultName(handleRobotCommand(cmd)));
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
  if (!strcmp(cmd, "square-stats")) {
    if (count == 1) {
      showSquareStats();
      return;
    }
    if (count == 2 && !strcmp(words[1], "reset") && !enabled && !running &&
        !squarePattern.active && !stopRetries) {
      squareStats = SquareStats{};
      showSquareStats();
      return;
    }
    Serial.println("ERR: square-stats [reset]; reset requires idle/disarmed, "
                   "no pending stop retries.");
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
  if (running || squarePattern.active) {
    Serial.println("ERR: busy (motion or square pause); stop or wait first.");
    return;
  }
  if (!strcmp(cmd, "orientation") && count == 2) {
    bool success = false;
    if (!strcmp(words[1], "lab"))
      success = setOrientation(DriveOrientation::lab);
    else if (!strcmp(words[1], "v9"))
      success = setOrientation(DriveOrientation::v9);
    if (!success) {
      Serial.println("ERR: orientation lab|v9; idle/disarmed, no pending stop "
                     "retries required.");
      return;
    }
    showOrientation();
    Serial.println("Orientation stored in RAM; no driver write or init needed. "
                   "Reset restores lab.");
    return;
  }
  if (!strcmp(cmd, "trim")) {
    long channel = 0;
    float multiplier = 0;
    const bool success = count == 2 && !strcmp(words[1], "reset")
                             ? resetMotorTrims()
                             : count == 3 && parseInt(words[1], channel) &&
                                   channel >= 1 && channel <= 3 &&
                                   parseFloat(words[2], multiplier) &&
                                   setMotorTrim(unsigned(channel), multiplier);
    if (!success) {
      Serial.println("ERR: trim CHANNEL 0.5..1.5 or trim reset; idle/disarmed, "
                     "no pending stop retries required.");
      return;
    }
    showMotorTrims();
    Serial.println("Trim stored in RAM; reset loses edits. No driver write; no "
                   "init needed.");
    return;
  }
  if (!strcmp(cmd, "square-leg")) {
    if (enabled || stopRetries) {
      Serial.println(
          "ERR: square-leg requires idle/disarmed, no pending stop retries.");
      return;
    }
    if (count == 2 && !strcmp(words[1], "reset")) {
      for (auto &tuning : squareTuning)
        tuning = SquareLegTuning{};
    } else if (count != 1) {
      const int leg = count == 6 ? squareLegIndex(words[1]) : -1;
      SquareLegTuning candidate;
      bool valid = leg >= 0;
      if (valid) {
        valid = parseFloat(words[2], candidate.timeScale) &&
                candidate.timeScale >= 0.5f && candidate.timeScale <= 1.5f;
        for (unsigned i = 0; i < 3; ++i)
          valid = parseFloat(words[i + 3], candidate.trim[i]) &&
                  candidate.trim[i] >= 0.5f && candidate.trim[i] <= 1.5f &&
                  valid;
      }
      if (!valid) {
        Serial.println("ERR: square-leg DIRECTION TIME_SCALE M1 M2 M3; "
                       "direction forward/right/backward/left; all factors "
                       "0.5..1.5; or square-leg [reset].");
        return;
      }
      squareTuning[leg] = candidate;
    }
    showSquareTuning();
    return;
  }
  if (!strcmp(cmd, "square-test")) {
    const int leg = count >= 2 ? squareLegIndex(words[1]) : -1;
    long sideMs = motion::defaultSquareMs, speed = motion::defaultSpeed;
    int planned[4][3] = {};
    unsigned durations[4] = {};
    if (count < 2 || count > 4 || leg < 0 ||
        (count >= 3 && !parseInt(words[2], sideMs)) ||
        (count == 4 && !parseInt(words[3], speed)) ||
        sideMs < motion::minDurationMs || sideMs > legacy::maxDurationMs ||
        speed < 1 || speed > motion::maxSpeed ||
        !planSquare(unsigned(sideMs), int(speed), 0, planned, durations)) {
      Serial.println("ERR: square-test DIRECTION [SIDE_MS [SPEED]]; check "
                     "direction, limits and all tuned leg durations.");
      return;
    }
    if (pwmMode || !startTest(planned[leg][0], planned[leg][1], planned[leg][2],
                              durations[leg]))
      Serial.println("ERR: square-test requires speed mode, successful init "
                     "and separate arm.");
    return;
  }
  const bool previewSquare = !strcmp(cmd, "preview-square");
  const bool endlessSquare = !strcmp(cmd, "square-endless");
  const bool repeatedSquare = !strcmp(cmd, "square-repeat");
  if (!strcmp(cmd, "square") || previewSquare || endlessSquare ||
      repeatedSquare) {
    const unsigned offset = repeatedSquare ? 1 : 0;
    long cycleLimit = endlessSquare ? 0 : 1;
    if (count < 1 + offset || count > 4 + offset ||
        (repeatedSquare &&
         (!parseInt(words[1], cycleLimit) || cycleLimit < 1))) {
      Serial.println("ERR: square[-endless]/preview-square [SIDE_MS [SPEED "
                     "[PAUSE_MS]]] or square-repeat COUNT [SIDE_MS [SPEED "
                     "[PAUSE_MS]]]; COUNT 1..1000000.");
      return;
    }
    long sideMs = motion::defaultSquareMs, speed = motion::defaultSpeed,
         pauseMs = motion::defaultPauseMs;
    int planned[4][3] = {};
    unsigned durations[4] = {};
    if ((count >= 2 + offset && !parseInt(words[1 + offset], sideMs)) ||
        (count >= 3 + offset && !parseInt(words[2 + offset], speed)) ||
        (count == 4 + offset && !parseInt(words[3 + offset], pauseMs)) ||
        sideMs < motion::minDurationMs || sideMs > legacy::maxDurationMs ||
        speed < 1 || speed > motion::maxSpeed || pauseMs < 0 ||
        pauseMs > motion::maxPauseMs ||
        !planSquare(unsigned(sideMs), int(speed), unsigned(pauseMs), planned,
                    durations)) {
      Serial.printf(
          "ERR: square/preview-square [SIDE_MS [SPEED [PAUSE_MS]]]; side "
          "%u..%u; speed 1..%d; pause 0..%u; nonzero legs required.\n",
          motion::minDurationMs, legacy::maxDurationMs, motion::maxSpeed,
          motion::maxPauseMs);
      return;
    }
    if (previewSquare) {
      if (enabled || stopRetries) {
        Serial.println("ERR: preview-square requires idle/disarmed, no pending "
                       "stop retries.");
        return;
      }
      showMotorTrims();
      showSquareTuning();
      const unsigned total = durations[0] + durations[1] + durations[2] +
                             durations[3] + 3 * unsigned(pauseMs);
      Serial.printf("SQUARE PREVIEW mixer=%s orientation=%s side=%ldms "
                    "speed=%ld pause=%ldms total=%ldms; NO driver writes.\n",
                    mixerName(), orientationName(), sideMs, speed, pauseMs,
                    long(total));
      Serial.printf("Repeating cycle interval=%ums including fourth-corner "
                    "pause; final bounded cycle omits that last pause.\n",
                    total + unsigned(pauseMs));
      for (unsigned i = 0; i < 4; ++i)
        Serial.printf("Leg %u/4 %s X=%.0f Y=%.0f W=0: M1=%d M2=%d M3=%d "
                      "pulses/10ms; DRIVER M1=%d M2=%d M3=%d; duration=%ums\n",
                      i + 1, squareNames[i], double(squareX[i]),
                      double(squareY(i)), planned[i][0], planned[i][1],
                      planned[i][2], driverCommand(0, planned[i][0]),
                      driverCommand(1, planned[i][1]),
                      driverCommand(2, planned[i][2]), durations[i]);
    } else if (!driveSquare(unsigned(sideMs), int(speed), unsigned(pauseMs),
                            uint32_t(cycleLimit)))
      Serial.println("ERR: square rejected; speed mode, successful init and "
                     "separate arm required.");
    return;
  }
  if (!strcmp(cmd, "mixer") && count == 2) {
    if (enabled || stopRetries) {
      Serial.println(
          "ERR: mixer edits require idle/disarmed, no pending stop retries.");
      return;
    }
    if (!strcmp(words[1], "legacy"))
      selectedMixer = Mixer::legacy;
    else if (!strcmp(words[1], "original"))
      selectedMixer = Mixer::original;
    else if (!strcmp(words[1], "geometry"))
      selectedMixer = Mixer::geometry;
    else {
      Serial.println("ERR: mixer legacy|original|geometry.");
      return;
    }
    Serial.printf("Mixer=%s stored in RAM; no driver write; orientation=%s.\n",
                  mixerName(), orientationName());
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
    Serial.println("Wheel geometry stored in RAM; applies only to mixer "
                   "geometry. No driver write; reset loses edits.");
    return;
  }
  if (!strcmp(cmd, "preview") && count >= 3 && count <= 5) {
    float x = 0, y = 0, w = 0;
    long speed = motion::defaultSpeed;
    int outputs[3] = {};
    if (enabled || stopRetries) {
      Serial.println(
          "ERR: preview requires idle/disarmed, no pending stop retries.");
      return;
    }
    if (!parseFloat(words[1], x) || !parseFloat(words[2], y) ||
        (count >= 4 && !parseFloat(words[3], w)) ||
        (count == 5 && !parseInt(words[4], speed)) ||
        !computeWheelCommands(x, y, w, int(speed), outputs)) {
      Serial.printf(
          "ERR: preview X Y [W [SPEED]]; weights -1..1; speed 1..%d.\n",
          motion::maxSpeed);
      return;
    }
    showMotorTrims();
    Serial.printf(
        "PREVIEW mixer=%s orientation=%s X=%.2f Y=%.2f W=%.2f scale=%ld: M1=%d "
        "M2=%d M3=%d pulses/10ms; NO driver write.\n",
        mixerName(), orientationName(), double(x), double(y), double(w), speed,
        outputs[0], outputs[1], outputs[2]);
    Serial.printf("PREVIEW DRIVER: M1=%d M2=%d M3=%d; NO driver write.\n",
                  driverCommand(0, outputs[0]), driverCommand(1, outputs[1]),
                  driverCommand(2, outputs[2]));
    return;
  }
  const bool twist = !strcmp(cmd, "twist");
  if ((!strcmp(cmd, "drive") && count >= 3 && count <= 5) ||
      (twist && count >= 4 && count <= 6)) {
    float x = 0, y = 0, w = 0;
    long milliseconds = motion::defaultDriveMs, speed = motion::defaultSpeed;
    const unsigned timeIndex = twist ? 4 : 3;
    if (pwmMode || !parseFloat(words[1], x) || !parseFloat(words[2], y) ||
        (twist && !parseFloat(words[3], w)) ||
        (count > timeIndex && !parseInt(words[timeIndex], milliseconds)) ||
        (count > timeIndex + 1 && !parseInt(words[timeIndex + 1], speed)) ||
        milliseconds < motion::minDurationMs ||
        milliseconds > legacy::maxDurationMs || speed < 1 ||
        speed > motion::maxSpeed || std::fabs(x) > 1 || std::fabs(y) > 1 ||
        std::fabs(w) > 1) {
      Serial.printf(
          "ERR: drive X Y [MS [SPEED]] or twist X Y W [MS [SPEED]]; weights "
          "-1..1; time %u..%ums; speed 1..%d; speed mode only.\n",
          motion::minDurationMs, legacy::maxDurationMs, motion::maxSpeed);
      return;
    }
    Serial.printf("Drive mixer=%s; orientation=%s.\n", mixerName(),
                  orientationName());
    if (!driveRobot(x, y, w, unsigned(milliseconds), int(speed)))
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
  if (count == 1 && !strcmp(cmd, "arm")) {
    if (!armMovement())
      Serial.println("ERR: successful init required.");
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
        speed < -limit || speed > limit || speed == 0 ||
        milliseconds < motion::minDurationMs ||
        milliseconds > legacy::maxDurationMs) {
      Serial.printf("ERR: channel 1..3; nonzero magnitude 1..%d in current "
                    "mode; time %u..%ums.\n",
                    limit, motion::minDurationMs, legacy::maxDurationMs);
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
  if (spinActive) {
    if (!configured || !enabled || broken || pwmMode || !running)
      stopTest("Spin cancelled: motion state changed.");
    return; // Speed register holds the guarded request until explicit
            // stop/fault.
  }
  if (running) {
    if (now - began >= testMs) {
      if (!squarePattern.active) {
        stopTest("DONE; disarmed.");
        return;
      }
      if (squarePattern.leg == 3 && squarePattern.cycleLimit &&
          squarePattern.cycles + 1 >= squarePattern.cycleLimit) {
        stopTest("SQUARE last leg ended; disarmed.", true);
        if (!broken)
          Serial.println(
              "SQUARE DONE: four timed legs finished. Position not measured.");
        else
          Serial.println(
              "FAULT: square ending stop failed; pattern cancelled.");
        return;
      }
      // A corner must get an acknowledged zero before the next leg. On
      // failure stopTest cancels the ENTIRE sequence, including retries.
      if (!sendSpeeds()) {
        broken = true;
        stopTest("FAULT: square corner stop failed; pattern cancelled.");
        return;
      }
      running = false;
      for (int &value : activeOutputs)
        value = 0;
      squarePattern.pausing = true;
      squarePattern.pauseBegan =
          lastWriteEnded; // Stop acknowledgment, before logging/gap.
      if (squarePattern.leg == 3) {
        recordSquareCompletion();
        showSquareStats();
        Serial.printf("SQUARE CYCLE %lu complete; paused %ums before next "
                      "forward. Position not measured.\n",
                      (unsigned long)squarePattern.cycles,
                      squarePattern.pauseMs);
      } else
        Serial.printf("SQUARE leg %u/4 ended; pause %ums.\n",
                      squarePattern.leg + 1, squarePattern.pauseMs);
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
  } else if (squarePattern.active) {
    if (!enabled || !configured || broken || pwmMode) {
      stopTest("Square cancelled: motion state changed.");
      return;
    }
    if (now - squarePattern.pauseBegan >= squarePattern.pauseMs) {
      squarePattern.leg = (squarePattern.leg + 1) % 4;
      if (squarePattern.leg == 0) {
        ++squareStats.attempts;
        squarePattern.attemptOpen = true;
      }
      squarePattern.pausing = false;
      const int *output = squarePattern.outputs[squarePattern.leg];
      if (!beginTimedOutput(output[0], output[1], output[2],
                            squarePattern.durations[squarePattern.leg])) {
        if (squarePattern.active)
          stopTest("FAULT: square next leg rejected; pattern cancelled.");
        return;
      }
      Serial.printf("SQUARE leg %u/4 %s.\n", squarePattern.leg + 1,
                    squareNames[squarePattern.leg]);
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

#if defined(SWRM_ENABLE_LED_PANEL) && SWRM_ENABLE_LED_PANEL
// Declare an instance of a direct memory access display for writing to the LED
// board
MatrixPanel_I2S_DMA *dma_display = nullptr;

#endif
void setup() {
  Serial.begin(legacy::baud);
  Wire.begin(legacy::sda, legacy::scl);
  Wire.setClock(legacy::busHz);
  Wire.setTimeOut(legacy::transactionTimeoutMs);
  delay(100); // Original setup's initial I2C pause.
#ifdef SWRM_ENABLE_MQTT
  // Battery-only startup: configure once and request zero, never arm/start.
  // Share the zero-first guarded init used by deliberate recovery. A failure
  // remains latched. Reconnect paths never call initialization.
  Serial.println("BOOT INIT: one driver configuration attempt; no motion.");
  initializeRobotMovement();
  Serial.println(
      configured && !broken
          ? "BOOT READY: initialized, DISARMED; awaiting an explicit command."
          : "BOOT FAULT: movement disabled; inspect driver/power before "
            "recovery.");
#else
  Serial.println("DISARMED. Enter init to test original driver settings.");
#endif
  showRobotProfile();
  showHelp(); // Read-only instructions; initialization above never arms/moves.
#if defined(SWRM_ENABLE_LED_PANEL) && SWRM_ENABLE_LED_PANEL
  // Begin Initializing LED board

  // init pin connections between esp32 and the LED board
  HUB75_I2S_CFG::i2s_pins _pins = {
      legacy::r1, legacy::g1,  legacy::b1, legacy::r2, legacy::g2,
      legacy::b2, legacy::a,   legacy::b,  legacy::c,  legacy::d,
      legacy::e,  legacy::lat, legacy::oe, legacy::clk};
  delay(10);

  // Configuration for the LED panel
  HUB75_I2S_CFG mxconfig(legacy::res_x,       // Module width
                         legacy::res_y,       // Module height
                         legacy::panel_chain, // Chain length
                         _pins                // Pin mapping
  );

  // set I2S clock speed
  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_10M;

  // Board driver
  mxconfig.driver = HUB75_I2S_CFG::FM6124;

  // Do not use clock phase
  mxconfig.clkphase = false;

  // Display Setup
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);

  dma_display->begin();

  // 0-255
  // RGB values for certain color we want
  dma_display->setBrightness8(100);
  dma_display->clearScreen();

  // Fill the screen with white pixels
  dma_display->fillScreen(dma_display->color565(255, 255, 255));
#endif
  beginRobotNetwork(); // Networking alone never initializes or arms motors.
  publishRobotState(true);
}

// Snapshot only from the motor loop; network code never reads live motor
// globals.
robot::Status getRobotStatus() {
  robot::Status value;
  value.configured = configured;
  value.armed = enabled;
  value.running = running;
  value.fault = broken;
  value.square = squarePattern.active;
  value.paused = squarePattern.active && squarePattern.pausing;
  value.spin = spinActive;
  value.retries = stopRetries;
  value.error = lastError;
  value.reg = lastReg;
  value.firstError = firstWriteFault.valid ? firstWriteFault.error : 0;
  value.firstReg = firstWriteFault.valid ? firstWriteFault.reg : 0;
  value.faultLeg = firstWriteFault.valid ? firstWriteFault.leg : 0;
  value.cycles = squareStats.completed;
  value.ready =
      configured && !broken && !enabled && !running && !squarePattern.active &&
      !spinActive && !stopRetries && !maintenanceBusy.load() && !pwmMode &&
      !singleWrites && selectedMixer == Mixer::legacy &&
      selectedOrientation == DriveOrientation::lab &&
      selectedType == legacy::type && selectedPolarity == legacy::polarity &&
      selectedClock == legacy::busHz && !writeGapMs;
  for (float trim : motorTrim)
    if (trim != 1)
      value.ready = false;
  return value;
}
static void publishRobotState(bool force) {
  static robot::Status previous;
  static bool havePrevious = false;
  static uint32_t sequence = 0;
  auto value = getRobotStatus();
  const bool changed =
      !havePrevious || value.ready != previous.ready ||
      value.configured != previous.configured ||
      value.armed != previous.armed || value.running != previous.running ||
      value.fault != previous.fault || value.square != previous.square ||
      value.paused != previous.paused || value.spin != previous.spin ||
      value.retries != previous.retries || value.error != previous.error ||
      value.reg != previous.reg || value.firstError != previous.firstError ||
      value.firstReg != previous.firstReg ||
      value.faultLeg != previous.faultLeg || value.cycles != previous.cycles;
  if (force || changed) {
    value.sequence = ++sequence;
    previous = value;
    havePrevious = true;
    reportRobotState(value);
  }
}

// Stop/action inbox is processed before scheduling; status requests cannot
// consume, replace or delay a queued stop or init/start action.
void updateRobotMovement() {
  const auto pending = pendingRobotCommand.exchange(robot::Command::none);
  if (pending != robot::Command::none) {
    const auto result = handleRobotCommand(robot::commandName(pending));
    Serial.println(robot::resultName(result));
    reportRobotCommandResult(pending, result);
  }
  updateTest();
  publishRobotState(pendingStatusRequest.exchange(false));
}
void loop() {
  pollInput();
  updateRobotMovement();
  delay(1);
}
