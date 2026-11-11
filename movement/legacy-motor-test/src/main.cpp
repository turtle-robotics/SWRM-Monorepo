/*
V10 MONOREPO LAB: observed-axis adapter; +X right, +Y forward, +W clockwise.
`orientation v9` restores the prior input mapping for exact comparisons.
Lab wheel map reported by user: M1 rear-left, M2 front, M3 rear-right.
The adapter rotates the working legacy frame to that front. Pure lab forward
leaves M2 at zero. Rolling signs/accuracy still need testing; no heading hold.
Teammate's M2 adapter, wheel models, trims, previews and timed squares remain.
Boot mixer=legacy preserves the teammate's V5 wheel model with an axis adapter.
Original mode preserves the old SWRM_Leds coefficients. Geometry mode is adjustable
and provisional; reference drawing labels are NOT verified motor channels.
See ../docs/LAB_RUNBOOK.md for exact Mac commands, models, and lab test options.
Current boot defaults: motor type 1, encoder polarity 0, block speed writes.
The user verified independent M1/M2/M3, reverse M2 and M2 at speed 30 with
these settings. Geometry below remains provisional. Historical context follows:
Motor comparison V4: original speed interface plus refreshed open-loop PWM
==================================================
The original demo starts motion automatically and repeats it. This program keeps
its important driver values, but requires init -> arm -> a test command and runs
each test only once. Speed mode uses register 51; explicit PWM mode uses register
31 with repeated full snapshots. Neither mode sends periodic idle writes.

EXACT VALUES FROM THE ORIGINAL SOURCE:
  device 0x34; SDA=21; SCL=22; 100kHz; motor-type register 20 receives 1;
  encoder-polarity register 21 receives 1; speed register 51 gets four bytes;
  driveRobot(0,1,0) produces about +58,-58,0,0, for 3000ms, then speed zeros.

The original calls type 1 "N20"; current Hiwonder docs call N20 type 2. Here 1 is
an explicit COMPATIBILITY TEST, not a claim that the docs are wrong for all boards.
Speed values are pulses per 10ms, not percentages. +58 can be fast. Start with a
single wheel at +10 and keep every wheel raised for this bench experiment.

Power-cycle the USB-only setup before uploading; this program cannot guarantee a
stop from a previously powered driver if communication is broken. It does not
automatically start motors on boot, init, help or status. `!` sends a software
stop, but physical power removal is the fallback if that write fails.
*/
#include <Arduino.h>
#include <Wire.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath> // Dot-product wheel mixing uses sin/cos; inputs are checked for NaN/Inf.
#include <initializer_list> // Enables iterating over the three candidate motor values.

namespace legacy {
constexpr uint8_t address = 0x34; // Original Hiwonder device address.
constexpr int sda = 21, scl = 22; // Original ESP32 GPIO signal pins.
constexpr unsigned busHz = 100000, baud = 115200;
constexpr uint8_t typeReg = 20, polarityReg = 21, speedReg = 51;
constexpr uint8_t type = 1, polarity = 0; // Bench-verified on this robot: independent motion and positive counts.
constexpr int maxCommand = 58; // Original active movement's largest wheel command.
constexpr unsigned maxDurationMs = 3000; // Original three-second movement interval.
constexpr unsigned transactionTimeoutMs = 50; // Arduino Wire default; original had no override.
constexpr unsigned settingDelayMs = 10; // Original pause after each configuration write.
constexpr unsigned retryIntervalMs = 250; // Our bounded attempts to deliver a failed stop.
constexpr unsigned maxStopRetries = 4; // Then remain faulted/quiet; do not flood the bus forever.
constexpr int maxPwmDuty = 50; // Chosen raised-wheel bench cap; protocol allows +/-100.
constexpr unsigned pwmRefreshMs = 20; // Chosen 50Hz refresh for continuous PWM control.
}

// State starts explicitly disabled. Configuration and arming are separate steps.
bool configured = false, enabled = false, running = false, broken = false;
// V3: pending, operator-selected comparisons. Edits are allowed only while idle
// and disarmed, and invalidate initialization. No edit automatically runs motors.
uint8_t selectedType = legacy::type, selectedPolarity = legacy::polarity;
bool singleWrites = false; // Full four-channel snapshots, verified on this robot.
bool pwmMode = false; // V4 alternative: register 31 open-loop duty, not encoder speed regulation.
int activeOutputs[3] = {}; // Snapshot for PWM refresh, reset to zeros on every stop.
uint32_t lastPwmRefresh = 0;
unsigned selectedClock = legacy::busHz;
unsigned writeGapMs = 0; // Optional settling pause AFTER a transaction, not between its bytes.
uint32_t began = 0, testMs = 0, lastReport = 0, lastRetry = 0;
unsigned stopRetries = 0; // Zero means no failed-stop retry is pending.
uint8_t lastError = 0, lastReg = 0; // Last failed write, retained for status.
char buffer[96] = {}; // Fixed command storage; no heap allocation needed.
unsigned used = 0;
bool discard = false; // Reject overlong/unsupported input through the next newline.

// One arm authorizes one four-leg square. Cache the complete plan at its start;
// normal commands cannot replace a leg or change settings during corner pauses.
struct SquarePattern {
    bool active = false, pausing = false;
    unsigned leg = 0, sideMs = 0, pauseMs = 0;
    uint32_t pauseBegan = 0;
    int outputs[4][3] = {};
};
SquarePattern squarePattern;
constexpr float squareX[4] = {0,1,0,-1};
constexpr float squareV9Y[4] = {-1,0,1,0};
const char* const squareNames[4] = {"forward", "right", "backward", "left"};

enum class Mixer { legacy, original, geometry };
Mixer selectedMixer = Mixer::legacy; // Start with the teammate's working model.
const char* mixerName() {
    if (selectedMixer==Mixer::legacy) return "legacy";
    return selectedMixer==Mixer::original ? "original" : "geometry";
}

enum class DriveOrientation { lab, v9 };
DriveOrientation selectedOrientation = DriveOrientation::lab;
const char* orientationName() { return selectedOrientation==DriveOrientation::lab ? "lab" : "v9"; }
bool setOrientation(DriveOrientation orientation) {
    if (enabled || running || squarePattern.active || stopRetries
        || (orientation!=DriveOrientation::lab && orientation!=DriveOrientation::v9)) return false;
    selectedOrientation=orientation;
    return true;
}
void showOrientation() {
    if (selectedOrientation==DriveOrientation::lab)
        Serial.println("Orientation=lab: +X intended RIGHT, +Y intended FORWARD; M1 rear-left, M2 front, M3 rear-right; no heading correction.");
    else Serial.println("Orientation=v9: previous X/Y inputs restored (formerly labeled +X right/+Y backward; physical directions differ).");
}
float squareY(unsigned leg) {
    return selectedOrientation==DriveOrientation::lab ? -squareV9Y[leg] : squareV9Y[leg];
}

// Preserve the teammate's physical channel correction ONCE at the I2C boundary.
// These are hardware output signs, separate from geometry wheelSign and positive
// motorTrim. Raw motor commands also pass through this adapter, as in GitHub V5.
constexpr int hardwareSign[3] = {1,-1,1};
int driverCommand(unsigned channelIndex, int logical) { return logical*hardwareSign[channelIndex]; }
void showHardwareSigns() {
    Serial.println("Hardware output signs: M1=+1 M2=-1 M3=+1 (applied once; raw encoders retain driver signs)");
}

// GEOMETRY MODE ONLY: reference R1 upper-left, R2 upper-right, R3 rear, assuming
// those are M1/M2/M3. This channel mapping is UNKNOWN on the physical robot.
// Angles are the CCW tangent directions from FORWARD toward LEFT (not +public X).
// Each tangent belongs to a wheel on an equal-radius circle about chassis center;
// changing an angle assumes the associated wheel position rotates with it.
// Positive CCW chassis rotation needs all three positive tangent speeds.
// Use wheelSign=-1 if positive driver speed propels against that tangent.
// `wheel CHANNEL ANGLE SIGN` affects only geometry mode, never driver polarity.
float wheelAngle[3] = {150,30,270};
int wheelSign[3] = {1,1,1};

// Empirical SPEED-request multipliers, not mass or power settings. Unity keeps
// prior behavior. Applied only to calculated drive/twist/square outputs, never
// raw motor/PWM tests or fixed `original` packets. Runtime edits need no init.
float motorTrim[3] = {1,1,1};
constexpr float minMotorTrim = 0.5f, maxMotorTrim = 1.5f; // Chosen experiment bounds.
constexpr int maxDriveCommand = 30; // Keep the prior movement ceiling after trims.

bool setMotorTrim(unsigned channel, float multiplier) {
    if (enabled || running || squarePattern.active || stopRetries || channel<1 || channel>3
        || !std::isfinite(multiplier) || multiplier<minMotorTrim || multiplier>maxMotorTrim) return false;
    motorTrim[channel-1] = multiplier;
    return true;
}

bool resetMotorTrims() {
    if (enabled || running || squarePattern.active || stopRetries) return false;
    for (float& trim : motorTrim) trim=1;
    return true;
}

void showMotorTrims() {
    Serial.printf("Speed trims: M1=%.3f M2=%.3f M3=%.3f (drive/twist/square only; RAM; wheel commands capped at %d)\n",
        double(motorTrim[0]),double(motorTrim[1]),double(motorTrim[2]),maxDriveCommand);
}

// Pure calculation: NO I2C writes, initialization, arming, or motion.
// Lab: x=right, y=forward; v9: previous input axes. w=clockwise in both.
// Normalized weights, NOT distances/radians. Orientation changes the public
// frame ONLY; all existing mixer equations below retain their V9 model frame.
// speed is a reference pulses/10ms scale. Common limiting preserves ratios
// before rounding. Original mode uses the old coefficients; at the same output
// scale, rounding differs from old truncation by at most one count command.
bool computeWheelCommands(float x, float y, float w, int speed, int (&out)[3]) {
    for (int& value : out) value = 0;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w)
        || std::fabs(x)>1 || std::fabs(y)>1 || std::fabs(w)>1
        || speed<1 || speed>30) return false; // 30 is the successfully tested speed, not a motor rating.
    // Actual channel positions: M1 rear-left, M2 front, M3 rear-right. Rotate
    // the existing model frame instead of changing its mixer or motor signs.
    // For legacy this yields M1=.5*x-.866*y, M2=-x, M3=.5*x+.866*y (W=0).
    // Therefore new +Y forward leaves FRONT M2 stopped, as ideal kinematics
    // requires. This assumes symmetric 120-degree geometry and needs floor QA.
    constexpr float root3Over2 = 0.8660254037844386f;
    const float modelX = selectedOrientation==DriveOrientation::lab ? -0.5f*x+root3Over2*y : x;
    const float modelY = selectedOrientation==DriveOrientation::lab ? -root3Over2*x-0.5f*y : y;
    float mix[3] = {}; float peak = 1; // Scale down only; small inputs remain small.
    if (selectedMixer == Mixer::original) {
        mix[0] = 0.33f*modelX + 0.58f*modelY - 0.33f*w;
        mix[1] = 0.33f*modelX - 0.58f*modelY - 0.33f*w;
        mix[2] = 0.67f*modelX               + 0.33f*w;
    } else {
        const float forward = -modelY, left = -modelX;
        constexpr float radiansPerDegree = 3.14159265358979323846f/180;
        for (unsigned i=0;i<3;++i) {
            const bool legacyModel = selectedMixer==Mixer::legacy;
            const float angleDegrees = legacyModel ? 90.0f+120.0f*i : wheelAngle[i];
            const int sign = legacyModel ? 1 : wheelSign[i];
            if (!std::isfinite(angleDegrees) || (sign!=1 && sign!=-1)) return false;
            const float angle = angleDegrees*radiansPerDegree;
            // w is an L*omega-equivalent normalized weight, CW hence minus.
            mix[i] = (std::cos(angle)*forward+std::sin(angle)*left-w)*sign;
        }
    }
    for (float value : mix) if (std::fabs(value)>peak) peak=std::fabs(value);
    float requests[3] = {}; float trimmedPeak = 0;
    for (unsigned i=0;i<3;++i) {
        if (!std::isfinite(motorTrim[i]) || motorTrim[i]<minMotorTrim || motorTrim[i]>maxMotorTrim) return false;
        requests[i] = speed*mix[i]/peak*motorTrim[i];
        if (std::fabs(requests[i])>trimmedPeak) trimmedPeak=std::fabs(requests[i]);
    }
    // If a trim pushes a wheel above 30, scale ALL requests down together to
    // preserve the trimmed ratios. Never clip wheels independently or energize
    // a wheel whose original request was zero. Rounding has finite resolution.
    const float capScale = trimmedPeak>maxDriveCommand ? maxDriveCommand/trimmedPeak : 1;
    for (unsigned i=0;i<3;++i) out[i]=int(std::lround(requests[i]*capScale));
    return true;
}

// Pure square planner: all sides must be valid/nonzero before any motor write.
// Equal elapsed times do not guarantee equal measured distances or closure.
bool planSquare(unsigned sideMs, int speed, unsigned pauseMs, int (&outputs)[4][3]) {
    if (sideMs<50 || sideMs>legacy::maxDurationMs || pauseMs>2000) return false;
    for (unsigned i=0;i<4;++i) {
        if (!computeWheelCommands(squareX[i],squareY(i),0,speed,outputs[i])
            || (!outputs[i][0] && !outputs[i][1] && !outputs[i][2])) return false;
    }
    return true;
}

// Same register-plus-byte-array transaction as the original, but report every
// write result. Numeric result 0 means acknowledgment, not verified wheel motion.
bool writeRegister(uint8_t reg, const uint8_t* data, unsigned length, bool verbose = true) {
    Wire.beginTransmission(legacy::address);
    Wire.write(reg);
    for (unsigned i = 0; i < length; ++i) Wire.write(data[i]);
    const uint8_t result = Wire.endTransmission();
    // Print the actual data too, not just the caller's intended wheel values.
    if (verbose || result) {
        Serial.printf("WRITE register=%u bytes=%u result=%u data=", unsigned(reg), length, unsigned(result));
        for (unsigned i = 0; i < length; ++i) Serial.printf("%02X ",unsigned(data[i]));
        Serial.println("");
    }
    if (writeGapMs) delay(writeGapMs);
    if (result) { lastError = result; lastReg = reg; }
    return result == 0;
}

// Same four logical channel requests as original setMotorSpeeds(). Default is a
// four-byte block; individual writes remain a diagnostic option. M4 stays zero.
bool sendSpeeds(int a = 0, int b = 0, int c = 0, bool quietRefresh = false) {
    a=driverCommand(0,a); b=driverCommand(1,b); c=driverCommand(2,c);
    if (pwmMode) {
        // Open-loop PWM uses an entirely new full four-channel snapshot each
        // time. No speed-register write follows it, so we do not switch back
        // into speed regulation at a stop. No encoder counts are used here.
        const uint8_t data[4] = {uint8_t(int8_t(a)),uint8_t(int8_t(b)),uint8_t(int8_t(c)),0};
        return writeRegister(31,data,4,!quietRefresh);
    }
    if (!singleWrites) {
        const uint8_t data[4] = {uint8_t(int8_t(a)),uint8_t(int8_t(b)),uint8_t(int8_t(c)),0};
        return writeRegister(legacy::speedReg,data,4);
    }
    // V2 channel-isolation test: Hiwonder's official Pi example addresses one
    // motor with register 50+motor_id and a ONE-byte payload. Thus 51=M1,
    // 52=M2, 53=M3, 54=M4. These are register numbers, not I2C device addresses.
    // https://wiki.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/4_Raspberry%20Pi_formatted.html
    // The previous four-byte block was acknowledged but physically activated
    // extra motors. Its cause is not established. This bypasses its packing /
    // auto-increment assumptions rather than claiming the board is now verified.
    const int values[4] = {a,b,c,0};
    bool cleared = true;
    // Clear every unselected channel FIRST. Always try every zero write even
    // if an earlier zero fails, and never energize a channel after such a failure.
    for (unsigned i = 0; i < 4; ++i) if (values[i] == 0) {
        const uint8_t zero = 0;
        if (!writeRegister(uint8_t(legacy::speedReg+i), &zero, 1)) cleared = false;
    }
    if (!cleared) return false;
    for (unsigned i = 0; i < 4; ++i) if (values[i] != 0) {
        const uint8_t value = uint8_t(int8_t(values[i]));
        if (!writeRegister(uint8_t(legacy::speedReg+i), &value, 1)) return false;
    }
    return true;
}

// A normal stop always disables the next motion. Failure latches a fault and
// schedules a few zero-only retries; it never permits continuing a motor test.
void stopTest(const char* reason) {
    squarePattern.active = squarePattern.pausing = false; // Stop/fault never resumes a pattern.
    running = enabled = false;
    for (int& value : activeOutputs) value = 0;
    if (!sendSpeeds()) {
        broken = true;
        stopRetries = legacy::maxStopRetries;
        lastRetry = millis();
        Serial.println("FAULT: output-zero write failed. Use physical power removal if needed.");
    } else stopRetries = 0;
    Serial.println(reason);
}

// Test the original numeric settings step by step. Stop at the first failure,
// preserving which register failed. This is not an address-only connectivity check.
void initialize() {
    if (squarePattern.active) stopTest("Square cancelled by initialization.");
    configured = enabled = running = false;
    broken = false; stopRetries = 0; lastError = lastReg = 0;
    Serial.printf("INIT: type=%u polarity=%u transport=%s clock=%uHz gap=%ums\n",
        unsigned(selectedType),unsigned(selectedPolarity),singleWrites ? "single" : "block",selectedClock,writeGapMs);
    Wire.setClock(selectedClock);
    // In PWM mode first request zero duty before editing motor settings. This
    // requests leaving speed control; it is not a documented PID-state reset.
    if (pwmMode && !sendSpeeds()) { broken = true; stopTest("FAULT: initial PWM zero."); return; }
    const uint8_t type = selectedType, polarity = selectedPolarity;
    if (!writeRegister(legacy::typeReg, &type, 1)) {
        broken = true; stopTest("FAULT: original motor-type write failed."); return;
    }
    delay(legacy::settingDelayMs);
    if (!writeRegister(legacy::polarityReg, &polarity, 1)) {
        broken = true; stopTest("FAULT: original polarity write failed."); return;
    }
    delay(legacy::settingDelayMs);
    if (!sendSpeeds()) {
        broken = true; stopRetries = legacy::maxStopRetries; lastRetry = millis(); return;
    }
    configured = true;
    Serial.println("READY: selected settings acknowledged; DISARMED. No motion verified.");
}

// Read-only diagnostics: selecting a register writes only its address, never a
// motor value. Require idle/disarmed so these blocking reads cannot delay a stop.
// Register layout follows Hiwonder's register table and PX4's HiwonderEMM driver:
// reg 0 = two little-endian supply millivolt bytes; reg 60 = four signed counts.
// https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/hiwonder_emm/HiwonderEMM.cpp
bool readDiagnostic(uint8_t reg, uint8_t* data, uint8_t length) {
    if (enabled || running || stopRetries) {
        Serial.println("ERR: diagnostics require idle/disarmed; stop first."); return false;
    }
    Wire.beginTransmission(legacy::address);
    Wire.write(reg);
    const uint8_t result = Wire.endTransmission(false); // Repeated START before reading.
    if (result) {
        Serial.printf("READ register=%u select error=%u; no value available.\n",unsigned(reg),unsigned(result));
        return false;
    }
    const unsigned received = Wire.requestFrom(legacy::address,length);
    if (received != length || Wire.available() != length) {
        Serial.printf("READ register=%u short response=%u expected=%u; no value available.\n",
            unsigned(reg),received,unsigned(length));
        while (Wire.available()) Wire.read();
        return false;
    }
    for (unsigned i = 0; i < length; ++i) data[i] = uint8_t(Wire.read());
    return true;
}

void showVoltage() {
    uint8_t data[2] = {}; // Never print these initial zeros if the read fails.
    if (!readDiagnostic(0,data,sizeof(data))) return;
    const unsigned millivolts = unsigned(data[0]) | (unsigned(data[1]) << 8);
    Serial.printf("SUPPLY reported=%u mV (%.3f V); raw=%02X %02X. Board reading, not a meter measurement.\n",
        millivolts,millivolts/1000.0,unsigned(data[0]),unsigned(data[1]));
}

void showEncoders() {
    uint8_t data[16] = {}; // Four channels, four bytes each; no reset/write to counters.
    if (!readDiagnostic(60,data,sizeof(data))) return;
    Serial.printf("%s","ENC cumulative:");
    for (unsigned i = 0; i < 4; ++i) {
        const unsigned offset = 4*i;
        const uint32_t raw = uint32_t(data[offset]) | (uint32_t(data[offset+1]) << 8)
            | (uint32_t(data[offset+2]) << 16) | (uint32_t(data[offset+3]) << 24);
        // Explicit sign conversion avoids overflowing signed shifts or relying
        // on implementation-defined unsigned-to-signed conversion for negatives.
        const int64_t count = raw <= INT32_MAX ? int64_t(raw) : int64_t(raw)-4294967296LL;
        Serial.printf(" M%u=%ld",i+1,long(count));
    }
    Serial.println(" (counts; wheel distance/calibration not inferred)");
}

void showHelp() {
    Serial.println("KIWI MONOREPO LAB v10: orientation=lab (+Y forward); M2 reversed; mixer=legacy; DISARMED.");
    Serial.println("init | status | arm | stop | ! | help");
    Serial.println("voltage | enc (idle/disarmed only; read driver supply/counters; no motor command)");
    Serial.println("drive X Y [MS [SPEED]] (speed mode only; X/Y -1..1; defaults 1000ms,20 pulses/10ms; max 3000ms,30)");
    Serial.println("twist X Y W [MS [SPEED]] (same limits; W -1..1, positive clockwise)");
    Serial.println("preview X Y [W [SPEED]] (idle/disarmed; calculate only, NO driver writes)");
    Serial.println("square [SIDE_MS [SPEED [PAUSE_MS]]] (one arm, forward/right/backward/left; defaults 1200,20,1000 = 7.8s)");
    Serial.println("preview-square [SIDE_MS [SPEED [PAUSE_MS]]] (disarmed; all four legs, NO driver writes)");
    Serial.println("Square: side 50..3000ms, speed 1..30, pause 0..2000ms; timed pattern, NOT measured distance.");
    Serial.println("mixer legacy|original|geometry (idle/disarmed; RAM only; no init needed)");
    Serial.println("orientation lab|v9 (idle/disarmed; RAM only; lab = +X right/+Y forward; v9 restores old commands)");
    Serial.println("trim CHANNEL MULTIPLIER | trim reset (idle/disarmed; 0.5..1.5; defaults 1; no init needed)");
    Serial.println("Trims affect drive/twist/square and previews; raw motor/PWM/original tests bypass them. Movement cap remains 30.");
    Serial.println("wheel CHANNEL ANGLE_DEG SIGN (geometry only; tangent angles from FORWARD toward LEFT; sign +/-1)");
    showOrientation();
    Serial.println("X/Y/W are weights, NOT centimeters; changing orientation does not fix heading drift.");
    Serial.println("Geometry defaults 150/30/270,+1 assume reference R1/R2/R3=M1/M2/M3; mapping UNKNOWN.");
    Serial.println("profile TYPE POLARITY (TYPE 1 or 2; POLARITY 0 or 1)");
    Serial.println("transport block|single | bus 25000|50000|100000 | gap 0..10");
    Serial.println("mode speed|pwm (disarmed only; init afterward; PWM always uses a full block)");
    Serial.println("Change settings only while disarmed; init required after edits. Settings reset at boot.");
    Serial.println("motor CHANNEL SPEED MS  (channel 1..3; signed speed 1..58; time 50..3000)");
    Serial.println("original  (after arm: old LOGICAL pattern +58,-58,0; wire +58,+58,0; 3000ms ONCE)");
    Serial.println("Speed units are pulses/10ms, NOT percent. Start: motor 1 10 1000");
    Serial.println("In PWM mode motor CHANNEL DUTY MS uses percent, +/-1..50; refreshed every 20ms.");
}

void showStatus() {
    Serial.printf("configured=%d armed=%d running=%d fault=%d lastError=%u lastRegister=%u\n",
        configured, enabled, running, broken, unsigned(lastError), unsigned(lastReg));
    Serial.printf("type=%u polarity=%u speedTransport=%s clock=%uHz gap=%ums\n",
        unsigned(selectedType),unsigned(selectedPolarity),singleWrites ? "single" : "block",selectedClock,writeGapMs);
    Serial.printf("MODE=%s; output register=%u; units=%s\n",pwmMode ? "PWM" : "SPEED",
        pwmMode ? 31u : 51u,pwmMode ? "percent duty" : "pulses/10ms");
    Serial.printf("mixer=%s; orientation=%s; weights, NOT distances\n",mixerName(),orientationName());
    showOrientation();
    showHardwareSigns();
    showMotorTrims();
    Serial.printf("squareActive=%d leg=%u/4 pausing=%d sideMs=%u pauseMs=%u\n",
        squarePattern.active,squarePattern.active ? squarePattern.leg+1 : 0,
        squarePattern.pausing,squarePattern.sideMs,squarePattern.pauseMs);
    for (unsigned i=0;i<3;++i) Serial.printf("M%u tangentAngle=%.1f sign=%d (geometry only; from forward toward left; RAM)\n",
        i+1,double(wheelAngle[i]),wheelSign[i]);
}

// Validate again at this boundary: even C++ callers cannot bypass arm/fault/limits.
// Internal primitive also used by the pattern scheduler. Public startTest and
// driveRobot reject an active square, including a corner's zero-output pause.
static bool beginTimedOutput(int a, int b, int c, unsigned milliseconds) {
    if (!configured || !enabled || broken || running || milliseconds < 50
        || stopRetries || milliseconds > legacy::maxDurationMs) return false;
    const int limit = pwmMode ? legacy::maxPwmDuty : legacy::maxCommand;
    for (int value : {a,b,c}) if (value < -limit || value > limit) return false;
    if (!a && !b && !c) { stopTest("Zero pattern; stopped/disarmed."); return true; }
    began = lastReport = lastPwmRefresh = millis(); testMs = milliseconds;
    Serial.printf("REQUEST: M1=%d M2=%d M3=%d %s, %ums\n", a,b,c,
        pwmMode ? "percent PWM" : "pulses/10ms",milliseconds);
    Serial.printf("DRIVER: M1=%d M2=%d M3=%d (hardware signs applied)\n",
        driverCommand(0,a),driverCommand(1,b),driverCommand(2,c));
    if (!sendSpeeds(a,b,c)) {
        // Attempt zero even though command acknowledgment failed. The packet
        // might have partly reached the driver; a fault must not mean "keep going".
        broken = true;
        stopTest("FAULT: test command failed; disarmed."); return false;
    }
    running = true;
    activeOutputs[0] = a; activeOutputs[1] = b; activeOutputs[2] = c;
    Serial.println("STARTED: command acknowledged. Observe wheel; ! stops early.");
    return true;
}

bool startTest(int a, int b, int c, unsigned milliseconds) {
    if (squarePattern.active) return false;
    return beginTimedOutput(a,b,c,milliseconds);
}

// Public C++ entry point, sharing the serial commands' limits and state checks.
// Uses the selected orientation; lab has +X right/+Y forward. Starts ONE test.
// true means accepted, not measured physical motion. Keep loop() running; do not
// call every iteration or follow with delay(). init + arm are still required.
bool driveRobot(float x, float y, float w = 0, unsigned milliseconds = 1000, int speed = 20) {
    int outputs[3] = {};
    if (pwmMode || !computeWheelCommands(x,y,w,speed,outputs)) return false;
    return startTest(outputs[0],outputs[1],outputs[2],milliseconds);
}

// Starts ONE timed square: forward -> right -> backward -> left, W=0 each leg.
// Call once after init + arm, then keep loop() running. No blocking delay and no
// arm is needed between legs. Uses the selected mixer; no physical units assumed.
// A reported leg-3 I2C failure with 300ms pauses did not recur in the user's
// comparison with 1000ms pauses. Use that tested schedule by default, without
// inferring an electrical cause or retrying failed nonzero packets automatically.
bool driveSquare(unsigned sideMs = 1200, int speed = 20, unsigned pauseMs = 1000) {
    if (pwmMode || squarePattern.active || !configured || !enabled || broken || running || stopRetries) return false;
    int planned[4][3] = {};
    if (!planSquare(sideMs,speed,pauseMs,planned)) return false;
    if (!startTest(planned[0][0],planned[0][1],planned[0][2],sideMs)) return false;
    squarePattern = SquarePattern{};
    squarePattern.active = true;
    squarePattern.sideMs = sideMs; squarePattern.pauseMs = pauseMs;
    for (unsigned i=0;i<4;++i) for (unsigned j=0;j<3;++j) squarePattern.outputs[i][j]=planned[i][j];
    Serial.printf("SQUARE mixer=%s: leg 1/4 forward; %ums per side, %ums between sides. Timed, not measured.\n",
        mixerName(),sideMs,pauseMs);
    return true;
}

// Whole base-10 numbers only, with bounds checked before any smaller-type cast.
bool parseInt(const char* text, long& value) {
    char* end = nullptr;
    value = std::strtol(text, &end, 10);
    return end != text && *end == '\0' && value >= -1000000 && value <= 1000000;
}

bool parseFloat(const char* text, float& value) {
    char* end = nullptr; value = std::strtof(text,&end);
    return end != text && *end == '\0' && std::isfinite(value);
}

void command(char* text) {
    char* words[6] = {}; unsigned count = 0; char* saved = nullptr;
    if (text[0]) Serial.printf("Command received: %s\n", text);
    for (char* w = strtok_r(text," \t",&saved); w; w = strtok_r(nullptr," \t",&saved)) {
        if (count == 6) { Serial.println("ERR: too many arguments."); return; }
        words[count++] = w;
    }
    if (!count) return;
    const char* cmd = words[0];
    if (count == 1 && !strcmp(cmd,"stop")) { stopTest("STOPPED; disarmed."); return; }
    if (count == 1 && !strcmp(cmd,"status")) { showStatus(); return; }
    if (count == 1 && !strcmp(cmd,"help")) { showHelp(); return; }
    if (count == 1 && !strcmp(cmd,"voltage")) { showVoltage(); return; }
    if (count == 1 && !strcmp(cmd,"enc")) { showEncoders(); return; }
    if (running || squarePattern.active) { Serial.println("ERR: busy (motion or square pause); stop or wait first."); return; }
    if (!strcmp(cmd,"orientation") && count==2) {
        bool success=false;
        if (!strcmp(words[1],"lab")) success=setOrientation(DriveOrientation::lab);
        else if (!strcmp(words[1],"v9")) success=setOrientation(DriveOrientation::v9);
        if (!success) { Serial.println("ERR: orientation lab|v9; idle/disarmed, no pending stop retries required."); return; }
        showOrientation();
        Serial.println("Orientation stored in RAM; no driver write or init needed. Reset restores lab.");
        return;
    }
    if (!strcmp(cmd,"trim")) {
        long channel=0; float multiplier=0;
        const bool success = count==2 && !strcmp(words[1],"reset") ? resetMotorTrims()
            : count==3 && parseInt(words[1],channel) && channel>=1 && channel<=3
                && parseFloat(words[2],multiplier) && setMotorTrim(unsigned(channel),multiplier);
        if (!success) { Serial.println("ERR: trim CHANNEL 0.5..1.5 or trim reset; idle/disarmed, no pending stop retries required."); return; }
        showMotorTrims();
        Serial.println("Trim stored in RAM; reset loses edits. No driver write; no init needed.");
        return;
    }
    const bool previewSquare = !strcmp(cmd,"preview-square");
    if ((!strcmp(cmd,"square") || previewSquare) && count<=4) {
        long sideMs=1200,speed=20,pauseMs=1000; int planned[4][3] = {};
        if ((count>=2 && !parseInt(words[1],sideMs)) || (count>=3 && !parseInt(words[2],speed))
            || (count==4 && !parseInt(words[3],pauseMs)) || sideMs<50 || sideMs>3000
            || speed<1 || speed>30 || pauseMs<0 || pauseMs>2000
            || !planSquare(unsigned(sideMs),int(speed),unsigned(pauseMs),planned)) {
            Serial.println("ERR: square/preview-square [SIDE_MS [SPEED [PAUSE_MS]]]; side 50..3000; speed 1..30; pause 0..2000; nonzero legs required."); return;
        }
        if (previewSquare) {
            if (enabled || stopRetries) { Serial.println("ERR: preview-square requires idle/disarmed, no pending stop retries."); return; }
            showMotorTrims();
            Serial.printf("SQUARE PREVIEW mixer=%s orientation=%s side=%ldms speed=%ld pause=%ldms total=%ldms; NO driver writes.\n",
                mixerName(),orientationName(),sideMs,speed,pauseMs,4*sideMs+3*pauseMs);
            for (unsigned i=0;i<4;++i) Serial.printf("Leg %u/4 %s X=%.0f Y=%.0f W=0: M1=%d M2=%d M3=%d pulses/10ms; DRIVER M1=%d M2=%d M3=%d\n",
                i+1,squareNames[i],double(squareX[i]),double(squareY(i)),planned[i][0],planned[i][1],planned[i][2],
                driverCommand(0,planned[i][0]),driverCommand(1,planned[i][1]),driverCommand(2,planned[i][2]));
        } else if (!driveSquare(unsigned(sideMs),int(speed),unsigned(pauseMs)))
            Serial.println("ERR: square rejected; speed mode, successful init and separate arm required.");
        return;
    }
    if (!strcmp(cmd,"mixer") && count == 2) {
        if (enabled || stopRetries) { Serial.println("ERR: mixer edits require idle/disarmed, no pending stop retries."); return; }
        if (!strcmp(words[1],"legacy")) selectedMixer = Mixer::legacy;
        else if (!strcmp(words[1],"original")) selectedMixer = Mixer::original;
        else if (!strcmp(words[1],"geometry")) selectedMixer = Mixer::geometry;
        else { Serial.println("ERR: mixer legacy|original|geometry."); return; }
        Serial.printf("Mixer=%s stored in RAM; no driver write; orientation=%s.\n",mixerName(),orientationName());
        return;
    }
    if (!strcmp(cmd,"wheel") && count == 4) {
        long channel=0, sign=0; float angle=0;
        if (enabled || stopRetries || !parseInt(words[1],channel) || channel<1 || channel>3
            || !parseFloat(words[2],angle) || std::fabs(angle)>360
            || !parseInt(words[3],sign) || (sign!=1 && sign!=-1)) {
            Serial.println("ERR: wheel needs idle/disarmed, channel 1..3, angle -360..360, sign +/-1."); return;
        }
        wheelAngle[channel-1]=angle; wheelSign[channel-1]=int(sign);
        Serial.println("Wheel geometry stored in RAM; applies only to mixer geometry. No driver write; reset loses edits."); return;
    }
    if (!strcmp(cmd,"preview") && count>=3 && count<=5) {
        float x=0,y=0,w=0; long speed=20; int outputs[3] = {};
        if (enabled || stopRetries) { Serial.println("ERR: preview requires idle/disarmed, no pending stop retries."); return; }
        if (!parseFloat(words[1],x) || !parseFloat(words[2],y)
            || (count>=4 && !parseFloat(words[3],w)) || (count==5 && !parseInt(words[4],speed))
            || !computeWheelCommands(x,y,w,int(speed),outputs)) {
            Serial.println("ERR: preview X Y [W [SPEED]]; weights -1..1; speed 1..30."); return;
        }
        showMotorTrims();
        Serial.printf("PREVIEW mixer=%s orientation=%s X=%.2f Y=%.2f W=%.2f scale=%ld: M1=%d M2=%d M3=%d pulses/10ms; NO driver write.\n",
            mixerName(),orientationName(),double(x),double(y),double(w),speed,outputs[0],outputs[1],outputs[2]);
        Serial.printf("PREVIEW DRIVER: M1=%d M2=%d M3=%d; NO driver write.\n",
            driverCommand(0,outputs[0]),driverCommand(1,outputs[1]),driverCommand(2,outputs[2]));
        return;
    }
    const bool twist = !strcmp(cmd,"twist");
    if ((!strcmp(cmd,"drive") && count>=3 && count<=5) || (twist && count>=4 && count<=6)) {
        float x=0,y=0,w=0; long milliseconds=1000,speed=20;
        const unsigned timeIndex = twist ? 4 : 3;
        if (pwmMode || !parseFloat(words[1],x) || !parseFloat(words[2],y)
            || (twist && !parseFloat(words[3],w))
            || (count>timeIndex && !parseInt(words[timeIndex],milliseconds))
            || (count>timeIndex+1 && !parseInt(words[timeIndex+1],speed))
            || milliseconds<50 || milliseconds>3000 || speed<1 || speed>30
            || std::fabs(x)>1 || std::fabs(y)>1 || std::fabs(w)>1) {
            Serial.println("ERR: drive X Y [MS [SPEED]] or twist X Y W [MS [SPEED]]; weights -1..1; time 50..3000ms; speed 1..30; speed mode only."); return;
        }
        Serial.printf("Drive mixer=%s; orientation=%s.\n",mixerName(),orientationName());
        if (!driveRobot(x,y,w,unsigned(milliseconds),int(speed)))
            Serial.println("ERR: drive rejected; successful init and separate arm required.");
        return;
    }
    // Compare only documented type/polarity choices and standard bus rates.
    // These edits store preferences; init is the separate register-write step.
    const bool settings = !strcmp(cmd,"profile") || !strcmp(cmd,"transport")
        || !strcmp(cmd,"bus") || !strcmp(cmd,"gap") || !strcmp(cmd,"mode");
    if (settings) {
        if (enabled || stopRetries) { Serial.println("ERR: stop/disarm and finish stop retries before settings edits."); return; }
        long first = 0, second = 0;
        bool valid = false;
        if (!strcmp(cmd,"mode") && count == 2
            && (!strcmp(words[1],"pwm") || !strcmp(words[1],"speed"))) {
            pwmMode = !strcmp(words[1],"pwm"); valid = true;
        } else
        if (!strcmp(cmd,"profile") && count == 3 && parseInt(words[1],first) && parseInt(words[2],second)
            && (first == 1 || first == 2) && (second == 0 || second == 1)) {
            selectedType = uint8_t(first); selectedPolarity = uint8_t(second); valid = true;
        } else if (!strcmp(cmd,"transport") && count == 2
            && (!strcmp(words[1],"block") || !strcmp(words[1],"single"))) {
            singleWrites = !strcmp(words[1],"single"); valid = true;
        } else if (!strcmp(cmd,"bus") && count == 2 && parseInt(words[1],first)
            && (first == 25000 || first == 50000 || first == 100000)) {
            selectedClock = unsigned(first); valid = true;
        } else if (!strcmp(cmd,"gap") && count == 2 && parseInt(words[1],first) && first >= 0 && first <= 10) {
            writeGapMs = unsigned(first); valid = true;
        }
        if (!valid) { Serial.println("ERR: invalid settings command; type help."); return; }
        configured = false;
        Serial.println("Settings stored; no driver write yet. Enter init before arm.");
        return;
    }
    if (count == 1 && !strcmp(cmd,"init")) { initialize(); return; }
    if (count == 1 && !strcmp(cmd,"arm")) {
        if (!configured || broken || stopRetries) { Serial.println("ERR: successful init required."); return; }
        if (!sendSpeeds()) { broken = true; stopTest("FAULT: arm zero write."); return; }
        enabled = true; Serial.println("ARMED; no motion yet."); return;
    }
    if (count == 1 && !strcmp(cmd,"original")) {
        if (pwmMode) { Serial.println("ERR: original is a speed-mode test only."); return; }
        if (!startTest(58,-58,0,3000)) Serial.println("ERR: original test rejected.");
        return;
    }
    if (count == 4 && !strcmp(cmd,"motor")) {
        long channel = 0, speed = 0, milliseconds = 0;
        const int limit = pwmMode ? legacy::maxPwmDuty : legacy::maxCommand;
        if (!parseInt(words[1],channel) || !parseInt(words[2],speed)
            || !parseInt(words[3],milliseconds) || channel < 1 || channel > 3
            || speed < -limit || speed > limit || speed == 0 || milliseconds < 50 || milliseconds > 3000) {
            Serial.println("ERR: check channel, mode's magnitude limit and time 50..3000ms."); return;
        }
        int values[3] = {}; values[channel-1] = int(speed);
        if (!startTest(values[0],values[1],values[2],unsigned(milliseconds)))
            Serial.println("ERR: test rejected; check status/init/arm.");
        return;
    }
    Serial.println("ERR: unknown command. Type help.");
}

// Short input updates keep the deadline active even if serial input is flooded.
void pollInput() {
    for (unsigned budget = 0; budget < 64 && Serial.available(); ++budget) {
        const uint8_t b = uint8_t(Serial.read()); const char c = char(b);
        if (c == '!') { stopTest("STOPPED by !; disarmed."); used = 0; discard = true; }
        else if (c == '\n' || c == '\r') {
            if (!discard) { buffer[used] = 0; command(buffer); }
            used = 0; discard = false;
        } else if (!discard) {
            if (b == 8 || b == 127) { if (used) --used; }
            else if (b == 21) used = 0;
            else if ((b < 32 && c != '\t') || b > 126 || used == sizeof(buffer)-1) {
                used = 0; discard = true;
                Serial.println("ERR: unsupported/overlong input; Enter, then retype.");
            } else buffer[used++] = c;
        }
    }
}

// Speed mode holds the original one-write request until the deadline. PWM mode
// instead refreshes a full snapshot at 50Hz, with stop/fault checks on every loop.
void updateTest() {
    const uint32_t now = millis();
    if (running) {
        if (now-began >= testMs) {
            if (!squarePattern.active) { stopTest("DONE; disarmed."); return; }
            if (squarePattern.leg==3) {
                stopTest("SQUARE last leg ended; disarmed.");
                if (!broken) Serial.println("SQUARE DONE: four timed legs finished. Position not measured.");
                else Serial.println("FAULT: square ending stop failed; pattern cancelled.");
                return;
            }
            // A corner must get an acknowledged zero before the next leg. On
            // failure stopTest cancels the ENTIRE sequence, including retries.
            if (!sendSpeeds()) {
                broken = true; stopTest("FAULT: square corner stop failed; pattern cancelled."); return;
            }
            running = false;
            for (int& value : activeOutputs) value = 0;
            squarePattern.pausing = true;
            squarePattern.pauseBegan = millis(); // Pause begins after stop acknowledgment.
            Serial.printf("SQUARE leg %u/4 ended; pause %ums.\n",squarePattern.leg+1,squarePattern.pauseMs);
            return;
        }
        if (pwmMode && now-lastPwmRefresh >= legacy::pwmRefreshMs) {
            lastPwmRefresh = now;
            if (!sendSpeeds(activeOutputs[0],activeOutputs[1],activeOutputs[2],true)) {
                broken = true; stopTest("FAULT: PWM refresh; disarmed."); return;
            }
        }
        if (now-lastReport >= 1000) {
            lastReport = now;
            Serial.printf("ACTIVE: %ums remaining.\n", unsigned(testMs-(now-began)));
        }
    } else if (squarePattern.active) {
        if (!enabled || !configured || broken || pwmMode) {
            stopTest("Square cancelled: motion state changed."); return;
        }
        if (now-squarePattern.pauseBegan >= squarePattern.pauseMs) {
            ++squarePattern.leg;
            squarePattern.pausing = false;
            const int* output = squarePattern.outputs[squarePattern.leg];
            if (!beginTimedOutput(output[0],output[1],output[2],squarePattern.sideMs)) {
                if (squarePattern.active) stopTest("FAULT: square next leg rejected; pattern cancelled.");
                return;
            }
            Serial.printf("SQUARE leg %u/4 %s.\n",squarePattern.leg+1,squareNames[squarePattern.leg]);
        }
    } else if (stopRetries && now-lastRetry >= legacy::retryIntervalMs) {
        lastRetry = now; --stopRetries;
        if (sendSpeeds()) { stopRetries = 0; Serial.println("Stop retry acknowledged; remains disarmed."); }
        else if (!stopRetries) Serial.println("FAULT: stop retries exhausted; use physical power removal.");
    }
}

void setup() {
    Serial.begin(legacy::baud);
    Wire.begin(legacy::sda,legacy::scl);
    Wire.setClock(legacy::busHz);
    Wire.setTimeOut(legacy::transactionTimeoutMs);
    delay(100); // Original setup's initial I2C pause.
    showHelp(); // No driver writes or automatic movement at boot.
    Serial.println("DISARMED. Enter init to test original driver settings.");
}

void loop() { updateTest(); pollInput(); delay(1); }
