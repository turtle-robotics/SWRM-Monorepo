// Runs the REAL firmware command/state logic against simulated serial and I2C.
// Does not validate electronics, internal driver feedback, traction, or geometry.
#include <cassert>
#include <climits>
#include <iostream>
#include "../../src/main.cpp"

void fresh() {
    Wire = HostWire{}; Serial = HostSerial{}; hostTime = 0;
    ready = armed = fault = false; motion = Motion::idle;
    lineSize = 0; discardLine = false; lastIdleStop = 0;
    setup();
    assert(ready && !armed && !fault && motion == Motion::idle);
    assert(Wire.activeReg == 31); // Stop leaves open-loop zero, even before feedback is checked.
    for (auto m : Wire.motor) assert(m == 0);
}
void input(const std::string& s) {
    for (char c : s) Serial.input.push_back(c);
    while (Serial.available()) loop();
}
void cmd(const std::string& s) { input(s+"\n"); }
void calibrated() { cmd("cal 0 6 700"); cmd("arm"); }
void assertStopped() {
    assert(!armed && motion == Motion::idle);
    for (auto m : Wire.motor) assert(m == 0);
}
int32_t wrap(int32_t count, int d) {
    uint32_t u = uint32_t(count)+uint32_t(d);
    uint8_t b[4] = {uint8_t(u), uint8_t(u>>8), uint8_t(u>>16), uint8_t(u>>24)};
    return kiwi::decode(b);
}
void simulateMove() {
    for (unsigned i = 0; i < 1000 && motion != Motion::idle; ++i) {
        for (int j = 0; j < 3; ++j)
            Wire.encoder[j] = wrap(Wire.encoder[j], int(Wire.motor[j])*2*wheels[j].encoderSign);
        hostTime += 20; loop();
    }
    assertStopped(); assert(!fault);
    for (int j = 0; j < 3; ++j) {
        float tol = fmaxf(3, wheels[j].countsPerRev*0.1f/(kiwi::pi*wheels[j].diameterCm));
        assert(std::fabs(target[j]-kiwi::delta(Wire.encoder[j], origin[j])) <= tol);
    }
    for (const auto& w : Wire.writes) if (w[0] == 51) {
        assert(w.size() == 5 && w[4] == 0);
        for (int j = 1; j < 4; ++j) assert(std::abs(int(int8_t(w[j]))) <= config::maxSpeed);
    }
}
int main() {
    // Longer direct tests must last for their requested duration, keep all
    // unselected channels zero, remain stoppable, and fault on lost contact.
    fresh(); cmd("arm"); cmd("pulse 1 25 3000");
    const uint32_t longStart = started;
    assert(motion == Motion::pulse && durationMs == 3000 && Wire.motor[0] == 25);
    assert(Wire.motor[1] == 0 && Wire.motor[2] == 0 && Wire.motor[3] == 0);
    hostTime = longStart+1000; loop();
    assert(motion == Motion::pulse && Wire.motor[0] == 25);
    assert(Serial.output.find("Wheel test active: 2000ms remaining.") != std::string::npos);
    hostTime = longStart+2999; tickMotion(); assert(motion == Motion::pulse);
    hostTime = longStart+3000; tickMotion(); assertStopped();
    cmd("arm"); cmd("jog 3 -3 5000");
    assert(motion == Motion::jog && Wire.activeReg == 51 && Wire.motor[2] == -3);
    hostTime += 2000; loop(); assert(motion == Motion::jog);
    cmd("stop"); assertStopped();
    cmd("arm"); cmd("pulse 2 -25 5000");
    hostTime += 100; loop(); assert(Wire.motor[1] == -25 && Wire.activeReg == 31);
    input("!\n"); assertStopped();
    cmd("arm");
    for (const char* bad : {"pulse 1 25 5001", "jog 1 3 5001", "jog 1 4 3000",
                           "pulse 1 26 3000", "pulse 1 20 3000.5"}) {
        cmd(bad); assert(motion == Motion::idle);
        for (auto m : Wire.motor) assert(m == 0);
    }
    cmd("jog 1 2 3000"); Wire.ack = false; hostTime += 100; loop();
    assert(fault && !armed && motion == Motion::idle);
    Wire.ack = true; hostTime += 250; loop(); assertStopped();
    fresh(); hostTime = UINT_MAX-1000; cmd("arm"); cmd("pulse 1 25 5000");
    hostTime += 5000; loop(); assertStopped();

    // Direct monitors send Backspace/DEL bytes; a visually corrected command
    // must be parsed as the edited text, never as hidden characters in a token.
    fresh(); input("arx\bm\n"); assert(armed);
    assert(Serial.output.find("Command received: arm\n") != std::string::npos);
    cmd("stop"); input("\b\x7f" "arx\x7f" "m \t\r\n"); assert(armed);
    cmd("stop"); input("wrong text\x15" "arm\n"); assert(armed);
    input("drive 1 0 5000\x7f\n"); assert(motion == Motion::timedPwm && durationMs == 500);
    cmd("stop");
    input(std::string("arm")+char(0)+"ignored\n"); assertStopped();
    input("arm\x1b[A\n"); assertStopped(); // Arrow escape invalidates that whole line.
    cmd("arm"); input("drive 1 0\x1b[A!\n"); assertStopped(); // ! works even in discarded input.
    cmd("arm"); Wire.ack = false; hostTime += 250; loop();
    assert(fault && !armed && lastI2cError != 0);
    const uint8_t savedError = lastI2cError;
    Wire.ack = true; hostTime += 250; loop(); assert(lastI2cError == savedError);
    cmd("init"); assert(!fault && lastI2cError == 0 && lastI2cErrorReg == 0);

    // First-day mode MUST work before diameter, encoder count or channel mapping
    // is discovered. No encoder read is allowed here, even if none would succeed.
    fresh(); Wire.readLength = 0;
    cmd("drive 1 0"); assertStopped(); // Disarmed input cannot start anything.
    cmd("arm"); cmd("drive 1 0");
    assert(motion == Motion::timedPwm && Wire.activeReg == 31);
    assert(wheels[0].diameterCm == 0 && wheels[0].countsPerRev == 0);
    assert(Wire.readRequests == 0 && durationMs == 500);
    assert(Wire.motor[0] == 0 && Wire.motor[1] == -17 && Wire.motor[2] == 17 && Wire.motor[3] == 0);
    const unsigned forwardStarted = started;
    cmd("motors 1 2 3"); // Busy input must not replace the running pattern.
    assert(started == forwardStarted && Wire.motor[1] == -17);
    hostTime += 100; loop(); // Refresh the PWM pattern without reading counts.
    assert(Wire.readRequests == 0 && motion == Motion::timedPwm);
    hostTime = forwardStarted+500; loop(); assertStopped(); assert(!fault);

    cmd("arm"); cmd("drive -1 0");
    assert(Wire.motor[0] == 0 && Wire.motor[1] == 17 && Wire.motor[2] == -17);
    cmd("stop"); assertStopped();
    cmd("arm"); cmd("drive 0 1 700 24");
    assert(Wire.motor[0] == 24 && Wire.motor[1] == -12 && Wire.motor[2] == -12);
    assert(durationMs == 700); input("!\n"); assertStopped();
    cmd("arm"); cmd("drive 1 1 2000 25");
    assert(motion == Motion::timedPwm);
    for (auto m : Wire.motor) assert(std::abs(int(m)) <= 25);
    cmd("stop");
    cmd("arm"); cmd("motors 8 -20 25 1234");
    assert(Wire.motor[0] == 8 && Wire.motor[1] == -20 && Wire.motor[2] == 25 && Wire.motor[3] == 0);
    assert(durationMs == 1234); cmd("stop");
    cmd("arm"); cmd("motors 0 0 0"); assertStopped();
    cmd("arm"); cmd("drive 0 0"); assertStopped();

    // Bounds, non-finite numbers and partial tokens must not reach either PWM interface.
    cmd("arm");
    for (const char* bad : {"drive nan 0", "drive 1x 0", "drive 2 0", "drive 0 -1.1",
            "drive 1 0 49", "drive 1 0 2001", "drive 1 0 1e30", "drive 1 0 200.5",
            "drive 1 0 500 26", "drive 1 0 500 0", "drive 1 0 500 1.5",
            "drive 1 0 500 inf", "drive 1", "drive 1 0 500 20 extra",
            "motors 0 0 26", "motors -26 0 0", "motors 1.1 0 0", "motors nan 0 0",
            "motors 1 2", "motors 1 2 3 1e30", "motors 1 2 3 50 extra"}) {
        const auto writesBefore = Wire.writes.size();
        cmd(bad); assert(motion == Motion::idle);
        // Periodic zero writes are okay, but invalid commands cannot energize a motor.
        for (size_t i = writesBefore; i < Wire.writes.size(); ++i)
            for (size_t j = 1; j < Wire.writes[i].size(); ++j) assert(Wire.writes[i][j] == 0);
        for (auto m : Wire.motor) assert(m == 0);
    }
    assert(Wire.readRequests == 0);
    cmd("stop"); cmd("wheel 2 210 -1 1"); cmd("arm"); cmd("drive 1 0");
    assert(Wire.motor[1] == 17); cmd("stop"); // A later mapping edit actually changes the pattern.

    fresh(); cmd("arm"); cmd("drive 1 0");
    Wire.ack = false; hostTime += 100; loop();
    assert(fault && !armed && motion == Motion::idle); // Mid-pattern refresh detects bus loss.
    Wire.ack = true; hostTime += 250; loop(); assertStopped();
    fresh(); hostTime = UINT_MAX-200; cmd("arm"); cmd("drive 0 1");
    hostTime += 501; loop(); assertStopped(); // Timed PWM deadline across millis wrap.
    fresh(); cmd("arm"); cmd("motors 20 0 -20");
    input(std::string(200,'x')+"\n"); hostTime += 501; loop(); assertStopped();

    // The original calibrated path must continue to behave as before.
    fresh(); cmd("move 10 0"); assertStopped(); // Disarmed.
    cmd("arm"); cmd("move 10 0"); assert(motion == Motion::idle); // Uncalibrated.
    cmd("stop"); calibrated();
    for (const char* bad : {"move nan 0", "move inf 0", "move 1x 0", "move 51 0",
                           "move 40 40", "move 1 0 extra", "jog 1 99 300", "jog 4 1 300",
                           "jog 1.5 1 300", "pulse 1 20 5001", "jog 1 1 1e30"}) {
        cmd(bad); assert(motion == Motion::idle);
        for (auto m : Wire.motor) assert(m == 0);
    }
    cmd("move 10 0"); assert(motion == Motion::distance);
    assert(std::fabs(target[0]) < 0.001 && target[1] < 0 && target[2] > 0);
    cmd("move 0 10"); // Busy request cannot replace the existing move.
    assert(target[1] < 0 && target[2] > 0);
    simulateMove();
    cmd("arm"); cmd("move 0 10");
    assert(target[0] > 0 && target[1] < 0 && target[2] < 0); simulateMove();
    cmd("arm"); cmd("move -10 -10"); simulateMove();
    cmd("arm"); cmd("move 0 0"); hostTime += 20; loop(); assertStopped();

    fresh(); calibrated(); cmd("jog 2 2 300"); assert(Wire.motor[1] == 2);
    hostTime += 300; loop(); assertStopped();
    cmd("arm"); cmd("pulse 1 -20 200"); assert(Wire.activeReg == 31 && Wire.motor[0] == -20);
    input("!"); assertStopped(); input("\n");
    cmd("arm"); cmd("jog 1 1 100"); input(std::string(200,'x')+"\n");
    hostTime += 100; loop(); assertStopped();
    cmd("arm"); cmd(std::string(100,'x')+"move 10 0"); assert(motion == Motion::idle);

    fresh(); calibrated(); cmd("move 10 0");
    Wire.readLength = 8; hostTime += 20; loop(); assertStopped(); assert(fault);
    Wire.readLength = 12; cmd("arm"); assert(!armed); cmd("init"); assert(!fault);
    cmd("arm"); cmd("move 10 0"); hostTime += 1300; loop(); assertStopped(); assert(fault);

    fresh(); calibrated(); cmd("move 10 0"); hostTime += 20; loop();
    Wire.encoder[1] += 10; hostTime += 20; loop(); assertStopped(); assert(fault);
    fresh(); calibrated(); cmd("move 10 0"); Wire.encoder[1] = 10000;
    hostTime += 20; loop(); assertStopped(); assert(fault);
    fresh(); calibrated(); cmd("move 10 0"); hostTime += 30000; loop(); assertStopped(); assert(fault);

    fresh(); calibrated(); cmd("move 10 0"); Wire.ack = false;
    hostTime += 20; loop(); assert(!armed && fault && motion == Motion::idle);
    Wire.ack = true; hostTime += 250; loop(); assertStopped(); // Stop retried after bus recovers.

    fresh(); calibrated(); Wire.encoder[0] = INT_MAX-10; Wire.encoder[1] = INT_MIN+10;
    Wire.encoder[2] = INT_MAX-10; cmd("move 10 10"); simulateMove();
    fresh(); cmd("wheel 2 210 -1 -1"); calibrated(); cmd("move 10 0"); simulateMove();
    assert(kiwi::delta(INT_MIN, INT_MAX) == 1);
    const uint8_t neg[] = {0xff,0xff,0xff,0xff}; assert(kiwi::decode(neg) == -1);

    fresh(); hostTime = UINT_MAX-200; cmd("arm"); cmd("jog 1 1 300");
    hostTime += 301; loop(); assertStopped(); // millis wrap.
    std::cout << "PASS: uncalibrated timed PWM, raw motor patterns, parsing, movement simulation, limits, deadlines, encoder faults, stops, wraparound.\n";
}
