#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <climits>
#include <iostream>

void freshLegacy() {
    Wire = HostWire{}; Serial = HostSerial{}; hostTime = 0;
    configured = enabled = running = broken = false;
    squarePattern = SquarePattern{}; selectedOrientation=DriveOrientation::v9;
    selectedType = 1; selectedPolarity = 1; singleWrites = true;
    selectedClock = legacy::busHz; writeGapMs = 0;
    pwmMode = false; for (int& value : activeOutputs) value = 0;
    selectedMixer = Mixer::original;
    const float angles[3] = {150,30,270};
    for (unsigned i=0;i<3;++i) { wheelAngle[i]=angles[i]; wheelSign[i]=1; motorTrim[i]=1; }
    stopRetries = used = 0; discard = false; lastError = lastReg = 0;
    setup();
    assert(!configured && !enabled && !running && !broken);
    assert(Wire.writes.empty()); // Boot only prints instructions, no register writes.
}
void inputLegacy(const std::string& text) {
    for (char c : text) Serial.input.push_back(c);
    while (Serial.available()) loop();
}
void cmdLegacy(const std::string& text) { inputLegacy(text+"\n"); }
void stoppedLegacy() {
    assert(!enabled && !running);
    for (auto m : Wire.motor) assert(m == 0);
}
// Compare existing logical-command regressions after decoding the hardware adapter.
int logicalMotor(unsigned i) { return Wire.motor[i]*(i==1 ? -1 : 1); }

int main() {
    assert(selectedType==1 && selectedPolarity==0 && !singleWrites && !pwmMode);
    assert(selectedMixer==Mixer::legacy);
    // Preview/mixer edits work before init and cannot touch the bus. Outputs
    // use +X right, +Y backward, +W clockwise in BOTH selectable models.
    freshLegacy(); cmdLegacy("preview 0 -1 0 20");
    assert(Serial.output.find("M1=-12 M2=12 M3=0")!=std::string::npos);
    cmdLegacy("preview 1 0");
    assert(Serial.output.find("M1=7 M2=7 M3=13")!=std::string::npos);
    cmdLegacy("mixer geometry"); cmdLegacy("preview 0 -1");
    assert(Serial.output.find("M1=-17 M2=17 M3=0")!=std::string::npos);
    cmdLegacy("preview 1 0");
    assert(Serial.output.find("M1=-10 M2=-10 M3=20")!=std::string::npos);
    cmdLegacy("preview 0 0 1");
    assert(Serial.output.find("M1=-20 M2=-20 M3=-20")!=std::string::npos);
    assert(Wire.writes.empty() && Wire.readRequests==0 && !enabled && !running);
    int candidate[3] = {};
    assert(computeWheelCommands(0,1,0,20,candidate));
    assert(candidate[0]==17 && candidate[1]==-17 && candidate[2]==0);
    assert(computeWheelCommands(-1,0,0,20,candidate));
    assert(candidate[0]==10 && candidate[1]==10 && candidate[2]==-20);
    cmdLegacy("mixer original");
    assert(computeWheelCommands(1,1,-1,30,candidate));
    assert(candidate[0]==30 && candidate[1]==2 && candidate[2]==8); // Shared limiting.
    assert(!driveRobot(0,-1)); // Pure computation never bypasses init/arm.
    // A channel permutation/sign edit only changes geometry mode, not the
    // preserved original coefficients or motor-driver feedback configuration.
    cmdLegacy("wheel 1 30 -1");
    assert(computeWheelCommands(0,-1,0,20,candidate) && candidate[0]==-12);
    cmdLegacy("mixer geometry");
    assert(computeWheelCommands(0,-1,0,20,candidate) && candidate[0]==-17);
    assert(selectedPolarity==1 && Wire.writes.empty());
    cmdLegacy("mixer invalid"); assert(selectedMixer==Mixer::geometry);
    for (const char* bad : {"preview nan 0", "preview 0 inf", "preview 0 0 2",
                            "preview 0 0 0 0", "preview 0 0 0 128"}) {
        Serial.output.clear(); cmdLegacy(bad);
        assert(Serial.output.find("PREVIEW mixer=")==std::string::npos && Wire.writes.empty());
    }
    // Use the bench-confirmed driver profile and test real serial drive parsing,
    // multiwheel packet construction, reversals, clipping and timed zero stops.
    freshLegacy(); cmdLegacy("profile 1 0"); cmdLegacy("transport block"); cmdLegacy("init");
    cmdLegacy("drive 1 0"); assert(!running && !enabled);
    cmdLegacy("arm"); cmdLegacy("drive 0 -1 1000 20");
    assert(running && logicalMotor(0)==-12 && logicalMotor(1)==12 && logicalMotor(2)==0);
    assert((Wire.writes.back()==std::vector<uint8_t>{51,244,244,0,0}));
    cmdLegacy("mixer geometry"); assert(selectedMixer==Mixer::original); // Busy edits rejected.
    hostTime=began+1000; updateTest(); stoppedLegacy();
    cmdLegacy("arm"); cmdLegacy("drive 0 1");
    assert(running && logicalMotor(0)==12 && logicalMotor(1)==-12); cmdLegacy("stop");
    cmdLegacy("mixer geometry"); assert(configured && selectedMixer==Mixer::geometry);
    cmdLegacy("arm"); cmdLegacy("drive 1 0 1500 20");
    assert(running && logicalMotor(0)==-10 && logicalMotor(1)==-10 && logicalMotor(2)==20);
    cmdLegacy("wheel 1 150 -1"); assert(wheelSign[0]==1); cmdLegacy("stop");
    cmdLegacy("wheel 1 150 -1"); assert(wheelSign[0]==-1);
    cmdLegacy("arm"); cmdLegacy("drive 1 0"); assert(logicalMotor(0)==10); cmdLegacy("stop");
    cmdLegacy("wheel 1 150 1");
    cmdLegacy("arm"); auto noWrites=Wire.writes.size();
    cmdLegacy("mixer original"); cmdLegacy("preview 0 0");
    assert(selectedMixer==Mixer::geometry && Wire.writes.size()==noWrites);
    cmdLegacy("twist 0 0 1 500 10");
    assert(running && logicalMotor(0)==-10 && logicalMotor(1)==-10 && logicalMotor(2)==-10);
    hostTime=began+500; updateTest(); stoppedLegacy();
    cmdLegacy("arm"); assert(driveRobot(0,-1,0,500,20));
    assert(logicalMotor(0)==-17 && logicalMotor(1)==17 && logicalMotor(2)==0);
    hostTime=began+500; updateTest(); stoppedLegacy();
    cmdLegacy("arm"); cmdLegacy("drive 1 1 1000 30");
    for (int value : Wire.motor) assert(value>=-30 && value<=30);
    inputLegacy("!\n"); stoppedLegacy(); cmdLegacy("arm");
    for (const char* bad : {"drive nan 0","drive 2 0","drive 1 0 3001","drive 1 0 1000 128","drive 1 0 1000 0",
                            "twist 0 0 nan","twist 0 0 2","twist 0 0 1 49","twist 0 0 1 500 128",
                            "twist 0 0 1 500 20 extra"}) {
        const auto n=Wire.writes.size(); cmdLegacy(bad); assert(!running && Wire.writes.size()==n);
    }
    const auto beforeInvalid=Wire.writes.size();
    assert(!driveRobot(NAN,0) && !driveRobot(0,0,INFINITY) && !driveRobot(0,0,2));
    assert(!driveRobot(0,-1,0,49) && !driveRobot(0,-1,0,3001) && !driveRobot(0,-1,0,500,128));
    assert(Wire.writes.size()==beforeInvalid);
    cmdLegacy("stop"); cmdLegacy("mode pwm"); cmdLegacy("init"); cmdLegacy("arm");
    cmdLegacy("drive 1 0"); cmdLegacy("twist 0 0 1"); assert(!running);
    assert(!driveRobot(0,-1)); cmdLegacy("stop");
    // Diagnostics never configure/arm/write a motor payload, even on failure.
    freshLegacy(); cmdLegacy("voltage");
    assert(Serial.output.find("reported=5000 mV (5.000 V)") != std::string::npos);
    Wire.readLength = 16;
    Wire.encoder[0] = INT32_MIN; Wire.encoder[1] = INT32_MAX;
    Wire.encoder[2] = -17; Wire.encoder[3] = 42;
    cmdLegacy("enc");
    assert(Serial.output.find("M1=-2147483648 M2=2147483647 M3=-17 M4=42") != std::string::npos);
    assert(Wire.writes.empty() && !configured && !enabled);
    Serial.output.clear(); Wire.voltageReadLength = 1; cmdLegacy("voltage");
    assert(Serial.output.find("short response") != std::string::npos);
    assert(Serial.output.find("SUPPLY reported") == std::string::npos);
    Serial.output.clear(); Wire.readLength = 12; cmdLegacy("enc");
    assert(Serial.output.find("ENC cumulative") == std::string::npos);
    Wire.ack = false; Serial.output.clear(); cmdLegacy("voltage");
    assert(Serial.output.find("select error=4") != std::string::npos);
    Wire.ack = true; cmdLegacy("init"); cmdLegacy("arm");
    auto reads = Wire.readRequests; cmdLegacy("enc"); cmdLegacy("voltage");
    assert(Wire.readRequests == reads && enabled);
    cmdLegacy("motor 1 10 1000"); cmdLegacy("voltage");
    assert(Wire.readRequests == reads && running);
    freshLegacy(); cmdLegacy("motor 1 10 1000"); assert(Wire.writes.empty());
    cmdLegacy("init"); assert(configured && !enabled && !running && !broken);
    assert(Wire.writes.size() == 6);
    assert((Wire.writes[0] == std::vector<uint8_t>{20,1}));
    assert((Wire.writes[1] == std::vector<uint8_t>{21,1}));
    for (unsigned i = 0; i < 4; ++i)
        assert((Wire.writes[2+i] == std::vector<uint8_t>{uint8_t(51+i),0}));
    auto writes = Wire.writes.size(); hostTime += 10000; loop();
    assert(Wire.writes.size() == writes); // No alternating or repeated idle-mode writes.
    cmdLegacy("arm"); cmdLegacy("motor 1 10 1000");
    assert(running && logicalMotor(0) == 10 && logicalMotor(1) == 0 && logicalMotor(2) == 0 && logicalMotor(3) == 0);
    const auto start = began; writes = Wire.writes.size();
    hostTime = start+999; updateTest(); assert(running && Wire.writes.size() == writes);
    hostTime = start+1000; updateTest(); stoppedLegacy(); assert(!broken);
    cmdLegacy("arm"); cmdLegacy("original"); assert(running);
    assert(logicalMotor(0) == 58 && logicalMotor(1) == -58 && logicalMotor(2) == 0 && logicalMotor(3) == 0);
    const auto originalStart = began; writes = Wire.writes.size();
    hostTime = originalStart+2000; updateTest(); assert(running && Wire.writes.size() == writes);
    cmdLegacy("motor 3 5 1000"); assert(began == originalStart && logicalMotor(0) == 58);
    hostTime = originalStart+3000; updateTest(); stoppedLegacy();
    writes = Wire.writes.size(); hostTime += 10000; updateTest(); assert(Wire.writes.size() == writes);
    for (const auto& w : Wire.writes) {
        assert(w[0] == 20 || w[0] == 21 || (w[0] >= 51 && w[0] <= 54));
        assert(w.size() == 2); // Every speed payload is now individual, never four bytes.
    }
    // Regression for the physical report: M2 must be addressed at register 52
    // after zeroing M1/M3/M4; then M3 at 53 after zeroing M1/M2/M4.
    cmdLegacy("arm"); writes = Wire.writes.size(); cmdLegacy("motor 2 10 1000");
    assert((Wire.writes[writes] == std::vector<uint8_t>{51,0}));
    assert((Wire.writes[writes+1] == std::vector<uint8_t>{53,0}));
    assert((Wire.writes[writes+2] == std::vector<uint8_t>{54,0}));
    assert((Wire.writes[writes+3] == std::vector<uint8_t>{52,246}));
    assert(logicalMotor(0) == 0 && logicalMotor(1) == 10 && logicalMotor(2) == 0);
    cmdLegacy("stop"); cmdLegacy("arm"); cmdLegacy("motor 3 10 1000");
    assert(logicalMotor(0) == 0 && logicalMotor(1) == 0 && logicalMotor(2) == 10);
    cmdLegacy("stop");
    assert(Wire.readRequests == 0); // No hidden encoder resets/reads or other registers.

    cmdLegacy("arm");
    for (const char* invalid : {"motor 1 128 1000", "motor 1 -128 1000", "motor 1 10 3001",
                               "motor 0 10 1000", "motor 1 10.0 1000", "motor 1 nan 1000",
                               "motor 1 10 1e30", "motor 1 0 1000", "motor 1 1 49"}) {
        writes = Wire.writes.size(); cmdLegacy(invalid);
        assert(!running && Wire.writes.size() == writes);
    }
    cmdLegacy("motor 3 -58 3000"); assert(running && logicalMotor(2) == -58);
    inputLegacy("!\n"); stoppedLegacy();
    inputLegacy("arx\bm\n"); assert(enabled); cmdLegacy("motor 2 5 1000");
    cmdLegacy("stop"); stoppedLegacy();
    cmdLegacy("arm"); Wire.ack = false; cmdLegacy("motor 1 10 1000");
    assert(!running && !enabled && broken && stopRetries);
    Wire.ack = true; hostTime += 250; updateTest(); stoppedLegacy(); assert(broken && !stopRetries);
    cmdLegacy("arm"); assert(!enabled); cmdLegacy("init"); assert(!broken && configured);

    cmdLegacy("arm"); cmdLegacy("motor 1 10 1000"); Wire.ack = false;
    hostTime += 1000; updateTest(); assert(broken && stopRetries == legacy::maxStopRetries);
    for (unsigned i = 0; i < legacy::maxStopRetries; ++i) { hostTime += 250; updateTest(); }
    assert(stopRetries == 0 && broken && !enabled && !running);
    Wire.ack = true; // Remaining output can only be stopped after communication recovers.
    cmdLegacy("stop"); stoppedLegacy();
    cmdLegacy("init"); cmdLegacy("arm");
    hostTime = UINT_MAX-100; cmdLegacy("motor 1 10 1000");
    hostTime += 1000; updateTest(); stoppedLegacy();
    // Runtime comparison settings must never send a packet/motion by themselves.
    freshLegacy(); cmdLegacy("profile 2 1"); cmdLegacy("transport block");
    cmdLegacy("bus 25000"); cmdLegacy("gap 5"); assert(Wire.writes.empty());
    assert(!configured && !enabled && selectedType == 2 && selectedPolarity == 1 && !singleWrites);
    cmdLegacy("arm"); assert(!enabled);
    cmdLegacy("init"); assert(configured);
    assert((Wire.writes[0] == std::vector<uint8_t>{20,2}));
    assert((Wire.writes[1] == std::vector<uint8_t>{21,1}));
    assert((Wire.writes[2] == std::vector<uint8_t>{51,0,0,0,0}));
    cmdLegacy("arm"); cmdLegacy("profile 1 0"); assert(selectedType == 2 && selectedPolarity == 1);
    cmdLegacy("motor 2 10 1000"); assert(running && logicalMotor(0) == 0 && logicalMotor(1) == 10);
    assert((Wire.writes.back() == std::vector<uint8_t>{51,0,246,0,0}));
    cmdLegacy("transport single"); assert(!singleWrites); // Busy edits rejected.
    cmdLegacy("stop"); stoppedLegacy(); cmdLegacy("profile 2 0");
    assert(!configured && !enabled && selectedPolarity == 0);
    cmdLegacy("init"); assert(configured);
    for (const char* bad : {"profile 0 0","profile 2 2","transport arbitrary","bus 0","gap 11"}) {
        writes = Wire.writes.size(); cmdLegacy(bad); assert(Wire.writes.size() == writes);
        assert(selectedType == 2 && selectedPolarity == 0 && selectedClock == 25000 && writeGapMs == 5);
    }
    // Model the reported order M2 -> M1 -> M2 in PWM mode. Check every refresh
    // has a complete fresh snapshot, and that no speed packet re-enables feedback.
    freshLegacy(); cmdLegacy("mode pwm"); cmdLegacy("profile 2 1");
    assert(Wire.writes.empty()); cmdLegacy("init");
    assert(pwmMode && configured && Wire.activeReg == 31);
    for (const auto& w : Wire.writes) assert(w[0] == 20 || w[0] == 21 || w[0] == 31);
    for (int channel : {2,1,2}) {
        cmdLegacy("arm");
        cmdLegacy(std::string("motor ")+std::to_string(channel)+" 40 1000");
        assert(running && Wire.activeReg == 31);
        const auto pwmStart = began;
        hostTime += 20; updateTest();
        for (int i = 0; i < 4; ++i) assert(logicalMotor(i) == (i == channel-1 ? 40 : 0));
        hostTime = pwmStart+1000; updateTest(); stoppedLegacy();
        for (int v : activeOutputs) assert(v == 0);
        assert(Wire.activeReg == 31);
    }
    cmdLegacy("arm"); cmdLegacy("motor 1 51 1000"); assert(!running);
    cmdLegacy("motor 1 -40 1000"); assert(running); inputLegacy("!\n"); stoppedLegacy();
    cmdLegacy("arm"); cmdLegacy("mode speed"); assert(pwmMode); // Armed edit rejected.
    cmdLegacy("motor 1 40 1000"); Wire.ack = false; hostTime += 20; updateTest();
    assert(broken && !enabled && !running && stopRetries);
    Wire.ack = true; hostTime += 250; updateTest(); stoppedLegacy(); assert(Wire.activeReg == 31);
    std::cout << "PASS: verified boot defaults, directional speed packets/reversals/limits, geometry edits, read diagnostics, runtime profiles, PWM history/refresh, stops, faults, bounded retries and clock wrap.\n";
}
