#define SWRM_ENABLE_MQTT 1
#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

unsigned networkStarts=0;
void beginRobotNetwork() {
    ++networkStarts;
    assert(!enabled && !running && !spinActive && !squarePattern.active);
}
void reportRobotCommandResult(robot::Command, robot::Result) {}
void reportRobotState(const robot::Status&) {}

void boot(const std::deque<int>& results={}, bool busReady=true) {
    Wire=HostWire{}; Serial=HostSerial{}; hostTime=0;
    Wire.endResults=results; Wire.ack=busReady;
    configured=enabled=running=broken=spinActive=false;
    squarePattern=SquarePattern{}; squareStats=SquareStats{};
    firstWriteFault=WriteFault{};
    pendingRobotCommand.store(robot::Command::none);
    pendingStatusRequest.store(false); maintenanceBusy.store(false);
    selectedType=1; selectedPolarity=0; singleWrites=pwmMode=false;
    selectedClock=legacy::busHz; writeGapMs=stopRetries=used=0; discard=false;
    selectedMixer=Mixer::legacy; selectedOrientation=DriveOrientation::lab;
    for (unsigned i=0;i<3;++i) { motorTrim[i]=1; activeOutputs[i]=0; }
    for (unsigned i=0;i<4;++i) squareTuning[i]=robot::selected.square.legs[i];
    networkStarts=0; setup(); assert(networkStarts==1);
}
void onlyZeroMotorWrites() {
    for (const auto& packet : Wire.writes) {
        if (packet[0]!=legacy::speedReg && packet[0]!=31) continue;
        for (unsigned i=1;i<packet.size();++i) assert(packet[i]==0);
    }
    for (int value : Wire.motor) assert(value==0);
    assert(!enabled && !running && !spinActive && !squarePattern.active);
}
void noAutomaticConfiguration() {
    const auto writes=Wire.writes.size();
    for (unsigned i=0;i<10;++i) { hostTime+=10000; updateRobotMovement(); }
    assert(Wire.writes.size()==writes); onlyZeroMotorWrites();
}
int main() {
    using robot::Result;
    boot(); assert(configured && !broken && !enabled);
    assert((Wire.writes==std::vector<std::vector<uint8_t>>{{51,0,0,0,0},{20,1},{21,0},{51,0,0,0,0}}));
    assert(Serial.output.find("BOOT READY")!=std::string::npos);
    onlyZeroMotorWrites(); noAutomaticConfiguration();
    // No serial init/arm occurs before this explicit command.
    assert(queueRobotCommand("square",6)==Result::queued);
    updateRobotMovement(); assert(running && enabled && squarePattern.active);
    assert(Wire.motor[0]==-18 && Wire.motor[1]==0 && Wire.motor[2]==19);
    Wire.writes.clear(); stopRobotMovement(); onlyZeroMotorWrites();

    // Preflight zero, type, polarity and final-zero failures all latch; idle/reconnect stop
    // handling cannot retry configuration, clear the fault or resume movement.
    for (const auto& fault : {std::deque<int>{4,0},std::deque<int>{0,4,0},std::deque<int>{0,0,4,0},std::deque<int>{0,0,0,4}}) {
        boot(fault); assert(!configured && broken && !enabled);
        assert(Serial.output.find("BOOT FAULT")!=std::string::npos);
        onlyZeroMotorWrites();
        while (stopRetries) { hostTime=lastRetry+legacy::retryIntervalMs; updateRobotMovement(); }
        assert(broken && !configured); noAutomaticConfiguration();
        const auto writes=Wire.writes.size();
        assert(handleRobotCommand("square")==Result::fault);
        assert(handleRobotCommand("spin")==Result::fault);
        assert(Wire.writes.size()==writes);
        onRobotConnectionLost(); updateRobotMovement();
        assert(broken && !configured); noAutomaticConfiguration();
        // Deliberate local recovery remains available, disarmed and stationary.
        assert(initializeRobotMovement()); onlyZeroMotorWrites();
        noAutomaticConfiguration();
    }
    // A completely unavailable driver remains faulted after bounded ZERO-only
    // stop retries. Successful communication later does not silently recover.
    boot({},false); assert(broken && !configured && stopRetries==4);
    while (stopRetries) { hostTime=lastRetry+legacy::retryIntervalMs; updateRobotMovement(); }
    Wire.ack=true; noAutomaticConfiguration(); assert(broken && !configured);
    std::cout << "PASS: battery boot robot " << robot::selected.identity
              << " initializes once without arming, accepts explicit starts without USB, latches every init fault, performs only bounded zero retries and never auto-recovers.\n";
}
