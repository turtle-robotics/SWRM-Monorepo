#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void serial(const char* text) {
    for (char c : std::string(text)+"\n") Serial.input.push_back(c);
    while (Serial.available()) loop();
}
void fresh(bool init=true) {
    Wire=HostWire{}; Serial=HostSerial{}; hostTime=0;
    configured=enabled=running=broken=spinActive=false;
    pendingRobotCommand.store(robot::Command::none);
    squarePattern=SquarePattern{}; squareStats=SquareStats{}; firstWriteFault=WriteFault{};
    selectedType=1; selectedPolarity=0; singleWrites=pwmMode=false;
    selectedMixer=Mixer::legacy; selectedOrientation=DriveOrientation::lab;
    selectedClock=legacy::busHz; writeGapMs=stopRetries=used=0; discard=false;
    for (unsigned i=0;i<3;++i) { motorTrim[i]=1; activeOutputs[i]=0; }
    for (unsigned i=0;i<4;++i) squareTuning[i]=robot::selected.square.legs[i];
    setup();
    assert(Wire.writes.empty() && !configured && !enabled && !running);
    if (init) {
        assert(initializeRobotMovement());
        assert(configured && !enabled && !running && !squarePattern.active);
        for (int value : Wire.motor) assert(value==0);
    }
}
void packet(int a,int b,int c) {
    assert(Wire.motor[0]==a && Wire.motor[1]==b && Wire.motor[2]==c && Wire.motor[3]==0);
}
void endSide() { hostTime=began+testMs; updateRobotMovement(); }
void nextSide() { hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; updateRobotMovement(); }
void stopped() {
    assert(!running && !enabled && !squarePattern.active && !spinActive);
    assert(pendingRobotCommand.load()==robot::Command::none);
    packet(0,0,0);
}
void noResume() {
    const auto writes=Wire.writes.size();
    for (unsigned i=0;i<10;++i) { hostTime+=10000; updateRobotMovement(); }
    assert(Wire.writes.size()==writes); stopped();
}
int main() {
    using robot::Result;
    // Test real static boot defaults before fixtures can replace them.
    assert(squareTuning[0].trim[0]==1.04f && squareTuning[0].trim[2]==1.10f);
    assert(squareTuning[1].trim[0]==0.90f && squareTuning[1].trim[1]==0.95f && squareTuning[1].trim[2]==0.90f);
    assert(squareTuning[2].trim[0]==1 && squareTuning[2].trim[1]==1 && squareTuning[2].trim[2]==1);
    assert(squareTuning[3].trim[0]==0.90f && squareTuning[3].trim[1]==0.95f && squareTuning[3].trim[2]==1);
    for (const auto& tuning : squareTuning) assert(tuning.timeScale==1);
    assert(robot::selected.square.sideMs==1500 && robot::selected.square.speed==20 && robot::selected.square.pauseMs==1000);
    assert(std::strcmp(robot::robotA.mqttClientId,robot::robotB.mqttClientId));
    assert(std::strcmp(robot::robotA.commandTopic,robot::robotB.commandTopic));
#if SWRM_ROBOT_ID == 2
    assert(!std::strcmp(robot::selected.identity,"B") && robot::selected.inheritsUnvalidated);
#else
    assert(!std::strcmp(robot::selected.identity,"A") && !robot::selected.inheritsUnvalidated);
#endif
    fresh(false);
    assert(handleRobotCommand("square")==Result::notReady);
    assert(handleRobotCommand("spin")==Result::notReady);
    assert(Wire.writes.empty());
    assert(initializeRobotMovement()); stopped();

    // Exact lengths; no read beyond a nonterminated payload, no prefix commands.
    const char squareBytes[]={'s','q','u','a','r','e'};
    const char stopBytes[]={'s','t','o','p'};
    const char binary[]={'s','t','o','p','\0','s','p','i','n'};
    auto writes=Wire.writes.size();
    for (const char* bad : {"", "Square", " square", "square\n", "spin ", "stop spin", "on", "off", "square 1500"}) {
        assert(handleRobotCommand(bad)==Result::unknown);
        assert(queueRobotCommand(bad,std::strlen(bad))==Result::unknown);
    }
    assert(handleRobotCommand(nullptr,6)==Result::unknown);
    assert(handleRobotCommand(binary,sizeof(binary))==Result::unknown);
    assert(handleRobotCommand(squareBytes,5)==Result::unknown);
    assert(Wire.writes.size()==writes && !enabled);
    assert(handleRobotCommand(squareBytes,sizeof(squareBytes))==Result::started);
    assert(enabled && running && squarePattern.cycleLimit==0 && testMs==1500);
    packet(-18,0,19);
    // Verify all calibrated DRIVER packets, repeated cycle, and stop while running.
    endSide(); nextSide(); packet(9,19,9);
    endSide(); nextSide(); packet(17,0,-17);
    endSide(); nextSide(); packet(-9,-19,-10);
    endSide(); assert(squareStats.completed==1 && squarePattern.active);
    nextSide(); packet(-18,0,19);
    writes=Wire.writes.size();
    assert(handleRobotCommand("spin")==Result::busy);
    assert(handleRobotCommand("square")==Result::busy);
    assert(handleRobotCommand("garbage")==Result::unknown);
    assert(Wire.writes.size()==writes); packet(-18,0,19);
    assert(handleRobotCommand(stopBytes,sizeof(stopBytes))==Result::stopped); noResume();

    // Every side/pause including between completed cycles: no scheduled replay.
    for (unsigned leg=0;leg<4;++leg) for (bool pause : {false,true}) {
        fresh(); serial("square");
        for (unsigned i=0;i<leg;++i) { endSide(); nextSide(); }
        if (pause) endSide();
        assert(squarePattern.leg==leg && squarePattern.active);
        assert(queueRobotCommand(stopBytes,sizeof(stopBytes))==Result::queued);
        assert(queueRobotCommand(squareBytes,sizeof(squareBytes))==Result::busy);
        hostTime=pause ? squarePattern.pauseBegan+squarePattern.pauseMs : began+testMs;
        writes=Wire.writes.size(); updateRobotMovement();
        assert(Wire.writes.size()==writes+1); stopped(); noResume();
    }
    // Callbacks never do I2C. Stop supersedes a start before it can arm.
    fresh(); writes=Wire.writes.size();
    assert(queueRobotCommand(squareBytes,sizeof(squareBytes))==Result::queued);
    assert(queueRobotCommand("spin",4)==Result::busy);
    assert(queueRobotCommand(stopBytes,sizeof(stopBytes))==Result::queued);
    assert(Wire.writes.size()==writes && !running);
    updateRobotMovement(); assert(Wire.writes.size()==writes+1); noResume();
    assert(queueRobotCommand(squareBytes,sizeof(squareBytes))==Result::queued);
    onRobotConnectionLost(); updateRobotMovement(); noResume();
    // No reconnect action starts anything; connection loss cancels active squares/spin.
    serial("square"); endSide(); onRobotConnectionLost(); updateRobotMovement(); noResume();
    serial("spin"); assert(spinActive && running && enabled); packet(-10,10,-10);
    const uint32_t beganSpin=began;
    for (uint32_t t : {3001u,100000u,UINT32_MAX}) { hostTime=beganSpin+t; updateRobotMovement(); assert(spinActive); packet(-10,10,-10); }
    onRobotConnectionLost(); updateRobotMovement(); noResume();
    serial("spin"); serial("stop"); noResume();

    // Acknowledgement failure in arm, start, or stop latches a fault; a new
    // request and zero retries cannot clear it. Only explicit init can recover.
    for (unsigned point=0;point<3;++point) {
        fresh();
        if (point==0) Wire.endResults={4,0};
        if (point==1) Wire.endResults={0,4,0};
        if (point<2) assert(handleRobotCommand("square")==Result::fault);
        else {
            assert(handleRobotCommand("spin")==Result::started);
            Wire.endResults={4,0}; assert(handleRobotCommand("stop")==Result::fault);
            hostTime=lastRetry+legacy::retryIntervalMs; updateRobotMovement();
        }
        assert(broken); stopped(); writes=Wire.writes.size();
        assert(handleRobotCommand("square")==Result::fault);
        assert(handleRobotCommand("spin")==Result::fault);
        assert(Wire.writes.size()==writes); noResume();
        assert(initializeRobotMovement()); stopped(); noResume();
        assert(handleRobotCommand("spin")==Result::started); stopRobotMovement();
    }
    // Invalid settings never silently change diagnostics or energize outputs.
    fresh(); selectedOrientation=DriveOrientation::v9; writes=Wire.writes.size();
    assert(handleRobotCommand("square")==Result::invalidSettings);
    assert(!enabled && Wire.writes.size()==writes);
    selectedOrientation=DriveOrientation::lab; squareTuning[0].timeScale=99;
    assert(handleRobotCommand("square")==Result::invalidSettings);
    assert(Wire.writes.size()==writes && !enabled);
    squareTuning[0]=robot::selected.square.legs[0]; motorTrim[0]=1.1f;
    assert(handleRobotCommand("spin")==Result::invalidSettings);
    assert(Wire.writes.size()==writes && !enabled);
    std::cout << "PASS: robot " << robot::selected.identity << " boot profile, exact bounded payloads, guarded auto-arm, calibrated repeats, every stop phase, callback inbox, connection-loss hook, continuous spin, retained faults and explicit recovery.\n";
}
