#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <climits>
#include <iostream>

void input(const std::string& text) {
    for (char c : text) Serial.input.push_back(c);
    while (Serial.available()) loop();
}
void cmd(const std::string& text) { input(text+"\n"); }
void fresh() {
    Wire=HostWire{}; Serial=HostSerial{}; hostTime=0;
    configured=enabled=running=broken=false;
    selectedType=1; selectedPolarity=0; singleWrites=pwmMode=false;
    selectedClock=legacy::busHz; writeGapMs=0;
    stopRetries=used=0; discard=false; lastError=lastReg=0;
    squarePattern=SquarePattern{}; selectedMixer=Mixer::original; selectedOrientation=DriveOrientation::v9;
    const float angles[3]={150,30,270};
    for (unsigned i=0;i<3;++i) { wheelAngle[i]=angles[i]; wheelSign[i]=1; activeOutputs[i]=0; motorTrim[i]=1; }
    setup(); assert(Wire.writes.empty() && !squarePattern.active);
}
// Arguments are logical requests; M2 must be negated in the actual packet.
void motors(int a,int b,int c) {
    assert(Wire.motor[0]==a && Wire.motor[1]==-b && Wire.motor[2]==c && Wire.motor[3]==0);
}
void corner() {
    hostTime=began+testMs; updateTest();
    assert(squarePattern.active && squarePattern.pausing && !running && enabled);
    motors(0,0,0);
}
void nextLeg() {
    hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; updateTest();
    assert(squarePattern.active && running && !squarePattern.pausing && enabled);
}
void stopped() {
    assert(!squarePattern.active && !squarePattern.pausing && !enabled && !running);
    motors(0,0,0);
}
int main() {
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{}; // Explicit unity diagnostic fixture; boot profile is tested separately.
    fresh(); cmd("preview-square 500 20 200");
    assert(Serial.output.find("total=2600ms")!=std::string::npos);
    assert(Serial.output.find("Leg 1/4 forward X=0 Y=-1")!=std::string::npos);
    assert(Serial.output.find("Leg 4/4 left X=-1 Y=0")!=std::string::npos);
    assert(Wire.writes.empty() && Wire.readRequests==0 && !enabled);
    cmd("square"); assert(!squarePattern.active && Wire.writes.empty());
    cmd("init"); const auto initWrites=Wire.writes.size();
    assert(!driveSquare()); assert(Wire.writes.size()==initWrites);
    cmd("arm");
    for (const char* invalid : {"square 49","square 3001","square 500 0","square 500 128",
                               "square 500 20 -1","square 500 20 2001","square nan",
                               "square 500 20 100 extra"}) {
        const auto count=Wire.writes.size(); cmd(invalid);
        assert(!running && !squarePattern.active && Wire.writes.size()==count);
    }
    assert(!driveSquare(49) && !driveSquare(3001) && !driveSquare(500,128) && !driveSquare(500,20,2001));
    cmd("preview-square"); assert(Serial.output.find("preview-square requires idle/disarmed")!=std::string::npos);
    // Original-axis sequence, correct order, actual zero packets between legs,
    // pause deadline, and no per-leg operator rearm. Tests include clock wrap.
    hostTime=UINT_MAX-250;
    cmd("square 500 20 200");
    assert(squarePattern.active && running && squarePattern.leg==0); motors(-12,12,0);
    assert(!driveSquare() && !driveRobot(1,0) && !startTest(10,0,0,500));
    hostTime=began+499; updateTest(); assert(running);
    corner(); const auto cornerWrites=Wire.writes.size();
    assert((Wire.writes.back()==std::vector<uint8_t>{51,0,0,0,0}));
    // Busy guard includes pauses: settings, arm, and other motions cannot take over.
    for (const char* busy : {"arm","init","mixer geometry","wheel 1 30 -1",
                            "mode pwm","drive 1 0","motor 1 10 500","square"}) cmd(busy);
    assert(Wire.writes.size()==cornerWrites && selectedMixer==Mixer::original && !pwmMode);
    assert(!driveRobot(1,0) && !driveSquare() && !startTest(10,0,0,500));
    hostTime=squarePattern.pauseBegan+199; updateTest(); assert(!running && squarePattern.leg==0);
    nextLeg(); assert(squarePattern.leg==1); motors(7,7,13);
    corner(); nextLeg(); assert(squarePattern.leg==2); motors(12,-12,0);
    corner(); nextLeg(); assert(squarePattern.leg==3); motors(-7,-7,-13);
    hostTime=began+500; updateTest(); stopped();
    assert(Serial.output.find("SQUARE DONE")!=std::string::npos);
    const auto completedWrites=Wire.writes.size(); hostTime+=10000; updateTest();
    assert(Wire.writes.size()==completedWrites); // Never repeats automatically.
    // Geometry mode shares direction conventions; zero pause still writes zeros
    // and advances on a later tick, not through a blocking loop.
    fresh(); cmd("mixer geometry"); cmd("init"); cmd("arm");
    assert(driveSquare(50,20,0)); motors(-17,17,0);
    corner(); nextLeg(); motors(-10,-10,20);
    corner(); nextLeg(); motors(17,-17,0);
    corner(); nextLeg(); motors(10,10,-20);
    hostTime=began+50; updateTest(); stopped();
    // Explicit stops cancel both moving and paused patterns, including later legs.
    for (bool duringPause : {false,true}) {
        fresh(); cmd("init"); cmd("arm"); cmd("square 500 20 200");
        if (duringPause) corner();
        input("!\n"); stopped();
        const auto count=Wire.writes.size(); hostTime+=10000; updateTest();
        assert(Wire.writes.size()==count);
    }
    // Packet failure on first leg, corner zero, next-leg start, and final stop
    // aborts the whole pattern. Recovering the bus may retry ZERO only.
    for (int failurePoint : {0,1,2,3}) {
        fresh(); cmd("init"); cmd("arm");
        if (failurePoint==0) Wire.ack=false;
        const bool accepted=driveSquare(50,20,100);
        if (failurePoint==0) assert(!accepted);
        else {
            assert(accepted);
            if (failurePoint==1) { Wire.ack=false; hostTime=began+50; updateTest(); }
            if (failurePoint==2) { corner(); Wire.ack=false; hostTime=squarePattern.pauseBegan+100; updateTest(); }
            if (failurePoint==3) {
                corner(); nextLeg(); corner(); nextLeg(); corner(); nextLeg();
                Wire.ack=false; hostTime=began+50; updateTest();
                assert(Serial.output.find("SQUARE DONE")==std::string::npos);
            }
        }
        assert(broken && !squarePattern.active && !running && !enabled && stopRetries);
        Wire.ack=true; const auto count=Wire.writes.size(); hostTime+=250; updateTest(); stopped();
        assert(Wire.writes.size()==count+1 && !stopRetries && broken);
        const auto stoppedWrites=Wire.writes.size(); hostTime+=10000; updateTest();
        assert(Wire.writes.size()==stoppedWrites);
    }
    // PWM is excluded, and invalid geometry cannot start a partial square.
    fresh(); cmd("mode pwm"); cmd("init"); cmd("arm");
    assert(!driveSquare() && !squarePattern.active && !running);
    cmd("stop"); cmd("mode speed"); cmd("mixer geometry");
    for (unsigned i=0;i<3;++i) wheelAngle[i]=0; // Sideways leg projects to zero.
    cmd("init"); cmd("arm"); const auto count=Wire.writes.size();
    assert(!driveSquare() && Wire.writes.size()==count && !running);
    std::cout<<"PASS: four-leg square order, original/geometry models, previews, boundaries, pauses, cancellation, faults/recovery, no repeats, and clock wrap.\n";
}
