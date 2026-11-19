#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void cmd(const char* text) {
    for (char c : std::string(text)+"\n") Serial.input.push_back(c);
    while (Serial.available()) loop();
}
void fresh() {
    Wire=HostWire{}; Serial=HostSerial{}; hostTime=0;
    configured=enabled=running=broken=false;
    squarePattern=SquarePattern{}; squareStats=SquareStats{}; firstWriteFault=WriteFault{};
    selectedType=1; selectedPolarity=0; singleWrites=pwmMode=false;
    selectedMixer=Mixer::legacy; selectedOrientation=DriveOrientation::lab;
    selectedClock=legacy::busHz; writeGapMs=stopRetries=used=0;
    discard=false; lastError=lastReg=0;
    for (unsigned i=0;i<3;++i) { motorTrim[i]=1; activeOutputs[i]=0; }
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{};
    setup(); cmd("init");
}
void endSide() { hostTime=began+testMs; updateTest(); }
void nextSide() { hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; updateTest(); }
void cycle() {
    for (unsigned i=0;i<4;++i) { endSide(); if (i<3) nextSide(); }
}
void stopped() {
    assert(!squarePattern.active && !running && !enabled);
    for (int value : Wire.motor) assert(value==0);
}
void noResume() {
    const auto writes=Wire.writes.size(); hostTime+=10000; updateTest();
    assert(!squarePattern.active && !running && !enabled && Wire.writes.size()==writes);
}
int main() {
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{}; // Explicit unity diagnostic fixture; boot profile is tested separately.
    fresh(); const auto initialWrites=Wire.writes.size();
    cmd("square-endless 500 20 100"); assert(!squarePattern.active && Wire.writes.size()==initialWrites);
    for (const char* invalid : {"square-repeat", "square-repeat 0", "square-repeat -1", "square-repeat 1000001",
         "square-repeat 2junk", "square-endless 49", "square-endless 500 128", "square-endless 500 20 2001",
         "square-repeat 3 500 20 100 extra", "square-leg north 1 1 1 1", "square-leg forward nan 1 1 1",
         "square-leg forward 1 inf 1 1", "square-leg forward 1 0.49 1 1", "square-leg forward 1.51 1 1 1",
         "square-leg forward 1 1 1", "square-test north 500 20", "square-test forward -1 20"}) {
        cmd(invalid); assert(!squarePattern.active && !running && Wire.writes.size()==initialWrites);
    }
    assert(squareTuning[0].timeScale==1 && squareTuning[0].trim[0]==1);

    // One leg's empirical correction must not alter other directions or raw drive.
    cmd("square-leg forward 1 0.94 1 1"); cmd("square-leg backward 0.96 1 1 1");
    int outputs[4][3]={}; unsigned durations[4]={};
    assert(planSquare(1500,20,1000,outputs,durations));
    assert(outputs[0][0]==-16 && outputs[0][1]==0 && outputs[0][2]==17);
    assert(outputs[2][0]==17 && outputs[2][1]==0 && outputs[2][2]==-17);
    assert(durations[0]==1500 && durations[2]==1440);
    int normal[3]={}; assert(computeWheelCommands(0,1,0,20,normal));
    assert(normal[0]==-17 && normal[2]==17);
    cmd("preview-square 1500 20 1000");
    assert(Wire.writes.size()==initialWrites && Serial.output.find("duration=1440ms")!=std::string::npos);
    cmd("arm"); cmd("square-test forward 1500 20");
    assert(running && !squarePattern.active && testMs==1500 && Wire.motor[0]==-16 && Wire.motor[2]==17);
    endSide(); stopped();
    cmd("arm"); cmd("square-test backward 1500 20");
    assert(testMs==1440 && Wire.motor[0]==17 && Wire.motor[2]==-17); endSide(); stopped();

    // Validate adjusted deadlines before ANY motion; never clamp an invalid leg.
    cmd("square-leg forward 0.5 1 1 1"); cmd("arm");
    auto writes=Wire.writes.size(); cmd("square-endless 50 20 100");
    assert(!running && Wire.writes.size()==writes); cmd("stop");
    cmd("square-leg forward 1.5 1 1 1"); cmd("arm"); writes=Wire.writes.size();
    cmd("square-repeat 3 3000 20 100"); assert(!running && Wire.writes.size()==writes); cmd("stop");
    cmd("square-leg reset");

    // Bounded repeat completes exactly N cycles and preserves one-shot behavior.
    fresh(); cmd("arm"); cmd("square-repeat 3 500 20 100");
    for (unsigned i=1;i<=3;++i) {
        cycle(); assert(squareStats.completed==i && squareStats.attempts==i && squarePattern.cycles==i);
        if (i<3) {
            assert(squarePattern.active && squarePattern.pausing && !running && enabled);
            const auto count=Wire.writes.size(); hostTime=squarePattern.pauseBegan+99; updateTest();
            assert(!running && Wire.writes.size()==count); nextSide(); assert(squarePattern.leg==0 && running);
        }
    }
    stopped(); noResume(); assert(squareStats.streak==3);
    cmd("arm"); cmd("square 500 20 100"); cycle(); stopped(); noResume();

    // Explicit endless request survives many cycles and rollover, not a blocking loop.
    fresh(); hostTime=UINT32_MAX-100; cmd("arm"); cmd("square-endless 500 20 100");
    for (unsigned i=1;i<=20;++i) {
        cycle(); assert(squarePattern.active && squarePattern.cycles==i && squareStats.completed==i);
        nextSide(); assert(running && squarePattern.leg==0);
    }
    cmd("!"); stopped(); noResume(); assert(squareStats.cancelled==1 && squareStats.completed==20);

    // Stop during every leg and every pause, including the completed-cycle pause.
    for (unsigned leg=0;leg<4;++leg) for (bool pause : {false,true}) {
        fresh(); cmd("arm"); cmd("square-endless 500 20 100");
        for (unsigned i=0;i<leg;++i) { endSide(); nextSide(); }
        if (pause) endSide();
        assert(squarePattern.active && squarePattern.leg==leg);
        cmd("stop"); stopped(); noResume();
        assert(squareStats.cancelled==unsigned(!(leg==3 && pause)));
    }

    // Tuning is frozen while armed/running/paused; outputs/deadlines are snapshots.
    fresh(); cmd("square-leg forward 1 0.94 1 1"); cmd("arm");
    cmd("square-leg reset"); assert(squareTuning[0].trim[0]==0.94f);
    cmd("square-endless 500 20 100"); endSide(); writes=Wire.writes.size();
    cmd("square-leg forward 0.5 1 1 1"); cmd("init"); cmd("arm"); cmd("square-repeat 3");
    assert(squareTuning[0].timeScale==1 && Wire.writes.size()==writes && squarePattern.active);
    nextSide(); assert(running && testMs==500); cmd("stop");

    // No-pause mode still writes zero and yields a scheduler tick before restarting.
    fresh(); cmd("arm"); cmd("square-endless 500 20 0"); cycle();
    assert(!running && squarePattern.pausing && squarePattern.cycles==1);
    for (int value : Wire.motor) assert(value==0);
    nextSide(); assert(running && squarePattern.leg==0); cmd("stop");

    // A stop already queued at a cycle deadline must not send next-cycle motion.
    fresh(); cmd("arm"); cmd("square-endless 500 20 100"); cycle();
    hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; writes=Wire.writes.size();
    cmd("stop"); stopped(); assert(Wire.writes.size()==writes+1); noResume();

    // Cached outputs and per-leg durations stay unchanged throughout a run.
    fresh(); cmd("square-leg backward 0.96 1 1 1"); cmd("arm"); cmd("square-repeat 2 500 20 100");
    squareTuning[0].trim[0]=0.5f; squareTuning[2].timeScale=0.5f; // Even direct API-side edits cannot change the active plan.
    endSide(); nextSide(); endSide(); nextSide(); assert(testMs==480);
    endSide(); nextSide(); endSide(); nextSide();
    assert(testMs==500 && Wire.motor[0]==-17); cmd("stop");

    // An end-of-cycle zero failure is not a completed cycle; first fault stays leg 4.
    fresh(); cmd("arm"); cmd("square-endless 500 20 100");
    for (unsigned i=0;i<3;++i) { endSide(); nextSide(); }
    Wire.endResults={4,0}; endSide(); stopped(); noResume();
    assert(broken && squareStats.completed==0 && squareStats.faults==1 && firstWriteFault.leg==4);

    // Failure beginning the next cycle cancels endless mode and counts that attempt once.
    fresh(); cmd("arm"); cmd("square-endless 500 20 100"); cycle();
    Wire.endResults={4,0}; nextSide(); stopped(); noResume();
    assert(broken && squareStats.attempts==2 && squareStats.completed==1 && squareStats.faults==1);
    assert(squareStats.streak==0 && firstWriteFault.leg==1);
    cmd("init"); noResume(); assert(squareStats.completed==1);

    // A failed explicit stop can only retry zeros, never restart a repeat run.
    fresh(); cmd("arm"); cmd("square-endless 500 20 100"); Wire.ack=false; cmd("stop");
    assert(!squarePattern.active && !enabled && broken && stopRetries==4);
    Wire.ack=true; hostTime=lastRetry+legacy::retryIntervalMs; updateTest();
    stopped(); noResume(); assert(stopRetries==0);
    std::cout << "PASS: per-direction square tuning/test, preview/deadlines, bounded/endless cycles, rollover, every stop phase, busy guards, faults and no replay.\n";
}
