#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void cmd(const char* text) {
    char line[96]; std::snprintf(line,sizeof(line),"%s",text); command(line);
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
    setup(); cmd("init");
}
void endSide() { hostTime=began+testMs; updateTest(); }
void nextSide() { hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; updateTest(); }
void finish() {
    for (unsigned i=0;i<3;++i) { endSide(); nextSide(); }
    endSide();
    assert(!running && !enabled && !squarePattern.active && !broken);
}
int main() {
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{}; // Explicit unity diagnostic fixture; boot profile is tested separately.
    fresh(); cmd("arm");
    // Before V12 the deadline was already ticking during REQUEST/DRIVER text
    // and the I2C call. Slow logs and the optional gap must not shorten a side.
    Serial.printDelayMs=2; Wire.transactionDelayMs=7; writeGapMs=10;
    hostTime=0xfffffff0u;
    assert(driveSquare(500,20,100));
    assert(began==Wire.transactionEnded && uint32_t(hostTime-began)>0);
    Serial.printDelayMs=0;
    auto writes=Wire.writes.size();
    hostTime=began+499; updateTest(); assert(running && Wire.writes.size()==writes);
    endSide(); assert(squarePattern.pausing && squarePattern.pauseBegan==Wire.transactionEnded);
    hostTime=squarePattern.pauseBegan+99; updateTest(); assert(!running);
    nextSide(); assert(running && began==Wire.transactionEnded);
    for (unsigned i=0;i<2;++i) { endSide(); nextSide(); }
    endSide(); assert(squareStats.completed==1 && squareStats.streak==1);

    // Ten consecutive SOFTWARE schedules exercise all eight motion/zero writes
    // per square. They are not ten physical demonstrations.
    fresh();
    for (unsigned trial=1;trial<=10;++trial) {
        cmd("arm"); assert(driveSquare(1500,20,1000)); finish();
        assert(squareStats.attempts==trial && squareStats.completed==trial);
        assert(squareStats.streak==trial && squareStats.best==trial);
    }
    assert(squareStats.faults==0 && squareStats.cancelled==0);
    writes=Wire.writes.size(); hostTime+=10000; updateTest();
    assert(Wire.writes.size()==writes); // No automatic repeat.
    cmd("init"); assert(squareStats.streak==10); // Cannot hide failure with init.

    // Failed backward write then successful stop: retain exact packet/leg/time,
    // count ONE failed attempt, reset streak, never resume nonzero commands.
    cmd("arm"); assert(driveSquare(1500,20,1000));
    endSide(); nextSide(); endSide();
    Wire.endResults={4,0}; Wire.transactionDelayMs=6; nextSide();
    assert(broken && !running && !enabled && !squarePattern.active);
    assert(squareStats.attempts==11 && squareStats.completed==10 && squareStats.faults==1);
    assert(squareStats.streak==0 && squareStats.best==10);
    assert(firstWriteFault.valid && firstWriteFault.leg==3 && firstWriteFault.error==4);
    assert(firstWriteFault.reg==51 && firstWriteFault.elapsed==6 && firstWriteFault.clock==100000);
    assert(firstWriteFault.data[0]==0x11 && firstWriteFault.data[1]==0 && firstWriteFault.data[2]==0xef);
    const uint32_t faultAt=firstWriteFault.at;
    Wire.endResults={5,0}; cmd("stop"); // A later error cannot replace initial evidence.
    assert(firstWriteFault.error==4 && firstWriteFault.at==faultAt && squareStats.faults==1);
    writes=Wire.writes.size(); hostTime+=250; updateTest();
    assert(Wire.writes.size()==writes+1 && !stopRetries); // The scheduled ZERO retry succeeds.
    writes=Wire.writes.size(); hostTime+=10000; updateTest(); assert(Wire.writes.size()==writes);
    cmd("init"); assert(!firstWriteFault.valid && squareStats.faults==1 && squareStats.streak==0);

    // First-leg and final-stop faults are included; a successful zero-only
    // retry must never convert a failed square into a completion.
    for (bool finalStop : {false,true}) {
        fresh(); cmd("arm");
        if (!finalStop) {
            Wire.endResults={4,0}; assert(!driveSquare());
            assert(firstWriteFault.leg==1);
        } else {
            assert(driveSquare());
            for (unsigned i=0;i<3;++i) { endSide(); nextSide(); }
            Wire.endResults={5,4}; endSide(); assert(firstWriteFault.leg==4 && stopRetries);
            hostTime+=250; updateTest(); assert(stopRetries && broken); // First zero retry also fails.
            hostTime+=250; updateTest(); assert(!stopRetries && broken); // Next zero succeeds.
        }
        assert(squareStats.attempts==1 && squareStats.faults==1 && squareStats.completed==0);
        assert(Serial.output.find("SQUARE DONE")==std::string::npos);
    }
    // Intentional cancellation is recorded separately and resets the streak.
    for (bool pause : {false,true}) {
        fresh(); cmd("arm"); assert(driveSquare()); if (pause) endSide();
        cmd("square-stats reset"); assert(squareStats.attempts==1); // Busy reset rejected.
        cmd("stop"); assert(squareStats.cancelled==1 && squareStats.faults==0);
        cmd("square-stats reset"); assert(squareStats.attempts==0);
    }
    std::cout<<"PASS: acknowledged-write deadlines with slow logging/bus/gap and rollover; 10 simulated squares; first/third/final faults, retained packet evidence, init-preserved counts, cancellation and zero-only recovery.\n";
}
