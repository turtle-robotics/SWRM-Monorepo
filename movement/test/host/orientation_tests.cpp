#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void cmd(const std::string& text) {
    for(char c:text+"\n") Serial.input.push_back(c);
    while(Serial.available()) loop();
}
void packet(int a,int b,int c) {
    assert((Wire.writes.back()==std::vector<uint8_t>{51,uint8_t(int8_t(a)),uint8_t(int8_t(b)),uint8_t(int8_t(c)),0}));
}
void endSide() { hostTime=began+testMs; updateTest(); }
void nextSide() { hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; updateTest(); }
int main() {
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{}; // Explicit unity diagnostic fixture; boot profile is tested separately.
    assert(selectedOrientation==DriveOrientation::lab && selectedMixer==Mixer::legacy);
    setup(); cmd("preview 0 1 0 20");
    assert(Serial.output.find("PREVIEW DRIVER: M1=-17 M2=0 M3=17")!=std::string::npos);
    cmd("preview 1 0 0 20");
    assert(Serial.output.find("PREVIEW DRIVER: M1=10 M2=20 M3=10")!=std::string::npos);
    cmd("preview-square 1200 20 300");
    assert(Serial.output.find("Leg 1/4 forward X=0 Y=1")!=std::string::npos);
    assert(Serial.output.find("Leg 3/4 backward X=0 Y=-1")!=std::string::npos);
    assert(Wire.writes.empty() && Wire.readRequests==0);
    // Test every mixer against its unchanged V9 output, including W. The new
    // frame changes input interpretation only, not calibration or M2 inversion.
    for(Mixer m:{Mixer::legacy,Mixer::original,Mixer::geometry}) {
        selectedMixer=m;
        int old[3]={},adapted[3]={};
        for(float x:{-0.5f,0.0f,0.5f}) for(float y:{-0.5f,0.0f,0.5f}) {
            assert(setOrientation(DriveOrientation::v9));
            assert(computeWheelCommands(-0.5f*x+0.8660254037844386f*y,
                -0.8660254037844386f*x-0.5f*y,0.2f,20,old));
            assert(setOrientation(DriveOrientation::lab));
            assert(computeWheelCommands(x,y,0.2f,20,adapted));
            for(unsigned i=0;i<3;++i) assert(old[i]==adapted[i]);
        }
    }
    selectedMixer=Mixer::legacy;
    cmd("orientation invalid"); assert(selectedOrientation==DriveOrientation::lab);
    cmd("orientation v9"); cmd("preview 1 -1 0 20");
    assert(Serial.output.find("PREVIEW DRIVER: M1=-15 M2=5 M3=20")!=std::string::npos);
    cmd("init"); assert(selectedOrientation==DriveOrientation::v9);
    cmd("orientation lab"); const auto n=Wire.writes.size();
    cmd("arm"); const auto armedWrites=Wire.writes.size();
    cmd("orientation v9"); assert(selectedOrientation==DriveOrientation::lab && Wire.writes.size()==armedWrites);
    assert(!setOrientation(DriveOrientation::v9));
    assert(driveRobot(0,1,0,1000,20)); packet(-17,0,17); cmd("stop");
    cmd("arm"); cmd("drive 1 0 1000 20"); packet(10,20,10); cmd("stop");
    cmd("arm"); cmd("drive 0 -1 1000 20"); packet(17,0,-17); cmd("stop");
    cmd("arm"); cmd("drive -1 0 1000 20"); packet(-10,-20,-10); cmd("stop");
    cmd("arm"); cmd("square 1200 20 300"); packet(-17,0,17);
    endSide(); packet(0,0,0);
    cmd("orientation v9"); assert(selectedOrientation==DriveOrientation::lab);
    assert(!setOrientation(DriveOrientation::v9));
    nextSide(); packet(10,20,10);
    endSide(); nextSide(); packet(17,0,-17);
    endSide(); nextSide(); packet(-10,-20,-10);
    endSide(); packet(0,0,0);
    assert(!enabled && !running && !squarePattern.active && Wire.writes.size()>n);
    // Raw motor-channel diagnostics must be unaffected by public orientation.
    cmd("arm"); cmd("motor 2 10 500"); packet(0,-10,0); cmd("stop");
    cmd("orientation v9"); cmd("arm"); cmd("motor 2 10 500"); packet(0,-10,0); cmd("stop");
    const auto writes=Wire.writes.size();
    cmd("orientation lab"); cmd("status"); assert(Wire.writes.size()==writes);
    assert(Serial.output.find("Orientation=lab: +X intended RIGHT, +Y intended FORWARD")!=std::string::npos);
    // Physical log regression: forward and right finish, backward's exact packet
    // returns 4, the following zero succeeds. Abort is a bus fault, not a square
    // timeout; a successful stop MUST NOT clear the error or restart later legs.
    cmd("arm"); cmd("square 1200 20 300"); packet(-17,0,17);
    endSide(); nextSide(); packet(10,20,10); endSide();
    const auto beforeFailedLeg=Wire.writes.size(); Serial.output.clear();
    Wire.endResults={4,0}; nextSide();
    packet(0,0,0);
    assert(Wire.writes.size()==beforeFailedLeg+1 && Wire.endResults.empty());
    assert(lastError==4 && lastReg==51 && broken && !enabled && !running && !squarePattern.active);
    assert(Serial.output.find("result=4 data=11 00 EF 00")!=std::string::npos);
    assert(Serial.output.find("result=0 data=00 00 00 00")!=std::string::npos);
    assert(Serial.output.find("SQUARE DONE")==std::string::npos);
    const auto abortedWrites=Wire.writes.size(); hostTime+=10000; updateTest();
    assert(Wire.writes.size()==abortedWrites); // No hidden global timer or auto-resume.
    // Field-tested default: 1.2s sides, one second at zero between sides.
    cmd("init"); cmd("preview-square");
    assert(Serial.output.find("side=1200ms speed=20 pause=1000ms total=7800ms")!=std::string::npos);
    cmd("arm"); assert(driveSquare());
    assert(squarePattern.sideMs==1200 && squarePattern.pauseMs==1000);
    for(unsigned i=0;i<3;++i) {
        endSide(); const auto count=Wire.writes.size();
        hostTime=squarePattern.pauseBegan+999; updateTest();
        assert(squarePattern.pausing && !running && Wire.writes.size()==count);
        nextSide(); assert(running);
    }
    endSide(); packet(0,0,0); assert(!squarePattern.active && !enabled);
    std::cout<<"PASS: observed-axis adapter, V9 rollback, every model/W equivalence, public directions, square order, raw bypass, edit guards, previews and exact leg-3 transient-fault regression.\n";
}
