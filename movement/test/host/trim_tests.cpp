#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void cmd(const std::string& s) {
    for(char c:s+"\n") Serial.input.push_back(c);
    while(Serial.available()) loop();
}
void reset() {
    Wire=HostWire{}; Serial=HostSerial{}; hostTime=0;
    configured=enabled=running=broken=false;
    squarePattern=SquarePattern{}; selectedMixer=Mixer::original; selectedOrientation=DriveOrientation::v9;
    selectedType=1; selectedPolarity=0; singleWrites=pwmMode=false;
    selectedClock=legacy::busHz; writeGapMs=0; stopRetries=used=0; discard=false;
    const float angles[3]={150,30,270};
    for(unsigned i=0;i<3;++i) { wheelAngle[i]=angles[i]; wheelSign[i]=1; motorTrim[i]=1; activeOutputs[i]=0; }
    setup(); assert(Wire.writes.empty());
}
void expected(float x,float y,float w,int speed,int a,int b,int c) {
    int out[3]={}; assert(computeWheelCommands(x,y,w,speed,out));
    assert(out[0]==a && out[1]==b && out[2]==c);
}
// Arguments are logical requests; M2 must be negated in the actual packet.
void motors(int a,int b,int c) {
    assert(Wire.motor[0]==a && Wire.motor[1]==-b && Wire.motor[2]==c && Wire.motor[3]==0);
}
int main() {
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{}; // Explicit unity diagnostic fixture; boot profile is tested separately.
    assert(motorTrim[0]==1 && motorTrim[1]==1 && motorTrim[2]==1);
    reset(); expected(0,-1,0,20,-12,12,0);
    cmd("trim 2 1.2"); assert(motorTrim[1]==1.2f);
    expected(0,-1,0,20,-12,14,0); expected(0,1,0,20,12,-14,0);
    cmd("preview 0 -1");
    assert(Serial.output.find("M1=-12 M2=14 M3=0")!=std::string::npos);
    assert(Wire.writes.empty() && Wire.readRequests==0 && !configured && !enabled);
    cmd("mixer geometry"); expected(0,-1,0,20,-17,21,0);
    expected(0,1,0,20,17,-21,0); // Negative speed scales without changing direction.
    cmd("trim 2 1.5");
    expected(0,0,1,127,-85,-127,-85); // Shared cap, not independent clipping.
    expected(0,-1,0,127,-85,127,0); // Zero request remains zero after trim/cap.
    expected(0,0,1,30,-30,-45,-30); // Raised wheel cap also applies to trim headroom.
    for(const char* invalid:{"trim 0 1.2","trim 4 1.2","trim 2 nan","trim 2 inf",
                            "trim 2 -1","trim 2 0.49","trim 2 1.51","trim 2 1.2junk",
                            "trim all 1","trim 2","trim reset extra"}) {
        cmd(invalid); assert(motorTrim[0]==1 && motorTrim[1]==1.5f && motorTrim[2]==1);
    }
    assert(!setMotorTrim(0,1) && !setMotorTrim(4,1) && !setMotorTrim(2,NAN));
    assert(Wire.writes.empty() && Wire.readRequests==0);
    cmd("trim reset"); expected(0,0,1,30,-30,-30,-30);
    cmd("trim 2 0.5"); expected(0,-1,0,20,-17,9,0);
    cmd("trim reset"); cmd("mixer original"); cmd("trim 2 1.2");
    cmd("init"); assert(configured && motorTrim[1]==1.2f); // init preserves trim.
    cmd("arm"); const auto count=Wire.writes.size();
    cmd("trim reset"); cmd("trim 2 0.8");
    assert(motorTrim[1]==1.2f && Wire.writes.size()==count);
    assert(!resetMotorTrims() && !setMotorTrim(1,1.2f));
    cmd("motor 2 20 500"); motors(0,20,0); // Raw speed test deliberately bypasses trim.
    cmd("stop"); cmd("arm"); cmd("original"); motors(58,-58,0);
    cmd("stop"); cmd("arm"); assert(driveRobot(0,-1,0,500,20)); motors(-12,14,0);
    cmd("trim reset"); assert(motorTrim[1]==1.2f); cmd("stop");
    const auto afterStop=Wire.writes.size(); cmd("trim 2 1.1");
    assert(configured && Wire.writes.size()==afterStop); // No init, no hardware write.
    cmd("trim 2 1.2"); cmd("preview-square 50 20 100");
    assert(Serial.output.find("M1=7 M2=8 M3=13")!=std::string::npos);
    cmd("arm"); assert(driveSquare(50,20,100)); motors(-12,14,0);
    hostTime=began+50; updateTest(); motors(0,0,0);
    assert(squarePattern.active && squarePattern.pausing);
    cmd("trim reset"); cmd("trim 2 0.8");
    assert(!setMotorTrim(2,0.8f) && !resetMotorTrims() && motorTrim[1]==1.2f);
    hostTime=squarePattern.pauseBegan+100; updateTest(); motors(7,8,13);
    cmd("stop");
    // Directly corrupting settings cannot produce a partial motor command.
    motorTrim[1]=NAN; int out[3]={99,99,99};
    assert(!computeWheelCommands(0,-1,0,20,out));
    assert(out[0]==0 && out[1]==0 && out[2]==0);
    assert(resetMotorTrims());
    // PWM is a separate interface; trim never changes raw duty.
    cmd("mode pwm"); cmd("init"); assert(setMotorTrim(2,1.5f));
    cmd("arm"); cmd("motor 2 20 500"); motors(0,20,0);
    std::cout<<"PASS: unity/defaults, per-motor trim, sign/zero preservation, common ceiling, previews, raw bypasses, square snapshots, edit guards and reset.\n";
}
