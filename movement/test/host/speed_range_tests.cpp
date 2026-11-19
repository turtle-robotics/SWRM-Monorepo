#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void cmd(const std::string& text) {
    for(char c:text+"\n") Serial.input.push_back(c);
    while(Serial.available()) loop();
}
void packet(int a,int b,int c) {
    const std::vector<uint8_t> expected={51,uint8_t(int8_t(a)),uint8_t(int8_t(b)),uint8_t(int8_t(c)),0};
    if (Wire.writes.back()!=expected)
        std::cerr<<"Expected "<<a<<","<<b<<","<<c<<" got "<<int(Wire.motor[0])<<","<<int(Wire.motor[1])<<","<<int(Wire.motor[2])<<"\n";
    assert(Wire.writes.back()==expected);
}
void finishSide() { hostTime=began+testMs; updateTest(); }
void nextSide() { hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; updateTest(); }
int main() {
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{}; // Explicit unity diagnostic fixture; boot profile is tested separately.
    static_assert(motion::maxSpeed==127 && legacy::maxCommand==127);
    static_assert(motion::defaultSpeed==20 && legacy::maxPwmDuty==50);
    setup(); assert(Wire.writes.empty());
    cmd("status"); assert(Serial.output.find("drive/twist/square=1..127")!=std::string::npos);
    cmd("preview-square 500 40 1000"); assert(Wire.writes.empty() && !enabled);
    cmd("init"); cmd("arm");
    // Explicit literal high-speed packets, not only a planner self-comparison.
    cmd("drive 0 1 500 40"); assert(running); packet(-35,0,35);
    finishSide(); packet(0,0,0); assert(!enabled);
    cmd("arm"); cmd("drive 1 0 500 60"); assert(running); packet(30,60,30);
    finishSide();
    cmd("arm"); cmd("twist 0 0 1 500 127"); assert(running); packet(-127,127,-127);
    cmd("stop");
    // M2 reversal at both symmetric endpoints, including single transport.
    // -128 must be rejected; negating it would need unrepresentable +128.
    for(bool single:{false,true}) {
        cmd(single ? "transport single" : "transport block"); cmd("init");
        for(int channel:{1,2,3}) for(int speed:{-127,-100,-60,40,60,100,127}) {
            cmd("arm"); cmd("motor "+std::to_string(channel)+" "+std::to_string(speed)+" 500");
            assert(running);
            for(unsigned i=0;i<4;++i)
                assert(Wire.motor[i]==(i==unsigned(channel-1) ? speed*hardwareSign[i] : 0));
            finishSide(); assert(!running && !enabled);
            for(int value:Wire.motor) assert(value==0);
        }
    }
    cmd("transport block"); cmd("init"); cmd("arm");
    for(const char* invalid:{"motor 2 -128 500","motor 1 128 500","motor 3 -1000000 500",
                            "drive 1 0 500 128","drive 0 1 500 -40","twist 0 0 1 500 1000000",
                            "square 500 128 1000","preview-square 500 128 1000"}) {
        const auto writes=Wire.writes.size(); cmd(invalid);
        assert(!running && !squarePattern.active && Wire.writes.size()==writes);
    }
    const auto writes=Wire.writes.size();
    assert(!startTest(128,0,0,500) && !startTest(0,-128,0,500));
    assert(!driveRobot(1,0,0,500,128) && !driveSquare(500,128,1000));
    assert(!sendSpeeds(0,-128,0) && !sendSpeeds(128,0,0) && Wire.writes.size()==writes);
    assert(driveSquare(500,127,1000)); packet(-110,0,110); cmd("stop");
    cmd("arm");
    // Fully complete a high-input square; even 126 avoids platform-dependent
    // half-integer rounding at 127*.5 in the floating-point legacy projection.
    assert(driveSquare(500,126,1000)); packet(-109,0,109);
    finishSide(); nextSide(); packet(63,126,63);
    finishSide(); nextSide(); packet(109,0,-109);
    finishSide(); nextSide(); packet(-63,-126,-63);
    finishSide(); packet(0,0,0); assert(!running && !enabled && !broken);
    // High-speed write errors still latch/stop; the next ZERO does not resume.
    cmd("arm"); Wire.endResults={4,0}; cmd("drive 0 1 500 80");
    assert(broken && !enabled && !running && firstWriteFault.error==4); packet(0,0,0);
    const auto afterFault=Wire.writes.size(); hostTime+=10000; updateTest(); assert(Wire.writes.size()==afterFault);
    // PWM duty limits remain independent of speed encoding.
    cmd("init"); cmd("mode pwm"); cmd("init"); cmd("arm");
    const auto beforePwm=Wire.writes.size(); cmd("motor 1 51 500");
    assert(!running && Wire.writes.size()==beforePwm && !sendSpeeds(51,0,0));
    cmd("motor 1 50 500"); assert(running && Wire.activeReg==31 && Wire.motor[0]==50);
    finishSide(); assert(!running && !enabled);
    std::cout<<"PASS: expanded speed parser/C++ range, literal 40/60/127 packets, M2 endpoint reversal in block/single mode, no wrap, full square, fault cancellation, default/PWM limits preserved.\n";
}
