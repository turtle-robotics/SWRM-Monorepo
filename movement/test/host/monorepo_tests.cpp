#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void cmd(const std::string& text) {
    for(char c:text+"\n") Serial.input.push_back(c);
    while(Serial.available()) loop();
}
void packet(int a,int b,int c) {
    assert(Wire.motor[0]==a && Wire.motor[1]==b && Wire.motor[2]==c && Wire.motor[3]==0);
    assert((Wire.writes.back()==std::vector<uint8_t>{51,uint8_t(int8_t(a)),uint8_t(int8_t(b)),uint8_t(int8_t(c)),0}));
}
void finishSide() { hostTime=began+testMs; updateTest(); }
void nextSide() { hostTime=squarePattern.pauseBegan+squarePattern.pauseMs; updateTest(); }
int main() {
    for (auto& tuning : squareTuning) tuning=SquareLegTuning{}; // Explicit unity diagnostic fixture; boot profile is tested separately.
    assert(selectedMixer==Mixer::legacy && selectedType==1 && selectedPolarity==0 && !singleWrites);
    setup(); assert(Wire.writes.empty() && selectedOrientation==DriveOrientation::lab);
    cmd("orientation v9"); // Retain literal historical V9 regression cases.
    cmd("preview 0 -1 0 20");
    assert(Serial.output.find("M1=0 M2=-17 M3=17")!=std::string::npos);
    assert(Serial.output.find("PREVIEW DRIVER: M1=0 M2=17 M3=17")!=std::string::npos);
    cmd("preview-square 1200 20 300");
    assert(Serial.output.find("total=5700ms")!=std::string::npos && Wire.writes.empty());
    cmd("preview-square 1800 20 200"); assert(Serial.output.find("total=7800ms")!=std::string::npos);
    cmd("preview-square 2300 20 200"); assert(Serial.output.find("total=9800ms")!=std::string::npos);
    cmd("init");
    // Independently asserted literal wire patterns: catches a missing OR double
    // M2 inversion, and confirms public X-right/Y-backward in all three mixers.
    cmd("arm"); cmd("drive 0 -1 1000 20"); packet(0,17,17); cmd("stop");
    cmd("arm"); cmd("drive 0 1 1000 20"); packet(0,-17,-17); cmd("stop");
    cmd("arm"); cmd("drive 1 0 1000 20"); packet(-20,-10,10); cmd("stop");
    cmd("arm"); cmd("drive -1 0 1000 20"); packet(20,10,-10); cmd("stop");
    cmd("mixer original"); cmd("arm"); cmd("drive 0 -1 1000 20"); packet(-12,-12,0); cmd("stop");
    cmd("mixer geometry"); cmd("arm"); cmd("drive 0 -1 1000 20"); packet(-17,-17,0); cmd("stop");
    // Raw logical motor 2 and PWM also get exactly one hardware sign reversal.
    cmd("arm"); cmd("motor 2 10 500"); packet(0,-10,0); cmd("stop");
    cmd("mode pwm"); cmd("init"); cmd("arm"); cmd("motor 2 20 500");
    assert(Wire.activeReg==31 && Wire.motor[1]==-20); cmd("stop");
    cmd("mode speed"); cmd("mixer legacy"); cmd("init");
    cmd("trim 2 1.2"); cmd("arm"); cmd("drive 0 -1 500 20"); packet(0,21,17); cmd("stop");
    cmd("trim reset");
    for(const char* preset:{"square 1200 20 300","square 1800 20 200","square 2300 20 200"}) {
        cmd("arm"); const uint32_t start=hostTime;
        cmd(preset); assert(squarePattern.active); packet(0,17,17);
        finishSide(); packet(0,0,0); nextSide(); packet(-20,-10,10);
        finishSide(); nextSide(); packet(0,-17,-17);
        finishSide(); nextSide(); packet(20,10,-10);
        finishSide(); packet(0,0,0);
        assert(!running && !enabled && !squarePattern.active && hostTime-start>=5700);
    }
    cmd("preview-square"); assert(Serial.output.find("total=7800ms")!=std::string::npos);
    // Geometry edits cannot silently change the retained legacy comparison.
    cmd("wheel 1 30 -1"); int out[3]={};
    assert(computeWheelCommands(0,-1,0,20,out) && out[0]==0 && out[1]==-17 && out[2]==17);
    std::cout<<"PASS: monorepo legacy baseline, original axes, all mixer packets, exactly one M2 inversion, trim/PWM/raw paths, and 5.7/7.8/9.8s square presets.\n";
}
