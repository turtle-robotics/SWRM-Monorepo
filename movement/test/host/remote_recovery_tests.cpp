#include "../../legacy-motor-test/src/main.cpp"
#include <cassert>
#include <iostream>

void fresh() {
    Wire=HostWire{}; Serial=HostSerial{}; hostTime=0;
    configured=enabled=running=broken=spinActive=false;
    squarePattern=SquarePattern{}; squareStats=SquareStats{}; firstWriteFault=WriteFault{};
    pendingRobotCommand.store(robot::Command::none);
    pendingStatusRequest.store(false); maintenanceBusy.store(false);
    selectedType=1; selectedPolarity=0; singleWrites=pwmMode=false;
    selectedClock=legacy::busHz; writeGapMs=stopRetries=used=0; discard=false;
    selectedMixer=Mixer::legacy; selectedOrientation=DriveOrientation::lab;
    for (unsigned i=0;i<3;++i) { motorTrim[i]=1; activeOutputs[i]=0; }
    for (unsigned i=0;i<4;++i) squareTuning[i]=robot::selected.square.legs[i];
    setup(); assert(Wire.writes.empty()); // Serial-only target remains manual.
}
void stopped() {
    assert(!enabled && !running && !squarePattern.active && !spinActive);
    for (int value : Wire.motor) assert(value==0);
}
void noResume() {
    updateRobotMovement(); const auto writes=Wire.writes.size();
    hostTime+=10000; updateRobotMovement(); stopped(); assert(Wire.writes.size()==writes);
}
void onlyZeros() {
    for (const auto& packet : Wire.writes) if (packet[0]==51 || packet[0]==31)
        for (unsigned i=1;i<packet.size();++i) assert(packet[i]==0);
}
int main() {
    using robot::Result;
    const char initBytes[]={'i','n','i','t'}, statusBytes[]={'s','t','a','t','u','s'};
    fresh();
    auto writes=Wire.writes.size(); const auto reads=Wire.readRequests;
    assert(handleRobotCommand(statusBytes,sizeof(statusBytes))==Result::reported);
    assert(Wire.writes.size()==writes && Wire.readRequests==reads && !getRobotStatus().ready);
    assert(handleRobotCommand("init\0x",6)==Result::unknown);
    assert(handleRobotCommand(initBytes,sizeof(initBytes))==Result::ready);
    assert((Wire.writes==std::vector<std::vector<uint8_t>>{{51,0,0,0,0},{20,1},{21,0},{51,0,0,0,0}}));
    stopped(); assert(getRobotStatus().ready); noResume();

    // Status is read-only even during movement/pause. Init must reject both.
    for (bool pause : {false,true}) {
        fresh(); assert(handleRobotCommand("init")==Result::ready);
        assert(handleRobotCommand("square")==Result::started);
        if (pause) { hostTime=began+testMs; updateRobotMovement(); }
        writes=Wire.writes.size(); const auto runningBefore=running;
        assert(handleRobotCommand("status")==Result::reported);
        assert(handleRobotCommand("init")==Result::busy);
        assert(Wire.writes.size()==writes && squarePattern.active && running==runningBefore);
        stopRobotMovement(); noResume();
    }
    fresh(); assert(handleRobotCommand("init")==Result::ready);
    assert(armMovement()); writes=Wire.writes.size();
    assert(handleRobotCommand("init")==Result::busy && enabled && Wire.writes.size()==writes);
    stopRobotMovement();

    // Stop dominates queued init and repeated status cannot replace it.
    fresh(); assert(queueRobotCommand("init",4)==Result::queued);
    assert(queueRobotCommand("square",6)==Result::busy);
    for (unsigned i=0;i<20;++i) assert(queueRobotCommand("status",6)==Result::queued);
    assert(queueRobotCommand("stop",4)==Result::queued); updateRobotMovement();
    assert(!configured && Wire.writes.size()==1); onlyZeros(); noResume();

    // Failed preflight ZERO must not write motor type/polarity or clear fault.
    fresh(); Wire.endResults={4,0};
    assert(handleRobotCommand("init")==Result::fault && broken && !configured);
    assert(firstWriteFault.error==4 && firstWriteFault.reg==51);
    for (const auto& packet : Wire.writes) assert(packet[0]==51);
    noResume();
    fresh(); Wire.endResults={0,4,0};
    assert(handleRobotCommand("init")==Result::fault && firstWriteFault.reg==20);
    Wire.endResults={4,0};
    assert(handleRobotCommand("init")==Result::fault);
    assert(getRobotStatus().firstReg==20 && getRobotStatus().reg==51);
    // Explicit recovery needs all acknowledgements; errors in every later stage
    // keep the fault latched. Zero retries alone cannot make the robot ready.
    for (const auto& results : {std::deque<int>{0,4,0},std::deque<int>{0,0,4,0},std::deque<int>{0,0,0,4}}) {
        fresh(); Wire.endResults=results; assert(handleRobotCommand("init")==Result::fault);
        assert(broken && !configured); stopped(); onlyZeros();
        if (stopRetries) {
            writes=Wire.writes.size(); assert(handleRobotCommand("init")==Result::busy);
            assert(Wire.writes.size()==writes);
            hostTime=lastRetry+legacy::retryIntervalMs; updateRobotMovement();
        }
        assert(broken && !getRobotStatus().ready); noResume();
        assert(handleRobotCommand("init")==Result::ready);
        assert(!broken && configured && getRobotStatus().ready); noResume();
    }

    // Commands arriving DURING preflight cannot turn recovery into a start.
    fresh();
    Wire.transactionHook=[] {
        Wire.transactionHook=nullptr;
        assert(queueRobotCommand("spin",4)==Result::busy);
        assert(queueRobotCommand("status",6)==Result::queued);
        assert(queueRobotCommand("stop",4)==Result::queued);
    };
    assert(handleRobotCommand("init")==Result::busy);
    for (const auto& packet : Wire.writes) assert(packet[0]==51);
    noResume();
    fresh();
    Wire.transactionHook=[] {
        if (Wire.tx[0]!=20) return;
        Wire.transactionHook=nullptr;
        assert(queueRobotCommand("square",6)==Result::busy);
        assert(queueRobotCommand("stop",4)==Result::queued);
    };
    assert(handleRobotCommand("init")==Result::busy);
    onlyZeros(); noResume();

    // Fault status retains exact first-packet context until deliberate recovery.
    fresh(); assert(handleRobotCommand("init")==Result::ready);
    assert(handleRobotCommand("square")==Result::started);
    Wire.endResults={4,0}; hostTime=began+testMs; updateRobotMovement();
    const auto status=getRobotStatus();
    assert(status.fault && !status.ready && !status.armed && !status.running);
    assert(status.firstError==4 && status.firstReg==51 && status.faultLeg==1);
    writes=Wire.writes.size(); assert(handleRobotCommand("status")==Result::reported);
    assert(Wire.writes.size()==writes); assert(handleRobotCommand("square")==Result::fault);
    assert(queueRobotCommand("init",4)==Result::queued); updateRobotMovement();
    assert(getRobotStatus().ready && !getRobotStatus().fault); noResume();
    assert(handleRobotCommand("spin")==Result::started); stopRobotMovement(); noResume();
    std::cout << "PASS: remote recovery robot " << robot::selected.identity
              << " read-only status, zero-first init, busy/retry guards, stop priority, no configuration after failed zero, latched faults, no queued/concurrent starts and explicit restart only.\n";
}
