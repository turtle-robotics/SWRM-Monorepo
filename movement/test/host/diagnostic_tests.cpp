// This small adapter supplies only the digitalRead API needed by the diagnostic.
// Reuse the existing simulated serial/I2C, without touching a physical port.
inline int digitalRead(int) { return 1; }
#include "../../i2c-diagnostic/src/main.cpp"
#include <cassert>
#include <iostream>

int main() {
    setup();
    assert(samples == 1);
    assert(Serial.output.find("result=0") != std::string::npos);
    for (unsigned n = 1; n < sampleLimit; ++n) {
        hostTime += samplePeriodMs;
        loop();
    }
    assert(samples == sampleLimit);
    assert(Serial.output.find("FINISHED:") != std::string::npos);
    const auto quietOutput = Serial.output;
    hostTime += 30000;
    loop();
    assert(Serial.output == quietOutput);
    assert(Wire.writes.empty()); // No register writes or motor payloads, including zero commands.
    assert(Wire.readRequests == 0); // No hidden encoder requests.
    for (auto m : Wire.motor) assert(m == 0);
    samples = 0;
    Wire.ack = false;
    probeDriver();
    assert(Serial.output.find("result=4") != std::string::npos);
    assert(Wire.writes.empty());
    std::cout << "PASS: diagnostic probes only, no motor register writes, ten samples, then quiet; failure reported.\n";
}
