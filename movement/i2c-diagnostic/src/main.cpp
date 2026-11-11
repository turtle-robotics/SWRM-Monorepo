/*
Communication-only diagnostic. No geometry, PWM, speed commands, motor type or
encoder-polarity writes are involved. We send only the I2C device address and
ask whether a device acknowledges it, like the probe used in an I2C scanner.

This firmware does NOT stop a previously powered motor driver. Power everything
off before this experiment and keep wheels clear; use physical power removal
if needed. There is deliberately no `arm` or motion command here.

The current movement program retries stop writes every 250ms when idle. This
experiment removes that traffic to test basic contact with the driver alone.
*/
#include <Arduino.h>
#include <Wire.h>
#include "../../include/robot_config.h" // Use exactly the same pins/address/clock as the controls.

constexpr unsigned sampleLimit = 10; // Ten independent observations, then remain quiet.
constexpr unsigned samplePeriodMs = 1000; // One observation per second, easy to compare to LED blinking.
unsigned samples = 0; // Zero probes performed until setup() calls probeDriver().
uint32_t lastSample = 0; // millis timestamp set whenever a probe is performed.

void probeDriver() {
    // Read digital logic levels WITHOUT changing pin direction or driving them.
    // These are NOT voltage measurements and cannot validate 3.3V compatibility.
    const int sdaBefore = digitalRead(config::sda);
    const int sclBefore = digitalRead(config::scl);
    Wire.beginTransmission(config::driverAddress);
    // Intentionally no Wire.write(): no register number or motor payload.
    const uint8_t result = Wire.endTransmission();
    ++samples;
    lastSample = millis();
    Serial.printf("Probe %u/%u addr=0x%02X result=%u SDA=%s SCL=%s\n",
        samples, sampleLimit, unsigned(config::driverAddress), unsigned(result),
        sdaBefore ? "HIGH" : "LOW", sclBefore ? "HIGH" : "LOW");
    if (samples == sampleLimit)
        Serial.println("FINISHED: no more probes until reset. No motor commands sent.");
}

void setup() {
    Serial.begin(config::serialBaud);
    delay(config::startupWaitMs);
    Serial.println("SWRM I2C-ONLY DIAGNOSTIC v1. No motor commands.");
    Serial.println("Results: 0=ACK, 2=NACK, 4=other error, 5=timeout (installed ESP32 Wire library).");
    Serial.println("Pins report digital levels only; motor supply voltage is not measured.");
    Wire.begin(config::sda, config::scl);
    Wire.setClock(config::i2cClockHz);
    Wire.setTimeOut(config::i2cTimeoutMs);
    probeDriver();
}

void loop() {
    if (samples < sampleLimit && millis()-lastSample >= samplePeriodMs) probeDriver();
    delay(config::loopPauseMs);
}
