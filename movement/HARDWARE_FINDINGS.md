# Photo/schematic review and next diagnostic

Reviewed the four supplied robot photos and `/Users/calvinfisher/Downloads/calvingoon.pdf` on November 10, 2026. No wiring was changed, no serial port was opened by the agent, and no motor was commanded by the agent.

## Established observations

- The photos show an ESP32 development board on a screw-terminal breakout, a Hiwonder motor module, three motor/wheel assemblies and a separate adjustable DC-DC converter. The converter's exact model, setting and complete wire path cannot be established from these views.
- The schematic connects `ME_SDA` to ESP32 GPIO21 and `ME_SCL` to GPIO22, matching the software. This establishes the intended connection, rather than proving continuity in the assembled harness.
- The schematic connects the ESP32's 5V net `ME_5V` to the driver's I2C 5V pin and shares ground. The driver's `VM_+`, `VM_-` and alternate power-input pins are marked unconnected on this sheet. Battery/converter motor power is therefore not documented by this sheet. That is a drawing finding, not proof that the assembled board has no motor supply.
- Driver and encoder indicator LEDs are visible. LEDs and I2C acknowledgments alone do not establish the motor rail's voltage or current capacity.
- The user observed wheel rotation on the earlier program, and reports USB-only power during the current attempts. Earlier rotation is evidence that a usable motor-power path existed at that time. Missing/weak motor power remains a hypothesis, not a diagnosis.

## Software difference worth testing first

**Later result:** all ten address-only probes returned result 0, with SDA/SCL HIGH before each probe. PWR LED blinking continued after the diagnostic stopped transmitting. This establishes stable address acknowledgment during that run and rules out ongoing diagnostic I2C transmissions as the blink trigger; it does not establish motor power or the LED's meaning. A manual [legacy-settings baseline](legacy-motor-test/README.md) now compares the original type=1, polarity=1 and speed-only register 51 transactions against the earlier main controls' differing settings/traffic.

The original `SWRM_Leds-master/src/main.cpp` wrote driver register **51**, the encoder-regulated speed interface. The new timed `drive` mode writes register **31**, the open-loop PWM interface. Both are documented in Hiwonder's tutorial, but equal numerical commands in the two registers do not have equal meaning. The initial PWM cap may not overcome friction, and exact board/firmware behavior has not been validated.

These existing commands compare the two interfaces on one channel, without knowing wheel diameter, channel location or counts per revolution. No further firmware update is required if the current version is already uploaded.

With the robot supported so all wheels are clear, an accessible power disconnect, and the monitor open, enter each command separately and wait for its reply:

```text
init
status
```

If initialization succeeds and status shows no fault, test PWM:

```text
arm
pulse 1 25 500
```

Wait for `Wheel test done; disarmed.` Record whether any wheel moves. Then test the earlier program's register type:

```text
arm
jog 1 2 500
```

This is 2 encoder pulses per 10 ms for 500 ms, not 2% power. The driver can raise PWM internally to try to meet the requested speed. No measured CPR is needed for this short raw speed request. Internal encoder wiring/polarity still affects that regulation. If motion is uncontrolled, disconnect motor power rather than continuing tests. `!` / `stop` is the software stop.

If an I2C fault returns, stop this comparison and inspect the communication/supply setup; do not bypass the fault by forcing the armed flag.

## Interpret the comparison

| PWM test | Speed test | Next interpretation |
| --- | --- | --- |
| No motion | Motion | The board can actuate this channel. Investigate PWM threshold/interface behavior; timed direction mode can be adapted to speed commands if this result is repeatable. |
| Motion | Motion | Both interfaces can actuate this channel at the tested values. Investigate starting duty/load and then the three-wheel mix. |
| Motion | No motion/uncontrolled motion | Investigate the driver's speed-feedback configuration, encoder connections and polarity. Continue treating PWM as open-loop only. |
| No motion | No motion | Still ambiguous: channel connection, supply, command threshold and configuration remain candidates. A motor-input voltage measurement becomes a useful next discriminator. |

If a meter is needed, measure across the motor driver's labeled **VM+ / VM- input**, not M+/M- (motor output) or the I2C 5V pin. Use DC-voltage mode and proper voltage-probe sockets; do not use current mode across a supply. A knowledgeable teammate can help. Do not change a converter trimmer or infer battery hookups from wire colors.

Sources: [Hiwonder register definitions](https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/3._Arduino_checked.html), [Hiwonder product/power information](https://www.hiwonder.com/products/4-channel-encoder-motor-driver), original and current local firmware, supplied photos and schematic. The current product page and tutorial show different board revisions; the installed revision must govern electrical limits.
