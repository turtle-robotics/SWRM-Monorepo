# Run relative kiwi-drive moves today

**New to Terminal or PlatformIO?** Use the detailed [step-by-step walkthrough](WALKTHROUGH.md), including connection, exact commands, monitor setup, notes and troubleshooting.

**Longer direct motor tests:** `pulse` and `jog` now accept up to **5000 ms** and report remaining time while active. Rebuild/upload to use these new limits. See [the three-second test procedure](LONG_MOTOR_TESTS.md). Their existing PWM/speed caps are unchanged; `drive` / `motors` retain their two-second caps.

**Troubleshooting update:** direct-monitor Backspace/DEL now edits the buffered command, Ctrl+U clears it, and unsupported control bytes reject the whole line with a specific error. Each submitted nonempty line is printed as `Command received: ...`. `status` retains the last I²C write error/register until `init`, and explicitly notes that motor supply is not measured. These changes take effect after rebuilding/uploading this version. A successful timed command means writes were acknowledged; it does not prove motor voltage or motion. USB-only power needs an actual motor-supply check before diagnosing a stationary wheel as a mixer problem.

This is a standalone PlatformIO project for the **ESP32 + Hiwonder-compatible I²C motor driver** expected by the existing `SWRM_Leds-master/src/main.cpp`. It uses three motor/encoder channels and keeps channel 4 at zero. It contains no display, Wi-Fi, or UWB code. The original project is unchanged.

**You can now start without measuring wheel diameter, encoder counts, or channel placement.** Upload once, then use `drive X Y` to send a definite timed motor pattern. Observe the actual motion and label it in your notes. See [FIRST_RUN.md](FIRST_RUN.md) for the shortest path and [TEST_NOTES.md](TEST_NOTES.md) for a notes sheet.

| First-day command | Requested pattern (500 ms at default power) |
| --- | --- |
| `drive 1 0` | Assumed forward: M1=0, M2=-17, M3=+17 percent PWM |
| `drive -1 0` | Opposite: M1=0, M2=+17, M3=-17 |
| `drive 0 1` | Assumed left: M1=+20, M2=-10, M3=-10 |
| `drive 0 -1` | Opposite: M1=-20, M2=+10, M3=+10 |
| `drive 1 1` | Combined direction, scaled so no wheel exceeds the selected duty |

Type `arm` before **each** trial. `drive 1 0 1000` overrides the duration to 1000 ms. `drive 1 0 500 25` selects 500 ms and 25% duty. X/Y are dimensionless direction weights in -1..1, not centimeters. Defaults are 500 ms and 20% duty; limits are 50..2000 ms and 1..25% duty. Small inputs may round to zero or fail to overcome friction. All tests finish by sending zeros and disarming.

For direct experimentation, `motors 0 -20 20` sends exactly those three signed PWM commands, in driver-channel order, for 500 ms. Add a fourth number for time, e.g. `motors 0 -20 20 800`. This command ignores all wheel geometry and encoder settings. Use `pulse` to isolate one wheel if helpful. `stop` (or `!`) and Enter stops/disarms during a trial.

**What is consistent:** with unchanged settings, the same command sends the same motor pattern. **What is unknown:** its actual direction, turning component, speed and distance on your robot. You may redefine your front after observing a translation. If a pattern spins the chassis, reorienting the robot alone won't turn that into straight translation; try raw patterns or edit the motor signs/channel-angle assignments later with `wheel`. PWM duty is not a measured speed and load/battery charge can change the response. Neither encoders nor localization are read during timed PWM trials.

You can do the measurement steps later. The separate **calibrated** mode uses `move`:

| Command | Requested movement |
| --- | --- |
| `move 10 0` | 10 centimeters forward |
| `move -10 0` | 10 centimeters backward |
| `move 0 10` | 10 centimeters left |
| `move 0 -10` | 10 centimeters right |
| `move 10 10` | A diagonal: 10 cm forward and 10 cm left |

For **`move` only**, X = forward/backward and Y = left/right, in centimeters. Each move starts from the robot's current location and facing direction. This convention intentionally differs from the old demo. One meter would be 100 cm; initial firmware limits each calibrated move's vector length to 50 cm. `move` retains its calibration requirement so it never pretends to know centimeters before you measure them. `drive` needs no scale calibration.

The code builds for the generic `esp32dev` target, selected for the ESP-WROOM-32 development boards in your parts list. This replaces the old project's uPesy-specific target. A successful build does not confirm the physical board, wiring, wheel mapping, driver firmware, or movement accuracy. Use a raised-wheel first trial and the robot's established power/wiring arrangement. No firmware was uploaded or physically tested while preparing this project.

## What your parts list confirms

| Part | Consequence for this project |
| --- | --- |
| [Hiwonder 4-channel encoder motor driver](https://www.hiwonder.com/products/4-channel-encoder-motor-driver) | Matches the intended driver family; still verify installed revision, address and wiring. |
| [Adafruit 4638 motor, manufacturer page](https://www.adafruit.com/product/4638) | N20 motor, 1:50 gearbox, nominal 6 V motor supply. Keep Hiwonder's N20 type setting `2`. |
| ESP-WROOM-32 development boards, [standalone listing](https://www.amazon.com/dp/B08D5ZD528) and [board/breakout listing](https://www.amazon.com/dp/B0C8DBN29X) | Use generic ESP32 target `esp32dev`; the breakout listing describes a USB-C 38-pin version. Identify which board is actually installed and connect to its own USB port. |
| 3.7 V lithium cells | Cell voltage alone does not identify the assembled motor supply. Check the existing battery arrangement, any regulator, and actual driver VM voltage. |
| UWB module, bearings and collars | No additional software is required for these relative wheel-distance tests. |

Adafruit describes 14 encoder counts per internal motor revolution and a 50:1 gearbox, giving a nominal product of 700 at the output shaft under that counting convention. The Hiwonder counter's decoding must still be measured: **do not assume its reported wheel CPR is 700 or 1400**. The motor shaft diameter and bearing dimensions also do not give the wheel's outside rolling diameter.

**Power check before motor tests:** Hiwonder's current specification table lists **5–15 V at VM**, whereas Adafruit specifies **4.5–6 V nominal for this motor**. A single 3.7 V cell connected directly would be below that published driver supply range; increasing battery voltage without accounting for the motor's rating is not a solution to assume. Have the team confirm the installed driver revision, regulation and actual motor supply. The driver accepting a voltage does not mean the motor can accept the same voltage. Leave battery wiring unchanged until that existing arrangement is established. These ranges come from the manufacturer pages linked above; Hiwonder's page contains differing range information in an image description, so the installed board's documentation takes precedence.

## 1. Connect the laptop to the controller

1. Put the robot on a secure stand with **all driven wheels clear of the table/floor**. Find the robot's motor-power switch/disconnect. The old firmware can move immediately on reset or USB connection, so isolate motor power **before plugging in**.
2. Find the **ESP32 development board's USB programming connector**. Plug that into your Mac with a USB **data** cable. Use the controller's USB port, rather than a battery connector, motor connector, or LED connector. The connector may be USB-C or micro-USB; the repository does not identify the actual hardware connector.
3. Keep the existing, verified robot wiring. The expected signal connections are ESP32 GPIO21 → driver SDA, GPIO22 → driver SCL, and a compatible common ground. GPIO numbers are not physical connector positions. Confirm the actual board/driver labels with the team; power wiring and I²C signal voltage/pull-ups cannot be established from this repository. Do not infer power-wire roles from color.
4. USB normally provides programming/serial communication and may power the ESP32. It does **not** establish the correct motor supply. Use the robot's existing approved motor-power arrangement and voltage. A teammate familiar with the robot should confirm any USB/battery simultaneous-power arrangement.

The chain is: **Mac —USB→ ESP32 —I²C→ motor driver —motor wires→ wheels**. The laptop does not directly power/control the motors through USB.

## 2. Build and upload this project

In VS Code with the PlatformIO IDE extension installed:

1. Choose **File → Open Folder** and open `/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today`.
2. Check that `platformio.ini` is the one in this folder. Open `src/main.cpp` to see the new program.
3. Use PlatformIO **Build** (checkmark). This compiles without contacting the robot.
4. With motor power isolated and USB connected, use **Upload** (right arrow, or the `esp32dev` environment's Upload task). This replaces the ESP32's installed program. The old source remains available, but uploading the old project again would restore its automatic movement behavior.
5. Use PlatformIO **Monitor** to open the serial terminal at **115200 baud**. Press the ESP32's EN/RESET button once if the boot text was missed. The new firmware does not automatically move after reset.

The expected message is `Driver acknowledged. DISARMED. No physical motion verified.` If the driver is unpowered during flashing, a driver fault is expected; after restoring its verified power, type `init` and press Enter. `init` configures the driver and sends zero commands; it does not move the robot.

You can use the PlatformIO terminal instead. PlatformIO is already installed on this Mac at the path shown here, so these commands do not require `pio` to be on your shell PATH:

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
/Users/calvinfisher/.platformio/penv/bin/pio run
/Users/calvinfisher/.platformio/penv/bin/pio device list
```

Identify the USB serial port that appears when this board is connected. It might look like `/dev/cu.SLAB_USBtoUART`, `/dev/cu.usbserial-...`, or `/dev/cu.usbmodem...`. In the next commands **replace `/dev/cu.YOUR_PORT` with the port you actually found**:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio run --target upload --upload-port /dev/cu.YOUR_PORT
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --port /dev/cu.YOUR_PORT --baud 115200
```

Type one command per line and press **Enter**. This project's monitor uses `send_on_enter` and local echo. On another terminal, enable sending a newline (LF or CRLF). For the stop shortcut, type `!` and Enter; this monitor buffers input until Enter. The firmware stops as soon as it actually receives `!`. `Ctrl+C` exits the PlatformIO monitor; exiting the monitor is not a stop command.

Close the monitor before another upload. On a different computer, install PlatformIO IDE and use its terminal/UI; the Mac-specific executable path above will differ.

## 3. Optional discovery: identify and check one wheel at a time

The uncalibrated `drive` / `motors` mode above can run before any of these measurements. Use this section to turn your observations into a known channel map, and to prepare for calibrated distance moves later.

Keep the wheels raised. Restore the robot's verified motor/driver power. Use:

```text
init
status
enc
arm
pulse 1 20 300
```

`pulse 1 20 300` requests motor channel 1 at 20% PWM for 300 milliseconds. This tests raw motor actuation without relying on the driver's internal encoder speed regulation. It automatically sends zero commands and disarms afterward. If it does not turn, do not assume the code is wrong: check driver model, supply, connector mapping, and motor wiring. Its maximum permitted duty is 25%; the program deliberately does not offer full-power tests. `arm` is a deliberate enable command and never starts motion by itself.

Record **which physical wheel moved**. Repeat separately for channels 2 and 3, rearming for each test:

```text
arm
pulse 2 20 300
arm
pulse 3 20 300
```

Observe each test finish before entering another. Commands received during motion are rejected; the program does not provide a motion queue.

Now check encoder direction: run `enc` before and after a positive pulse. The **difference** for that channel should be nonzero. Record whether it increased (`encoderSign = 1`) or decreased (`encoderSign = -1`). A negative pulse should produce the opposite count change. An encoder that stays constant needs a wiring/driver check before distance moves.

Finally, test the driver's closed-loop speed mode on each channel:

```text
arm
jog 1 1 300
enc
```

`jog` uses **encoder pulses per 10 ms**, not percentage power. Start with 1, then 2 if needed; the limit is 3. Check that it rotates smoothly in the same direction as a positive pulse and stops at the deadline. Repeat for motors 2 and 3. The default motor type is Hiwonder's published N20 value `2`. The driver's encoder polarity is initially `1`, inherited from the old source. **If jog races, oscillates, or disagrees with pulse direction, cut motor power and verify the motor type/encoder wiring and driver polarity with the team.** These settings are in `include/robot_config.h`; change, rebuild, and upload as needed.

The driver's `encoderPolarity` setting controls its internal speed feedback. The per-wheel `encoderSign` below only interprets the counts for this program's distance loop. They serve different purposes: setting `encoderSign` cannot repair unstable internal driver feedback.

`stop` or `!` sends zeros and disarms. A physical motor-power disconnect is the fallback if the controller or communication fails. USB unplugging, a monitor exit, or ESP32 reset does not guarantee that a separately powered motor driver stops.

## 4. Match the wheel layout

Choose and label a physical **front** of the robot. The supplied layout is this **assumption**, not a measurement of your robot:

```text
                   FORWARD (+X)
                        ^
                     M1 front
                        /\
                       /  \
          LEFT (+Y) <- /    \
                 M2 rear   M3 rear
                    left   right
```

The three wheels are assumed to sit 120° apart with their driven rolling directions tangent to the triangle's center:

| Physical position | Positive physical rolling direction | Angle from forward |
| --- | --- | --- |
| Front | Left | 90° |
| Rear left | Backward and right | 210° |
| Rear right | Forward and right | 330° |

The angle describes the direction the **whole chassis would be propelled along that wheel's driven axis**, not the motor shaft direction, the wheel's radial position, or a passive roller's axis. Viewed from above, a driven wheel's **top rim** moves along that positive propulsion direction; its bottom contact surface moves oppositely relative to the chassis. For the table's convention, positive directions run counterclockwise around the robot center.

If the channel positions match and positive driver commands propel each wheel along the listed direction, `motorSign = 1`. If a positive command propels it oppositely, use `motorSign = -1`. Determine this from the bench tests. Use the independently observed encoder count sign as `encoderSign`.

Enter changes while disarmed, for example:

```text
stop
wheel 1 90 1 1
wheel 2 210 -1 -1
wheel 3 330 1 1
status
```

This example says M2 is physically rear left, a **negative** driver command produces its positive physical rolling direction, and a **positive** driver command makes its counter decrease. Those signs are examples to replace with observations. If M1 is physically rear left, give M1 angle 210; assign the other angles to their actual channels. If no wheel is at your chosen front, choose a convenient chassis front at one wheel for this first test, or measure the actual rolling angles.

There is no reason to reuse the old mix's signs without checking them. Establish all three channels, physical directions, count signs, and stable speed-mode behavior before relying on **calibrated centimeter moves**. Timed PWM trials can be used to discover those directions without first knowing them.

## 5. Calibrate centimeters without recompiling

For calibrated `move` only, you need the **wheel diameter** and the **driver-reported counts per complete wheel revolution**. Timed `drive` / `motors` skip this entire section. Source comments mention 700 and 1400 as alternatives; neither is verified for this driver, so the new code does not guess. `move` is intentionally rejected until scale is entered.

1. Measure wheel diameter in cm (outside rolling diameter, not radius or motor size).
2. Mark the wheel at a repeatable reference position. Run `enc` and record the counts.
3. With outputs stopped and wheels raised, rotate the **main wheel** a known number of complete turns if the mechanism turns gently. Do not force a gearbox. Keep the electronics powered as required to count encoder transitions. If it cannot be turned gently, use short controlled pulses and the marker to measure completed revolutions, or obtain the tested count scale from the team.
4. Run `enc` again. `CPR = absolute count difference / number of full wheel turns`. Five or ten turns improves the estimate. Measure each channel if the motors/encoders differ. Passive roller rotation is not a wheel revolution.
5. Enter the measured values. **Only if** your diameter is 6.0 cm and measurement gives 700 counts per turn on all three wheels:

```text
cal 0 6.0 700
status
```

`0` means apply to all three. For different wheels, use `cal 1 DIAM CPR`, `cal 2 DIAM CPR`, and `cal 3 DIAM CPR` separately. These commands only store configuration and do not move anything.

Calibration and `wheel` changes live in **RAM** and are lost after reset/reconnection that resets the ESP32. Re-enter them after startup, or edit the defaults in `include/robot_config.h` with the proven values, rebuild, and upload. Use `status` every time before testing. `init` retries the driver and clears a fault after a fix; it preserves RAM calibration. `stop` preserves calibration too.

## 6. Calibrated floor moves (later)

Clear a small level test area, use low-load conditions, keep access to motor power, and begin at 5 cm. After the bench checks and measured calibration:

```text
arm
move 5 0
```

Wait for `DONE: encoder targets reached; disarmed.` Confirm it moved approximately forward and retained heading. Then test:

```text
arm
move 0 5
```

This requests left. Then test backward/right with negative arguments and a small diagonal. Each completed move disarms, so **re-enter `arm` before each move**. Type `stop` or `!` and Enter at any time to stop/disarm. If a fault appears, investigate its cause and type `init` after the fix, followed by `arm` when ready.

Measure actual floor displacement with a ruler. Encoder counts measure wheel rotation; slip, incorrect signs, inaccurate diameter, coasting, and heading drift affect chassis travel. Fix geometry/sign errors before adjusting scale. If every axis is uniformly off by a factor, remeasure circumference/CPR; do not hide a sign/mapping problem with calibration.

**This is a first relative wheel-distance controller.** It does not measure room position or heading, avoid obstacles, or correct wheel slip. The intended motion has zero rotation, but no heading sensor holds that angle. It closes a distance loop per wheel with a proportional speed request and common speed limiting. Integer driver commands can cause small path/heading errors near the end. Targets within about 0.1 cm wheel travel (or 3 encoder counts, whichever is larger) count as reached. A small target may therefore produce no movement.

Calibrated moves time out after 30 seconds; a wheel without measured progress for about 1.2 seconds faults. Timed PWM trials do not read encoders, so they cannot detect a stalled wheel; `drive` / `motors` are limited to 2 seconds and single-wheel `pulse` / `jog` diagnostics to 5 seconds. The longer single-wheel tests refresh their fixed commands every 100 ms, fault on a failed refresh write, and report remaining time about once per second. Stop commands are retried while idle if I²C recovers. These software checks depend on a running ESP32 and working driver. There is **no serial disconnect heartbeat**: motion may continue after the USB cable is removed until its completion/fault/deadline. Driver watchdog behavior has not been established. Timed commands stop only if the controller can deliver their ending zero command.

## How two displacement arguments become three wheel targets

For ideal omniwheels, project the desired chassis displacement onto each wheel's driven direction. If wheel `i` has positive rolling angle `a_i`:

```text
wheel travel_i = cos(a_i) * forward_cm + sin(a_i) * left_cm
wheel revolutions_i = wheel travel_i / (pi * wheel diameter_i)
raw encoder target delta_i = wheel revolutions_i * counts_per_rev_i
                             * motorSign_i * encoderSign_i
```

For the assumed layout, before applying motor/encoder signs:

```text
M1 wheel travel =              left_cm
M2 wheel travel = -0.8660254 * forward_cm - 0.5 * left_cm
M3 wheel travel =  0.8660254 * forward_cm - 0.5 * left_cm
```

For a 10 cm forward move with 6 cm wheels, 700 CPR, and all signs +1, the raw targets are approximately **0, -321.6, +321.6 counts**. A left move drives all three wheels at different ratios: **1, -0.5, -0.5**. Omniwheel rollers allow the un-driven sideways component of travel.

Every 20 ms, the program reads all three counters, computes remaining signed count errors, requests speeds proportional to those errors, and scales requests together to a maximum of 3 pulses/10 ms. The Hiwonder driver handles its own inner speed regulation. It sends zero when the distance targets are reached. Geometry is in `include/kiwi_math.h`; the hardware, commands, and distance state machine are in `src/main.cpp`.

For the first-day mode, C++ can call **`driveRelative(forwardWeight, leftWeight)`**, with optional duration and duty arguments. This calculates the same projection, scales dimensionless contributions to PWM duty, and stops on elapsed time. It never converts to revolutions or encoder counts. It starts with provisional geometry, so physical direction is discovered by observation.

For calibrated distance, C++ can call **`moveRelative(forwardCm, leftCm)`** after calibration/arming. Both functions start a nonblocking motion and return `true` if accepted. Keep calling the existing `loop()`/`tickMotion()` so they can update and stop. Do not repeatedly call them every loop or put a blocking `delay()` after them. The serial interface is the easiest way to start today.

## Where the numbers come from / how to read the code

Start with the opening explanation in `src/main.cpp`, then read `setup()`, `loop()`, `driveRelative()` and `startTimedPwm()`. Each function has a purpose comment. State variables explain what they remember and why they start at zero/false. `include/robot_config.h` gathers the editable settings with source and rationale comments.

| Values | Origin and meaning |
| --- | --- |
| Address `0x34`, GPIO21/22, serial 115200, I²C 100000 Hz | Existing `SWRM_Leds-master/src/main.cpp` baseline. |
| Registers 20/21/51/60 | Existing source and Hiwonder protocol. They select motor type, encoder polarity, speed and counts. |
| Register 31, N20 type `2` | Hiwonder documentation. PWM is percentage duty; speed register 51 uses pulses/10 ms. The original type `1` comment was inconsistent with that documentation. |
| Angles 90°, 210°, 330°; signs initially +1 | An explicitly assumed ideal kiwi arrangement, not measurements of your channel order. |
| 0.8660254 and 0.5 in the mix | `sqrt(3)/2` and `1/2`, from sin/cos for the ideal 120° arrangement. `kiwi_math.h` explains the projection equation. |
| 20% default PWM, 25% test cap, 500 ms default, 2 s timed cap | Chosen initial test settings, not motor specifications or guaranteed speeds. |
| 3 pulses/10 ms, gain 2/s, 20 ms feedback interval, 0.1 cm tolerance | Chosen tuning for optional calibrated distance mode. The conversion factor 0.01 seconds comes from the protocol's 10 ms speed unit. |
| Diameter and CPR initially zero | Explicitly unknown; timed PWM works anyway. Only centimeter mode requires filling them in. |

The old `computeSpeeds()` and the new `directionPwm()` serve the same purpose: mix desired chassis movement into three wheel contributions. The new equations are derived for a documented ideal layout; their signs have not been established by physical testing. The old `delay(3000)` movement demo is replaced by a state machine: `loop()` keeps checking elapsed time and reading commands instead of blocking while motors run.

## Troubleshooting

| Symptom | Next check |
| --- | --- |
| No USB device appears | Use a known data cable, another port/adapter; identify the actual USB bridge before installing its manufacturer's driver. |
| Upload cannot connect | Close serial monitors; confirm port and ESP32 board. On boards with manual BOOT, hold BOOT while upload is connecting, release when writing begins; some require an EN/RESET tap while BOOT is held. Follow the actual board's instructions. |
| Garbled terminal | Match 115200 baud and select the correct USB port. |
| `FAULT: no driver/stop acknowledgement` | Confirm driver power, common reference, SDA/SCL, installed protocol and address `0x34`; type `init` after fixing. An I²C acknowledgment alone does not prove the right hardware. |
| `move rejected` | Check `status`: armed, no fault, all three calibration values present, and vector length <=50 cm. |
| `drive rejected` | Type `arm`; check X/Y each -1..1, optional duration 50..2000 ms and optional duty 1..25. No diameter/CPR is needed. |
| Timed command accepted but no wheel turns | Duty may be too small to overcome friction. Try `drive 1 0 500 25`; check driver power/wiring if still stationary. Driver PWM support must match the documented protocol. |
| Pulse works but jog does not | Check installed motor type, encoder connections, driver polarity and protocol revision. Fix the speed feedback before distance tests. |
| No count change/stall fault | Check that the commanded wheel really turns and its encoder reaches the expected driver channel. |
| Encoder direction fault | Recheck measured `encoderSign`; also check feedback stability and whether a physical disturbance drove the wheel backward. |
| Timed pattern goes an unexpected direction or spins | Record it; direction is provisional. Try an exact `motors` pattern or later adjust `wheel` angles/signs. Test on the stand while identifying channels. |
| Stops at 30 seconds before completing | First check load, count scale and feedback. The initial cap is intentionally slow; use shorter moves. Only tune limits after short tests work. |
| Physical robot still moves after stop | Cut motor power. A software stop is not a hardware emergency stop; verify the driver/protocol and communication before proceeding. |

## Checks performed and how to repeat

The ESP32 build succeeded using PlatformIO `espressif32@8.0.0`, the installed Arduino ESP32 framework, and board `esp32dev`. Host tests run the real firmware against simulated serial/I²C and cover **timed driving with unknown dimensions and unavailable encoder data**, exact raw motor outputs, diagonal/reverse patterns, optional timing/duty, calibration gating for centimeters, malformed input, speed/distance limits, automatic deadlines, stop handling, partial I²C reads, bus failure/recovery, stalls, wrong count direction, counter wrap, and clock wrap. These tests do not certify physical motion.

From the workspace root on this Mac:

```sh
clang++ -std=c++17 -Wall -Wextra -Wpedantic -fsanitize=address,undefined -I kiwi-drive-today/test/host -I kiwi-drive-today/include kiwi-drive-today/test/host/tests.cpp -o /private/tmp/swrm-kiwi-tests
/private/tmp/swrm-kiwi-tests
```

## References

- [Hiwonder register/motor configuration documentation](https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/3._Arduino_checked.html): address `0x34`; type register 20; polarity register 21; PWM register 31 with -100..100 percentage duty; speed register 51 in pulses/10 ms; encoder counters beginning at 60. Published N20 type is `2`, whereas the original source labels `1` N20. Verify the installed driver revision.
- [Modern Robotics: omniwheel kinematics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/): projection of chassis motion into driven wheel directions.
- [PlatformIO device list](https://docs.platformio.org/en/latest/core/userguide/device/cmd_list.html), [serial monitor](https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html), and [ESP32 upload documentation](https://docs.platformio.org/en/latest/platforms/espressif32.html).
