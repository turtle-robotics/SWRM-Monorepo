# V5: proceed from working motor tests to direction tests

This is the same `legacy-motor-test` project that now has working motor control. Do not upload the older top-level `kiwi-drive-today` program yet: its PWM interface and settings differ. V5 keeps the working speed register 51, type 1, polarity 0 and four-byte block format. No distance scale is assumed. No motion starts on boot or init.

## 1. Upload from Mac Terminal

Keep wheels raised. In the robot monitor enter `stop`, wait for acknowledgment, then press Ctrl+C. Ctrl+C closes the monitor; it does not stop a running motor.

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
/Users/calvinfisher/.platformio/penv/bin/pio run --project-dir legacy-motor-test --target upload --upload-port /dev/cu.usbserial-0001
```

After SUCCESS, open the monitor:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --project-dir legacy-motor-test --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
```

The new banner is `KIWI SPEED DRIVE v5`. There is no need to deliberately power-cycle again while communication and stops remain reliable. Settings now default to the working values after every ESP32 reboot. Geometry edits remain RAM only.

## 2. Initialize and check

In the robot monitor enter separately:

```text
init
status
```

Expect type=1, polarity=0, speedTransport=block, MODE=SPEED, configured=1, armed=0, fault=0. If a write fails, do not proceed to motion. Acknowledgment means the packet arrived, not that the physical direction is verified.

## 3. First combined pattern with wheels raised

```text
arm
drive 1 0 1000 10
```

This sends M1=0, M2=-9, M3=+9 for one second, then all zeros and disarms. This is the provisional forward pattern; the physical front and channel geometry are not yet verified. Every new movement requires a new `arm`. `!` or `stop` requests early termination. If a stop cannot reach the board, remove motor power.

## 4. Current ground-testing sequence and communication recovery

The user clarified that the initial pattern appears to translate rather than rotate. Do not assume a sign correction is needed. The latest logged forward request is M1=0, M2=-17, M3=+17 at reference speed 20, consistent with the default geometry. Previous sign-comparison suggestions were diagnostic alternatives, not a confirmed fix. Keep a record of `status` and do not change wheel signs just because a low-speed test is hard to see.

### 4A. Recover after failed writes before any movement

The latest log has I2C result 4 on both nonzero and zero packets, followed by exhausted stop retries. This is a communication failure, not an out-of-range drive command. Result 4 is the installed ESP32 Wire implementation's generic/other error; it does not identify a unique electrical cause. A failed nonzero request may have partly reached the board. A failed stop is not proof that the motors stopped. Do not keep issuing arm or movement commands in that state.

1. Remove motor power if motion persists. In the reported USB-only setup, disconnect USB and confirm both ESP32 and driver LEDs go out. If any other supply is attached, USB removal alone does not establish power-off.
2. Close the serial monitor with Ctrl+C. With everything powered off, check externally accessible plugs for looseness; do not rewire powered circuits. Wait five seconds before reconnecting power. No motor-harness unplugging is needed for a full USB-only restart.
3. Reopen the monitor using the command in section 1. No upload is needed.
4. Enter these lines separately, without arming:

```text
voltage
enc
init
status
voltage
enc
```

Continue only if reads succeed, initialization writes all report result=0, and status says configured=1, armed=0, running=0, fault=0, MODE=SPEED, type=1, polarity=0, speedTransport=block. If communication still fails while idle, stop movement attempts: changing duration/speed or wheel geometry cannot make failed I2C transactions reliable. Investigate supply, cable/ground and SDA/SCL connections. An idle ~4.9 V report does not certify supply stability during movement.

### 4B. Latest result: mostly rotation even at speed 30

The user now reports that `drive 1 0 2000 30` mostly rotates rather than translates. Do not define this rotation as forward, increase duration, or assume higher speed will correct it. The driver can execute wheel commands while the mixer uses incorrect mounting signs or channel positions. The result alone does not prove which assumption is wrong; load/friction differences remain possible.

First read `status` and record the geometry. For the default geometry M1=90/+1, M2=210/+1, M3=330/+1, the pattern is M1=0, M2 negative, M3 positive. One useful reversible comparison is changing only M2's mixer sign. Do not change encoder polarity: keep the verified driver value 0.

With clean communication, robot stopped/disarmed, default geometry and clear floor, enter separately:

```text
wheel 2 210 -1
status
arm
drive 1 0 1000 20
```

Expect REQUEST M1=0, M2=+17, M3=+17. Compare translation and rotation with the previous opposite-sign pattern. This is a diagnostic trial, not a confirmed correction. If the current geometry differs from defaults, record it before applying this comparison. Do not blindly overwrite other geometry edits.

If it now mostly translates, keep the temporary sign and record its direction relative to a mark on the chassis. Verify backward and sideways at moderate speed before increasing speed. A good two-wheel result does not validate M1's contribution or all wheel angles.

If it still mostly spins, restore M2 while disarmed using `wheel 2 210 1`, then pause combined floor tests. We need the physical mapping instead of further sign guesses: mark a front and label each wheel M1/M2/M3 using the already verified independent motor tests. Provide a top-view photo or describe each channel's position around that front. Also record each positive motor command's rolling direction, if identifiable. These observations let us assign angles/signs to the physical wheels. Wheel diameter is not needed.

The firmware has no sensor identifying a physical nose. Its X/Y reference comes entirely from stored rolling-axis angles/signs, initially 90/210/330 and all +1. A correct mapping should let X and Y generate approximately perpendicular translations with little rotation. Marking a front does not repair an incorrect mapping.

### 4C. Moderate and faster trials

After clean communication checks, put the robot on a clear floor with enough USB slack and no wires near wheels. Enter each line separately. Start from rest; do not queue or batch motion commands.

```text
enc
arm
drive 1 0 1000 20
```

Wait for DONE/disarming, then:

```text
enc
voltage
```

Proceed to this section only after mostly straight translation in 4B. Reference speed 20 should be more visible than the first speed-10 trial. With default geometry this requests M1=0, M2=-17, M3=+17; with only M2 sign flipped it requests 0,+17,+17. Record encoder differences, chassis travel direction and rotation. If any transaction fails, return to 4A rather than increasing speed.

If controlled and all writes succeed, try:

```text
arm
drive 1 0 1500 30
```

With default geometry that requests 0,-26,+26 for 1.5 seconds; with only M2 sign flipped it requests 0,+26,+26. Speed 30 is within the existing drive limit; combined floor behavior is still a new trial. If a motor stalls/buzzes, stop rather than extending duration. `!` requests an early stop; if communication fails, physical power removal is the fallback.

### 4D. Compare directions, one at a time

After forward translation is controlled, test these at moderate speed. Enter `arm` before EACH drive, wait for DONE, and return the robot to the same initial pose when comparing directions.

| Command | Intended direction after orientation is established |
| --- | --- |
| `drive -1 0 1000 20` | Backward |
| `drive 0 1 1000 20` | Left |
| `drive 0 -1 1000 20` | Right |
| `drive 1 1 1000 20` | Forward-left diagonal |
| `drive 1 -1 1000 20` | Forward-right diagonal |

The sideways/diagonal patterns introduce M1, so they also test geometry not verified by the forward pair. Do not assume all axes are correct from one successful forward trial. If significant rotation occurs, pause and record channel locations and positive rolling directions before changing signs.

### 4E. Notes to record

For each trial record the exact command, `status` geometry, direction relative to the front mark, rough distance, CW/CCW rotation and encoder counts before/after. Record physical M1/M2/M3 wheel positions. Note whether failures happen while idle or only when movement starts. Neither wheel diameter nor a measured distance scale is required for these timed trials.

## 5. Meaning of the arguments

`drive X Y [MS [SPEED]]` accepts X and Y from -1 to +1. Positive X means forward and positive Y means left, relative to the robot when the command begins. Defaults are 1000 milliseconds and speed 20. Explicitly supplying 10 starts slower. Time is limited to 50..3000 milliseconds, speed to 1..30 pulses per 10 ms. These bounds are bench choices, not motor ratings. This is timed travel, not closed-loop heading or distance control.

`drive 1 0` therefore means a forward pattern for one second at reference speed 20; it does NOT mean one centimeter or one meter. For diagonals use `drive 1 1 1000 10`. Large combined wheel requests are scaled together to respect the speed limit. Reversing both X/Y reverses the complete pattern.

## 6. Geometry and why the numbers exist

A wheel contributes motion along its driven rolling direction, perpendicular to its axle. With angle measured counterclockwise from the marked forward direction, the ideal wheel-speed contribution is:

```text
wheelSpeed = referenceSpeed * motorSign * (cos(angle)*X + sin(angle)*Y)
```

Provisional angles 90, 210 and 330 degrees are spaced 120 degrees apart. They give M1=Y, M2=-0.866X-0.5Y, M3=0.866X-0.5Y before scaling/sign correction. This assumes a symmetric kiwi layout and equal wheel travel per encoder count. Equal counts can still give imperfect travel if wheels/gear ratios differ or slip.

While idle and disarmed, `wheel CHANNEL ANGLE_DEG SIGN` changes the assumed rolling direction/sign without changing encoder polarity. For example `wheel 2 210 -1` reverses M2's contribution to the mixer; use it only when the observed mounting/direction supports that correction. SIGN is +1 or -1. Settings are shown by `status` and lost at reboot. No `init` is needed after a geometry-only edit. Leave the verified driver polarity at 0.

The C++ `mixDirection` function performs the projection; `startTest` sends the bounded speed pattern; `updateTest` stops it at the deadline. Motor 4 always receives zero. Driver faults still disarm and attempt zero writes. The program does not queue movement commands: a new drive during motion is rejected.

## Verification

Host checks exercise forward/left/reverse packets, diagonal limits, geometry edits, invalid input, arm requirements, PWM rejection, timed stops and the existing diagnostic/fault tests. The ESP32 firmware builds successfully. The latest ground test mostly rotates even at reference speed 30; physical translation mapping remains unverified. Later logs show communication failures including exhausted stop retries, so section 4 requires recovery before further movement. This guide update changes no firmware or speed limits.

## 7. Quick command reference

| Command | Arguments/limits | Effect |
| --- | --- | --- |
| `init` | None | Configure driver and request zero outputs; stays disarmed |
| `status` | None | Show flags, mode, settings and wheel geometry; no motion |
| `arm` | None | Requires successful init and no fault; sends zeros, enables one subsequent test |
| `drive X Y [MS [SPEED]]` | X/Y -1..1, MS integer 50..3000, SPEED integer 1..30 | Robot-relative timed translation pattern in speed mode |
| `motor CHANNEL SPEED MS` | Channel 1..3, signed integer speed -58..58 excluding zero, MS 50..3000 | One-channel test; use raised wheels and the already tested magnitudes up to 30 for now |
| `enc` / `voltage` | None; idle/disarmed only | Read cumulative counts / reported supply |
| `stop` / `!` | None | Request zeros and disarm; cannot guarantee a stop if communication fails |
| `help` | None | Print available commands |

For `drive`, omitted MS defaults to 1000 and omitted SPEED to 20. To specify SPEED, include MS first. SPEED is a positive reference magnitude in pulses/10ms, not percent duty or torque. Use negative X/Y for reverse directions. X/Y may be fractional, e.g. `drive 0.5 0 1000 20` requests half the reference forward pattern. `(0,0)` requests a stop. Every completed movement disarms. A drive while already moving is rejected. Initialization is needed after reboot, a fault or driver-setting changes; a normal completed test requires only `arm` before the next test. A driver setting edit is distinct from `wheel`, which edits mixer geometry only.
