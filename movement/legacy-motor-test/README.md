# V5: working speed control and timed kiwi movement

**Current version:** follow [MOVEMENT.md](MOVEMENT.md). The user verified independent M1/M2/M3 motion, reverse M2, faster M2 and timed stops with motor type **1**, encoder polarity **0**, **block** speed writes. V5 makes these the boot defaults and adds `drive X Y [MS [SPEED]]` using that same speed interface. Wheel geometry is still provisional. The older comparison notes below describe the investigation, not the current first-run instructions.

# Manual compatibility test of the original motor settings

**Current version: MOTOR COMPARISON v4.** The next diagnostic is the [refreshed PWM comparison](PWM_COMPARISON.md), which uses register 31 and fresh full-channel snapshots without speed-register writes. V1–V3 results below remain historical/unresolved. This is not yet a verified physical fix.

**Current version: MOTOR COMPARISON v3.** Use [COMPARISONS.md](COMPARISONS.md) for the current software-only test procedure and runtime `profile`, `transport`, `bus`, and `gap` controls. Earlier V1/V2 observations below are historical. Harnesses can stay connected for the V3 matrix. No physical channel-isolation fix is claimed yet.

**Physical result of V2 (not a verified fix):** the user reports all three separate motors moving for channel 2 and channel 3, plus intermittent Wire result=4 during zero-output writes. Independent channel control is NOT established. Do not use this version for floor/directional movement or the `original` combined pattern while diagnosing. Software tests verify packet construction only, not the driver's physical interpretation.

**V2 update:** the user observed extra motor assemblies moving with the four-byte block requests, although all stopped at the deadline. The cause is unproven. V2 tests Hiwonder's documented single-channel writes: **51=M1, 52=M2, 53=M3, 54=M4**, each with one signed byte. Unselected channels are cleared before the selected channel is enabled. Failed clearing prevents enabling.

Upload V2 as below. With **all wheels clear of every surface and the chassis stationary**, run `init`, `arm`, then `motor 2 10 1000`. Only one assembly should turn. After completion, rearm and try `motor 3 10 1000`. Do not try combined/original patterns until physical channel independence is verified.

[Hiwonder single-motor example](https://wiki.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/4_Raspberry%20Pi_formatted.html) uses register `50 + motor_id` and one speed byte.

This separate firmware tests the original `SWRM_Leds-master/src/main.cpp` driver settings and speed interface without its automatic/repeating movement. It does not change the original source or the main `kiwi-drive-today` source. The ESP32 runs only whichever firmware you last uploaded.

## What the original actually does

| Detail | Original source | First-day controls project |
| --- | --- | --- |
| I2C address / GPIOs / bus clock | 0x34 / SDA21 / SCL22 / 100kHz | Same |
| Motor-type byte sent to register 20 | **1**, labeled N20 in its comments | **2**, following current Hiwonder documentation |
| Encoder-polarity byte at register 21 | 1 | 1 |
| Movement interface | Speed register **51** | Timed drive/PWM tests use **31**; jog uses 51 |
| Active demo's wheel commands | Approximately **+58,-58,0,0** | Jog speed limited to +/-3; PWM duty is a different unit |
| Active interval | 3 seconds, then speed zeros for 2 seconds, repeating | Manually started bounded tests |
| Idle stop traffic | One speed-zero packet before each 2-second stop interval | Speed zero plus PWM zero every 250ms while idle |
| Wire transaction timeout | No explicit override; installed library defaults to 50ms | Explicit 20ms |
| Total encoder counters | Reset once at startup, then read and printed | Used only when requested / in calibrated mode |

Current Hiwonder documentation calls type 1 TT and type 2 N20. This test deliberately sends the **original numeric value 1** to compare behavior with the source the team used. It does not establish that type 1 is the proper N20 setting on every firmware revision.

All ten address-only diagnostic probes succeeded on the user's robot. That supports basic contact with address 0x34 during those observations, but it does not validate register writes, motor supply or motion. The PWR LED continued blinking after probes ended; its exact revision-specific meaning remains unverified. Do not treat that blink alone as a diagnosed voltage fault.

## What this firmware preserves and changes

It preserves the original type/polarity bytes, speed interface (with individual channel writes in V2), pins, clock, setting pauses and original three-second output pattern. It uses no PWM register 31 writes and no periodic idle traffic. It restores the installed Wire default timeout of 50ms. This is a compatibility comparison with several differences changed together, not an experiment that isolates one cause at a time.

It adds manual arming, bounded one-shot tests, per-write status messages, input editing and a few bounded zero-command retries if a stop write fails. It does not reset/read total encoder counters and does not use the old wheel mixer for individual motor commands. That makes this a controlled baseline rather than an exact copy of every original startup transaction.

**No motors are automatically commanded at boot.** `init` writes the configuration and zero speeds, `arm` writes zero speeds and enables a future test, and only `motor` or `original` sends nonzero speeds. Wheel speed values are encoder pulses per 10ms, not PWM percentages. The driver may increase power to achieve the request. Keep all wheels raised, and use physical power removal if software stopping fails.

## Upload and open the monitor

Exit the diagnostic monitor with Ctrl+C. With the reported USB-only power arrangement, unplug USB to power-cycle the setup; leave other supplies disconnected. Keep the wheels clear, reconnect USB for programming, then run in **Mac Terminal**:

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
/Users/calvinfisher/.platformio/penv/bin/pio run --project-dir legacy-motor-test --target upload --upload-port /dev/cu.usbserial-0001
```

Wait for success and open the corresponding monitor:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --project-dir legacy-motor-test --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
```

If startup text was missed, briefly press/release EN/RESET once. This firmware prints `LEGACY SPEED TEST v2`. It has its own command menu; `drive`, `pulse`, and `jog` are not commands in this particular program.

## First check: original configuration writes

In **robot monitor**, type:

```text
init
```

Expected successful write results:

```text
INIT: original values type=1, polarity=1; speed-only writes.
WRITE register=20 bytes=1 result=0
WRITE register=21 bytes=1 result=0
WRITE register=51 bytes=1 result=0
WRITE register=52 bytes=1 result=0
WRITE register=53 bytes=1 result=0
WRITE register=54 bytes=1 result=0
READY: original settings acknowledged; DISARMED. No motion verified.
```

If any result is nonzero, copy those lines and `status`; do not start a test. Unlike an address probe, each line tests a real register/payload transaction.

## First motor command

With wheels raised, enter separately:

```text
arm
```

```text
motor 1 10 1000
```

This requests **M1 at +10 pulses per 10ms for one second**. M2/M3/M4 receive zero. On a successful command, you should see `STARTED`, then a speed-zero write and `DONE; disarmed.` Observe actual wheel motion and stopping.

If this works, try another channel after rearming: `motor 2 10 1000` or `motor 3 10 1000`. A negative command such as `motor 1 -10 1000` reverses the requested direction. Accepted range is +/-1..58, for 50..3000ms. Start at 10; larger numbers can be fast.

Type `!` (and Enter if needed) or `stop` to send zero speeds and disarm. Ctrl+C exits the monitor; it does not stop a motor. Failed stop writes trigger up to four zero-only retries, after which the program remains faulted/quiet. Physical power removal is the fallback.

## Optional original active pattern

Only when ready to deliberately compare the original active demo, with every wheel securely clear and a power disconnect accessible:

```text
arm
```

```text
original
```

This requests **M1=+58, M2=-58, M3=0, M4=0 for 3000ms once**, then speed zeros. It can be fast and is not the first test. It does not automatically repeat or restart after completion/reset.

## USB cable question

The current cable supports data: the robot has accepted firmware uploads and returned readable serial output. A charge-only cable could not provide those operations. No special ESP32-branded cable is required; use a data-capable USB cable that matches the physical board socket. A short known-good cable/direct Mac connection can still help check marginal power/contact issues, but the ten successful address probes do not establish a cable fault.

## Verification

This firmware built for esp32dev with the installed ESP32 Arduino framework. Host tests run its actual source against simulated serial/I2C to check original configuration bytes, individual signed-byte speed packets at registers 51..54, no register-31 writes, no idle polling, no automatic boot/repeat motion, deadline/stop behavior, invalid commands, faults, bounded stop retries and clock wrap. These tests do not verify the physical board, motor supply or installed firmware interpretation.

References: original local main.cpp; [Hiwonder register documentation](https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/3._Arduino_checked.html); installed Arduino ESP32 Wire implementation. The first step after a successful baseline is to change one difference at a time before updating the main movement program.
