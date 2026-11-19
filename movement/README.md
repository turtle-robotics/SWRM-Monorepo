# Robot movement: start here

Use `movement/legacy-motor-test`. The parent `movement/src/main.cpp` is an older, separate program. This firmware accepts **square**, **stop** and **spin** over serial or the lead’s MQTT controller. Boot, initialization and reconnection never start motion.

## Files and robot selection

| File | Purpose |
| --- | --- |
| `legacy-motor-test/src/main.cpp` | Working motor/I²C layer, diagnostics, guarded starts, nonblocking movement and stop/fault handling; retains main’s HUB75 startup |
| `legacy-motor-test/src/mqtt_controller.cpp` | Lead’s WiFiClient/PubSubClient connection, exact callback, LED commands and disconnect handling |
| `legacy-motor-test/include/network_config.h` | Wi-Fi, broker and connection timing; owned by the networking lead |
| `legacy-motor-test/include/robot_commands.h` | Networking-independent command API and callback inbox |
| `legacy-motor-test/include/robot_settings.h`, `include/profiles/` | Select A/B, shared square settings and later individual overrides |
| `legacy-motor-test/platformio.ini` | Build/upload environments and existing LED-panel dependencies |
| `legacy-motor-test/lib/PubSubClient/` | MIT-licensed v2.8 client with one retained-flag accessor; see its `SWRM_PATCH.md` |
| `test/run_active_tests.sh` | Reproducible movement and real-MQTT-packet host tests |

| Robot | MQTT build environment | Client ID | Command / status topics |
| --- | --- | --- | --- |
| A | `esp32dev_mqtt` | `robot1` | `robots/robot1/cmd`, `robots/robot1/status` |
| B | `esp32dev_b_mqtt` | `robot2` | `robots/robot2/cmd`, `robots/robot2/status` |

`esp32dev` (default) and `esp32dev_b` run the same movement code without networking. B builds set `SWRM_ROBOT_ID=2`; A defaults to 1. Boot/status print the identity. Later per-robot changes belong in `profiles/robot_a.h` or `robot_b.h`; each has its own profile value and identity.

Main’s HUB75 pins, FM6124 configuration, brightness 100 and white startup display are retained. `SWRM_ENABLE_LED_PANEL=1` in the base build flags enables it for all environments. Set that flag to 0 for the previous motor-only lab configuration. GPIO2 MQTT `on`/`off` controls the onboard LED, not the panel. The combined panel/Wi-Fi/movement build has not been physically validated.

## Build, upload and monitor

Run from the repository root with PlatformIO installed. On Calvin’s Mac, replace `pio` with `/Users/calvinfisher/.platformio/penv/bin/pio` if needed. Before MQTT use, review `include/network_config.h`: it retains the lead’s `TAMU_IoT` open Wi-Fi and broker `10.251.20.11:1883`; update the laptop address when it changes.

```sh
# A; change the environment to esp32dev_b_mqtt for B.
pio run -d movement/legacy-motor-test -e esp32dev_mqtt
pio run -d movement/legacy-motor-test -e esp32dev_mqtt --target upload --upload-port /dev/cu.usbserial-0001
pio device monitor -d movement/legacy-motor-test --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
```

Close the monitor before uploading. Confirm `KIWI MONOREPO LAB v15` and the intended `ROBOT=A` or `ROBOT=B`. Uploading replaces the installed firmware. The checks for this change did not upload firmware or operate motors.

## Initialize, square, stop, spin

In the serial monitor, enter one line at a time:

```text
init
status
preview-square 1500 20 1000
```

Expect `configured=1 armed=0 running=0 fault=0`, legacy mixer/lab orientation, type 1/polarity 0, speed/block transport, unity global trims and the profile below. `init` configures the driver and requests zero without moving. No manual direction settings are needed after reboot.

| Command | Behavior |
| --- | --- |
| `square` | Explicitly checks readiness, performs the existing acknowledged-zero arm, then repeats the calibrated square until stop/fault |
| `stop` | Cancels movement and queued starts/repetitions, including during pauses; requests zero and disarms |
| `spin` | Same guarded auto-arm, then holds a clockwise mixer request until stop/fault; speed 10 reference pulses/10ms, physically untested |
| `!` | Existing serial stop shortcut |

No separate `arm` is needed for bare square/spin. Starting while moving/paused is rejected; stop first. Unknown commands do not change outputs. A fault remains latched: inspect `status`, correct the problem, explicitly `init`, then issue a fresh start. Failed stop writes retain the existing bounded zero-only retries; physical power removal remains the fallback if communication fails. Closing the monitor/Ctrl+C does not send stop.

`spinSpeed` is configurable per robot from 1..20; initial 10 gives driver requests -10,+10,-10. No measured angular rate is claimed. Speed register requests hold until changed; spin does not periodically poll the motor hardware for faults.

## Shared square profile

A’s accepted November 17 USB-only settings are compiled into both robots: **1500ms sides, reference 20, 1000ms stopped corners**, legacy/lab, global trims 1/1/1. M1 rear-left, M2 front, M3 rear-right; M2 inversion is applied exactly once at I²C. B inherits these settings without physical validation.

| Body direction | Time scale | M1 / M2 / M3 trims | Driver request at reference 20 |
| --- | --- | --- | --- |
| Forward | 1.00 | 1.04 / 1.00 / 1.10 | -18, 0, 19 |
| Right | 1.00 | 0.90 / 0.95 / 0.90 | 9, 19, 9 |
| Backward | 1.00 | 1.00 / 1.00 / 1.00 | 17, 0, -17 |
| Left | 1.00 | 0.90 / 0.95 / 1.00 | -9, -19, -10 |

Sequence: forward → right → backward → left, without deliberately rotating at corners. B starts at the opposite corner facing 180° opposite A and uses the same body-relative sequence. Each repeating cycle is nominally 10 s including its final pause; I/O/logging add overhead. Five A cycles completed with no write faults, including three consecutive cycles; about 1 cm rightward drift accumulated per cycle. Fixed trims do not guarantee exact closure indefinitely.

Runtime tuning remains volatile: `square-leg` edits survive init but are lost on reboot; `square-leg reset` deliberately selects unity for diagnostics. Reboot or restore the four rows to return to the compiled profile.

## MQTT connection and examples

Based on [the lead’s mqtt_test source](https://github.com/agfoti04/SWRM_Leds/blob/decd230fd4ac8e2549a3414a74e3bbcf34821334/src/main.cpp). The callback is exactly:

```cpp
void onMessage(char* topic, byte* payload, unsigned int len);
```

It preserves WiFiClient/PubSubClient, server/callback registration, clean-session connect, `robots/<id>/cmd` subscription, online/status messages and GPIO2 `on`/`off`. Complete payloads are compared using `len`; no null terminator is assumed. Movement words go to `queueRobotCommand(payload, len)`; the motor loop drains that inbox before advancing motion. Callback code performs no motor I/O. Socket waits run on one networking task so serial stop and movement timing remain serviced. Wi-Fi/detected MQTT loss queues the common stop; reconnect only subscribes/reports online.

After local `init`, publish nonretained QoS0 commands:

```sh
mosquitto_pub -h 10.251.20.11 -t robots/robot1/cmd -m square -q 0
mosquitto_pub -h 10.251.20.11 -t robots/robot1/cmd -m stop -q 0
mosquitto_pub -h 10.251.20.11 -t robots/robot2/cmd -m spin -q 0
mosquitto_pub -h 10.251.20.11 -t robots/robot2/cmd -m stop -q 0
mosquitto_sub -h 10.251.20.11 -t 'robots/+/status'
```

Movement first reports `ack:square:QUEUED`, then e.g. `result:square:STARTED` or the readiness/fault error. Queued is not proof of physical motion; final status is best effort. LED commands retain `ack:on`/`ack:off`. MQTT payloads accept exact lowercase words with no whitespace, JSON, arguments or NUL suffix. Wrong topics/unknown words are rejected. PubSubClient’s 256-byte whole-packet limit drops oversized messages before callback.

The original PubSubClient callback omits RETAIN metadata. The local v2.8 header adds only `messageRetained()`, read before publishing reuses its buffer; `.cpp` is unchanged upstream. Retained square/spin are rejected, including after reconnect; retained stop is accepted. Keep starts nonretained regardless. No alternative networking stack was introduced.

## Diagnostics, checks and remaining validation

`help`, `status`, `enc`, `voltage`, previews and `square-stats` remain available. Diagnostics/settings edits require idle/disarmed state. After separate `arm`, `square 1500 20 1000` runs one square; `square-repeat 3 1500 20 1000` runs exactly three; `square-test right 1500 20` tests one leg. Timed `drive X Y [MS [SPEED]]` and `twist X Y W [MS [SPEED]]` remain (+X right/+Y forward/+W clockwise, weights not distance). Encoder reads remain diagnostics; experimental encoder-distance `move`/`cal` control is outside this focused change.

Run `sh movement/test/run_active_tests.sh`. The suites compile the actual firmware and actual PubSubClient with simulated serial/I²C/TCP; they check command/fault guards, calibrated packets, repeat/pause cancellation, retained/binary/oversized payloads, LED on/off, disconnect/reconnect and serial stop during socket waits. Builds/tests establish software behavior, not physical robot performance.

Before a joint demo: briefly check B’s identity/wheels, one supervised square and stop during a side/pause; full recalibration is not a prerequisite. Physically verify spin, batteries/changed mounting/floor, accumulating drift, intermittent inconsistent encoder reads, active-repeat stops and actual Wi-Fi/broker loss/reconnect. Broker-loss detection is delayed by keepalive/socket behavior (configured 5 s/1 s), not instantaneous. Publisher loss while robot-to-broker stays connected still needs a controller heartbeat/lease. The combined panel/Wi-Fi/movement behavior has not yet been tested on hardware.
