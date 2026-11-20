# Robot movement: start here

Use `movement/legacy-motor-test`. The parent `movement/src/main.cpp` is an older, separate program. V16 accepts **square**, **stop** and **spin**, plus deliberate **init** recovery and read-only **status**, over serial or the lead’s MQTT controller. MQTT builds initialize the driver once at battery startup and remain disarmed. Boot, initialization and reconnection never start motion.

## Files and robot selection

| File | Purpose |
| --- | --- |
| `legacy-motor-test/src/main.cpp` | Working motor/I²C layer, diagnostics, guarded starts, nonblocking movement and stop/fault handling; retains main’s HUB75 startup |
| `legacy-motor-test/src/mqtt_controller.cpp` | Lead’s WiFiClient/PubSubClient connection, exact callback, LED commands and disconnect handling |
| `legacy-motor-test/include/network_config.h` | Wi-Fi, broker and connection timing; owned by the networking lead |
| `legacy-motor-test/include/robot_commands.h` | Networking-independent command API, status snapshot and callback inbox |
| `legacy-motor-test/include/mqtt_controller.h` | Loop-to-network result/status reporting; copied snapshots keep motor state on the loop task |
| `legacy-motor-test/include/robot_settings.h`, `include/profiles/` | Select A/B, shared square settings and later individual overrides |
| `legacy-motor-test/platformio.ini` | Build/upload environments and existing LED-panel dependencies |
| `legacy-motor-test/lib/PubSubClient/` | MIT-licensed v2.8 client with one retained-flag accessor; see its `SWRM_PATCH.md` |
| `test/run_active_tests.sh` | Reproducible movement and real-MQTT-packet host tests |

| Robot | MQTT build environment | Client ID | Command / status topics |
| --- | --- | --- | --- |
| A | `esp32dev_mqtt` | `robot1` | `robots/robot1/cmd`, `robots/robot1/status` |
| B | `esp32dev_b_mqtt` | `robot2` | `robots/robot2/cmd`, `robots/robot2/status` |

**The default build/upload target is `esp32dev_mqtt` (A with Wi-Fi/MQTT).** Select `esp32dev_b_mqtt` explicitly for B. `esp32dev` and `esp32dev_b` run the same movement code without networking and retain manual serial `init`. The MQTT targets initialize once at boot for battery-only use. B builds set `SWRM_ROBOT_ID=2`; A defaults to 1. Boot/status print the identity. Later per-robot changes belong in `profiles/robot_a.h` or `robot_b.h`; each has its own profile value and identity.

Main’s HUB75 pins, FM6124 configuration, brightness 100 and white startup display are retained. `SWRM_ENABLE_LED_PANEL=1` in the base build flags enables it for all environments. Set that flag to 0 for the previous motor-only lab configuration. GPIO2 MQTT `on`/`off` controls the onboard LED, not the panel. The combined panel/Wi-Fi/movement build has not been physically validated.

## Build, upload and monitor

Run from the repository root with PlatformIO installed. On Calvin’s Mac, replace `pio` with `/Users/calvinfisher/.platformio/penv/bin/pio` if needed. Before MQTT use, review `include/network_config.h`: it retains the lead’s `TAMU_IoT` open Wi-Fi and broker `10.251.20.11:1883`; update the laptop address when it changes.

```sh
# A; change the environment to esp32dev_b_mqtt for B.
pio run -d movement/legacy-motor-test -e esp32dev_mqtt
pio run -d movement/legacy-motor-test -e esp32dev_mqtt --target upload --upload-port /dev/cu.usbserial-0001
pio device monitor -d movement/legacy-motor-test --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
```

Close the monitor before uploading. Confirm `KIWI MONOREPO LAB v16` and the intended `ROBOT=A` or `ROBOT=B`. Uploading replaces the installed firmware. The checks for this change did not upload firmware or operate motors.

## Battery-only startup and movement

For A use `esp32dev_mqtt`; for B use `esp32dev_b_mqtt`. Normal battery-only use no longer requires USB initialization:

1. Upload the correct target through USB using the established lab upload/power arrangement.
2. Close the serial monitor and disconnect USB. This procedure does not require simultaneous USB and battery supplies.
3. Place the robot with wheels raised for the first test and power it from the team’s approved battery circuit. That circuit must supply **both the ESP32 and motor driver**.
4. MQTT builds make **one** guarded initialization attempt at startup, before networking starts: acknowledged preflight zero, driver configuration, then final zero. Successful initialization leaves the robot disarmed. Wi-Fi/MQTT connect automatically; publish `on`/`off` first to verify the connection, then `status` and `stop`. A fresh state snapshot also follows each connection.
5. Send an explicit `square` or `spin` only when ready. Stop and verify the raised-wheel test before moving to the floor.

With serial available, a successful MQTT boot prints `BOOT READY: initialized, DISARMED`; `status` should show `configured=1 armed=0 running=0 fault=0`, legacy/lab, type 1/polarity 0, speed/block and the compiled profile below. `preview-square 1500 20 1000` remains available while disarmed. Serial-only builds require local `init` first; an explicit serial `init` also remains available for diagnostics/recovery on MQTT builds.

A failed boot initialization prints `BOOT FAULT` and latches the fault. Only the existing bounded zero-output retries may run; driver configuration and motion are not automatically retried. Wi-Fi reconnection does not initialize, clear a fault or resume movement. Correct the driver/power issue, then deliberately request `init` through MQTT or serial. If the driver powers up too late for the one startup attempt, the robot remains faulted until that explicit recovery; it does not silently retry configuration.

| Command | Behavior |
| --- | --- |
| `status` | Report readiness/state and last/first driver error without reading or writing motor hardware; works while moving or faulted |
| `init` | Explicit recovery: requires idle/disarmed, no pending stop retries, and an acknowledged zero before configuration; success stays disarmed |
| `square` | Explicitly checks readiness, performs the existing acknowledged-zero arm, then repeats the calibrated square until stop/fault |
| `stop` | Cancels movement and queued starts/repetitions, including during pauses; requests zero and disarms |
| `spin` | Same guarded auto-arm, then holds a clockwise mixer request until stop/fault; speed 10 reference pulses/10ms, physically untested |
| `!` | Existing serial stop shortcut |

No separate `arm` is needed for bare square/spin. Starting while moving/paused is rejected; stop first. Unknown commands do not change outputs. A fault remains latched: inspect/save `status`, correct the problem, send `stop`, wait until `retries=0`, explicitly `init`, then inspect `status` before issuing a fresh start. This recovery sequence works over MQTT without USB as long as the ESP32/network are alive. Failed stop writes retain the existing bounded zero-only retries; physical power removal remains the fallback if communication fails. Closing the monitor/Ctrl+C does not send stop.

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

## MQTT connection, MacBook setup and recovery

Based on [the lead’s original mqtt_test source](https://github.com/agfoti04/SWRM_Leds/blob/decd230fd4ac8e2549a3414a74e3bbcf34821334/src/main.cpp). Rechecked November 19: the branch moved this identical example to [sample_code/sameple_MQTT.cpp](https://github.com/agfoti04/SWRM_Leds/blob/4ea371ab0d5c3d460cfb2b1149eb452f77ae2017/sample_code/sameple_MQTT.cpp); its current `src/main.cpp` is an LED demo. The callback remains exactly:

```cpp
void onMessage(char* topic, byte* payload, unsigned int len);
```

It preserves WiFiClient/PubSubClient, server/callback registration, clean-session connect, `robots/<id>/cmd` subscriptions, online/status messages and GPIO2 `on`/`off`. Payload comparisons use `len`, never a presumed null terminator. The callback queues decoded commands; the motor loop handles them before advancing motion. Networking/socket waits run on one separate task. Copied state/fault snapshots cross through bounded FreeRTOS queues; the networking task never reads live motor globals or performs I²C. State/results are best effort; the latest snapshot can replace an older one. A fault snapshot is kept separately so successful recovery cannot erase its diagnostic context before publication.

For the networking lead, the corresponding code is all in `src/mqtt_controller.cpp`:

| Lead's example | Movement firmware |
| --- | --- |
| `connectWiFi()` | Same station-mode Wi-Fi credentials and `WiFi.begin`; one paced attempt per call |
| `connectMQTT()` | Same `mqtt.connect(id)`, subscribe and `online` publication; one paced attempt per call |
| Networking part of `setup()` | `initializeNetworkTransport()` registers the callback/server, sets the LED and prints MAC, identity, broker and topics; called by the networking task |
| `onMessage()` | Preserves LED commands and queues exact-length movement/recovery/status words |
| Networking part of `loop()` | `serviceNetwork()` calls both connection helpers and `mqtt.loop()` on the networking task |

The motor `setup()` in `src/main.cpp` retains I²C/panel setup, performs stationary initialization and calls `beginRobotNetwork()` once. Keep MQTT client calls on the network task. The lead's retrying `while` loops are replaced with paced attempts so repeated connection failures cannot hold up movement stopping. Startup prints the ESP32 station MAC for lab registration if required, plus broker connection errors for USB troubleshooting; battery operation does not wait for a serial monitor.

Your MacBook can publish to the same broker running on the lead’s Windows PC. Confirm its current reachable IP/port, Wi-Fi access and existing no-login connection settings. The ESP32 joins `TAMU_IoT` and connects to the broker; it is not a Wi-Fi hotspot, and you do not need its own IP for normal control. If using your Mac as the broker instead, configure a LAN listener and put the Mac’s lab-network IP in `network_config.h` before rebuilding. `localhost` in ESP32 firmware would refer to the ESP32, not the laptop.

The lead can keep his current publisher and send plain lowercase `status`, `square`, `stop`, `spin` or `init` to A's `robots/robot1/cmd` (B: `robots/robot2/cmd`), without USB connected to either robot. Subscribe to the corresponding `/status` topic for acknowledgements and readiness. Publisher client IDs must differ from the robots' `robot1`/`robot2` IDs; the command-line examples below use automatically generated publisher IDs. Use nonretained commands and confirm `ready=1` before starting.

On the Mac, install the command-line clients if needed (`brew install mosquitto`; [official instructions](https://mosquitto.org/download/)). When using the lead’s broker, there is no need to start another broker locally. Confirm/update `10.251.20.11` in both firmware and the commands below.

After upload, disconnect USB and boot the robot on the approved battery supply, initially with wheels raised. Use two Mac terminals:

```sh
# Terminal 1: keep replies visible. Open before battery boot to catch online.
mosquitto_sub -h 10.251.20.11 -p 1883 -V mqttv311 -t 'robots/+/status' -v
```

```sh
# Terminal 2: define a short publisher for A, once per terminal session.
SWRM_BROKER=10.251.20.11
swrm_a() {
  mosquitto_pub -h "$SWRM_BROKER" -p 1883 -V mqttv311 -q 0 \
    -t robots/robot1/cmd -m "$1"
}
# Send individually; these do not start motion.
swrm_a on
swrm_a off
swrm_a status
swrm_a stop
```

Expect `ack:on`/`ack:off`, then a `state:ready` snapshot with `ready=1 configured=1 armed=0 running=0 fault=0`. `online` means the broker connection succeeded; it does not by itself prove motor readiness. Example snapshot (sequence/cycle counts vary):

```text
state:ready ready=1 configured=1 armed=0 running=0 fault=0 retries=0 err=0 reg=0 firstErr=0 firstReg=0 leg=0 square=0 pause=0 spin=0 cycles=0 seq=2
```

When ready, send `swrm_a square`; expect `ack:square:QUEUED` then `result:square:STARTED`. Stop with `swrm_a stop`. Repeat the stop test during both a moving side and a corner pause. Start/stop `spin` briefly with wheels raised before floor testing. To address B, change the topic to `robots/robot2/cmd` and use the B firmware environment. Unknown/extra/binary/NUL-suffixed/whitespace-suffixed words are rejected. Accepted MQTT words are `on`, `off`, `square`, `stop`, `spin`, `init`, `status`. `!` remains serial-only.

### Recover a fault over Wi-Fi

1. Send `swrm_a status` and save the fault fields. Autonomous faults also publish `event:fault ...`; `err`/`reg` identify the latest driver error, while `firstErr`/`firstReg`/`leg` retain the first failed write’s context until explicit recovery. `seq` is a per-boot snapshot sequence, not a timestamp.
2. Correct the physical cause if needed. Send `swrm_a stop`; check status and wait for `retries=0` before requesting initialization.
3. Send `swrm_a init`. It is rejected while armed, moving, spinning or in an active square/pause. An acknowledged preflight zero is required before configuration. Failure to acknowledge zero prevents configuration and leaves the fault latched. A stop overrides queued init; starts received before/during recovery are cancelled/rejected.
4. Successful recovery reports `result:init:READY; disarmed`. Check `swrm_a status` for `ready=1`, `armed=0`, `running=0`, `fault=0`. It must remain stationary until a **fresh** movement command.
5. If it still faults, retain the diagnostic output and inspect power/wiring/driver state. Reinitialization cannot fix a dead battery, disconnected wiring or unreachable ESP32; some bus/power faults need physical inspection or a battery power cycle. No USB is needed for normal MQTT recovery, but serial remains useful for deeper diagnostics.

The same guarded init applies to explicit serial/C++ recovery. It preserves runtime trims and square statistics; reboot loads the compiled profile again. Normal successful stop does not require init before another start. Wi-Fi/detected MQTT loss queues a stop; the network task discards a stale TCP session if Wi-Fi returns before the socket notices the loss. Reconnection only reports online/requests a fresh state snapshot and never clears faults or resumes movement. Recovery never automatically retries configuration or starts a prior plan.

PubSubClient invokes the callback once per complete packet; its 256-byte packet limit drops oversized messages before callback. The local v2.8 header adds only `messageRetained()`, read before publishing reuses its buffer; `.cpp` is unchanged upstream. Retained square/spin and retained init are rejected. Retained stop/status are safe to accept. Publish operational commands nonretained with QoS0 (the helper omits `-r`). No alternative MQTT stack or remote ESP32-reboot command was added.

## Diagnostics, checks and remaining validation

`help`, `status`, `enc`, `voltage`, previews and `square-stats` remain available. Diagnostics/settings edits require idle/disarmed state. After separate `arm`, `square 1500 20 1000` runs one square; `square-repeat 3 1500 20 1000` runs exactly three; `square-test right 1500 20` tests one leg. Timed `drive X Y [MS [SPEED]]` and `twist X Y W [MS [SPEED]]` remain (+X right/+Y forward/+W clockwise, weights not distance). Encoder reads remain diagnostics; experimental encoder-distance `move`/`cal` control is outside this focused change.

Run `sh movement/test/run_active_tests.sh`. The suites compile the actual firmware and actual PubSubClient with simulated serial/I²C/TCP; they check one-time stationary battery startup, zero-first remote recovery, busy/retained-init guards, independent fault/status snapshots, command/fault guards, calibrated packets, repeat/pause cancellation, retained/binary/oversized payloads, LED on/off, disconnect/reconnect and serial stop during socket waits. Builds/tests establish software behavior, not physical robot performance.

Before a joint demo: briefly check B’s identity/wheels, one supervised square and stop during a side/pause; full recalibration is not a prerequisite. Physically verify spin, batteries/changed mounting/floor, accumulating drift, intermittent inconsistent encoder reads, active-repeat stops and actual Wi-Fi/broker loss/reconnect. Broker-loss detection is delayed by keepalive/socket behavior (configured 5 s/1 s), not instantaneous. Publisher loss while robot-to-broker stays connected still needs a controller heartbeat/lease. The combined panel/Wi-Fi/movement behavior has not yet been tested on hardware.
