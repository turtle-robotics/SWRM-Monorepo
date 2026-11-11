# Robot first-run walkthrough — Mac, from connection to trial notes

This guide uses the code already saved on your Mac in `/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today`. You do not need to measure wheel diameter or identify motor placement to try timed patterns. The program sends repeatable motor commands; their physical translation/turning is what you observe. It cannot yet promise an accurate distance or known direction on the real robot.

Follow the steps in order. Commands marked **MAC TERMINAL** run on your laptop. Commands marked **ROBOT MONITOR** go to the ESP32, after Step 6 opens its serial connection. Press **Return/Enter after every command line**. Do not copy the surrounding triple backticks or sample output.

## Jump to a step

- [1. Connect the robot to the Mac](#step-1-connect-the-robot-to-the-mac)
- [2. Open Terminal and enter the correct folder](#step-2-open-terminal-and-enter-the-correct-folder)
- [3. Build the firmware](#step-3-build-the-firmware)
- [4. Find the USB serial port](#step-4-find-the-usb-serial-port)
- [5. Upload the firmware](#step-5-upload-the-firmware)
- [6. Open the robot monitor at 115200 baud](#step-6-open-the-robot-monitor-at-115200-baud)
- [7. Check communication and understand arm](#step-7-check-communication-and-understand-arm)
- [8. Send the first timed pattern](#step-8-send-the-first-timed-pattern)
- [9. Test other patterns and record notes](#step-9-test-other-patterns-and-record-notes)
- [10. Stop, exit, reconnect, and upload changes](#step-10-stop-exit-reconnect-and-upload-changes)
- [11. Fix errors](#step-11-fix-errors)
- [12. Understand what was verified](#step-12-understand-what-was-verified)
- [Command reference](#command-reference)

## Step 1: Connect the robot to the Mac

**Where:** at the robot; no terminal command yet.

1. Identify the **ESP32 development board**. This is the small programmable board with an ESP-WROOM-32 module and a USB programming socket. The separate Hiwonder board has motor connectors and receives commands from the ESP32.
2. Support the robot so its driven wheels are clear for the first test. Find its existing motor-power switch/disconnect. Isolate motor power **before connecting USB**: the old installed demo may start movement on reset. The new firmware waits for commands, but it is not installed until Step 5 finishes.
3. Plug a **USB data cable** into the ESP32 board's USB socket and plug its other end into the Mac (through your USB adapter if needed). Use the connector actually on the installed board. The breakout-board listing describes USB-C; the other development board may use another connector. A charge-only cable can light the board without creating a usable programming port.
4. Leave the robot's established electronics and motor wiring intact. You are plugging into the ESP32's USB programming port, not a battery connector, bare GPIO pins, or motor output.

```text
Mac USB port / adapter
          |
          | USB DATA cable
          v
ESP32 development board's USB socket
          |
          | existing I2C wiring (signals, not motor power)
          v
Hiwonder motor driver
          |
          | existing motor connectors
          v
Three motor/wheel channels
```

The code expects ESP32 GPIO21 → driver SDA, GPIO22 → driver SCL, and a compatible common ground. These describe the existing software assumptions; you do not need to rearrange pins to try uploading. If the assembled robot uses other signal pins, the program's settings will need to match them.

**Power:** USB programming power and the motor supply are separate questions. Use the team's established power arrangement. Your list of 3.7 V cells does not reveal the final battery/regulator arrangement. Hiwonder's current specification table lists 5–15 V at VM, while Adafruit specifies nominal 4.5–6 V for this motor. Confirm the installed supply arrangement with the teammate responsible for wiring rather than inferring battery connections from the list. See the [Hiwonder product page](https://www.hiwonder.com/products/4-channel-encoder-motor-driver) and [Adafruit motor page](https://www.adafruit.com/product/4638).

**Checkpoint:** ESP32 connected to the Mac by a data cable; wheels clear; motor power isolated while replacing old firmware. You can compile/upload with the motor driver unpowered. A driver communication fault at that stage is expected.

## Step 2: Open Terminal and enter the correct folder

**Where:** your Mac's **Terminal** app. This path does not require learning VS Code's menus or installing anything new; PlatformIO is already installed on this Mac.

1. Press **Command + Space** to open Spotlight.
2. Type `Terminal`, then press Return.
3. A window opens with a prompt, usually ending in `%`. That is where you type Mac commands. The prompt is not part of the command.
4. Copy this command, paste it into Terminal, and press Return:

**MAC TERMINAL:**

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
```

`cd` means change directory: it selects the project folder. The single quotes are necessary because `SWRM TURTLE` contains a space.

Now run:

```sh
pwd
```

**Expected output:**

```text
/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today
```

Check the available PlatformIO executable:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio --version
```

The checked installation reports `PlatformIO Core, version 6.2.0`. The long executable path avoids the common `command not found: pio` problem. A different computer will have a different installation path.

**Checkpoint:** `pwd` shows `kiwi-drive-today`, and the version command runs. Stay in this same Terminal window for subsequent steps. You can read/edit code in VS Code separately, but you do not need a second terminal.

## Step 3: Build the firmware

**Where:** MAC TERMINAL, in the project folder selected above.

```sh
/Users/calvinfisher/.platformio/penv/bin/pio run
```

This compiles the C++ source into firmware for an ESP32. It does not upload it or move any wheels. Wait until it finishes and the `%` prompt returns. A first build may take longer than later builds.

Look for:

```text
Processing esp32dev ...
...
[SUCCESS]
```

You may also see RAM/Flash usage and a line saying a `firmware.bin` was created. These are normal. If it says `[FAILED]`, go to Step 11; do not treat a failed build as an uploadable program.

**Checkpoint:** build ends with `[SUCCESS]`. The robot does not need to be connected for this step.

## Step 4: Find the USB serial port

**Where:** MAC TERMINAL. ESP32 connected by USB, motor power still isolated.

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device list
```

This lists available serial devices. A USB serial port is a name the Mac uses to reach your board. Example **illustrative** output:

```text
/dev/cu.SLAB_USBtoUART
--------------------
Description: CP2102 USB to UART Bridge Controller
```

Your board may instead appear as `/dev/cu.usbserial-...` or `/dev/cu.usbmodem...`. The exact suffix can differ between boards/USB ports. Do not choose `/dev/cu.Bluetooth-Incoming-Port` for this USB connection.

If several entries appear, compare the list with the ESP32 unplugged and plugged in, keeping motor power isolated throughout. The entry that appears with the ESP32 is the one to use. If none appears, try a known data cable and another USB adapter/port before considering a driver issue.

**Write down the exact `/dev/cu....` line.** In the next two steps, `/dev/cu.YOUR_PORT` is a placeholder: replace that whole text with your actual port. Do not type the placeholder literally.

**Checkpoint:** one specific port identified. For official examples see [PlatformIO device list](https://docs.platformio.org/en/latest/core/userguide/device/cmd_list.html).

## Step 5: Upload the firmware

**Where:** MAC TERMINAL. Motor power isolated, ESP32 connected by USB. Close any serial monitor that already has this board's port open.

Replace `/dev/cu.YOUR_PORT` with the exact port from Step 4, then run:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio run --target upload --upload-port /dev/cu.usbserial-0001
```

For example, **only if** your actual port was `/dev/cu.SLAB_USBtoUART`:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio run --target upload --upload-port /dev/cu.SLAB_USBtoUART
```

Run the command with your actual port once; you do not need to run both versions. Uploading replaces the program installed in the ESP32. Your original source folder on the Mac remains unchanged.

**Expected sequence:** build checks, connection to ESP32, flash-writing progress, then `[SUCCESS]`. Wording may include `Writing at ...`, verified data, and a reset message. Wait for the Mac prompt to return.

If it stays at `Connecting...` or says `Failed to connect`, try:

1. Close other serial monitors and check the port name.
2. If your board has **BOOT** and **EN/RESET** buttons: hold BOOT, briefly press/release EN/RESET, and keep BOOT held as upload connects. Release BOOT when flash writing begins. Follow your board's labels/instructions. Do not bridge random pins.
3. Retry the upload. If upload finishes but the program remains in download mode, release BOOT and tap EN/RESET once.

See [Espressif upload troubleshooting](https://docs.espressif.com/projects/esptool/en/latest/esp32/troubleshooting.html) and [boot-mode selection](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html) for the manufacturer's explanation.

**Checkpoint:** upload ends `[SUCCESS]`. This is separate from build success: build creates the program; upload copies it onto the ESP32.

## Step 6: Open the robot monitor at 115200 baud

**Where:** begin in MAC TERMINAL. This command changes that terminal into the ROBOT MONITOR.

Again replace `/dev/cu.YOUR_PORT` with your actual port:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter send_on_enter --filter log2file --echo
```

**What that command means:**

- `device monitor`: open the text connection to the ESP32.
- `--port ...`: which board to connect to, from Step 4.
- `--baud 115200`: use the same communication rate as `Serial.begin(115200)` in the firmware. You do not need to change a separate Mac setting. Baud is the link rate, unrelated to wheel speed.
- `--exit-char 3`: explicitly choose **Ctrl+C** to exit the monitor.
- `--eol LF`: send a newline when you press Enter; the firmware uses that to finish a command.
- `--filter send_on_enter`: collect your typed command and send the whole line when you press Enter.
- `--filter log2file`: save received robot messages in a log file in this project's `logs` folder. The monitor prints its exact file path.
- `--echo`: display what you type so you can see it in the terminal.

When this opens, you will see a monitor header instead of a normal `%` prompt. **From now until you press Ctrl+C, type robot commands such as `help`; do not type `cd`, `pio run`, or other Mac commands.**

Opening a port may reset some ESP32 development boards. If no boot text appears, with wheels still supported, tap **EN/RESET** once while the monitor is open. It restarts the ESP32 and prints the startup messages. Release BOOT if it was held during flashing. Startup may include normal ESP32 boot text before this program's text.

**Expected program text:**

```text
SWRM relative kiwi drive. Boots DISARMED; never auto-moves.
Driver acknowledged. DISARMED. No physical motion verified.
```

If you see a driver fault instead, the ESP32 may still be successfully running: the motor driver may simply be unpowered. Leave the monitor open and proceed to Step 7 to initialize after restoring its established power.

If the text is garbled, exit with Ctrl+C and reopen with the command above; confirm it says 115200 and uses the correct port. See [PlatformIO monitor documentation](https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html) for the supported flags and logging.

**Checkpoint:** monitor open at 115200 and you can receive messages. Once it is open, robot commands are bare words—no `pio` prefix.

## Step 7: Check communication and understand arm

**Where:** ROBOT MONITOR. Wheels still clear for the first test. Restore the robot's existing, verified motor/driver power when ready.

First type each of these on its own line, pressing Enter:

```text
help
status
```

`help` prints the command menu. `status` prints software state and the current wheel assumptions. Neither starts motion. After successful initialization, the relevant status fields should be:

```text
ready=1 armed=0 fault=0 busy=0
```

The real line also includes `speedLimit=3 pulses/10ms`, which is for optional encoder speed mode. It does not set the timed PWM percentage. Other lines show the timed defaults of 500 ms / 20%, and wheel diameter/CPR may be zero. Those zeros are okay for timed `drive` and `motors`.

| Field | Meaning |
| --- | --- |
| `ready=1` | Driver acknowledged initialization. This does not certify physical motion. |
| `armed=0` | Disabled from starting a motion until you type `arm`. |
| `fault=0` | No latched communication/distance fault. |
| `busy=0` | No motion currently active. |

If `ready=0` or `fault=1`, first check that the driver is now powered and its existing I²C connection is intact, then type:

```text
init
status
```

`init` sends zero-output requests, writes motor-type/encoder-polarity settings, and clears a fault **only if initialization succeeds**. It leaves the robot disarmed. It does not erase firmware or require wheel measurements.

Now type:

```text
arm
```

Expected response:

```text
ARMED. Motion requires a separate command.
```

`arm` means **enable the next motion request**. It sends zero commands first; it does not move wheels by itself, and it has nothing to do with a physical robot arm. A completed/stopped trial disarms automatically. An invalid motion command can leave it armed, so type `stop` if you want to cancel the enabled state.

**Checkpoint:** successful initialization, no fault, arm accepted. No wheel measurement or encoder read is required for these timed tests. You can skip `enc`, `jog`, `wheel`, `cal`, and `move` for today's first trial.

## Step 8: Send the first timed pattern

**Where:** ROBOT MONITOR, wheels clear for the initial check. If you have already armed in Step 7, sending `arm` again just requests zeros and leaves it armed.

Type each line with Enter:

```text
arm
drive 1 0
```

Expected output with unchanged defaults:

```text
TIMED PWM: M1=0 M2=-17 M3=17 percent, 500ms. Direction/distance unmeasured.
Timed pattern done; disarmed.
```

It sends that signed duty pattern for about half a second, then requests zero outputs. The monitor remains open afterward; no new upload is needed for another command.

The two numbers `1 0` are **direction weights**. They are not centimeters, seconds, or wheel revolutions. With the assumed ideal layout, this is called forward. On the actual robot the first observation establishes what it does; it can translate or turn because motor signs/placement have not been measured. The motor pattern is deterministic; physical response depends on friction, charge and load.

**Stop during a trial:** type either of these and press Enter:

```text
stop
```

or:

```text
!
```

These send zeros and disarm. Since the monitor buffers input until Enter, `!` must be followed by Enter in this setup. If a partially typed line is in the way, adding `!` then Enter still triggers the firmware's stop shortcut; the rest of that line is discarded. **Ctrl+C exits the monitor and does not send a robot stop.** If software stopping fails, use the physical motor-power disconnect.

The initial half-second trial is short enough that it may finish before you can type `stop`. If you want to practice stopping, with wheels raised you can use `arm`, then `drive 1 0 2000`, then immediately `!` and Enter. Its maximum duration is still 2 seconds. This verifies a delivered software stop, not a hardware emergency-stop circuit.

If no wheel turns but the trial is accepted, do not interpret it as correct physical motion. The 20% cap may be insufficient to overcome friction; after the trial ends you may try `arm`, then `drive 1 0 500 25`. This raises duty to the program's 25% maximum. If that still does not move a wheel, check actual driver power/protocol/motor connections rather than extending trials indefinitely.

**Checkpoint:** output pattern printed, ending/disarmed message received, wheels observed stopped. Physical movement is confirmed only by your observation, not the printed message.

## Step 9: Test other patterns and record notes

**Where:** ROBOT MONITOR. Once the initial stop behavior has worked on the stand, use a clear level floor area for short chassis-motion trials. Reposition the stationary robot to the same marked starting orientation before comparisons. Do not command motion while repositioning it.

Pick a visible front reference such as a sticker and define left/right from the robot's point of view. If you are facing the robot, its left is your right. For each trial:

1. Start from the marked reference pose.
2. Type `arm` and Enter.
3. Type exactly one test command and Enter.
4. Observe the motion and wait for `Timed pattern done; disarmed.` Do not paste a whole test sequence at once; commands received while busy are rejected.
5. Record the command, the printed M1/M2/M3 duty values, observed translation and any rotation.

Try these separately:

| Trial command, after `arm` | Default duty pattern | Record |
| --- | --- | --- |
| `drive 1 0` | 0, -17, +17 | Baseline direction A; did it turn? |
| `drive -1 0` | 0, +17, -17 | Opposite commanded signs; compare with A. |
| `drive 0 1` | +20, -10, -10 | Baseline direction B; did it turn? |
| `drive 0 -1` | -20, +10, +10 | Opposite commanded signs; compare with B. |
| `drive 1 1` | +15, -20, +5 | Combined pattern; compare with A/B. |

Repeat a useful command three times from the same starting orientation to see how consistent the physical motion is. If a pattern produces mostly translation you can later redefine the robot's front relative to it. If it spins, simply rotating the robot before starting will not remove that spin; use channel/sign observations later to improve the mix.

**Optional time and power:**

| Command | Meaning |
| --- | --- |
| `drive 1 0` | Defaults: 500 ms, 20% maximum duty in the mix. |
| `drive 1 0 1000` | 1000 ms (1 second), same default duty. |
| `drive 1 0 500 25` | 500 ms, up to 25% duty. |
| `drive 0.5 0` | Smaller contribution; rounded output and friction may limit response. |

X/Y must each be -1..1. Duration must be a whole 50..2000 milliseconds. Duty must be a whole 1..25. Longer time does not establish calibrated distance. The 20/25% values are chosen initial test settings, not a promised motor speed.

**Direct motor patterns:** if you want to bypass geometry completely, use `arm` then:

```text
motors 0 -20 20
```

This commands M1=0, M2=-20%, M3=+20% for 500 ms. A fourth number overrides duration, e.g. `motors 0 -20 20 800`. Each duty must be a whole number from -25 to +25. Negative reverses the requested rotation; it does not mean a known chassis direction.

**Optional single-wheel discovery:** while raised, try `arm` then `pulse 1 20 300` to request M1 at 20% for 300 ms. Repeat separately for channels 2 and 3, rearming each time. Record which physical wheel moves. Updated `pulse` / `jog` diagnostics allow up to 5000 ms; use [LONG_MOTOR_TESTS.md](LONG_MOTOR_TESTS.md) for three-second direct tests after uploading the updated firmware.

**Notes to keep:** use [TEST_NOTES.md](TEST_NOTES.md), or write the same fields in your notebook:

- Robot/date; marked front; floor surface; battery state; payload/LED assembly attached or removed.
- Exact command, duration, requested duty, and printed M1/M2/M3 values.
- Observed direction from the marked front; clockwise/counterclockwise turn and rough angle.
- Approximate travel, if easy to measure; run-to-run variation; which wheels moved or stalled.
- Whether it stopped automatically or via `stop`; exact errors; monitor log filename.

Example **illustrative note, not a measured result**: “Trial A1, sticker front, drive 1 0, 500 ms, output 0/-17/+17, moved toward sticker-right about 4 cm, slight clockwise turn, stopped automatically.”

The monitor log records received firmware messages, not physical motion. Record the typed command explicitly in your notes as well. Find the logs later in this project’s `logs` folder; its printed file path identifies the current session. You can view that folder from MAC TERMINAL after exiting the monitor using `open logs`.

## Step 10: Stop, exit, reconnect, and upload changes

**Normal shutdown, in ROBOT MONITOR:**

1. Type `stop` and press Enter. Confirm stopped/disarmed and observe stationary wheels.
2. Switch off/isolate motor power using the established disconnect.
3. Hold **Control** and press **C** once to exit the monitor. This is Control+C, not Command+C. The Mac `%` prompt returns.
4. Unplug USB when finished.

The ESP32 retains uploaded firmware even after power-off. Tomorrow, if code has not changed, repeat connection/port discovery/monitor steps; **no rebuild or re-upload is needed**. Reconnecting/resetting starts disarmed and restores the compiled defaults. RAM-only `wheel`/`cal` edits are lost, but the firmware itself remains installed.

To reopen the monitor from MAC TERMINAL, use the Step 6 command with the current port. After reconnecting, check `status`; use `init` if the driver was powered later. Then `arm` before each new trial.

To change code, first stop, isolate motor power and exit the monitor. Edit/save files in VS Code, then repeat build/upload/monitor. Only one application should use a serial port at a time. Editing a source file on the Mac does not automatically change the program already running on the ESP32.

If USB disconnects during a trial, that is **not** a guaranteed stop. A still-powered controller/driver can continue until the command's deadline, and a controller/bus failure can prevent the final zero from reaching the driver. Use the physical motor-power disconnect if there is unwanted movement. There is no established hardware driver watchdog or serial-disconnect heartbeat in this program.

## Step 11: Fix errors

First identify **where** the error happened: a MAC TERMINAL command, upload connection, or a ROBOT MONITOR response. Take a photo/copy the exact message and last command. If the robot is moving unexpectedly, cut motor power first.

| Error or symptom | What to do |
| --- | --- |
| `command not found: pio` | Use the full `/Users/calvinfisher/.platformio/penv/bin/pio` path shown here. |
| `no such file or directory` for `cd` | Copy the quoted project path from Step 2. Check that you are on this Mac and the folder exists. |
| `Not a PlatformIO project` / missing `platformio.ini` | Run `pwd`; enter the exact `kiwi-drive-today` folder. Do not use the parent folder or old `SWRM_Leds-master` project. |
| Build `[FAILED]` | Copy the first meaningful error above the summary and your last command. Dependencies/network downloads may fail separately from C++ compile errors. Do not upload until build succeeds. |
| No new USB serial entry | Known data cable, another adapter/port, reconnect. If still absent, identify the board's actual USB bridge before installing its official driver. Power LEDs alone do not prove a data link. |
| `/dev/cu.YOUR_PORT` not found | Replace the placeholder with the actual line from `pio device list`. Relist after a reconnect. |
| Port busy / cannot open / resource busy | Close other VS Code/Arduino serial monitors; use Ctrl+C to close this monitor before uploading. Keep one application on the port. |
| Upload `Connecting...` / `Failed to connect` / wrong boot mode | Correct port, closed monitor, data cable, then BOOT/EN procedure in Step 5. |
| Nothing in monitor | Confirm correct port, tap EN/RESET once with BOOT released, try `help` then Enter; missing boot text alone does not mean failure. |
| Garbled characters | Reopen using Step 6 with `--baud 115200`. |
| `zsh: command not found: arm` or `drive` | You are in MAC TERMINAL, not the robot monitor. Start the Step 6 command, then type robot commands there. |
| `ERR: unknown command/argument count` | You may have typed a Mac command into the monitor, made a typo, or included extra words. Type `help`. Every command is one line with plain spaces. |
| Mistyped a command | Updated firmware supports Backspace/DEL and Ctrl+U to clear an unfinished line. Unsupported control bytes reject that line. If you need to cancel the line and disarm, type `!` then Enter. The original uploaded version lacked editing support; rebuild/upload to get this fix. |
| `FAULT: no driver/stop acknowledgement at 0x34` | Driver may be unpowered. Check existing motor/driver power and I²C connection, then `init`. USB upload success alone does not confirm driver communication. |
| `FAULT: timed PWM write` / `refresh write` | Stop request attempted, fault latched. Check supply/I²C, isolate motor power if needed, then `init` after the cause is fixed. |
| `ERR: drive rejected` | Type `status`. If no fault, type `arm` then the valid drive command. Check X/Y -1..1, optional duration 50..2000 ms, optional duty 1..25. No wheel diameter is needed. |
| `ERR: motors rejected` | Same state check; use three whole signed duty values -25..25 and optional whole duration 50..2000 ms. |
| `ERR: busy; stop first. No queue.` | Wait for completion or type `stop` then Enter; rearm before the next trial. Do not paste batches of motions. |
| `ERR: invalid number` / line too long | Enter a fresh short command exactly as shown. Invalid input does not start a new motion; it does not automatically stop an already running one. Use `stop` to stop. |
| Accepted timed command, no wheel motion | Note the printed duties; if appropriate try the allowed 25% duty. Check driver supply, motor connection and PWM protocol support if still stationary. |
| USB-only power; accepted commands but no motion | Verify the actual driver motor-supply input and established battery/regulator arrangement. I²C acknowledgement does not confirm adequate motor voltage. Do not infer battery wiring or increase power limits as a substitute for that check. |
| Unexpected direction/spin | Expected to be possible with unknown layout/signs. Record the pattern/result, try raw `motors` patterns, then correct `wheel` settings later if desired. |
| `move rejected`, wheel diameter/CPR are zero | `move` is calibrated centimeters. Use `drive` for today's unmeasured tests. |
| Stop write failed / robot still moves | Use physical motor-power disconnect. Software cannot guarantee a stop through a broken bus/controller. Fix hardware/communication before continuing. |
| Permission/cache error from PlatformIO | Save the exact message. Do not delete global caches or change ownership blindly. Agent verification used a temporary metadata directory because its sandbox cannot write `~/.platformio`; this does not by itself establish an ownership problem in your own Terminal session. |

**Normal recovery sequence in ROBOT MONITOR, after fixing the actual issue:**

```text
init
status
arm
```

Check each reply before continuing. If `init` still fails, `arm` should stay rejected. Repeating initialization is not a repair for a loose wire or wrong supply.

## Step 12: Understand what was verified

The verification for this walkthrough includes an **ESP32 firmware build** for `esp32dev` using the installed ESP32 Arduino toolchain and PlatformIO, plus host tests with compiler warnings treated as errors and address/undefined-behavior sanitizers enabled.

The host tests execute the actual `src/main.cpp` command/state code against simulated serial and I²C. They check that:

- Timed `drive` accepts unknown diameter/CPR and does not read encoders.
- Default, reversed, diagonal and raw channel commands generate the expected outputs; channel 4 stays zero.
- Invalid/non-finite/out-of-range input does not start a motion.
- Disarmed/busy/fault states gate commands; normal completion and `stop`/`!` disarm.
- Timed deadlines stop requests, including across timer wrap; serial input does not remove those deadlines.
- I²C errors latch a fault and stop writes are retried after communication recovers.
- Optional calibrated distance control still passes its motion simulations and encoder-fault tests.

Build/test evidence is in `verification/build.log`, `verification/host-tests.log`, and `verification/summary.json`. The JSON includes hashes of the exact firmware/configuration sources compiled. Verification used the existing installed toolchain/packages with a temporary writable PlatformIO metadata directory because this agent cannot write to `~/.platformio`. Your normal Terminal commands above use your ordinary installation.

**What remains unverified:** upload to your physical board, electrical compatibility, installed driver revision, actual motor response, movement direction, floor distance, and physical stopping. Compilation and simulation are evidence about software; your first bench trial supplies hardware evidence. No wheel movement was performed while writing this guide.

## Command reference

Type these in ROBOT MONITOR, one command per line. Optional arguments are shown in square brackets in syntax descriptions; do not type the brackets.

| Command | Meaning / when to use |
| --- | --- |
| `help` | Print the command menu. Works while moving. |
| `status` | Print ready/armed/fault/busy and settings. Works while moving. |
| `init` | Retry driver setup and zero outputs after a fix; stays disarmed. Only while idle. |
| `arm` | Enable a motion request; does not start motion itself. Only while idle and no fault. |
| `stop` | Send zeros and disarm, including during movement. |
| `!` | Stop shortcut, recognized even inside a typed line; press Enter in this monitor. |
| `drive X Y [MS [PWM]]` | Timed assumed-direction mix, no measurements needed. Default 500 ms / 20%. |
| `motors M1 M2 M3 [MS]` | Direct timed PWM by driver channel, no geometry/encoder data. Default 500 ms. |
| `pulse MOTOR PWM MS` | Direct single-channel PWM trial; channel 1..3, signed duty 1..25, duration 50..5000 ms. |
| `enc` | Read the three encoder counters; optional later. It can latch a communication fault if reads fail. |
| `jog MOTOR SPEED MS` | Direct single-channel encoder-regulated speed test, signed speed 1..3 pulses/10ms, duration 50..5000 ms; requires correct internal feedback configuration. |
| `wheel MOTOR ANGLE MOTOR_SIGN ENCODER_SIGN` | Optional later: edit that channel's assumed geometry/signs, while disarmed. RAM only. |
| `cal MOTOR DIAM_CM CPR` | Optional later: enter measured scale, while disarmed; channel 0 applies to all three. RAM only. |
| `move X_CM Y_CM` | Optional later: calibrated relative distance; requires known scale and mapping. |

In **MAC TERMINAL**, `pwd` shows your folder, `cd 'path'` changes it, `pio run` builds, upload copies firmware, and `pio device monitor` opens the robot conversation. **Control+C** cancels a Mac command/exits this monitor; it is not a robot stop. No Python script or separate laptop movement program is needed for the serial workflow above.
