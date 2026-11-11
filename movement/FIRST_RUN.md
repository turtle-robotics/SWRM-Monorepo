# Plug in, send an input, observe the direction

For a beginner walkthrough with exact Mac commands, expected replies, notes and error recovery, start with [WALKTHROUGH.md](WALKTHROUGH.md). This file is the shorter checklist.

**No wheel measurements or channel identification are needed for this mode.** The code is complete for timed PWM trials. Its default patterns are assumptions; your first job is to observe what each one actually does. It does not promise a known direction or a distance in centimeters yet.

## Upload once

1. Support the robot with wheels clear. Isolate motor power before USB connection because old firmware may auto-move. Use the robot's already verified supply/wiring arrangement; USB alone does not establish motor power.
2. Connect the ESP32 board's USB programming port to your Mac with a data cable.
3. Open `/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today` in VS Code. Use PlatformIO **Build**, then **Upload**, selecting `esp32dev` if asked.
4. Open PlatformIO **Monitor**, at **115200 baud**. Press EN/RESET if needed to see startup text.
5. Restore the robot's established motor/driver power. If the driver was unpowered during startup, type `init` and Enter. Wait for the acknowledged/disarmed message. The new program never starts moving by itself.

See [the full guide](README.md#2-build-and-upload-this-project) if you need terminal commands, port discovery or upload troubleshooting.

## First input

Type each line and press Enter:

```text
arm
drive 1 0
```

It prints the exact PWM pattern (default `0, -17, +17` for M1/M2/M3), applies it for **half a second**, then sends zeros and disarms. It does not read encoders or ask for diameter. With the wheels raised you can observe rotation first; use short trials in a clear floor area when ready to record actual chassis motion.

The terminal should say `Timed pattern done; disarmed.` before you start the next trial. Type `arm` again for each one:

| Input after `arm` | What to compare |
| --- | --- |
| `drive 1 0` | Baseline pattern; label its observed direction A. |
| `drive -1 0` | Same channels with reversed signs; compare with A. |
| `drive 0 1` | Second pattern; label observed direction B. |
| `drive 0 -1` | Opposite of B's motor pattern. |
| `drive 1 1` | Mix A and B, with outputs scaled to the duty cap. |

X/Y are **direction weights**, each -1..1. They do not mean centimeters, feet, meters, or revolutions. Internally +X is called forward and +Y left; until mapping is learned, A/B are useful observation labels. Repeating an input repeats the motor command, but battery/load/friction can change the physical result. A turning pattern cannot be made into pure translation merely by relabeling the robot's front.

To override the duration or power:

```text
arm
drive 1 0 1000
```

That uses 1000 ms (1 second), at the default 20% duty. If the initial duty does not overcome friction:

```text
arm
drive 1 0 500 25
```

That uses 500 ms, with duty up to 25%. The program limits `drive` / `motors` to 2 seconds and their PWM output to 25%. Direct single-wheel `pulse` / `jog` diagnostics now allow up to 5 seconds; see [LONG_MOTOR_TESTS.md](LONG_MOTOR_TESTS.md). These are chosen first-test limits, not manufacturer speed ratings. Increasing time does not calibrate distance. Very small direction weights may round to zero.

## Try exact motor combinations

This skips geometry entirely:

```text
arm
motors 0 -20 20
```

The three numbers are direct signed PWM percentages for **M1, M2, M3**, respectively. Zero means that channel is stopped; a negative sign reverses the requested direction. It runs for the same default half second, then stops/disarms. Add time as the fourth number: `motors 0 -20 20 800`.

Record useful combinations in [TEST_NOTES.md](TEST_NOTES.md). If needed, isolate one wheel using `arm` then `pulse 1 20 300` (M1 at 20% for 300 ms); substitute channel 2 or 3. This discovery is optional for trying patterns, and useful later for assigning physical direction labels.

## Stop and read the code

Type **`stop` and Enter**, or **`!` and Enter**, during a trial to send zero commands and disarm. Exiting the serial monitor or unplugging USB is not a stop command. Software stopping depends on the ESP32 and motor driver still communicating; keep the physical motor-power disconnect accessible.

Read these files in this order:

1. `src/main.cpp`: opening overview, `setup()`, `loop()`, `driveRelative()`, then `startTimedPwm()`. Comments explain the state and each function.
2. `include/robot_config.h`: all hardware assumptions and operating choices, with their sources and rationale.
3. `include/kiwi_math.h`: the direction projection and where the trigonometric coefficients come from.

You can ignore `move`, `cal`, and encoder feedback until you want **centimeter-based movement**. The full [README](README.md) explains that optional later mode.
