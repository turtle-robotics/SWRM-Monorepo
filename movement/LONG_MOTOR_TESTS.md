# Direct motor tests lasting several seconds

The firmware now accepts single-wheel `pulse` / `jog` tests up to **5000 ms (5 seconds)**. Start with **3000 ms (3 seconds)**. This makes activity easier to observe and gives you time to type a stop. It does not establish that the previous lack of motion was caused by a short duration.

Only the selected channel is commanded. These tests bypass kiwi mixing, wheel geometry, diameter and CPR calibration. The PWM cap remains 25%; the driver speed cap remains 3 pulses/10 ms. We have not increased either command magnitude in this update. The program resends the same command every 100 ms to check I²C contact, reports remaining time about once per second, and sends zeros/disarms at the deadline.

## 1. Upload the new limits

The version already on the robot may reject these longer durations. With wheels clear, send `stop` and Enter in the monitor, then Ctrl+C to return to the Mac shell. Do not change the existing supply wiring for this software test.

In **Mac Terminal**, run:

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
/Users/calvinfisher/.platformio/penv/bin/pio run --target upload --upload-port /dev/cu.usbserial-0001
```

Wait for `[SUCCESS]`, then open the monitor:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
```

In the **robot monitor**, type `help`. It should show **50..5000ms** on the pulse/jog lines. If it still shows the old limits, check that the upload succeeded from this project to the correct board.

## 2. Prepare and initialize

Support the robot securely with all wheels free to turn. Keep access to the physical power disconnect. With USB-only power as previously reported, unplugging that cable removes that source; do not rely on Ctrl+C to stop a motor.

Enter one line at a time:

```text
init
status
```

If initialization fails or a bus fault returns, pause the test and copy the message. Fault gating stays enabled.

## 3. Direct PWM test on M1, for 3 seconds

```text
arm
```

After the armed reply:

```text
pulse 1 25 3000
```

The first number is driver channel **M1**. The second is **25% signed PWM duty**, and the last is **3000 milliseconds**. Channels M2/M3/M4 receive zero.

Expect a start message naming M1, register 31 and 3000 ms; periodic messages show roughly 2000 ms and 1000 ms remaining; then `Wheel test done; disarmed.` These messages describe requested output and elapsed time, not measured rotation.

If a motor buzzes or strains without rotating, type **`!`** and Enter to end the test early and record that. There is no benefit to repeatedly holding a visibly stalled motor for longer. If there is no activity throughout the three-second test, record that outcome rather than repeatedly extending it. If motion is uncontrolled or the software stop is not effective, use the power disconnect.

## 4. Compare speed mode on M1

After the PWM test has ended/stopped, enter separately:

```text
arm
```

```text
jog 1 3 3000
```

This uses register 51, the same interface used in the original motor program. **3 means pulses per 10 ms, not 3% power.** The driver may increase duty internally to try to achieve that speed, so observe the first test with wheels raised. It ends after 3 seconds or a received stop. Encoder feedback configuration still affects this mode; it does not need measured wheel CPR for this raw request.

## 5. Other direct tests

Run one at a time, type `arm` before each, and wait for the ending message before another test:

| Command | Requested test |
| --- | --- |
| `pulse 2 25 3000` | M2 at +25% PWM for 3 seconds. |
| `pulse 3 25 3000` | M3 at +25% PWM for 3 seconds. |
| `pulse 1 -25 3000` | Reverse M1 at 25% duty for 3 seconds. |
| `jog 2 3 3000` | M2 at +3 pulses/10 ms for 3 seconds. |
| `jog 3 3 3000` | M3 at +3 pulses/10 ms for 3 seconds. |
| `jog 1 -3 3000` | Reverse M1 at -3 pulses/10 ms for 3 seconds. |

If a wheel turns and stops normally, you can observe it for up to 5 seconds by replacing `3000` with `5000`. Do not extend a no-motion test just to wait longer. No indefinite/latching motor command was added; every test has a fixed stop deadline.

Optional encoder evidence, while idle: type `enc` before a test and again afterward. A changing counter supports actual encoder rotation; constant counts are ambiguous without observing the wheel. An encoder-read error can latch a fault, so this is not required for PWM trials.

Record the exact start/countdown/end text, which physical wheel moved, direction, noise/twitching, and whether it stopped. If neither mode moves any wheel, the next step is to distinguish command/configuration, channel connections and motor supply; duration alone has not explained the failure.

`drive` / `motors` retain their 2-second limits. This extended diagnostic applies only to direct single-channel `pulse` / `jog`, whose magnitude limits are shown by `help`.
