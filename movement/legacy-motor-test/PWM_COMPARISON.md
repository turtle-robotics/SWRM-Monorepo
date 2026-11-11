# V4: check open-loop PWM after speed-mode history effects

The physical A/B result with type 2/polarity 1 showed previously activated motors joining later tests, even after speed zeros and `init`. The software submitted fresh four-byte speed snapshots with explicit zeros; the driver interpretation/internal controller state remains unverified. This does not prove that integral windup, wiring or any other single cause is responsible.

V4 adds `mode pwm` to request Hiwonder's documented open-loop duty interface at register 31. It avoids the speed controller's encoder regulation. PWM mode always uses four-channel snapshots, irrespective of the stored speed transport preference. Each snapshot sets unselected channels and channel 4 to zero. During an active PWM trial, the snapshot is refreshed every 20ms (a chosen 50Hz interval), and every failed write ends the test with a fault and zero attempts. Successful refreshes are not printed individually, to avoid flooding Serial; start/stop and failed packets remain logged.

Earlier PWM experiments used up to 25% duty with different refresh behavior. This experiment starts at **40% for one second** and caps duty at **50%**, both chosen diagnostic limits rather than motor specifications. It changes control mode, refresh behavior and starting duty; a successful result would not by itself isolate a unique cause of the earlier failure.

## Upload

Leave the motor/encoder harnesses connected and keep all wheels fully raised. Send `stop`, then Ctrl+C to exit the monitor. In Mac Terminal:

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
/Users/calvinfisher/.platformio/penv/bin/pio run --project-dir legacy-motor-test --target upload --upload-port /dev/cu.usbserial-0001
```

After success:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --project-dir legacy-motor-test --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
```

Verify the **MOTOR COMPARISON v4.1** banner. This is the same test-project folder, not another separate project.

## Select PWM mode

In robot monitor, enter each line separately and inspect its response:

```text
mode pwm
profile 2 1
init
status
```

Settings edits require disarmed/idle state and no pending stop retries; they do not send motor output themselves. `init` in PWM mode requests PWM zero before and after configuration. Status must say `MODE=PWM`, output register 31 and percent-duty units. PWM initialization, arming, trials, completion and stop retries do not write speed register 51. This is not a documented full reset of driver internals.

If any initialization/arm write fails, pause and copy the error rather than continuing to motion.

## Repeat the observed order

Enter `arm`, then `motor 2 40 1000`. After DONE/disarming, enter `arm`, then `motor 1 40 1000`. After that completes, enter `arm`, then `motor 2 40 1000` again. Wait between trials; do not paste them as a batch.

| Trial | Requested nonzero channel | Expected PWM data at register 31 |
| --- | --- | --- |
| A | M2 at 40% | `00 28 00 00` |
| B | M1 at 40% | `28 00 00 00` |
| C | M2 at 40% | `00 28 00 00` |

Hexadecimal 28 is decimal 40. The stop packet is `00 00 00 00` at register 31. All four values are sent again on every refresh, so the application cannot accumulate an old channel value between trials. That is a software property, not proof of how this board executes the bytes.

Type `!` or `stop` to send zero duty and disarm. Use physical power removal if motion does not stop. Do not continue a stalled/buzzing motor or repeat tests that cause bus faults. No calibrated geometry or distance input is needed here.

Record which assemblies move for A/B/C, whether they stop, and any error messages. Independent PWM with coupled speed behavior would narrow the investigation toward feedback/mode handling; coupling in PWM too would leave broader driver interpretation, electrical and wiring possibilities. No result is assumed in advance.

To return to speed mode later: while disarmed use `mode speed`, then `init`; `motor` values then mean pulses/10ms again. Keep the units explicit. `original` is rejected in PWM mode.

Source: [Hiwonder register definitions](https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/3._Arduino_checked.html). The selected duty cap and refresh interval are engineering choices, not claimed manufacturer defaults. Tests cover PWM history snapshots, refreshes, no speed-mode writes, zero stops, magnitude limits and failure recovery; physical isolation is still unverified.

## V4.1: after only a tiny twitch at 50% for two seconds

Do not increase duration or duty again yet. V4.1 adds `voltage` and `enc` read-only commands to the same project. Upload using the commands above. No harness unplugging is required. Keep wheels raised.

After reconnecting the serial monitor, enter each line separately:

```text
status
voltage
enc
voltage
enc
```

Do not arm for these checks. A fresh boot starts disarmed; neither diagnostic needs `init`. `voltage` selects register 0 and reads two little-endian bytes, interpreted as reported millivolts. `enc` selects register 60 and reads all four signed 32-bit cumulative counts without resetting them. The register-address selection contains no motor payload. Failed or short reads print an error and no interpreted value. Reads are refused while armed, running or stop retries are pending, so they cannot delay a timed stop. They do not clear or bypass a motion fault.

Paste the complete output and the prior tiny-movement test's `mode`, REQUEST and WRITE lines, if available. The numeric argument to `motor` means percent only when status says MODE=PWM; in SPEED mode it means encoder pulses per 10ms. Do not assume that a bare number 50 has the same meaning in both modes.

A reported voltage at rest alone cannot rule out supply drop during motion, and its accuracy/sense point depends on the board. A zero or implausible report could also indicate protocol/revision differences. Encoder snapshots at rest are a baseline, not a motion test. No unique cause is assumed from the twitch.

Protocol cross-check: [PX4 Hiwonder driver, read_adc and read_encoder_counts](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/hiwonder_emm/HiwonderEMM.cpp). These diagnostics do not change the existing PWM cap, refresh rate or motion behavior. Host checks cover signed count extremes, partial responses, failed address selection, no motor payloads and armed/running rejection.
