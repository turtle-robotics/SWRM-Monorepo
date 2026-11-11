# V3 software-only motor comparisons — leave harnesses connected

You do not have to disconnect motor/encoder plugs to run these comparisons. There is no automatic sweep: the operator selects one profile, initializes, arms, and requests one short test. Every motion is bounded and fault gating remains enabled.

Changing a C++ function's name does not change driver behavior. The meaningful alternatives here change the transmitted configuration bytes, speed-packet format, or I2C timing. These are comparisons of documented interfaces/settings, not claims that any profile has been physically verified on this robot.

## Upload once

With wheels still fully raised and chassis stationary, send `stop` in the current monitor and press Ctrl+C. No harness rewiring is needed. In Mac Terminal:

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
/Users/calvinfisher/.platformio/penv/bin/pio run --project-dir legacy-motor-test --target upload --upload-port /dev/cu.usbserial-0001
```

After success:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --project-dir legacy-motor-test --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
```

The banner is **MOTOR COMPARISON v3**. `init`, `arm`, `motor`, `stop`, `!`, and `status` still work. No motion starts automatically. Settings are in RAM; reset restores type=1, polarity=1, single-register format, 100kHz and no extra gap.

## New controls

| Command | Meaning |
| --- | --- |
| `profile 2 1` | Store motor type 2 (published N20 setting), encoder polarity 1. |
| `profile 2 0` | Same motor type, opposite encoder-polarity setting. |
| `profile 1 1` | Original project's numeric type/polarity values. |
| `transport block` | One packet: register 51 plus four signed channel bytes, as in the original code. |
| `transport single` | Separate one-byte channel writes at registers 51..54; clear unselected channels before enabling selected ones. |
| `bus 25000` | Store a slower 25kHz I2C clock for a comparison when write errors persist. Other allowed values: 50000 and 100000. |
| `gap 5` | Pause 5ms after each completed I2C write. Optional range 0..10ms; this is a chosen settling experiment, not a protocol requirement. |
| `status` | Show the selected settings and flags; no register write. |

Settings edits require idle/disarmed state and no pending failed-stop retries. They send **no I2C payloads themselves**, and invalidate initialization. Run `init` to apply them; `arm` remains rejected until that succeeds. Changing settings does not clear a fault by itself.

Each write now logs its actual data bytes in hexadecimal. Example: a block request only for M2 at +10 should log register 51, four bytes, `data=00 0A 00 00`. 0A in hexadecimal is decimal 10. This logs the bytes software hands to Wire; it is not an electrical logic-analyzer capture.

## First comparison: type 2, polarity 1, block format

In the robot monitor, enter **one line at a time**, checking each response:

```text
stop
transport block
profile 2 1
bus 100000
gap 0
init
status
```

If every initialization write succeeds and status shows no fault:

```text
arm
```

```text
motor 2 10 1000
```

Observe precisely which motor assemblies turn and whether they stop. Type `!` or `stop` to end early. If a wheel races or motion fails to stop, use physical power removal; correct encoder feedback still matters in speed mode.

This uses the published N20 configuration with a raw speed request of 10 pulses/10ms. Earlier type-2 tests used only up to 3 pulses/10ms or low-duty PWM, so they did not cover this combination.

## Compare profiles manually

Use the same `motor 2 10 1000` after successful `init` and a fresh `arm` for each row:

| Trial | Transport command | Profile command | Record M1 / M2 / M3 movement and errors |
| --- | --- | --- | --- |
| A | `transport block` | `profile 2 1` | |
| B | `transport block` | `profile 2 0` | |
| C | `transport single` | `profile 2 1` | |
| D | `transport single` | `profile 2 0` | |

Between rows, wait for automatic completion/disarming, or send `stop`. Set the next row's preferences, run `init`, check its replies, then `arm` and `motor 2 10 1000`. Do not paste a batch that would start multiple tests automatically. Changes are not allowed while armed/running.

If a row gives independent M2 motion, stop the matrix and verify M1 and M3 individually with the SAME settings, rearming each time. Do not change settings again until those observations are recorded.

If a write returns a nonzero result, no motor test should follow on that attempt. Record it. For a timing-only comparison, keep the row's profile/transport unchanged, select `bus 25000`, then `init`. If it remains unreliable, compare `gap 5` separately. Do not keep rearming through repeated communication faults.

These tests may narrow configuration versus transfer behavior. They cannot prove or repair arbitrary wiring faults or guarantee that the installed driver firmware implements the published protocol correctly. If all profiles remain coupled, an electrical isolation/measurement test or the team's independently working single-motor example will still be needed later; it is not required to try this matrix now.

## Evidence and sources

Build/tests are saved in the parent `verification` folder. Tests cover runtime settings, required re-initialization, rejected edits while armed/running, actual packet construction, bounds, stop deadlines and fault handling. The model assumes the documented motor register semantics; it does not reproduce the unexplained physical coupling already observed on this robot.

Sources: original local main.cpp; [Hiwonder motor-type and polarity definitions](https://docs.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/3._Arduino_checked.html); [Hiwonder block and single-channel examples](https://wiki.hiwonder.com/projects/Black-Mecanum-Wheel-Chassis/en/latest/docs/4_Raspberry%20Pi_formatted.html).
