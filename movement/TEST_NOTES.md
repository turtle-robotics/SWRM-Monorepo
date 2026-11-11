# Timed robot trials

Use the same marked starting orientation and similar battery/load conditions when comparing commands. Note the exact PWM numbers printed by the firmware; those are the repeatable inputs. Keep direction names provisional until you have observed them. Each trial requires `arm` first and ends by disarming.

Date / robot label:

Chosen front reference (a sticker, USB side, etc.):

Battery / load notes:

USB serial port from `pio device list`:

Monitor log filename (printed when opening the monitor with `log2file`):

Floor surface / payload / front marker:

| Trial ID | Exact command | Duration (ms) | Selected PWM | Printed M1/M2/M3 | Movement from marked front | Rotation / rough angle | Stop behavior / error |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A1 | | | | | | | |
| A2 (same start) | | | | | | | |
| A3 (same start) | | | | | | | |

The rows below are suggested commands, not measured results. Include the command in your own notes: the monitor log records received output and may not include your echoed typed input.

| Command | Printed M1/M2/M3 PWM | Observed translation | Turned? Which way? | Approx. distance | Notes |
| --- | --- | --- | --- | --- | --- |
| `drive 1 0` | Default: 0 / -17 / +17 | | | | |
| `drive -1 0` | Default: 0 / +17 / -17 | | | | |
| `drive 0 1` | Default: +20 / -10 / -10 | | | | |
| `drive 0 -1` | Default: -20 / +10 / +10 | | | | |
| `drive 1 1` | | | | | |
| `motors 0 -20 20` | 0 / -20 / +20 | | | | |
| | | | | | |
| | | | | | |

## Discover later, if useful

| Driver channel | Physical wheel position | Positive command's rolling direction | Positive command increases/decreases encoder count |
| --- | --- | --- | --- |
| M1 | | | |
| M2 | | | |
| M3 | | | |

Measured wheel diameter(s):

Measured driver counts per wheel revolution:

Useful `wheel` / `cal` commands to re-enter after reset:

Remember: `drive` means timed PWM, `motors` means direct timed channel commands, and `move` means calibrated centimeters. Time/PWM observations alone do not establish an accurate distance model.
