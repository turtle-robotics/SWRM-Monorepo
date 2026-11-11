# Communication-only driver check

Purpose: distinguish a basic I2C connection failure from motor-control/configuration traffic. This separate firmware probes address 0x34 ten times, once per second. It does **not** send a register number, zero-output command, motor type, PWM value, speed value or encoder request. It stops probing after ten samples and stays quiet. EN/RESET repeats the check.

The installed ESP32 Wire library returns 0 for success, 2 for no acknowledgment, 5 for timeout and 4 for other errors. Error 4 alone does not diagnose low voltage. The exact blink meaning of this board revision's PWR LED has not been established.

## Upload

Keep wheels clear. Power everything off before changing this firmware experiment, so the driver does not retain a previous motor command. With the reported USB-only arrangement, disconnect USB first; leave other supplies disconnected. Reconnect for programming. This diagnostic contains no software motor-stop command; use physical power removal if necessary.

Exit any monitor with Ctrl+C, then run in **Mac Terminal**:

```sh
cd '/Users/calvinfisher/Desktop/SWRM TURTLE/kiwi-drive-today'
/Users/calvinfisher/.platformio/penv/bin/pio run --project-dir i2c-diagnostic --target upload --upload-port /dev/cu.usbserial-0001
```

After success, open its monitor:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio device monitor --project-dir i2c-diagnostic --port /dev/cu.usbserial-0001 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0
```

Briefly press/release ESP32 EN/RESET once to see a complete run. Do not hold BOOT. Do not type arm/init/status; this is a different, diagnostic-only program, with no command parser.

Copy the ten `Probe` lines. Example format, **not a real hardware result**:

```text
Probe 1/10 addr=0x34 result=0 SDA=HIGH SCL=HIGH
...
FINISHED: no more probes until reset. No motor commands sent.
```

SDA/SCL readings are the digital levels immediately before each probe. They are not measurements of pull-up voltage, motor supply or signal quality. They can help flag a line held low but do not certify electrical correctness.

Observe whether the Hiwonder LED keeps blinking **after FINISHED**, when this diagnostic is no longer transmitting. Blinking then is not being triggered by ongoing I2C writes from this firmware; it still could be a normal indicator or a supply/reset symptom. Do not assign a fault meaning without the board revision's documentation or measurement.

| Result | Interpretation |
| --- | --- |
| Ten result=0 lines | Address acknowledgment is stable for this short, no-payload test. Motor supply and motor operation remain unverified. |
| Mixed success/failure or all failures | Basic communication is unstable/failing without motor commands. Check the existing signal/ground/power path and ESP32 I2C state before further motor tests. |
| A line consistently LOW | Investigate whether that line is held low by wiring/device state. This does not identify the component responsible. |
| Repeated ESP32 startup banners without pressing reset | Controller itself is restarting; inspect reset/power behavior separately. |

## Restore the movement program

After exiting the monitor, from the parent `kiwi-drive-today` folder, upload:

```sh
/Users/calvinfisher/.platformio/penv/bin/pio run --project-dir . --target upload --upload-port /dev/cu.usbserial-0001
```

This restores the already compiled movement firmware; it boots disarmed. The diagnostic does not change source files in the main project or overwrite the Hiwonder board's own firmware.
