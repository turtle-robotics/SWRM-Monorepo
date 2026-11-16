# How to test motor encoder data
This guide details how to properly format encoder data, export it as a CSV file, and plot both raw and differential encoder data for all three motors on the SWRM bots. For the best results, make sure to do everything as specified in the instructions

>[!NOTE]
>Since ```read_serial.py``` uses serial to read output from the encoders and there can only be one serial connection per port, the script will **not** work if the serial monitor is open in your IDE. Likewise, it is recommended that you use commands like ```pio --target upload``` when uploading your code.

## Ensuring proper formatting in the main code
Before you run any of the scripts, you must first output your encoder data in a CSV format that is readable to the Python code. Currently, this is the **only convention** that ```process_csv.py``` will support:

|Loop Iteration | Motor 1 | Motor 2 | Motor3 |
|---------------|---------|---------|--------|
| 0 | e1 | e2 | e3 |
| 1 | e1 | e2 | e3 |
| 2 | e1 | e2 | e3 |
| ... | ... | ... | ... |
| n | e1 | e2 | e3 |


>[!NOTE]
>Please ensure that your variables used to read encoder data are of ```uint32_t``` type because raw encoder values are highly precise and will cause any smaller type to quickly overflow.

1. Before you output your encoder data, add a header for the CSV at the end of your setup function:
```c++
Serial.println("Loop Iteration, Motor 1, Motor 2, Motor 3")
```

2. Set up your loop function in a similar fashion to the example below. From what I gather, motor speed and delay don't matter, but the encoder values must be read at the very beginning of each loop iteration and your printf function should be identical to the one below for best results.

```c++
// ── Loop
// ──────────────────────────────────────────────────────────────────────
uint16_t loop_count = 0; // Count loop iterations (for conventional CSV formatting)
void loop() {
  // Read encoder data
  uint32_t e1 = readEncoder(ENCODER1_ADDR);
  uint32_t e2 = readEncoder(ENCODER2_ADDR);
  uint32_t e3 = readEncoder(ENCODER3_ADDR);

  // Output encoder data
  Serial.printf("%d,%ld,%ld,%ld\n", loop_count, e1, e2, e3);

  // Drive motors
  setMotorSpeeds(50, 50, 50);
  delay(1000);

  // Stop robot and wait
  eStopRobot();
  delay(1000);

  loop_count += 1; //Next iteration
}
```

> [!WARNING]
> This example uses code from the old SWRM_Leds repo and not the monorepo that you are reading this guide in because the monorepo's movement codebase has not yet been updated to output encoder data in a .csv format.

## Pulling encoder data using ```read_serial.py```
>[!NOTE] 
>```read_serial.py``` uses the pyserial library in order to make a serial connection. Run ```pip install pyserial``` if not already installed.

It is here where this guide recommends using the command line to upload your code instead of directly through VS Code or some other IDE. Begin by going about the typical PlatformIO workflow:

**(For Windows Users, replace '/' with '\')**
1. Change into the directory containing the code
```bash
cd /path/to/codebase
```

2. Compile the platform and dependencies specified in the ```platformio.ini``` (if necessary)
```bash
pio run --target=compiledb
```

3. Upload the code to the ESP32 and run the python script to start pulling serial data
```bash
pio run --target upload && python3 encoder_diagnostics/read_serial.py /dev/YOUR_SERIAL_PORT
```

  - Replace YOUR_SERIAL_PORT with the name of whatever serial port the ESP32 is connected to as per the output of ```pio device list```. The most common names are:
    - **Windows:**
    ```
    cu.usbserial-0001
    ```

    - **Linux:**
    ```
    ttyUSB0 ttyUSB1
    ```

    - **MacOS:**
    ```
    tty.usbmodemxxxxxx (could be any string of numbers)
    ```

4. The Python script should now be outputting the print statements that you specified in the last step and writing them to a file named ```encoder_data.csv```. For best practice, you should rename this file to something descriptive like ```encoder_data_black_bot_11-16-2026``` to eliminate ambiguity and avoid overwriting data whenever you run the script again.

## Processing data using ```process_csv.py```
