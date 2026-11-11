#!/bin/bash
pio device monitor --project-dir legacy-motor-test --port /dev/ttyUSB0 --baud 115200 --exit-char 3 --eol LF --filter direct --dtr 0 --rts 0 --echo
