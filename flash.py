#!/usr/bin/python3

import os
import sys

from pathlib import Path

MCU_NAME = "m8"
ISP_NAME = "avrisp2"
FILE_NAME = "ATMega8_pwm_based_faan_controller_with_timer"

if not MCU_NAME:
    print("MCU_NAME is not set")
    sys.exit()

if not ISP_NAME:
    print("ISP_NAME is not set")
    sys.exit()

if not FILE_NAME:
    print("HEX_FILE_NAME is not set")
    sys.exit()

hexFile = Path("./bin/Release/" + FILE_NAME + ".hex")
if not hexFile.exists():
    print("File './bin/Release/" + FILE_NAME + "' does not exists")
    sys.exit()

os.system("avrdude -p " + MCU_NAME + " -c " + ISP_NAME + " -P usb -U flash:w:./bin/Release/" + FILE_NAME + ".hex")
os.system("avr-size -B ./bin/Release/" + FILE_NAME + ".elf")
