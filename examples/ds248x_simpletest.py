# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

'''Adafruit DS248x DS18B20 Example'''

import time
import board
import busio
from adafruit_ds248x import Adafruit_DS248x

# Initialize I2C bus and DS248x
i2c = board.STEMMA_I2C()
ds248x = Adafruit_DS248x(i2c)

try:
    ds248x.reset()
    print("DS248x OK!")
except RuntimeError:
    print("DS248x initialization failed.")

while not ds248x.OneWireReset():
    print("Failed to do a 1W reset")
    if ds248x.short_detected():
        print("\tShort detected")
    if not ds248x.presence_pulse_detected():
        print("\tNo presence pulse")
    time.sleep(1)
print("One Wire bus reset OK")
rom = bytearray(8)
if not ds248x.OneWireSearch(rom):
    print("No more devices found\n\n")
    raise

print("Found device ROM: ", end="")
for byte in rom:
    print(f"{byte:02X} ", end="")
print()
while True:
    
    temperature = ds248x.ds18b20_temperature(rom)
    print(f"Temperature: {temperature:.2f} °C")

    time.sleep(1)