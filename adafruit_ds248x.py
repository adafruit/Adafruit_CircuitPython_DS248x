# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ds248x`
================================================================================

CircuitPython driver for the DS2484 I2C to 1-Wire Bus Adapter


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Adafruit DS2484 I2C to 1-Wire Bus Adapter Breakout <https://www.adafruit.com/product/5976>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_struct import Struct, ROUnaryStruct
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_bits import RWBits

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_DS248x.git"

# DS248x Command Definitions
DS248X_CMD_RESET = const(0xF0)
DS248X_CMD_SET_READ_PTR = const(0xE1)
WRITE_CONFIG = const(0xD2)
DS248X_CMD_1WIRE_RESET = const(0xB4)
DS248X_CMD_1WIRE_SINGLE_BIT = const(0x87)
DS248X_CMD_1WIRE_WRITE_BYTE = const(0xA5)
DS248X_CMD_1WIRE_READ_BYTE = const(0x96)
TRIPLET = const(0x78)

# DS248x Register Definitions
DS248X_REG_STATUS = const(0xF0)
DS248X_REG_READ_DATA = const(0xE1)
REG_CONFIG = const(0xC3)

# DS18B20 Commands
DS18B20_FAMILY = const(0x28)
DS18B20_T = const(0x44)
DS18B20_ROM = const(0x55)
DS18B20_SCRATCHPAD = const(0xBE)

class Adafruit_DS248x:
    def __init__(self, i2c, address=0x18):
        self.i2c_dev = I2CDevice(i2c, address)
        self._address = address
        self.ROM_NO = bytearray(8)
        self.LastDiscrepancy = 0
        self.LastDeviceFlag = False
        self.LastFamilyDiscrepancy = 0

    def begin(self):
        return self.reset()

    def reset(self):
        cmd = bytearray([DS248X_CMD_RESET])
        with self.i2c_dev as i2c:
            i2c.write(cmd)
        status = self.read_status()
        return (status != 0xFF) and (status & 0x10)  # Check if the RST bit (bit 4) is set

    def OneWireReset(self):
        if not self.busy_wait(1000):
            return False  # Return false if the bus is busy after the timeout
        cmd = bytearray([DS248X_CMD_1WIRE_RESET])
        with self.i2c_dev as i2c:
            i2c.write(cmd)
        status = self.read_status()
        return (status != 0xFF) and not self.short_detected() and self.presence_pulse_detected()

    def OneWireWriteByte(self, byte):
        if not self.busy_wait(1000):
            return False  # Return false if the bus is busy after the timeout
        cmd = bytearray([DS248X_CMD_1WIRE_WRITE_BYTE, byte])
        with self.i2c_dev as i2c:
            i2c.write(cmd)
        return self.busy_wait(1000)  # Return false if the bus is busy after the timeout

    def OneWireReadByte(self):
        if not self.busy_wait(1000):
            return False  # Return false if the bus is busy after the timeout
        cmd = bytearray([DS248X_CMD_1WIRE_READ_BYTE])
        with self.i2c_dev as i2c:
            i2c.write(cmd)
        if not self.busy_wait(1000):
            return False  # Return false if the bus is busy after the timeout
        if not self.set_read_pointer(DS248X_REG_READ_DATA):
            return False  # Return false if setting the read pointer fails
        read_buffer = bytearray(1)
        with self.i2c_dev as i2c:
            i2c.readinto(read_buffer)
        return read_buffer[0]

    def OneWireReadBit(self):
        #print("Starting OneWireReadBit")
        if not self.busy_wait(1000):
            #print("Busy wait failed in OneWireReadBit")
            return False  # Return false if the bus is busy after the timeout
        if not self.OneWireWriteBit(1):
            #print("OneWireWriteBit failed in OneWireReadBit")
            return False  # Return false if writing the command fails
        if not self.busy_wait(1000):
            #print("Busy wait failed after write in OneWireReadBit")
            return False  # Return false if the bus is busy after the timeout
        status = self.read_status()
        if status == 0xFF:
            #print("Failed to read status in OneWireReadBit")
            return False  # Return false if reading the status fails
        result = self.single_bit_result()
        #print(f"Read bit: {result}")
        return result

    def OneWireWriteBit(self, bit):
        #print(f"Starting OneWireWriteBit with bit: {bit}")
        if not self.busy_wait(1000):
            #print("Busy wait failed in OneWireWriteBit")
            return False  # Return false if the bus is busy after the timeout
        cmd = bytearray([DS248X_CMD_1WIRE_SINGLE_BIT, 0x80 if bit else 0x00])
        with self.i2c_dev as i2c:
            i2c.write(cmd)
        #print("Write bit completed")
        return True

    def OneWireSearchReset(self):
        self.LastDiscrepancy = 0
        self.LastDeviceFlag = False
        self.LastFamilyDiscrepancy = 0
        return True

    def OneWireSearch(self, newAddr):
        search_result = False
        id_bit_number = 1
        last_zero = 0
        rom_byte_number = 0
        rom_byte_mask = 1
        id_bit, cmp_id_bit = 0, 0
        search_direction = 0

        if not self.LastDeviceFlag:
            if not self.OneWireReset():
                self.LastDiscrepancy = 0
                self.LastDeviceFlag = False
                self.LastFamilyDiscrepancy = 0
                return False

            if not self.OneWireWriteByte(0xF0):
                return False

            while rom_byte_number < 8:
                id_bit = int(self.OneWireReadBit()) & 0x01
                cmp_id_bit = int(self.OneWireReadBit()) & 0x01
                if id_bit == 0xFF or cmp_id_bit == 0xFF:
                    return False

                if id_bit and cmp_id_bit:
                    break  # No devices participating
                else:
                    if id_bit != cmp_id_bit:
                        search_direction = id_bit
                    else:
                        if id_bit_number < self.LastDiscrepancy:
                            search_direction = (self.ROM_NO[rom_byte_number] & rom_byte_mask) > 0
                        else:
                            search_direction = (id_bit_number == self.LastDiscrepancy)
                        if not search_direction:
                            last_zero = id_bit_number
                            if last_zero < 9:
                                self.LastFamilyDiscrepancy = last_zero

                    if search_direction:
                        self.ROM_NO[rom_byte_number] = (self.ROM_NO[rom_byte_number] | rom_byte_mask) & 0xFF
                    else:
                        self.ROM_NO[rom_byte_number] = (self.ROM_NO[rom_byte_number] & ~rom_byte_mask) & 0xFF

                    # print(f"Updated ROM_NO[{rom_byte_number}]: {self.ROM_NO[rom_byte_number]:02X}, rom_byte_mask: {rom_byte_mask:02X}")

                    if not self.OneWireWriteBit(search_direction):
                        return False

                    id_bit_number += 1
                    rom_byte_mask <<= 1

                    if rom_byte_mask == 0x100:
                        rom_byte_number += 1
                        rom_byte_mask = 1

            if id_bit_number >= 65:
                self.LastDiscrepancy = last_zero
                if self.LastDiscrepancy == 0:
                    self.LastDeviceFlag = True
                search_result = True

        if not search_result or not self.ROM_NO[0]:
            self.LastDiscrepancy = 0
            self.LastDeviceFlag = False
            self.LastFamilyDiscrepancy = 0
            search_result = False

        for i in range(8):
            newAddr[i] = self.ROM_NO[i]
        return search_result

    def read_status(self):
        if not self.set_read_pointer(DS248X_REG_STATUS):
            return 0xFF  # Return an invalid value if setting the pointer fails
        status = bytearray(1)
        with self.i2c_dev as i2c:
            i2c.readinto(status)
        return status[0]

    def set_read_pointer(self, reg):
        cmd = bytearray([DS248X_CMD_SET_READ_PTR, reg])
        with self.i2c_dev as i2c:
            i2c.write(cmd)
        return True

    def read_config(self):
        if not self.set_read_pointer(DS248X_REG_CONFIG):
            return 0xFF  # Return an invalid value if setting the pointer fails
        config = bytearray(1)
        with self.i2c_dev as i2c:
            i2c.readinto(config)
        return config[0]

    def write_config(self, config):
        if not self.busy_wait(1000):
            return False  # Return false if the bus is busy after the timeout
        config_value = (config & 0x0F) | ((~config & 0x0F) << 4)
        cmd = bytearray([DS248X_CMD_WRITE_CONFIG, config_value])
        with self.i2c_dev as i2c:
            i2c.write(cmd)
        return True

    def busy_wait(self, timeout_ms):
        start = time.monotonic()
        while (time.monotonic() - start) < timeout_ms / 1000:
            if not self.is_1w_busy():
                return True
            time.sleep(0.001)  # Short delay to prevent busy waiting
        return False

    def is_1w_busy(self):
        status = self.read_status()
        return status != 0xFF and (status & 0x01)  # Check the 1-Wire busy bit

    def presence_pulse_detected(self):
        status = self.read_status()
        return status != 0xFF and (status & 0x02)  # Check the presence pulse detected bit

    def short_detected(self):
        status = self.read_status()
        return status != 0xFF and (status & 0x04)  # Check the short detected bit

    def logic_level(self):
        status = self.read_status()
        return status != 0xFF and (status & 0x08)  # Check the logic level bit

    def single_bit_result(self):
        status = self.read_status()
        result = (status & 0x20) != 0  # Check the single bit result bit
        #print(f"Single bit result: {result}")
        return result

    def triplet_second_bit(self):
        status = self.read_status()
        return status != 0xFF and (status & 0x40)  # Check the triplet second bit

    def branch_dir_taken(self):
        status = self.read_status()
        return status != 0xFF and (status & 0x80)  # Check the branch direction taken bit

    def ds18b20_temperature(self, rom):
        """
        Reads the temperature from a DS18B20 sensor.

        :param rom: The ROM address of the DS18B20 sensor
        :return: The temperature in Celsius
        """
        if rom[0] == DS18B20_FAMILY:
            try:
                # Select the DS18B20 device
                if not self.OneWireReset():
                    raise RuntimeError("1-Wire reset failed")

                self.OneWireWriteByte(0x55)  # Match ROM command
                for byte in rom:
                    self.OneWireWriteByte(byte)

                # Start temperature conversion
                self.OneWireWriteByte(0x44)  # Convert T command
                time.sleep(0.75)  # Wait for conversion (750ms for maximum precision)

                # Read scratchpad
                if not self.OneWireReset():
                    raise RuntimeError("1-Wire reset failed")

                self.OneWireWriteByte(0x55)  # Match ROM command
                for byte in rom:
                    self.OneWireWriteByte(byte)
                self.OneWireWriteByte(0xBE)  # Read Scratchpad command

                data = bytearray(9)
                for i in range(9):
                    byte = self.OneWireReadByte()
                    if byte is False:
                        raise RuntimeError("Failed to read scratchpad")
                    data[i] = byte

                # Calculate temperature
                raw = (data[1] << 8) | data[0]
                if raw & 0x8000:  # if negative number
                    raw -= 1 << 16
                celsius = raw / 16.0

                return celsius

            except Exception as e:
                raise RuntimeError("Temperature read failed: {}".format(e))
        else:
            return "Device attached is not a DS18B20."
