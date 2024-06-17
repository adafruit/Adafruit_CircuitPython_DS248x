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
"""

import time
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

try:
    from typing import List  # pylint: disable=unused-import
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_DS248x.git"

# DS248x Command Definitions
_RESET = const(0xF0)
_READ_PTR = const(0xE1)
_WRITE_CONFIG = const(0xD2)
_1WIRE_RESET = const(0xB4)
_1WIRE_SINGLE_BIT = const(0x87)
_1WIRE_WRITE_BYTE = const(0xA5)
_1WIRE_READ_BYTE = const(0x96)
_TRIPLET = const(0x78)

# DS248x Register Definitions
_REG_STATUS = const(0xF0)
_REG_READ_DATA = const(0xE1)
_REG_CONFIG = const(0xC3)

# DS18B20 Commands
_DS18B20_FAMILY = const(0x28)
_DS18B20_T = const(0x44)
_DS18B20_ROM = const(0x55)
_DS18B20_SCRATCHPAD = const(0xBE)


class Adafruit_DS248x:
    """
    Driver for the DS2484 1-Wire to I2C Bus Adapter.
    """

    def __init__(self, i2c: I2C, address: int = 0x18):
        """
        Initialize the DS248x driver.

        :param i2c: The I2C bus object
        :param address: The I2C address of the DS248x device
        """
        try:
            self.i2c_device = I2CDevice(i2c, address)
            self._address = address
            self.rom_no: bytearray = bytearray(8)
            self.last_discrepancy: int = 0
            self.last_device_flag: bool = False
            self.last_family_discrepancy: int = 0
            self.reset()
            while not self.onewire_reset():
                print("Failed to do a 1W reset")
                if self.short_detected:
                    raise RuntimeError("\tShort detected")
                if not self.presence_pulse_detected:
                    raise RuntimeError("\tNo presence pulse")
                time.sleep(1)
        except RuntimeError as exception:
            raise RuntimeError("DS248x initialization failed.") from exception

    def reset(self) -> bool:
        """
        Reset the DS248x device.

        :return: True if the reset was successful, False otherwise
        """
        cmd = bytearray([_RESET])
        with self.i2c_device as i2c:
            i2c.write(cmd)
        status = self.status
        return (status != 0xFF) and (status & 0x10)

    def onewire_reset(self) -> bool:
        """
        Perform a 1-Wire reset.

        :return: True if the reset was successful, False otherwise
        """
        self._busy_wait(1000)
        cmd = bytearray([_1WIRE_RESET])
        with self.i2c_device as i2c:
            i2c.write(cmd)
        status = self.status
        return (
            (status != 0xFF)
            and not self.short_detected
            and self.presence_pulse_detected
        )

    @property
    def onewire_byte(self) -> int:
        """
        Byte to the 1-Wire bus.

        :return: The byte read from the 1-Wire bus
        """
        self._busy_wait(1000)
        cmd = bytearray([_1WIRE_READ_BYTE])
        with self.i2c_device as i2c:
            i2c.write(cmd)
        self._busy_wait(1000)
        self._set_read_pointer(_REG_READ_DATA)
        read_buffer = bytearray(1)
        with self.i2c_device as i2c:
            i2c.readinto(read_buffer)
        return read_buffer[0]

    @onewire_byte.setter
    def onewire_byte(self, byte: int) -> None:
        self._busy_wait(1000)
        cmd = bytearray([_1WIRE_WRITE_BYTE, byte])
        with self.i2c_device as i2c:
            i2c.write(cmd)
        self._busy_wait(1000)

    @property
    def onewire_bit(self) -> bool:
        """
        Bit to the 1-Wire bus.

        :param bit: The bit to write to the 1-Wire bus
        """
        self._busy_wait(1000)
        self.onewire_bit = 1
        self._busy_wait(1000)
        return self.single_bit_result

    @onewire_bit.setter
    def onewire_bit(self, bit: int) -> None:
        self._busy_wait(1000)
        cmd = bytearray([_1WIRE_SINGLE_BIT, 0x80 if bit else 0x00])
        with self.i2c_device as i2c:
            i2c.write(cmd)

    def onewire_search_reset(self) -> None:
        """
        Reset the 1-Wire search state.
        """
        self.last_discrepancy = 0
        self.last_device_flag = False
        self.last_family_discrepancy = 0

    # pylint: disable=too-many-branches
    def onewire_search(self, new_addr: List[int]) -> bool:
        """
        Perform a 1-Wire search to find devices on the bus.

        :param new_addr: The list to store the address of the found device
        :return: True if a device was found, False otherwise
        """
        search_result = False
        id_bit_number = 1
        last_zero = 0
        rom_byte_number = 0
        rom_byte_mask = 1
        id_bit, cmp_id_bit = 0, 0
        search_direction = 0

        if not self.last_device_flag:
            if not self.onewire_reset():
                self.last_discrepancy = 0
                self.last_device_flag = False
                self.last_family_discrepancy = 0
                return False

            self.onewire_byte = 0xF0

            while rom_byte_number < 8:
                id_bit = int(self.onewire_bit) & 0x01
                cmp_id_bit = int(self.onewire_bit) & 0x01

                if id_bit and cmp_id_bit:
                    break
                if id_bit != cmp_id_bit:
                    search_direction = id_bit
                else:
                    if id_bit_number < self.last_discrepancy:
                        search_direction = (
                            self.rom_no[rom_byte_number] & rom_byte_mask
                        ) > 0
                    else:
                        search_direction = id_bit_number == self.last_discrepancy
                    if not search_direction:
                        last_zero = id_bit_number
                        if last_zero < 9:
                            self.last_family_discrepancy = last_zero

                if search_direction:
                    self.rom_no[rom_byte_number] |= rom_byte_mask
                else:
                    self.rom_no[rom_byte_number] &= ~rom_byte_mask

                self.onewire_bit = search_direction

                id_bit_number += 1
                rom_byte_mask <<= 1

                if rom_byte_mask == 0x100:
                    rom_byte_number += 1
                    rom_byte_mask = 1

            if id_bit_number >= 65:
                self.last_discrepancy = last_zero
                if self.last_discrepancy == 0:
                    self.last_device_flag = True
                search_result = True

        if not search_result or not self.rom_no[0]:
            self.last_discrepancy = 0
            self.last_device_flag = False
            self.last_family_discrepancy = 0
            search_result = False

        for i in range(8):
            new_addr[i] = self.rom_no[i]
        return search_result

    @property
    def status(self) -> int:
        """
        Status of the DS248x device.

        :return: The status byte of the DS248x device
        """
        self._set_read_pointer(_REG_STATUS)
        status = bytearray(1)
        with self.i2c_device as i2c:
            i2c.readinto(status)
        return status[0]

    @property
    def config(self) -> int:
        """
        The configuration of the DS248x device.

        :return: The configuration byte of the DS248x device
        """
        self._set_read_pointer(_REG_CONFIG)
        config = bytearray(1)
        with self.i2c_device as i2c:
            i2c.readinto(config)
        return config[0]

    @config.setter
    def config(self, value: int) -> None:
        self._busy_wait(1000)
        config_value = (value & 0x0F) | ((~value & 0x0F) << 4)
        cmd = bytearray([_WRITE_CONFIG, config_value])
        with self.i2c_device as i2c:
            i2c.write(cmd)

    def _set_read_pointer(self, reg: int) -> None:
        """
        Set the read pointer to a specific register.

        :param reg: The register address to set the read pointer to
        """
        cmd = bytearray([_READ_PTR, reg])
        with self.i2c_device as i2c:
            i2c.write(cmd)

    def _busy_wait(self, timeout_ms: int) -> None:
        """
        Wait until the 1-Wire bus is not busy or until a timeout occurs.

        :param timeout_ms: The timeout in milliseconds
        :raises RuntimeError: If the bus is still busy after the timeout
        """
        start = time.monotonic()
        while (time.monotonic() - start) < timeout_ms / 1000:
            if not self.onewire_busy:
                return
            time.sleep(0.001)
        raise RuntimeError("Bus is busy after timeout")

    @property
    def onewire_busy(self) -> bool:
        """
        Check if the 1-Wire bus is busy.

        :return: True if the 1-Wire bus is busy, False otherwise
        """
        status = self.status
        return status != 0xFF and (status & 0x01)

    @property
    def presence_pulse_detected(self) -> bool:
        """
        Check if a presence pulse is detected on the 1-Wire bus.

        :return: True if a presence pulse is detected, False otherwise
        """
        status = self.status
        return status != 0xFF and (status & 0x02)

    @property
    def short_detected(self) -> bool:
        """
        Check if a short circuit is detected on the 1-Wire bus.

        :return: True if a short circuit is detected, False otherwise
        """
        status = self.status
        return status != 0xFF and (status & 0x04)

    @property
    def logic_level(self) -> bool:
        """
        Check the logic level of the 1-Wire bus.

        :return: True if the logic level is high, False if it is low
        """
        status = self.status
        return status != 0xFF and (status & 0x08)

    @property
    def single_bit_result(self) -> bool:
        """
        Get the result of the last single bit operation on the 1-Wire bus.

        :return: True if the result bit is 1, False if it is 0
        """
        status = self.status
        return bool(status & 0x20)

    @property
    def triplet_second_bit(self) -> bool:
        """
        Get the second bit of the last triplet operation on the 1-Wire bus.

        :return: True if the second bit is 1, False if it is 0
        """
        status = self.status
        return status != 0xFF and (status & 0x40)

    @property
    def branch_dir_taken(self) -> bool:
        """
        Check if the branch direction was taken during the last triplet operation on the 1-Wire bus.

        :return: True if the branch direction was taken, False otherwise
        """
        status = self.status
        return status != 0xFF and (status & 0x80)

    def ds18b20_temperature(self, rom: bytearray) -> float:
        """
        Reads the temperature from a DS18B20 sensor.

        :param rom: The ROM address of the DS18B20 sensor
        :return: The temperature in Celsius
        """
        if rom[0] != _DS18B20_FAMILY:
            raise ValueError("Device attached is not a DS18B20")

        self.onewire_reset()
        self.onewire_byte = 0x55  # Match ROM command
        for byte in rom:
            self.onewire_byte = byte

        self.onewire_byte = 0x44  # Convert T command
        time.sleep(0.75)

        self.onewire_reset()
        self.onewire_byte = 0x55
        for byte in rom:
            self.onewire_byte = byte
        self.onewire_byte = 0xBE  # Read Scratchpad command

        data = bytearray(9)
        for i in range(9):
            data[i] = self.onewire_byte

        raw = (data[1] << 8) | data[0]
        if raw & 0x8000:
            raw -= 1 << 16
        celsius = raw / 16.0

        return celsius
