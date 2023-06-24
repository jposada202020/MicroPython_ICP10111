# SPDX-FileCopyrightText: Copyright (c) 2023 Jose D. Montoya
#
# SPDX-License-Identifier: MIT
"""
`icp10111`
================================================================================

MicroPython Driver for the TDK ICP-10111 Barometric Pressure and Temperature sensor


* Author(s): Jose D. Montoya


"""
import struct
import time

from micropython import const
from micropython_icp10111.i2c_helpers import CBits, RegisterStruct

try:
    from typing import Tuple, Union
except ImportError:
    pass


__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/jposada202020/MicroPython_ICP10111.git"


_DEVICE_ID = const(0xEFC8)
_SET_OTP = const(0xC59500669C)
_GET_VALUES = const(0xC7F7)

LOW_POWER = const(0x401A)
NORMAL = const(0x48A3)
LOW_NOISE = const(0x5059)
ULTRA_LOW_NOISE = const(0x58E0)
operation_mode_values = (LOW_POWER, NORMAL, LOW_NOISE, ULTRA_LOW_NOISE)


class ICP10111:
    """Driver for the ICP10111 Sensor connected over I2C.

    :param ~machine.I2C i2c: The I2C bus the ICP10111 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x63`

    :raises RuntimeError: if the sensor is not found

    **Quickstart: Importing and using the device**

    Here is an example of using the :class:`ICP10111` class.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        from machine import Pin, I2C
        from micropython_icp10111 import icp10111

    Once this is done you can define your `machine.I2C` object and define your sensor object

    .. code-block:: python

        i2c = I2C(1, sda=Pin(2), scl=Pin(3))
        icp10111 = icp10111.ICP10111(i2c)

    Now you have access to the attributes

    .. code-block:: python

    """

    def __init__(self, i2c, address: int = 0x63) -> None:
        self._i2c = i2c
        self._address = address

        if self._get_device_id() != 0x48:
            raise RuntimeError("Failed to find ICP10111")

        self.get_conversion_values()
        #  Conversion constant from the datasheet
        self.p_Pa_calib = [45000.0, 80000.0, 105000.0]
        self.LUT_lower = 3.5 * (2**20.0)
        self.LUT_upper = 11.5 * (2**20.0)
        self.quadr_factor = 1 / 16777216.0
        self.offst_factor = 2048.0

        self._mode = NORMAL

    def _get_device_id(self):
        """
        Get the device ID
        """
        data = bytearray(3)
        self._i2c.writeto(self._address, _DEVICE_ID.to_bytes(2, "big"), False)
        self._i2c.readfrom_into(self._address, data, True)
        return data[1]

    def reset(self):
        """
        Reset the sensor
        """
        self._i2c.writeto(self._address, bytes([0x80, 0x5D]), False)
        time.sleep(0.1)

    @staticmethod
    def _generate_crc(data: Union[bytearray, memoryview], initialization=0xFF) -> int:
        """
        8-bit CRC algorithm for checking data
        """

        crc = initialization

        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
            crc &= 0xFF
        return crc & 0xFF

    def get_conversion_values(self) -> None:
        self._sensor_constants = []

        data = bytearray(3)
        self._i2c.writeto(self._address, _SET_OTP.to_bytes(5, "big"), False)
        for i in range(4):
            self._i2c.writeto(self._address, _GET_VALUES.to_bytes(2, "big"), True)
            self._i2c.readfrom_into(self._address, data, True)

            if data[2] != self._generate_crc(memoryview(data[:2])):
                raise RuntimeError("OTP calibration reading did not get correct values")

            self._sensor_constants.append(struct.unpack("H", memoryview(data[:2]))[0])

        return self._sensor_constants

    def calculate_conversion_constants(self, p_Pa, p_LUT):
        """calculate temperature dependent constants
        Arguments:
        p_Pa -- List of 3 values corresponding to applied pressure in Pa
        p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.
        """

        C = (
            p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1])
            + p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2])
            + p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])
        ) / (
            p_LUT[2] * (p_Pa[0] - p_Pa[1])
            + p_LUT[0] * (p_Pa[1] - p_Pa[2])
            + p_LUT[1] * (p_Pa[2] - p_Pa[0])
        )
        A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (
            p_LUT[0] - p_LUT[1]
        )
        B = (p_Pa[0] - A) * (p_LUT[0] + C)
        return A, B, C

    def get_pressure(self, p_LSB, T_LSB):
        """Convert an output from a calibrated sensor to a pressure in Pa.

        p_LSB -- Raw pressure data from sensor
        T_LSB -- Raw temperature data from sensor
        """
        t = T_LSB - 32768.0
        s1 = (
            self.LUT_lower
            + float(self._sensor_constants[0] * t * t) * self.quadr_factor
        )
        s2 = (
            self.offst_factor * self._sensor_constants[3]
            + float(self._sensor_constants[1] * t * t) * self.quadr_factor
        )
        s3 = self.LUT_upper + float(self._sensor_constants[2] * t * t) * self.quadr_factor
        A, B, C = self.calculate_conversion_constants(self.p_Pa_calib, [s1, s2, s3])
        return A + B / (C + p_LSB)

    @property
    def measurements(self) -> float:
        data = bytearray(9)
        self._i2c.writeto(self._address, bytes([0xC7]), False)
        self._i2c.writeto(self._address, self._mode.to_bytes(2, "big"), False)
        time.sleep(0.03)
        self._i2c.readfrom_into(self._address, data, False)

        press_raw = data[0] << 16 | data[1] << 8 | data[3]
        temp_raw = data[6] << 8 | data[7]

        press = self.get_pressure(press_raw, temp_raw)
        temp = -45 + (175 / 2**16.0 * temp_raw)
        return press, temp

    @property
    def operation_mode(self) -> str:
        """
        Sensor operation_mode

        +--------------------------------------+--------------------+
        | Mode                                 | Value              |
        +======================================+====================+
        | :py:const:`icp10111.LOW_POWER`       | :py:const:`0x401A` |
        +--------------------------------------+--------------------+
        | :py:const:`icp10111.NORMAL`          | :py:const:`0x48A3` |
        +--------------------------------------+--------------------+
        | :py:const:`icp10111.LOW_NOISE`       | :py:const:`0x5059` |
        +--------------------------------------+--------------------+
        | :py:const:`icp10111.ULTRA_LOW_NOISE` | :py:const:`0x58E0` |
        +--------------------------------------+--------------------+
        """
        values = {0x401A: "LOW_POWER", 0x48A3: "NORMAL", 0x5059: "LOW_NOISE", 0x58E0:"ULTRA_LOW_NOISE"}
        return values[self._mode]

    @operation_mode.setter
    def operation_mode(self, value: int) -> None:
        if value not in operation_mode_values:
            raise ValueError("Value must be a valid operation_mode setting")
        self._mode = value

