# SPDX-FileCopyrightText: 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_icm20x`
================================================================================

Library for the ST ICM20X Motion Sensor Family

* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* Adafruit's ICM20649 Breakout: https://adafruit.com/product/4464
* Adafruit's ICM20948 Breakout: https://adafruit.com/product/4554

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads


* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ICM20X.git"
# Common imports; remove if unused or pylint will complain
from time import sleep
from adafruit_bus_device import i2c_device

from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct, Struct
from adafruit_register.i2c_bit import RWBit, ROBit
from adafruit_register.i2c_bits import RWBits

_IIM42652_DEFAULT_ADDRESS = 0x68
_IIM42652_DEVICE_ID = 0x6F

# Registers are set up in banks
# Functions using these bank-specific registers are responsible for ensuring
# that the correct bank is set
# Bank 0
_IIM42652_DEVICE_CONFIG      = 0x11
_IIM42652_DRIVE_CONFIG       = 0x13
_IIM42652_INT_CONFIG         = 0x14
_IIM42652_FIFO_CONFIG        = 0x16
_IIM42652_TEMP_DATA1_UI      = 0x1D
_IIM42652_TEMP_DATA0_UI      = 0x1E
_IIM42652_ACCEL_DATA_X1_UI   = 0x1F
_IIM42652_ACCEL_DATA_X0_UI   = 0x20
_IIM42652_ACCEL_DATA_Y1_UI   = 0x21
_IIM42652_ACCEL_DATA_Y0_UI   = 0x22
_IIM42652_ACCEL_DATA_Z1_UI   = 0x23
_IIM42652_ACCEL_DATA_Z0_UI   = 0x24
_IIM42652_GYRO_DATA_X1_UI    = 0x25
_IIM42652_GYRO_DATA_X0_UI    = 0x26
_IIM42652_GYRO_DATA_Y1_UI    = 0x27
_IIM42652_GYRO_DATA_Y0_UI    = 0x28
_IIM42652_GYRO_DATA_Z1_UI    = 0x29
_IIM42652_GYRO_DATA_Z0_UI    = 0x2A
_IIM42652_TMST_FSYNCH        = 0x2B
_IIM42652_TMST_FSYNCL        = 0x2C
_IIM42652_INT_STATUS         = 0x2D
_IIM42652_FIFO_COUNTH        = 0x2E
_IIM42652_FIFO_COUNTL        = 0x2F
_IIM42652_FIFO_DATA          = 0x30
_IIM42652_APEX_DATA0         = 0x31
_IIM42652_APEX_DATA1         = 0x32
_IIM42652_APEX_DATA2         = 0x33
_IIM42652_APEX_DATA3         = 0x34
_IIM42652_APEX_DATA4         = 0x35
_IIM42652_APEX_DATA5         = 0x36
_IIM42652_INT_STATUS2        = 0x37
_IIM42652_INT_STATUS3        = 0x38
_IIM42652_SIGNAL_PATH_RESET  = 0x4B
_IIM42652_INTF_CONFIG0       = 0x4C
_IIM42652_INTF_CONFIG1       = 0x4D
_IIM42652_PWR_MGMT0          = 0x4E
_IIM42652_GYRO_CONFIG0       = 0x4F
_IIM42652_ACCEL_CONFIG0      = 0x50
_IIM42652_GYRO_CONFIG1       = 0x51
_IIM42652_GYRO_ACCEL_CONFIG0 = 0x52
_IIM42652_ACCEL_CONFIG1      = 0x53
_IIM42652_TMST_CONFIG        = 0x54
_IIM42652_APEX_CONFIG0       = 0x56
_IIM42652_SMD_CONFIG         = 0x57
_IIM42652_FIFO_CONFIG1       = 0x5F
_IIM42652_FIFO_CONFIG2       = 0x60
_IIM42652_FIFO_CONFIG3       = 0x61
_IIM42652_FSYNC_CONFIG       = 0x62
_IIM42652_INT_CONFIG0        = 0x63
_IIM42652_INT_CONFIG1        = 0x64
_IIM42652_INT_SOURCE0        = 0x65
_IIM42652_INT_SOURCE1        = 0x66
_IIM42652_INT_SOURCE3        = 0x68
_IIM42652_INT_SOURCE4        = 0x69
_IIM42652_FIFO_LOST_PKT0     = 0x6C
_IIM42652_FIFO_LOST_PKT1     = 0x6D
_IIM42652_SELF_TEST_CONFIG   = 0x70
_IIM42652_WHO_AM_I           = 0x75
_IIM42652_REG_BANK_SEL       = 0x76

# Bank 1
_IIM42652_SENSOR_CONFIG0       = 0x03
_IIM42652_GYRO_CONFIG_STATIC2  = 0x0B
_IIM42652_GYRO_CONFIG_STATIC3  = 0x0C
_IIM42652_GYRO_CONFIG_STATIC4  = 0x0D
_IIM42652_GYRO_CONFIG_STATIC5  = 0x0E
_IIM42652_GYRO_CONFIG_STATIC6  = 0x0F
_IIM42652_GYRO_CONFIG_STATIC7  = 0x10
_IIM42652_GYRO_CONFIG_STATIC8  = 0x11
_IIM42652_GYRO_CONFIG_STATIC9  = 0x12
_IIM42652_GYRO_CONFIG_STATIC10 = 0x13
_IIM42652_XG_ST_DATA           = 0x5F
_IIM42652_YG_ST_DATA           = 0x60
_IIM42652_ZG_ST_DATA           = 0x61
_IIM42652_TMSTVAL0             = 0x62
_IIM42652_TMSTVAL1             = 0x63
_IIM42652_TMSTVAL2             = 0x64
_IIM42652_INTF_CONFIG4         = 0x7A
_IIM42652_INTF_CONFIG5         = 0x7B
_IIM42652_INTF_CONFIG6         = 0x7C

# Bank 2
_IIM42652_ACCEL_CONFIG_STATIC2 = 0x03
_IIM42652_ACCEL_CONFIG_STATIC3 = 0x04
_IIM42652_ACCEL_CONFIG_STATIC4 = 0x05
_IIM42652_XA_ST_DATA           = 0x3B
_IIM42652_YA_ST_DATA           = 0x3C
_IIM42652_ZA_ST_DATA           = 0x3D

# Bank 3
_IIM42652_PU_PD_CONFIG1 = 0x06
_IIM42652_PU_PD_CONFIG2 = 0x0E

# Bank 4
_IIM42652_FDR_CONFIG      = 0x09
_IIM42652_APEX_CONFIG1    = 0x40
_IIM42652_APEX_CONFIG2    = 0x41
_IIM42652_APEX_CONFIG3    = 0x42
_IIM42652_APEX_CONFIG4    = 0x43
_IIM42652_APEX_CONFIG5    = 0x44
_IIM42652_APEX_CONFIG6    = 0x45
_IIM42652_APEX_CONFIG7    = 0x46
_IIM42652_APEX_CONFIG8    = 0x47
_IIM42652_APEX_CONFIG9    = 0x48
_IIM42652_APEX_CONFIG10   = 0x49
_IIM42652_ACCEL_WOM_X_THR = 0x4A
_IIM42652_ACCEL_WOM_Y_THR = 0x4B
_IIM42652_ACCEL_WOM_Z_THR = 0x4C
_IIM42652_INT_SOURCE6     = 0x4D
_IIM42652_INT_SOURCE7     = 0x4E
_IIM42652_INT_SOURCE8     = 0x4F
_IIM42652_INT_SOURCE9     = 0x50
_IIM42652_INT_SOURCE10    = 0x51
_IIM42652_OFFSET_USER0    = 0x77
_IIM42652_OFFSET_USER1    = 0x78
_IIM42652_OFFSET_USER2    = 0x79
_IIM42652_OFFSET_USER3    = 0x7A
_IIM42652_OFFSET_USER4    = 0x7B
_IIM42652_OFFSET_USER5    = 0x7C
_IIM42652_OFFSET_USER6    = 0x7D
_IIM42652_OFFSET_USER7    = 0x7E
_IIM42652_OFFSET_USER8    = 0x7F

ACCEL_LSB_PER_G_DEF = 2048.0
GYRO_LSB_PER_DEGPSEC_DEF = 16.4

# temp = (REGDATA/132.48) + 25
TEMP_LSB_PER_DEGC = 132.48 
TEMP_OFFSET = 25

class IIM42652:  # pylint:disable=too-many-instance-attributes
    """Library for the IIM-42652 6-DoF Accelerometer and Gyro Family


    :param ~busio.I2C i2c_bus: The I2C bus the IIM42652 is connected to.
    :param int address: The I2C address of the device.

    """

    _bank_reg = UnaryStruct(_IIM42652_REG_BANK_SEL, "B")
    
    # Bank 0
    _device_id = ROUnaryStruct(_IIM42652_WHO_AM_I, "B")
    _reset = RWBit(_IIM42652_DEVICE_CONFIG, 0)

    _pwr_mgmt0 = UnaryStruct(_IIM42652_PWR_MGMT0, "B")
    _pwr_mgmt0_tmp_dis = RWBit(_IIM42652_PWR_MGMT0, 5)
    _pwr_mgmt0_idle = RWBit(_IIM42652_PWR_MGMT0, 4)
    _pwr_mgmt0_gyro_mode = RWBits(2, _IIM42652_PWR_MGMT0, 2)
    _pwr_mgmt0_accel_mode = RWBits(2, _IIM42652_PWR_MGMT0, 0)

    _raw_temp_data = Struct(_IIM42652_TEMP_DATA1_UI, ">h")
    _raw_accel_data = Struct(_IIM42652_ACCEL_DATA_X1_UI, ">hhh")
    _raw_gyro_data = Struct(_IIM42652_GYRO_DATA_X1_UI, ">hhh")


    @property
    def _bank(self):
        return self._bank_reg

    @_bank.setter
    def _bank(self, value):
        self._bank_reg = value

    def __init__(self, i2c_bus, address=_IIM42652_DEFAULT_ADDRESS):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._bank = 0
        if not self._device_id in [_IIM42652_DEVICE_ID]:
            raise RuntimeError("Failed to find an IIM42652 sensor - check your wiring!")
        self.reset()
        self.initialize()

    def initialize(self):
        """Configure the sensors with the default settings."""

        self._bank = 0
        self._pwr_mgmt0 = 0x0F  # turn on the gyro and accelerometer to low noise mode
        sleep(0.045)  # minimum on time for gyro

    def reset(self):
        """Resets the internal registers and restores the default settings"""
        self._bank = 0

        sleep(0.005)
        self._reset = True
        sleep(0.005)
        while self._reset:
            sleep(0.005)

    @property
    def temperature(self):
        """The temperature value is in degrees C.`"""
        self._bank = 0
        raw_temp_data = self._raw_temp_data
        sleep(0.005)

        temp = (raw_temp_data / TEMP_LSB_PER_DEGC) + TEMP_OFFSET

        return temp

    @property
    def acceleration(self):
        """The x, y, z acceleration values returned in a 3-tuple and are in g.`"""
        self._bank = 0
        raw_accel_data = self._raw_accel_data
        sleep(0.005)

        x = self._scale_xl_data(raw_accel_data[0])
        y = self._scale_xl_data(raw_accel_data[1])
        z = self._scale_xl_data(raw_accel_data[2])

        return (x, y, z)

    @property
    def gyro(self):
        """The x, y, z angular velocity values returned in a 3-tuple and
        are in degrees per second"""
        self._bank = 0
        raw_gyro_data = self._raw_gyro_data
        sleep(0.005)
        x = self._scale_gyro_data(raw_gyro_data[0])
        y = self._scale_gyro_data(raw_gyro_data[1])
        z = self._scale_gyro_data(raw_gyro_data[2])

        return (x, y, z)

    def _scale_xl_data(self, raw_measurement):
        return raw_measurement / ACCEL_LSB_PER_G_DEF

    def _scale_gyro_data(self, raw_measurement):
        return raw_measurement / GYRO_LSB_PER_DEGPSEC_DEF
