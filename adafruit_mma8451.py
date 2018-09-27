# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_mma8451`
====================================================

CircuitPython module for the MMA8451 3 axis accelerometer.  See
examples/simpletest.py for a demo of the usage.

* Author(s): Tony DiCola
"""
import time

try:
    import struct
except ImportError:
    import ustruct as struct

from micropython import const

import adafruit_bus_device.i2c_device as i2c_device

import RPi.GPIO as GPIO

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MMA8451.git"


#pylint: disable=bad-whitespace
# Internal constants:
_MMA8451_DEFAULT_ADDRESS   = const(0x1D)
_MMA8451_REG_F_STATUS      = const(0x00)
_MMA8451_REG_OUT_X_MSB     = const(0x01)
_MMA8451_REG_FIFO_SETUP    = const(0x09);
_MMA8451_REG_SYSMOD        = const(0x0B)
_MMA8451_REG_WHOAMI        = const(0x0D)
_MMA8451_REG_XYZ_DATA_CFG  = const(0x0E)
_MMA8451_REG_PL_STATUS     = const(0x10)
_MMA8451_REG_PL_CFG        = const(0x11)
_MMA8451_REG_CTRL_REG1     = const(0x2A)
_MMA8451_REG_CTRL_REG2     = const(0x2B)
_MMA8451_REG_CTRL_REG4     = const(0x2D)
_MMA8451_REG_CTRL_REG5     = const(0x2E)
_MMA8451_DATARATE_MASK     = const(0b111)
_SENSORS_GRAVITY_EARTH     = 9.80665

# External user-facing constants:
PL_PUF           = 0      # Portrait, up, front
PL_PUB           = 1      # Portrait, up, back
PL_PDF           = 2      # Portrait, down, front
PL_PDB           = 3      # Portrait, down, back
PL_LRF           = 4      # Landscape, right, front
PL_LRB           = 5      # Landscape, right, back
PL_LLF           = 6      # Landscape, left, front
PL_LLB           = 7      # Landscape, left, back
RANGE_8G         = 0b10   # +/- 8g
RANGE_4G         = 0b01   # +/- 4g (default value)
RANGE_2G         = 0b00   # +/- 2g
DATARATE_800HZ   = 0b000  #  800Hz
DATARATE_400HZ   = 0b001  #  400Hz
DATARATE_200HZ   = 0b010  #  200Hz
DATARATE_100HZ   = 0b011  #  100Hz
DATARATE_50HZ    = 0b100  #   50Hz
DATARATE_12_5HZ  = 0b101  # 12.5Hz
DATARATE_6_25HZ  = 0b110  # 6.25Hz
DATARATE_1_56HZ  = 0b111  # 1.56Hz

FIFO_MODE_CIRCULAR = 0b01000000
FIFO_MODE_FILL     = 0b10000000
FIFO_MODE_TRIGGER  = 0b11000000
INT_CFG_FIFO  = 0b01000000
INT_EN_FIFO  = 0b01000000
#pylint: enable=bad-whitespace


class MMA8451:
    """MMA8451 accelerometer.  Create an instance by specifying:
    - i2c: The I2C bus connected to the sensor.

    Optionally specify:
    - address: The I2C address of the sensor if not the default of 0x1D.
    """

    # Class-level buffer to reduce allocations and fragmentation.
    # Note this is NOT thread-safe or re-entrant by design!
    _BUFFER = bytearray(6)

    _FIFO_BUFFER = bytearray(192)

    def __init__(self, i2c, *, address=_MMA8451_DEFAULT_ADDRESS):
        self._device = i2c_device.I2CDevice(i2c, address)
        # Verify device ID.
        if self._read_u8(_MMA8451_REG_WHOAMI) != 0x1A:
            raise RuntimeError('Failed to find MMA8451, check wiring!')
        # Reset and wait for chip to be ready.
        self._write_u8(_MMA8451_REG_CTRL_REG2, 0x40)
        time.sleep(0.01)
        while self._read_u8(_MMA8451_REG_CTRL_REG2) & 0x40 > 0:
            pass
        # Enable 4G range.
        self._write_u8(_MMA8451_REG_XYZ_DATA_CFG, RANGE_4G)
        # High resolution mode.
        self._write_u8(_MMA8451_REG_CTRL_REG2, 0x02)
        # DRDY on INT1
        self._write_u8(_MMA8451_REG_CTRL_REG4, 0x01)
        self._write_u8(_MMA8451_REG_CTRL_REG5, 0x01)
        # Turn on orientation config
        self._write_u8(_MMA8451_REG_PL_CFG, 0x40)
        # Activate at max rate, low noise mode
        self._write_u8(_MMA8451_REG_CTRL_REG1, 0x01 | 0x04)

    def _read_into(self, address, buf, count=None):
        # Read bytes from the specified address into the provided buffer.
        # If count is not specified (the default) the entire buffer is filled,
        # otherwise only count bytes are copied in.
        # It's silly that pylint complains about an explicit check that buf
        # has at least 1 value.  I don't trust the implicit true/false
        # recommendation as it was not designed for bytearrays which may not
        # follow that semantic.  Ignore pylint's superfulous complaint.
        assert len(buf) > 0  #pylint: disable=len-as-condition
        assert count is None or count <= len(buf)
        if count is None:
            count = len(buf)
        #print("_read_into add={0:d} buf len={1:d} count={2:d}".format(address, len(buf), count))
        with self._device as i2c:
            i2c.write_then_readinto(bytes([address & 0xFF]), buf,
                                    in_end=count, stop=False)

    def _read_u8(self, address):
        # Read an 8-bit unsigned value from the specified 8-bit address.
        self._read_into(address, self._BUFFER, count=1)
        return self._BUFFER[0]

    def _write_u8(self, address, val):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)

    @property
    def range(self):
        """Get and set the range of the sensor.  Must be a value of:
         - RANGE_8G: +/- 8g
         - RANGE_4G: +/- 4g (the default)
         - RANGE_2G: +/- 2g
        """
        return self._read_u8(_MMA8451_REG_XYZ_DATA_CFG) & 0x03

    @range.setter
    def range(self, val):
        assert 0 <= val <= 2
        reg1 = self._read_u8(_MMA8451_REG_CTRL_REG1)
        self._write_u8(_MMA8451_REG_CTRL_REG1, 0x00)        # deactivate
        self._write_u8(_MMA8451_REG_XYZ_DATA_CFG, val)
        self._write_u8(_MMA8451_REG_CTRL_REG1, reg1 | 0x01) # activate

    @property
    def data_rate(self):
        """Get and set the data rate of the sensor.  Must be a value of:
         - DATARATE_800HZ:   800Hz (the default)
         - DATARATE_400HZ:   400Hz
         - DATARATE_200HZ:   200Hz
         - DATARATE_100HZ:   100Hz
         - DATARATE_50HZ:     50Hz
         - DATARATE_12_5HZ: 12.5Hz
         - DATARATE_6_25HZ: 6.25Hz
         - DATARATE_1_56HZ: 1.56Hz
        """
        return (self._read_u8(_MMA8451_REG_CTRL_REG1) >> 3) & \
               _MMA8451_DATARATE_MASK

    @data_rate.setter
    def data_rate(self, val):
        assert 0 <= val <= 7
        ctl1 = self._read_u8(_MMA8451_REG_CTRL_REG1)
        self._write_u8(_MMA8451_REG_CTRL_REG1, 0x00)         # deactivate
        ctl1 &= ~(_MMA8451_DATARATE_MASK << 3)              # mask off bits
        ctl1 |= (val << 3)
        self._write_u8(_MMA8451_REG_CTRL_REG1, ctl1 | 0x01) # activate

    def reset(self):
        reg2 = self._read_u8(_MMA8451_REG_CTRL_REG2)
        # reset bit, keep all others same
        self._write_u8(_MMA8451_REG_CTRL_REG2, reg2 & 0x40)
        while self._read_u8(_MMA8451_REG_CTRL_REG2) & 0x40 > 0:
            pass

    def enableFifoBuffer(self, watermark, pin, callback):
        assert 0 < watermark <= 32
        self._fifoCallback = callback
        # enable interupt
        self.reset()
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.FALLING, callback=self._internalFifoCallback, bouncetime=70)

        self._write_u8(_MMA8451_REG_CTRL_REG1, 0x00) # deactivate
        self._write_u8(_MMA8451_REG_FIFO_SETUP, FIFO_MODE_CIRCULAR | watermark)
        self._write_u8(_MMA8451_REG_CTRL_REG4, INT_EN_FIFO)
        self._write_u8(_MMA8451_REG_CTRL_REG5, INT_CFG_FIFO)
        reg1 = self._read_u8(_MMA8451_REG_CTRL_REG1)
        self._write_u8(_MMA8451_REG_CTRL_REG1, reg1 | 0x01)
        print("FIFO mode enabled status: {0:b} fifo: {1:b}".format(self._read_u8(_MMA8451_REG_F_STATUS), self._read_u8(_MMA8451_REG_FIFO_SETUP)))

    def _internalFifoCallback(self, channel):
        status, samplesAvail, data = self.dataBuffer()
        self._fifoCallback(status, samplesAvail, data)

    @property
    def fifoStatus(self):
        return self._read_u8(_MMA8451_REG_F_STATUS)

    def dataBuffer(self, samples=32):
        assert 0 < samples <= 32
        status = self._read_u8(_MMA8451_REG_F_STATUS)
        statusRedo = self._read_u8(_MMA8451_REG_F_STATUS)
        #print("dataBuffer status: {0:b}  {1:b}".format(status, statusRedo))
        samplesAvail = status & 0x3F
        if samplesAvail > 32:
            samplesAvail = 32
        #print("sampleAvail: {0:d} {0:b} & {1:b}".format(samplesAvail, 0x3F))
        numBytes = samplesAvail*2*3
        bytesRead = self._read_into(_MMA8451_REG_OUT_X_MSB, self._FIFO_BUFFER, count=numBytes)
        return status, samplesAvail, self._FIFO_BUFFER.copy()[0:numBytes]


    @property
    def configuration(self):
        out = []
        out.append(self._read_u8(_MMA8451_REG_F_STATUS))
        out.append(self._read_u8(_MMA8451_REG_CTRL_REG1))
        out.append(self._read_u8(_MMA8451_REG_CTRL_REG2))
        out.append(self._read_u8(_MMA8451_REG_CTRL_REG3))
        out.append(self._read_u8(_MMA8451_REG_CTRL_REG4))
        out.append(self._read_u8(_MMA8451_REG_CTRL_REG5))
        return out

    def demux(self, data):
        count=0
        outX = []
        outY = []
        outZ = []
        sixBytes = data[0:6]
        while count < len(data):
            x, y, z = struct.unpack('>hhh', sixBytes)
            outX.append(x >> 2)
            outY.append(y >> 2)
            outZ.append(z >> 2)
            count += 6
            sixBytes = data[count:count+6]
        return outX, outY, outZ

    @property
    def acceleration(self):
        # pylint: disable=no-else-return
        # This needs to be refactored when it can be tested
        """Get the acceleration measured by the sensor.  Will return a 3-tuple
        of X, Y, Z axis acceleration values in m/s^2.
        """
        # Read 6 bytes for 16-bit X, Y, Z values.
        self._read_into(_MMA8451_REG_OUT_X_MSB, self._BUFFER, count=6)
        # Reconstruct signed 16-bit integers.
        x, y, z = struct.unpack('>hhh', self._BUFFER)
        x >>= 2
        y >>= 2
        z >>= 2
        # Scale values based on current sensor range to get proper units.
        _range = self.range
        if _range == RANGE_8G:
            return (x/1024.0*_SENSORS_GRAVITY_EARTH,
                    y/1024.0*_SENSORS_GRAVITY_EARTH,
                    z/1024.0*_SENSORS_GRAVITY_EARTH)
        elif _range == RANGE_4G:
            return (x/2048.0*_SENSORS_GRAVITY_EARTH,
                    y/2048.0*_SENSORS_GRAVITY_EARTH,
                    z/2048.0*_SENSORS_GRAVITY_EARTH)
        elif _range == RANGE_2G:
            return (x/4096.0*_SENSORS_GRAVITY_EARTH,
                    y/4096.0*_SENSORS_GRAVITY_EARTH,
                    z/4096.0*_SENSORS_GRAVITY_EARTH)
        else:
            raise RuntimeError('Unexpected range!')

    @property
    def orientation(self):
        """Get the orientation of the MMA8451.  Will return a value of:
         - PL_PUF: Portrait, up, front
         - PL_PUB: Portrait, up, back
         - PL_PDF: Portrait, down, front
         - PL_PDB: Portrait, down, back
         - PL_LRF: Landscape, right, front
         - PL_LRB: Landscape, right, back
         - PL_LLF: Landscape, left, front
         - PL_LLB: Landscape, left, back
        """
        return self._read_u8(_MMA8451_REG_PL_STATUS) & 0x07
