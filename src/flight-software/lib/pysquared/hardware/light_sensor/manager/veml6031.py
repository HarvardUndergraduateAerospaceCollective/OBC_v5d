"""
Python library for the VEML6030 sensor.
Datasheet:
https://www.vishay.com/docs/80007/veml6031x00.pdf

Based on:
https://github.com/n8many/VEML6030py/blob/master/veml6030.py
"""

from busio import I2C
from enum import IntEnum, IntFlag
from adafruit_tca9548a import TCA9548A_Channel

# done: VEML6031X00
ADDRESS = [0x29, 0x10]

# done: Page 8
SETTING_CONF_0              = 0x00  # ALS_CONF_0
SETTING_CONF_1              = 0x01  # ALS_CONF_1
AMBIENT_LIGHT_DATA_L_REG    = 0x10  # ALS_DATA_L
AMBIENT_LIGHT_DATA_H_REG    = 0x11  # ALS_DATA_H
INTERRUPT_REG               = 0x06  # ALS_INT

# done: ALS_GAIN
class Gain(IntEnum):
    x1      = 0b00      # ALS gain x1
    x2      = 0b01      # ALS gain x2
    x2_3    = 0b10      # ALS gain x0.66
    x1_2    = 0b11      # ALS gain x0.5

# done: ALS_IT
class IntegrationTime(IntEnum):
    ms3_125     = 0b000
    ms6_25      = 0b001
    ms12_5      = 0b010
    ms25        = 0b011
    ms50        = 0b100
    ms100       = 0b101
    ms200       = 0b110
    ms400       = 0b111

# done: ALS_PERS
class ProtectNum(IntEnum):
    n1 = 0b00
    n2 = 0b01
    n4 = 0b10
    n8 = 0b11

# done: ALS_INT_EN
class Interrupt(IntFlag):
    Low     = 0b0
    High    = 0b1

# done: Table 9
_lux_coeff_4_4 = {
    IntegrationTime.ms400: {
        Gain.x2     : .0034,
        Gain.x1     : .0068,
        Gain.x2_3   : .0103,
        Gain.x1_2   : .0136
    },
    IntegrationTime.ms200: {
        Gain.x2     : .0068,
        Gain.x1     : .0136,
        Gain.x2_3   : .0206,
        Gain.x1_2   : .0272
    },
    IntegrationTime.ms100: {
        Gain.x2     : .0136,
        Gain.x1     : .0272,
        Gain.x2_3   : .0412,
        Gain.x1_2   : .0544
    },
    IntegrationTime.ms50: {
        Gain.x2     : .0272,
        Gain.x1     : .0544,
        Gain.x2_3   : .0824,
        Gain.x1_2   : .1088
    },
    IntegrationTime.ms25: {
        Gain.x2     : .0544,
        Gain.x1     : .1088,
        Gain.x2_3   : .1648,
        Gain.x1_2   : .2176
    },
    IntegrationTime.ms12_5: {
        Gain.x2     : .1088,
        Gain.x1     : .2176,
        Gain.x2_3   : .3297,
        Gain.x1_2   : .4352
    },
    IntegrationTime.ms6_25: {
        Gain.x2     : .2176,
        Gain.x1     : .4352,
        Gain.x2_3   : .6594,
        Gain.x1_2   : .8704
    },
    IntegrationTime.ms3_125: {
        Gain.x2     : .4352,
        Gain.x1     : .8704,
        Gain.x2_3   : 1.3188,
        Gain.x1_2   : 1.7408
    },
}

# done: Table 10
_lux_coeff_1_4 = {
    IntegrationTime.ms400: {
        Gain.x2     : .0136,
        Gain.x1     : .0272,
        Gain.x2_3   : .0412,
        Gain.x1_2   : .0544
    },
    IntegrationTime.ms200: {
        Gain.x2     : .0272,
        Gain.x1     : .0544,
        Gain.x2_3   : .0824,
        Gain.x1_2   : .1088
    },
    IntegrationTime.ms100: {
        Gain.x2     : .0544,
        Gain.x1     : .1088,
        Gain.x2_3   : .1648,
        Gain.x1_2   : .2176
    },
    IntegrationTime.ms50: {
        Gain.x2     : 0.1088,
        Gain.x1     : 0.2176,
        Gain.x2_3   : 0.3297,
        Gain.x1_2   : 0.4352
    },
    IntegrationTime.ms25: {
        Gain.x2     : 0.2176,
        Gain.x1     : 0.4352,
        Gain.x2_3   : 0.6594,
        Gain.x1_2   : 0.8704
    },
    IntegrationTime.ms12_5: {
        Gain.x2     : 0.4352,
        Gain.x1     : 0.8704,
        Gain.x2_3   : 1.3188,
        Gain.x1_2   : 1.7408
    },
    IntegrationTime.ms6_25: {
        Gain.x2     : 0.8704,
        Gain.x1     : 1.7408,
        Gain.x2_3   : 2.6376,
        Gain.x1_2   : 3.4816
    },
    IntegrationTime.ms3_125: {
        Gain.x2     : 1.7408,
        Gain.x1     : 3.4816,
        Gain.x2_3   : 5.2752,
        Gain.x1_2   : 6.9632
    },
}

def set_bits(register: int, value, index, length=1):
    """
    Set selected bits in register and return new value

    :param register: Input register value
    :type register: int
    :param value: Bits to write to register
    :type value: int
    :param index: Start index (from right)
    :type index: int
    :param length: Number of bits (default 1)
    :type length: int
    :return: Output new register value
    :rtype: int
    """
    mask = (1 << length)-1
    register = register & ~(mask << index)
    register = register | (mask & value) << index
    return register


def get_bits(register, index, length=1):
    """
    Get selected bit(s) from register while masking out the rest.
    Returns as boolean if length==1

    :param register: Register value
    :type register: int
    :param index: Start index (from right)
    :type index: int
    :param length: Number of bits (default 1)
    :type length: int
    :return: Selected bit(s)
    :rtype: Union[int, bool]
    """
    result = (register >> index) & ((1 << length) - 1)
    if length == 1:
        return result == 1
    return result


MAX_LUX = 120000
MIN_LUX = 0


class VEML6031(object):

    def __init__(self, bus, address=ADDRESS[0]):
        """
        Object for connecting to VEML6030 sensor via i2c.

        :param bus: i2c bus to connect to VEML6030 sensor. Must be compatible with smbus.SMBus object
        :type bus: smbus.SMBus
        :param address: i2c address of VEML6030 sensor
        :type address: int
        """

        # Must be one of two valid addresses for this board
        if address not in ADDRESS:
            raise ValueError("Invalid Address: {0:#x}".format(address))
        self.address = address

        # Make sure the bus input is valid
        if bus is None:
            raise ValueError("Invalid bus, must pass in SMBus object")
        self.bus : I2C | TCA9548A_Channel = bus

        # Initialize sensor with default values
        self.power_on()
        self.set_gain(Gain.x1_2) # lowest gain
        self.set_integration_time(IntegrationTime.ms50)

    def set_gain(self, gain):
        """
        Set gain for VEML6030

        :param gain:
        :type gain: Gain
        :return:
        """
        self._write_bits_to_register(SETTING_CONF_1, gain, 3, 1) # Table 2

    def get_gain(self):
        """
        Get gain for VEML6030

        :return: Set gain value
        :rtype: Gain
        """
        return Gain(self._read_bits_from_register(SETTING_CONF_1, 3, 1)) # Table 2

    def set_integration_time(self, time):
        """
        Set integration time for VEML6030

        :param time: Integration Time setting
        :type time: IntegrationTime
        """
        self._write_bits_to_register(SETTING_CONF_0, time, 4, 3) # Table 1

    def get_integration_time(self):
        """
        Get set integration time for VEML6030

        :return: Integration Time setting
        :rtype: IntegrationTime
        """
        return IntegrationTime(self._read_bits_from_register(SETTING_CONF_0, 4, 3)) # Table 1

    def set_pd_div(self, pd_div):
        """
        Set pd_div for PD_DIV4

        :param pd_div: PD_DIV4 setting (0 = 4/4, 1 = 1/4)
        :type pd_div: int
        """
        self._write_bits_to_register(SETTING_CONF_1, pd_div, 6, 1) # Table 2

    def get_pd_div(self):
        """
        Get pd_div for VEML6031

        :return: PD_DIV4 setting (0 = 4/4, 1 = 1/4)
        :rtype: int
        """
        return int(self._read_bits_from_register(SETTING_CONF_1, 6, 1)) # Table 2

    def shutdown(self):
        """
        Powers off VEML6030.
        The current value will be saved for reading.
        No further readings will be taken until "power_on" is called.
        """
        self.write_both_conf_registers(True)
    
    def power_on(self):
        """
        Powers on VEML6030 from off
        """
        self.write_both_conf_registers(False)

    def read_light(self, compensate=True):
        """
        Reads ambient light reading from VEML6030 in lux

        :param compensate: Whether to compensate lux values over 1000.
        Only set to false if gathering values for thresholds
        :type compensate: bool
        :return: Ambient light read by sensor
        :rtype: float
        """
        count_l = self._read_register(AMBIENT_LIGHT_DATA_L_REG)
        count_h = self._read_register(AMBIENT_LIGHT_DATA_H_REG)
        count = (count_h << 8) | (count_l & 0xFF)
        lux = self._calculate_lux(count)
        # Compensate lux if high enough
        if lux > 1000 and compensate:
            lux = self._lux_compensation(lux)
        return lux

    def _lux_compensation(self, lux):
        """
        Compensates lux when raw lux reading is >1000. Only used when gathering readings.
        There is no reverse compensation available.

        :param lux: Raw lux value (only use if lux > 1000)
        :type lux: float
        :return: Compensated lux
        :rtype: float
        """
        # TODO: Create reverse lux compensation
        return (0.00000000000060135 * lux**4
                - 0.0000000093924 * lux**3
                + 0.000081488 * lux**2
                + 1.0023 * lux)

    def _calculate_lux(self, count):
        """
        Calculate lux from digital count bits (based on current gain and integration time)

        :param count:
        :type count: int
        :return:
        :rtype: float
        """
        g = self.get_gain()
        it = self.get_integration_time()
        pd_div = self.get_pd_div()
        if pd_div == 0:
            return count*_lux_coeff_4_4[it][g]
        else:
            return count*_lux_coeff_1_4[it][g]
    
    def _read_bits_from_register(self, register, index, length=1):
        """
        Read value from register on VEML6030

        :param register: Register address to acquire
        :type register: int
        :param index: position to pull bits from
        :type index: int
        :param length: number of bits to grab
        :type length: int
        :return: Bits from register
        :rtype: Union[int, bool]
        """
        return get_bits(self._read_register(register), index, length)

    def _write_bits_to_register(self, register, value, index, length=1):
        """
        Write value to register on VEML6030

        :param register: Address of register you are writing to
        :type register: int
        :param value: Bits to write to register
        :type value: Union[int, bool]
        :param index: position to write bits to
        :type index: int
        :param length: number of bits to grab
        :type length: int
        """
        self._write_register(register, set_bits(self._read_register(register), value, index, length))

    def _read_register(self, register):
        """
        Read value from register on VEML6030

        :param register: Register address to acquire
        :type register: int
        :return: Word in register
        :rtype: int
        """
        # original: read_word_data, so 2 bytes
        result = bytearray(2)
        # Convert the bytearray to a 16-bit integer (little-endian)
        # VEML6031 uses little-endian byte order (LSB first)
        self.bus.readfrom_into(register, result)
        value = result[0] | (result[1] << 8)
        return value

    def _write_register(self, register, value):
        """
        Write value to register on VEML6030

        :param register: Address of register you are writing to
        :type register: int
        :param value: Word to write to register
        :type value: int
        """
        self.bus.writeto(register, value)

    def write_both_conf_registers(self, shutdown_bit):
        """
        Write to both configuration registers (CONF_0 and CONF_1) together
        as required by the datasheet.
        
        :param shutdown_bit: Boolean value for the shutdown bit in both registers
        :type shutdown_bit: bool
        """
        # Read current values first to preserve other settings
        current_conf0 = self._read_register(SETTING_CONF_0)
        current_conf1 = self._read_register(SETTING_CONF_1)
        
        # Update shutdown bit (bit 0) in both registers
        if shutdown_bit:
            new_conf0 = current_conf0 | 0x0001  # Set bit 0
            new_conf1 = current_conf1 | 0x0001  # Set bit 0
        else:
            new_conf0 = current_conf0 & 0xFFFE  # Clear bit 0
            new_conf1 = current_conf1 & 0xFFFE  # Clear bit 0
        
        # Lock the bus to perform both writes in rapid succession
        self.bus.try_lock()
        try:
            # Write to CONF_0 register (0x00)
            # Format: [register address, LSB, MSB]
            data0 = bytearray([SETTING_CONF_0, new_conf0 & 0xFF, (new_conf0 >> 8) & 0xFF])
            self.bus.writeto(self.address, data0)
            # Write to CONF_1 register (0x01) immediately after
            data1 = bytearray([SETTING_CONF_1, new_conf1 & 0xFF, (new_conf1 >> 8) & 0xFF])
            self.bus.writeto(self.address, data1)
        finally:
            self.bus.unlock()