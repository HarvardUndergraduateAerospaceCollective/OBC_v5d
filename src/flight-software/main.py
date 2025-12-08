# +++++++++ IMPORTS +++++++++ #
import gc
import os
import time
import board
import asyncio
import digitalio
import microcontroller
from fsm.fsm import FSM
from version import __version__
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter
from lib.pysquared.beacon import Beacon
from lib.adafruit_tca9548a import TCA9548A 
from lib.pysquared.watchdog import Watchdog
from lib.pysquared.config.config import Config
from lib.pysquared.cdh import CommandDataHandler
from lib.adafruit_mcp230xx.mcp23017 import MCP23017
from lib.proveskit_rp2350_v5b.register import Register
from fsm.data_processes.data_process import DataProcess
from lib.pysquared.config.jokes_config import JokesConfig
from lib.pysquared.detumbler_manager import DetumblerManager
from lib.pysquared.hardware.digitalio import initialize_pin
from lib.pysquared.protos.power_monitor import PowerMonitorProto
from lib.pysquared.hardware.radio.manager.rfm9x import RFM9xManager
from lib.pysquared.hardware.radio.manager.sx1280 import SX1280Manager
from lib.pysquared.hardware.busio import _spi_init, initialize_i2c_bus
from lib.pysquared.hardware.imu.manager.lsm6dsox import LSM6DSOXManager
from lib.pysquared.rtc.manager.microcontroller import MicrocontrollerManager
from lib.pysquared.hardware.burnwire.manager.burnwire import BurnwireManager
from lib.pysquared.hardware.power_monitor.manager.ina219 import INA219Manager
from lib.pysquared.hardware.magnetometer.manager.lis2mdl import LIS2MDLManager
from lib.pysquared.hardware.radio.packetizer.packet_manager import PacketManager
from lib.pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031x00Manager
from lib.pysquared.hardware.magnetorquer.manager.magnetorquer import MagnetorquerManager
from lib.pysquared.hardware.load_switch.manager.loadswitch_manager import LoadSwitchManager
 

# +++++++++ INIT LOGGER/CONFIGS +++++++++ #
logger: Logger = Logger(
    error_counter=Counter(0),
    colorized=False,
)

config: Config = Config("config.json")

jokes_config: JokesConfig = JokesConfig("jokes.json")


# +++++++++++++ INIT GENERAL PINS ++++++++++++++ #
# GPIO RESET manually sets the pin high to allow mcp to be detected
GPIO_RESET = initialize_pin(logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True)
SPI0_CS0 = initialize_pin(logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True)
SPI1_CS0 = initialize_pin(logger, board.SPI1_CS0, digitalio.Direction.OUTPUT, True)
# Payload Pins
RX0_OUTPUT = initialize_pin(logger, board.RX0, digitalio.Direction.OUTPUT, False)
RX1_OUTPUT = initialize_pin(logger, board.RX1, digitalio.Direction.OUTPUT, False)
TX0_OUTPUT = initialize_pin(logger, board.TX0, digitalio.Direction.OUTPUT, False)
TX1_OUTPUT = initialize_pin(logger, board.TX1, digitalio.Direction.OUTPUT, False)


# ++++++++++++ INIT I2C/SPI ++++++++++++ #
i2c1 = initialize_i2c_bus(
    logger,
    board.SCL1,
    board.SDA1,
    100000,
)

i2c0 = initialize_i2c_bus(
    logger,
    board.SCL0,
    board.SDA0,
    100000,
)

spi0 = _spi_init(
    logger,
    board.SPI0_SCK,
    board.SPI0_MOSI,
    board.SPI0_MISO,
)

spi1 = _spi_init(
    logger,
    board.SPI1_SCK,
    board.SPI1_MOSI,
    board.SPI1_MISO,
)


# ++++++++++++ INIT MCP/GENERAL SWITCHES ++++++++++++ #
mcp = MCP23017(i2c1)

# set these to high so that we have all our circuitry ready for use
PAYLOAD_PWR_ENABLE = mcp.get_pin(1)
PAYLOAD_PWR_ENABLE.direction = digitalio.Direction.OUTPUT
PAYLOAD_PWR_ENABLE.value = False

RF2_IO0 = mcp.get_pin(6)


# ++++++++++ INIT IMU ++++++++++++ #
magnetometer = LIS2MDLManager(logger, i2c1)

imu = LSM6DSOXManager(logger, i2c1, 0x6B)


# ++++++++++ INIT BATTERY ++++++++++++ #
PAYLOAD_BATT_ENABLE = mcp.get_pin(3)
PAYLOAD_BATT_ENABLE.direction = digitalio.Direction.OUTPUT
PAYLOAD_BATT_ENABLE.value = False

ENABLE_HEATER = mcp.get_pin(0)
ENABLE_HEATER.direction = digitalio.Direction.OUTPUT

battery_power_monitor: PowerMonitorProto = INA219Manager(logger, i2c0, 0x40)


# ++++++++++++ INIT RADIOS ++++++++++++ #
"""
uhf_radio = RFM9xManager(
    logger,
    config.radio,
    spi0,
    SPI0_CS0,
    initialize_pin(logger, board.RF1_RST, digitalio.Direction.OUTPUT, True),
)

uhf_packet_manager = PacketManager(
    logger,
    uhf_radio,
    config.radio.license,
    Counter(2),
    0.2,
)

cdh = CommandDataHandler(logger, config, uhf_packet_manager, jokes_config)
"""

sband_radio = SX1280Manager(
    logger,
    config.radio,
    spi1,
    SPI1_CS0,
    initialize_pin(logger, board.RF2_RST, digitalio.Direction.OUTPUT, True),
    RF2_IO0,
    2.4,
    initialize_pin(logger, board.RF2_TX_EN, digitalio.Direction.OUTPUT, False),
    initialize_pin(logger, board.RF2_RX_EN, digitalio.Direction.OUTPUT, False),
)

sband_packet_manager = PacketManager(
    logger,
    sband_radio,
    config.radio.license,
    Counter(2),
    0.2,
)

beacon = Beacon(
    logger,
    config.cubesat_name,
    sband_packet_manager,
    time.monotonic(),
    None, # instantiated later
    imu,
)


# ++++++++++++ LIGHT SENSOR INITIALIZATION ++++++++++++ #
FACE0_ENABLE = mcp.get_pin(9)
FACE1_ENABLE = mcp.get_pin(10)
FACE2_ENABLE = mcp.get_pin(11)
FACE3_ENABLE = mcp.get_pin(12)
FACE0_ENABLE.direction = digitalio.Direction.OUTPUT
FACE1_ENABLE.direction = digitalio.Direction.OUTPUT
FACE2_ENABLE.direction = digitalio.Direction.OUTPUT
FACE3_ENABLE.direction = digitalio.Direction.OUTPUT
MUX_RESET = initialize_pin(logger, board.MUX_RESET, digitalio.Direction.OUTPUT, False)
load_switch_0 = LoadSwitchManager(FACE0_ENABLE, True)
load_switch_1 = LoadSwitchManager(FACE1_ENABLE, True)
load_switch_2 = LoadSwitchManager(FACE2_ENABLE, True)
load_switch_3 = LoadSwitchManager(FACE3_ENABLE, True)
load_switch_0.enable_load()
load_switch_1.enable_load()
load_switch_2.enable_load()
load_switch_3.enable_load()
time.sleep(0.1)
MUX_RESET.value = True

tca = TCA9548A(i2c0, address=int(0x77))  # all 3 connected to high
light_sensors = []
face0_sensor = None
face1_sensor = None
face2_sensor = None
face3_sensor = None
try:
    face0_sensor = VEML6031x00Manager(logger, tca[0])
    light_sensors.append(face0_sensor)
except Exception as e:
    logger.debug(f"WARNING!!! Light sensor 0 failed to initialize {e}")
    light_sensors.append(None)
try:
    face1_sensor = VEML6031x00Manager(logger, tca[1])
    light_sensors.append(face1_sensor)
except Exception as e:
    logger.debug(f"WARNING!!! Light sensor 1 failed to initialize {e}")
    light_sensors.append(None)
try:
    face2_sensor = VEML6031x00Manager(logger, tca[2])
    light_sensors.append(face2_sensor)
except Exception as e:
    logger.debug(f"WARNING!!! Light sensor 2 failed to initialize {e}")
    light_sensors.append(None)
try:
    face3_sensor = VEML6031x00Manager(logger, tca[3])
    light_sensors.append(face3_sensor)
except Exception as e:
    logger.debug(f"WARNING!!! Light sensor 3 failed to initialize {e}")
    light_sensors.append(None)
try:
    face4_sensor = VEML6031x00Manager(logger, tca[4])
    light_sensors.append(face4_sensor)
except Exception as e:
    logger.debug(f"WARNING!!! Light sensor 4 failed to initialize {e}")
    light_sensors.append(None)


# +++++++++++++ INIT BURNWIRE +++++++++++++ #
burnwire_heater_enable = initialize_pin(
    logger, board.FIRE_DEPLOY1_A, digitalio.Direction.OUTPUT, False
)

burnwire1_fire = initialize_pin(
    logger, board.FIRE_DEPLOY1_B, digitalio.Direction.OUTPUT, False
)

antenna_deployment = BurnwireManager(
    logger, burnwire_heater_enable, burnwire1_fire, enable_logic=True
)


# +++++++++++++ INIT DETUMBLER/MAGNETORQUER +++++++++++++ #
detumbler_manager = DetumblerManager(gain=1.0)

"""
magnetorquer_manager = MagnetorquerManager( logger=logger,
                                            i2c_addr        =0x5a,
                                            addr_x_plus     =tca[0],
                                            addr_x_minus    =tca[1],
                                            addr_y_plus     =tca[2],
                                            addr_y_minus    =tca[3],
                                            addr_z_minus    =tca[4])
"""



# +++++++++++++ READ SENSORS +++++++++++++ #
while True:
    # Light Sensors
    if face0_sensor is not None:
        print("[LIGHT_SENSOR_FACE0] Light, Lux:\t", face0_sensor.get_light()._value, face0_sensor.get_lux()._value)
    if face1_sensor is not None:
        print("[LIGHT_SENSOR_FACE1] Light, Lux:\t", face1_sensor.get_light()._value, face1_sensor.get_lux()._value)
    if face2_sensor is not None:
        print("[LIGHT_SENSOR_FACE2] Light, Lux:\t", face2_sensor.get_light()._value, face2_sensor.get_lux()._value)
    if face3_sensor is not None:
        print("[LIGHT_SENSOR_FACE3] Light, Lux:\t", face3_sensor.get_light()._value, face3_sensor.get_lux()._value)
    # Battery Sensor
    print("[BATTERY SENSOR] Voltage:\t", battery_power_monitor.get_bus_voltage()._value)
    # IMU Sensor
    print("[IMU] Acceleration: \t\t", imu.get_acceleration().value)
    print("[IMU] Angular Velocity: \t", imu.get_angular_velocity().value)
    # Magnetometer Sensor
    print("[MAGNETOMETER] Magnetic Field: \t", magnetometer.get_magnetic_field().value)
    time.sleep(5)
