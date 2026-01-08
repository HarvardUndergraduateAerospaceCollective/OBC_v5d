import asyncio
import json
import time

import board
import digitalio
from fsm.data_processes.data_process import DataProcess
from fsm.fsm import FSM
from lib.adafruit_mcp230xx.mcp23017 import MCP23017
from lib.adafruit_tca9548a import TCA9548A
from lib.pysquared.beacon import Beacon
from lib.pysquared.cdh import CommandDataHandler
from lib.pysquared.config.config import Config
from lib.pysquared.config.jokes_config import JokesConfig
from lib.pysquared.detumbler_manager import DetumblerManager
from lib.pysquared.hardware.burnwire.manager.burnwire import BurnwireManager
from lib.pysquared.hardware.busio import _spi_init, initialize_i2c_bus
from lib.pysquared.hardware.digitalio import initialize_pin
from lib.pysquared.hardware.imu.manager.lsm6dsox import LSM6DSOXManager
from lib.pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031x00Manager
from lib.pysquared.hardware.load_switch.manager.loadswitch_manager import (
    LoadSwitchManager,
)
from lib.pysquared.hardware.magnetometer.manager.lis2mdl import LIS2MDLManager
from lib.pysquared.hardware.power_monitor.manager.ina219 import INA219Manager
from lib.pysquared.hardware.radio.manager.rfm9x import RFM9xManager
from lib.pysquared.hardware.radio.manager.sx1280 import SX1280Manager
from lib.pysquared.hardware.radio.packetizer.packet_manager import PacketManager
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter
from lib.pysquared.protos.power_monitor import PowerMonitorProto
from lib.pysquared.rtc.manager.microcontroller import MicrocontrollerManager
from lib.pysquared.hardware.magnetorquer.manager.magnetorquer import MagnetorquerManager

# ----- Initializations ----- #
logger: Logger = Logger(
    error_counter=Counter(0),
    colorized=False,
)

config: Config = Config("config.json")
jokes_config: JokesConfig = JokesConfig("jokes.json")

# manually set the pin high to allow mcp to be detected
GPIO_RESET = initialize_pin(
    logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True
)

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

mcp = MCP23017(i2c1)
ENABLE_HEATER = mcp.get_pin(0)
PAYLOAD_PWR_ENABLE = mcp.get_pin(1)
PAYLOAD_BATT_ENABLE = mcp.get_pin(3)
ENABLE_HEATER.direction = digitalio.Direction.OUTPUT
PAYLOAD_PWR_ENABLE.direction = digitalio.Direction.OUTPUT  # never need it
PAYLOAD_BATT_ENABLE.direction = digitalio.Direction.OUTPUT


FACE2_ENABLE = mcp.get_pin(11)
FACE2_ENABLE.direction = digitalio.Direction.OUTPUT
load_switch_2 = LoadSwitchManager(FACE2_ENABLE, True)  # type: ignore , upstream on mcp TODO
load_switch_2.enable_load()

# set these to high so that we have the circuitry ready for use
PAYLOAD_PWR_ENABLE.value = False
PAYLOAD_BATT_ENABLE.value = False

SPI0_CS0 = initialize_pin(logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True)

SPI1_CS0 = initialize_pin(logger, board.SPI1_CS0, digitalio.Direction.OUTPUT, True)

RF2_IO0 = mcp.get_pin(6)

rtc = MicrocontrollerManager()

magnetometer = LIS2MDLManager(logger, i2c1)

imu = LSM6DSOXManager(logger, i2c1, 0x6B)

burnwire_heater_enable = initialize_pin(
    logger, board.FIRE_DEPLOY1_A, digitalio.Direction.OUTPUT, False
)

burnwire1_fire = initialize_pin(
    logger, board.FIRE_DEPLOY1_B, digitalio.Direction.OUTPUT, False
)

antenna_deployment = BurnwireManager(
    logger, burnwire_heater_enable, burnwire1_fire, enable_logic=True
)

battery_power_monitor: PowerMonitorProto = INA219Manager(logger, i2c0, 0x40)

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
    None,  # instantiated later
    imu,
)

# Payload Pins
RX0_OUTPUT = initialize_pin(logger, board.RX0, digitalio.Direction.OUTPUT, False)
RX1_OUTPUT = initialize_pin(logger, board.RX1, digitalio.Direction.OUTPUT, False)
TX0_OUTPUT = initialize_pin(logger, board.TX0, digitalio.Direction.OUTPUT, False)
TX1_OUTPUT = initialize_pin(logger, board.TX1, digitalio.Direction.OUTPUT, False)

# Light Sensors
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

# Magnetorquer Initializations
"""
In main.py no need to instantiate these, these will be used by State Detumble
We're instantiating here to test it can be instantiated.
"""
detumbler_manager = DetumblerManager(gain=1.0)
try:
    magnetorquer_manager = MagnetorquerManager( logger=logger,
                                            i2c_addr        =0x5a,
                                            addr_x_plus     =tca[0],
                                            addr_x_minus    =tca[2],
                                            addr_y_plus     =tca[1],
                                            addr_y_minus    =tca[3],
                                            addr_z_minus    =tca[4])
except:
    magnetorquer_manager = None

# CDH
cdh = CommandDataHandler(logger, config, uhf_packet_manager, jokes_config)


# ----- Test Functions ----- #
def test_dm_obj_initialization():
    dm_obj = DataProcess(
        magnetometer=magnetometer, imu=imu, battery_power_monitor=battery_power_monitor
    )
    try:
        res = dm_obj.data
        if res is not None:
            print("\033[92mPASSED\033[0m [dm_obj test]")
    except Exception as e:
        print("\033[91mFAILED:\033[0m [dm_obj test]", e)


async def test_dm_obj_magnetometer():
    dm_obj = DataProcess(
        magnetometer=magnetometer, imu=imu, battery_power_monitor=battery_power_monitor
    )
    dm_obj.running = True
    dm_obj.start_run_all_data()
    print(
        "Monitor magnetoruqer for 10 seconds.  Move/don't move around FC to see if the value changes."
    )
    await asyncio.sleep(1)
    for _ in range(100):
        print(dm_obj.data["data_magnetometer_vector"])
        await asyncio.sleep(0.1)
    return input("Is this acceptable? (Y/N): ").strip().upper()


async def test_dm_obj_imu():
    dm_obj = DataProcess(
        magnetometer=magnetometer, imu=imu, battery_power_monitor=battery_power_monitor
    )
    dm_obj.running = True
    dm_obj.start_run_all_data()
    # imu av
    print(
        "Monitor imu av for 10 seconds.  Move/don't move around FC to see if the value changes."
    )
    await asyncio.sleep(1)
    for _ in range(100):
        print(dm_obj.data["data_imu_av"])
        await asyncio.sleep(0.1)
    input("Is the av acceptable? (Y/N): ").strip().upper()
    # imu acc
    print(
        "Monitor imu acc for 10 seconds.  Move/don't move around FC to see if the value changes."
    )
    await asyncio.sleep(1)
    for _ in range(100):
        print(dm_obj.data["data_imu_acc"])
        await asyncio.sleep(0.1)
    return input("Is the acc acceptable? (Y/N): ").strip().upper()


async def test_dm_obj_battery():
    dm_obj = DataProcess(
        magnetometer=magnetometer, imu=imu, battery_power_monitor=battery_power_monitor
    )
    dm_obj.running = True
    dm_obj.start_run_all_data()
    input("Please plug in the batteries and press Enter when done.")
    print("Voltage with batteries:", dm_obj.data["data_batt_volt"])
    input("Please unplug the batteries and press Enter when done.")
    print("Voltage without batteries:", dm_obj.data["data_batt_volt"])
    return input("Did the voltage drop by >= 4V? (Y/N): ").strip().upper()


async def test_dm_obj_get_data_updates():
    dm_obj = DataProcess(
        magnetometer=magnetometer, imu=imu, battery_power_monitor=battery_power_monitor
    )
    try:
        # [:] allows for a shallow copy
        # enable the loop once
        dm_obj.running = True
        dm_obj.start_run_all_data()
        await asyncio.sleep(1)
        battery_voltage_before = dm_obj.data["data_batt_volt"]
        imu_av_before = dm_obj.data["data_imu_av"][:]
        imu_av_mag_before = dm_obj.data["data_imu_av_magnitude"]
        imu_acc_before = dm_obj.data["data_imu_acc"][:]
        mag_vector_before = dm_obj.data["data_magnetometer_vector"][:]
        await asyncio.sleep(1)
        # stop the infinite loop
        # let tasks exit cleanly
        dm_obj.running = False
        await asyncio.sleep(0.1)
        # check if data was updated
        battery_voltage_after = dm_obj.data["data_batt_volt"]
        imu_av_after = dm_obj.data["data_imu_av"][:]
        imu_av_mag_after = dm_obj.data["data_imu_av_magnitude"]
        imu_acc_after = dm_obj.data["data_imu_acc"][:]
        mag_vector_after = dm_obj.data["data_magnetometer_vector"][:]
        if battery_voltage_before != battery_voltage_after:
            print("\033[92mPASSED\033[0m [test_dm_obj_get_data_updates data_batt_volt]")
        else:
            print("\033[91mFAILED\033[0m [test_dm_obj_get_data_updates data_batt_volt]")
        if imu_av_before != imu_av_after:
            print("\033[92mPASSED\033[0m [test_dm_obj_get_data_updates imu_av]")
        else:
            print("\033[91mFAILED\033[0m [test_dm_obj_get_data_updates imu_av]")
        if imu_av_mag_before != imu_av_mag_after:
            print("\033[92mPASSED\033[0m [test_dm_obj_get_data_updates imu_av_mag]")
        else:
            print("\033[91mFAILED\033[0m [test_dm_obj_get_data_updates imu_av_mag]")
        if imu_acc_before != imu_acc_after:
            print("\033[92mPASSED\033[0m [test_dm_obj_get_data_updates imu_acc]")
        else:
            print("\033[91mFAILED\033[0m [test_dm_obj_get_data_updates imu_acc]")
        if mag_vector_before != mag_vector_after:
            print("\033[92mPASSED\033[0m [test_dm_obj_get_data_updates mag_vector]")
        else:
            print("\033[91mFAILED\033[0m [test_dm_obj_get_data_updates mag_vector]")
    except Exception as e:
        print("\033[91mFAILED\033[0m [test_dm_obj_get_data_updates Exception]", e)

# ========== MAIN FUNCTION ========== #
def test_all():
    # ========== DM_OBJ TESTS ========== #
    # dm_obj tests
    test_dm_obj_initialization()                     # TESTED
    asyncio.run(test_dm_obj_get_data_updates())      # TESTED
    asyncio.run(test_dm_obj_magnetometer())          # TESTED
    asyncio.run(test_dm_obj_imu())                   # TESTED
    asyncio.run(test_dm_obj_battery())               # TESTED
    pass

while True:
    time.sleep(1)
    test_all()