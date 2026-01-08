# +++++++++++++ IMPORTS +++++++++++++ #
import asyncio
import gc
import os
import time

import board
import digitalio
import microcontroller
from fsm.data_processes.data_process import DataProcess
from fsm.fsm import FSM
from lib.adafruit_mcp230xx.mcp23017 import MCP23017
from lib.adafruit_tca9548a import TCA9548A
from lib.proveskit_rp2350_v5b.register import Register
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
from lib.pysquared.hardware.magnetometer.manager.lis2mdl import LIS2MDLManager
from lib.pysquared.hardware.magnetorquer.manager.magnetorquer import MagnetorquerManager
from lib.pysquared.hardware.power_monitor.manager.ina219 import INA219Manager
from lib.pysquared.hardware.radio.manager.rfm9x import RFM9xManager
from lib.pysquared.hardware.radio.manager.sx1280 import SX1280Manager
from lib.pysquared.hardware.radio.packetizer.packet_manager import PacketManager
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter
from lib.pysquared.rtc.manager.microcontroller import MicrocontrollerManager
from lib.pysquared.watchdog import Watchdog
from version import __version__

# +++++++++++++ RTC + NVM REGISTERS +++++++++++++ #
boot_time: float = time.time()

rtc = MicrocontrollerManager()

boot_count: Counter = Counter(index=Register.boot_count)

error_count: Counter = Counter(index=Register.error_count)

deployed_count: Counter = Counter(index=Register.deployed_count)

except_reset_count: Counter = Counter(index=Register.except_reset_count)


# +++++++++++++ LOGGER + CONFIGS +++++++++++++ #
logger: Logger = Logger(
    error_counter=error_count,
    colorized=False,
)

logger.info(
    "Booting",
    hardware_version=os.uname().version,  # type: ignore[attr-defined]
    software_version=__version__,
)

logger.debug("Initializing Config")

config = None  # Will be set to either Config or MinimalConfig
try:
    config = Config("config.json")
except Exception as e:
    logger.critical(
        "Config Failed to initialize - using hardcoded survival defaults", e
    )

    # Minimal fallback config for survival mode - satellite must be able to operate
    # without config.json in case of filesystem corruption
    class MinimalConfig:
        except_reset_allowed_attemps = 3
        watchdog_reset_sleep = 60
        sleep_if_yet_deployed_count = 1
        sleep_if_yet_booted_count = 3
        critical_battery_voltage = 6.6
        fsm_batt_threshold_deploy = 7.0
        fsm_batt_threshold_orient = 6.8
        longest_allowable_sleep_time = 600
        cdh_listen_command_timeout = 10
        cubesat_name = "HUCSat"
        detumble_gain = 1.0
        detumble_max_time = 90.0
        detumble_stabilize_threshold = 0.2
        detumble_adjust_frequency = 2.0
        orient_light_threshold = 10.0
        orient_payload_setting = 1
        orient_payload_periodic_time = 25.0
        orient_heat_duration = 10.0
        deploy_burn_duration = 15.0
        deploy_max_attempts = 3
        deploy_retry_delay = 60.0

        class _MinimalRadioConfig:
            license = "WP2XZJ"
            transmit_frequency = 437.4
            modulation = "LoRa"

            class lora:
                spreading_factor = 8
                coding_rate = 8
                transmit_power = 23
                max_output = True
                ack_delay = 0.2
                cyclic_redundancy_check = True

            class fsk:
                broadcast_address = 255
                node_address = 1
                modulation_type = 0

        radio = _MinimalRadioConfig()

    config = MinimalConfig()

jokes_config = None
try:
    jokes_config = JokesConfig("jokes.json")
except Exception as e:
    logger.debug(f"[WARNING] JokesConfig Failed to initialize: {e}")
    # Jokes are non-critical, continue without them


# +++++++++++++ LOITER TIME +++++++++++++ #
loiter_time: int = 5
for i in range(loiter_time):
    logger.info(f"Code Starting in {loiter_time-i} seconds")
    time.sleep(1)


# ++++++++++ GPIO RESET FOR INIT +++++++++ #
GPIO_RESET = initialize_pin(
    logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True
)

# ++++++++++++ INIT Watchdog ++++++++++++ #
watchdog = Watchdog(logger, board.WDT_WDI)

# ++++++++++++ INIT SPI/I2C ++++++++++++ #
watchdog.pet()
SPI0_CS0 = initialize_pin(
    logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True
)

SPI1_CS0 = initialize_pin(
    logger, board.SPI1_CS0, digitalio.Direction.OUTPUT, True
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

# ++++++++++++ INIT MCP ++++++++++++ #
watchdog.pet()
mcp = MCP23017(i2c1)

# GPB
FACE4_ENABLE = mcp.get_pin(8)
FACE0_ENABLE = mcp.get_pin(9)
FACE1_ENABLE = mcp.get_pin(10)
FACE2_ENABLE = mcp.get_pin(11)
FACE3_ENABLE = mcp.get_pin(12)

# Set face enable pins to output mode
FACE4_ENABLE.direction = digitalio.Direction.OUTPUT
FACE0_ENABLE.direction = digitalio.Direction.OUTPUT
FACE1_ENABLE.direction = digitalio.Direction.OUTPUT
FACE2_ENABLE.direction = digitalio.Direction.OUTPUT
FACE3_ENABLE.direction = digitalio.Direction.OUTPUT

# GPA
PAYLOAD_BATT_ENABLE = mcp.get_pin(3)
PAYLOAD_BATT_ENABLE.direction = digitalio.Direction.OUTPUT
PAYLOAD_BATT_ENABLE.value = False
RF2_IO0 = mcp.get_pin(6)

# ++++++++++++ INIT RADIOS ++++++++++++ #
watchdog.pet()
try:
    sband_radio = SX1280Manager(
        logger,
        config.radio,
        spi1,
        SPI1_CS0,
        initialize_pin(logger, board.RF2_RST, digitalio.Direction.OUTPUT, True),
        RF2_IO0,
        2.4,
        initialize_pin(
            logger, board.RF2_TX_EN, digitalio.Direction.OUTPUT, False
        ),
        initialize_pin(
            logger, board.RF2_RX_EN, digitalio.Direction.OUTPUT, False
        ),
    )
except Exception as e:
    sband_radio = None
    logger.debug(f"[WARNING] SX1280Manager Failed to initialize: {e}")
try:
    uhf_radio = RFM9xManager(
        logger,
        config.radio,
        spi0,
        SPI0_CS0,
        initialize_pin(logger, board.RF1_RST, digitalio.Direction.OUTPUT, True),
    )
except Exception as e:
    uhf_radio = None
    logger.debug(f"[WARNING] RFM9xManager Failed to initialize: {e}")
    if except_reset_count.get() <= config.except_reset_allowed_attemps:
        except_reset_count.increment()
        time.sleep(config.watchdog_reset_sleep)
try:
    if uhf_radio is None:
        raise ValueError("uhf_radio is None")
    uhf_packet_manager = PacketManager(
        logger,
        uhf_radio,
        config.radio.license,
        Counter(2),
        0.2,
    )
except Exception as e:
    uhf_packet_manager = None
    logger.debug(f"[WARNING] PacketManager Failed to initialize: {e}")
    if except_reset_count.get() <= config.except_reset_allowed_attemps:
        except_reset_count.increment()
        time.sleep(config.watchdog_reset_sleep)
time.sleep(5)

# ++++++++++++ DONT INIT BURNWIRE ++++++++++++ #
watchdog.pet()

# ++++++++++++ DONT INIT PAYLOAD ++++++++++++ #
watchdog.pet()

# +++++++++ FACE POWER AND TCA MUX SETUP +++++++++ #
def all_faces_off():
    """
    This function turns off all of the faces. Note the load switches are enabled low.
    """
    FACE0_ENABLE.value = False
    FACE1_ENABLE.value = False
    FACE2_ENABLE.value = False
    FACE3_ENABLE.value = False
    FACE4_ENABLE.value = False

def all_faces_on():
    """
    This function turns on all of the faces. Note the load switches are enabled high.
    """
    FACE0_ENABLE.value = True
    FACE1_ENABLE.value = True
    FACE2_ENABLE.value = True
    FACE3_ENABLE.value = True
    FACE4_ENABLE.value = True

# Reset TCA mux and power on faces BEFORE initializing sensors
mux_reset = initialize_pin(
    logger, board.MUX_RESET, digitalio.Direction.OUTPUT, False
)
all_faces_on()
time.sleep(0.1)
mux_reset.value = True
tca = TCA9548A(i2c0, address=int(0x77))

# +++++++++ INIT LIGHT SENSORS +++++++++ #
watchdog.pet()
light_sensors = []
face0_sensor = None
face1_sensor = None
face2_sensor = None
face3_sensor = None
face4_sensor = None
try:
    face0_sensor = VEML6031x00Manager(logger, tca[0])
    light_sensors.append(face0_sensor)
except Exception as e:
    logger.debug(f"[WARNING] Light sensor 0 failed to initialize {e}")
    light_sensors.append(None)
try:
    face1_sensor = VEML6031x00Manager(logger, tca[1])
    light_sensors.append(face1_sensor)
except Exception as e:
    logger.debug(f"[WARNING]! Light sensor 1 failed to initialize {e}")
    light_sensors.append(None)
try:
    face2_sensor = VEML6031x00Manager(logger, tca[2])
    light_sensors.append(face2_sensor)
except Exception as e:
    logger.debug(f"[WARNING] Light sensor 2 failed to initialize {e}")
    light_sensors.append(None)
try:
    face3_sensor = VEML6031x00Manager(logger, tca[3])
    light_sensors.append(face3_sensor)
except Exception as e:
    logger.debug(f"[WARNING] Light sensor 3 failed to initialize {e}")
    light_sensors.append(None)
try:
    face4_sensor = VEML6031x00Manager(logger, tca[5])
    light_sensors.append(face4_sensor)
except Exception as e:
    logger.debug(f"[WARNING] Light sensor 4 failed to initialize {e}")
    light_sensors.append(None)

# +++++++++ INIT DETUMBLER/MAGNETORUQER/IMU +++++++++ #
watchdog.pet()
try:
    detumbler_manager = DetumblerManager(gain=config.detumble_gain)
except Exception as e:
    detumbler_manager = None
    logger.debug(f"[WARNING] DetumblerManager Failed to initialize: {e}")
try:
    magnetorquer_manager = MagnetorquerManager(
        logger=logger,
        i2c_addr=0x5A,
        addr_x_plus=tca[3],
        addr_x_minus=tca[1],
        addr_y_plus=tca[0],
        addr_y_minus=tca[2],
        addr_z_minus=tca[5],
    )
except Exception as e:
    magnetorquer_manager = None
    logger.debug(f"[WARNING] MagnetorquerManager Failed to initialize: {e}")
try:
    magnetometer = LIS2MDLManager(logger, i2c1)
except Exception as e:
    magnetometer = None
    logger.debug(f"[WARNING] LIS2MDLManager Failed to initialize: {e}")
try:
    imu = LSM6DSOXManager(logger, i2c1, 0x6B)
except Exception as e:
    imu = None
    logger.debug(f"[WARNING] LSM6DSOXManager Failed to initialize: {e}")

# +++++++++ INIT CDH/BEACON +++++++++ #
watchdog.pet()
try:
    if uhf_packet_manager is None:
        raise ValueError("uhf_packet_manager is None")
    cdh = CommandDataHandler(logger, config, uhf_packet_manager, jokes_config)
except Exception as e:
    cdh = None
    logger.debug(f"[WARNING] CommandDataHandler Failed to initialize: {e}")
    if except_reset_count.get() <= config.except_reset_allowed_attemps:
        except_reset_count.increment()
        time.sleep(config.watchdog_reset_sleep)
time.sleep(5)
watchdog.pet()
try:
    if uhf_packet_manager is None:
        raise ValueError("beacon init, uhf_packet_manager is None")
    if uhf_radio is None:
        raise ValueError("beacon init, uhf_radio is None")
    if sband_radio is None:
        raise ValueError("beacon init, sband_radio is None")
    if imu is None:
        raise ValueError("beacon init, imu is None")
    if magnetometer is None:
        raise ValueError("beacon init, magnetometer is None")
    beacon = Beacon(
        logger,
        config.cubesat_name,
        uhf_packet_manager,
        time.monotonic(),
        None,
        imu,
        magnetometer,
        uhf_radio,
        sband_radio,
    )
except Exception as e:
    beacon = None
    logger.debug(f"[WARNING] Beacon Failed to initialize: {e}")
    if except_reset_count.get() <= config.except_reset_allowed_attemps:
        except_reset_count.increment()
        time.sleep(config.watchdog_reset_sleep)

# +++++++++ INIT BATTERY MONITOR +++++++++ #
watchdog.pet()
try:
    battery_power_monitor = INA219Manager(logger, i2c0, 0x40)
except Exception as e:
    battery_power_monitor = None
    logger.debug(f"[WARNING] INA219Manager Failed to initialize: {e}")
    if except_reset_count.get() <= config.except_reset_allowed_attemps:
        except_reset_count.increment()
        microcontroller.reset()
    else:
        logger.critical(
            "Exceeded reset attempts - continuing without battery monitor!",
            Exception("No battery monitor"),
        )

# +++++++++ INIT DP OBJ +++++++++ #
watchdog.pet()
dm_obj = DataProcess(
    magnetometer=magnetometer,
    imu=imu,
    battery_power_monitor=battery_power_monitor,
)

dm_obj.running = True
dm_obj.start_run_all_data()

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

async def test_dm_values():
    dm_obj = DataProcess(
        magnetometer=magnetometer, imu=imu, battery_power_monitor=battery_power_monitor
    )
    dm_obj.running = True
    dm_obj.start_run_all_data()
    while True:
        await asyncio.sleep(1)
        watchdog.pet()
        print("batt_volt:", dm_obj.data["data_batt_volt"])
        print("imu av:", dm_obj.data["data_imu_av"])
        print("imu_av_magnitude:", dm_obj.data["data_imu_av_magnitude"])
        print("imu_acc:", dm_obj.data["data_imu_acc"])
        print("magnetometer_vector:", dm_obj.data["data_magnetometer_vector"])
        if face0_sensor:
            print("light sensor face 0:", face0_sensor.get_light()._value)
        if face1_sensor:
            print("light sensor face 1:", face1_sensor.get_light()._value)
        if face2_sensor:
            print("light sensor face 2:", face2_sensor.get_light()._value)
        if face3_sensor:
            print("light sensor face 3:", face3_sensor.get_light()._value)
        if face4_sensor:
            print("light sensor face 4:", face4_sensor.get_light()._value)
        watchdog.pet()


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

asyncio.run(test_dm_values())