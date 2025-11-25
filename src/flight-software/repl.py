import json
import os
import time

import board
import asyncio
import digitalio
from lib.adafruit_mcp230xx.mcp23017 import MCP23017
from lib.adafruit_tca9548a import TCA9548A
from lib.pysquared.beacon import Beacon
from lib.pysquared.cdh import CommandDataHandler
from lib.pysquared.config.config import Config
from lib.pysquared.config.jokes_config import JokesConfig
from lib.pysquared.file_validation.manager.file_validation import FileValidationManager
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
from lib.pysquared.hardware.temperature_sensor.manager.mcp9808 import MCP9808Manager
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter
from lib.pysquared.protos.power_monitor import PowerMonitorProto
from lib.pysquared.rtc.manager.microcontroller import MicrocontrollerManager
from lib.pysquared.watchdog import Watchdog
from version import __version__
from fsm.data_processes.data_process import DataProcess
from fsm.fsm import FSM
from lib.pysquared.hardware.magnetorquer.manager.magnetorquer import MagnetorquerManager
from lib.pysquared.detumbler_manager import DetumblerManager


# ----- Initializations ----- #
logger: Logger = Logger(
    error_counter=Counter(0),
    colorized=False,
)

config: Config = Config("config.json")
jokes_config: JokesConfig = JokesConfig("jokes.json")

# manually set the pin high to allow mcp to be detected
GPIO_RESET = (
    initialize_pin(logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True),
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
PAYLOAD_PWR_ENABLE.direction = digitalio.Direction.OUTPUT # never need it
PAYLOAD_BATT_ENABLE.direction = digitalio.Direction.OUTPUT


FACE2_ENABLE = mcp.get_pin(11)
FACE2_ENABLE.direction = digitalio.Direction.OUTPUT
load_switch_2 = LoadSwitchManager(FACE2_ENABLE, True)  # type: ignore , upstream on mcp TODO
load_switch_2.enable_load()

# set these to high so that we have the circuitry ready for use
PAYLOAD_PWR_ENABLE.value = False
PAYLOAD_BATT_ENABLE.value = True

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
    None, # instantiated later
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
    face2_sensor = VEML6031x00Manager(logger, tca[2])
    light_sensors.append(face2_sensor)
    print("HEREEE")
except Exception as e:
    logger.debug(f"WARNING!!! Light sensor 2 failed to initialize {e}")
    light_sensors.append(None)

# Magnetorquer Initializations
"""
In main.py no need to instantiate these, these will be used by State Detumble
We're instantiating here to test it can be instantiated.
"""
detumbler_manager = DetumblerManager(gain=1.0)
magnetorquer_manager = None
"""
# Don't do until plugged in
magnetorquer_manager = MagnetorquerManager( logger=logger,
                                            i2c_addr        =0x5a, # TODO: DOUBLE CHECK!  this is default DRV2605 for adafruit
                                            addr_x_plus     =tca[0],
                                            addr_x_minus    =tca[1],
                                            addr_y_plus     =tca[2],
                                            addr_y_minus    =tca[3],
                                            addr_z_minus    =tca[4])
"""

# CDH
cdh = CommandDataHandler(logger, config, uhf_packet_manager, jokes_config)

# ----- Test Functions ----- #
def test_dm_obj_initialization():
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    try:
        res = dm_obj.data
        if res is not None:
            print("\033[92mPASSED\033[0m [dm_obj test]")
    except Exception as e:
            print("\033[91mFAILED:\033[0m [dm_obj test]", e) 

async def test_dm_obj_magnetometer():
    dm_obj = DataProcess(magnetometer=magnetometer,
                         imu=imu,
                         battery_power_monitor=battery_power_monitor)
    dm_obj.running = True       
    dm_obj.start_run_all_data()
    print("Monitor magnetoruqer for 10 seconds.  Move/don't move around FC to see if the value changes.")
    await asyncio.sleep(1)
    for _ in range(100):
        print(dm_obj.data["data_magnetometer_vector"])
        await asyncio.sleep(0.1)
    return input("Is this acceptable? (Y/N): ").strip().upper()

async def test_dm_obj_imu():
    dm_obj = DataProcess(magnetometer=magnetometer,
                         imu=imu,
                         battery_power_monitor=battery_power_monitor)
    dm_obj.running = True
    dm_obj.start_run_all_data()
    # imu av
    print("Monitor imu av for 10 seconds.  Move/don't move around FC to see if the value changes.")
    await asyncio.sleep(1)
    for _ in range(100):
        print(dm_obj.data["data_imu_av"])
        await asyncio.sleep(0.1)
    input("Is the av acceptable? (Y/N): ").strip().upper()
    # imu acc
    print("Monitor imu acc for 10 seconds.  Move/don't move around FC to see if the value changes.")
    await asyncio.sleep(1)
    for _ in range(100):
        print(dm_obj.data["data_imu_acc"])
        await asyncio.sleep(0.1)
    return input("Is the acc acceptable? (Y/N): ").strip().upper()

async def test_dm_obj_battery():
    dm_obj = DataProcess(magnetometer=magnetometer,
                         imu=imu,
                         battery_power_monitor=battery_power_monitor)
    dm_obj.running = True
    dm_obj.start_run_all_data()
    input("Please plug in the batteries and press Enter when done.")
    print("Voltage with batteries:", dm_obj.data["data_batt_volt"])
    input("Please unplug the batteries and press Enter when done.")
    print("Voltage without batteries:", dm_obj.data["data_batt_volt"])
    return input("Did the voltage drop by >= 4V? (Y/N): ").strip().upper()

async def test_dm_obj_get_data_updates():
    dm_obj = DataProcess(magnetometer=magnetometer,
                         imu=imu,
                         battery_power_monitor=battery_power_monitor)
    try:
        # [:] allows for a shallow copy
        battery_voltage_before = dm_obj.data["data_batt_volt"]
        imu_av_before = dm_obj.data["data_imu_av"][:]
        imu_av_mag_before = dm_obj.data["data_imu_av_magnitude"]
        imu_acc_before = dm_obj.data["data_imu_acc"][:]
        mag_vector_before = dm_obj.data["data_magnetometer_vector"][:]
        # enable the loop once
        dm_obj.running = True       
        dm_obj.start_run_all_data()  
        await asyncio.sleep(1.1)
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

def test_fsm_transitions():
        dm_obj = DataProcess(magnetometer=magnetometer,
                         imu=imu,
                         battery_power_monitor=battery_power_monitor)
        fsm_obj = FSM(dm_obj,
                logger,
                config,
                deployment_switch=antenna_deployment,
                tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
                face0_sensor=face0_sensor, face1_sensor=face1_sensor,
                face2_sensor=face2_sensor, face3_sensor=face3_sensor,
                magnetorquer_manager=magnetorquer_manager,
                detumbler_manager=detumbler_manager,
                PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
        beacon._fsm_obj = fsm_obj

        # Initially, FSM should be in bootup
        assert(fsm_obj.curr_state_name == "bootup")

        # Simulate bootup done
        fsm_obj.curr_state_object.done = True
        fsm_obj.execute_fsm_step()
        assert fsm_obj.curr_state_name == "detumble", "\033[91mFAILED\033[0m [test_fsm_transitions bootup -> detumble]"

        # Simulate detumble done
        fsm_obj.curr_state_object.done = True
        fsm_obj.dp_obj.data["data_batt_volt"] = 7.5
        fsm_obj.dp_obj.data["data_imu_av_magnitude"] = 0.2
        fsm_obj.execute_fsm_step()
        assert fsm_obj.curr_state_name == "deploy", "\033[91mFAILED\033[0m [test_fsm_transitions detumble -> deploy]"

        # Simulate deploy done
        fsm_obj.curr_state_object.done = True
        fsm_obj.execute_fsm_step()
        assert fsm_obj.curr_state_name == "orient", "\033[91mFAILED\033[0m [test_fsm_transitions deploy -> orient]"

        # Make sure to cleanup to keep effects isolated!
        if fsm_obj.curr_state_run_asyncio_task is not None:
            fsm_obj.curr_state_object.stop()
            fsm_obj.curr_state_run_asyncio_task.cancel()
        print("\033[92mPASSED\033[0m [test_fsm_transitions]")

def test_fsm_deploy_burnwire():
    choice = input("Would you like to try the burnwire test (Y/N)?: ").strip().lower()
    if choice == "y":
        input("Get ready to test fire deploy 1A, press enter when ready.").strip().upper()
        time.sleep(3)
        print("Burning for 5 seconds....")
        antenna_deployment.burn(5)
        print("Finished burning.")
        return input("Did the burnwire get hot? (Y/N): ").strip().upper()
    return "N/A"

def test_fsm_orient_current():
    choice = input("Would you like to try the orient current test (Y/N)?: ").strip().lower()
    if choice == "y":
        input("Get ready to test RX0, press enter when ready.").strip().upper()
        print("Running current for 5 seconds....")
        RX0_OUTPUT.value = True
        time.sleep(5)
        RX0_OUTPUT.value = False
        input("Did the wire get current? (Y/N): ").strip().upper()
        
        input("Get ready to test RX1, press enter when ready.").strip().upper()
        print("Running current for 5 seconds....")
        RX1_OUTPUT.value = True
        time.sleep(5)
        RX1_OUTPUT.value = False
        input("Did the wire get current? (Y/N): ").strip().upper()

        input("Get ready to test TX0, press enter when ready.").strip().upper()
        print("Running current for 5 seconds....")
        TX0_OUTPUT.value = True
        time.sleep(5)
        TX0_OUTPUT.value = False
        input("Did the wire get current? (Y/N): ").strip().upper()

        input("Get ready to test TX1, press enter when ready.").strip().upper()
        print("Running current for 5 seconds....")
        TX1_OUTPUT.value = True
        time.sleep(5)
        TX1_OUTPUT.value = False
        input("Did the wire get current? (Y/N): ").strip().upper()
    return "N/A"

def test_fsm_orient_config_change():
    choice = input("Would you like to try the orient config change test (Y/N)?: ").strip().lower()
    if choice == "y":
        dm_obj = DataProcess(magnetometer=magnetometer,
                imu=imu,
                battery_power_monitor=battery_power_monitor)
        fsm_obj = FSM(dm_obj,
                logger,
                config,
                deployment_switch=antenna_deployment,
                tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
                face0_sensor=face0_sensor, face1_sensor=face1_sensor,
                face2_sensor=face2_sensor, face3_sensor=face3_sensor,
                magnetorquer_manager=magnetorquer_manager,
                detumbler_manager=detumbler_manager,
                PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
        
        fsm_obj.set_state("orient")

        print(fsm_obj.curr_state_object.orient_payload_setting)
        print(fsm_obj.curr_state_object.orient_payload_periodic_time)
        time.sleep(1)
        
        config.update_config("orient_payload_setting", 0, temporary=True)
        print(fsm_obj.curr_state_object.orient_payload_setting)
        print(fsm_obj.curr_state_object.orient_payload_periodic_time)
        time.sleep(1)
        
        config.update_config("orient_payload_setting", 2, temporary=True)
        print(fsm_obj.curr_state_object.orient_payload_setting)
        print(fsm_obj.curr_state_object.orient_payload_periodic_time)
        time.sleep(1)

        config.update_config("orient_payload_periodic_time", 12, temporary=True)
        print(fsm_obj.curr_state_object.orient_payload_setting)
        print(fsm_obj.curr_state_object.orient_payload_periodic_time)
        
        # Make sure to cleanup to keep effects isolated!
        if fsm_obj.curr_state_run_asyncio_task is not None:
            fsm_obj.curr_state_object.stop()
            fsm_obj.curr_state_run_asyncio_task.cancel()
        return input("Did the orient mechanism and/or period change as intended? (Y/N): ").strip().upper()

def test_fsm_orient_command():
    choice = input("Would you like to try the orient command test (Y/N)?: ").strip().lower()
    if choice == "y":
        dm_obj = DataProcess(magnetometer=magnetometer,
                imu=imu,
                battery_power_monitor=battery_power_monitor)
        fsm_obj = FSM(dm_obj,
                logger,
                config,
                deployment_switch=antenna_deployment,
                tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
                face0_sensor=face0_sensor, face1_sensor=face1_sensor,
                face2_sensor=face2_sensor, face3_sensor=face3_sensor,
                magnetorquer_manager=magnetorquer_manager,
                detumbler_manager=detumbler_manager,
                PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
        fsm_obj.set_state("orient")
        print(fsm_obj.curr_state_object.orient_payload_setting)
        print(fsm_obj.curr_state_object.orient_payload_periodic_time)
        cdh.listen_for_commands(10)
        print(fsm_obj.curr_state_object.orient_payload_setting)
        print(fsm_obj.curr_state_object.orient_payload_periodic_time)
        # Make sure to cleanup to keep effects isolated!
        if fsm_obj.curr_state_run_asyncio_task is not None:
            fsm_obj.curr_state_object.stop()
            fsm_obj.curr_state_run_asyncio_task.cancel()
        return input("Did the orient mechanism and/or period change as intended? (Y/N): ").strip().upper()

def test_sband():
    choice = input("Choose to be a receiver R or sender S.  Make sure the receiver starts up first!").strip().upper()
    if choice == "R":
        # Wait for any response
        start_time = time.time()
        while time.time() - start_time < 10:
            message = sband_radio._radio.receive(keep_listening=False)
            if message:
                received = message
                print("Received...", received)
                break
            else:
                print("Still waiting...")
            time.sleep(0.5)
    else:
        # Attempt to send the beacon
        print("Sending in 2 seconds...")
        time.sleep(2)
        success = sband_radio._radio.send(b"Hello there from FCB!")
        if not success:
            print("sband failed to send.")
        else:
            print("sband sent successfully!")
    print("Stopping test...")
    return

async def test_fsm_emergency_detumble():
    """
    Test that the FSM immediately switches to 'detumble' when 
    angular velocity magnitude exceeds emergency threshold.
    """
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    
    # Initially, FSM should be in bootup
    assert(fsm_obj.curr_state_name == "bootup")
    # Simulate bootup for a little bit (lower than what it is supposed to be, i.e > 5 seconds)
    await asyncio.sleep(2)

    # Simulate dangerous angular velocity (trigger emergency detumble)
    fsm_obj.dp_obj.data["data_imu_av_magnitude"] = 2.0
    fsm_obj.execute_fsm_step()

    # FSM should immediately jump to detumble
    assert fsm_obj.curr_state_name == "detumble", "\033[91mFAILED\033[0m [test_fsm_emergency_detumble: not transitioned to detumble]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    print("\033[92mPASSED\033[0m [test_fsm_emergency_detumble]")

async def test_fsm_detumble_max_duration():
    """
    Test that the FSM stops detumble if it exceeds max duration
    """
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    
    # Set these parameters artificially so that it won't ever switch to deploy prematurely
    fsm_obj.dp_obj.data["data_batt_volt"] = 7.5
    fsm_obj.dp_obj.data["data_imu_av_magnitude"] = 0.5
    fsm_obj.set_state("detumble")

    # Simulate bootup for a little bit (lower than what it is supposed to be, i.e > 5 seconds)
    await asyncio.sleep(6)
    fsm_obj.execute_fsm_step()

    # FSM should have switched
    assert fsm_obj.curr_state_name == "deploy", "\033[91mFAILED\033[0m [test_fsm_detumble_max_duration: stayed in detumble]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    print("\033[92mPASSED\033[0m [test_fsm_emergency_detumble]")

async def test_fsm_detumble_stop_conditions():
    """
    Test that the FSM transitions from 'detumble' -> 'deploy'
    once angular velocity magnitude falls below the detumble threshold.
    More robust than test_fsm_transitions.
    """
    # part 1: test a successful stop detumble -> deploy
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj

    # Initially, FSM should be in bootup
    assert(fsm_obj.curr_state_name == "bootup")
    # Simulate bootup for a little bit (higher than what it is supposed to be, i.e > 5 seconds)
    await asyncio.sleep(6)
    fsm_obj.execute_fsm_step()

    # Now we should be in Detumble
    assert(fsm_obj.curr_state_name == "detumble")
    # So that the detumble trigger gets done, set arbitrarily to 7.5 and 0.02 (< 0.05)
    fsm_obj.dp_obj.data["data_batt_volt"] = 7.5
    fsm_obj.dp_obj.data["data_imu_av_magnitude"] = 0.02
    # Wait a little so that it execute detumble
    await asyncio.sleep(fsm_obj.curr_state_object.detumble_frequency + 0.1)
    fsm_obj.execute_fsm_step()

    # Now we should be in Deploy
    assert fsm_obj.curr_state_name == "deploy", "\033[91mFAILED\033[0m [test_fsm_detumble_stop_conditions]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    print("\033[92mPASSED\033[0m [test_fsm_detumble_stop_conditions [Part 1, batt and av mag both good]]")

    # part 2: test a non-successful stop detumble -> deploy due to battery voltage
    beacon._fsm_obj = None
    fsm_obj = None
    dm_obj = None

    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj

    # Initially, FSM should be in bootup
    assert(fsm_obj.curr_state_name == "bootup")
    # Simulate bootup for a little bit (higher than what it is supposed to be, i.e > 5 seconds)
    await asyncio.sleep(6)
    fsm_obj.execute_fsm_step()

    # Now we should be in Detumble
    assert(fsm_obj.curr_state_name == "detumble")
    # So that the detumble trigger gets done, set to 5 (low) and 0.02 (< 0.05)
    fsm_obj.dp_obj.data["data_batt_volt"] = 5
    fsm_obj.dp_obj.data["data_imu_av_magnitude"] = 0.02
    # Wait a little so that it execute detumble
    await asyncio.sleep(fsm_obj.curr_state_object.detumble_frequency + 0.1)
    fsm_obj.execute_fsm_step()

    # We should STILL be in Detumble
    assert fsm_obj.curr_state_name == "detumble", "\033[91mFAILED\033[0m [test_fsm_detumble_stop_conditions]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    print("\033[92mPASSED\033[0m [test_fsm_detumble_stop_conditions [Part 2, batt too low]]")


async def test_fsm_orient_run():
    """
    Test that the orient runs through an algorithm
    """

    # part 1: test a successful stop detumble -> orient
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    fsm_obj.set_state("orient")
    await asyncio.sleep(10)
    input("Is the log output acceptable? (Y/N): ").strip().upper()
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    print("\033[92mPASSED\033[0m [test_fsm_orient_run]")


async def test_fsm_orient_cdh_output():
    """
    Test that the orient's chosen direction gets outputted
    """

    # part 1: test a successful stop detumble -> orient
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    fsm_obj.set_state("orient")
    # note you may need to adjust the sleeps
    await asyncio.sleep(1)
    fsm_obj.execute_fsm_step()
    await asyncio.sleep(1)
    fsm_obj.execute_fsm_step()
    await asyncio.sleep(1)
    input("Is the log output acceptable? (Y/N): ").strip().upper()
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    print("\033[92mPASSED\033[0m [test_fsm_orient_run]")


async def test_fsm_orient_above_battery():
    """
    Test that the FSM transitions from 'detumble' or 'deploy' -> 'orient' when battery is sufficient.
    More robust than test_fsm_transitions.
    """

    # part 1: test a successful stop detumble -> orient
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    # Initially, FSM should be in bootup
    fsm_obj.deployed = True
    fsm_obj.set_state("detumble")
    assert(fsm_obj.curr_state_name == "detumble")
    # Wait a little so that it execute detumble
    fsm_obj.dp_obj.data["data_batt_volt"] = 7
    await asyncio.sleep(fsm_obj.curr_state_object.detumble_frequency + 0.1)
    fsm_obj.execute_fsm_step()
    # Now we should be in Orient
    assert fsm_obj.curr_state_name == "orient", "\033[91mFAILED\033[0m [test_fsm_orient_above_battery]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    beacon._fsm_obj = None
    fsm_obj = None
    dm_obj = None
    print("\033[92mPASSED\033[0m [test_fsm_orient_above_battery [Part 1, detumble, batt is good]]")

    # part 2: test a non-successful stop detumble -> orient
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    # Initially, FSM should be in bootup
    fsm_obj.deployed = True
    fsm_obj.set_state("detumble")
    assert(fsm_obj.curr_state_name == "detumble")
    # Wait a little so that it execute detumble
    fsm_obj.dp_obj.data["data_batt_volt"] = 5
    await asyncio.sleep(fsm_obj.curr_state_object.detumble_frequency + 0.1)
    fsm_obj.execute_fsm_step()
    # Now we should be in detumble
    assert fsm_obj.curr_state_name == "detumble", "\033[91mFAILED\033[0m [test_fsm_orient_above_battery]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    beacon._fsm_obj = None
    fsm_obj = None
    dm_obj = None
    print("\033[92mPASSED\033[0m [test_fsm_orient_above_battery [Part 2, detumble, batt is too low]]")

    # part 3: test a successful stop deploy -> orient
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    # Initially, FSM should be in bootup
    fsm_obj.deployed = True
    fsm_obj.set_state("deploy")
    assert(fsm_obj.curr_state_name == "deploy")
    # Wait a little so that it execute deploy
    fsm_obj.dp_obj.data["data_batt_volt"] = 7.5
    fsm_obj.curr_state_object.finished_burn = True
    await asyncio.sleep(2)
    fsm_obj.execute_fsm_step()
    # Now we should be in Orient
    assert fsm_obj.curr_state_name == "orient", "\033[91mFAILED\033[0m [test_fsm_orient_above_battery]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    beacon._fsm_obj = None
    fsm_obj = None
    dm_obj = None
    print("\033[92mPASSED\033[0m [test_fsm_orient_above_battery [Part 3, deploy, batt is good]]")

    # part 4: test a non-successful stop deploy -> orient
    dm_obj = DataProcess(magnetometer=magnetometer,
                        imu=imu,
                        battery_power_monitor=battery_power_monitor)
    fsm_obj = FSM(dm_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca, rx0=RX0_OUTPUT, rx1=RX1_OUTPUT, tx0=TX0_OUTPUT, tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor, face1_sensor=face1_sensor,
            face2_sensor=face2_sensor, face3_sensor=face3_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE)
    beacon._fsm_obj = fsm_obj
    # Initially, FSM should be in bootup
    fsm_obj.deployed = True
    fsm_obj.set_state("deploy")
    assert(fsm_obj.curr_state_name == "deploy")
    # Wait a little so that it execute deploy
    fsm_obj.dp_obj.data["data_batt_volt"] = 5
    fsm_obj.curr_state_object.finished_burn = True
    await asyncio.sleep(2)
    fsm_obj.execute_fsm_step()
    # We should still be in deploy
    print(fsm_obj.curr_state_name, fsm_obj.curr_state_object.is_done(), fsm_obj.dp_obj.data["data_batt_volt"])
    assert fsm_obj.curr_state_name == "deploy", "\033[91mFAILED\033[0m [test_fsm_orient_above_battery]"
    # Clean up asyncio task if created
    if fsm_obj.curr_state_run_asyncio_task is not None:
        fsm_obj.curr_state_object.stop()
        fsm_obj.curr_state_run_asyncio_task.cancel()
    beacon._fsm_obj = None
    fsm_obj = None
    dm_obj = None
    print("\033[92mPASSED\033[0m [test_fsm_orient_above_battery [Part 4, deploy, batt is too low]]")

def test_i2c_scan():
    """
    Scan all I2C buses and multiplexer channels to detect connected devices.
    Prints out the addresses of all detected I2C devices.
    """
    print("\n=== I2C Scanner ===")
    
    # Scan the primary I2C buses
    print("\nScanning i2c0 bus:")
    try:
        i2c0.try_lock()
        devices = i2c0.scan()
        i2c0.unlock()
        if devices:
            print(f"Found {len(devices)} devices:")
            for addr in devices:
                print(f"  - Device at address: 0x{addr:02X}")
        else:
            print("No devices found")
    except Exception as e:
        print(f"Error scanning i2c0: {e}")
    
    print("\nScanning i2c1 bus:")
    try:
        i2c1.try_lock()
        devices = i2c1.scan()
        i2c1.unlock()
        if devices:
            print(f"Found {len(devices)} devices:")
            for addr in devices:
                print(f"  - Device at address: 0x{addr:02X}")
        else:
            print("No devices found")
    except Exception as e:
        print(f"Error scanning i2c1: {e}")
    
    # Scan TCA9548A multiplexer channels if available
    print("\nScanning TCA9548A multiplexer channels:")
    try:
        channel = tca[2]
        channel.try_lock()
        devices = channel.scan()
        channel.unlock()
        if devices:
            print(f"Channel 2: Found {len(devices)} devices:")
            for addr in devices:
                print(f"  - Device at address: 0x{addr:02X}")
        else:
            print(f"Channel 2: No devices found")
    except Exception as e:
        print(f"Error scanning multiplexer: {e}")
    print("\n=== I2C Scan Complete ===")


# ========== MAIN FUNCTION ========== #

def test_all():
    # fsm tests
    #test_fsm_transitions()                           # TESTED
    #test_fsm_deploy_burnwire()                       # TESTED - do deploy aux 1 top one (or bottom) and GND in upper right
    #test_fsm_orient_current()                        # TESTED - do RX0, RX1, TX0, TX1 and GND in upper right
    #test_fsm_orient_config_change()                  # TESTED                        
    #test_fsm_orient_command()
    #asyncio.run(test_fsm_emergency_detumble())       # TESTED 
    #asyncio.run(test_fsm_detumble_stop_conditions()) # TESTED 
    #asyncio.run(test_fsm_orient_above_battery())     # TESTED
    #asyncio.run(test_fsm_detumble_max_duration())    # TESTED
    #asyncio.run(test_fsm_orient_run())               # TESTED - note though this is pretty much identical to cdh output and needs execute_fsm for log to show
    #asyncio.run(test_fsm_orient_cdh_output())        # TESTED

    # non-fsm tests
    #test_sband()                                    # TESTED
    #test_i2c_scan()

    # dm_obj tests
    #test_dm_obj_initialization()                     # TESTED
    #asyncio.run(test_dm_obj_get_data_updates())      # TESTED
    #asyncio.run(test_dm_obj_magnetometer())          # TESTED
    #asyncio.run(test_dm_obj_imu())                   # TESTED
    #asyncio.run(test_dm_obj_battery())               # TESTED
    pass