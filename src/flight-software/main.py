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

(boot_count := Counter(index=Register.boot_count)).increment()

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


# +++++ HELPER FUNCTION: SAFE SLEEP +++++ #
async def safe_sleep_async(
    duration: float,
    watchdog: Watchdog,
    logger: Logger,
    watchdog_timeout: float = 15.0,
    max_sleep: float = None,
):
    if max_sleep is not None and duration > max_sleep:
        logger.warning(
            "Requested sleep duration exceeds maximum allowed. Adjusting.",
            requested_duration=duration,
            max_sleep=max_sleep,
        )
        duration = max_sleep
    logger.info("Entering safe async sleep", duration=duration)
    end_time = time.monotonic() + duration
    watchdog.pet()
    while time.monotonic() < end_time:
        time_increment = min(end_time - time.monotonic(), watchdog_timeout)
        await asyncio.sleep(time_increment)
        watchdog.pet()
        logger.debug(
            "Petting watchdog during safe sleep",
            time_remaining=end_time - time.monotonic(),
        )


# +++++ MAIN FUNCTION: ASYNC LOOP +++++ #
async def main_async_loop():
    try:
        # ++++++++++ GPIO RESET FOR INIT +++++++++ #
        GPIO_RESET = initialize_pin(
            logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True
        )

        # ++++++++++++ INIT Watchdog ++++++++++++ #
        watchdog = Watchdog(logger, board.WDT_WDI)

        # ++++++++++++ SLEEP 30 Minutes ++++++++++++ #
        # only sleep if not yet deployed OR if booted less than 3 times (as a cut-off)
        if (
            deployed_count.get() < config.sleep_if_yet_deployed_count
            or boot_count.get() < config.sleep_if_yet_booted_count
        ):
            logger.info("[INFO] Sleeping for 30 minutes...")
            for _ in range(120):
                time.sleep(15)
                watchdog.pet()

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

        # ++++++++++++ INIT BURNWIRE ++++++++++++ #
        watchdog.pet()
        burnwire_heater_enable = initialize_pin(
            logger, board.FIRE_DEPLOY1_A, digitalio.Direction.OUTPUT, False
        )

        burnwire1_fire = initialize_pin(
            logger, board.FIRE_DEPLOY1_B, digitalio.Direction.OUTPUT, False
        )

        try:
            antenna_deployment = BurnwireManager(
                logger,
                burnwire_heater_enable,
                burnwire1_fire,
                enable_logic=True,
                watchdog=watchdog,
            )
        except Exception as e:
            antenna_deployment = None
            logger.debug(f"[WARNING] BurnwireManager Failed to initialize: {e}")

        # ++++++++++++ INIT PAYLOAD ++++++++++++ #
        watchdog.pet()
        RX0_OUTPUT = initialize_pin(
            logger, board.RX0, digitalio.Direction.OUTPUT, False
        )
        RX1_OUTPUT = initialize_pin(
            logger, board.RX1, digitalio.Direction.OUTPUT, False
        )
        TX0_OUTPUT = initialize_pin(
            logger, board.TX0, digitalio.Direction.OUTPUT, False
        )
        TX1_OUTPUT = initialize_pin(
            logger, board.TX1, digitalio.Direction.OUTPUT, False
        )

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
            face4_sensor = VEML6031x00Manager(logger, tca[4])
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
                addr_x_plus=tca[0],
                addr_x_minus=tca[2],
                addr_y_plus=tca[1],
                addr_y_minus=tca[3],
                addr_z_minus=tca[4],
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
            if cdh:
                try:
                    cdh.listen_for_commands(config.cdh_listen_command_timeout)
                except Exception as e:
                    logger.debug(f"[WARNING] cdh failed to listen or respond: {e}")
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

        # +++++++++ INIT DP OBJ AND FSM +++++++++ #
        watchdog.pet()
        dp_obj = DataProcess(
            magnetometer=magnetometer,
            imu=imu,
            battery_power_monitor=battery_power_monitor,
        )
        dp_obj.start_run_all_data()

        # Wait for sensor data to initialize before FSM starts
        logger.info("[INFO] Waiting for sensor data initialization...")
        await asyncio.sleep(2)
        logger.info(f"[INFO] Initial battery voltage: {dp_obj.data['data_batt_volt']}V")

        fsm_obj = FSM(
            dp_obj,
            logger,
            config,
            deployment_switch=antenna_deployment,
            tca=tca,
            rx0=RX0_OUTPUT,
            rx1=RX1_OUTPUT,
            tx0=TX0_OUTPUT,
            tx1=TX1_OUTPUT,
            face0_sensor=face0_sensor,
            face1_sensor=face1_sensor,
            face2_sensor=face2_sensor,
            face3_sensor=face3_sensor,
            face4_sensor=face4_sensor,
            magnetorquer_manager=magnetorquer_manager,
            detumbler_manager=detumbler_manager,
            PAYLOAD_BATT_ENABLE=PAYLOAD_BATT_ENABLE,
        )
        if beacon:
            beacon._fsm_obj = fsm_obj

        def nominal_power_loop():
            logger.debug(
                "FC Board Stats",
                bytes_remaining=gc.mem_free(),
            )

            all_faces_on()

            try:
                if uhf_packet_manager:
                    uhf_packet_manager.send(config.radio.license.encode("utf-8"))
            except Exception as e:
                logger.debug(f"[WARNING] uhf_packet_manager failed to send: {e}")
                # trigger Watchdog hard reset
                if except_reset_count.get() <= config.except_reset_allowed_attemps:
                    except_reset_count.increment()
                    time.sleep(config.watchdog_reset_sleep)

            if beacon:
                beacon.send_if_interval_elapsed()

            try:
                if cdh:
                    cdh.listen_for_commands(
                        config.cdh_listen_command_timeout
                    ) if cdh is not None else None
            except Exception as e:
                logger.debug(f"[WARNING] cdh failed to listen or respond: {e}")
                # trigger Watchdog hard reset
                if except_reset_count.get() <= config.except_reset_allowed_attemps:
                    except_reset_count.increment()
                    time.sleep(config.watchdog_reset_sleep)

            try:
                if uhf_packet_manager:
                    uhf_packet_manager.send(config.radio.license.encode("utf-8"))
            except Exception as e:
                logger.debug(f"[WARNING] uhf_packet_manager failed to send: {e}")
                # trigger Watchdog hard reset
                if except_reset_count.get() <= config.except_reset_allowed_attemps:
                    except_reset_count.increment()
                    time.sleep(config.watchdog_reset_sleep)

            # Second beacon opportunity (interval still enforced)
            if beacon:
                beacon.send_if_interval_elapsed()

            try:
                if cdh:
                    cdh.listen_for_commands(
                        config.cdh_listen_command_timeout
                    ) if cdh is not None else None
            except Exception as e:
                logger.debug(f"[WARNING] cdh failed to listen or respond: {e}")
                # trigger Watchdog hard reset
                if except_reset_count.get() <= config.except_reset_allowed_attemps:
                    except_reset_count.increment()
                    time.sleep(config.watchdog_reset_sleep)

        try:
            logger.info("Entering main loop")
            # Reset exception counter on successful main loop entry
            # This prevents permanent degraded mode after cumulative failures
            except_reset_count.set(0)
            logger.info("Exception reset counter cleared on successful boot")
            while True:
                val = fsm_obj.execute_fsm_step()
                current_voltage = fsm_obj.dp_obj.data["data_batt_volt"]

                # Tiered sleep based on battery voltage level
                # Critical: sleep longer to allow significant recharge
                # Low (FSM returned -1): sleep moderate amount
                if (
                    current_voltage is not None
                    and current_voltage <= config.critical_battery_voltage
                ):
                    # Battery critically low - sleep for extended period (10 minutes)
                    # to allow significant recharge before trying again
                    logger.warning(
                        f"[Main] Battery critically low ({current_voltage}V <= {config.critical_battery_voltage}V), "
                        "sleeping 10 minutes to recharge"
                    )
                    await safe_sleep_async(
                        duration=600,  # 10 minutes
                        watchdog=watchdog,
                        logger=logger,
                        watchdog_timeout=15,
                        max_sleep=config.longest_allowable_sleep_time,
                    )
                elif val == -1:
                    # FSM needs more charge (not critical, but below operational threshold)
                    # Sleep moderate amount (2 minutes)
                    logger.info(
                        f"[Main] Battery below operational threshold ({current_voltage}V), "
                        "sleeping 2 minutes to recharge"
                    )
                    await safe_sleep_async(
                        duration=120,  # 2 minutes
                        watchdog=watchdog,
                        logger=logger,
                        watchdog_timeout=15,
                        max_sleep=config.longest_allowable_sleep_time,
                    )

                await asyncio.sleep(1)
                watchdog.pet()
                nominal_power_loop()

        except Exception as e:
            logger.critical("Critical in Main Loop", e)
            # Pet watchdog during delay to avoid cascade reset
            for _ in range(5):
                time.sleep(2)
                try:
                    watchdog.pet()
                except Exception:
                    pass  # Watchdog may not be accessible
            microcontroller.on_next_reset(microcontroller.RunMode.NORMAL)
            microcontroller.reset()
        finally:
            logger.info("Going Neutral!")

    except Exception as e:
        logger.critical("An exception occured within main.py", e)


asyncio.run(main_async_loop())
