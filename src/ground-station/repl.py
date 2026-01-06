import board
import digitalio
from lib.adafruit_mcp230xx.mcp23017 import MCP23017
from lib.pysquared.config.config import Config
from lib.pysquared.config.jokes_config import JokesConfig
from lib.pysquared.hardware.busio import _spi_init, initialize_i2c_bus
from lib.pysquared.hardware.digitalio import initialize_pin
from lib.pysquared.hardware.radio.manager.sx1280 import SX1280Manager
from lib.pysquared.logger import Logger
from lib.pysquared.nvm.counter import Counter

logger: Logger = Logger(
    error_counter=Counter(1),
    colorized=False,
)

config: Config = Config("config.json")

jokes_config: JokesConfig = JokesConfig("jokes.json")

# manually set the pin high to allow mcp to be detected
GPIO_RESET = (
    initialize_pin(logger, board.GPIO_EXPANDER_RESET, digitalio.Direction.OUTPUT, True),
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

SPI0_CS0 = initialize_pin(logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True)

SPI1_CS0 = initialize_pin(logger, board.SPI1_CS0, digitalio.Direction.OUTPUT, True)

i2c1 = initialize_i2c_bus(
    logger,
    board.SCL1,
    board.SDA1,
    100000,
)

mcp = MCP23017(i2c1)

RF2_IO0 = mcp.get_pin(6)

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

"""
radio = RFM9xManager(
    logger,
    config.radio,
    spi0,
    initialize_pin(logger, board.SPI0_CS0, digitalio.Direction.OUTPUT, True),
    initialize_pin(logger, board.RF1_RST, digitalio.Direction.OUTPUT, True),
)


packet_manager = PacketManager(
    logger,
    sband_radio,
    config.radio.license,
    Counter(2),
    0.2,
)

cdh = CommandDataHandler(
    logger,
    config,
    packet_manager,
    jokes_config,
)

ground_station = GroundStation(
    logger,
    config,
    packet_manager,
    cdh,
)

# ground_station.run()
"""

import time


def test_sband():
    choice = (
        input(
            "Choose to be a receiver R or sender S.  Make sure the receiver starts up first!"
        )
        .strip()
        .upper()
    )
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
            print("❌ Beacon failed to send.")
        else:
            print("✅ Beacon sent successfully!")
    print("Stopping test...")
    return


test_sband()
