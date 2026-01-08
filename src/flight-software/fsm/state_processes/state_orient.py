# state_orient.py


# ++++++++++++++ Imports/Installs ++++++++++++++ #
import asyncio
import math

from lib.pysquared.hardware.light_sensor.manager.veml6031x00 import VEML6031x00Manager
from lib.pysquared.sensor_reading.light import Light


# ++++++++++++++ Functions: Helper ++++++++++++++ #
class StateOrient:
    def __init__(
        self,
        dp_obj,
        logger,
        config,
        tca,
        rx0,
        rx1,
        tx0,
        tx1,
        face0_sensor,
        face1_sensor,
        face2_sensor,
        face3_sensor,
        face4_sensor,
        PAYLOAD_BATT_ENABLE,
    ):
        """
        Initialize the class object
        """
        self.dp_obj = dp_obj
        self.logger = logger
        self.orient_best_direction = "None Better That Others"
        self.tca = tca
        self.running = False
        self.done = False
        self.rx0 = rx0
        self.rx1 = rx1
        self.tx0 = tx0
        self.tx1 = tx1
        self.config = config
        self.best_direction = -1
        self.PAYLOAD_BATT_ENABLE = PAYLOAD_BATT_ENABLE
        self.changed = False

        self.face0_sensor: VEML6031x00Manager | None = face0_sensor
        self.face1_sensor: VEML6031x00Manager | None = face1_sensor
        self.face2_sensor: VEML6031x00Manager | None = face2_sensor
        self.face3_sensor: VEML6031x00Manager | None = face3_sensor
        self.face4_sensor: VEML6031x00Manager | None = face4_sensor

        self.light_intensity = []
        self.payload_light_intensity = 0.0

    def _safe_all_off(self):
        self.rx0.value = False
        self.rx1.value = False
        self.tx0.value = False
        self.tx1.value = False

    @property
    def orient_payload_setting(self):
        return self.config.orient_payload_setting

    @property
    def orient_payload_periodic_time(self):
        return self.config.orient_payload_periodic_time

    def vector_mul_scalar(self, v, scalar):
        """Multiply two vectors by a scalar."""
        return [v[0] * scalar, v[1] * scalar]

    def vector_add(self, v1, v2):
        """Add two vectors."""
        return [v1[0] + v2[0], v1[1] + v2[1]]

    def vector_norm(self, v):
        """Get the norm of a vector."""
        return math.sqrt(v[0] ** 2 + v[1] ** 2)

    def dot_product(self, v1, v2):
        """Compute dot product between two vectors."""
        return v1[0] * v2[0] + v1[1] * v2[1]

    async def run(self):
        """
        Run the deployment sequence asynchronously
        """
        self.running = True
        in_sunlight = False
        while self.running:
            if self.config.orient_payload_setting == 0:
                # don't do anything
                # set all pins for false for sanity
                self._safe_all_off()
                self.logger.info("[Orient] Payload setting set to 0")
                await asyncio.sleep(2)
            elif not self.PAYLOAD_BATT_ENABLE.value:
                # don't do anything
                # set all pins for false for sanity
                self._safe_all_off()
                self.logger.info(
                    "[Orient] Payload battery is disabled: PAYLOAD_BATT_ENABLE = False"
                )
                await asyncio.sleep(2)
            else:
                # wait a little bit before beginning, just for stabilization
                self.logger.info("[Orient] Payload setting set to 1")
                await asyncio.sleep(2)

                # step 0: get light readings
                # lights: [scalar, scalar, scalar, scalar]
                try:
                    light0 = (
                        self.face0_sensor.get_light()
                        if self.face0_sensor is not None
                        else Light(0.0)
                    )
                    light1 = (
                        self.face1_sensor.get_light()
                        if self.face1_sensor is not None
                        else Light(0.0)
                    )
                    light2 = (
                        self.face2_sensor.get_light()
                        if self.face2_sensor is not None
                        else Light(0.0)
                    )
                    light3 = (
                        self.face3_sensor.get_light()
                        if self.face3_sensor is not None
                        else Light(0.0)
                    )
                    light4 = (
                        self.face4_sensor.get_light()
                        if self.face4_sensor is not None
                        else Light(0.0)
                    )
                    self.payload_light_intensity = light4._value
                    lights = [light0, light1, light2, light3]
                # if fail, set all to 0
                except Exception as e:
                    self.logger.debug(f"Failed to read light sensors: {e}")
                    lights = [Light(0.0), Light(0.0), Light(0.0), Light(0.0)]
                self.light_intensity = [lights[i]._value for i in range(4)]

                # step 1: determine if we are in sunlight
                # if we are in sunlight, pull one of the sensors and wait 25 minutes
                max_light = max(self.light_intensity)
                in_sunlight = max_light > self.config.orient_light_threshold
                if not in_sunlight:
                    # remember we don't have a valid direction
                    self.best_direction = -1
                    self._safe_all_off()
                    self.changed = False
                    self.logger.info(
                        "[Orient] Threshold not exceeded, not in sunlight.  Waiting two minutes..."
                    )
                    # if not in sunlight, try again another read in 2 minutes
                    # wait 2 minutes to allow for quick adjustment once we're back in sunlight
                    await asyncio.sleep(2 * 60)
                else:
                    # step 2: get the index of the face from which we saw the max light
                    # see if this face is different from last time
                    best_direction_index = self.light_intensity.index(
                        max(self.light_intensity)
                    )
                    self.changed = self.best_direction != best_direction_index
                    self.best_direction = best_direction_index

                    # step 3: log results
                    self.logger.info(f"[Orient] Best Dir Index {self.best_direction}")
                    self.logger.info(
                        f"[Orient] All X/Y Light Readings: {self.light_intensity}"
                    )

                    # step 4: actuate only when direction changed
                    # Direction mapping:
                    # FACE 0: +X, TX1
                    # FACE 1: +Y, RX1
                    # FACE 2: -X, TX0
                    # FACE 3: -Y, RX0
                    if self.best_direction == -1:
                        self.logger.info(
                            "None better than others, due to light sensors not on."
                        )
                        self._safe_all_off()
                        self.orient_best_direction = "None Better That Others"
                        await asyncio.sleep(
                            self.config.orient_payload_periodic_time * 60
                        )
                        continue

                    if not self.changed:
                        self.logger.info(
                            "[Orient] Best direction unchanged; skipping actuation."
                        )
                        self._safe_all_off()
                        await asyncio.sleep(
                            self.config.orient_payload_periodic_time * 60
                        )
                        continue

                    self.logger.info(
                        "Turning off payload actuators, giving 2 seconds for spring to settle"
                    )
                    self._safe_all_off()
                    await asyncio.sleep(2)

                    if self.best_direction == 0:
                        self.logger.info("Activating +Y spring")
                        self.rx0.value = False
                        self.rx1.value = False
                        self.tx0.value = False
                        self.tx1.value = True
                        self.orient_best_direction = "+Y Axis"
                    elif self.best_direction == 1:
                        self.logger.info("Activating -X spring")
                        self.rx0.value = False
                        self.rx1.value = True
                        self.tx0.value = False
                        self.tx1.value = False
                        self.orient_best_direction = "-X Axis"
                    elif self.best_direction == 2:
                        self.logger.info("Activating -Y spring")
                        self.rx0.value = False
                        self.rx1.value = False
                        self.tx0.value = True
                        self.tx1.value = False
                        self.orient_best_direction = "-Y Axis"
                    elif self.best_direction == 3:
                        self.logger.info("Activating +X spring")
                        self.rx0.value = True
                        self.rx1.value = False
                        self.tx0.value = False
                        self.tx1.value = False
                        self.orient_best_direction = "+X Axis"

                    # enforce max on-time
                    await asyncio.sleep(self.config.orient_heat_duration)
                    self._safe_all_off()

                    # reset change flag and wait for next observation window
                    self.changed = False
                    await asyncio.sleep(self.config.orient_payload_periodic_time * 60)

    def stop(self):
        """
        Used by FSM to manually stop run()
        """
        self.running = False
        # Ensure all actuators are off when stopping orient state
        self._safe_all_off()

    def is_done(self):
        """
        Checked by FSM to see if the run() completed on its own
        If it did complete, it shuts down the async task run()
        """
        return self.done
