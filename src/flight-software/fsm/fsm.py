# fsm.py


# ++++++++++++++++++ Imports and Installs ++++++++++++++++++ #
import asyncio

from fsm.state_processes.state_bootup import StateBootup
from fsm.state_processes.state_deploy import StateDeploy
from fsm.state_processes.state_detumble import StateDetumble
from fsm.state_processes.state_orient import StateOrient
from lib.proveskit_rp2350_v5b.register import Register
from lib.pysquared.nvm.counter import Counter


# ++++++++++++++++++++ Class Definition ++++++++++++++++++++ #
class FSM:
    def __init__(
        self,
        dp_obj,
        logger,
        config,
        deployment_switch,
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
        magnetorquer_manager,
        detumbler_manager,
        PAYLOAD_BATT_ENABLE,
    ):
        self.dp_obj = dp_obj  # object of type DataProcess
        self.logger = logger  # logging status of FSM states
        self.config = config
        self.deployment_switch = deployment_switch
        self.PAYLOAD_BATT_ENABLE = PAYLOAD_BATT_ENABLE
        self.enable_detumble = self.config.enable_detumble

        self.state_objects = {
            "bootup": StateBootup(dp_obj, logger),
            "detumble": StateDetumble(
                dp_obj, logger, config, tca, magnetorquer_manager, detumbler_manager,
                self.enable_detumble
            ),
            "deploy": StateDeploy(dp_obj, logger, config, deployment_switch),
            "orient": StateOrient(
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
            ),
        }
        self.curr_state_name = "bootup"
        self.curr_state_object = self.state_objects["bootup"]
        self.curr_state_run_asyncio_task = asyncio.create_task(
            self.curr_state_object.run()
        )

        # Initialize deployed status from NVM for persistence across reboots
        self.deployed_counter = Counter(index=Register.deployed_count)
        self.deployed = self.deployed_counter.get() > 0
        self.orient_best_direction = "None Better That Others"
        self.orient_light_intensity = []
        self.payload_light_intensity = 0.0

        # Emergency detumble debounce counter - requires multiple consecutive
        # readings above threshold to trigger (prevents false triggers from noise)
        self._emergency_detumble_counter = 0
        self._emergency_detumble_threshold = (
            3  # Number of consecutive readings required
        )

    def set_state(self, new_state_name):
        """
        This function is called when we switch states from execute_fsm()
        """
        # Stop current state's background task
        # OK that orient may be in the future be stopped by detumble
        if self.curr_state_run_asyncio_task is not None:
            self.curr_state_object.stop()
            self.curr_state_run_asyncio_task.cancel()
            self.curr_state_run_asyncio_task = None

        # Safety: Ensure magnetorquers are stopped when leaving detumble
        if self.curr_state_name == "detumble":
            detumble_state = self.state_objects["detumble"]
            if detumble_state.magnetorquer_manager:
                detumble_state.magnetorquer_manager.stop_dipole_moments()

        self.curr_state_name = new_state_name
        self.curr_state_object = self.state_objects[new_state_name]
        self.curr_state_run_asyncio_task = asyncio.create_task(
            self.curr_state_object.run()
        )

    def execute_fsm_step(self):
        """
        This function runs a single execution of the finite state machine (fsm)
        It checks its current state and data points and sees if we
        need to change state, take action, etc.
        Note: because we pass in db_obj, its data variable will update
        automatically if any changes are made for that db_obj
        """

        # NOTE: Emergency override for low battery and power consumption is handled in main.py

        # Startup → Detumble
        if self.curr_state_name == "bootup" and self.curr_state_object.is_done():
            self.set_state("detumble")
            return 0

        # Emergency Detumble - ONLY check when NOT in bootup state
        # During bootup, IMU data may be uninitialized/garbage and could trigger false emergency
        # Also skip if already in detumble state to prevent re-entry loops
        if (
            self.curr_state_name not in ("bootup", "detumble")
            and self.dp_obj.data["data_imu_av_magnitude"] is not None
            and self.enable_detumble
            and self.dp_obj.data["data_imu_av_magnitude"]
            > self.config.detumble_stabilize_threshold * 1.5
        ):  # some added-buffer to not trigger this too much, only in emergency
            # Debounce: require multiple consecutive readings above threshold
            self._emergency_detumble_counter += 1
            if self._emergency_detumble_counter >= self._emergency_detumble_threshold:
                self.logger.warning(
                    "[FSM] Emergency detumble triggered after debounce",
                    av_magnitude=self.dp_obj.data["data_imu_av_magnitude"],
                    threshold=self.config.detumble_stabilize_threshold * 1.5,
                    counter=self._emergency_detumble_counter,
                )
                # Reset counter for next time
                self._emergency_detumble_counter = 0
                # if we were coming from orient, disable power to payload immediately
                self.PAYLOAD_BATT_ENABLE.value = False
                # Don't wait for other state to be done, shut it off immediately
                self.set_state("detumble")
                return 0
        else:
            # Reset debounce counter if reading is normal
            self._emergency_detumble_counter = 0

        # Detumble → Deploy or Orient
        # Guard against None battery voltage (sensor failure) - stay in detumble if unknown
        batt_volt = self.dp_obj.data["data_batt_volt"]
        if self.curr_state_name == "detumble" and self.curr_state_object.is_done():
            if batt_volt is None:
                # Battery sensor failure - stay in detumble (conservative)
                return -1
            elif self.deployed and batt_volt > self.config.fsm_batt_threshold_orient:
                self.set_state("orient")
                return 0
            elif (
                not self.deployed and batt_volt > self.config.fsm_batt_threshold_deploy
            ):
                # Don't set deployed flag yet - wait until burnwire actually fires
                self.set_state("deploy")
                return 0
            else:
                # Let the main file know we need to charge a bit more
                return -1

        # Deploy → Orient
        # Guard against None battery voltage
        if (
            self.curr_state_name == "deploy"
            and self.curr_state_object.is_done()
            and batt_volt is not None
            and batt_volt > self.config.fsm_batt_threshold_orient
        ):
            # Mark deployment as complete and persist to NVM
            if not self.deployed:
                self.deployed = True
                self.deployed_counter.increment()
                self.logger.info(
                    "[FSM] Deployment confirmed complete, persisted to NVM"
                )

            if self.config.orient_payload_setting == 0:
                self.PAYLOAD_BATT_ENABLE.value = False
            else:
                self.PAYLOAD_BATT_ENABLE.value = True
            self.set_state("orient")
            return 0
        elif (
            self.curr_state_name == "deploy"
            and self.curr_state_object.is_done()
            and (batt_volt is None or batt_volt < self.config.fsm_batt_threshold_orient)
        ):
            # Let the main file know we need to charge a bit more
            return -1

        # Orient Parameters
        if self.curr_state_name == "orient":
            # Payload enable/disable
            if self.config.orient_payload_setting == 0:
                self.PAYLOAD_BATT_ENABLE.value = False
            else:
                self.PAYLOAD_BATT_ENABLE.value = True

            # Cache the best direction and light intensity
            self.orient_best_direction = self.curr_state_object.orient_best_direction
            self.orient_light_intensity = self.curr_state_object.light_intensity
            self.payload_light_intensity = (
                self.curr_state_object.payload_light_intensity
            )
            self.logger.info(
                "[FSM] orient_best_direction: "
                + str(self.curr_state_object.best_direction)
                + ", "
                + str(self.curr_state_object.orient_best_direction)
            )
            self.logger.info(
                "[FSM] orient_light_intensity: "
                + str(self.curr_state_object.light_intensity)
            )
