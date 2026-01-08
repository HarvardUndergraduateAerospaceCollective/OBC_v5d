# state_detumble.py


# ++++++++++++++ Imports/Installs ++++++++++++++ #
import asyncio
import time

from lib.pysquared.detumbler_manager import DetumblerManager
from lib.pysquared.hardware.magnetorquer.manager.magnetorquer import MagnetorquerManager


# ++++++++++++++ Functions: Helper ++++++++++++++ #
class StateDetumble:
    def __init__(
        self, dp_obj, logger, config, tca, magnetorquer_manager, detumbler_manager,
        enable_detumble
    ):
        """
        Initialize the class object
        """
        self.dp_obj = dp_obj
        self.logger = logger
        self.running = False
        self.done = False
        self.config = config
        self.start_time = None
        self.magnetorquer_manager: MagnetorquerManager | None = magnetorquer_manager
        self.detumbler_manager: DetumblerManager | None = detumbler_manager
        self.enable_detumble = enable_detumble

    async def run(self):
        """
        Run the detumble sequence asynchronously
        """
        self.running = True
        self.done = (
            False  # Reset done flag for state re-entry (e.g., emergency detumble)
        )
        self.start_time = time.monotonic()

        while self.running:
            if not self.enable_detumble:
                self.logger.info(
                    f"[FSM: Detumble] Detumble is set to not be enabled, skipping."
                )
                self.done = True
                break
            # stop the dipole from the previous run, if there was one
            if self.magnetorquer_manager:
                self.magnetorquer_manager.stop_dipole_moments()
            # let it settle for the adjust frequency
            await asyncio.sleep(self.config.detumble_adjust_frequency)

            # Pull data from dp_obj
            mag_field = self.dp_obj.data["data_magnetometer_vector"]
            ang_vel_mag = self.dp_obj.data["data_imu_av_magnitude"]
            ang_vel_tuple = self.dp_obj.data["data_imu_av"]

            if mag_field is None and ang_vel_mag is None:
                self.logger.warning(
                    "[FSM: Detumble] Magnetometer and angular velocity unavailable; retrying next cycle."
                )
                continue
            # Check for timeout
            elapsed_time = time.monotonic() - self.start_time
            if elapsed_time >= self.config.detumble_max_time * 60:
                self.logger.info(
                    f"[FSM: Detumble] Timeout after {elapsed_time:.1f} seconds, curr ang_vel_mag: {ang_vel_mag}"
                )
                self.done = True
                break
            # If you know mag field but not ang vel, just let it pass
            if mag_field is not None and ang_vel_mag is None:
                self.logger.info(
                    "[FSM: Detumble] Ang Vel is None, Mag Field is not None."
                )
                continue
            # If Ang Vel is sufficintly stabilized, return
            if ang_vel_mag < self.config.detumble_stabilize_threshold:
                self.logger.info("[FSM: Detumble] Ang Vel Sufficiently Stabilized")
                self.done = True
                break
            # If Ang Vel is not stable, compute dipole
            # This is the quantity you want your magnetorquers to generate to stabilize HUCSat
            try:
                if self.detumbler_manager is None:
                    raise ValueError("detumbler_manager is None")
                dipole_vector = tuple(
                    self.detumbler_manager.magnetorquer_dipole(
                        tuple(mag_field), tuple(ang_vel_tuple)
                    )
                )
            except Exception as e:
                # most likely a divison by 0, which can happen if the numbers are small
                # in this case, the dipole would already be zero, so to 0
                self.logger.warning(
                    f"[FSM: Detumble] Error calculating dipole: {e}. Most likely because divison by 0/already close to 0.  Setting dipole to zero."
                )
                dipole_vector = (0.0, 0.0, 0.0)
            # Send result to magnetorquer
            if self.magnetorquer_manager:
                set_z_high = False
                # set Z high if we only don't have the magnetic field
                if mag_field is None and ang_vel_mag is not None:
                    self.logger.info(
                        "[FSM: Detumble] Mag Field is None, Ang Vel is not None, setting -Z to high (127)."
                    )
                    set_z_high = True
                self.magnetorquer_manager.set_dipole_moment(dipole_vector, set_z_high)
                # let it run for the adjust frequency
                await asyncio.sleep(self.config.detumble_adjust_frequency)

        # loop exited; reflect stopped state
        self.running = False

    def stop(self):
        """
        Used by FSM to manually stop run()
        """
        self.running = False
        # Immediately stop magnetorquers for safety
        if self.magnetorquer_manager:
            self.magnetorquer_manager.stop_dipole_moments()

    def is_done(self):
        """
        Checked by FSM to see if the run() completed on its own
        If it did complete, it shuts down the async task run()
        """
        return self.done
