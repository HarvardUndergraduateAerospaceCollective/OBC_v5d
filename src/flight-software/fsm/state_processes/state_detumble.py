# state_detumble.py



# ++++++++++++++ Imports/Installs ++++++++++++++ #
import time
import asyncio
from lib.pysquared.detumbler_manager import DetumblerManager
from lib.pysquared.hardware.magnetorquer.manager.magnetorquer import MagnetorquerManager


# ++++++++++++++ Functions: Helper ++++++++++++++ #
class StateDetumble:
    def __init__(self, dp_obj, logger, config,
                 tca, magnetorquer_manager, detumbler_manager):
        """
        Initialize the class object
        """
        self.dp_obj = dp_obj
        self.logger = logger
        self.running = False
        self.done = False
        self.config = config
        self.start_time = None
        self.magnetorquer_manager : MagnetorquerManager | None = magnetorquer_manager                          
        self.detumbler_manager = detumbler_manager

    async def run(self):
        """
        Run the deployment sequence asynchronously
        """
        self.running = True
        self.start_time = time.monotonic() 

        while self.running:
            await asyncio.sleep(self.config.detumble_adjust_frequency)

            # Pull data from dp_obj
            mag_field = self.dp_obj.data["data_magnetometer_vector"]
            ang_vel_mag = self.dp_obj.data["data_imu_av_magnitude"]
            ang_vel_tuple = self.dp_obj.data["data_imu_av"]

            # Check for timeout
            elapsed_time = time.monotonic() - self.start_time
            if elapsed_time >= self.config.detumble_max_time:
                self.logger.info(f"[FSM: Detumble] Timeout after {elapsed_time:.1f} seconds, curr ang_vel_mag: {ang_vel_mag}")
                self.done = True
                break

            # Verify data is present, if not, skip calculations
            if mag_field is None or ang_vel_mag is None:
                note1 = "[FSM: Detumble] Waiting on Mag Field"
                note2 = "[FSM: Detumble] Waiting on Ang Vel"
                self.logger.info(note1) if mag_field is None else self.logger.info(note2)
                continue
            # If Ang Vel is sufficintly stabilized, return
            if ang_vel_mag < self.config.detumble_stabilize_threshold:
                self.logger.info("[FSM: Detumble] Ang Vel Sufficiently Stabilized")
                self.done = True
                continue
            # If Ang Vel is not stable, compute dipole
            # This is the quantity you want your magnetorquers to generate to stabilize CubeSAT
            try:
                dipole_vector = tuple(self.detumbler_manager.magnetorquer_dipole(tuple(mag_field), tuple(ang_vel_tuple)))
            except Exception as e:
                # most likely a divison by 0, which can happen if the numbers are small
                # in this case, the dipole would already be zero, so to 0
                self.logger.warning(f"[FSM: Detumble] Error calculating dipole: {e}. Most likely because divison by 0/already close to 0.  Setting dipole to zero.")
                dipole_vector = (0.0,0.0,0.0)
            # Send result to magnetorquer
            if self.magnetorquer_manager:
                self.magnetorquer_manager.set_dipole_moment(dipole_vector)

    def stop(self):
        """
        Used by FSM to manually stop run()
        """
        self.running = False

    def is_done(self):
        """
        Checked by FSM to see if the run() completed on its own
        If it did complete, it shuts down the async task run()
        """
        return self.done