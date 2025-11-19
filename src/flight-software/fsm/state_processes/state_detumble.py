# state_detumble.py



# ++++++++++++++ Imports/Installs ++++++++++++++ #
import time
import asyncio
from lib.pysquared.detumbler_manager import DetumblerManager
from lib.pysquared.hardware.magnetorquer.manager.magnetorquer import MagnetorquerManager


# ++++++++++++++ Functions: Helper ++++++++++++++ #
class StateDetumble:
    def __init__(self, dp_obj, logger, tca):
        """
        Initialize the class object
        """
        self.dp_obj = dp_obj
        self.logger = logger
        self.running = False
        self.done = False
        self.detumble_frequency = 5 # in seconds; how long to wait between data reads
        self.detumble_threshold = 0.05
        self.max_time = 5400        # 90 minutes (~1 orbit) to try and stabilize
        self.start_time = None
        self.magnetorquer_manager = None
        """
        TODO:
        MagnetorquerManager( logger=self.logger,
                                            i2c_addr        =0x5a,
                                            addr_x_plus     =tca[0],
                                            addr_x_minus    =tca[1],
                                            addr_y_plus     =tca[2],
                                            addr_y_minus    =tca[3],
                                            addr_z_minus    =tca[4])
        """                               
        self.detumbler_manager = DetumblerManager(gain=1.0)

    async def run(self):
        """
        Run the deployment sequence asynchronously
        """
        self.running = True
        self.start_time = time.monotonic() 

        while self.running:
            await asyncio.sleep(self.detumble_frequency)

            # Pull data from dp_obj
            mag_field = self.dp_obj.data["data_magnetometer_vector"]
            ang_vel_mag = self.dp_obj.data["data_imu_av_magnitude"]
            ang_vel_tuple = self.dp_obj.data["data_imu_av"]

            # Check for timeout
            elapsed_time = time.monotonic() - self.start_time
            if elapsed_time >= self.max_time:
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
            if ang_vel_mag < self.detumble_threshold:
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