# data_process.py


# ++++++++++++++++++ Imports and Installs ++++++++++++++++++ #
import asyncio
import time


# ++++++++++++++++++++ Class Definition ++++++++++++++++++++ #
class DataProcess:
    """
    Class with functions to grab all the data that we need
    """

    def __init__(self, magnetometer, imu, battery_power_monitor):
        self.protos_power_monitor = battery_power_monitor  # INA219Manager
        self.protos_imu = imu  # LSM6DSOXManager
        self.protos_magnetometer = magnetometer  # LIS2MDLManager
        self.last_imu_time = time.monotonic()
        self.running = True
        self.data = {
            "data_batt_volt": None,  # battery voltage in V (None = sensor failure)
            "data_imu_av": None,  # imu angular velocity [ωx, ωy, ωz] in rad/s (None = sensor failure)
            "data_imu_av_magnitude": None,  # imu angular velocity magnitude in rad/s (None = sensor failure)
            "data_imu_acc": None,  # imu acceleration [ax, ay, az] in m/s² (None = sensor failure)
            "data_magnetometer_vector": None,  # magnetometer vector in μT (None = sensor failure)
            "data_magnetometer_magnitude": None,  # magnetometer magnitude in μT (None = sensor failure)
        }

    def start_run_all_data(self):
        """
        This schedules a coroutine (a program that can be paused/resumed infinitely,
        allowing for scheduled concurrency).  Specifically, it schedules the
        run_all_data function
        """
        try:
            asyncio.create_task(self.run_all_data())
        except RuntimeError as e:
            print("Asyncio loop already running:", e)

    async def run_all_data(self):
        """
        Run all the data-gathering functions in an infinite loop.
        """
        await asyncio.gather(
            self.get_data_battery(),
            self.get_data_imu_av(),
            self.get_data_imu_acc(),
            self.get_data_magnetometer_vector(),
        )

    async def get_data_battery(self):
        """
        Get battery voltage (bv)
        """
        while self.running:
            try:
                if self.protos_power_monitor is None:
                    # No sensor available
                    self.data["data_batt_volt"] = None
                else:
                    voltage = self.protos_power_monitor.get_bus_voltage()._value
                    self.data["data_batt_volt"] = voltage
            except Exception as e:
                # On sensor read failure, keep last known value and log error
                print(f"[ERROR] Battery voltage read failed: {e}")
            await asyncio.sleep(1)

    async def get_data_imu_av(self):
        """
        Get data_imu_av and data_imu_av_magnitude
        """
        while self.running:
            try:
                if self.protos_imu:
                    imu_acc_data = list(self.protos_imu.get_angular_velocity().value)
                    self.data["data_imu_av"] = imu_acc_data
                    # compute the magnitude of angular velocity
                    ωx, ωy, ωz = imu_acc_data
                    magnitude = (ωx**2 + ωy**2 + ωz**2) ** 0.5
                    self.data["data_imu_av_magnitude"] = magnitude
                else:
                    self.data["data_imu_av"] = None
                    self.data["data_imu_av_magnitude"] = None
            except Exception as e:
                # On sensor read failure, keep last known value and log error
                print(f"[ERROR] IMU angular velocity read failed: {e}")
            await asyncio.sleep(1)

    async def get_data_imu_acc(self):
        """
        Get imu acceleration
        """
        while self.running:
            try:
                if self.protos_imu:
                    imu_acc_data = list(self.protos_imu.get_acceleration().value)
                    self.data["data_imu_acc"] = imu_acc_data
                else:
                    self.data["data_imu_acc"] = None
            except Exception as e:
                # On sensor read failure, keep last known value and log error
                print(f"[ERROR] IMU acceleration read failed: {e}")
            await asyncio.sleep(1)

    async def get_data_magnetometer_vector(self):
        """
        Get magnetometer vector and magnitude
        """
        while self.running:
            try:
                if self.protos_magnetometer:
                    magnetometer_data = list(
                        self.protos_magnetometer.get_magnetic_field().value
                    )
                    # Compute magnitude for B-dot algorithm
                    bx, by, bz = magnetometer_data
                    magnitude = (bx**2 + by**2 + bz**2) ** 0.5
                    self.data["data_magnetometer_vector"] = magnetometer_data
                    self.data["data_magnetometer_magnitude"] = magnitude
                else:
                    self.data["data_magnetometer_vector"] = None
                    self.data["data_magnetometer_magnitude"] = None
            except Exception as e:
                # On sensor read failure, keep last known value and log error
                print(f"[ERROR] Magnetometer read failed: {e}")
            await asyncio.sleep(1)
