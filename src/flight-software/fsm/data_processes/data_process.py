# data_process.py



# ++++++++++++++++++ Imports and Installs ++++++++++++++++++ #
import time
import asyncio


# ++++++++++++++++++++ Class Definition ++++++++++++++++++++ #
class DataProcess():
    """
    Class with functions to grab all the data that we need
    """

    def __init__(self, magnetometer, imu, battery_power_monitor):
        self.protos_power_monitor = battery_power_monitor   # INA219Manager
        self.protos_imu = imu                               # LSM6DSOXManager                
        self.protos_magnetometer = magnetometer             # LIS2MDLManager
        self.last_imu_time = time.monotonic()
        self.running = True
        self.data = {
            "data_batt_volt" : 0.0,                     # battery voltage
            "data_imu_av" : [0.0,0.0,0.0],              # imu angular velocity [ax, ay, az] in rad/s²
            "data_imu_av_magnitude" : 0.0,              # imu angular velocity magnitude (Euclidian norm aka length of data_imu_av vector)
            "data_imu_acc" : [0.0,0.0,0.0],             # imu acceleration [ax, ay, az] in m/s²" : [0.0,0.0,0.0],             # imu position
            "data_magnetometer_vector" : [0.0,0.0,0.0]  # magnetometer vector
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
            voltage = self.protos_power_monitor.get_bus_voltage()._value
            self.data["data_batt_volt"] = voltage
            await asyncio.sleep(1)

    async def get_data_imu_av(self):
        """
        Get data_imu_av and data_imu_av_magnitude
        """
        while self.running:
            imu_acc_data = list(self.protos_imu.get_angular_velocity().value)
            self.data["data_imu_av"] = imu_acc_data
            # compute the magnitude of angular velocity
            ωx, ωy, ωz = imu_acc_data
            magnitude = (ωx**2 + ωy**2 + ωz**2) ** 0.5
            self.data["data_imu_av_magnitude"] = magnitude
            await asyncio.sleep(1)
    
    async def get_data_imu_acc(self):
        """
        Get imu acceleration
        """
        while self.running:
            imu_acc_data = list(self.protos_imu.get_acceleration().value)
            self.data["data_imu_acc"] = imu_acc_data
            await asyncio.sleep(1)

    async def get_data_magnetometer_vector(self):
        """
        Get magnetometer vector
        """
        while self.running:
            magnetometer_data = list(self.protos_magnetometer.get_magnetic_field().value)
            self.data["data_magnetometer_vector"] = magnetometer_data
            await asyncio.sleep(1)

    