"""This module provides the Config, which encapsulates the configuration
logic for the PySquared project. It loads, validates, and updates configuration
values from a JSON file, and distributes these values across the application.

Classes:
    Config: Handles loading, validating, and updating configuration values,
        including radio settings.

**Usage:**
```python
config = Config("config.json")
config.update_config("cubesat_name", "Cube1", temporary=False)
```
"""

import json

from .radio import RadioConfig


class Config:
    """
    Configuration handler for PySquared.

    Loads configuration from a JSON file, validates values, and provides
    methods to update configuration settings. Supports both temporary (RAM-only)
    and permanent (file-persisted) updates. Delegates radio-related validation
    and updates to the RadioConfig class. Jokes are loaded from a separate
    jokes.json file in the same directory as the config file.

    Attributes:
        config_file (str): Path to the configuration JSON file.
        radio (RadioConfig): Radio configuration handler.
        cubesat_name (str): Name of the cubesat.
        sleep_duration (int): Sleep duration in seconds.
        detumble_enable_z (bool): Enable detumbling on Z axis.
        detumble_enable_x (bool): Enable detumbling on X axis.
        detumble_enable_y (bool): Enable detumbling on Y axis.
        detumble_gain (float): Gain constant for detumble control law (default 1.0).
        jokes (list[str]): List of jokes for the cubesat (loaded from jokes.json).
        debug (bool): Debug mode flag.
        heating (bool): Heating system enabled flag.
        normal_temp (int): Normal operating temperature.
        normal_battery_temp (int): Normal battery temperature.
        normal_micro_temp (int): Normal microcontroller temperature.
        normal_charge_current (float): Normal charge current.
        normal_battery_voltage (float): Normal battery voltage.
        critical_battery_voltage (float): Critical battery voltage.
        reboot_time (int): Time before reboot in seconds.
        turbo_clock (bool): Turbo clock enabled flag.
        super_secret_code (str): Secret code for special operations.
        repeat_code (str): Code for repeated operations.
        longest_allowable_sleep_time (int): Maximum allowable sleep time.
        CONFIG_SCHEMA (dict): Validation schema for configuration keys.

    Methods:
        validate(key, value):
            Validates a configuration value against its schema.
        _save_config(key, value):
            Saves a configuration value to the JSON file.
        update_config(key, value, temporary):
            Updates a configuration value, either temporarily or permanently.
    """

    def __init__(self, config_path: str) -> None:
        """
        Initializes the Config object by loading values from the given JSON file.

        Args:
            config_path (str): Path to the configuration JSON file.

        Raises:
            FileNotFoundError: If the configuration file or jokes.json does not exist.
            json.JSONDecodeError: If the configuration file or jokes.json is not valid JSON.
        """

        self.config_file = config_path
        # parses json & assigns data to variables
        with open(self.config_file, "r") as f:
            json_data = json.loads(f.read())

        self.radio: RadioConfig = RadioConfig(json_data["radio"])
        self.cubesat_name: str = json_data["cubesat_name"]
        self.sleep_duration: int = json_data["sleep_duration"]
        self.detumble_enable_z: bool = json_data["detumble_enable_z"]
        self.detumble_enable_x: bool = json_data["detumble_enable_x"]
        self.detumble_enable_y: bool = json_data["detumble_enable_y"]

        # Load jokes from separate jokes.json file in the same directory
        # Extract directory from config_path using string operations (CircuitPython compatible)
        if "/" in config_path:
            config_dir = "/".join(config_path.split("/")[:-1])
        else:
            config_dir = "."

        # Construct jokes path
        if config_dir == ".":
            jokes_path = "jokes.json"
        else:
            jokes_path = config_dir + "/jokes.json"

        with open(jokes_path, "r") as f:
            self.jokes: list[str] = json.loads(f.read())

        self.debug: bool = json_data["debug"]
        self.heating: bool = json_data["heating"]
        self.normal_temp: int = json_data["normal_temp"]
        self.normal_battery_temp: int = json_data["normal_battery_temp"]
        self.normal_micro_temp: int = json_data["normal_micro_temp"]
        self.normal_charge_current: float = json_data["normal_charge_current"]
        self.normal_battery_voltage: float = json_data["normal_battery_voltage"]
        self.degraded_battery_voltage: float = json_data["degraded_battery_voltage"]
        self.critical_battery_voltage: float = json_data["critical_battery_voltage"]
        self.reboot_time: int = json_data["reboot_time"]
        self.turbo_clock: bool = json_data["turbo_clock"]
        self.super_secret_code: str = json_data["super_secret_code"]
        self.repeat_code: str = json_data["repeat_code"]
        self.longest_allowable_sleep_time: int = json_data[
            "longest_allowable_sleep_time"
        ]

        self.orient_payload_setting = json_data["orient_payload_setting"]
        self.orient_payload_periodic_time = json_data["orient_payload_periodic_time"]
        self.orient_light_threshold = json_data["orient_light_threshold"]
        self.orient_heat_duration = json_data["orient_heat_duration"]
        self.fsm_batt_threshold_orient = json_data["fsm_batt_threshold_orient"]
        self.fsm_batt_threshold_deploy = json_data["fsm_batt_threshold_deploy"]
        self.deploy_burn_duration = json_data["deploy_burn_duration"]
        self.deploy_max_attempts = json_data.get("deploy_max_attempts", 3)
        self.deploy_retry_delay = json_data.get("deploy_retry_delay", 60.0)
        self.detumble_adjust_frequency = json_data["detumble_adjust_frequency"]
        self.detumble_stabilize_threshold = json_data["detumble_stabilize_threshold"]
        self.detumble_max_time = json_data["detumble_max_time"]
        self.detumble_gain = json_data["detumble_gain"]
        self.watchdog_reset_sleep = json_data["watchdog_reset_sleep"]
        self.except_reset_allowed_attemps = json_data["except_reset_allowed_attemps"]
        self.sleep_if_yet_booted_count = json_data["sleep_if_yet_booted_count"]
        self.sleep_if_yet_deployed_count = json_data["sleep_if_yet_deployed_count"]
        self.cdh_listen_command_timeout = json_data["cdh_listen_command_timeout"]
        self.enable_detumble = json_data["enable_detumble"]

        self.CONFIG_SCHEMA = {
            "cubesat_name": {"type": str, "min_length": 1, "max_length": 10},
            "super_secret_code": {"type": bytes, "min": 1, "max": 24},
            "repeat_code": {"type": bytes, "min": 1, "max": 4},
            "normal_charge_current": {"type": float, "min": 0.0, "max": 2000.0},
            "normal_battery_voltage": {"type": float, "min": 6.0, "max": 8.4},
            "degraded_battery_voltage": {"type": float, "min": 5.4, "max": 8.0},
            "critical_battery_voltage": {"type": float, "min": 5.4, "max": 7.2},
            "sleep_duration": {"type": int, "min": 1, "max": 86400},
            "normal_temp": {"type": int, "min": 5, "max": 40},
            "normal_battery_temp": {"type": int, "min": 1, "max": 35},
            "normal_micro_temp": {"type": int, "min": 1, "max": 50},
            "reboot_time": {"type": int, "min": 3600, "max": 604800},
            "detumble_enable_z": {"type": bool},
            "detumble_enable_x": {"type": bool},
            "detumble_enable_y": {"type": bool},
            "debug": {"type": bool},
            "heating": {"type": bool},
            "turbo_clock": {"type": bool},
            "orient_payload_setting": {"type": int},
            "orient_payload_periodic_time": {"type": float},
            "orient_light_threshold": {"type": float},
            "orient_heat_duration": {"type": float},
            "fsm_batt_threshold_orient": {"type": float},
            "fsm_batt_threshold_deploy": {"type": float},
            "deploy_burn_duration": {"type": float},
            "deploy_max_attempts": {"type": int},
            "deploy_retry_delay": {"type": float},
            "detumble_adjust_frequency": {"type": float},
            "detumble_stabilize_threshold": {"type": float},
            "detumble_max_time": {"type": float},
            "detumble_gain": {"type": float},
            "watchdog_reset_sleep": {"type": int},
            "except_reset_allowed_attemps": {"type": int},
            "sleep_if_yet_booted_count": {"type": int},
            "sleep_if_yet_deployed_count": {"type": int},
            "cdh_listen_command_timeout": {"type": int},
            "enable_detumble": {"type": bool},
        }

    # validates values from input
    def validate(self, key: str, value) -> None:
        """
        Validates a configuration value against its schema.

        Args:
            key (str): The configuration key to validate.
            value: The value to validate.

        Raises:
            TypeError: If the value is not of the expected type.
            ValueError: If the value is out of the allowed range or length.
        """

        if key in self.CONFIG_SCHEMA:
            schema = self.CONFIG_SCHEMA[key]
            expected_type = schema["type"]

            # checks value is of same type; also covers bools
            if not isinstance(value, expected_type):
                raise TypeError

            # checks int, float, and bytes range
            if isinstance(value, (int, float, bytes)):
                if "min" in schema and value < schema["min"]:
                    raise ValueError
                if "max" in schema and value > schema["max"]:
                    raise ValueError

            # checks string range
            else:
                if "min_length" in schema and len(value) < schema["min_length"]:
                    raise ValueError
                if "max_length" in schema and len(value) > schema["max_length"]:
                    raise ValueError
        else:
            # Delegate radio-related validation to RadioConfig
            self.radio.validate(key, value)

    # permanently updates values
    def _save_config(self, key: str, value) -> None:
        """
        Saves a configuration value to the JSON file.

        Args:
            key (str): The configuration key to save.
            value: The value to save.
        """

        with open(self.config_file, "r") as f:
            json_data = json.loads(f.read())

        json_data[key] = value

        with open(self.config_file, "w") as f:
            f.write(json.dumps(json_data))

    # handles temp or permanent updates
    def update_config(self, key: str, value, temporary: bool) -> None:
        """
        Updates a configuration value, either temporarily (RAM only) or permanently (persisted to file).

        Args:
            key (str): The configuration key to update.
            value: The new value to set.
            temporary (bool): If True, update only in RAM; if False, persist to file.

        Raises:
            TypeError: If the value is not of the expected type.
            ValueError: If the value is out of the allowed range or length.
        """

        # validates key and value and should raise error if any
        if key in self.CONFIG_SCHEMA:
            self.validate(key, value)
            # if permanent, saves to config
            if not temporary:
                self._save_config(key, value)
            # updates RAM
            setattr(self, key, value)
        else:
            # Delegate radio-related validation to RadioConfig
            self.radio.validate(key, value)
            # if permanent, saves to config
            if not temporary:
                with open(self.config_file, "r") as f:
                    json_data = json.loads(f.read())
                if key in self.radio.RADIO_SCHEMA:
                    json_data["radio"][key] = value
                elif key in self.radio.fsk.FSK_SCHEMA:
                    json_data["radio"]["fsk"][key] = value
                else:  # key is in self.radio.lora.LORA_SCHEMA
                    json_data["radio"]["lora"][key] = value
                with open(self.config_file, "w") as f:
                    f.write(json.dumps(json_data))
            # updates RAM
            if key in self.radio.RADIO_SCHEMA:
                setattr(self.radio, key, value)
            elif key in self.radio.fsk.FSK_SCHEMA:
                setattr(self.radio.fsk, key, value)
            else:  # key is in self.radio.lora.LORA_SCHEMA
                setattr(self.radio.lora, key, value)
