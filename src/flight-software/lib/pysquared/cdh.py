"""This module provides the CommandDataHandler for managing and processing commands.

This module is responsible for handling commands received by the satellite. It
includes command parsing, validation, execution, and handling of radio
communications. The CommandDataHandler class is the main entry point for this
functionality.

**Usage:**
```python
logger = Logger()
config = Config("config.json")
packet_manager = PacketManager(logger, radio)
cdh = CommandDataHandler(logger, config, packet_manager)
cdh.listen_for_commands(timeout=60)
```
"""

import json
import random
import time
import traceback

import microcontroller

from .config.config import Config
from .config.jokes_config import JokesConfig
from .hardware.radio.packetizer.packet_manager import PacketManager
from .logger import Logger


class CommandDataHandler:
    """Handles command parsing, validation, and execution for the satellite."""

    command_reset: str = "reset"
    command_exec: str = "exec"
    command_change_radio_modulation: str = "change_radio_modulation"
    command_send_joke: str = "send_joke"
    command_get_counter: str = "get_counter"
    command_orient_payload: str = "orient_payload"
    command_orient_payload_setting: str = "orient_payload_setting"
    command_change_orient_payload_periodic_time: str = "orient_payload_periodic_time"
    command_change_orient_light_threshold: str = "orient_light_threshold"
    command_change_orient_heat_duration: str = "orient_heat_duration"
    command_change_fsm_batt_threshold_orient : str = "fsm_batt_threshold_orient"
    command_change_fsm_batt_threshold_deploy : str = "fsm_batt_threshold_deploy"
    command_change_deploy_burn_duration : str = "deploy_burn_duration"
    command_change_detumble_adjust_frequency : str = "detumble_adjust_frequency"
    command_change_detumble_stabilize_threshold : str = "detumble_stabilize_threshold"
    command_change_detumble_max_time : str = "detumble_max_time"
    command_change_critical_battery_voltage : str = "critical_battery_voltage"
    command_change_degraded_battery_voltage : str = "degraded_battery_voltage"
    command_change_sleep_if_yet_booted_count : str = "sleep_if_yet_booted_count"
    command_change_sleep_if_yet_deployed_count : str = "sleep_if_yet_deployed_count"
    command_change_cdh_listen_command_timeout : str = "cdh_listen_command_timeout"
    command_change_watchdog_reset_sleep : str = "watchdog_reset_sleep"
    command_change_except_reset_allowed_attemps : str = "except_reset_allowed_attemps"
    
    oscar_password: str = "Hello World!"  # Default password for OSCAR commands

    def __init__(
        self,
        logger: Logger,
        config: Config,
        packet_manager: PacketManager,
        jokes_config: JokesConfig,
        send_delay: float = 0.2,
    ) -> None:
        """Initializes the CommandDataHandler.

        Args:
            logger: The logger to use.
            config: The configuration to use.
            packet_manager: The packet manager to use for sending and receiving data.
            send_delay: The delay between sending an acknowledgement and the response.
        """
        self._log: Logger = logger
        self._config: Config = config
        self._jokes_config: JokesConfig = jokes_config
        self._packet_manager: PacketManager = packet_manager
        self._send_delay: float = send_delay

    def listen_for_commands(self, timeout: int) -> None:
        """Listens for commands from the radio and handles them.

        Args:
            timeout: The time in seconds to listen for commands.
        """
        self._log.debug("Listening for commands...", timeout=timeout)

        json_bytes = self._packet_manager.listen(timeout)
        if json_bytes is None:
            return

        try:
            json_str = json_bytes.decode("utf-8")

            msg: dict[str, str] = json.loads(json_str)

            # Check for OSCAR password first
            if msg.get("password") == self.oscar_password:
                self._log.debug("OSCAR command received", msg=msg)
                cmd = msg.get("command")
                if cmd is None:
                    self._log.warning("No OSCAR command found in message", msg=msg)
                    self._packet_manager.send(
                        f"No OSCAR command found in message: {msg}".encode("utf-8")
                    )
                    return

                args: list[str] = []
                raw_args = msg.get("args")
                if isinstance(raw_args, list):
                    args: list[str] = raw_args

                # Delay to give the ground station time to switch to listening mode
                time.sleep(self._send_delay)
                self._packet_manager.send_acknowledgement()

                self.oscar_command(cmd, args)
                return

            # If message has password field, check it
            if msg.get("password") != self._config.super_secret_code:
                self._log.debug(
                    "Invalid password in message",
                    msg=msg,
                )
                return

            if msg.get("name") != self._config.cubesat_name:
                self._log.debug(
                    "Satellite name mismatch in message",
                    msg=msg,
                )
                return

            # If message has command field, execute the command
            cmd = msg.get("command")
            if cmd is None:
                self._log.warning("No command found in message", msg=msg)
                self._packet_manager.send(
                    f"No command found in message: {msg}".encode("utf-8")
                )
                return

            args: list[str] = []
            raw_args = msg.get("args")
            if isinstance(raw_args, list):
                args: list[str] = raw_args

            self._log.debug("Received command message", cmd=cmd, args=args)

            # Delay to give the ground station time to switch to listening mode
            time.sleep(self._send_delay)
            self._packet_manager.send_acknowledgement()

            self._log.debug("Sent Acknowledgement", cmd=cmd, args=args)

            if cmd == self.command_orient_payload:
                self.set_orient_payload(args)
            elif cmd == self.command_orient_payload_setting:
                self.change_orient_payload_setting(args)
            elif cmd == self.command_change_orient_payload_periodic_time:
                self.change_orient_payload_periodic_time(args)
            elif cmd == self.command_change_orient_light_threshold:
                self.change_orient_light_threshold(args)
            elif cmd == self.command_change_orient_heat_duration:
                self.change_orient_heat_duration(args)
            elif cmd == self.command_change_fsm_batt_threshold_orient:
                self.change_fsm_batt_threshold_orient(args)
            elif cmd == self.command_change_fsm_batt_threshold_deploy:
                self.change_fsm_batt_threshold_deploy(args)
            elif cmd == self.command_change_deploy_burn_duration:
                self.change_deploy_burn_duration(args)
            elif cmd == self.command_change_detumble_adjust_frequency:
                self.change_detumble_adjust_frequency(args)
            elif cmd == self.command_change_detumble_stabilize_threshold:
                self.change_detumble_stabilize_threshold(args)
            elif cmd == self.command_change_detumble_max_time:
                self.change_detumble_max_time(args)
            elif cmd == self.command_change_critical_battery_voltage:
                self.change_critical_battery_voltage(args)
            elif cmd == self.command_change_degraded_battery_voltage:
                self.change_degraded_battery_voltage(args)
            elif cmd == self.command_change_sleep_if_yet_booted_count:
                self.change_sleep_if_yet_booted_count(args)
            elif cmd == self.command_change_sleep_if_yet_deployed_count:
                self.change_sleep_if_yet_deployed_count(args)
            elif cmd == self.command_change_cdh_listen_command_timeout:
                self.change_cdh_listen_command_timeout(args)
            elif cmd == self.command_change_watchdog_reset_sleep:
                self.change_watchdog_reset_sleep(args)
            elif cmd == self.command_change_except_reset_allowed_attemps:
                self.change_except_reset_allowed_attemps(args)
            elif cmd == self.command_exec:
                self.exec_command(args)
            elif cmd == self.command_reset:
                self.reset()
            elif cmd == self.command_change_radio_modulation:
                self.change_radio_modulation(args)
            elif cmd == self.command_send_joke:
                self.send_joke()
            else:
                self._log.warning("Unknown command received", cmd=cmd)
                self._packet_manager.send(
                    f"Unknown command received: {cmd}".encode("utf-8")
                )

        except Exception as e:
            self._log.error("Failed to process command message", err=e)
            self._packet_manager.send(
                f"Failed to process command message: {traceback.format_exception(e)}".encode(
                    "utf-8"
                )
            )
            return

    def send_joke(self) -> None:
        """Sends a random joke from the config."""
        joke = random.choice(self._jokes_config.jokes)
        self._log.info("Sending joke", joke=joke)
        self._packet_manager.send(joke.encode("utf-8"))

    def change_radio_modulation(self, args: list[str]) -> None:
        """Changes the radio modulation.

        Args:
            args: A list of arguments, the first item must be the new modulation. All other items in the args list are ignored.
        """
        modulation = "UNSET"

        if len(args) < 1:
            self._log.warning("No modulation specified")
            self._packet_manager.send(
                "No modulation specified. Please provide a modulation type.".encode(
                    "utf-8"
                )
            )
            return

        modulation = args[0]

        try:
            self._config.update_config("modulation", modulation, temporary=False)
            self._log.info("Radio modulation changed", modulation=modulation)
            self._packet_manager.send(
                f"Radio modulation changed: {modulation}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change radio modulation", err=e)
            self._packet_manager.send(
                f"Failed to change radio modulation: {e}".encode("utf-8")
            )

    def change_orient_payload_setting(self, args: list[str]) -> None:
        """Changes the orient payload setting.

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        orient_payload_setting = self._config.orient_payload_setting

        if len(args) < 1:
            self._log.warning("No orient payload setting specified")
            self._packet_manager.send(
                "No orient payload setting specified. Please provide an integer, 1 or 0.".encode(
                    "utf-8"
                )
            )
            return

        orient_payload_setting = int(args[0])

        try:
            self._config.update_config("orient_payload_setting", orient_payload_setting, temporary=False)
            self._log.info("Orient payload setting changed")
            self._packet_manager.send(
                f"Orient payload setting time changed: {orient_payload_setting}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change orient periodic time", err=e)
            self._packet_manager.send(
                f"Failed to change orient periodic time: {e}".encode("utf-8")
            )

    def change_orient_payload_periodic_time(self, args: list[str]) -> None:
        """Changes the orient payload periodic time (in minutes) when under the Sun.

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        orient_payload_periodic_time = self._config.orient_payload_periodic_time

        if len(args) < 1:
            self._log.warning("No payload periodic time specified")
            self._packet_manager.send(
                "No payload periodic time specified. Please provide an integer (in minutes).".encode(
                    "utf-8"
                )
            )
            return

        orient_payload_periodic_time = float(args[0])

        try:
            self._config.update_config("orient_payload_periodic_time", orient_payload_periodic_time, temporary=False)
            self._log.info("Orient periodic time changed")
            self._packet_manager.send(
                f"Orient periodic time changed: {orient_payload_periodic_time}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change orient periodic time", err=e)
            self._packet_manager.send(
                f"Failed to change orient periodic time: {e}".encode("utf-8")
            )

    def change_orient_light_threshold(self, args: list[str]) -> None:
        """Changes the orient light threshold (in units).

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        orient_light_threshold = self._config.orient_light_threshold

        if len(args) < 1:
            self._log.warning("No light threshold time specified")
            self._packet_manager.send(
                "No light threshold specified. Please provide an integer.".encode(
                    "utf-8"
                )
            )
            return

        orient_light_threshold = float(args[0])

        try:
            self._config.update_config("orient_light_threshold", orient_light_threshold, temporary=False)
            self._log.info("Orient light threhsold changed")
            self._packet_manager.send(
                f"Orient light threhsold changed: {orient_light_threshold}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change orient light threshold", err=e)
            self._packet_manager.send(
                f"Failed to change orient light threshold: {e}".encode("utf-8")
            )

    def change_orient_heat_duration(self, args: list[str]) -> None:
        """Changes the orient heat duration (in seconds).

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        orient_heat_duration = self._config.orient_heat_duration

        if len(args) < 1:
            self._log.warning("No orient heat duration specified")
            self._packet_manager.send(
                "No orient heat duration specified. Please provide an integer (in seconds).".encode(
                    "utf-8"
                )
            )
            return

        orient_heat_duration = float(args[0])

        try:
            self._config.update_config("orient_heat_duration", orient_heat_duration, temporary=False)
            self._log.info("Orient heat duration changed")
            self._packet_manager.send(
                f"Orient heat duration changed: {orient_heat_duration}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change orient heat duration", err=e)
            self._packet_manager.send(
                f"Failed to change orient heat duration: {e}".encode("utf-8")
            )

    def change_fsm_batt_threshold_orient(self, args: list[str]) -> None:
        """Changes the battery threshold for orienting.

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        fsm_batt_threshold_orient = self._config.fsm_batt_threshold_orient

        if len(args) < 1:
            self._log.warning("No orient battery threshold specified")
            self._packet_manager.send(
                "No orient battery threshold specified.".encode(
                    "utf-8"
                )
            )
            return

        fsm_batt_threshold_orient = float(args[0])

        # OK if too high- will be ignored

        try:
            self._config.update_config("fsm_batt_threshold_orient", fsm_batt_threshold_orient, temporary=False)
            self._log.info("Orient battery threshold changed")
            self._packet_manager.send(
                f"Orient battery threshold changed: {fsm_batt_threshold_orient}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change orient battery threshold", err=e)
            self._packet_manager.send(
                f"Failed to change orient battery threshold changed: {e}".encode("utf-8")
            )

    def change_fsm_batt_threshold_deploy(self, args: list[str]) -> None:
        """Changes the battery threshold for deploy.

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        fsm_batt_threshold_deploy = self._config.fsm_batt_threshold_deploy

        if len(args) < 1:
            self._log.warning("No deploy battery threshold specified")
            self._packet_manager.send(
                "No deploy battery threshold specified.".encode(
                    "utf-8"
                )
            )
            return

        fsm_batt_threshold_deploy = float(args[0])

        # OK if too high- will be ignored

        try:
            self._config.update_config("fsm_batt_threshold_deploy", fsm_batt_threshold_deploy, temporary=False)
            self._log.info("Deploy battery threshold changed")
            self._packet_manager.send(
                f"Deploy battery threshold changed: {fsm_batt_threshold_deploy}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change deploy battery threshold", err=e)
            self._packet_manager.send(
                f"Failed to change deploy battery threshold changed: {e}".encode("utf-8")
            )

    def change_deploy_burn_duration(self, args: list[str]) -> None:
        """Changes the burn duration for deploy (in seconds).

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        deploy_burn_duration = self._config.deploy_burn_duration

        if len(args) < 1:
            self._log.warning("No deploy burn duration specified")
            self._packet_manager.send(
                "No deploy burn duration specified.".encode(
                    "utf-8"
                )
            )
            return

        deploy_burn_duration = float(args[0])

        try:
            self._config.update_config("deploy_burn_duration", deploy_burn_duration, temporary=False)
            self._log.info("Deploy burn duration changed")
            self._packet_manager.send(
                f"Deploy burn duration changed: {deploy_burn_duration}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change deploy burn duration", err=e)
            self._packet_manager.send(
                f"Failed to change deploy burn duration: {e}".encode("utf-8")
            )

    def change_detumble_adjust_frequency(self, args: list[str]) -> None:
        """Changes the adjustment frequency for detumble (in seconds).

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        detumble_adjust_frequency = self._config.detumble_adjust_frequency

        if len(args) < 1:
            self._log.warning("No detumble_adjust_frequency specified")
            self._packet_manager.send(
                "No detumble_adjust_frequency specified.".encode(
                    "utf-8"
                )
            )
            return

        detumble_adjust_frequency = float(args[0])

        try:
            self._config.update_config("detumble_adjust_frequency", detumble_adjust_frequency, temporary=False)
            self._log.info("Detumble adjust frequency changed")
            self._packet_manager.send(
                f"Detumble adjust frequency changed: {detumble_adjust_frequency}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change detumble adjust frequency", err=e)
            self._packet_manager.send(
                f"Failed to change detumble adjust frequency: {e}".encode("utf-8")
            )

    def change_detumble_stabilize_threshold(self, args: list[str]) -> None:
        """Changes the stabilize threshold for detumble (based on angular velocity magnitude)

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        detumble_stabilize_threshold = self._config.detumble_stabilize_threshold

        if len(args) < 1:
            self._log.warning("No detumble stabilize threshold specified")
            self._packet_manager.send(
                "No detumble stabilize threshold specified.".encode(
                    "utf-8"
                )
            )
            return

        detumble_stabilize_threshold = float(args[0])

        try:
            self._config.update_config("detumble_stabilize_threshold", detumble_stabilize_threshold, temporary=False)
            self._log.info("Detumble stabilize threshold changed")
            self._packet_manager.send(
                f"Detumble stabilize threshold changed: {detumble_stabilize_threshold}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change detumble stabilize threshold", err=e)
            self._packet_manager.send(
                f"Failed to change detumble stabilize threshold: {e}".encode("utf-8")
            )

    def change_detumble_max_time(self, args: list[str]) -> None:
        """Changes the max time for detumble (in seconds).

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        detumble_max_time = self._config.detumble_max_time

        if len(args) < 1:
            self._log.warning("No detumble max time specified")
            self._packet_manager.send(
                "No detumble max time specified.".encode(
                    "utf-8"
                )
            )
            return

        detumble_max_time = float(args[0])

        try:
            self._config.update_config("detumble_max_time", detumble_max_time, temporary=False)
            self._log.info("Detumble max time changed")
            self._packet_manager.send(
                f"Detumble max time changed: {detumble_max_time}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change detumble max time", err=e)
            self._packet_manager.send(
                f"Failed to change detumble max time: {e}".encode("utf-8")
            )
    
    def change_critical_battery_voltage(self, args: list[str]) -> None:
        """Changes the critical battery voltage (in Watts)

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        critical_battery_voltage = self._config.critical_battery_voltage

        if len(args) < 1:
            self._log.warning("No critical battery voltage specified")
            self._packet_manager.send(
                "No critical battery voltage specified.".encode(
                    "utf-8"
                )
            )
            return

        critical_battery_voltage = float(args[0])

        try:
            self._config.update_config("critical_battery_voltage", critical_battery_voltage, temporary=False)
            self._log.info("Critical battery voltage changed")
            self._packet_manager.send(
                f"Critical battery voltage changed: {critical_battery_voltage}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change critical battery voltage", err=e)
            self._packet_manager.send(
                f"Failed to change critical battery voltage: {e}".encode("utf-8")
            )
    
    def change_degraded_battery_voltage(self, args: list[str]) -> None:
        """Changes the degraded battery voltage (in Watts)

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        degraded_battery_voltage = self._config.degraded_battery_voltage

        if len(args) < 1:
            self._log.warning("No degraded battery voltage specified")
            self._packet_manager.send(
                "No degraded battery voltage specified.".encode(
                    "utf-8"
                )
            )
            return

        degraded_battery_voltage = float(args[0])

        try:
            self._config.update_config("degraded_battery_voltage", degraded_battery_voltage, temporary=False)
            self._log.info("Degraded battery voltage changed")
            self._packet_manager.send(
                f"Degraded battery voltage changed: {degraded_battery_voltage}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change degraded battery voltage", err=e)
            self._packet_manager.send(
                f"Failed to change degraded battery voltage: {e}".encode("utf-8")
            )

    def change_sleep_if_yet_booted_count(self, args: list[str]) -> None:
        """Changes the 30 min sleep condition based on boot count

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        sleep_if_yet_booted_count = self._config.sleep_if_yet_booted_count

        if len(args) < 1:
            self._log.warning("No sleep condition for boot count specified")
            self._packet_manager.send(
                "No sleep condition for boot count specified.".encode(
                    "utf-8"
                )
            )
            return

        sleep_if_yet_booted_count = int(args[0])

        try:
            self._config.update_config("sleep_if_yet_booted_count", sleep_if_yet_booted_count, temporary=False)
            self._log.info("Sleep if yet booted value changed")
            self._packet_manager.send(
                f"Sleep if yet booted value changed: {sleep_if_yet_booted_count}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change sleep condition for boot count", err=e)
            self._packet_manager.send(
                f"Failed to change sleep condition for boot count: {e}".encode("utf-8")
            )

    def change_sleep_if_yet_deployed_count(self, args: list[str]) -> None:
        """Changes the 30 min sleep condition based on deployed count

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        sleep_if_yet_deployed_count = self._config.sleep_if_yet_deployed_count

        if len(args) < 1:
            self._log.warning("No sleep condition for deployed count specified")
            self._packet_manager.send(
                "No sleep condition for deployed count specified.".encode(
                    "utf-8"
                )
            )
            return

        sleep_if_yet_deployed_count = int(args[0])

        try:
            self._config.update_config("sleep_if_yet_deployed_count", sleep_if_yet_deployed_count, temporary=False)
            self._log.info("Sleep if yet deployed value changed")
            self._packet_manager.send(
                f"Sleep if yet deployed value changed: {sleep_if_yet_deployed_count}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change sleep condition for deployed count", err=e)
            self._packet_manager.send(
                f"Failed to change sleep condition for deployed count: {e}".encode("utf-8")
            )
    
    def change_cdh_listen_command_timeout(self, args: list[str]) -> None:
        """Changes the cdh listen command timeout

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        cdh_listen_command_timeout = self._config.cdh_listen_command_timeout

        if len(args) < 1:
            self._log.warning("No cdh listen command timeout specified")
            self._packet_manager.send(
                "No cdh listen command timeout specified.".encode(
                    "utf-8"
                )
            )
            return

        cdh_listen_command_timeout = int(args[0])

        try:
            self._config.update_config("cdh_listen_command_timeout", cdh_listen_command_timeout, temporary=False)
            self._log.info("cdh listen command timeout changed")
            self._packet_manager.send(
                f"cdh listen command timeout changed: {cdh_listen_command_timeout}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change cdh listen command timeout", err=e)
            self._packet_manager.send(
                f"Failed to change cdh listen command timeout: {e}".encode("utf-8")
            )

    def change_watchdog_reset_sleep(self, args: list[str]) -> None:
        """Changes the watchdog_reset_sleep

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        watchdog_reset_sleep = self._config.watchdog_reset_sleep

        if len(args) < 1:
            self._log.warning("No watchdog_reset_sleep specified")
            self._packet_manager.send(
                "No watchdog_reset_sleep specified.".encode(
                    "utf-8"
                )
            )
            return

        watchdog_reset_sleep = int(args[0])

        try:
            self._config.update_config("watchdog_reset_sleep", watchdog_reset_sleep, temporary=False)
            self._log.info("watchdog_reset_sleep changed")
            self._packet_manager.send(
                f"watchdog_reset_sleep changed: {watchdog_reset_sleep}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change watchdog_reset_sleep", err=e)
            self._packet_manager.send(
                f"Failed to change watchdog_reset_sleep: {e}".encode("utf-8")
            )

    def change_except_reset_allowed_attemps(self, args: list[str]) -> None:
        """Changes the except_reset_allowed_attemps

        Args:
            args: A list of arguments, the first item must be the new value. All other items in the args list are ignored.
        """
        except_reset_allowed_attemps = self._config.except_reset_allowed_attemps

        if len(args) < 1:
            self._log.warning("No except_reset_allowed_attemps specified")
            self._packet_manager.send(
                "No except_reset_allowed_attemps specified.".encode(
                    "utf-8"
                )
            )
            return

        except_reset_allowed_attemps = int(args[0])

        try:
            self._config.update_config("except_reset_allowed_attemps", except_reset_allowed_attemps, temporary=False)
            self._log.info("except_reset_allowed_attemps changed")
            self._packet_manager.send(
                f"except_reset_allowed_attemps changed: {except_reset_allowed_attemps}".encode("utf-8")
            )
        except ValueError as e:
            self._log.error("Failed to change except_reset_allowed_attemps", err=e)
            self._packet_manager.send(
                f"Failed to change except_reset_allowed_attemps: {e}".encode("utf-8")
            )

    def reset(self) -> None:
        """Resets the hardware."""

        self._log.info("Resetting satellite")
        self._packet_manager.send(data="Resetting satellite".encode("utf-8"))
        microcontroller.on_next_reset(microcontroller.RunMode.NORMAL)
        microcontroller.reset()

    def oscar_command(self, command: str, args: list[str]) -> None:
        """Handles OSCAR commands.

        Args:
            command: The OSCAR command to execute.
            args: A list of arguments for the command.
        """

        if command == "ping":
            self._log.info("OSCAR ping command received. Sending pong response.")
            self._packet_manager.send(
                f"Pong! {self._packet_manager.get_last_rssi()}".encode("utf-8")
            )

        elif command == "repeat":
            if len(args) < 1:
                self._log.warning("No message specified for repeat command")
                self._packet_manager.send(
                    "No message specified for repeat command.".encode("utf-8")
                )
                return
            repeat_message = " ".join(args)
            self._log.info("OSCAR repeat command received. Repeating message.")
            self._packet_manager.send(repeat_message.encode("utf-8"))

        else:
            self._log.warning("Unknown OSCAR command received", command=command)
            self._packet_manager.send(
                f"Unknown OSCAR command received: {command}".encode("utf-8")
            )

    def set_orient_payload(self, args: list[str]):
        try:
            if len(args) < 1:
                self._log.debug("Not enough arguments for orient_payload command. Requires setting (ex: 0 or 1).")
                return
            orient_payload_setting = args[0]
            if str(orient_payload_setting) in ["0", "1"]:
                self._config.update_config("orient_payload_setting", int(orient_payload_setting), temporary=False)
            else:
                self._log.debug("Invalid orient payload setting.  Set as 0 or 1")
        except ValueError as e:
            self._log.error("Failed to change orient modulation", err=e)
        self._packet_manager.send(f"New feature executed with args: {args}".encode("utf-8"))
    
    def exec_command(self, args: list[str]) -> None:
        """Executes arbitrary Python code sent from the ground station.
        
        Args:
            args: A list of arguments, where all items joined together form the Python code to execute.
        
        Warning:
            This is potentially dangerous as it allows execution of any code.
        """
        if len(args) < 1:
            self._log.warning("No code specified for execution")
            self._packet_manager.send(
                "No code specified for execution.".encode("utf-8")
            )
            return
        
        code_to_execute = " ".join(args)
        self._log.info("Executing code", code=code_to_execute)
        
        try:
            # Create a string buffer to capture output
            import io
            import sys
            original_stdout = sys.stdout
            captured_output = io.StringIO()
            sys.stdout = captured_output
            
            # Execute the code
            exec(code_to_execute)
            
            # Restore stdout and get the output
            sys.stdout = original_stdout
            output = captured_output.getvalue()
            
            self._log.info("Code executed successfully")
            self._packet_manager.send(
                f"Code executed successfully. Output:\n{output}".encode("utf-8")
            )
        except Exception as e:
            # Restore stdout in case of error
            import sys
            sys.stdout = sys.__stdout__
            self._log.error("Failed to execute code", err=e)
            self._packet_manager.send(
                f"Failed to execute code: {str(e)}".encode("utf-8")
            )
