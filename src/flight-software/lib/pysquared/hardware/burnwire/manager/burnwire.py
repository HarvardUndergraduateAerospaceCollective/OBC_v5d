"""This module defines the `BurnwireManager` class, which provides a high-level interface
for controlling burnwire circuits, which are commonly used for deployment mechanisms in
satellites. It handles the timing and sequencing of the burnwire activation
and provides error handling and logging.

**Usage:**
```python
logger = Logger()
enable_pin = DigitalInOut(board.D1)
fire_pin = DigitalInOut(board.D2)
burnwire = BurnwireManager(logger, enable_pin, fire_pin)
burnwire.burn()
```
"""

import time

from digitalio import DigitalInOut

from ....logger import Logger
from ....protos.burnwire import BurnwireProto

from ....watchdog import Watchdog


class BurnwireManager(BurnwireProto):
    """Manages the activation of a burnwire."""

    # Safety limit: Maximum burn duration in seconds (prevents runaway burns)
    MAX_BURN_DURATION_SEC = 25.0

    # Watchdog pet interval during burns (seconds)
    WATCHDOG_PET_INTERVAL_SEC = 10.0

    def __init__(
        self,
        logger: Logger,
        enable_burn: DigitalInOut,
        fire_burn: DigitalInOut,
        enable_logic: bool = True,
        watchdog: "Watchdog | None" = None,
    ) -> None:
        """Initializes the burnwire manager.

        Args:
            logger: The logger to use.
            enable_burn: The pin used to enable the burnwire circuit.
            fire_burn: The pin used to fire the burnwire.
            enable_logic: The logic level to enable the burnwire.
            watchdog: Optional watchdog to pet during long burn operations.
        """
        self._log: Logger = logger
        self._enable_logic: bool = enable_logic
        self._watchdog: "Watchdog | None" = watchdog

        self._enable_burn: DigitalInOut = enable_burn
        self._fire_burn: DigitalInOut = fire_burn

        self.number_of_attempts: int = 0

    def burn(self, timeout_duration: float = 5.0) -> bool:
        """Fires the burnwire for a specified amount of time.

        Args:
            timeout_duration: The maximum amount of time to keep the burnwire on.
                             Capped at MAX_BURN_DURATION_SEC for safety.

        Returns:
            True if the burn was successful, False otherwise.

        Raises:
            Exception: If there is an error toggling the burnwire pins.
        """
        # Safety cap: limit burn duration to prevent runaway burns
        if timeout_duration > self.MAX_BURN_DURATION_SEC:
            self._log.warning(
                f"Burn duration {timeout_duration}s exceeds safety limit, capping to {self.MAX_BURN_DURATION_SEC}s"
            )
            timeout_duration = self.MAX_BURN_DURATION_SEC

        _start_time = time.monotonic()

        self._log.debug(
            f"BURN Attempt {self.number_of_attempts} Started with Duration {timeout_duration}s"
        )
        try:
            self._attempt_burn(timeout_duration)
            return True

        except KeyboardInterrupt:
            self._log.debug(
                f"Burn Attempt Interrupted after {time.monotonic() - _start_time:.2f} seconds"
            )
            return False

        except RuntimeError as e:
            self._log.critical(
                f"BURN Attempt {self.number_of_attempts} Failed!",
                e,
            )
            return False

    def _enable(self):
        """
        Activates the burnwire mechanism by enabling the control pins.

        This method sets the `_enable_burn` and `_fire_burn` pins to the logic level specified by `_enable_logic`.
        It first enables the burnwire circuit, waits briefly to allow load switches to stabilize, and then fires the burnwire.
        Raises a RuntimeError if setting either pin fails.

        Raises:
            RuntimeError: If unable to set the enable or fire pins due to hardware or communication errors.
        """
        try:
            self._enable_burn.value = self._enable_logic
        except Exception as e:
            raise RuntimeError("Failed to set enable_burn pin") from e

        time.sleep(0.1)  # Short pause to stabilize load switches

        # Burnwire becomes active
        try:
            self._fire_burn.value = self._enable_logic
        except Exception as e:
            raise RuntimeError("Failed to set fire_burn pin") from e

    def _disable(self):
        """
        Safes the burnwire by disabling the fire and enable pins.

        Sets the `_fire_burn` and `_enable_burn` pin values to the logical opposite of `_enable_logic`,
        effectively disabling the burnwire mechanism. Logs the action for traceability.
        Raises a RuntimeError if the operation fails.

        Raises:
            RuntimeError: If unable to set the burnwire pins to the safe state.
        """
        try:
            self._fire_burn.value = not self._enable_logic
            self._enable_burn.value = not self._enable_logic
            self._log.debug("Burnwire safed")
        except Exception as e:
            raise RuntimeError("Failed to safe burnwire pins") from e

    def _attempt_burn(self, duration: float = 5.0) -> None:
        """Attempts to actuate the burnwire for a set period of time.

        Pets the watchdog during long burns to prevent watchdog reset.

        Args:
            duration: The duration of the burn.

        Raises:
            RuntimeError: If there is an error toggling the burnwire pins.
        """
        error = None
        try:
            self.number_of_attempts += 1

            # Burnwire becomes active
            try:
                self._enable()
            except Exception as e:
                error = RuntimeError("Failed to set fire_burn pin")
                raise error from e

            # Sleep in increments, petting watchdog to prevent reset during long burns
            end_time = time.monotonic() + duration
            while time.monotonic() < end_time:
                # Sleep for watchdog interval or remaining time, whichever is shorter
                sleep_time = min(
                    end_time - time.monotonic(), self.WATCHDOG_PET_INTERVAL_SEC
                )
                if sleep_time > 0:
                    time.sleep(sleep_time)

                # Pet watchdog if available
                if self._watchdog is not None:
                    self._watchdog.pet()

        except RuntimeError as e:
            # Log the error if it occurs during the burn process
            self._log.critical(
                f"Burnwire failed on attempt {self.number_of_attempts}!", e
            )
            raise e

        except KeyboardInterrupt as exc:
            self._log.warning(f"BURN Attempt {self.number_of_attempts} Interrupted!")
            raise KeyboardInterrupt("Burnwire operation interrupted by user") from exc

        finally:
            # Burnwire cleanup in the finally block to ensure it always happens
            try:
                self._disable()
                self._log.debug("Burnwire Safed")
            except Exception as e:
                # Only log critical if this wasn't caused by the original error
                if error is None:
                    self._log.critical("Failed to safe burnwire pins!", e)

            # Re-raise the original error if there was one
            if error is not None:
                raise error
