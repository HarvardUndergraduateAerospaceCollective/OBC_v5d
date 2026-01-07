"""Safe mode handler for HUCSat flight software.

This file is executed when CircuitPython enters safe mode (after hard faults,
watchdog resets, etc.). It provides diagnostic information and attempts recovery.
"""
import time

import microcontroller

print("I am in safemode. Help!")

# Attempt normal mode reset
try:
    microcontroller.on_next_reset(microcontroller.RunMode.NORMAL)
except Exception:
    pass  # May not be available on all platforms
microcontroller.reset()
