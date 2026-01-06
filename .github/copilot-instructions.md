# AI Coding Agent Instructions for PROVES Kit RP2350 v5b

## Project Overview

This is **flight software for a CubeSat on-board computer (OBC)** running CircuitPython on an RP2350 microcontroller. The project contains two parallel codebases:
- **Flight Software** (`src/flight-software/`): Satellite control logic, state machine, hardware managers
- **Ground Station** (`src/ground-station/`): Communication and debugging interface

The system uses a **finite state machine (FSM)** architecture with asynchronous data collection, designed for resource-constrained space operations.

## Architecture Essentials

### FSM State Flow
The core control logic in [fsm/fsm.py](fsm/fsm.py) manages transitions through states:

1. **Bootup** → Initialization and 5-second loiter
2. **Detumble** → Angular velocity stabilization using magnetorquers (triggered if rotation > threshold × 1.5)
3. **Deploy** → Solar panel burnwire firing (requires battery voltage > 7.0V)
4. **Orient** → Sun-tracking and payload management

**Key insight**: State transitions are data-driven (battery voltage, IMU rotation magnitude). Emergency detumble overrides any state if angular velocity spikes.

### Data Pipeline
[fsm/data_processes/data_process.py](fsm/data_processes/data_process.py) runs asynchronous coroutines gathering:
- Battery voltage via INA219 power monitor
- 6-DOF IMU (acceleration, angular velocity) via LSM6DSOX
- Magnetometer vector via LIS2MDL

All data stored in `data` dict that FSM continuously monitors. This decouples sensor polling from state logic.

### Hardware Manager Pattern
Managers abstract hardware interaction (e.g., `RFM9xManager`, `INA219Manager`, `LSM6DSOXManager` from `lib/pysquared/`). Each:
- Handles I2C/SPI bus initialization
- Implements standardized read/write methods
- Manages resources in resource-limited CircuitPython environment

## Critical Developer Workflows

### Setup
```bash
make all              # Create venv, download libraries, install pre-commit hooks
make download-libraries-flight-software  # Refresh dependencies
```

### Development & Type Checking
```bash
make fmt              # Format with pre-commit (ruff, pyright stubs)
make typecheck        # Run pyright type checking (includes typeshed)
```

### Hardware Installation
```bash
make list-tty                           # Find board port
make install-flight-software BOARD_MOUNT_POINT=/Volumes/CIRCUITPY
```

**Note**: Uses `rsync` (not `cp`) to preserve board state during iterative installs.

### Debugging
- Enter REPL: Press `Ctrl+C` during main loop to interrupt
- REPL objects: `watchdog`, `logger`, `config`, `radio`, `imu`, `magnetometer`, `cdh`, `c`
- Use `help(OBJECT_NAME)` to discover available methods

## Project-Specific Patterns & Conventions

### Logging
All logging goes through [lib/pysquared/logger.py](lib/pysquared/logger.py):
```python
logger.info("State transition", current_state="detumble", reason="high_rotation")
logger.debug("Magnetometer reading", mag_vector=[0.1, 0.2, 0.3])
```
Logs are JSON-formatted for ground station parsing.

### Configuration
[config.json](config.json) is the single source of truth for mission parameters:
- Voltage thresholds: `fsm_batt_threshold_deploy` (7.0V), `fsm_batt_threshold_orient` (6.0V)
- State timings: `detumble_max_time`, `deploy_burn_duration`
- Sensor calibration: `detumble_stabilize_threshold`, `orient_light_threshold`

**When adding features**: Extract magic numbers into config.json first.

### Non-Volatile Memory (NVM) Counters
Used to track persistent state across resets:
```python
from lib.pysquared.nvm.counter import Counter
from lib.proveskit_rp2350_v5b.register import Register

boot_count = Counter(index=Register.boot_count)
boot_count.increment()  # Survives power cycles
```

### Watchdog Integration
Critical for satellite reliability. Petting the watchdog resets a timer; if timer expires, board resets:
```python
watchdog.pet()  # Called regularly in safe_sleep_async()
# If watchdog not petted for ~15 seconds, board hard-resets
```

### State Class Structure
Each state (see [fsm/state_processes/](fsm/state_processes/)) implements:
```python
async def run(self):
    # Long-running mission logic; runs in background task
    self.done = True  # Signal FSM when complete

def is_done(self):
    return self.done

def stop(self):
    self.running = False  # Graceful shutdown before FSM.set_state()
```

## Cross-Component Communication

### Managers → FSM
Sensor managers read hardware continuously; FSM polls `data` dict from [DataProcess](fsm/data_processes/data_process.py). Example:
```python
if self.dp_obj.data["data_batt_volt"] > self.config.fsm_batt_threshold_orient:
    self.set_state("orient")
```

### Command Processing
[CDH (Command Data Handler)](lib/pysquared/cdh.py) parses incoming radio packets, executes commands, responds with telemetry.

### Emergency Overrides
Located in [main.py](src/flight-software/main.py) main loop:
```python
if low_battery_emergency or power_consumption_spike:
    # Disable payload immediately
    fsm.PAYLOAD_BATT_ENABLE.value = False
    # OR trigger immediate detumble in FSM
```

## External Dependencies & Integration Points

### CircuitPython Core
- No standard library (`os`, `time`, `gc` are limited)
- Type stubs in `typings/` and `circuitpython-stubs` package
- Pyright configured to exclude `src/*/lib` (external vendor code)

### Hardware Abstraction
- **Bus initialization**: [lib/pysquared/hardware/busio.py](lib/pysquared/hardware/busio.py) manages I2C/SPI setup
- **Pin initialization**: [lib/pysquared/hardware/digitalio.py](lib/pysquared/hardware/digitalio.py) wraps `board` module

### Pysquared Library
Dependency imported from GitHub via Makefile:
```makefile
PYSQUARED_VERSION ?= copilot/fix-7bfb9c49-466e-42db-a6be-dece956d6d8c
PYSQUARED ?= git+https://github.com/proveskit/pysquared@$(PYSQUARED_VERSION)#subdirectory=circuitpython-workspaces/flight-software
```
This is a submodule dependency; keep version pinned to ensure reproducibility.

## Code Structure Notes

- **No shared `lib/` between codebases**: Flight software and ground station each have independent `lib/` directories with duplicated Adafruit drivers. This is intentional (avoids symlink issues in CircuitPython).
- **Version management**: [version.py](src/flight-software/version.py) tracks software version; auto-included in boot logs.
- **Git tags**: Release versions are derived from git tags (see Makefile `VERSION` variable).
- **Tests**: CI via GitHub Actions; manual hardware testing preferred over unit tests (CircuitPython resource constraints).

## When Adding Features

1. **Extract configuration** into [config.json](config.json) (thresholds, timings, enable flags)
2. **Add to DataProcess** if new sensor data needed
3. **Update FSM logic** in [fsm.py](fsm/fsm.py) or state classes
4. **Log state changes** with context (use `logger.info()`)
5. **Update REPL exports** in [repl.py](src/flight-software/repl.py) for debugging access
6. **Run `make typecheck`** before committing (pyright must pass)
