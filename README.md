# motor-wrapper

A C++17 wrapper for [Nanotec](https://www.nanotec.com) C5-E brushless motor controllers, built on Nanotec's NanoLib SDK. It runs one or more controllers in CiA 402 Profile Velocity mode behind a small, safety-checked API, and its drive logic can be tested without hardware.

Written for a university robotics team's [IGVC](http://www.igvc.org) ground vehicle: two C5-E-2-09 drives with Hall sensors and incremental encoders, connected over USB to an x86_64 Ubuntu computer.

> **Status:** compiles and links against NanoLib 1.4.3 (Ubuntu 24.04, g++ 13.3); 21 simulated-drive tests pass. **Not yet run on real hardware.** See [Known limitations](#known-limitations).

```cpp
#include "motor/nanolib_session.hpp"

motor::NanoLibSession session;
auto controllers = session.discover();         // every C5-E on every USB bus
auto left = session.connectMotor(0, "left");

left->verifyCommissioning();  // refuse to run on an unexpected drive configuration
left->enable();               // CiA 402: Shutdown -> Switch on -> Enable operation
left->setVelocityRpm(100);    // signed; negative reverses

while (running) {
    left->checkHealthy();     // throws on fault, overspeed, or wrong direction
    auto t = left->telemetry();
}

left->stopAndDisable();       // also happens automatically in the destructor
```

## Contents

- [Requirements](#requirements)
- [Quick start](#quick-start)
- [API overview](#api-overview)
- [How it works](#how-it-works)
- [Safety model](#safety-model)
- [Testing](#testing)
- [Project layout](#project-layout)
- [Known limitations](#known-limitations)
- [Background reading](#background-reading)

## Requirements

| | |
|---|---|
| Controller | Nanotec C5-E (tested configuration: C5-E-2-09, firmware FIR-v2627), commissioned in Plug & Drive Studio |
| Motor feedback | Hall sensors + incremental A/B/I encoder |
| Host | Linux x86_64 (NanoLib's Linux binaries are x86_64/arm64; this project is set up for x86_64) |
| Compiler | Any C++17 compiler; verified with g++ 13.3 and Apple clang 17 |
| SDK | [NanoLib C++ 1.4.3](https://www.nanotec.com/us/en/products/9985-nanolib), downloaded separately (proprietary, not included) |

The unit tests need only a C++17 compiler and run on any OS, including macOS.

## Quick start

### 1. Run the tests (no hardware, no SDK)

```bash
git clone https://github.com/trevor050/motor-wrapper.git
cd motor-wrapper
make test
```

### 2. Get the SDK

Download [`NanoLib_cpp_1.4.3.zip`](https://www.nanotec.com/fileadmin/files/Software/NanoLib/NanoLib_cpp_1.4.3.zip) from the [NanoLib page](https://www.nanotec.com/us/en/products/9985-nanolib). Extract it, then extract the inner `nanolib_cpp_linux_1.4.3.zip`. You need its `nanotec_nanolib/` directory, which contains `inc/` (headers) and `lib/` (shared libraries, including the `libnanolibm_usbmsc.so` USB module).

Either place that directory at `../vendor/nanolib` (next to this checkout), or pass its path as `NANOLIB_DIR`. Never commit the SDK; `.gitignore` already excludes the usual locations.

### 3. Build and run the bench demo

Close Plug & Drive Studio first. Only one process can own a controller.

```bash
make demo NANOLIB_DIR=/path/to/nanotec_nanolib

# Discover controllers and verify their configuration. Nothing moves.
make run-demo NANOLIB_DIR=/path/to/nanotec_nanolib ARGS="--list"

# Spin every controller at 50 rpm for 3 s. Asks you to type RUN first.
make run-demo NANOLIB_DIR=/path/to/nanotec_nanolib ARGS="50 3"
```

Before the first motion test: motors unloaded and secured, power supply monitored, hardware emergency stop within reach.

## API overview

All types live in namespace `motor`. Every failure throws `motor::MotorError`.

### `NanoLibSession` ([`nanolib_session.hpp`](include/motor/nanolib_session.hpp))

| Method | Description |
|---|---|
| `discover()` | Opens every USB (MSC) bus and scans it. Returns a `ControllerInfo` list with stable indices for this run. Call once. |
| `connectMotor(index, name, limits, timing)` | Connects to a discovered controller and returns a `std::unique_ptr<Motor>`. |
| `openDevice(index)` | Lower level: returns the raw `OdIo` object dictionary connection. |

The session keeps buses open until the last motor created from it is destroyed, so destruction order does not matter.

### `Motor` ([`motor.hpp`](include/motor/motor.hpp))

| Method | Description |
|---|---|
| `verifyCommissioning()` | Reads the configuration saved by Plug & Drive Studio and throws on the first problem. Required before `enable()`. |
| `enable()` | Zero target, Profile Velocity mode, then the CiA 402 sequence to *Operation enabled*. Cleans up on any failure. |
| `setVelocityRpm(rpm)` | Signed target velocity, limited to `SafetyLimits::maxRpm` and the drive's own profile limit. |
| `checkHealthy()` | Checks for faults, errors, state or mode changes, overspeed, and motion against the demand. On failure: zeroes the target, blocks new targets, throws. |
| `telemetry()` | Statusword, state, error register, target/demand/actual velocity, position, and per-sensor (Hall and encoder) velocity and position. |
| `stop()` | Zero target, then wait for confirmed standstill. Returns `false` on timeout. |
| `disable()` | Turns off the power stage and confirms it. |
| `stopAndDisable()` | `stop()` then `disable()`; disables even if the stop failed. |
| `resetFault()` | Clears a latched drive fault (rising edge of controlword bit 7). |
| `setCancelCheck(fn)` | Optional predicate, such as a Ctrl+C flag, that aborts `enable()` between steps. |

Free function: `shutdownAll(std::vector<Motor*>)`, a coordinated stop for several motors (see [Safety model](#safety-model)).

### Configuration

`SafetyLimits` sets the speed cap (default 350 rpm), current limits (3 A), the allowed ramp range (25–100 rpm/s), and the overspeed margin. `Timing` sets poll interval, state-transition and stop timeouts, and the standstill threshold. Both are passed per motor. Loosen limits deliberately; don't delete checks.

## How it works

```
 your code ──> Motor ──> OdIo ──> NanoLibDeviceIo ──> NanoLib ──USB──> C5-E ──> motor
                 │          ▲
                 │          └── FakeDrive (tests)
                 └── CiA 402 state machine, safety checks, telemetry
```

- **Everything is the object dictionary.** A C5-E exposes its settings, sensor readings, and commands as numbered entries, such as `0x6041` (statusword) or `0x60FF` (target velocity). Controlling the drive means reading and writing those entries.
- **`Motor` holds all the drive logic but never calls NanoLib directly.** It talks to the three-method `OdIo` interface (`readNumber`, `readString`, `writeNumber`). `NanoLibDeviceIo` implements it for real hardware; `FakeDrive` implements it for tests.
- **The drive runs its own velocity loop.** The PC only sets targets and monitors. NanoLib over USB is not real-time (each access is a slow round trip), so nothing time-critical depends on the PC.
- **Enabling is a state machine.** The drive must be walked from *Switch on disabled* through *Ready to switch on* and *Switched on* to *Operation enabled* with controlword values `0x06`, `0x07`, `0x0F`, waiting for the statusword to confirm each step.

[`docs/background.md`](docs/background.md) explains all of this in detail, including a table of the object dictionary entries used.

## Safety model

This library commands real motors. Its rules:

- **Refuse unknown configurations.** `enable()` throws unless `verifyCommissioning()` has passed. The check requires: no fault or error, no NanoJ program running, closed-loop operation, the expected sensor mapping (sensorless/Hall/encoder), encoder as the velocity source, Hall or encoder commutation, rpm units, 1:1 gearing and factors, current and ramp limits, and a linear motion profile.
- **Clean up on every failure path.** From the first write in `enable()` onward, any exception triggers stop-and-disable. The `Motor` destructor does the same for a motor left enabled.
- **Check both velocity sources.** `checkHealthy()` validates both the selected velocity (`0x606C`) and the encoder velocity (`0x60E5:03`) against the overspeed limit, and against the ramp demand for direction, so normal reversals are not flagged.
- **Trip fast, clean up together.** A failed health check zeroes that motor's target immediately and throws instead of blocking. With several motors, `shutdownAll()` zeroes **every** target first, then waits for all of them to stop against one shared deadline, then disables each one, attempting every motor even if another fails. The demo wraps its motors in a `MotorGroup` that calls `shutdownAll()` on every exit path.
- **Never assume it stopped.** `stop()` always confirms standstill from fresh readings, including on a disabled motor that may still be coasting.

**What software cannot do:** if the process is killed or USB disconnects, the controller keeps running at its last target velocity. A hardware emergency stop that removes motor power independently of the computer is required. IGVC rules also require physical and wireless E-stops that are "hardware based and not controlled through software".

## Testing

```bash
make test                                         # simulated-drive tests, any OS
make check-nanolib                                # type-check NanoLib code against stub headers
make check-nanolib NANOLIB_CHECK_INC=<sdk>/inc    # type-check against the real SDK headers
```

`tests/fake_drive.hpp` simulates a commissioned C5-E. It models the CiA 402 state machine, Profile Velocity mode, acceleration ramps, coasting with the power stage off, and delayed rising-edge fault reset. Tests inject faults, timeouts, communication loss, runaway or reversed sensor readings, and extreme values, and check that the motor always ends up stopped and disabled.

`tests/nanolib_stubs/` holds compile-only declarations of the small NanoLib API subset this project uses, so the NanoLib glue can be type-checked without the SDK. They prove nothing about runtime behavior.

## Project layout

| Path | Purpose |
|---|---|
| [`include/motor/motor.hpp`](include/motor/motor.hpp), [`src/motor.cpp`](src/motor.cpp) | `Motor`: commissioning checks, state machine, velocity control, telemetry, safety. No NanoLib dependency. |
| [`include/motor/od_io.hpp`](include/motor/od_io.hpp) | `OdIo` object dictionary interface and `MotorError`. |
| [`include/motor/nanolib_session.hpp`](include/motor/nanolib_session.hpp), [`src/nanolib_session.cpp`](src/nanolib_session.cpp) | The only NanoLib code: discovery, connection, reads and writes. |
| [`examples/motor_demo.cpp`](examples/motor_demo.cpp) | Bench demo for any number of controllers. |
| [`tests/`](tests) | Simulated drive, unit tests, NanoLib stubs. |
| [`docs/background.md`](docs/background.md) | Primer on the hardware, CiA 402, the object dictionary, and the C++ used here. |

## Known limitations

- **Not yet tested on hardware.** In particular: how NanoLib enumerates two controllers (two USB buses or one; `discover()` handles both), and negative target velocities (written as signed 32-bit values). Start at low speed.
- **Profile Velocity only.** No position or torque modes yet.
- **USB is slow.** Each object dictionary access is a USB mass-storage round trip, so `telemetry()` (11 reads) is costly. Poll only what you need. CANopen through a USB-to-CAN adapter is the faster path, and NanoLib supports it with the same API.
- **Bench defaults.** `verifyCommissioning()` assumes direct drive (1:1 gearing) and a 3 A current limit. A gearbox or different motor needs adjusted checks.

## Background reading

- [`docs/background.md`](docs/background.md): hardware overview, glossary, object dictionary table, CiA 402 state machine, C++ notes
- [NanoLib C++ user manual](https://www.nanotec.com/us/en/products/9985-nanolib) (Nanotec)
- [C5-E technical manual](https://www.nanotec.com/us/en/products/1768-c5-e-2-09-motor-controller-drive-for-canopen-or-usb) (object dictionary reference; V3.3.0 applies to firmware FIR-v2213 and later)
- [CiA 402 power state machine](https://www.nanotec.com/us/en/products/manual/m/c5e-modbusrtu-usb-en/p/general%2Fds402_power_state_machine.html) (Nanotec)

## License

[MIT](LICENSE). NanoLib is proprietary Nanotec software and is not covered by this license. This project is not affiliated with or endorsed by Nanotec.
