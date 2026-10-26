# Motor wrapper

C++17 wrapper around Nanotec NanoLib for C5-E controllers in Profile Velocity mode. It turns the logic of `velocity_test.cpp` into a reusable `motor::Motor` class and supports several controllers at once.

```cpp
motor::NanoLibSession session;
auto controllers = session.discover();          // every C5-E on every USB bus
auto left = session.connectMotor(0, "left");

left->verifyCommissioning();  // refuses unexpected Plug & Drive configuration
left->enable();               // CiA 402: Shutdown -> Switch on -> Enable operation
left->setVelocityRpm(100);    // signed; limited to SafetyLimits::maxRpm
while (running) {
    left->checkHealthy();     // throws and shuts down on fault, overspeed, wrong direction
    auto t = left->telemetry();
}
left->stopAndDisable();       // also runs automatically in the destructor
```

## Layout

| Path | Purpose |
|---|---|
| `include/motor/motor.hpp`, `src/motor.cpp` | Drive logic: commissioning checks, CiA 402 state machine, velocity control, telemetry, safety checks. No NanoLib dependency. |
| `include/motor/od_io.hpp` | Object dictionary interface the `Motor` class talks to. |
| `include/motor/nanolib_session.hpp`, `src/nanolib_session.cpp` | The only NanoLib code: bus discovery, device connection, OD reads/writes. |
| `examples/motor_demo.cpp` | Bench demo: lists controllers, verifies them, spins all of them. |
| `tests/` | Simulated C5-E (`fake_drive.hpp`), unit tests, and compile-only NanoLib stubs. |

## Commands

```bash
make test                         # unit tests against the simulated drive; runs anywhere
make check-nanolib                # type-check NanoLib code against stub headers
make demo                         # build bin/motor_demo against the real SDK (Linux x86_64)
make run-demo ARGS="--list"       # discover + verify commissioning, no motion
make run-demo ARGS="100 5"        # 100 rpm for 5 s on every controller (asks for RUN)
```

`NANOLIB_DIR` defaults to `../vendor/nanolib`, with headers in `inc/` and libraries in `lib/`. Override it, or `NANOLIB_INC` / `NANOLIB_LIB`, if the SDK lives elsewhere. Close Plug & Drive Studio before running: only one process may own a controller.

## Real SDK build

Download [NanoLib C++ 1.4.3](https://www.nanotec.com/fileadmin/files/Software/NanoLib/NanoLib_cpp_1.4.3.zip) from the [official NanoLib page](https://www.nanotec.com/us/en/products/9985-nanolib). Extract the outer ZIP, then `nanolib_cpp_linux_1.4.3.zip` for Linux x86_64. Its `nanotec_nanolib/` directory contains `inc/` and `lib/`; place that directory at `../vendor/nanolib`, or point `NANOLIB_DIR` at it. Keep all supplied libraries together, including `libnanolibm_usbmsc.so`, which is a runtime communication module. Do not commit the proprietary SDK.

The wrapper includes these real SDK headers: `accessor_factory.hpp`, `bus_hardware_options.hpp`, `nano_lib_accessor.hpp`, `nano_lib_hw_strings.hpp`, `nlc_callback.hpp`, `nlc_log_level.hpp`, and `od_index.hpp`. The SDK's bundled example has a separate `NanolibExample/nanolib/include/` copy; the standalone distribution uses `nanotec_nanolib/inc/`.

```bash
# macOS or Linux: syntax-check against the real headers without linking/running SDK code
make check-nanolib NANOLIB_CHECK_INC=/path/to/nanotec_nanolib/inc

# Linux x86_64: compile and link the demo, without running it
make -B demo CXX=g++-13 NANOLIB_DIR=/path/to/nanotec_nanolib
```

The default `make check-nanolib` uses an independently written compile-only API subset in `tests/nanolib_stubs`. It does not prove SDK ABI or runtime compatibility. Use the real-header command above and a Linux link build when updating the integration.

Build verification on 2026-10-26 used SDK 1.4.3: Apple clang 17 on macOS arm64 passed the real-header syntax check, and Ubuntu 24.04 linux/amd64 with g++ 13.3.0 compiled and linked the C++17 demo. ELF inspection confirmed `libnanolib.so` as a required shared library. The demo and SDK binaries were not run; hardware behavior remains unverified.

## Safety behavior

- `enable()` refuses to run until `verifyCommissioning()` passes. The checks match `velocity_test.cpp`: encoder velocity feedback, Hall or encoder commutation, rpm units, 1:1 factors, current ≤ 3 A, ramps 25–100 rpm/s, no NanoJ running. Adjust `SafetyLimits` rather than removing checks.
- Any failure during `enable()` triggers stop and disable before the exception propagates.
- `checkHealthy()` shuts the motor down before throwing.
- The `Motor` destructor stops and disables a motor that is still enabled.
- **Known hazard:** if the process is killed (`kill -9`, crash) or USB disconnects, the controller keeps running at its last target velocity. Software cannot prevent this over USB. Keep a hardware stop within reach.

## Not yet verified on hardware

- How NanoLib reports two controllers (two USB buses vs. one bus with two devices). `discover()` handles both; check the `--list` output.
- Negative velocity writes. The value is passed to `writeNumber` as a signed 32-bit integer. Test at low speed first.
