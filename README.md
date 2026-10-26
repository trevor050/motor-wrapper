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

`NANOLIB_DIR` defaults to `../vendor/nanolib`. Override it, or `NANOLIB_INC` / `NANOLIB_LIB`, if the SDK lives elsewhere. Close Plug & Drive Studio before running: only one process may own a controller.

## Safety behavior

- `enable()` refuses to run until `verifyCommissioning()` passes. The checks match `velocity_test.cpp`: encoder velocity feedback, Hall or encoder commutation, rpm units, 1:1 factors, current ≤ 3 A, ramps 25–100 rpm/s, no NanoJ running. Adjust `SafetyLimits` rather than removing checks.
- Any failure during `enable()` triggers stop and disable before the exception propagates.
- `checkHealthy()` shuts the motor down before throwing.
- The `Motor` destructor stops and disables a motor that is still enabled.
- **Known hazard:** if the process is killed (`kill -9`, crash) or USB disconnects, the controller keeps running at its last target velocity. Software cannot prevent this over USB. Keep a hardware stop within reach.

## Not yet verified on hardware

- How NanoLib reports two controllers (two USB buses vs. one bus with two devices). `discover()` handles both; check the `--list` output.
- Negative velocity writes. The value is passed to `writeNumber` as a signed 32-bit integer. Test at low speed first.
- Header location inside the NanoLib SDK (`NANOLIB_INC`).
