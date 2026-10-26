# AGENTS.md

C++17 wrapper for Nanotec C5-E-2-09 (FIR-v2627) BLDC controllers via NanoLib 1.4.3 over USB MSC, CiA 402 Profile Velocity. University robotics club project (IGVC). Public repo: keep personal info out. Drives real motors: safety > features.

## Layout / invariants
- `src/motor.cpp`: all drive logic; talks only to `OdIo`. No NanoLib includes here, ever. Keeps `make test` hardware-free.
- `src/nanolib_session.cpp`: only NanoLib code. `detail::NanoLibState` shared_ptr keeps buses open until last device closes.
- Multi-motor shutdown must go through `shutdownAll()` (zero all → wait all → disable all). Never stop motors sequentially.
- `checkHealthy()` trips = commandZero + throw (non-blocking); blocking cleanup is caller's/MotorGroup's job.
- Velocity magnitudes: widen to int64 before abs (INT32_MIN).
- Commissioning checks mirror the team's original `velocity_test.cpp` (not in repo) (1:1 gearing, ≤3 A, rpm units, encoder velocity). Loosen via `SafetyLimits`, don't delete.

## Commands
- `make test`: fake-drive tests (21). docs/background.md = newcomer primer; keep README API table in sync with motor.hpp. Fake models ramps/coasting/rising-edge reset; keep it realistic, instant dynamics hid bugs before.
- `make check-nanolib [NANOLIB_CHECK_INC=<sdk>/nanotec_nanolib/inc]`: stubs or real headers.
- `make demo NANOLIB_DIR=<sdk>/nanotec_nanolib`: Linux x86_64 only (headers in `inc/`, not `include/`).
- SDK: public zip https://www.nanotec.com/fileadmin/files/Software/NanoLib/NanoLib_cpp_1.4.3.zip, inner `nanolib_cpp_linux_1.4.3.zip`. Proprietary: never commit.

## Unverified on hardware
- Two-controller enumeration (two MSC buses vs one), negative target writes.
- Process kill / USB loss leaves motor at last target; no software fix over USB.
- Plug & Drive Studio is Windows-only; must be closed while running (single owner).
