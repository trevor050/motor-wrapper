# Background: Nanotec drives, CiA 402, and the C++ in this repo

A primer for team members who are new to motor control or C++. It explains the hardware, the vocabulary, the drive state machine that `Motor` implements, and the C++ idioms used in the source.

---

## 1. The big picture

```
 Your C++ program  ──>  NanoLib (.so / .dll)  ──USB──>  C5-E controller  ──wires──>  BLDC motor
                                                          │                           │
                                                          └──<── Hall sensors + encoder ──┘
```

- **Motor**: a brushless DC (BLDC) motor. It has no internal brushes, so something external has to decide which coil to energize at each instant. That something is the controller.
- **Controller (drive)**: the Nanotec **C5-E-2-09**. A small box that takes 24 V power, switches current through the motor coils, reads the sensors, and runs a control loop (for example "hold 100 rpm"). It does the real-time work. Your PC only tells it what to do.
- **NanoLib**: Nanotec's C++ library. It finds controllers on a bus (USB here), connects to them, and reads and writes their settings.
- **Plug & Drive Studio (P&DS)**: Nanotec's Windows GUI. It is used for commissioning: setting motor type, current limits, encoder resolution and units, auto-tuning, and saving all of that to the controller. The code assumes P&DS has already done this.
- **This wrapper**: the `motor::Motor` class hides the NanoLib boilerplate behind `enable()`, `setVelocityRpm()`, `telemetry()`, and so on, so robot code never touches raw object dictionary addresses.

### Why USB access is slow

Over USB, the C5-E shows up as a **USB mass-storage device** (a fake thumb drive). The drive holds `cfg.txt` (settings applied at boot), `vmmcode.usr` (a compiled NanoJ program), and `info.bin`. NanoLib's USB transport (`nanolibm_usbmsc`) talks to the controller through that drive interface, so every `readNumber`/`writeNumber` is a slow round trip. Expect roughly milliseconds per access, not microseconds. Practical consequences:

- Don't poll 20 values in a tight loop and expect it to be fast.
- The NanoLib manual states it is **not real-time** and not for synchronized multi-axis motion. Good enough for a club robot, but don't design around precise timing.
- If performance ever matters, CANopen (with a USB-to-CAN adapter) is the usual upgrade path, and NanoLib supports it with the same API.

---

## 2. Glossary

| Term | Meaning |
|---|---|
| **Object Dictionary (OD)** | The controller's entire state as a big table of numbered variables. Everything is an OD entry: settings, sensor readings, commands. **This is the most important concept.** |
| **Index:Subindex** | The address of an OD entry, written in hex, for example `0x6041:00`. Some indexes are arrays: `0x60E4:01..03` is "position from feedback 1, 2, 3". In code: `nlc::OdIndex(0x6041, 0)`. |
| **CANopen / CiA 402** | The industry standard that defines what the OD addresses *mean*. CiA 402 is the motor-drive profile. Addresses `0x6000–0x6FFF` are standard (same on any brand). `0x2000–0x5FFF` are Nanotec-specific. |
| **Controlword (`0x6040`)** | What you write to tell the drive to change state (enable, disable, reset fault). |
| **Statusword (`0x6041`)** | What you read to see the drive's state. Bit 3 = fault. |
| **Mode of operation (`0x6060`)** | What kind of control to run. `1` = Profile Position, `3` = Profile Velocity, `4` = Profile Torque, and so on. `0x6061` reports the mode the drive is actually in. |
| **Power stage** | The transistors that drive current into the motor. "Disabled" = motor is limp and you can turn it by hand. "Operation enabled" = motor is energized and obeys commands. |
| **Hall sensors** | 3 cheap magnetic sensors in the motor. Low resolution (roughly 6 states per electrical revolution), but they tell the drive coarsely where the rotor is. Good for **commutation**. |
| **Encoder (incremental, A/B/I)** | A high-resolution position sensor. A and B are two square waves 90° apart (direction comes from which leads). I is an index pulse once per revolution. Thousands of counts per rev. Good for **velocity and position**. |
| **Commutation** | Deciding which motor coil to energize, based on rotor angle. Hall or encoder can provide it. |
| **Feedback** | A sensor channel the drive uses. The C5-E has 3 slots: `1 = sensorless`, `2 = Hall`, `3 = encoder` (per `0x3204`). |
| **Selection mask (`0x3203`)** | Per feedback: what it is used for. Bit values `1 = position`, `2 = velocity`, `4 = commutation`. Mask `6` = velocity + commutation. |
| **Closed loop** | The drive measures actual speed/position and corrects. Open loop = it just pushes and hopes (stepper-style). |
| **User units** | The drive converts raw encoder counts into units you choose (for example rpm). `0x60A9 = 0x00B44700` means "velocity in rpm". |
| **Profile Velocity mode** | You give a target speed (`0x60FF`) and the drive ramps up/down using the acceleration (`0x6083`) and deceleration (`0x6084`) you configured. |
| **Demand vs actual** | Demand (`0x606B`) = what the ramp generator currently wants. Actual (`0x606C`) = what the sensor measures. A big gap means it can't keep up (load, current limit, bad tuning). |
| **NanoJ** | Small programs that run *on* the controller itself. The velocity test refuses to run if one is active (`0x2300` bit 0) so they don't fight over the motor. |
| **Firmware (`0x100A`)** | The controller's own software version, here `FIR-v2627`. |
| **Bus / bus hardware** | The communication channel (USB MSC, CAN adapter, Ethernet, ...). You open the bus, scan for devices on it, then connect to one. |
| **`.so` / `.dll`** | Compiled shared libraries (Linux / Windows). NanoLib ships as these, not as source. |

### Most-used OD entries

| Address | What | Read/Write |
|---|---|---|
| `0x6040` | Controlword | W |
| `0x6041` | Statusword | R |
| `0x6060` / `0x6061` | Mode (set / actual) | W / R |
| `0x60FF` | Target velocity | W |
| `0x606B` | Velocity demand | R |
| `0x606C` | Actual velocity (selected feedback) | R |
| `0x6064` | Actual position (selected feedback) | R |
| `0x60E4:n` / `0x60E5:n` | Position / velocity from feedback n | R |
| `0x6083` / `0x6084` | Profile accel / decel | config |
| `0x607F` | Max profile velocity | config |
| `0x2031`, `0x6075`, `0x6073` | Max current, rated current, peak current | config |
| `0x1001` | Error register (0 = OK) | R |

---

## 3. The CiA 402 state machine

A drive cannot simply be told "spin". It must be walked through states, like a gearshift. `Motor::enable()` and `Motor::disable()` implement exactly this sequence.

```
 Switch on disabled  (status & 0x4F == 0x40)     motor limp, safe
        │  write controlword 0x06  ("Shutdown", confusingly)
        ▼
 Ready to switch on  (status & 0x6F == 0x21)
        │  write 0x07  ("Switch on")
        ▼
 Switched on         (status & 0x6F == 0x23)
        │  write 0x0F  ("Enable operation")
        ▼
 Operation enabled   (status & 0x6F == 0x27)     motor energized, obeys target
        │  write 0x00  ("Disable voltage")
        ▼
 back to Switch on disabled

 Any state ──(fault)──> Fault  (status bit 3 set). Reset with controlword bit 7 (0x80).
```

The `& 0x6F` is a **bitmask**: it keeps only the bits that identify the state and ignores the others (such as "target reached").

---

## 4. Just enough C++

Everything you need to read the code in this repository.

| You see | It means |
|---|---|
| `#include <iostream>` | Import a library. `<...>` = standard library, `"..."` = project/NanoLib headers. |
| `std::cout << "hi" << x << '\n';` | Print. `std::cerr` = print to the error stream. |
| `int64_t`, `uint16_t`, `uint8_t` | Integers with exact sizes: signed 64-bit, unsigned 16-bit, unsigned 8-bit. Hardware code cares about exact sizes. |
| `0x6041` | A hex number. OD addresses are always written in hex. |
| `status & 0x6F` | Bitwise AND, which keeps only the selected bits. `mask & 2` asks "is bit value 2 set?". |
| `auto x = ...;` | Let the compiler figure out the type. |
| `const auto& x` | A read-only *reference*: use the original without copying it. |
| `std::vector<T>` | A growable array (like a Python list). `.push_back()` = append. |
| `nlc::Something` | `Something` from the `nlc` namespace (NanoLib's prefix). `std::` = standard library. |
| `ptr->method()` | Call a method through a pointer, for example `motor->enable()`. |
| `std::unique_ptr<T>` / `std::shared_ptr<T>` | Owning pointers that free the object automatically. `unique_ptr` has one owner (each `Motor`); `shared_ptr` is reference-counted (the NanoLib session state, kept alive until the last motor is gone). |
| `auto f = [&](int a) { ... };` | A **lambda**: an inline function. `[&]` = it can use the surrounding variables. `verifyCommissioning()` uses one called `require`. |
| `result.hasError()` / `.getResult()` / `.getError()` | NanoLib never throws. Every call returns a `Result` object you must check. |
| `throw std::runtime_error("msg")` ... `catch (const std::exception& e)` | Exceptions: bail out of the `try` block and land in `catch`. |
| `static_cast<uint32_t>(x)` | Explicit type conversion. |
| `virtual int64_t readNumber(...) = 0;` | A pure virtual function: `OdIo` is an interface. `NanoLibDeviceIo` implements it for real hardware, `FakeDrive` for tests. |
| `class QuietScanCallback : public nlc::NlcScanBusCallback { ... override ... }` | Inheritance. NanoLib calls `callback()` during a bus scan. |
| `~Motor()` | Destructor: runs automatically when the object goes away, including during an exception. This is how motors get stopped on every exit path (RAII). |
| `noexcept` | Promise that a function never throws. Cleanup code (`shutdownAll`) is `noexcept` so it can always finish. |
| `namespace { ... }` | Anonymous namespace: these helpers are private to this file. |
| `volatile std::sig_atomic_t` | A variable that is safe to change from a signal handler (Ctrl+C). |

### Signed 32-bit values

`readNumber` returns the raw 32 bits as a big positive number. If the motor spins backward, velocity -5 comes back as `4294967291`. `Motor::readSigned32()` reinterprets the bits as a signed 32-bit integer (`static_cast<int32_t>(static_cast<uint32_t>(raw))`) to recover -5. That is **two's complement**, which is how computers store negative integers.

One trap: `std::abs(INT32_MIN)` is undefined behavior, because +2³¹ does not fit in 32 bits. The code widens values to `int64_t` before taking magnitudes (`magnitude()` in `motor.cpp`).

### Build flow

`make demo` compiles the `.cpp` files and links them against `libnanolib.so` to produce `bin/motor_demo`. `make run-demo` sets `LD_LIBRARY_PATH` so Linux can find the `.so` files at runtime. The NanoLib binaries are x86_64 Linux (or Windows) only, so they won't run natively on an Apple Silicon Mac. `make test` needs no NanoLib and runs anywhere.
