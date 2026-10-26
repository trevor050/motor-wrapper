#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "motor/od_io.hpp"

namespace motor {

// CiA 402 drive states, decoded from the statusword (0x6041).
enum class DriveState {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown,
};

DriveState decodeState(uint16_t statusword);
const char* toString(DriveState state);

// Bench limits enforced by verifyCommissioning() and setVelocityRpm().
// Defaults match the agreed velocity_test.cpp bench setup.
struct SafetyLimits {
    int maxRpm = 350;
    int64_t maxCurrentMilliamps = 3000;   // 0x2031 and 0x6075
    int64_t maxPeakCurrentPermille = 1000; // 0x6073, 0.1 % of rated current
    int64_t minAccelerationRpmPerS = 25;   // 0x6083 / 0x6084
    int64_t maxAccelerationRpmPerS = 100;
    // Reported velocity beyond maxRpm + this margin is treated as a runaway.
    int overspeedMarginRpm = 50;
};

struct Timing {
    std::chrono::milliseconds pollInterval{50};
    std::chrono::milliseconds stateTimeout{3000};
    std::chrono::milliseconds stopTimeout{16000};
    // Velocity within this band for stationarySamples consecutive polls counts as stopped.
    int stationaryRpm = 5;
    int stationarySamples = 3;
};

struct Telemetry {
    uint16_t statusword = 0;
    DriveState state = DriveState::Unknown;
    int64_t errorRegister = 0;   // 0x1001, 0 = no error
    int32_t targetRpm = 0;       // 0x60FF
    int32_t demandRpm = 0;       // 0x606B, ramp generator output
    int32_t actualRpm = 0;       // 0x606C, selected velocity feedback
    int32_t position = 0;        // 0x6064, selected position feedback (user units)
    int32_t hallRpm = 0;         // 0x60E5:02
    int32_t encoderRpm = 0;      // 0x60E5:03
    int32_t hallPosition = 0;    // 0x60E4:02
    int32_t encoderPosition = 0; // 0x60E4:03
};

// One Nanotec C5-E controller running in Profile Velocity mode.
//
// Typical use:
//   motor.verifyCommissioning();  // refuses unsafe or unexpected configuration
//   motor.enable();               // CiA 402: Shutdown -> Switch on -> Enable operation
//   motor.setVelocityRpm(100);
//   ... poll motor.telemetry() / motor.checkHealthy() ...
//   motor.stopAndDisable();
//
// For several motors, use shutdownAll() so every motor is commanded to zero
// before waiting on any of them.
//
// The destructor makes a best-effort stop and disable. No software cleanup can
// stop the motor if the process is killed or USB is lost; keep a hardware stop.
class Motor {
public:
    Motor(std::unique_ptr<OdIo> io, std::string name, SafetyLimits limits = {}, Timing timing = {});
    ~Motor();

    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    const std::string& name() const { return name_; }
    std::string describe() const;
    std::string firmwareVersion();

    // Reads the configuration saved by Plug & Drive Studio and throws a
    // MotorError naming the first problem. enable() requires a passing check.
    void verifyCommissioning();

    // Optional cancellation (for example a Ctrl+C flag). enable() checks it
    // before every write and while waiting, and aborts into shutdown.
    void setCancelCheck(std::function<bool()> cancelled) { cancelled_ = std::move(cancelled); }

    // Enters Profile Velocity mode with a zero target and walks the drive to
    // Operation enabled. Requires Switch on disabled without a fault.
    void enable();

    // Signed target velocity. Requires enable(). Rejects |rpm| above the limit.
    void setVelocityRpm(int rpm);

    // Writes a zero target velocity without waiting.
    void commandZero();

    // Polls until selected velocity, encoder velocity, and demand all read zero
    // (within Timing::stationaryRpm) for Timing::stationarySamples polls.
    bool waitForStandstill(std::chrono::steady_clock::time_point deadline);

    // commandZero() then waitForStandstill(). Returns false if standstill was
    // not confirmed before Timing::stopTimeout.
    bool stop();

    // Disables the power stage and confirms Switch on disabled.
    void disable();

    // stop() then disable(); throws if either step fails.
    void stopAndDisable();

    // Clears a drive fault (controlword bit 7 rising edge). The cause must be fixed first.
    void resetFault();

    DriveState state();
    bool isEnabled() const { return enabled_; }
    // True once any enable write was sent, until disable is confirmed.
    bool needsShutdown() const { return controlAttempted_; }
    const Timing& timing() const { return timing_; }
    Telemetry telemetry();

    // Throws if the drive faulted, left Operation enabled, changed mode, reports
    // an error, exceeds the overspeed limit, or moves against the ramp demand
    // (selected and encoder velocity are both checked). On failure it commands
    // zero immediately, refuses further setVelocityRpm() calls, and throws; the
    // caller then runs stopAndDisable() or shutdownAll(). Call it every loop.
    void checkHealthy();

private:
    int64_t read(uint16_t index, uint8_t subindex = 0);
    int32_t readSigned32(uint16_t index, uint8_t subindex = 0);
    void write(uint16_t index, int64_t value, unsigned bitLength);
    uint16_t statusword();
    void controlword(uint16_t value);
    void waitForState(DriveState expected);
    void throwIfCancelled(const char* step);
    void bestEffortShutdown() noexcept;

    std::unique_ptr<OdIo> io_;
    std::string name_;
    SafetyLimits limits_;
    Timing timing_;
    bool commissioningVerified_ = false;
    bool controlAttempted_ = false; // any enable write sent; cleanup required
    bool enabled_ = false;          // reached Operation enabled
    int64_t profileVelocityLimitRpm_ = 0;
    std::function<bool()> cancelled_;
};

// Coordinated shutdown: commands zero on every motor first, then waits for
// standstill on all of them against one shared deadline, then disables each.
// Every motor is attempted even if another one fails. Never throws.
void shutdownAll(const std::vector<Motor*>& motors) noexcept;

} // namespace motor
