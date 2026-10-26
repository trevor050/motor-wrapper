#include "motor/motor.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <thread>

namespace motor {

namespace {

// Object dictionary addresses (C5-E technical manual V3.3.0, FIR-v2213 and higher).
constexpr uint16_t kErrorRegister = 0x1001;
constexpr uint16_t kFirmwareVersion = 0x100A;
constexpr uint16_t kMaxMotorCurrent = 0x2031;
constexpr uint16_t kNanoJControl = 0x2300;
constexpr uint16_t kMotorDriveSubmodeSelect = 0x3202;
constexpr uint16_t kFeedbackSelection = 0x3203;
constexpr uint16_t kFeedbackMapping = 0x3204;
constexpr uint16_t kControlword = 0x6040;
constexpr uint16_t kStatusword = 0x6041;
constexpr uint16_t kModesOfOperation = 0x6060;
constexpr uint16_t kModesOfOperationDisplay = 0x6061;
constexpr uint16_t kPositionActual = 0x6064;
constexpr uint16_t kVelocityDemand = 0x606B;
constexpr uint16_t kVelocityActual = 0x606C;
constexpr uint16_t kMaxCurrent = 0x6073;
constexpr uint16_t kRatedCurrent = 0x6075;
constexpr uint16_t kMaxProfileVelocity = 0x607F;
constexpr uint16_t kProfileAcceleration = 0x6083;
constexpr uint16_t kProfileDeceleration = 0x6084;
constexpr uint16_t kMotionProfileType = 0x6086;
constexpr uint16_t kGearRatio = 0x6091;
constexpr uint16_t kVelocityFactor = 0x6096;
constexpr uint16_t kAccelerationFactor = 0x6097;
constexpr uint16_t kVelocityUnit = 0x60A9;
constexpr uint16_t kMaxAcceleration = 0x60C5;
constexpr uint16_t kMaxDeceleration = 0x60C6;
constexpr uint16_t kFeedbackPosition = 0x60E4;
constexpr uint16_t kFeedbackVelocity = 0x60E5;
constexpr uint16_t kFeedbackIncrements = 0x60E6;
constexpr uint16_t kFeedbackGearMotorRevs = 0x60E8;
constexpr uint16_t kFeedbackMotorRevolutions = 0x60EB;
constexpr uint16_t kFeedbackGearShaftRevs = 0x60ED;
constexpr uint16_t kTargetVelocity = 0x60FF;

constexpr int64_t kModeProfileVelocity = 3;
constexpr int64_t kVelocityUnitRpm = 0x00B44700;

constexpr uint8_t kFeedbackSensorless = 1;
constexpr uint8_t kFeedbackHall = 2;
constexpr uint8_t kFeedbackEncoder = 3;
constexpr int64_t kMappingSensorless = 0x3380;
constexpr int64_t kMappingHall = 0x3390;
constexpr int64_t kMappingEncoder = 0x33A0;

constexpr int64_t kSelectVelocity = 2;
constexpr int64_t kSelectCommutation = 4;

constexpr uint16_t kCwDisableVoltage = 0x0000;
constexpr uint16_t kCwShutdown = 0x0006;
constexpr uint16_t kCwSwitchOn = 0x0007;
constexpr uint16_t kCwEnableOperation = 0x000F;
constexpr uint16_t kCwFaultReset = 0x0080;

constexpr uint16_t kSwFaultBit = 0x0008;

std::string hex(uint16_t index, uint8_t subindex) {
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << index << ':' << std::dec << unsigned(subindex);
    return out.str();
}

} // namespace

DriveState decodeState(uint16_t statusword) {
    // CiA 402 statusword patterns; x = don't care.
    if ((statusword & 0x4F) == 0x00) return DriveState::NotReadyToSwitchOn;
    if ((statusword & 0x4F) == 0x40) return DriveState::SwitchOnDisabled;
    if ((statusword & 0x6F) == 0x21) return DriveState::ReadyToSwitchOn;
    if ((statusword & 0x6F) == 0x23) return DriveState::SwitchedOn;
    if ((statusword & 0x6F) == 0x27) return DriveState::OperationEnabled;
    if ((statusword & 0x6F) == 0x07) return DriveState::QuickStopActive;
    if ((statusword & 0x4F) == 0x0F) return DriveState::FaultReactionActive;
    if ((statusword & 0x4F) == 0x08) return DriveState::Fault;
    return DriveState::Unknown;
}

const char* toString(DriveState state) {
    switch (state) {
    case DriveState::NotReadyToSwitchOn: return "Not ready to switch on";
    case DriveState::SwitchOnDisabled: return "Switch on disabled";
    case DriveState::ReadyToSwitchOn: return "Ready to switch on";
    case DriveState::SwitchedOn: return "Switched on";
    case DriveState::OperationEnabled: return "Operation enabled";
    case DriveState::QuickStopActive: return "Quick stop active";
    case DriveState::FaultReactionActive: return "Fault reaction active";
    case DriveState::Fault: return "Fault";
    case DriveState::Unknown: break;
    }
    return "Unknown";
}

Motor::Motor(std::unique_ptr<OdIo> io, std::string name, SafetyLimits limits, Timing timing)
    : io_(std::move(io)), name_(std::move(name)), limits_(limits), timing_(timing) {
    if (!io_) throw MotorError("Motor requires an OdIo implementation.");
}

Motor::~Motor() {
    if (controlAttempted_) bestEffortShutdown();
}

std::string Motor::describe() const {
    return name_ + " (" + io_->describe() + ")";
}

std::string Motor::firmwareVersion() {
    return io_->readString(kFirmwareVersion, 0);
}

int64_t Motor::read(uint16_t index, uint8_t subindex) {
    return io_->readNumber(index, subindex);
}

int32_t Motor::readSigned32(uint16_t index, uint8_t subindex) {
    // readNumber returns the raw 32 bits; reinterpret them as two's complement.
    return static_cast<int32_t>(static_cast<uint32_t>(read(index, subindex)));
}

void Motor::write(uint16_t index, int64_t value, unsigned bitLength) {
    io_->writeNumber(index, 0, value, bitLength);
}

uint16_t Motor::statusword() {
    return static_cast<uint16_t>(read(kStatusword));
}

void Motor::controlword(uint16_t value) {
    write(kControlword, value, 16);
}

DriveState Motor::state() {
    return decodeState(statusword());
}

void Motor::waitForState(DriveState expected) {
    const auto deadline = std::chrono::steady_clock::now() + timing_.stateTimeout;
    while (true) {
        const auto status = statusword();
        if (status & kSwFaultBit) {
            throw MotorError(describe() + ": drive fault while waiting for " + toString(expected) + ".");
        }
        if (decodeState(status) == expected) return;
        if (std::chrono::steady_clock::now() >= deadline) {
            throw MotorError(describe() + ": timed out waiting for " + toString(expected)
                             + "; drive is in " + toString(decodeState(status)) + ".");
        }
        std::this_thread::sleep_for(timing_.pollInterval);
    }
}

void Motor::verifyCommissioning() {
    commissioningVerified_ = false;
    auto require = [&](bool condition, const std::string& message) {
        if (!condition) throw MotorError(describe() + ": " + message);
    };

    const auto status = statusword();
    require(decodeState(status) == DriveState::SwitchOnDisabled,
            std::string("start with the drive in Switch on disabled; it is in ") + toString(decodeState(status)) + ".");
    require(read(kErrorRegister) == 0, "clear the controller error (0x1001) first.");
    require(!(read(kNanoJControl) & 1), "stop the NanoJ program in Plug & Drive Studio first.");
    require(read(kMotorDriveSubmodeSelect) & 1, "closed-loop operation is not enabled (0x3202 bit 0).");

    require(read(kFeedbackMapping, 0) == 3 && read(kFeedbackMapping, kFeedbackSensorless) == kMappingSensorless
                && read(kFeedbackMapping, kFeedbackHall) == kMappingHall
                && read(kFeedbackMapping, kFeedbackEncoder) == kMappingEncoder,
            "expected feedback mapping sensorless=1, Hall=2, incremental encoder=3 (0x3204).");

    // The drive uses the first physical feedback (Hall, then encoder) with the bit set.
    uint8_t velocitySource = 0;
    uint8_t commutationSource = 0;
    for (uint8_t sub : {kFeedbackHall, kFeedbackEncoder}) {
        const auto increments = readSigned32(kFeedbackIncrements, sub);
        const auto revolutions = read(kFeedbackMotorRevolutions, sub);
        require(increments != 0 && revolutions > 0,
                "feedback " + hex(kFeedbackIncrements, sub) + " has no resolution; check the saved configuration.");
        const auto mask = read(kFeedbackSelection, sub);
        if (!velocitySource && (mask & kSelectVelocity)) velocitySource = sub;
        if (!commutationSource && (mask & kSelectCommutation)) commutationSource = sub;
    }
    require(velocitySource == kFeedbackEncoder,
            "the encoder must be the velocity source (enable encoder velocity and clear Hall velocity).");
    require(commutationSource == kFeedbackHall || commutationSource == kFeedbackEncoder,
            "select Hall or encoder commutation; sensorless-only commutation is not accepted.");

    require(read(kVelocityUnit) == kVelocityUnitRpm, "velocity user unit (0x60A9) must be rpm.");
    for (uint16_t index : {kVelocityFactor, kAccelerationFactor, kGearRatio}) {
        const auto numerator = read(index, 1);
        const auto denominator = read(index, 2);
        require(numerator > 0 && numerator == denominator,
                "factor " + hex(index, 1) + " must be 1:1 for direct drive.");
    }
    for (uint8_t sub : {kFeedbackHall, kFeedbackEncoder}) {
        const auto motorRevs = read(kFeedbackGearMotorRevs, sub);
        require(motorRevs > 0 && motorRevs == read(kFeedbackGearShaftRevs, sub),
                "feedback gear ratio " + hex(kFeedbackGearMotorRevs, sub) + " must be 1:1.");
    }

    const auto maxCurrent = read(kMaxMotorCurrent);
    const auto ratedCurrent = read(kRatedCurrent);
    const auto peakCurrent = read(kMaxCurrent);
    require(maxCurrent > 0 && maxCurrent <= limits_.maxCurrentMilliamps && ratedCurrent > 0
                && ratedCurrent <= limits_.maxCurrentMilliamps && peakCurrent > 0
                && peakCurrent <= limits_.maxPeakCurrentPermille,
            "current limits exceed the configured bench limits (0x2031, 0x6075, 0x6073).");

    const auto acceleration = read(kProfileAcceleration);
    const auto deceleration = read(kProfileDeceleration);
    auto inRange = [&](int64_t value) {
        return value >= limits_.minAccelerationRpmPerS && value <= limits_.maxAccelerationRpmPerS;
    };
    require(inRange(acceleration) && inRange(deceleration),
            "profile acceleration/deceleration (0x6083/0x6084) outside the configured range.");
    require(read(kMotionProfileType) == 0, "motion profile type (0x6086) must be linear (0).");
    require(read(kMaxAcceleration) >= acceleration && read(kMaxDeceleration) >= deceleration,
            "maximum acceleration/deceleration (0x60C5/0x60C6) would restrict the profile ramps.");

    profileVelocityLimitRpm_ = read(kMaxProfileVelocity);
    require(profileVelocityLimitRpm_ > 0, "max profile velocity (0x607F) is zero.");

    // Probe telemetry now so a missing object fails before any write.
    telemetry();
    commissioningVerified_ = true;
}

void Motor::enable() {
    if (enabled_) return;
    if (!commissioningVerified_) {
        throw MotorError(describe() + ": call verifyCommissioning() before enable().");
    }
    const auto current = state();
    if (current == DriveState::Fault || current == DriveState::FaultReactionActive) {
        throw MotorError(describe() + ": drive is faulted; fix the cause, then call resetFault().");
    }
    if (current != DriveState::SwitchOnDisabled) {
        throw MotorError(describe() + ": expected Switch on disabled before enable, found "
                         + toString(current) + ".");
    }

    // From the first write on, cleanup must run even if a later step fails.
    controlAttempted_ = true;
    try {
        write(kTargetVelocity, 0, 32);
        lastTargetRpm_ = 0;
        write(kModesOfOperation, kModeProfileVelocity, 8);
        const auto deadline = std::chrono::steady_clock::now() + timing_.stateTimeout;
        while (read(kModesOfOperationDisplay) != kModeProfileVelocity) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw MotorError(describe() + ": Profile Velocity mode was not acknowledged (0x6061).");
            }
            std::this_thread::sleep_for(timing_.pollInterval);
        }
        controlword(kCwShutdown);
        waitForState(DriveState::ReadyToSwitchOn);
        controlword(kCwSwitchOn);
        waitForState(DriveState::SwitchedOn);
        controlword(kCwEnableOperation);
        waitForState(DriveState::OperationEnabled);
    } catch (...) {
        bestEffortShutdown();
        throw;
    }
    enabled_ = true;
}

void Motor::setVelocityRpm(int rpm) {
    if (!enabled_) throw MotorError(describe() + ": enable() the motor before setting a velocity.");
    const auto limit = std::min<int64_t>(limits_.maxRpm, profileVelocityLimitRpm_);
    if (std::abs(rpm) > limit) {
        throw MotorError(describe() + ": " + std::to_string(rpm) + " rpm exceeds the "
                         + std::to_string(limit) + " rpm limit.");
    }
    write(kTargetVelocity, rpm, 32);
    lastTargetRpm_ = rpm;
}

bool Motor::stop() {
    if (!controlAttempted_) return true;
    write(kTargetVelocity, 0, 32);
    lastTargetRpm_ = 0;
    const auto deadline = std::chrono::steady_clock::now() + timing_.stopTimeout;
    auto still = [&](int32_t rpm) { return std::abs(rpm) <= timing_.stationaryRpm; };
    int stationary = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const bool stopped = still(readSigned32(kVelocityActual))
            && still(readSigned32(kFeedbackVelocity, kFeedbackEncoder))
            && readSigned32(kVelocityDemand) == 0;
        stationary = stopped ? stationary + 1 : 0;
        if (stationary >= timing_.stationarySamples) return true;
        std::this_thread::sleep_for(timing_.pollInterval);
    }
    return false;
}

void Motor::disable() {
    controlword(kCwDisableVoltage);
    const auto deadline = std::chrono::steady_clock::now() + timing_.stateTimeout;
    while (true) {
        const auto current = state();
        // Fault also means the power stage is off; the fault stays latched for resetFault().
        if (current == DriveState::SwitchOnDisabled || current == DriveState::Fault) break;
        if (std::chrono::steady_clock::now() >= deadline) {
            throw MotorError(describe() + ": disable not confirmed; drive is in " + toString(current)
                             + ". Use the hardware stop.");
        }
        std::this_thread::sleep_for(timing_.pollInterval);
    }
    enabled_ = false;
    controlAttempted_ = false;
}

void Motor::stopAndDisable() {
    std::string stopProblem;
    try {
        if (!stop()) stopProblem = "standstill was not confirmed before disabling.";
    } catch (const std::exception& error) {
        stopProblem = std::string("stop failed: ") + error.what();
    }
    // Disable even when the stop failed.
    disable();
    if (!stopProblem.empty()) throw MotorError(describe() + ": " + stopProblem);
}

void Motor::bestEffortShutdown() noexcept {
    try {
        if (!stop()) std::cerr << describe() << ": standstill not confirmed before disable.\n";
    } catch (const std::exception& error) {
        std::cerr << describe() << ": stop failed: " << error.what() << '\n';
    }
    try {
        disable();
    } catch (const std::exception& error) {
        std::cerr << describe() << ": DISABLE FAILED: " << error.what() << " Use the hardware stop.\n";
    }
}

void Motor::resetFault() {
    const auto current = state();
    if (current != DriveState::Fault) {
        throw MotorError(describe() + ": resetFault() called, but the drive is in " + toString(current) + ".");
    }
    // The reset triggers on the rising edge of bit 7.
    controlword(0);
    controlword(kCwFaultReset);
    waitForState(DriveState::SwitchOnDisabled);
    controlword(0);
    enabled_ = false;
    controlAttempted_ = false;
}

Telemetry Motor::telemetry() {
    Telemetry t;
    t.statusword = statusword();
    t.state = decodeState(t.statusword);
    t.errorRegister = read(kErrorRegister);
    t.targetRpm = readSigned32(kTargetVelocity);
    t.demandRpm = readSigned32(kVelocityDemand);
    t.actualRpm = readSigned32(kVelocityActual);
    t.position = readSigned32(kPositionActual);
    t.hallRpm = readSigned32(kFeedbackVelocity, kFeedbackHall);
    t.encoderRpm = readSigned32(kFeedbackVelocity, kFeedbackEncoder);
    t.hallPosition = readSigned32(kFeedbackPosition, kFeedbackHall);
    t.encoderPosition = readSigned32(kFeedbackPosition, kFeedbackEncoder);
    return t;
}

void Motor::checkHealthy() {
    std::string problem;
    try {
        const auto status = statusword();
        const auto current = decodeState(status);
        const auto actual = readSigned32(kVelocityActual);
        const int margin = limits_.overspeedMarginRpm;
        if (status & kSwFaultBit) {
            problem = std::string("drive fault (") + toString(current) + ").";
        } else if (read(kErrorRegister) != 0) {
            problem = "error register 0x1001 is nonzero.";
        } else if (enabled_ && current != DriveState::OperationEnabled) {
            problem = std::string("drive left Operation enabled; now ") + toString(current) + ".";
        } else if (enabled_ && read(kModesOfOperationDisplay) != kModeProfileVelocity) {
            problem = "operation mode changed unexpectedly.";
        } else if (std::abs(actual) > limits_.maxRpm + margin) {
            problem = "overspeed: " + std::to_string(actual) + " rpm.";
        } else if ((lastTargetRpm_ > 0 && actual < -margin) || (lastTargetRpm_ < 0 && actual > margin)) {
            problem = "moving opposite to the commanded direction (" + std::to_string(actual) + " rpm).";
        }
    } catch (const std::exception& error) {
        problem = std::string("health check read failed: ") + error.what();
    }
    if (problem.empty()) return;
    if (controlAttempted_) bestEffortShutdown();
    throw MotorError(describe() + ": " + problem);
}

} // namespace motor
