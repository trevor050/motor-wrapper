// Hardware-free tests for motor::Motor against the simulated drive.
// Build and run with `make test`; no NanoLib installation required.

#include <algorithm>
#include <climits>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "fake_drive.hpp"
#include "motor/motor.hpp"

using motor::DriveState;
using motor::Motor;
using motor::MotorError;
using motor::testing::FakeDrive;
using motor::testing::FakeDriveState;

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        ++failures;
        std::cerr << "  FAIL: " << what << '\n';
    }
}

template <typename Fn>
bool throwsMotorError(Fn&& fn, const std::string& containing = "") {
    try {
        fn();
    } catch (const MotorError& error) {
        if (containing.empty() || std::string(error.what()).find(containing) != std::string::npos) return true;
        std::cerr << "  (unexpected message: " << error.what() << ")\n";
        return false;
    }
    return false;
}

motor::Timing fastTiming() {
    motor::Timing timing;
    timing.pollInterval = std::chrono::milliseconds(1);
    timing.stateTimeout = std::chrono::milliseconds(30);
    timing.stopTimeout = std::chrono::milliseconds(200);
    return timing;
}

std::unique_ptr<Motor> makeMotor(FakeDriveState& state) {
    return std::make_unique<Motor>(std::make_unique<FakeDrive>(state), "test", motor::SafetyLimits{}, fastTiming());
}

FakeDriveState commissioned() {
    FakeDriveState state;
    FakeDrive::commission(state);
    return state;
}

std::vector<int64_t> controlwords(const FakeDriveState& state) {
    std::vector<int64_t> result;
    for (const auto& [index, value] : state.writes) {
        if (index == 0x6040) result.push_back(value);
    }
    return result;
}

const std::vector<std::pair<std::string, std::function<void()>>> tests = {
    {"decodes CiA 402 states", [] {
         check(motor::decodeState(0x0240) == DriveState::SwitchOnDisabled, "0x0240");
         check(motor::decodeState(0x0221) == DriveState::ReadyToSwitchOn, "0x0221");
         check(motor::decodeState(0x0223) == DriveState::SwitchedOn, "0x0223");
         check(motor::decodeState(0x0627) == DriveState::OperationEnabled, "0x0627 with target reached");
         check(motor::decodeState(0x0207) == DriveState::QuickStopActive, "0x0207");
         check(motor::decodeState(0x0208) == DriveState::Fault, "0x0208");
         check(motor::decodeState(0x020F) == DriveState::FaultReactionActive, "0x020F");
     }},
    {"accepts a commissioned drive", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         check(state.writes.empty(), "verification must not write");
     }},
    {"rejects bad commissioning", [] {
         const std::vector<std::pair<std::string, std::function<void(FakeDriveState&)>>> cases = {
             {"NanoJ", [](auto& s) { s.od[{0x2300, 0}] = 1; }},
             {"velocity source", [](auto& s) { s.od[{0x3203, 2}] = 6; }},
             {"rpm", [](auto& s) { s.od[{0x60A9, 0}] = 0; }},
             {"current limits", [](auto& s) { s.od[{0x2031, 0}] = 5000; }},
             {"acceleration", [](auto& s) { s.od[{0x6083, 0}] = 1000; }},
             {"1:1", [](auto& s) { s.od[{0x6091, 1}] = 10; }},
             {"Switch on disabled", [](auto& s) { s.od[{0x6041, 0}] = 0x0208; }},
         };
         for (const auto& [expected, mutate] : cases) {
             auto state = commissioned();
             mutate(state);
             auto m = makeMotor(state);
             check(throwsMotorError([&] { m->verifyCommissioning(); }, expected), "rejects: " + expected);
         }
     }},
    {"refuses to enable without verification", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         check(throwsMotorError([&] { m->enable(); }, "verifyCommissioning"), "enable without check");
         check(state.writes.empty(), "no writes");
     }},
    {"enables through the CiA 402 sequence", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         check(m->isEnabled(), "enabled");
         check(state.writes.size() >= 2 && state.writes[0] == std::make_pair<uint16_t, int64_t>(0x60FF, 0),
               "target zeroed first");
         check(state.od[{0x6061, 0}] == 3, "Profile Velocity mode");
         check(controlwords(state) == std::vector<int64_t>{0x06, 0x07, 0x0F}, "controlword sequence");
     }},
    {"sets velocity within limits only", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         check(throwsMotorError([&] { m->setVelocityRpm(100); }, "enable()"), "set before enable");
         m->enable();
         m->setVelocityRpm(100);
         check(m->telemetry().actualRpm == 100, "drive spinning at 100");
         m->setVelocityRpm(-100);
         check(m->telemetry().actualRpm == -100, "negative velocity decoded as signed");
         check(throwsMotorError([&] { m->setVelocityRpm(351); }, "limit"), "rejects 351 rpm");
         check(throwsMotorError([&] { m->setVelocityRpm(-351); }, "limit"), "rejects -351 rpm");
         m->checkHealthy();
     }},
    {"stops and disables", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         m->setVelocityRpm(200);
         m->stopAndDisable();
         check(!m->isEnabled(), "not enabled");
         check(m->state() == DriveState::SwitchOnDisabled, "power stage off");
         check(state.od[{0x60FF, 0}] == 0, "target zero");
     }},
    {"destructor stops and disables a running motor", [] {
         auto state = commissioned();
         {
             auto m = makeMotor(state);
             m->verifyCommissioning();
             m->enable();
             m->setVelocityRpm(150);
         }
         check((state.od[{0x6041, 0}] & 0x4F) == 0x40, "disabled after destruction");
         check(state.od[{0x60FF, 0}] == 0, "target zero after destruction");
         check(controlwords(state).back() == 0x00, "last controlword is disable");
     }},
    {"fault during enable cleans up and throws", [] {
         auto state = commissioned();
         state.faultOnEnable = true;
         auto m = makeMotor(state);
         m->verifyCommissioning();
         check(throwsMotorError([&] { m->enable(); }, "fault"), "enable throws");
         check(!m->isEnabled(), "not enabled");
         check(m->state() == DriveState::Fault, "fault latched");
         m->resetFault();
         check(m->state() == DriveState::SwitchOnDisabled, "fault reset");
     }},
    {"unresponsive drive times out", [] {
         auto state = commissioned();
         state.ignoreControlword = true;
         auto m = makeMotor(state);
         m->verifyCommissioning();
         check(throwsMotorError([&] { m->enable(); }, "timed out"), "enable timeout");
     }},
    {"health check catches wrong direction and shuts down", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         state.velocityOverride = -120;
         m->setVelocityRpm(100);
         check(throwsMotorError([&] { m->checkHealthy(); }, "opposite"), "direction check");
         check(!m->isEnabled(), "shut down after failed health check");
     }},
    {"health check catches overspeed", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         state.velocityOverride = 500;
         m->setVelocityRpm(300);
         check(throwsMotorError([&] { m->checkHealthy(); }, "overspeed"), "overspeed check");
     }},
    {"health check reports lost communication", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         state.failReads = true;
         check(throwsMotorError([&] { m->checkHealthy(); }, "USB"), "comm loss surfaces");
     }},
    {"health check uses encoder velocity independently", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         m->setVelocityRpm(100);
         state.encoderOverride = 500;
         check(throwsMotorError([&] { m->checkHealthy(); }, "overspeed"), "encoder overspeed");
         auto m2State = commissioned();
         auto m2 = makeMotor(m2State);
         m2->verifyCommissioning();
         m2->enable();
         m2->setVelocityRpm(100);
         m2State.encoderOverride = -120;
         check(throwsMotorError([&] { m2->checkHealthy(); }, "opposite"), "encoder direction");
     }},
    {"health check trip zeroes target without blocking", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         m->setVelocityRpm(200);
         state.velocityOverride = 500;
         check(throwsMotorError([&] { m->checkHealthy(); }, "overspeed"), "trips");
         check(state.od[{0x60FF, 0}] == 0, "target zeroed on trip");
         check(throwsMotorError([&] { m->setVelocityRpm(50); }, "enable()"), "no new targets after trip");
         check(m->needsShutdown(), "still needs shutdown");
     }},
    {"INT32_MIN feedback is never treated as safe", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         state.velocityOverride = INT32_MIN;
         check(throwsMotorError([&] { m->checkHealthy(); }, "overspeed"), "health check rejects INT32_MIN");
         check(!m->stop(), "stop does not report standstill at INT32_MIN");
         check(throwsMotorError([&] { m->setVelocityRpm(INT_MIN); }), "INT_MIN target rejected");
     }},
    {"stop confirms standstill even after disable while coasting", [] {
         auto state = commissioned();
         state.rampStepRpm = 20;
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         m->setVelocityRpm(200);
         for (int i = 0; i < 30; ++i) m->telemetry();
         check(state.velocity == 200, "spun up");
         state.velocityOverride = 200; // keeps coasting after disable
         m->disable();
         check(!m->stop(), "stop must not claim standstill while coasting");
         state.velocityOverride.reset();
         check(m->stop(), "standstill confirmed once coasted down");
         check(state.velocity == 0, "velocity really zero");
     }},
    {"reversal through zero does not trip the direction check", [] {
         auto state = commissioned();
         state.rampStepRpm = 5;
         auto m = makeMotor(state);
         m->verifyCommissioning();
         m->enable();
         m->setVelocityRpm(100);
         for (int i = 0; i < 100; ++i) m->checkHealthy();
         check(state.velocity == 100, "at +100");
         m->setVelocityRpm(-100);
         for (int i = 0; i < 200 && state.velocity != -100; ++i) m->checkHealthy();
         check(state.velocity == -100, "reached -100 with no trip");
     }},
    {"delayed rising-edge fault reset", [] {
         auto state = commissioned();
         state.faultOnEnable = true;
         state.resetDelayReads = 3;
         auto m = makeMotor(state);
         m->verifyCommissioning();
         check(throwsMotorError([&] { m->enable(); }, "fault"), "enable faults");
         m->resetFault();
         check(m->state() == DriveState::SwitchOnDisabled, "reset completed after delay");
         check(controlwords(state).back() == 0, "bit 7 released");
     }},
    {"cancellation during enable aborts before Operation enabled", [] {
         auto state = commissioned();
         auto m = makeMotor(state);
         m->verifyCommissioning();
         int calls = 0;
         m->setCancelCheck([&] { return ++calls > 3; });
         check(throwsMotorError([&] { m->enable(); }, "cancelled"), "enable cancelled");
         const auto cws = controlwords(state);
         check(std::find(cws.begin(), cws.end(), 0x0F) == cws.end(), "never sent Enable operation");
         check(m->state() == DriveState::SwitchOnDisabled, "left disabled");
         check(!m->needsShutdown(), "cleanup completed");
     }},
    {"shutdownAll zeroes every motor before waiting on any", [] {
         std::vector<std::string> journal;
         auto a = commissioned();
         auto b = commissioned();
         a.name = "A";
         b.name = "B";
         a.journal = &journal;
         b.journal = &journal;
         a.rampStepRpm = b.rampStepRpm = 10;
         auto ma = makeMotor(a);
         auto mb = makeMotor(b);
         for (auto* m : {ma.get(), mb.get()}) {
             m->verifyCommissioning();
             m->enable();
             m->setVelocityRpm(150);
         }
         for (int i = 0; i < 40; ++i) {
             ma->telemetry();
             mb->telemetry();
         }
         journal.clear();
         motor::shutdownAll({ma.get(), mb.get()});
         check(journal.size() >= 4, "zero + disable for both");
         check(journal.size() >= 2 && journal[0] == "A:60FF=0" && journal[1] == "B:60FF=0",
               "both targets zeroed first");
         check(a.velocity == 0 && b.velocity == 0, "both stopped");
         check(ma->state() == DriveState::SwitchOnDisabled && mb->state() == DriveState::SwitchOnDisabled,
               "both disabled");
     }},
};

} // namespace

int main() {
    for (const auto& [name, test] : tests) {
        const int before = failures;
        try {
            test();
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "  FAIL: unexpected exception: " << error.what() << '\n';
        }
        std::cout << (failures == before ? "PASS  " : "FAIL  ") << name << '\n';
    }
    std::cout << (failures ? "\nFAILED: " : "\nAll tests passed. ") << failures << " failure(s)\n";
    return failures ? 1 : 0;
}
