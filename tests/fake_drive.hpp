#pragma once

// Simulated C5-E for testing Motor without hardware. It models the CiA 402
// state machine, Profile Velocity mode with optional ramping and coasting,
// rising-edge fault reset, and a correctly commissioned bench configuration.
// Tests modify `od` or the injection fields to create problems.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "motor/od_io.hpp"

namespace motor::testing {

struct FakeDriveState {
    std::map<std::pair<uint16_t, uint8_t>, int64_t> od;
    std::vector<std::pair<uint16_t, int64_t>> writes; // (index, value) in order

    // Fault injection.
    bool faultOnEnable = false;
    bool ignoreControlword = false;
    bool failReads = false;
    int resetDelayReads = 0;                 // statusword reads before a reset completes
    std::optional<int64_t> velocityOverride; // reported by 0x606C and both feedbacks
    std::optional<int64_t> encoderOverride;  // reported by 0x60E5:03 only

    // Dynamics. 0 = velocity follows demand instantly.
    int64_t rampStepRpm = 0; // change per simulated tick (one tick per read)

    // Shared write journal for multi-drive ordering tests ("name:index=value").
    std::string name = "fake";
    std::vector<std::string>* journal = nullptr;

    // Simulation internals.
    int64_t demand = 0;
    int64_t velocity = 0;
    uint16_t lastControlword = 0;
    int pendingResetReads = -1;
};

class FakeDrive final : public OdIo {
public:
    explicit FakeDrive(FakeDriveState& state) : s_(state) {}

    static void commission(FakeDriveState& s) {
        auto& od = s.od;
        od[{0x6041, 0}] = 0x0240; // Switch on disabled
        od[{0x1001, 0}] = 0;
        od[{0x2300, 0}] = 0;
        od[{0x3202, 0}] = 1;
        od[{0x3204, 0}] = 3;
        od[{0x3204, 1}] = 0x3380;
        od[{0x3204, 2}] = 0x3390;
        od[{0x3204, 3}] = 0x33A0;
        od[{0x3203, 1}] = 0;
        od[{0x3203, 2}] = 4; // Hall: commutation
        od[{0x3203, 3}] = 3; // encoder: position + velocity
        for (uint8_t sub = 1; sub <= 3; ++sub) {
            od[{0x60E6, sub}] = sub == 1 ? 0 : (sub == 2 ? 24 : 4096);
            od[{0x60EB, sub}] = 1;
            od[{0x60E8, sub}] = 1;
            od[{0x60ED, sub}] = 1;
            od[{0x60E4, sub}] = 0;
            od[{0x60E5, sub}] = 0;
        }
        od[{0x60A9, 0}] = 0x00B44700;
        for (uint16_t index : {0x6096, 0x6097, 0x6091}) {
            od[{index, 1}] = 1;
            od[{index, 2}] = 1;
        }
        od[{0x2031, 0}] = 3000;
        od[{0x6075, 0}] = 3000;
        od[{0x6073, 0}] = 1000;
        od[{0x607F, 0}] = 3000;
        od[{0x6083, 0}] = 50;
        od[{0x6084, 0}] = 50;
        od[{0x6086, 0}] = 0;
        od[{0x60C5, 0}] = 5000;
        od[{0x60C6, 0}] = 5000;
        od[{0x6060, 0}] = 0;
        od[{0x6061, 0}] = 0;
        od[{0x60FF, 0}] = 0;
        od[{0x606B, 0}] = 0;
        od[{0x606C, 0}] = 0;
        od[{0x6064, 0}] = 0;
    }

    int64_t readNumber(uint16_t index, uint8_t sub) override {
        if (s_.failReads) throw MotorError("simulated USB loss");
        tick();
        if (index == 0x6041) advanceReset();
        const auto it = s_.od.find({index, sub});
        if (it == s_.od.end()) throw MotorError("simulated: object does not exist");
        return it->second;
    }

    std::string readString(uint16_t index, uint8_t) override {
        if (index == 0x100A) return "FIR-v2627-SIM";
        throw MotorError("simulated: not a string object");
    }

    void writeNumber(uint16_t index, uint8_t sub, int64_t value, unsigned bits) override {
        if (s_.failReads) throw MotorError("simulated USB loss");
        s_.writes.push_back({index, value});
        if (s_.journal) {
            char entry[64];
            std::snprintf(entry, sizeof entry, "%s:%04X=%lld", s_.name.c_str(), index, static_cast<long long>(value));
            s_.journal->push_back(entry);
        }
        // Real NanoLib stores the bits; mimic two's complement truncation.
        s_.od[{index, sub}] = bits == 32 ? raw32(value) : value;
        if (index == 0x6060) s_.od[{0x6061, 0}] = value;
        if (index == 0x6040) applyControlword(static_cast<uint16_t>(value));
        publish();
    }

    std::string describe() const override { return s_.name; }

private:
    static int64_t raw32(int64_t value) { return int64_t(uint32_t(int32_t(value))); }
    static int64_t signed32(int64_t raw) { return int32_t(uint32_t(raw)); }

    bool operationEnabled() const { return (s_.od.at({0x6041, 0}) & 0x6F) == 0x27; }

    static int64_t approach(int64_t from, int64_t to, int64_t step) {
        if (step <= 0) return to;
        if (from < to) return std::min(from + step, to);
        return std::max(from - step, to);
    }

    void tick() {
        const int64_t target = signed32(s_.od[{0x60FF, 0}]);
        if (operationEnabled()) {
            s_.demand = approach(s_.demand, target, s_.rampStepRpm);
            s_.velocity = approach(s_.velocity, s_.demand, s_.rampStepRpm);
        } else {
            // Power stage off: no demand, the rotor coasts down.
            s_.demand = 0;
            s_.velocity = approach(s_.velocity, 0, s_.rampStepRpm);
        }
        publish();
    }

    void publish() {
        const int64_t selected = s_.velocityOverride.value_or(s_.velocity);
        const int64_t encoder = s_.encoderOverride.value_or(selected);
        s_.od[{0x606B, 0}] = raw32(s_.demand);
        s_.od[{0x606C, 0}] = raw32(selected);
        s_.od[{0x60E5, 2}] = raw32(selected);
        s_.od[{0x60E5, 3}] = raw32(encoder);
    }

    void advanceReset() {
        if (s_.pendingResetReads < 0) return;
        if (s_.pendingResetReads-- > 0) return;
        s_.od[{0x6041, 0}] = 0x0240;
        s_.od[{0x1001, 0}] = 0;
    }

    void applyControlword(uint16_t cw) {
        const uint16_t previous = s_.lastControlword;
        s_.lastControlword = cw;
        if (s_.ignoreControlword) return;
        auto& status = s_.od[{0x6041, 0}];
        if ((status & 0x4F) == 0x08) {
            // Fault: only a rising edge of bit 7 starts a reset.
            if ((cw & 0x80) && !(previous & 0x80) && s_.pendingResetReads < 0) {
                s_.pendingResetReads = s_.resetDelayReads;
            }
            return;
        }
        switch (cw) {
        case 0x06: status = 0x0221; break;
        case 0x07: status = 0x0223; break;
        case 0x0F:
            status = s_.faultOnEnable ? 0x0208 : 0x0227;
            if (s_.faultOnEnable) s_.od[{0x1001, 0}] = 1;
            break;
        case 0x00: status = 0x0240; break;
        default: break;
        }
    }

    FakeDriveState& s_;
};

} // namespace motor::testing
