#pragma once

// Simulated C5-E for testing Motor without hardware. It models the CiA 402
// state machine, Profile Velocity mode, and a correctly commissioned bench
// configuration. Tests modify `od` or the fault flags to inject problems.

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "motor/od_io.hpp"

namespace motor::testing {

struct FakeDriveState {
    std::map<std::pair<uint16_t, uint8_t>, int64_t> od;
    std::vector<std::pair<uint16_t, int64_t>> writes; // (index, value) in order
    bool faultOnEnable = false;
    bool ignoreControlword = false;
    bool failReads = false;
    int64_t reverseVelocity = 0; // nonzero: report this velocity regardless of target
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
        // Real NanoLib stores the bits; mimic two's complement truncation.
        const int64_t stored = bits == 32 ? int64_t(uint32_t(int32_t(value))) : value;
        s_.od[{index, sub}] = stored;
        if (index == 0x6060) s_.od[{0x6061, 0}] = value;
        if (index == 0x60FF) updateVelocity();
        if (index == 0x6040) applyControlword(uint16_t(value));
    }

    std::string describe() const override { return "fake-drive"; }

private:
    void updateVelocity() {
        const bool enabled = (s_.od[{0x6041, 0}] & 0x6F) == 0x27;
        const int64_t target = enabled ? s_.od[{0x60FF, 0}] : 0;
        const int64_t velocity = s_.reverseVelocity ? int64_t(uint32_t(int32_t(s_.reverseVelocity))) : target;
        s_.od[{0x606B, 0}] = target;
        s_.od[{0x606C, 0}] = velocity;
        s_.od[{0x60E5, 3}] = velocity;
        s_.od[{0x60E5, 2}] = velocity;
    }

    void applyControlword(uint16_t cw) {
        if (s_.ignoreControlword) return;
        auto& status = s_.od[{0x6041, 0}];
        const bool faulted = (status & 0x4F) == 0x08;
        if (faulted) {
            if (cw & 0x80) {
                status = 0x0240;
                s_.od[{0x1001, 0}] = 0;
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
        updateVelocity();
    }

    FakeDriveState& s_;
};

} // namespace motor::testing
