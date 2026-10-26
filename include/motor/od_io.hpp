#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace motor {

// Thrown for every communication failure, safety-check failure, or drive fault.
class MotorError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Minimal object dictionary access. The Motor class only talks to this
// interface, so its logic can be tested against a simulated drive without
// NanoLib or hardware. NanoLibSession provides the real implementation.
class OdIo {
public:
    virtual ~OdIo() = default;

    // Returns the raw value. Implementations throw MotorError on failure.
    virtual int64_t readNumber(uint16_t index, uint8_t subindex) = 0;
    virtual std::string readString(uint16_t index, uint8_t subindex) = 0;
    virtual void writeNumber(uint16_t index, uint8_t subindex, int64_t value, unsigned bitLength) = 0;

    // Human-readable identifier for log and error messages.
    virtual std::string describe() const = 0;
};

} // namespace motor
