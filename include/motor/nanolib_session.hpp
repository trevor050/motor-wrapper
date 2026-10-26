#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "motor/motor.hpp"
#include "motor/od_io.hpp"

namespace motor {

namespace detail {
struct NanoLibState;
}

struct ControllerInfo {
    std::size_t index = 0;
    std::string bus;    // NanoLib bus description
    std::string device; // NanoLib device description
};

// Owns the NanoLib connection: discovers C5-E controllers on every USB (MSC)
// bus and opens them. Motors created here keep the session alive, so buses
// close only after the last motor is gone.
//
// Close Plug & Drive Studio first; only one process may own a controller.
class NanoLibSession {
public:
    NanoLibSession();
    ~NanoLibSession();

    NanoLibSession(const NanoLibSession&) = delete;
    NanoLibSession& operator=(const NanoLibSession&) = delete;

    // Opens all USB MSC buses and scans them. Returns every controller found,
    // in a stable index order for this run.
    std::vector<ControllerInfo> discover();

    // Connects to a discovered controller.
    std::unique_ptr<OdIo> openDevice(std::size_t index);
    std::unique_ptr<Motor> connectMotor(std::size_t index, std::string name,
                                        SafetyLimits limits = {}, Timing timing = {});

private:
    std::shared_ptr<detail::NanoLibState> state_;
};

} // namespace motor
