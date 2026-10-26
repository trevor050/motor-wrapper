#include "motor/nanolib_session.hpp"

#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include "accessor_factory.hpp"
#include "bus_hardware_options.hpp"
#include "nano_lib_accessor.hpp"
#include "nano_lib_hw_strings.hpp"
#include "nlc_callback.hpp"
#include "nlc_log_level.hpp"
#include "od_index.hpp"

namespace motor {

namespace {

constexpr int kScanAttempts = 3;
constexpr auto kScanDelay = std::chrono::seconds(2);

class QuietScanCallback final : public nlc::NlcScanBusCallback {
public:
    nlc::ResultVoid callback(nlc::BusScanInfo, const std::vector<nlc::DeviceId>&, int32_t) override {
        return nlc::ResultVoid();
    }
};

std::string odName(uint16_t index, uint8_t subindex) {
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << index << ':' << std::dec << unsigned(subindex);
    return out.str();
}

} // namespace

namespace detail {

struct NanoLibState {
    nlc::NanoLibAccessor* nanolib = nullptr;
    std::vector<nlc::BusHardwareId> openBuses;
    std::vector<nlc::DeviceId> devices;
    bool discovered = false;

    ~NanoLibState() {
        for (const auto& bus : openBuses) {
            const auto result = nanolib->closeBusHardware(bus);
            if (result.hasError()) {
                std::cerr << "Closing " << bus.toString() << " failed: " << result.getError() << '\n';
            }
        }
    }
};

} // namespace detail

namespace {

class NanoLibDeviceIo final : public OdIo {
public:
    NanoLibDeviceIo(std::shared_ptr<detail::NanoLibState> session, const nlc::DeviceId& id)
        : session_(std::move(session)), handle_(add(*session_, id)), description_(id.toString()) {
        const auto connected = session_->nanolib->connectDevice(handle_);
        if (connected.hasError()) {
            session_->nanolib->removeDevice(handle_);
            throw MotorError("Could not connect to " + description_ + ": " + connected.getError());
        }
    }

    ~NanoLibDeviceIo() override {
        const auto disconnected = session_->nanolib->disconnectDevice(handle_);
        const auto removed = session_->nanolib->removeDevice(handle_);
        if (disconnected.hasError() || removed.hasError()) {
            std::cerr << "Cleanup of " << description_ << " reported an error.\n";
        }
    }

    int64_t readNumber(uint16_t index, uint8_t subindex) override {
        const auto result = session_->nanolib->readNumber(handle_, nlc::OdIndex(index, subindex));
        if (result.hasError()) {
            throw MotorError("Read " + odName(index, subindex) + " failed: " + result.getError());
        }
        return result.getResult();
    }

    std::string readString(uint16_t index, uint8_t subindex) override {
        const auto result = session_->nanolib->readString(handle_, nlc::OdIndex(index, subindex));
        if (result.hasError()) {
            throw MotorError("Read " + odName(index, subindex) + " failed: " + result.getError());
        }
        return result.getResult();
    }

    void writeNumber(uint16_t index, uint8_t subindex, int64_t value, unsigned bitLength) override {
        const auto result =
            session_->nanolib->writeNumber(handle_, value, nlc::OdIndex(index, subindex), bitLength);
        if (result.hasError()) {
            throw MotorError("Write " + odName(index, subindex) + " failed: " + result.getError());
        }
    }

    std::string describe() const override { return description_; }

private:
    static nlc::DeviceHandle add(detail::NanoLibState& session, const nlc::DeviceId& id) {
        const auto result = session.nanolib->addDevice(id);
        if (result.hasError()) {
            throw MotorError("Could not add " + id.toString() + ": " + result.getError());
        }
        return result.getResult();
    }

    std::shared_ptr<detail::NanoLibState> session_;
    nlc::DeviceHandle handle_;
    std::string description_;
};

} // namespace

NanoLibSession::NanoLibSession() : state_(std::make_shared<detail::NanoLibState>()) {
    state_->nanolib = getNanoLibAccessor();
    if (!state_->nanolib) throw MotorError("getNanoLibAccessor() returned null.");
    state_->nanolib->setLoggingLevel(nlc::LogLevel::Off);
}

NanoLibSession::~NanoLibSession() = default;

std::vector<ControllerInfo> NanoLibSession::discover() {
    if (state_->discovered) throw MotorError("discover() may only be called once per session.");
    state_->discovered = true;

    const auto busResult = state_->nanolib->listAvailableBusHardware();
    if (busResult.hasError()) throw MotorError("Bus hardware scan failed: " + busResult.getError());

    QuietScanCallback callback;
    std::vector<ControllerInfo> found;
    for (const auto& bus : busResult.getResult()) {
        if (bus.getProtocol() != nlc::BUS_HARDWARE_ID_PROTOCOL_MSC) continue;

        nlc::BusHardwareOptions options;
        const auto opened = state_->nanolib->openBusHardwareWithProtocol(bus, options);
        if (opened.hasError()) {
            std::cerr << "Skipping " << bus.toString() << ": " << opened.getError() << '\n';
            continue;
        }
        state_->openBuses.push_back(bus);

        // The USB interface needs a moment after opening before devices respond.
        std::string lastError;
        bool busHasDevices = false;
        for (int attempt = 1; attempt <= kScanAttempts; ++attempt) {
            std::this_thread::sleep_for(kScanDelay);
            const auto scan = state_->nanolib->scanDevices(bus, &callback);
            if (scan.hasError()) {
                lastError = scan.getError();
                continue;
            }
            if (scan.getResult().empty()) continue;
            for (const auto& id : scan.getResult()) {
                state_->devices.push_back(id);
                found.push_back({found.size(), bus.toString(), id.toString()});
            }
            busHasDevices = true;
            break;
        }
        if (!busHasDevices && !lastError.empty()) {
            std::cerr << "Scan of " << bus.toString() << " failed: " << lastError << '\n';
        }
    }
    return found;
}

std::unique_ptr<OdIo> NanoLibSession::openDevice(std::size_t index) {
    if (index >= state_->devices.size()) {
        throw MotorError("Controller index " + std::to_string(index) + " was not discovered.");
    }
    return std::make_unique<NanoLibDeviceIo>(state_, state_->devices[index]);
}

std::unique_ptr<Motor> NanoLibSession::connectMotor(std::size_t index, std::string name,
                                                    SafetyLimits limits, Timing timing) {
    return std::make_unique<Motor>(openDevice(index), std::move(name), limits, timing);
}

} // namespace motor
