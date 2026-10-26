#pragma once
// Compile-only stand-ins for the NanoLib 1.4.3 headers, written from the
// signatures in the NanoLib C++ User Manual V1.4.5. Used by `make check-nanolib`
// to type-check src/nanolib_session.cpp on machines without the SDK.
// Never link against these; the real SDK headers take precedence on the robot.
#include <cstdint>
#include <string>
#include <vector>

namespace nlc {

enum class LogLevel { Trace, Debug, Info, Warn, Error, Off };
enum class BusScanInfo { Start, Progress, Finished };

class Result {
public:
    bool hasError() const { return !error_.empty(); }
    const std::string getError() const { return error_; }
protected:
    std::string error_;
};
class ResultVoid : public Result {};
class ResultInt : public Result { public: int64_t getResult() const { return 0; } };
class ResultString : public Result { public: std::string getResult() const { return {}; } };

class BusHardwareId {
public:
    std::string getProtocol() const { return {}; }
    std::string toString() const { return {}; }
};
class BusHardwareOptions {};
class DeviceId {
public:
    std::string toString() const { return {}; }
};
class DeviceHandle {
public:
    DeviceHandle(const DeviceHandle&) = default;
private:
    DeviceHandle() = default;
    friend class ResultDeviceHandle;
};
class ResultBusHwIds : public Result { public: std::vector<BusHardwareId> getResult() const { return {}; } };
class ResultDeviceIds : public Result { public: std::vector<DeviceId> getResult() const { return {}; } };
class ResultDeviceHandle : public Result { public: DeviceHandle getResult() const { return DeviceHandle(); } };

class OdIndex {
public:
    OdIndex(uint16_t, uint8_t) {}
};

class NlcScanBusCallback {
public:
    virtual ~NlcScanBusCallback() = default;
    virtual ResultVoid callback(BusScanInfo info, const std::vector<DeviceId>& devicesFound, int32_t data) = 0;
};

class NanoLibAccessor {
public:
    virtual ~NanoLibAccessor() = default;
    virtual ResultBusHwIds listAvailableBusHardware() = 0;
    virtual ResultVoid openBusHardwareWithProtocol(BusHardwareId const& busHwId, BusHardwareOptions const& busHwOpt) = 0;
    virtual ResultDeviceIds scanDevices(const BusHardwareId& busHwId, NlcScanBusCallback* callback) = 0;
    virtual ResultDeviceHandle addDevice(DeviceId const& deviceId) = 0;
    virtual ResultVoid connectDevice(DeviceHandle const deviceHandle) = 0;
    virtual ResultInt readNumber(const DeviceHandle deviceHandle, const OdIndex odIndex) = 0;
    virtual ResultString readString(const DeviceHandle deviceHandle, const OdIndex odIndex) = 0;
    virtual ResultVoid writeNumber(const DeviceHandle deviceHandle, int64_t value, const OdIndex odIndex, unsigned int bitLength) = 0;
    virtual ResultVoid disconnectDevice(DeviceHandle const deviceHandle) = 0;
    virtual ResultVoid removeDevice(const DeviceHandle deviceHandle) = 0;
    virtual ResultVoid closeBusHardware(BusHardwareId const& busHwId) = 0;
    virtual void setLoggingLevel(LogLevel level) = 0;
};

inline const std::string BUS_HARDWARE_ID_PROTOCOL_MSC = "MSC";

} // namespace nlc

nlc::NanoLibAccessor* getNanoLibAccessor();
