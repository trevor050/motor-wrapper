#pragma once
// Independently written compile-only subset, checked against the official
// NanoLib C++ Linux x86_64 SDK 1.4.3 on 2026-10-26. These declarations model
// only the API used by this wrapper, not the complete SDK or its ABI.
// Never link against these. Use real headers and libraries for `make demo`.
#include <cstdint>
#include <string>
#include <vector>

namespace nlc {

enum class LogLevel : unsigned int { Trace, Debug, Info, Warning, Error, Critical, Off, n_levels };
enum class BusScanInfo { Start, Progress, FoundDevice, Finished };

class Result {
public:
    Result();
    virtual ~Result() = 0;
    bool hasError() const;
    const std::string getError() const;
};
class ResultVoid : public Result { public: ResultVoid(); };
class ResultInt : public Result {
public:
    ResultInt(int64_t);
    int64_t getResult() const;
};
class ResultString : public Result {
public:
    ResultString(const std::string&, bool);
    const std::string getResult() const;
};

class BusHardwareId {
public:
    virtual ~BusHardwareId() = default;
    std::string getProtocol() const;
    std::string toString() const;
};
class BusHardwareOptions {};
class DeviceId {
public:
    std::string toString() const;
};
struct DeviceHandle {
    DeviceHandle();
    DeviceHandle(uint32_t);
    bool equals(DeviceHandle) const;
    std::string toString() const;
    uint32_t get() const;
};
class ResultBusHwIds : public Result {
public:
    ResultBusHwIds(const std::vector<BusHardwareId>&);
    const std::vector<BusHardwareId> getResult() const;
};
class ResultDeviceIds : public Result {
public:
    ResultDeviceIds(const std::vector<DeviceId>&);
    const std::vector<DeviceId> getResult() const;
};
class ResultDeviceHandle : public Result {
public:
    ResultDeviceHandle(const DeviceHandle&);
    DeviceHandle getResult() const;
};

struct OdIndex {
    OdIndex();
    OdIndex(uint16_t, uint8_t);
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

} // namespace nlc
