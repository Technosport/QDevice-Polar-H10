#pragma once

#include "qdevice_polar/PolarSampleBuffer.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace qdevice_polar {

struct DiscoveredPolarDevice {
    std::string name;
    std::string address_str;
    uint64_t bluetooth_address = 0;
};

/// Windows WinRT BLE client for a single Polar H10/H9/Verity Sense strap
/// (Heart Rate Service 0x180D + optional PMD ECG 130 Hz).
class PolarBleClient {
public:
    PolarBleClient();
    ~PolarBleClient();

    PolarBleClient(const PolarBleClient&) = delete;
    PolarBleClient& operator=(const PolarBleClient&) = delete;

    /// Discovers Polar BLE devices on a dedicated MTA worker thread (safe from STA UI threads).
    static std::vector<DiscoveredPolarDevice> discoverAll(uint32_t timeout_ms = 2500,
                                                          const std::string& filter = "");

    bool connect(uint64_t bluetooth_address, bool enable_ecg);
    void disconnect();
    bool isConnected() const;

    PolarSampleBuffer& buffer() { return buffer_; }
    const PolarSampleBuffer& buffer() const { return buffer_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    PolarSampleBuffer buffer_;
};

} // namespace qdevice_polar
