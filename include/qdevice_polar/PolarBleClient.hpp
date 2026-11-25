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

    /// Retains a lightweight GattSession(MaintainConnection=true) for `bluetooth_address` across
    /// QTM qd_disconnect() -> qd_connect() cycles so Windows keeps the BLE link warm (~2s start
    /// instead of a 30s cold Nordic re-negotiation). Released in qd_deinitialize().
    static void retainWarmLink(uint64_t bluetooth_address);
    static void releaseWarmLinks();

    bool connect(uint64_t bluetooth_address,
                 bool enable_ecg,
                 bool enable_acc = false,
                 uint16_t acc_sample_rate_hz = 200,
                 uint16_t acc_range_g = 8);
    void disconnect();
    bool isConnected() const;
    bool isEcgActive() const;
    bool isAccActive() const;
    uint16_t activeAccRateHz() const;
    uint16_t negotiatedMtu() const;
    uint64_t streamRecoveries() const;
    std::string lastError() const;

    PolarSampleBuffer& buffer();
    const PolarSampleBuffer& buffer() const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

} // namespace qdevice_polar
