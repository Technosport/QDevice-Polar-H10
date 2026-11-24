#include "qdevice_polar/PolarBleClient.hpp"

#ifdef _WIN32

#include <winerror.h>
#include <winrt/Windows.Devices.Bluetooth.Advertisement.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Storage.Streams.h>

#include <chrono>
#include <future>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>

using namespace winrt;
using namespace Windows::Devices::Bluetooth;
using namespace Windows::Devices::Bluetooth::Advertisement;
using namespace Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace Windows::Storage::Streams;

namespace qdevice_polar {

namespace {

void initMtaSafe() {
    try {
        init_apartment(apartment_type::multi_threaded);
    } catch (const hresult_error& e) {
        if (e.code() != RPC_E_CHANGED_MODE) {
            throw;
        }
    }
}

std::string formatMac(uint64_t addr) {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0');
    for (int i = 5; i >= 0; --i) {
        oss << std::setw(2) << ((addr >> (8 * i)) & 0xFF);
        if (i > 0) oss << ":";
    }
    return oss.str();
}

// Polar PMD Service & Characteristics
constexpr guid kPmdServiceUuid{ 0xFB005C80, 0x02E7, 0xF387, { 0x1C, 0xAD, 0x8A, 0xCD, 0x2D, 0x8D, 0xF0, 0xC8 } };
constexpr guid kPmdControlUuid{ 0xFB005C81, 0x02E7, 0xF387, { 0x1C, 0xAD, 0x8A, 0xCD, 0x2D, 0x8D, 0xF0, 0xC8 } };
constexpr guid kPmdDataUuid   { 0xFB005C82, 0x02E7, 0xF387, { 0x1C, 0xAD, 0x8A, 0xCD, 0x2D, 0x8D, 0xF0, 0xC8 } };

} // namespace

struct PolarBleClient::Impl {
    BluetoothLEDevice device{ nullptr };
    BluetoothLEPreferredConnectionParametersRequest conn_params_req{ nullptr };
    GattSession session{ nullptr };
    GattDeviceService hr_service{ nullptr };
    GattDeviceService pmd_service{ nullptr };
    GattCharacteristic hr_char{ nullptr };
    GattCharacteristic pmd_ctrl_char{ nullptr };
    GattCharacteristic pmd_data_char{ nullptr };
    event_token hr_token{};
    event_token pmd_ctrl_token{};
    event_token pmd_token{};
    bool connected = false;
};

PolarBleClient::PolarBleClient() : impl_(std::make_unique<Impl>()) {}

PolarBleClient::~PolarBleClient() {
    disconnect();
}

std::vector<DiscoveredPolarDevice> PolarBleClient::discoverAll(uint32_t timeout_ms,
                                                               const std::string& filter) {
    // Run on a dedicated MTA worker thread so WinRT async calls never collide with QTM's STA UI thread.
    return std::async(std::launch::async, [timeout_ms, filter]() {
        initMtaSafe();
        std::mutex mtx;
        std::vector<DiscoveredPolarDevice> found;

        BluetoothLEAdvertisementWatcher watcher;
        watcher.ScanningMode(BluetoothLEScanningMode::Active);
        auto token = watcher.Received([&](const BluetoothLEAdvertisementWatcher&,
                                          const BluetoothLEAdvertisementReceivedEventArgs& args) {
            const std::string name = to_string(args.Advertisement().LocalName());
            if (name.find("Polar") == std::string::npos) {
                return;
            }
            const uint64_t addr = args.BluetoothAddress();
            const std::string mac = formatMac(addr);
            if (!filter.empty() &&
                name.find(filter) == std::string::npos &&
                mac.find(filter) == std::string::npos) {
                return;
            }

            std::lock_guard<std::mutex> lock(mtx);
            for (const auto& existing : found) {
                if (existing.bluetooth_address == addr) return;
            }
            found.push_back({ name, mac, addr });
        });

        watcher.Start();
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        watcher.Stop();
        watcher.Received(token);
        return found;
    }).get();
}

bool PolarBleClient::connect(uint64_t bluetooth_address, bool enable_ecg) {
    disconnect();

    return std::async(std::launch::async, [this, bluetooth_address, enable_ecg]() -> bool {
        initMtaSafe();
        try {
            impl_->device = BluetoothLEDevice::FromBluetoothAddressAsync(bluetooth_address).get();
            if (!impl_->device) return false;

            try {
                // Keep the request object alive for the connection lifetime; destroying it
                // immediately reverts Windows BLE connection parameters back to default.
                impl_->conn_params_req = impl_->device.RequestPreferredConnectionParameters(
                    BluetoothLEPreferredConnectionParameters::ThroughputOptimized());
            } catch (...) {}

            try {
                impl_->session = GattSession::FromDeviceIdAsync(impl_->device.BluetoothDeviceId()).get();
                if (impl_->session) {
                    impl_->session.MaintainConnection(true);
                }
            } catch (...) {}

            // 1. Standard Heart Rate Service (0x180D) -> 0x2A37
            auto hr_services = impl_->device.GetGattServicesForUuidAsync(
                BluetoothUuidHelper::FromShortId(0x180D), BluetoothCacheMode::Uncached).get();
            if (hr_services.Status() != GattCommunicationStatus::Success || hr_services.Services().Size() == 0) {
                disconnect();
                return false;
            }
            impl_->hr_service = hr_services.Services().GetAt(0);
            auto hr_chars = impl_->hr_service.GetCharacteristicsForUuidAsync(
                BluetoothUuidHelper::FromShortId(0x2A37), BluetoothCacheMode::Uncached).get();
            if (hr_chars.Status() != GattCommunicationStatus::Success || hr_chars.Characteristics().Size() == 0) {
                disconnect();
                return false;
            }

            impl_->hr_char = hr_chars.Characteristics().GetAt(0);
            impl_->hr_token = impl_->hr_char.ValueChanged([this](const GattCharacteristic&, const GattValueChangedEventArgs& args) {
                auto reader = DataReader::FromBuffer(args.CharacteristicValue());
                std::vector<uint8_t> bytes(reader.UnconsumedBufferLength());
                if (!bytes.empty()) {
                    reader.ReadBytes(bytes);
                    buffer_.pushHeartRate(parseHeartRateMeasurement(bytes.data(), bytes.size()));
                }
            });
            impl_->hr_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                GattClientCharacteristicConfigurationDescriptorValue::Notify).get();

            // 2. Optional Polar PMD ECG stream @ 130 Hz
            if (enable_ecg) {
                auto pmd_services = impl_->device.GetGattServicesForUuidAsync(kPmdServiceUuid, BluetoothCacheMode::Uncached).get();
                if (pmd_services.Status() == GattCommunicationStatus::Success && pmd_services.Services().Size() > 0) {
                    impl_->pmd_service = pmd_services.Services().GetAt(0);
                    auto ctrl_res = impl_->pmd_service.GetCharacteristicsForUuidAsync(kPmdControlUuid, BluetoothCacheMode::Uncached).get();
                    auto data_res = impl_->pmd_service.GetCharacteristicsForUuidAsync(kPmdDataUuid, BluetoothCacheMode::Uncached).get();

                    if (ctrl_res.Characteristics().Size() > 0 && data_res.Characteristics().Size() > 0) {
                        impl_->pmd_ctrl_char = ctrl_res.Characteristics().GetAt(0);
                        impl_->pmd_data_char = data_res.Characteristics().GetAt(0);

                        // Enable Indicate on PMD Control Point before writing Start ECG command (required by Polar PMD spec)
                        impl_->pmd_ctrl_token = impl_->pmd_ctrl_char.ValueChanged([](const GattCharacteristic&, const GattValueChangedEventArgs&) {});
                        impl_->pmd_ctrl_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                            GattClientCharacteristicConfigurationDescriptorValue::Indicate).get();

                        impl_->pmd_token = impl_->pmd_data_char.ValueChanged([this](const GattCharacteristic&, const GattValueChangedEventArgs& args) {
                            auto reader = DataReader::FromBuffer(args.CharacteristicValue());
                            std::vector<uint8_t> bytes(reader.UnconsumedBufferLength());
                            if (!bytes.empty()) {
                                reader.ReadBytes(bytes);
                                buffer_.pushEcg(parsePmdEcgFrame(bytes.data(), bytes.size()));
                            }
                        });
                        impl_->pmd_data_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                            GattClientCharacteristicConfigurationDescriptorValue::Notify).get();

                        // Clear any stale PMD ECG stream first, then Start ECG @ 130 Hz 14-bit
                        try {
                            DataWriter stop_w;
                            const uint8_t stop_ecg_cmd[] = { 0x03, 0x00 };
                            stop_w.WriteBytes(stop_ecg_cmd);
                            impl_->pmd_ctrl_char.WriteValueAsync(stop_w.DetachBuffer(), GattWriteOption::WriteWithResponse).get();
                        } catch (...) {}

                        // Start ECG stream: [0x02=Start, 0x00=ECG, 0x00=SampleRate, 0x01, 0x82, 0x00 (130Hz), 0x01=Resolution, 0x01, 0x0E, 0x00 (14bit)]
                        DataWriter writer;
                        const uint8_t start_ecg_cmd[] = { 0x02, 0x00, 0x00, 0x01, 0x82, 0x00, 0x01, 0x01, 0x0E, 0x00 };
                        writer.WriteBytes(start_ecg_cmd);
                        impl_->pmd_ctrl_char.WriteValueAsync(writer.DetachBuffer(), GattWriteOption::WriteWithResponse).get();

                        // Wait until the first ECG packet arrives (up to 16 s) so QTM's t=0.00 s
                        // recording starts with live ECG data instead of leading zeros.
                        for (int wait_ms = 0; wait_ms < 16000 && !buffer_.hasReceivedEcg(); wait_ms += 50) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        }
                    }
                }
            }

            impl_->connected = true;
            return true;
        } catch (...) {
            disconnect();
            return false;
        }
    }).get();
}

void PolarBleClient::disconnect() {
    if (impl_->pmd_ctrl_char) {
        // Send Polar PMD Stop ECG ([0x03=Stop, 0x00=ECG]) so the H10 resets its PMD state
        // cleanly instead of holding a stale stream for ~20-25s on the next connection.
        try {
            DataWriter writer;
            const uint8_t stop_ecg_cmd[] = { 0x03, 0x00 };
            writer.WriteBytes(stop_ecg_cmd);
            impl_->pmd_ctrl_char.WriteValueAsync(
                writer.DetachBuffer(), GattWriteOption::WriteWithResponse).get();
        } catch (...) {}
        try { impl_->pmd_ctrl_char.ValueChanged(impl_->pmd_ctrl_token); } catch (...) {}
        impl_->pmd_ctrl_char = nullptr;
    }
    if (impl_->hr_char) {
        try { impl_->hr_char.ValueChanged(impl_->hr_token); } catch (...) {}
        impl_->hr_char = nullptr;
    }
    if (impl_->pmd_data_char) {
        try {
            impl_->pmd_data_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                GattClientCharacteristicConfigurationDescriptorValue::None).get();
        } catch (...) {}
        try { impl_->pmd_data_char.ValueChanged(impl_->pmd_token); } catch (...) {}
        impl_->pmd_data_char = nullptr;
    }
    if (impl_->pmd_service) {
        try { impl_->pmd_service.Close(); } catch (...) {}
        impl_->pmd_service = nullptr;
    }
    if (impl_->hr_service) {
        try { impl_->hr_service.Close(); } catch (...) {}
        impl_->hr_service = nullptr;
    }
    if (impl_->session) {
        try { impl_->session.Close(); } catch (...) {}
        impl_->session = nullptr;
    }
    if (impl_->conn_params_req) {
        try { impl_->conn_params_req.Close(); } catch (...) {}
        impl_->conn_params_req = nullptr;
    }
    if (impl_->device) {
        impl_->device.Close();
        impl_->device = nullptr;
        // Give Windows BLE stack & Polar H10 time to release GATT and resume advertising.
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    impl_->connected = false;
}

bool PolarBleClient::isConnected() const {
    return impl_->connected;
}

} // namespace qdevice_polar

#else

namespace qdevice_polar {
struct PolarBleClient::Impl {};
PolarBleClient::PolarBleClient() : impl_(std::make_unique<Impl>()) {}
PolarBleClient::~PolarBleClient() = default;
std::vector<DiscoveredPolarDevice> PolarBleClient::discoverAll(uint32_t, const std::string&) { return {}; }
bool PolarBleClient::connect(uint64_t, bool) { return false; }
void PolarBleClient::disconnect() {}
bool PolarBleClient::isConnected() const { return false; }
} // namespace qdevice_polar

#endif
