#include "qdevice_polar/PolarBleClient.hpp"
#include "qdevice_polar/PolarDiagnostics.hpp"

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winerror.h>

#include <winrt/Windows.Devices.Bluetooth.Advertisement.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Enumeration.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Storage.Streams.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

using namespace winrt;
using namespace Windows::Devices::Bluetooth;
using namespace Windows::Devices::Bluetooth::Advertisement;
using namespace Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace Windows::Devices::Enumeration;
using namespace Windows::Storage::Streams;

namespace qdevice_polar {

namespace {

// Polar H10 PMD ECG packet is 10-byte header + 73 * 3 = 229 bytes -> requires ATT MTU >= 232 (229 + 3).
constexpr uint16_t kMinPmdEcgMtu    = 232;
constexpr uint8_t kPmdOpGetSettings = 0x01;
constexpr uint8_t kPmdOpStart       = 0x02;
constexpr uint8_t kPmdOpStop        = 0x03;
constexpr uint8_t kPmdMeasEcg       = 0x00;
constexpr uint8_t kPmdMeasAcc       = 0x02;

void debugLog(const std::string& msg) {
    logDiagnostic(msg);
}

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

bool containsIgnoreCase(const std::string& haystack, const std::string& needle) {
    if (needle.empty()) return true;
    auto it = std::search(
        haystack.begin(), haystack.end(),
        needle.begin(), needle.end(),
        [](unsigned char a, unsigned char b) {
            return std::tolower(a) == std::tolower(b);
        });
    return it != haystack.end();
}

const char* gattStatusToString(GattCommunicationStatus status) {
    switch (status) {
        case GattCommunicationStatus::Success:        return "Success";
        case GattCommunicationStatus::Unreachable:    return "Unreachable";
        case GattCommunicationStatus::ProtocolError:  return "ProtocolError";
        case GattCommunicationStatus::AccessDenied:   return "AccessDenied";
        default:                                      return "Unknown";
    }
}

// Polar PMD Service & Characteristics
constexpr guid kPmdServiceUuid{ 0xFB005C80, 0x02E7, 0xF387, { 0x1C, 0xAD, 0x8A, 0xCD, 0x2D, 0x8D, 0xF0, 0xC8 } };
constexpr guid kPmdControlUuid{ 0xFB005C81, 0x02E7, 0xF387, { 0x1C, 0xAD, 0x8A, 0xCD, 0x2D, 0x8D, 0xF0, 0xC8 } };
constexpr guid kPmdDataUuid   { 0xFB005C82, 0x02E7, 0xF387, { 0x1C, 0xAD, 0x8A, 0xCD, 0x2D, 0x8D, 0xF0, 0xC8 } };

} // namespace

struct PolarBleClient::Impl {
    PolarSampleBuffer buffer{ kDefaultEcgPrebufferSamples };

    BluetoothLEDevice device{ nullptr };
    GattSession session{ nullptr };
    GattDeviceService hr_service{ nullptr };
    GattDeviceService batt_service{ nullptr };
    GattDeviceService pmd_service{ nullptr };
    GattCharacteristic hr_char{ nullptr };
    GattCharacteristic batt_char{ nullptr };
    GattCharacteristic pmd_ctrl_char{ nullptr };
    GattCharacteristic pmd_data_char{ nullptr };

    event_token conn_status_token{};
    event_token session_pdu_token{};
    event_token session_status_token{};
    event_token hr_token{};
    event_token batt_token{};
    event_token pmd_ctrl_token{};
    event_token pmd_token{};

    bool has_conn_status_token = false;
    bool has_session_pdu_token = false;
    bool has_session_status_token = false;
    bool has_hr_token = false;
    bool has_batt_token = false;
    bool has_pmd_ctrl_token = false;
    bool has_pmd_token = false;

    std::atomic<bool> connected{ false };
    std::atomic<bool> ecg_active{ false };
    std::atomic<bool> acc_active{ false };
    std::atomic<uint16_t> active_acc_rate_hz{ 200 };
    std::atomic<float> acc_factor{ 1.0f };
    std::atomic<uint16_t> max_pdu_size{ 23 };
    std::atomic<uint64_t> invalid_ecg_frames{ 0 };
    std::atomic<uint64_t> invalid_acc_frames{ 0 };

    mutable std::mutex state_mutex;
    std::string last_error;

    // Serializes connect / disconnect / watchdog recovery so the WinRT objects above are never
    // mutated by two threads at once (QTM may call qd_disconnect from another thread, and
    // ~PolarBleClient runs disconnect() after an explicit disconnect()).
    std::mutex lifecycle_mutex;

    // Link-loss watchdog: Windows (MaintainConnection=true) silently re-establishes the BLE link
    // after a dropout, but the Polar H10 stops PMD streaming on disconnect and unbonded peers
    // lose their CCCD subscriptions -> without re-arming, HR freezes and ECG flat-lines forever.
    std::thread watchdog;
    std::mutex wd_mutex;
    std::condition_variable wd_cv;
    bool wd_stop = false;
    bool wd_kick = false;
    bool cfg_enable_ecg = false;
    bool cfg_enable_acc = false;
    std::vector<uint8_t> active_start_cmd;
    std::vector<uint8_t> active_acc_start_cmd;
    std::atomic<uint64_t> recoveries{ 0 };

    std::mutex pdu_mutex;
    std::condition_variable pdu_cv;

    std::mutex cp_mutex;
    std::condition_variable cp_cv;
    std::deque<PmdControlPointResponse> cp_queue;

    void setError(const std::string& msg) {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            last_error = msg;
        }
        if (!msg.empty()) {
            debugLog("ERROR: " + msg);
        }
    }

    std::string getError() const {
        std::lock_guard<std::mutex> lock(state_mutex);
        return last_error;
    }

    bool waitForMtu(uint16_t min_mtu, uint32_t timeout_ms) {
        if (session) {
            try {
                max_pdu_size.store(session.MaxPduSize());
            } catch (...) {}
        }
        if (max_pdu_size.load() >= min_mtu) {
            return true;
        }
        std::unique_lock<std::mutex> lock(pdu_mutex);
        return pdu_cv.wait_for(
            lock,
            std::chrono::milliseconds(timeout_ms),
            [this, min_mtu]() {
                if (session) {
                    try {
                        max_pdu_size.store(session.MaxPduSize());
                    } catch (...) {}
                }
                return max_pdu_size.load() >= min_mtu;
            });
    }

    std::optional<PmdControlPointResponse> sendPmdCommandAndWaitResponse(
        const std::vector<uint8_t>& cmd,
        uint8_t expected_op_code,
        uint8_t expected_meas_type,
        uint32_t timeout_ms) {
        if (!pmd_ctrl_char || cmd.empty()) {
            return std::nullopt;
        }

        {
            std::lock_guard<std::mutex> lock(cp_mutex);
            cp_queue.clear();
        }

        try {
            DataWriter writer;
            writer.WriteBytes(array_view<const uint8_t>(cmd.data(), cmd.data() + cmd.size()));
            auto write_result = pmd_ctrl_char.WriteValueWithResultAsync(
                writer.DetachBuffer(), GattWriteOption::WriteWithResponse).get();
            if (write_result.Status() != GattCommunicationStatus::Success) {
                std::ostringstream oss;
                oss << "PMD CP write failed for op=0x" << std::hex << static_cast<int>(expected_op_code)
                    << " status=" << gattStatusToString(write_result.Status());
                if (auto proto_err = write_result.ProtocolError()) {
                    oss << " att_err=0x" << static_cast<int>(proto_err.Value());
                }
                debugLog(oss.str());
                return std::nullopt;
            }
        } catch (const hresult_error& e) {
            debugLog("PMD CP write exception: " + to_string(e.message()));
            return std::nullopt;
        } catch (...) {
            debugLog("PMD CP write unknown exception");
            return std::nullopt;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        std::optional<PmdControlPointResponse> accumulated;

        while (true) {
            std::unique_lock<std::mutex> lock(cp_mutex);
            const bool got = cp_cv.wait_until(lock, deadline, [this]() { return !cp_queue.empty(); });
            if (!got) {
                return std::nullopt;
            }

            PmdControlPointResponse resp = std::move(cp_queue.front());
            cp_queue.pop_front();
            lock.unlock();

            if (resp.op_code != expected_op_code || resp.measurement_type != expected_meas_type) {
                continue;
            }

            if (!accumulated.has_value()) {
                accumulated = std::move(resp);
            } else {
                accumulated->parameters.insert(
                    accumulated->parameters.end(),
                    resp.parameters.begin(),
                    resp.parameters.end());
                accumulated->more_frames = resp.more_frames;
                if (resp.error_code != PmdControlPointError::Success) {
                    accumulated->error_code = resp.error_code;
                    break;
                }
            }

            if (!accumulated->more_frames) {
                break;
            }
        }

        return accumulated;
    }

    /// PMD Start ECG with 0xF0 verification. Sends Stop + retries once on ALREADY_IN_STATE /
    /// INVALID_STATE / timeout, waits for MTU on INVALID_MTU. Must run on an MTA thread.
    bool startEcgStream(const std::vector<uint8_t>& start_cmd, bool reset_buffer, std::string& err_detail) {
        const std::vector<uint8_t> stop_cmd = { kPmdOpStop, kPmdMeasEcg };
        err_detail = "No 0xF0 response from PMD Control Point";
        for (int attempt = 0; attempt < 2; ++attempt) {
            if (reset_buffer) {
                // ECG-only reset: keep HR/RR/Contact that may already have arrived.
                buffer.resetEcgStreamState();
            }
            auto start_resp = sendPmdCommandAndWaitResponse(start_cmd, kPmdOpStart, kPmdMeasEcg, 2000);
            if (!start_resp.has_value()) {
                err_detail = "Timed out waiting for PMD Start ECG 0xF0 indication";
                (void)sendPmdCommandAndWaitResponse(stop_cmd, kPmdOpStop, kPmdMeasEcg, 1000);
                continue;
            }

            debugLog(std::string("PMD Start ECG response (attempt ") + std::to_string(attempt + 1) +
                     "): " + pmdErrorToString(start_resp->error_code));

            if (start_resp->error_code == PmdControlPointError::Success) {
                return true;
            }

            err_detail = std::string("PMD Start ECG rejected: ") +
                         pmdErrorToString(start_resp->error_code) +
                         " (MTU=" + std::to_string(max_pdu_size.load()) + ")";

            if (start_resp->error_code == PmdControlPointError::AlreadyInState ||
                start_resp->error_code == PmdControlPointError::InvalidState) {
                (void)sendPmdCommandAndWaitResponse(stop_cmd, kPmdOpStop, kPmdMeasEcg, 1200);
            } else if (start_resp->error_code == PmdControlPointError::InvalidMtu) {
                (void)waitForMtu(kMinPmdEcgMtu, 1500);
            } else {
                break;
            }
        }
        return false;
    }

    bool startAccStream(const std::vector<uint8_t>& start_cmd, bool reset_buffer, std::string& err_detail) {
        const std::vector<uint8_t> stop_cmd = { kPmdOpStop, kPmdMeasAcc };
        err_detail = "No 0xF0 response from PMD Control Point for ACC";
        for (int attempt = 0; attempt < 2; ++attempt) {
            if (reset_buffer) {
                buffer.resetAccStreamState();
            }
            auto start_resp = sendPmdCommandAndWaitResponse(start_cmd, kPmdOpStart, kPmdMeasAcc, 2000);
            if (!start_resp.has_value()) {
                err_detail = "Timed out waiting for PMD Start ACC 0xF0 indication";
                (void)sendPmdCommandAndWaitResponse(stop_cmd, kPmdOpStop, kPmdMeasAcc, 1000);
                continue;
            }

            debugLog(std::string("PMD Start ACC response (attempt ") + std::to_string(attempt + 1) +
                     "): " + pmdErrorToString(start_resp->error_code));

            if (start_resp->error_code == PmdControlPointError::Success) {
                acc_factor.store(parsePmdStartFactor(start_resp->parameters.data(),
                                                     start_resp->parameters.size()));
                return true;
            }

            err_detail = std::string("PMD Start ACC rejected: ") +
                         pmdErrorToString(start_resp->error_code) +
                         " (MTU=" + std::to_string(max_pdu_size.load()) + ")";

            if (start_resp->error_code == PmdControlPointError::AlreadyInState ||
                start_resp->error_code == PmdControlPointError::InvalidState) {
                (void)sendPmdCommandAndWaitResponse(stop_cmd, kPmdOpStop, kPmdMeasAcc, 1200);
            } else if (start_resp->error_code == PmdControlPointError::InvalidMtu) {
                (void)waitForMtu(kMinPmdEcgMtu, 1500);
            } else {
                break;
            }
        }
        return false;
    }

    bool linkIsUp() const {
        if (!device) return false;
        try {
            return device.ConnectionStatus() == BluetoothConnectionStatus::Connected;
        } catch (...) {
            return false;
        }
    }

    bool watchdogShouldStop() {
        std::lock_guard<std::mutex> lock(wd_mutex);
        return wd_stop;
    }

    /// Re-arms CCCDs and restarts PMD ECG after Windows re-established the link or the H10 sent
    /// an unsolicited ONLINE_MEASUREMENT_STOPPED. Caller holds lifecycle_mutex, MTA thread.
    bool recoverStreamsLocked() {
        try {
            debugLog("Watchdog: BLE link up but streams inactive -> re-arming (connected=" +
                     std::string(connected.load() ? "1" : "0") + ", ecg_active=" +
                     std::string(ecg_active.load() ? "1" : "0") + ", acc_active=" +
                     std::string(acc_active.load() ? "1" : "0") + ")");
            if (hr_char) {
                const auto st = hr_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::Notify).get();
                if (st != GattCommunicationStatus::Success) {
                    debugLog(std::string("Watchdog: HR CCCD re-arm failed: ") + gattStatusToString(st));
                    return false;
                }
            }
            if ((cfg_enable_ecg || cfg_enable_acc) && pmd_ctrl_char && pmd_data_char) {
                if (watchdogShouldStop()) return false;
                const auto cp = pmd_ctrl_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::Indicate).get();
                const auto dt = pmd_data_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::Notify).get();
                if (cp != GattCommunicationStatus::Success || dt != GattCommunicationStatus::Success) {
                    debugLog("Watchdog: PMD CCCD re-arm failed");
                    return false;
                }
                (void)waitForMtu(kMinPmdEcgMtu, 1000);
                if (watchdogShouldStop()) return false;

                if (cfg_enable_ecg && !ecg_active.load() && !active_start_cmd.empty()) {
                    const uint64_t pkts_before = buffer.ecgStats().packets_received;
                    std::string err;
                    if (!startEcgStream(active_start_cmd, false, err)) {
                        debugLog("Watchdog: PMD ECG restart failed: " + err);
                        return false;
                    }
                    ecg_active.store(true);
                    if (!buffer.waitForEcg(2500, pkts_before + 1)) {
                        debugLog("Watchdog: PMD ECG restarted but no packet within 2500 ms");
                        ecg_active.store(false);
                        return false;
                    }
                }
                if (cfg_enable_acc && !acc_active.load() && !active_acc_start_cmd.empty()) {
                    const uint64_t pkts_before = buffer.accStats().packets_received;
                    std::string err;
                    if (!startAccStream(active_acc_start_cmd, false, err)) {
                        debugLog("Watchdog: PMD ACC restart failed: " + err);
                        return false;
                    }
                    acc_active.store(true);
                    if (!buffer.waitForAcc(2500, pkts_before + 1)) {
                        debugLog("Watchdog: PMD ACC restarted but no packet within 2500 ms");
                        acc_active.store(false);
                        return false;
                    }
                }
            }
            connected.store(true);
            const auto n = recoveries.fetch_add(1) + 1;
            debugLog("Watchdog: streams recovered (recovery #" + std::to_string(n) + ")");
            return true;
        } catch (const hresult_error& e) {
            debugLog("Watchdog: recovery exception: " + to_string(e.message()));
        } catch (...) {
            debugLog("Watchdog: recovery unknown exception");
        }
        return false;
    }

    void watchdogLoop() {
        try { initMtaSafe(); } catch (...) { return; }
        auto last_attempt = std::chrono::steady_clock::now() - std::chrono::seconds(10);
        while (true) {
            {
                std::unique_lock<std::mutex> lock(wd_mutex);
                wd_cv.wait_for(lock, std::chrono::milliseconds(1000), [this]() { return wd_stop || wd_kick; });
                if (wd_stop) return;
                wd_kick = false;
            }
            const bool needs_recovery = !connected.load() ||
                                        (cfg_enable_ecg && !ecg_active.load()) ||
                                        (cfg_enable_acc && !acc_active.load());
            if (!needs_recovery) continue;
            if (std::chrono::steady_clock::now() - last_attempt < std::chrono::seconds(3)) continue;

            std::unique_lock<std::mutex> life(lifecycle_mutex, std::try_to_lock);
            if (!life.owns_lock()) continue; // disconnect() in progress
            if (watchdogShouldStop() || !linkIsUp()) continue;
            last_attempt = std::chrono::steady_clock::now();
            (void)recoverStreamsLocked();
        }
    }

    void startWatchdog() {
        {
            std::lock_guard<std::mutex> lock(wd_mutex);
            wd_stop = false;
            wd_kick = false;
        }
        watchdog = std::thread([this]() { watchdogLoop(); });
    }

    /// Must NOT be called while holding lifecycle_mutex (the watchdog may be waiting on it).
    void stopWatchdog() {
        {
            std::lock_guard<std::mutex> lock(wd_mutex);
            wd_stop = true;
        }
        wd_cv.notify_all();
        if (watchdog.joinable() && watchdog.get_id() != std::this_thread::get_id()) {
            watchdog.join();
        }
    }

    void kickWatchdog() {
        {
            std::lock_guard<std::mutex> lock(wd_mutex);
            wd_kick = true;
        }
        wd_cv.notify_all();
    }

    /// Must be called on an MTA thread (safe for synchronous WinRT `.get()` calls).
    void disconnectOnMtaThread() {
        connected.store(false);

        bool link_up = false;
        if (device) {
            try { link_up = (device.ConnectionStatus() == BluetoothConnectionStatus::Connected); } catch (...) {}
        }

        if (pmd_ctrl_char) {
            // Skip the Stop handshake when the link is already gone: a GATT write on a dead
            // link with MaintainConnection=true can block for the full Windows GATT timeout.
            if (ecg_active.load() && link_up) {
                const std::vector<uint8_t> stop_cmd = { kPmdOpStop, kPmdMeasEcg };
                (void)sendPmdCommandAndWaitResponse(stop_cmd, kPmdOpStop, kPmdMeasEcg, 600);
            }
            if (acc_active.load() && link_up) {
                const std::vector<uint8_t> stop_acc = { kPmdOpStop, kPmdMeasAcc };
                (void)sendPmdCommandAndWaitResponse(stop_acc, kPmdOpStop, kPmdMeasAcc, 600);
            }
            try {
                pmd_ctrl_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::None).get();
            } catch (...) {}
            if (has_pmd_ctrl_token) {
                try { pmd_ctrl_char.ValueChanged(pmd_ctrl_token); } catch (...) {}
                has_pmd_ctrl_token = false;
            }
            pmd_ctrl_char = nullptr;
        }

        if (pmd_data_char) {
            try {
                pmd_data_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::None).get();
            } catch (...) {}
            if (has_pmd_token) {
                try { pmd_data_char.ValueChanged(pmd_token); } catch (...) {}
                has_pmd_token = false;
            }
            pmd_data_char = nullptr;
        }

        if (hr_char) {
            // Keep bonded 0x2A37 CCCD intact so Windows 11 maintains the warm paired BLE link
            // across rapid back-to-back disconnect/connect cycles (avoids 30s cold re-negotiation).
            if (has_hr_token) {
                try { hr_char.ValueChanged(hr_token); } catch (...) {}
                has_hr_token = false;
            }
            hr_char = nullptr;
        }
        if (batt_char) {
            if (has_batt_token) {
                try { batt_char.ValueChanged(batt_token); } catch (...) {}
                has_batt_token = false;
            }
            batt_char = nullptr;
        }

        if (pmd_service) {
            try { pmd_service.Close(); } catch (...) {}
            pmd_service = nullptr;
        }
        if (batt_service) {
            try { batt_service.Close(); } catch (...) {}
            batt_service = nullptr;
        }
        if (hr_service) {
            try { hr_service.Close(); } catch (...) {}
            hr_service = nullptr;
        }
        if (session) {
            if (has_session_pdu_token) {
                try { session.MaxPduSizeChanged(session_pdu_token); } catch (...) {}
                has_session_pdu_token = false;
            }
            if (has_session_status_token) {
                try { session.SessionStatusChanged(session_status_token); } catch (...) {}
                has_session_status_token = false;
            }
            try { session.MaintainConnection(false); } catch (...) {}
            try { session.Close(); } catch (...) {}
            session = nullptr;
        }
        if (device) {
            if (has_conn_status_token) {
                try { device.ConnectionStatusChanged(conn_status_token); } catch (...) {}
                has_conn_status_token = false;
            }
            try { device.Close(); } catch (...) {}
            device = nullptr;
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }

        ecg_active.store(false);
        acc_active.store(false);
        acc_factor.store(1.0f);
        max_pdu_size.store(23);
        pdu_cv.notify_all();
        cp_cv.notify_all();
        buffer.resetConnectionState();
    }
};

PolarBleClient::PolarBleClient() : impl_(std::make_shared<Impl>()) {}

PolarBleClient::~PolarBleClient() {
    // Destructors are implicitly noexcept: an escaping std::system_error (thread creation) or
    // rethrown WinRT exception from std::future::get() would std::terminate() the QTM process.
    try {
        disconnect();
    } catch (...) {}
}

PolarSampleBuffer& PolarBleClient::buffer() {
    return impl_->buffer;
}

const PolarSampleBuffer& PolarBleClient::buffer() const {
    return impl_->buffer;
}

std::vector<DiscoveredPolarDevice> PolarBleClient::discoverAll(uint32_t timeout_ms,
                                                               const std::string& filter) {
    // Run on a dedicated MTA worker thread so WinRT async calls never collide with QTM's STA UI thread.
    return std::async(std::launch::async, [timeout_ms, filter]() {
        initMtaSafe();

        // Heap-allocated state captured by value (shared_ptr): WinRT's AdvertisementWatcher is
        // asynchronous and may still be executing an in-flight Received callback on a threadpool
        // thread while watcher.Stop() returns. Capturing stack locals by [&] would be a
        // use-after-free if a callback ran as the lambda unwound.
        struct ScanState {
            std::mutex mtx;
            std::condition_variable stopped_cv;
            bool stopped = false;
            std::vector<DiscoveredPolarDevice> found;
        };
        auto st = std::make_shared<ScanState>();

        auto tryAddCandidate = [st, filter](const std::string& name, uint64_t addr) {
            if (addr == 0 || !containsIgnoreCase(name, "Polar")) {
                return;
            }
            const std::string mac = formatMac(addr);
            if (!filter.empty() &&
                !containsIgnoreCase(name, filter) &&
                !containsIgnoreCase(mac, filter)) {
                return;
            }
            std::lock_guard<std::mutex> lock(st->mtx);
            for (const auto& existing : st->found) {
                if (existing.bluetooth_address == addr) return;
            }
            st->found.push_back({ name, mac, addr });
        };

        // 1. Active BLE Advertisement Watcher (finds unpaired or dual-channel advertising Polar straps)
        BluetoothLEAdvertisementWatcher watcher;
        watcher.ScanningMode(BluetoothLEScanningMode::Active);
        auto recv_token = watcher.Received([tryAddCandidate](const BluetoothLEAdvertisementWatcher&,
                                                             const BluetoothLEAdvertisementReceivedEventArgs& args) {
            tryAddCandidate(to_string(args.Advertisement().LocalName()), args.BluetoothAddress());
        });
        auto stop_token = watcher.Stopped([st](const BluetoothLEAdvertisementWatcher&,
                                               const BluetoothLEAdvertisementWatcherStoppedEventArgs&) {
            {
                std::lock_guard<std::mutex> lock(st->mtx);
                st->stopped = true;
            }
            st->stopped_cv.notify_all();
        });

        watcher.Start();
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        watcher.Stop();
        {
            std::unique_lock<std::mutex> lock(st->mtx);
            st->stopped_cv.wait_for(lock, std::chrono::milliseconds(500), [&]() {
                return st->stopped ||
                       watcher.Status() == BluetoothLEAdvertisementWatcherStatus::Stopped ||
                       watcher.Status() == BluetoothLEAdvertisementWatcherStatus::Aborted;
            });
        }
        try { watcher.Received(recv_token); } catch (...) {}
        try { watcher.Stopped(stop_token); } catch (...) {}

        // 2. Always union with Windows 10/11 already-connected BLE devices (not only when
        // st->found.empty()): in a multi-strap lab where Strap A is bonded (Windows background-
        // connected its 0x180D service -> stopped advertising) and Strap B is unbonded
        // (advertising), `if (found.empty())` would permanently hide Strap A.
        // Do NOT call ble_dev.Close() here: calling Close() on a connected BluetoothLEDevice
        // asks Windows to tear down the BLE link right before connect() opens it.
        try {
            auto connected_devs = DeviceInformation::FindAllAsync(
                BluetoothLEDevice::GetDeviceSelectorFromConnectionStatus(
                    BluetoothConnectionStatus::Connected)).get();
            for (uint32_t i = 0; i < connected_devs.Size(); ++i) {
                const auto info = connected_devs.GetAt(i);
                const std::string info_name = to_string(info.Name());
                if (!containsIgnoreCase(info_name, "Polar")) {
                    continue;
                }
                if (auto ble_dev = BluetoothLEDevice::FromIdAsync(info.Id()).get()) {
                    const std::string dev_name = to_string(ble_dev.Name());
                    tryAddCandidate(dev_name.empty() ? info_name : dev_name,
                                    ble_dev.BluetoothAddress());
                }
            }
        } catch (...) {}

        // 3. Passive fallback for Windows-paired Polar devices that are neither advertising nor
        // marked Connected yet. Do NOT query GATT or call ble_dev.Close() here: opening/closing a
        // probe connection tears down the BLE link right as connect() starts.
        try {
            auto paired_devs = DeviceInformation::FindAllAsync(
                BluetoothLEDevice::GetDeviceSelectorFromPairingState(true)).get();
            for (uint32_t i = 0; i < paired_devs.Size(); ++i) {
                const auto info = paired_devs.GetAt(i);
                const std::string info_name = to_string(info.Name());
                if (!containsIgnoreCase(info_name, "Polar")) {
                    continue;
                }
                if (auto ble_dev = BluetoothLEDevice::FromIdAsync(info.Id()).get()) {
                    const std::string dev_name = to_string(ble_dev.Name());
                    tryAddCandidate(dev_name.empty() ? info_name : dev_name,
                                    ble_dev.BluetoothAddress());
                }
            }
        } catch (...) {}

        std::lock_guard<std::mutex> lock(st->mtx);
        return st->found;
    }).get();
}

bool PolarBleClient::connect(uint64_t bluetooth_address,
                             bool enable_ecg,
                             bool enable_acc,
                             uint16_t acc_sample_rate_hz,
                             uint16_t acc_range_g) {
    auto self = impl_;
    if (!self) return false;

    return std::async(std::launch::async, [self, bluetooth_address, enable_ecg, enable_acc, acc_sample_rate_hz, acc_range_g]() -> bool {
        initMtaSafe();
        self->stopWatchdog();
        std::lock_guard<std::mutex> life(self->lifecycle_mutex);
        self->disconnectOnMtaThread();
        self->setError("");
        self->buffer.resetConnectionState();
        self->invalid_ecg_frames.store(0);
        self->invalid_acc_frames.store(0);
        self->recoveries.store(0);
        self->cfg_enable_ecg = enable_ecg;
        self->cfg_enable_acc = enable_acc;
        self->active_start_cmd.clear();
        self->active_acc_start_cmd.clear();
        self->acc_factor.store(1.0f);

        std::weak_ptr<Impl> weak_self = self;

        try {
            std::ostringstream mode_oss;
            mode_oss << " (HR+Batt";
            if (enable_ecg) mode_oss << " + 130Hz ECG";
            if (enable_acc) mode_oss << " + " << acc_sample_rate_hz << "Hz/" << acc_range_g << "G ACC";
            mode_oss << ")";
            debugLog("Connecting to " + formatMac(bluetooth_address) + mode_oss.str());

            self->device = BluetoothLEDevice::FromBluetoothAddressAsync(bluetooth_address).get();
            if (!self->device) {
                self->setError("Failed to open BluetoothLEDevice handle for " + formatMac(bluetooth_address));
                return false;
            }
            const bool was_warm_link = (self->device.ConnectionStatus() == BluetoothConnectionStatus::Connected);
            debugLog(std::string("Initial BLE ConnectionStatus=") + (was_warm_link ? "Connected (warm)" : "Disconnected (cold)"));

            self->conn_status_token = self->device.ConnectionStatusChanged(
                [weak_self](const BluetoothLEDevice& dev, const winrt::Windows::Foundation::IInspectable&) {
                    if (auto sp = weak_self.lock()) {
                        const bool is_up = (dev.ConnectionStatus() == BluetoothConnectionStatus::Connected);
                        if (!is_up) {
                            debugLog("BLE link dropped by remote device");
                            sp->connected.store(false);
                            sp->ecg_active.store(false);
                            sp->acc_active.store(false);
                        } else if (!sp->connected.load() ||
                                   (sp->cfg_enable_ecg && !sp->ecg_active.load()) ||
                                   (sp->cfg_enable_acc && !sp->acc_active.load())) {
                            debugLog("BLE link re-established by Windows -> waking recovery watchdog");
                            sp->kickWatchdog();
                        }
                    }
                });
            self->has_conn_status_token = true;

            // Create and retain GattSession before GATT queries so Windows maintains the link
            // and notifies us when ATT MTU (MaxPduSize) is negotiated.
            try {
                self->session = GattSession::FromDeviceIdAsync(self->device.BluetoothDeviceId()).get();
                if (self->session) {
                    self->max_pdu_size.store(self->session.MaxPduSize());
                    self->session_pdu_token = self->session.MaxPduSizeChanged(
                        [weak_self](const GattSession& sess, const winrt::Windows::Foundation::IInspectable&) {
                            if (auto sp = weak_self.lock()) {
                                const uint16_t pdu = sess.MaxPduSize();
                                {
                                    std::lock_guard<std::mutex> lock(sp->pdu_mutex);
                                    sp->max_pdu_size.store(pdu);
                                }
                                debugLog("GattSession MaxPduSize updated: " + std::to_string(pdu));
                                sp->pdu_cv.notify_all();
                            }
                        });
                    self->has_session_pdu_token = true;

                    self->session_status_token = self->session.SessionStatusChanged(
                        [weak_self](const GattSession&, const GattSessionStatusChangedEventArgs& args) {
                            if (auto sp = weak_self.lock()) {
                                if (args.Status() == GattSessionStatus::Closed) {
                                    debugLog("GattSession closed");
                                    sp->connected.store(false);
                                    sp->ecg_active.store(false);
                                    sp->acc_active.store(false);
                                }
                            }
                        });
                    self->has_session_status_token = true;

                    self->session.MaintainConnection(true);
                }
            } catch (const hresult_error& e) {
                debugLog("GattSession setup warning: " + to_string(e.message()));
            } catch (...) {}

            // 1. Standard Heart Rate Service (0x180D) -> 0x2A37 (also establishes physical BLE link & MTU exchange)
            auto hr_services = self->device.GetGattServicesForUuidAsync(
                BluetoothUuidHelper::FromShortId(0x180D), BluetoothCacheMode::Uncached).get();
            if (hr_services.Status() != GattCommunicationStatus::Success || hr_services.Services().Size() == 0) {
                self->setError(std::string("Heart Rate service (0x180D) unavailable: ") +
                               gattStatusToString(hr_services.Status()));
                self->disconnectOnMtaThread();
                return false;
            }
            self->hr_service = hr_services.Services().GetAt(0);
            auto hr_chars = self->hr_service.GetCharacteristicsForUuidAsync(
                BluetoothUuidHelper::FromShortId(0x2A37), BluetoothCacheMode::Uncached).get();
            if (hr_chars.Status() != GattCommunicationStatus::Success || hr_chars.Characteristics().Size() == 0) {
                self->setError(std::string("Heart Rate characteristic (0x2A37) unavailable: ") +
                               gattStatusToString(hr_chars.Status()));
                self->disconnectOnMtaThread();
                return false;
            }

            // NOTE: Do NOT call RequestPreferredConnectionParameters(ThroughputOptimized/Balanced):
            // Polar H10 FW 5.0.0 GAP PPCP (0x2A04) specifies supervision_timeout = 6000 ms
            // (min=40 ms, max=62.5 ms, latency=1). Windows' ThroughputOptimized (timeout=2000 ms)
            // and Balanced (timeout=4000 ms) are rejected by Polar H10's Nordic SoftDevice with a
            // peripheral-initiated BLE disconnect (~380 ms later).

            self->hr_char = hr_chars.Characteristics().GetAt(0);
            self->hr_token = self->hr_char.ValueChanged(
                [weak_self](const GattCharacteristic&, const GattValueChangedEventArgs& args) {
                    if (auto sp = weak_self.lock()) {
                        auto val = args.CharacteristicValue();
                        if (!val) return;
                        auto reader = DataReader::FromBuffer(val);
                        std::vector<uint8_t> bytes(reader.UnconsumedBufferLength());
                        if (!bytes.empty()) {
                            reader.ReadBytes(bytes);
                            sp->buffer.pushHeartRate(parseHeartRateMeasurement(bytes.data(), bytes.size()));
                        }
                    }
                });
            self->has_hr_token = true;

            const auto hr_cccd = self->hr_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                GattClientCharacteristicConfigurationDescriptorValue::Notify).get();
            if (hr_cccd != GattCommunicationStatus::Success) {
                self->setError(std::string("Failed to enable HR notifications: ") + gattStatusToString(hr_cccd));
                self->disconnectOnMtaThread();
                return false;
            }


            // 1b. Optional Standard Battery Service (0x180F) -> Battery Level (0x2A19, 0..100%)
            try {
                auto batt_services = self->device.GetGattServicesForUuidAsync(
                    BluetoothUuidHelper::FromShortId(0x180F), BluetoothCacheMode::Uncached).get();
                if (batt_services.Status() == GattCommunicationStatus::Success && batt_services.Services().Size() > 0) {
                    self->batt_service = batt_services.Services().GetAt(0);
                    auto batt_chars = self->batt_service.GetCharacteristicsForUuidAsync(
                        BluetoothUuidHelper::FromShortId(0x2A19), BluetoothCacheMode::Uncached).get();
                    if (batt_chars.Status() == GattCommunicationStatus::Success && batt_chars.Characteristics().Size() > 0) {
                        self->batt_char = batt_chars.Characteristics().GetAt(0);
                        auto read_res = self->batt_char.ReadValueAsync(BluetoothCacheMode::Uncached).get();
                        if (read_res.Status() == GattCommunicationStatus::Success && read_res.Value()) {
                            auto reader = DataReader::FromBuffer(read_res.Value());
                            std::vector<uint8_t> bytes(reader.UnconsumedBufferLength());
                            if (!bytes.empty()) {
                                reader.ReadBytes(bytes);
                                const float pct = parseBatteryLevelPercent(bytes.data(), bytes.size());
                                if (pct >= 0.0f) {
                                    self->buffer.pushBatteryLevel(pct);
                                    debugLog("Battery Level: " + std::to_string(static_cast<int>(pct)) + "%");
                                }
                            }
                        }
                        self->batt_token = self->batt_char.ValueChanged(
                            [weak_self](const GattCharacteristic&, const GattValueChangedEventArgs& args) {
                                if (auto sp = weak_self.lock()) {
                                    auto val = args.CharacteristicValue();
                                    if (!val) return;
                                    auto reader = DataReader::FromBuffer(val);
                                    std::vector<uint8_t> bytes(reader.UnconsumedBufferLength());
                                    if (!bytes.empty()) {
                                        reader.ReadBytes(bytes);
                                        const float pct = parseBatteryLevelPercent(bytes.data(), bytes.size());
                                        if (pct >= 0.0f) sp->buffer.pushBatteryLevel(pct);
                                    }
                                }
                            });
                        self->has_batt_token = true;
                        (void)self->batt_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                            GattClientCharacteristicConfigurationDescriptorValue::Notify).get();
                    }
                }
            } catch (...) {}

            // 2. Optional Polar PMD streams (ECG 130 Hz and/or ACC 25..200 Hz)
            if (enable_ecg || enable_acc) {
                auto pmd_services = self->device.GetGattServicesForUuidAsync(
                    kPmdServiceUuid, BluetoothCacheMode::Uncached).get();
                if (pmd_services.Status() != GattCommunicationStatus::Success || pmd_services.Services().Size() == 0) {
                    self->setError(std::string("Polar PMD service not found: ") +
                                   gattStatusToString(pmd_services.Status()));
                    self->disconnectOnMtaThread();
                    return false;
                }

                self->pmd_service = pmd_services.Services().GetAt(0);
                auto ctrl_res = self->pmd_service.GetCharacteristicsForUuidAsync(
                    kPmdControlUuid, BluetoothCacheMode::Uncached).get();
                auto data_res = self->pmd_service.GetCharacteristicsForUuidAsync(
                    kPmdDataUuid, BluetoothCacheMode::Uncached).get();

                if (ctrl_res.Status() != GattCommunicationStatus::Success || ctrl_res.Characteristics().Size() == 0 ||
                    data_res.Status() != GattCommunicationStatus::Success || data_res.Characteristics().Size() == 0) {
                    self->setError("Polar PMD Control/Data characteristics unavailable");
                    self->disconnectOnMtaThread();
                    return false;
                }

                self->pmd_ctrl_char = ctrl_res.Characteristics().GetAt(0);
                self->pmd_data_char = data_res.Characteristics().GetAt(0);

                // Read PMD Control Point features (0x0F ...): validates ECG capability and triggers
                // BLE link security / MTU exchange if not yet completed.
                try {
                    auto feat_res = self->pmd_ctrl_char.ReadValueAsync(BluetoothCacheMode::Uncached).get();
                    if (feat_res.Status() == GattCommunicationStatus::Success && feat_res.Value()) {
                        auto reader = DataReader::FromBuffer(feat_res.Value());
                        std::vector<uint8_t> feat_bytes(reader.UnconsumedBufferLength());
                        if (!feat_bytes.empty()) {
                            reader.ReadBytes(feat_bytes);
                            const auto features = parsePmdFeatures(feat_bytes.data(), feat_bytes.size());
                            if (features.valid && enable_ecg && !features.ecg_supported) {
                                self->setError("Connected Polar sensor does not support PMD ECG");
                                self->disconnectOnMtaThread();
                                return false;
                            }
                            if (features.valid && enable_acc && !features.acc_supported) {
                                self->setError("Connected Polar sensor does not support PMD Accelerometer");
                                self->disconnectOnMtaThread();
                                return false;
                            }
                        }
                    }
                } catch (...) {}

                // Subscribe to PMD Control Point INDICATE and PMD Data NOTIFY before sending any
                // PMD commands (required by Polar PMD spec & BlePMDClient.kt).
                self->pmd_ctrl_token = self->pmd_ctrl_char.ValueChanged(
                    [weak_self](const GattCharacteristic&, const GattValueChangedEventArgs& args) {
                        if (auto sp = weak_self.lock()) {
                            auto val = args.CharacteristicValue();
                            if (!val) return;
                            auto reader = DataReader::FromBuffer(val);
                            std::vector<uint8_t> bytes(reader.UnconsumedBufferLength());
                            if (bytes.empty()) return;
                            reader.ReadBytes(bytes);

                            bool stopped_any = false;
                            if (isPmdServerStopNotification(bytes.data(), bytes.size(), kPmdMeasEcg)) {
                                debugLog("Polar H10 sent unsolicited ONLINE_MEASUREMENT_STOPPED for ECG");
                                sp->ecg_active.store(false);
                                stopped_any = true;
                            }
                            if (isPmdServerStopNotification(bytes.data(), bytes.size(), kPmdMeasAcc)) {
                                debugLog("Polar H10 sent unsolicited ONLINE_MEASUREMENT_STOPPED for ACC");
                                sp->acc_active.store(false);
                                stopped_any = true;
                            }
                            if (stopped_any) {
                                sp->kickWatchdog();
                                return;
                            }

                            auto parsed = parsePmdControlPointResponse(bytes.data(), bytes.size());
                            if (parsed.valid) {
                                {
                                    std::lock_guard<std::mutex> lock(sp->cp_mutex);
                                    sp->cp_queue.push_back(std::move(parsed));
                                }
                                sp->cp_cv.notify_all();
                            }
                        }
                    });
                self->has_pmd_ctrl_token = true;

                const auto cp_cccd = self->pmd_ctrl_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::Indicate).get();
                if (cp_cccd != GattCommunicationStatus::Success) {
                    self->setError(std::string("Failed to enable PMD Control Point Indicate: ") +
                                   gattStatusToString(cp_cccd));
                    self->disconnectOnMtaThread();
                    return false;
                }

                self->pmd_token = self->pmd_data_char.ValueChanged(
                    [weak_self](const GattCharacteristic&, const GattValueChangedEventArgs& args) {
                        if (auto sp = weak_self.lock()) {
                            auto val = args.CharacteristicValue();
                            if (!val) return;
                            auto reader = DataReader::FromBuffer(val);
                            std::vector<uint8_t> bytes(reader.UnconsumedBufferLength());
                            if (!bytes.empty()) {
                                reader.ReadBytes(bytes);
                                if (bytes[0] == kPmdMeasEcg) {
                                    auto pkt = parsePmdEcgFrame(bytes.data(), bytes.size());
                                    if (pkt.valid) {
                                        sp->buffer.pushEcg(pkt);
                                    } else {
                                        sp->invalid_ecg_frames.fetch_add(1);
                                    }
                                } else if (bytes[0] == kPmdMeasAcc) {
                                    auto pkt = parsePmdAccFrame(bytes.data(), bytes.size(), sp->acc_factor.load());
                                    if (pkt.valid) {
                                        sp->buffer.pushAcc(pkt);
                                    } else {
                                        sp->invalid_acc_frames.fetch_add(1);
                                    }
                                }
                            }
                        }
                    });
                self->has_pmd_token = true;

                const auto data_cccd = self->pmd_data_char.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::Notify).get();
                if (data_cccd != GattCommunicationStatus::Success) {
                    self->setError(std::string("Failed to enable PMD Data Notify: ") +
                                   gattStatusToString(data_cccd));
                    self->disconnectOnMtaThread();
                    return false;
                }

                // Verify ATT MTU >= 232 bytes (229-byte ECG frames + 3-byte ATT header) after CCCD round-trips.
                const bool mtu_ready = self->waitForMtu(kMinPmdEcgMtu, 1000);
                debugLog("Negotiated GattSession MaxPduSize=" + std::to_string(self->max_pdu_size.load()) +
                         (mtu_ready ? " (OK >= 232)" : " (warning: < 232, proceeding to CP check)"));

                uint16_t selected_acc_rate = 200;
                uint16_t selected_acc_range = 8;

                // Start requested PMD streams first so ECG and ACC share any cold-link parameter
                // negotiation in parallel (~0.6 s on warm BLE link, ~31 s once on cold BLE link).
                if (enable_ecg) {
                    uint16_t selected_rate_hz = 130;
                    uint16_t selected_res_bits = 14;
                    const std::vector<uint8_t> get_settings_cmd = { kPmdOpGetSettings, kPmdMeasEcg };
                    if (auto settings_resp = self->sendPmdCommandAndWaitResponse(
                            get_settings_cmd, kPmdOpGetSettings, kPmdMeasEcg, 1500)) {
                        if (settings_resp->error_code == PmdControlPointError::Success) {
                            const auto parsed_settings = parsePmdEcgSettings(
                                settings_resp->parameters.data(), settings_resp->parameters.size());
                            if (parsed_settings.valid) {
                                if (!parsed_settings.sample_rates_hz.empty() &&
                                    std::find(parsed_settings.sample_rates_hz.begin(),
                                              parsed_settings.sample_rates_hz.end(), 130) == parsed_settings.sample_rates_hz.end()) {
                                    selected_rate_hz = parsed_settings.sample_rates_hz.front();
                                }
                                if (!parsed_settings.resolutions_bits.empty() &&
                                    std::find(parsed_settings.resolutions_bits.begin(),
                                              parsed_settings.resolutions_bits.end(), 14) == parsed_settings.resolutions_bits.end()) {
                                    selected_res_bits = parsed_settings.resolutions_bits.front();
                                }
                            }
                        } else {
                            debugLog(std::string("PMD Get ECG Settings returned: ") +
                                     pmdErrorToString(settings_resp->error_code));
                        }
                    }

                    const auto start_cmd = buildPmdStartEcgCommand(selected_rate_hz, selected_res_bits);
                    self->active_start_cmd = start_cmd;
                    std::string start_err_detail;

                    if (!self->startEcgStream(start_cmd, true, start_err_detail)) {
                        self->setError(start_err_detail);
                        self->disconnectOnMtaThread();
                        return false;
                    }
                    self->ecg_active.store(true);
                }

                if (enable_acc) {
                    selected_acc_rate = (acc_sample_rate_hz == 25 || acc_sample_rate_hz == 50 ||
                                         acc_sample_rate_hz == 100 || acc_sample_rate_hz == 200)
                                        ? acc_sample_rate_hz : 200;
                    uint16_t selected_acc_res = 16;
                    selected_acc_range = (acc_range_g == 2 || acc_range_g == 4 || acc_range_g == 8)
                                         ? acc_range_g : 8;
                    bool include_channels = false;
                    uint8_t selected_channels = 3;

                    const std::vector<uint8_t> get_acc_settings_cmd = { kPmdOpGetSettings, kPmdMeasAcc };
                    if (auto acc_settings_resp = self->sendPmdCommandAndWaitResponse(
                            get_acc_settings_cmd, kPmdOpGetSettings, kPmdMeasAcc, 1500)) {
                        if (acc_settings_resp->error_code == PmdControlPointError::Success) {
                            const auto ps = parsePmdAccSettings(
                                acc_settings_resp->parameters.data(), acc_settings_resp->parameters.size());
                            if (ps.valid) {
                                if (!ps.sample_rates_hz.empty() &&
                                    std::find(ps.sample_rates_hz.begin(), ps.sample_rates_hz.end(), selected_acc_rate) == ps.sample_rates_hz.end()) {
                                    selected_acc_rate = *std::max_element(ps.sample_rates_hz.begin(), ps.sample_rates_hz.end());
                                }
                                if (!ps.resolutions_bits.empty() &&
                                    std::find(ps.resolutions_bits.begin(), ps.resolutions_bits.end(), selected_acc_res) == ps.resolutions_bits.end()) {
                                    selected_acc_res = ps.resolutions_bits.front();
                                }
                                if (!ps.ranges_g.empty() &&
                                    std::find(ps.ranges_g.begin(), ps.ranges_g.end(), selected_acc_range) == ps.ranges_g.end()) {
                                    selected_acc_range = *std::max_element(ps.ranges_g.begin(), ps.ranges_g.end());
                                }
                                if (!ps.channels.empty()) {
                                    include_channels = true;
                                    selected_channels = ps.channels.front();
                                }
                            }
                        } else {
                            debugLog(std::string("PMD Get ACC Settings returned: ") +
                                     pmdErrorToString(acc_settings_resp->error_code));
                        }
                    }

                    self->active_acc_rate_hz.store(selected_acc_rate);
                    self->buffer.setAccNominalRateHz(selected_acc_rate);

                    const auto start_acc_cmd = buildPmdStartAccCommand(
                        selected_acc_rate, selected_acc_res, selected_acc_range, include_channels, selected_channels);
                    self->active_acc_start_cmd = start_acc_cmd;
                    std::string acc_err_detail;

                    if (!self->startAccStream(start_acc_cmd, true, acc_err_detail)) {
                        self->setError(acc_err_detail);
                        self->disconnectOnMtaThread();
                        return false;
                    }
                    self->acc_active.store(true);
                }

                // Wait for first live PMD frame(s):
                // - Warm OS BLE link: arrives in ~400-800 ms (cv wakes immediately).
                // - Cold BLE link: Polar H10 Nordic SoftDevice takes ~30.5 s once to negotiate
                //   connection parameters before flushing 226/229-byte PMD notifications; do NOT
                //   interrupt a live link at 2.5 s with Stop/Start or it resets the 30 s timer.
                // - If the BLE link actually drops mid-wait, wait for Windows reconnect and
                //   re-arm CCCDs + PMD streams via recoverStreamsLocked().
                const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(35000);
                bool recovered_once = false;
                while (std::chrono::steady_clock::now() < wait_deadline) {
                    const bool ecg_ok = !enable_ecg || self->buffer.ecgStats().packets_received >= 1;
                    const bool acc_ok = !enable_acc || self->buffer.accStats().packets_received >= 1;
                    if (ecg_ok && acc_ok) {
                        break;
                    }
                    if (!self->linkIsUp()) {
                        if (recovered_once) {
                            break;
                        }
                        recovered_once = true;
                        debugLog("BLE link dropped during PMD startup -> waiting for Windows reconnect to re-arm CCCDs");
                        for (int w = 0; w < 25 && !self->linkIsUp(); ++w) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        }
                        if (self->linkIsUp()) {
                            (void)self->recoverStreamsLocked();
                        }
                        continue;
                    }
                    if (enable_ecg && self->buffer.ecgStats().packets_received == 0) {
                        (void)self->buffer.waitForEcg(400, 1);
                    } else if (enable_acc && self->buffer.accStats().packets_received == 0) {
                        (void)self->buffer.waitForAcc(400, 1);
                    }
                }

                if (enable_ecg) {
                    if (self->buffer.ecgStats().packets_received == 0) {
                        self->setError("Polar PMD ECG started but no ECG packets arrived (MTU=" +
                                       std::to_string(self->max_pdu_size.load()) + ")");
                        self->disconnectOnMtaThread();
                        return false;
                    }
                    (void)self->buffer.waitForEcg(1500, 2);
                    debugLog("Polar PMD ECG stream live (MTU=" + std::to_string(self->max_pdu_size.load()) +
                             ", primed=" + (self->buffer.isPrimed() ? "yes" : "no") +
                             ", queued=" + std::to_string(self->buffer.ecgStats().queued_samples) + ")");
                }

                if (enable_acc) {
                    if (self->buffer.accStats().packets_received == 0) {
                        self->setError("Polar PMD ACC started but no ACC packets arrived (MTU=" +
                                       std::to_string(self->max_pdu_size.load()) + ")");
                        self->disconnectOnMtaThread();
                        return false;
                    }
                    (void)self->buffer.waitForAcc(1200, 5);
                    debugLog("Polar PMD ACC stream live (rate=" + std::to_string(selected_acc_rate) + "Hz, range=" +
                             std::to_string(selected_acc_range) + "G, primed=" +
                             (self->buffer.isAccPrimed() ? "yes" : "no") +
                             ", queued=" + std::to_string(self->buffer.accStats().queued_samples) + ")");
                }
            }

            self->connected.store(true);
            self->startWatchdog();
            return true;
        } catch (const hresult_error& e) {
            self->setError("WinRT BLE exception: " + to_string(e.message()));
            self->disconnectOnMtaThread();
            return false;
        } catch (...) {
            self->setError("Unknown exception during Polar BLE connect");
            self->disconnectOnMtaThread();
            return false;
        }
    }).get();
}

void PolarBleClient::disconnect() {
    auto self = impl_;
    if (!self) return;

    // Always run WinRT teardown on a dedicated MTA thread so synchronous .get() calls
    // (PMD Stop ECG & CCCD=None) never fail when invoked from QTM's STA UI thread.
    std::async(std::launch::async, [self]() {
        initMtaSafe();
        self->stopWatchdog();
        std::lock_guard<std::mutex> life(self->lifecycle_mutex);
        self->disconnectOnMtaThread();
    }).get();
}

bool PolarBleClient::isConnected() const {
    return impl_ && impl_->connected.load();
}

bool PolarBleClient::isEcgActive() const {
    return impl_ && impl_->ecg_active.load();
}

bool PolarBleClient::isAccActive() const {
    return impl_ && impl_->acc_active.load();
}

uint16_t PolarBleClient::activeAccRateHz() const {
    return impl_ ? impl_->active_acc_rate_hz.load() : 200;
}

uint16_t PolarBleClient::negotiatedMtu() const {
    return impl_ ? impl_->max_pdu_size.load() : 23;
}

uint64_t PolarBleClient::streamRecoveries() const {
    return impl_ ? impl_->recoveries.load() : 0;
}

std::string PolarBleClient::lastError() const {
    return impl_ ? impl_->getError() : std::string();
}

namespace {
struct WarmBleEntry {
    uint64_t address = 0;
    BluetoothLEDevice device{ nullptr };
    GattSession session{ nullptr };
};

std::mutex& warmLinksMutex() {
    static std::mutex mtx;
    return mtx;
}

std::vector<WarmBleEntry>& warmLinksList() {
    static std::vector<WarmBleEntry> entries;
    return entries;
}
} // namespace

void PolarBleClient::retainWarmLink(uint64_t bluetooth_address) {
    if (bluetooth_address == 0) return;
    std::async(std::launch::async, [bluetooth_address]() {
        try {
            initMtaSafe();
            std::lock_guard<std::mutex> lock(warmLinksMutex());
            auto& entries = warmLinksList();
            for (const auto& e : entries) {
                if (e.address == bluetooth_address && e.session) {
                    return;
                }
            }
            auto dev = BluetoothLEDevice::FromBluetoothAddressAsync(bluetooth_address).get();
            if (!dev) return;
            auto sess = GattSession::FromDeviceIdAsync(dev.BluetoothDeviceId()).get();
            if (!sess) {
                try { dev.Close(); } catch (...) {}
                return;
            }
            sess.MaintainConnection(true);
            entries.push_back({ bluetooth_address, std::move(dev), std::move(sess) });
            debugLog("Retained warm BLE link for " + formatMac(bluetooth_address));
        } catch (...) {}
    }).get();
}

void PolarBleClient::releaseWarmLinks() {
    std::async(std::launch::async, []() {
        try {
            initMtaSafe();
            std::lock_guard<std::mutex> lock(warmLinksMutex());
            auto& entries = warmLinksList();
            for (auto& e : entries) {
                if (e.session) {
                    try { e.session.MaintainConnection(false); } catch (...) {}
                    try { e.session.Close(); } catch (...) {}
                    e.session = nullptr;
                }
                if (e.device) {
                    try { e.device.Close(); } catch (...) {}
                    e.device = nullptr;
                }
            }
            entries.clear();
            debugLog("Released warm BLE links");
        } catch (...) {}
    }).get();
}

} // namespace qdevice_polar

#else

namespace qdevice_polar {
struct PolarBleClient::Impl {
    PolarSampleBuffer buffer{ kDefaultEcgPrebufferSamples };
};
PolarBleClient::PolarBleClient() : impl_(std::make_shared<Impl>()) {}
PolarBleClient::~PolarBleClient() = default;
PolarSampleBuffer& PolarBleClient::buffer() { return impl_->buffer; }
const PolarSampleBuffer& PolarBleClient::buffer() const { return impl_->buffer; }
std::vector<DiscoveredPolarDevice> PolarBleClient::discoverAll(uint32_t, const std::string&) { return {}; }
void PolarBleClient::retainWarmLink(uint64_t) {}
void PolarBleClient::releaseWarmLinks() {}
bool PolarBleClient::connect(uint64_t, bool, bool, uint16_t, uint16_t) { return false; }
void PolarBleClient::disconnect() {}
bool PolarBleClient::isConnected() const { return false; }
bool PolarBleClient::isEcgActive() const { return false; }
bool PolarBleClient::isAccActive() const { return false; }
uint16_t PolarBleClient::activeAccRateHz() const { return 200; }
uint16_t PolarBleClient::negotiatedMtu() const { return 23; }
uint64_t PolarBleClient::streamRecoveries() const { return 0; }
std::string PolarBleClient::lastError() const { return "WinRT BLE is only available on Windows"; }
} // namespace qdevice_polar

#endif
