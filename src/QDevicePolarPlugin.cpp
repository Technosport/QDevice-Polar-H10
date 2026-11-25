#include "qdevice_polar/PolarBleClient.hpp"
#include "qdevice_polar/PolarDiagnostics.hpp"

#include <qualisys/qdevice-api/qdevice_api.h>
#include <qualisys/qdevice-api/qdevice_utilities.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using clock_t_qd = std::chrono::steady_clock;

constexpr const char* INTEGRATION_NAME    = "Polar H10 Heart Rate Monitor";
constexpr const char* INTEGRATION_DESC    = "BLE GATT integration for Polar H10 (BPM, RR, Contact, Battery, optional 130 Hz ECG & 200 Hz 3-axis Accelerometer)";
constexpr const char* INTEGRATION_URL     = "https://www.polar.com/en/sensors/h10-heart-rate-sensor";
constexpr const char* INTEGRATION_VERSION = "0.3.0";

constexpr std::int32_t kMinDiscoveryTimeoutMs = 500;
constexpr std::int32_t kMaxDiscoveryTimeoutMs = 15000;

bool envFlagEnabled(const char* env_name) {
#ifdef _WIN32
    char* buf = nullptr;
    size_t len = 0;
    if (_dupenv_s(&buf, &len, env_name) == 0 && buf != nullptr) {
        const std::string val(buf);
        std::free(buf);
        return (val == "1" || val == "true" || val == "TRUE" || val == "on" || val == "ON");
    }
    return false;
#else
    const char* val = std::getenv(env_name);
    if (!val) return false;
    const std::string s(val);
    return (s == "1" || s == "true" || s == "TRUE" || s == "on" || s == "ON");
#endif
}

bool defaultEnableEcgFromEnv() {
    return envFlagEnabled("QDEVICE_POLAR_ECG");
}

bool defaultEnableAccFromEnv() {
    return envFlagEnabled("QDEVICE_POLAR_ACC");
}

std::int32_t sanitizeAccRateHz(std::int32_t rate_hz) {
    if (rate_hz <= 37)  return 25;
    if (rate_hz <= 75)  return 50;
    if (rate_hz <= 150) return 100;
    return 200;
}

std::int32_t sanitizeAccRangeG(std::int32_t range_g) {
    if (range_g <= 3) return 2;
    if (range_g <= 6) return 4;
    return 8;
}

constexpr float kHrChannelRateHz  = 1.0f;
constexpr float kEcgChannelRateHz = 130.0f;

struct ManagedPolarDevice {
    qdevice_polar::DiscoveredPolarDevice info;
    std::unique_ptr<qdevice_polar::PolarBleClient> client;
};

struct PluginState {
    std::mutex mutex;
    qd_state state = qd_state_disconnected;
    std::int32_t discovery_timeout_ms = 2500;
    std::int32_t buffer_latency_ms = 850;
    bool enable_ecg = defaultEnableEcgFromEnv();
    bool enable_acc = defaultEnableAccFromEnv();
    std::int32_t acc_rate_hz = 200;
    std::int32_t acc_range_g = 8;

    bool active_enable_ecg = false;
    bool active_enable_acc = false;
    std::int32_t active_acc_rate_hz = 200;
    std::int32_t active_acc_range_g = 8;

    std::string device_filter;
    std::string last_discovery_filter;

    std::vector<qdevice_polar::DiscoveredPolarDevice> discovered;
    std::vector<ManagedPolarDevice> connected_devices;

    clock_t_qd::time_point stream_start_time{};
    std::optional<clock_t_qd::time_point> stream_stop_time{};
    std::uint64_t hr_samples_read = 0;
    std::uint64_t ecg_samples_read = 0;
    std::uint64_t acc_samples_read = 0;
};

PluginState& getState() {
    static PluginState instance;
    return instance;
}

qd_call_status okStatus() {
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status errorStatus(const std::string& msg) {
    // Kept in thread-local storage so the returned `const char*` remains valid after
    // the caller releases `PluginState::mutex` (even if another thread calls an entry point).
    thread_local std::string tl_last_error;
    tl_last_error = msg;
    qdevice_polar::logDiagnostic("Plugin Error: " + msg);
    return { qd_call_status_type_error, tl_last_error.c_str() };
}

std::int32_t clampDiscoveryTimeout(std::int32_t timeout_ms) {
    return std::clamp(timeout_ms, kMinDiscoveryTimeoutMs, kMaxDiscoveryTimeoutMs);
}

std::string deviceSerial(const qdevice_polar::DiscoveredPolarDevice& info, size_t index) {
    return info.address_str.empty()
        ? ("POLAR-H10-" + std::to_string(index + 1))
        : info.address_str;
}

void populateDeviceConfig(struct qd_device& dst,
                          const qdevice_polar::DiscoveredPolarDevice& dev,
                          size_t index,
                          bool ecg_enabled,
                          bool acc_enabled,
                          std::int32_t acc_rate_hz) {
    const size_t ch_count = 4 + (ecg_enabled ? 1 : 0) + (acc_enabled ? 3 : 0);
    const float acc_native_hz = static_cast<float>(sanitizeAccRateHz(acc_rate_hz));
    const std::string serial = deviceSerial(dev, index);

    dst = QDevice::Utilities::create_device(
        qd_device_category_generic, serial, ch_count, 1);

    dst.properties.data[0] = QDevice::Utilities::create_property(
        "Device Name", true, qd_property_type_string_utf8, dev.name, dev.name, "");

    // Native per-sensor hardware frequencies (Delsys/Noraxon/x-IMU3 multi-rate model):
    //   - HR / RR / Contact / Battery : 1 Hz
    //   - ECG (Polar PMD 0x00)        : 130 Hz native (no resampling)
    //   - Acc X/Y/Z (Polar PMD 0x02)  : 200 Hz native (or 25/50/100 Hz configured, no resampling)
    size_t ch = 0;
    dst.channels.data[ch++] = QDevice::Utilities::create_channel(
        "Heart Rate", qd_sample_encoding_float32, qd_channel_unit_beats_per_minute, kHrChannelRateHz);
    dst.channels.data[ch++] = QDevice::Utilities::create_channel(
        "RR Interval Ms", qd_sample_encoding_float32, qd_channel_unit_none, kHrChannelRateHz);
    dst.channels.data[ch++] = QDevice::Utilities::create_channel(
        "Sensor Contact", qd_sample_encoding_float32, qd_channel_unit_none, kHrChannelRateHz);
    dst.channels.data[ch++] = QDevice::Utilities::create_channel(
        "Battery Pct", qd_sample_encoding_float32, qd_channel_unit_none, kHrChannelRateHz);

    if (ecg_enabled) {
        dst.channels.data[ch++] = QDevice::Utilities::create_channel(
            "ECG", qd_sample_encoding_float32, qd_channel_unit_microvolts, kEcgChannelRateHz);
    }

    if (acc_enabled) {
        dst.channels.data[ch++] = QDevice::Utilities::create_channel(
            "Acc X", qd_sample_encoding_float32, qd_channel_unit_none, acc_native_hz);
        dst.channels.data[ch++] = QDevice::Utilities::create_channel(
            "Acc Y", qd_sample_encoding_float32, qd_channel_unit_none, acc_native_hz);
        dst.channels.data[ch++] = QDevice::Utilities::create_channel(
            "Acc Z", qd_sample_encoding_float32, qd_channel_unit_none, acc_native_hz);
    }
}

void assignConfigProperties(PluginState& st, const struct qd_configuration* config) {
    if (!config) return;
    QDevice::Utilities::try_assign_property_value("Discovery Timeout Ms", config->properties, st.discovery_timeout_ms);
    QDevice::Utilities::try_assign_property_value("Buffer Latency Ms", config->properties, st.buffer_latency_ms);
    QDevice::Utilities::try_assign_property_value("Enable ECG", config->properties, st.enable_ecg);
    QDevice::Utilities::try_assign_property_value("Enable Accelerometer", config->properties, st.enable_acc);
    QDevice::Utilities::try_assign_property_value("Accelerometer Rate Hz", config->properties, st.acc_rate_hz);
    QDevice::Utilities::try_assign_property_value("Accelerometer Range G", config->properties, st.acc_range_g);
    QDevice::Utilities::try_assign_property_value("Device Filter", config->properties, st.device_filter);
    st.discovery_timeout_ms = clampDiscoveryTimeout(st.discovery_timeout_ms);
    st.buffer_latency_ms = std::clamp(st.buffer_latency_ms, 300, 1500);
    st.acc_rate_hz = sanitizeAccRateHz(st.acc_rate_hz);
    st.acc_range_g = sanitizeAccRangeG(st.acc_range_g);
}

qd_call_status QD_STDCALL actionLocateDevices(const struct qd_configuration* config) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    if (st.state != qd_state_disconnected) {
        return errorStatus("Disconnect before locating devices");
    }
    try {
        assignConfigProperties(st, config);
    } catch (...) {}
    st.discovered = qdevice_polar::PolarBleClient::discoverAll(
        static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
    st.last_discovery_filter = st.device_filter;
    return okStatus();
}

} // namespace

qd_call_status qd_get_integration_version(struct qd_static_chars* version) {
    if (!version) return errorStatus("Null output pointer");
    *version = { INTEGRATION_VERSION, std::strlen(INTEGRATION_VERSION) };
    return okStatus();
}

qd_call_status qd_get_name(struct qd_static_chars* name) {
    if (!name) return errorStatus("Null output pointer");
    *name = { INTEGRATION_NAME, std::strlen(INTEGRATION_NAME) };
    return okStatus();
}

qd_call_status qd_get_description(struct qd_static_chars* desc) {
    if (!desc) return errorStatus("Null output pointer");
    *desc = { INTEGRATION_DESC, std::strlen(INTEGRATION_DESC) };
    return okStatus();
}

qd_call_status qd_get_manufacturer_url(struct qd_static_chars* url) {
    if (!url) return errorStatus("Null output pointer");
    *url = { INTEGRATION_URL, std::strlen(INTEGRATION_URL) };
    return okStatus();
}

qd_call_status qd_get_contracts(struct qd_contracts* contracts) {
    if (!contracts) return errorStatus("Null output pointer");
    *contracts = { nullptr, 0 };
    return okStatus();
}

qd_call_status qd_get_category(qd_device_category* category) {
    if (!category) return errorStatus("Null output pointer");
    *category = qd_device_category_generic;
    return okStatus();
}

qd_call_status qd_initialize() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.state = qd_state_disconnected;
    st.enable_ecg = defaultEnableEcgFromEnv();
    st.enable_acc = defaultEnableAccFromEnv();
    st.acc_rate_hz = 200;
    st.acc_range_g = 8;
    st.active_enable_ecg = false;
    st.active_enable_acc = false;
    st.active_acc_rate_hz = 200;
    st.active_acc_range_g = 8;
    st.hr_samples_read = 0;
    st.ecg_samples_read = 0;
    st.acc_samples_read = 0;
    return okStatus();
}

qd_call_status qd_deinitialize() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.connected_devices.clear();
    st.discovered.clear();
    qdevice_polar::PolarBleClient::releaseWarmLinks();
    st.active_enable_ecg = false;
    st.active_enable_acc = false;
    st.state = qd_state_disconnected;
    return okStatus();
}

qd_call_status qd_get_actions(struct qd_actions* actions) {
    if (!actions) return errorStatus("Null output pointer");
    static constexpr const char ACTION_NAME[] = "Locate Devices";
    static const qd_action kActions[] = {
        { { ACTION_NAME, sizeof(ACTION_NAME) - 1 }, &actionLocateDevices }
    };
    *actions = { kActions, 1 };
    return okStatus();
}

qd_call_status qd_get_configuration(struct qd_configuration* configuration) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (!configuration) {
        return errorStatus("Null configuration pointer");
    }

    const bool is_connected = (st.state != qd_state_disconnected);
    const bool ecg_enabled = is_connected ? st.active_enable_ecg : st.enable_ecg;
    const bool acc_enabled = is_connected ? st.active_enable_acc : st.enable_acc;
    const std::int32_t acc_rate = is_connected ? st.active_acc_rate_hz : st.acc_rate_hz;
    const std::int32_t acc_range = is_connected ? st.active_acc_range_g : st.acc_range_g;

    const char* default_ecg_str = defaultEnableEcgFromEnv() ? "true" : "false";
    const char* default_acc_str = defaultEnableAccFromEnv() ? "true" : "false";

    if (!is_connected && st.discovered.empty()) {
        st.discovery_timeout_ms = clampDiscoveryTimeout(st.discovery_timeout_ms);
        st.discovered = qdevice_polar::PolarBleClient::discoverAll(
            static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
        st.last_discovery_filter = st.device_filter;
    }

    const size_t dev_count = is_connected ? st.connected_devices.size() : st.discovered.size();
    *configuration = QDevice::Utilities::create_configuration(7, dev_count);

    configuration->properties.data[0] = QDevice::Utilities::create_property(
        "Discovery Timeout Ms", false, qd_property_type_int32,
        std::to_string(st.discovery_timeout_ms), "2500", "");

    configuration->properties.data[1] = QDevice::Utilities::create_property(
        "Buffer Latency Ms", false, qd_property_type_int32,
        std::to_string(st.buffer_latency_ms), "850", "");

    configuration->properties.data[2] = QDevice::Utilities::create_property(
        "Enable ECG", false, qd_property_type_boolean,
        ecg_enabled ? "true" : "false", default_ecg_str, "");

    configuration->properties.data[3] = QDevice::Utilities::create_property(
        "Enable Accelerometer", false, qd_property_type_boolean,
        acc_enabled ? "true" : "false", default_acc_str, "");

    configuration->properties.data[4] = QDevice::Utilities::create_property(
        "Accelerometer Rate Hz", false, qd_property_type_int32,
        std::to_string(acc_rate), "200", "");

    configuration->properties.data[5] = QDevice::Utilities::create_property(
        "Accelerometer Range G", false, qd_property_type_int32,
        std::to_string(acc_range), "8", "");

    configuration->properties.data[6] = QDevice::Utilities::create_property(
        "Device Filter", false, qd_property_type_string_utf8,
        st.device_filter, "", "");

    if (is_connected) {
        for (size_t d = 0; d < st.connected_devices.size(); ++d) {
            populateDeviceConfig(configuration->devices.data[d],
                                 st.connected_devices[d].info, d,
                                 ecg_enabled, acc_enabled, acc_rate);
        }
    } else {
        for (size_t d = 0; d < st.discovered.size(); ++d) {
            populateDeviceConfig(configuration->devices.data[d],
                                 st.discovered[d], d,
                                 ecg_enabled, acc_enabled, acc_rate);
        }
    }

    return okStatus();
}

qd_call_status qd_free_configuration(struct qd_configuration* configuration) {
    if (configuration) {
        QDevice::Utilities::free_configuration(*configuration);
    }
    return okStatus();
}

qd_call_status qd_connect(const struct qd_configuration* config) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_disconnected) {
        return errorStatus("Device already connected");
    }

    if (config) {
        try {
            assignConfigProperties(st, config);
        } catch (const std::exception& ex) {
            return errorStatus(std::string("Invalid configuration: ") + ex.what());
        }
    }

    const bool reused_cached = !st.discovered.empty() && (st.device_filter == st.last_discovery_filter);
    if (!reused_cached) {
        st.discovered = qdevice_polar::PolarBleClient::discoverAll(
            static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
        for (int attempt = 0; attempt < 2 && st.discovered.empty(); ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            st.discovered = qdevice_polar::PolarBleClient::discoverAll(
                static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
        }
        st.last_discovery_filter = st.device_filter;
    }

    if (st.discovered.empty()) {
        return errorStatus("No Polar H10 BLE device found");
    }

    std::string last_client_error;
    auto tryConnectAll = [&](const std::vector<qdevice_polar::DiscoveredPolarDevice>& candidates) {
        st.connected_devices.clear();
        for (const auto& info : candidates) {
            bool connected_ok = false;
            for (int attempt = 0; attempt < 2 && !connected_ok; ++attempt) {
                if (attempt > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(600));
                }
                auto client = std::make_unique<qdevice_polar::PolarBleClient>();
                const size_t ecg_prebuf = std::max<size_t>(
                    30, static_cast<size_t>(std::llround(static_cast<double>(st.buffer_latency_ms) * 130.0 / 1000.0)));
                client->buffer().setEcgPrebufferSamples(ecg_prebuf);
                if (client->connect(info.bluetooth_address,
                                    st.enable_ecg,
                                    st.enable_acc,
                                    static_cast<uint16_t>(st.acc_rate_hz),
                                    static_cast<uint16_t>(st.acc_range_g))) {
                    const size_t acc_prebuf = std::max<size_t>(
                        20, static_cast<size_t>(std::llround(
                            static_cast<double>(st.buffer_latency_ms) * static_cast<double>(client->activeAccRateHz()) / 1000.0)));
                    client->buffer().setAccPrebufferSamples(acc_prebuf);
                    qdevice_polar::PolarBleClient::retainWarmLink(info.bluetooth_address);
                    st.connected_devices.push_back({ info, std::move(client) });
                    connected_ok = true;
                } else if (!client->lastError().empty()) {
                    last_client_error = client->lastError();
                }
            }
        }
    };

    tryConnectAll(st.discovered);

    // If cached Locate Devices entries failed to connect, perform one fresh BLE scan and retry.
    if (st.connected_devices.empty() && reused_cached) {
        st.discovered = qdevice_polar::PolarBleClient::discoverAll(
            static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
        st.last_discovery_filter = st.device_filter;
        if (!st.discovered.empty()) {
            tryConnectAll(st.discovered);
        }
    }

    if (st.connected_devices.empty()) {
        if (!last_client_error.empty()) {
            return errorStatus("Failed to connect to Polar H10 via BLE: " + last_client_error);
        }
        return errorStatus("Failed to connect to any Polar H10 via BLE");
    }

    // Keep discovered list aligned with actually connected devices
    st.discovered.clear();
    for (const auto& dev : st.connected_devices) {
        st.discovered.push_back(dev.info);
    }

    st.active_enable_ecg = st.enable_ecg;
    st.active_enable_acc = st.enable_acc;
    st.active_acc_rate_hz = st.enable_acc
        ? static_cast<std::int32_t>(st.connected_devices.front().client->activeAccRateHz())
        : st.acc_rate_hz;
    st.active_acc_range_g = st.acc_range_g;
    st.state = qd_state_connected;

    for (size_t d = 0; d < st.connected_devices.size(); ++d) {
        const auto& dev = st.connected_devices[d];
        std::ostringstream oss;
        oss << "qd_connect OK: dev=" << deviceSerial(dev.info, d)
            << " name=\"" << dev.info.name << "\""
            << " ecg=" << (st.active_enable_ecg ? "on(130Hz)" : "off")
            << " acc=" << (st.active_enable_acc
                ? ("on(" + std::to_string(st.active_acc_rate_hz) + "Hz/" + std::to_string(st.active_acc_range_g) + "G)")
                : "off")
            << " batt=" << dev.client->buffer().latestBatteryPct() << "%"
            << " mtu=" << dev.client->negotiatedMtu();
        qdevice_polar::logDiagnostic(oss.str());
    }

    return okStatus();
}

qd_call_status qd_disconnect() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state == qd_state_disconnected) {
        return errorStatus("Device not connected");
    }

    qdevice_polar::logDiagnostic("qd_disconnect called (" + std::to_string(st.connected_devices.size()) + " device(s))");
    st.stream_stop_time.reset();
    st.connected_devices.clear();
    st.active_enable_ecg = false;
    st.active_enable_acc = false;
    st.state = qd_state_disconnected;
    return okStatus();
}

qd_call_status qd_start_streaming() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_connected) {
        return errorStatus("Can't start stream, device not connected");
    }

    for (size_t d = 0; d < st.connected_devices.size(); ++d) {
        auto& dev = st.connected_devices[d];
        dev.client->buffer().prepareForStreaming();
        const auto ecg_st = dev.client->buffer().ecgStats();
        const auto acc_st = dev.client->buffer().accStats();
        std::ostringstream oss;
        oss << "qd_start_streaming: dev=" << deviceSerial(dev.info, d)
            << " rates=[HR:1Hz"
            << (st.active_enable_ecg ? ",ECG:130Hz" : "")
            << (st.active_enable_acc ? (",ACC:" + std::to_string(st.active_acc_rate_hz) + "Hz") : "")
            << "]"
            << " ecg_primed=" << (ecg_st.primed ? "yes" : "no")
            << " ecg_q=" << ecg_st.queued_samples << "/" << ecg_st.prebuffer_samples
            << " acc_primed=" << (acc_st.primed ? "yes" : "no")
            << " acc_q=" << acc_st.queued_samples << "/" << acc_st.prebuffer_samples
            << " hr=" << dev.client->buffer().latestBpm()
            << " batt=" << dev.client->buffer().latestBatteryPct() << "%";
        qdevice_polar::logDiagnostic(oss.str());
    }
    st.stream_start_time = clock_t_qd::now();
    st.stream_stop_time.reset();
    st.hr_samples_read = 0;
    st.ecg_samples_read = 0;
    st.acc_samples_read = 0;
    st.state = qd_state_streaming;
    return okStatus();
}

qd_call_status qd_stop_streaming() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_streaming) {
        return errorStatus("Can't stop stream, device not streaming");
    }

    st.stream_stop_time = clock_t_qd::now();
    const double dur_s = std::chrono::duration<double>(*st.stream_stop_time - st.stream_start_time).count();

    for (size_t d = 0; d < st.connected_devices.size(); ++d) {
        const auto& dev = st.connected_devices[d];
        const auto s = dev.client->buffer().ecgStats();
        const auto a = dev.client->buffer().accStats();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "qd_stop_streaming AUDIT: dev=" << deviceSerial(dev.info, d)
            << " dur=" << dur_s << "s"
            << " emitted(hr/ecg/acc)=" << st.hr_samples_read << "/" << st.ecg_samples_read << "/" << st.acc_samples_read
            << " mtu=" << dev.client->negotiatedMtu()
            << " ecg_pkts=" << s.packets_received
            << " ecg_hz=" << s.effective_polar_hz
            << " ecg_underrun=" << s.underrun_samples
            << " ecg_gap_fill=" << s.gap_filled_samples
            << " acc_pkts=" << a.packets_received
            << " acc_hz=" << a.effective_polar_hz
            << " acc_underrun=" << a.underrun_samples
            << " acc_gap_fill=" << a.gap_filled_samples
            << " recoveries=" << dev.client->streamRecoveries();
        qdevice_polar::logDiagnostic(oss.str());
    }

    st.state = qd_state_connected;
    return okStatus();
}

qd_call_status qd_get_state(qd_state* state) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    if (!state) {
        return errorStatus("Null state pointer");
    }
    *state = st.state;
    return okStatus();
}

qd_call_status qd_read_samples(qd_sample_push_fn callback) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (!callback) {
        return errorStatus("Null sample callback");
    }

    clock_t_qd::time_point effective_now{};
    if (st.state == qd_state_streaming) {
        effective_now = clock_t_qd::now();
    } else if (st.state == qd_state_connected && st.stream_stop_time.has_value()) {
        effective_now = *st.stream_stop_time;
    } else {
        return okStatus();
    }

    if (st.connected_devices.empty()) {
        return okStatus();
    }

    const double elapsed_sec = std::max(0.0, std::chrono::duration<double>(effective_now - st.stream_start_time).count());
    const bool final_flush = (st.state != qd_state_streaming && st.stream_stop_time.has_value());

    auto computeChannelCount = [&](double rate_hz, std::uint64_t already_read) -> size_t {
        const auto target = static_cast<std::uint64_t>(std::floor(elapsed_sec * rate_hz));
        size_t cnt = (target > already_read) ? static_cast<size_t>(target - already_read) : 0;
        if (cnt == 0 && final_flush && already_read == 0) {
            cnt = 1;
        }
        return cnt;
    };

    const size_t hr_count = computeChannelCount(static_cast<double>(kHrChannelRateHz), st.hr_samples_read);
    const size_t ecg_count = st.active_enable_ecg
        ? computeChannelCount(static_cast<double>(kEcgChannelRateHz), st.ecg_samples_read)
        : 0;
    const double acc_rate_hz = static_cast<double>(sanitizeAccRateHz(st.active_acc_rate_hz));
    const size_t acc_count = st.active_enable_acc
        ? computeChannelCount(acc_rate_hz, st.acc_samples_read)
        : 0;

    for (size_t d = 0; d < st.connected_devices.size(); ++d) {
        auto& dev = st.connected_devices[d];
        const std::string serial = deviceSerial(dev.info, d);

        // 1. Slow channels at native 1 Hz
        if (hr_count > 0) {
            std::vector<float> bpm_buf(hr_count, dev.client->buffer().latestBpm());
            std::vector<float> rr_buf(hr_count, dev.client->buffer().latestRrMs());
            std::vector<float> cnt_buf(hr_count, dev.client->buffer().latestContact());
            std::vector<float> bat_buf(hr_count, dev.client->buffer().latestBatteryPct());

            callback(serial.c_str(), "Heart Rate",
                     { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(bpm_buf.data()), hr_count });
            callback(serial.c_str(), "RR Interval Ms",
                     { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(rr_buf.data()), hr_count });
            callback(serial.c_str(), "Sensor Contact",
                     { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(cnt_buf.data()), hr_count });
            callback(serial.c_str(), "Battery Pct",
                     { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(bat_buf.data()), hr_count });
        }

        // 2. ECG channel at native 130 Hz (0 resampling)
        if (ecg_count > 0) {
            auto ecg_buf = dev.client->buffer().popEcgSamples(ecg_count);
            callback(serial.c_str(), "ECG",
                     { st.ecg_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(ecg_buf.data()), ecg_count });
        }

        // 3. Accelerometer channels at native 200 Hz (or configured 25/50/100 Hz, 0 resampling)
        if (acc_count > 0) {
            auto acc_buf = dev.client->buffer().popAccSamples(acc_count);
            callback(serial.c_str(), "Acc X",
                     { st.acc_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(acc_buf.x_mg.data()), acc_count });
            callback(serial.c_str(), "Acc Y",
                     { st.acc_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(acc_buf.y_mg.data()), acc_count });
            callback(serial.c_str(), "Acc Z",
                     { st.acc_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(acc_buf.z_mg.data()), acc_count });
        }
    }

    st.hr_samples_read += hr_count;
    st.ecg_samples_read += ecg_count;
    st.acc_samples_read += acc_count;

    if (st.state != qd_state_streaming) {
        st.stream_stop_time.reset();
    }

    return okStatus();
}
