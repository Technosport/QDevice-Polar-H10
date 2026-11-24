#include "qdevice_polar/PolarBleClient.hpp"

#include <qualisys/qdevice-api/qdevice_api.h>
#include <qualisys/qdevice-api/qdevice_utilities.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace {

using clock_t_qd = std::chrono::steady_clock;

constexpr const char* INTEGRATION_NAME    = "Polar H10 Heart Rate Monitor";
constexpr const char* INTEGRATION_DESC    = "BLE GATT integration for Polar H10 (BPM, RR Interval, Contact, optional 130 Hz ECG)";
constexpr const char* INTEGRATION_URL     = "https://www.polar.com/en/sensors/h10-heart-rate-sensor";
constexpr const char* INTEGRATION_VERSION = "0.2.8";

struct ManagedPolarDevice {
    qdevice_polar::DiscoveredPolarDevice info;
    std::unique_ptr<qdevice_polar::PolarBleClient> client;
};

struct PluginState {
    std::mutex mutex;
    qd_state state = qd_state_disconnected;
    std::int32_t discovery_timeout_ms = 2500;
    bool enable_ecg = false;
    std::string device_filter;

    std::vector<qdevice_polar::DiscoveredPolarDevice> discovered;
    std::vector<ManagedPolarDevice> connected_devices;

    clock_t_qd::time_point stream_start_time{};
    std::optional<clock_t_qd::time_point> stream_stop_time{};
    std::uint64_t hr_samples_read = 0;
    std::uint64_t ecg_samples_read = 0;
    std::string last_error;
};

PluginState& getState() {
    static PluginState instance;
    return instance;
}

qd_call_status okStatus() {
    return { qd_call_status_type_ok, nullptr };
}

qd_call_status errorStatus(const std::string& msg) {
    auto& st = getState();
    st.last_error = msg;
    return { qd_call_status_type_error, st.last_error.c_str() };
}

qd_call_status QD_STDCALL actionLocateDevices(const struct qd_configuration* config) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    if (st.state != qd_state_disconnected) {
        return errorStatus("Disconnect before locating devices");
    }
    if (config) {
        try {
            QDevice::Utilities::try_assign_property_value("Discovery Timeout Ms", config->properties, st.discovery_timeout_ms);
            QDevice::Utilities::try_assign_property_value("Enable ECG", config->properties, st.enable_ecg);
            QDevice::Utilities::try_assign_property_value("Device Filter", config->properties, st.device_filter);
        } catch (...) {}
    }
    st.discovered = qdevice_polar::PolarBleClient::discoverAll(
        static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
    return okStatus();
}

} // namespace

qd_call_status qd_get_integration_version(struct qd_static_chars* version) {
    *version = { INTEGRATION_VERSION, std::strlen(INTEGRATION_VERSION) };
    return okStatus();
}

qd_call_status qd_get_name(struct qd_static_chars* name) {
    *name = { INTEGRATION_NAME, std::strlen(INTEGRATION_NAME) };
    return okStatus();
}

qd_call_status qd_get_description(struct qd_static_chars* desc) {
    *desc = { INTEGRATION_DESC, std::strlen(INTEGRATION_DESC) };
    return okStatus();
}

qd_call_status qd_get_manufacturer_url(struct qd_static_chars* url) {
    *url = { INTEGRATION_URL, std::strlen(INTEGRATION_URL) };
    return okStatus();
}

qd_call_status qd_get_contracts(struct qd_contracts* contracts) {
    *contracts = { nullptr, 0 };
    return okStatus();
}

qd_call_status qd_get_category(qd_device_category* category) {
    *category = qd_device_category_generic;
    return okStatus();
}

qd_call_status qd_initialize() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.state = qd_state_disconnected;
    st.hr_samples_read = 0;
    st.ecg_samples_read = 0;
    return okStatus();
}

qd_call_status qd_deinitialize() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    st.connected_devices.clear();
    st.discovered.clear();
    st.state = qd_state_disconnected;
    return okStatus();
}

qd_call_status qd_get_actions(struct qd_actions* actions) {
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

    if (st.state == qd_state_disconnected && st.discovered.empty()) {
        st.discovered = qdevice_polar::PolarBleClient::discoverAll(
            static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
    }

    *configuration = QDevice::Utilities::create_configuration(3, st.discovered.size());

    configuration->properties.data[0] = QDevice::Utilities::create_property(
        "Discovery Timeout Ms", false, qd_property_type_int32,
        std::to_string(st.discovery_timeout_ms), "2500", "");

    configuration->properties.data[1] = QDevice::Utilities::create_property(
        "Enable ECG", false, qd_property_type_boolean,
        st.enable_ecg ? "true" : "false", "false", "");

    configuration->properties.data[2] = QDevice::Utilities::create_property(
        "Device Filter", false, qd_property_type_string_utf8,
        st.device_filter, "", "");

    const size_t ch_count = st.enable_ecg ? 4 : 3;
    const float board_rate_hz = st.enable_ecg ? 130.0f : 1.0f;

    for (size_t d = 0; d < st.discovered.size(); ++d) {
        const auto& dev = st.discovered[d];
        const std::string serial = dev.address_str.empty() ? ("POLAR-H10-" + std::to_string(d + 1)) : dev.address_str;

        configuration->devices.data[d] = QDevice::Utilities::create_device(
            qd_device_category_generic, serial, ch_count, 1);

        configuration->devices.data[d].properties.data[0] = QDevice::Utilities::create_property(
            "Device Name", true, qd_property_type_string_utf8, dev.name, dev.name, "");

        configuration->devices.data[d].channels.data[0] = QDevice::Utilities::create_channel(
            "Heart Rate", qd_sample_encoding_float32, qd_channel_unit_beats_per_minute, board_rate_hz);
        configuration->devices.data[d].channels.data[1] = QDevice::Utilities::create_channel(
            "RR Interval Ms", qd_sample_encoding_float32, qd_channel_unit_none, board_rate_hz);
        configuration->devices.data[d].channels.data[2] = QDevice::Utilities::create_channel(
            "Sensor Contact", qd_sample_encoding_float32, qd_channel_unit_none, board_rate_hz);

        if (st.enable_ecg) {
            configuration->devices.data[d].channels.data[3] = QDevice::Utilities::create_channel(
                "ECG", qd_sample_encoding_float32, qd_channel_unit_microvolts, board_rate_hz);
        }
    }

    return okStatus();
}

qd_call_status qd_free_configuration(struct qd_configuration* configuration) {
    QDevice::Utilities::free_configuration(*configuration);
    return okStatus();
}

qd_call_status qd_connect(const struct qd_configuration* config) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_disconnected) {
        return errorStatus("Device already connected");
    }

    try {
        QDevice::Utilities::try_assign_property_value("Discovery Timeout Ms", config->properties, st.discovery_timeout_ms);
        QDevice::Utilities::try_assign_property_value("Enable ECG", config->properties, st.enable_ecg);
        QDevice::Utilities::try_assign_property_value("Device Filter", config->properties, st.device_filter);
    } catch (const std::exception& ex) {
        return errorStatus(std::string("Invalid configuration: ") + ex.what());
    }

    st.discovered = qdevice_polar::PolarBleClient::discoverAll(
        static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
    for (int attempt = 0; attempt < 2 && st.discovered.empty(); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        st.discovered = qdevice_polar::PolarBleClient::discoverAll(
            static_cast<uint32_t>(st.discovery_timeout_ms), st.device_filter);
    }

    if (st.discovered.empty()) {
        return errorStatus("No Polar H10 BLE device found");
    }

    st.connected_devices.clear();
    for (const auto& info : st.discovered) {
        auto client = std::make_unique<qdevice_polar::PolarBleClient>();
        if (client->connect(info.bluetooth_address, st.enable_ecg)) {
            st.connected_devices.push_back({ info, std::move(client) });
        }
    }

    if (st.connected_devices.empty()) {
        return errorStatus("Failed to connect to any Polar H10 via BLE");
    }

    st.state = qd_state_connected;
    return okStatus();
}

qd_call_status qd_disconnect() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state == qd_state_disconnected) {
        return errorStatus("Device not connected");
    }

    st.stream_stop_time.reset();
    st.connected_devices.clear();
    st.state = qd_state_disconnected;
    return okStatus();
}

qd_call_status qd_start_streaming() {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

    if (st.state != qd_state_connected) {
        return errorStatus("Can't start stream, device not connected");
    }

    for (auto& dev : st.connected_devices) {
        dev.client->buffer().clear();
    }
    st.stream_start_time = clock_t_qd::now();
    st.stream_stop_time.reset();
    st.hr_samples_read = 0;
    st.ecg_samples_read = 0;
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
    st.state = qd_state_connected;
    return okStatus();
}

qd_call_status qd_get_state(qd_state* state) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);
    *state = st.state;
    return okStatus();
}

qd_call_status qd_read_samples(qd_sample_push_fn callback) {
    auto& st = getState();
    std::lock_guard<std::mutex> lock(st.mutex);

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

    const double elapsed_sec = std::chrono::duration<double>(effective_now - st.stream_start_time).count();
    const double board_rate_hz = st.enable_ecg ? 130.0 : 1.0;
    const auto target_samples = static_cast<std::uint64_t>(std::max(0.0, elapsed_sec * board_rate_hz));

    size_t count = (target_samples > st.hr_samples_read)
        ? static_cast<size_t>(target_samples - st.hr_samples_read)
        : 0;
    if (count == 0 && st.state != qd_state_streaming && st.stream_stop_time.has_value() && st.hr_samples_read == 0) {
        count = 1;
    }

    if (count > 0) {
        for (size_t d = 0; d < st.connected_devices.size(); ++d) {
            auto& dev = st.connected_devices[d];
            const std::string serial = dev.info.address_str.empty()
                ? ("POLAR-H10-" + std::to_string(d + 1))
                : dev.info.address_str;

            std::vector<float> bpm_buf(count, dev.client->buffer().latestBpm());
            std::vector<float> rr_buf(count, dev.client->buffer().latestRrMs());
            std::vector<float> cnt_buf(count, dev.client->buffer().latestContact());

            callback(serial.c_str(), "Heart Rate",
                     { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(bpm_buf.data()), count });
            callback(serial.c_str(), "RR Interval Ms",
                     { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(rr_buf.data()), count });
            callback(serial.c_str(), "Sensor Contact",
                     { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(cnt_buf.data()), count });

            if (st.enable_ecg) {
                auto ecg_buf = dev.client->buffer().popEcgSamples(count);
                callback(serial.c_str(), "ECG",
                         { st.hr_samples_read, qd_sample_encoding_float32, reinterpret_cast<uint8_t*>(ecg_buf.data()), count });
            }
        }
        st.hr_samples_read += count;
        st.ecg_samples_read = st.hr_samples_read;
    }

    if (st.state != qd_state_streaming) {
        st.stream_stop_time.reset();
    }

    return okStatus();
}
