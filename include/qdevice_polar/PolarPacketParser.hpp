#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace qdevice_polar {

struct HeartRatePacket {
    bool valid = false;
    float bpm = 0.0f;
    bool contact_supported = false;
    bool contact_detected = false;
    std::vector<float> rr_intervals_ms;
};

struct PmdEcgPacket {
    bool valid = false;
    uint64_t timestamp_ns = 0;
    uint8_t frame_type = 0;
    bool is_compressed = false;
    std::vector<float> samples_uv;
};

struct AccSampleMg {
    float x_mg = 0.0f;
    float y_mg = 0.0f;
    float z_mg = 0.0f;
};

struct PmdAccPacket {
    bool valid = false;
    uint64_t timestamp_ns = 0;
    uint8_t frame_type = 0;
    bool is_compressed = false;
    std::vector<AccSampleMg> samples_mg;
};

/// Polar PMD Control Point response error codes (Polar Measurement Data Specification / Polar BLE SDK).
enum class PmdControlPointError : uint8_t {
    Success                                = 0x00,
    InvalidOpCode                          = 0x01,
    InvalidMeasurementType                 = 0x02,
    NotSupported                           = 0x03,
    InvalidLength                          = 0x04,
    InvalidParameter                       = 0x05,
    AlreadyInState                         = 0x06,
    InvalidResolution                      = 0x07,
    InvalidSampleRate                      = 0x08,
    InvalidRange                           = 0x09,
    InvalidMtu                             = 0x0A,
    InvalidNumberOfChannels                = 0x0B,
    InvalidState                           = 0x0C,
    DeviceInCharger                        = 0x0D,
    DiskFull                               = 0x0E,
    InvalidSourceMeasurementType           = 0x0F,
    InvalidSourceMeasurementRate           = 0x10,
    InvalidDerivedMeasurementSettingsGroup = 0x11,
    InvalidDerivedMeasurementMethod        = 0x12,
    Unknown                                = 0xFF,
};

struct PmdControlPointResponse {
    bool valid = false;
    uint8_t op_code = 0;
    uint8_t measurement_type = 0;
    PmdControlPointError error_code = PmdControlPointError::Unknown;
    bool more_frames = false;
    std::vector<uint8_t> parameters;
};

struct PmdFeatures {
    bool valid = false;
    bool ecg_supported = false;
    bool ppg_supported = false;
    bool acc_supported = false;
    bool ppi_supported = false;
};

struct PmdEcgSettings {
    bool valid = false;
    std::vector<uint16_t> sample_rates_hz;
    std::vector<uint16_t> resolutions_bits;
};

struct PmdAccSettings {
    bool valid = false;
    std::vector<uint16_t> sample_rates_hz;
    std::vector<uint16_t> resolutions_bits;
    std::vector<uint16_t> ranges_g;
    std::vector<uint8_t> channels;
};

/// Parses a standard BLE GATT Heart Rate Measurement (UUID 0x2A37) payload.
HeartRatePacket parseHeartRateMeasurement(const uint8_t* data, size_t size);

/// Parses a standard BLE GATT Battery Level (UUID 0x2A19) payload (0..100%).
/// Returns -1.0f if payload is empty or invalid.
float parseBatteryLevelPercent(const uint8_t* data, size_t size);

/// Parses a Polar H10 PMD Data (FB005C82-02E7-F387-1CAD-8ACD2D8DF0C8) ECG frame (type 0x00).
/// Supports raw TYPE_0 (frame_type 0x00) and compressed delta TYPE_0 (frame_type 0x80).
PmdEcgPacket parsePmdEcgFrame(const uint8_t* data, size_t size);

/// Parses a Polar H10 PMD Data (FB005C82-02E7-F387-1CAD-8ACD2D8DF0C8) Accelerometer frame (type 0x02).
/// Supports raw TYPE_0 (8-bitx3), TYPE_1 (16-bitx3), TYPE_2 (24-bitx3) and compressed delta
/// TYPE_0 (0x80) & TYPE_1 (0x81, 3 channels x 16-bit signed reference + LSB packed deltas) in mG.
PmdAccPacket parsePmdAccFrame(const uint8_t* data, size_t size, float factor = 1.0f);

/// Parses a Polar PMD Control Point indication (0xF0 response frame).
PmdControlPointResponse parsePmdControlPointResponse(const uint8_t* data, size_t size);

/// Returns true if a PMD Control Point indication is an unsolicited device-initiated
/// ONLINE_MEASUREMENT_STOPPED (0x01) command for the specified measurement type.
bool isPmdServerStopNotification(const uint8_t* data, size_t size, uint8_t measurement_type);

/// Parses the Polar PMD Control Point feature read payload (starts with 0x0F).
PmdFeatures parsePmdFeatures(const uint8_t* data, size_t size);

/// Parses GET_MEASUREMENT_SETTINGS (0x01) response parameter bytes for ECG (0x00).
PmdEcgSettings parsePmdEcgSettings(const uint8_t* params, size_t size);

/// Parses GET_MEASUREMENT_SETTINGS (0x01) response parameter bytes for ACC (0x02).
PmdAccSettings parsePmdAccSettings(const uint8_t* params, size_t size);

/// Extracts optional IEEE-754 float FACTOR (Setting Type 0x05) from a PMD Start response, or 1.0f.
float parsePmdStartFactor(const uint8_t* params, size_t size);

/// Builds a Polar PMD REQUEST_MEASUREMENT_START (0x02) command for ECG (0x00).
std::vector<uint8_t> buildPmdStartEcgCommand(uint16_t sample_rate_hz = 130,
                                             uint16_t resolution_bits = 14);

/// Builds a Polar PMD REQUEST_MEASUREMENT_START (0x02) command for ACC (0x02).
/// Polar H10 supports sample_rate_hz in {25, 50, 100, 200}, resolution_bits = 16, range_g in {2, 4, 8}.
/// Only includes CHANNELS (0x04 = 3) if include_channels == true (advertised by GetSettings).
std::vector<uint8_t> buildPmdStartAccCommand(uint16_t sample_rate_hz = 200,
                                             uint16_t resolution_bits = 16,
                                             uint16_t range_g = 8,
                                             bool include_channels = false,
                                             uint8_t channels = 3);

/// Human-readable name for a Polar PMD Control Point error code.
const char* pmdErrorToString(PmdControlPointError err);

} // namespace qdevice_polar
