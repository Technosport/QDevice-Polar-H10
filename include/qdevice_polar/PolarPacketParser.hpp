#pragma once

#include <cstddef>
#include <cstdint>
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
    std::vector<float> samples_uv;
};

/// Parses a standard BLE GATT Heart Rate Measurement (UUID 0x2A37) payload.
HeartRatePacket parseHeartRateMeasurement(const uint8_t* data, size_t size);

/// Parses a Polar H10 PMD Data (FB005C82-02E7-F387-1CAD-8ACD2D8DF0C8) ECG frame (type 0x00).
PmdEcgPacket parsePmdEcgFrame(const uint8_t* data, size_t size);

} // namespace qdevice_polar
