#include "qdevice_polar/PolarPacketParser.hpp"

namespace qdevice_polar {

HeartRatePacket parseHeartRateMeasurement(const uint8_t* data, size_t size) {
    HeartRatePacket pkt;
    if (!data || size < 2) {
        return pkt;
    }

    const uint8_t flags = data[0];
    const bool hr_uint16        = (flags & 0x01) != 0;
    const uint8_t contact_bits  = (flags >> 1) & 0x03;
    const bool energy_present   = (flags & 0x08) != 0;
    const bool rr_present       = (flags & 0x10) != 0;

    pkt.contact_supported = (contact_bits == 2 || contact_bits == 3);
    pkt.contact_detected  = (contact_bits == 3);

    size_t offset = 1;
    if (hr_uint16) {
        if (offset + 2 > size) return pkt;
        const uint16_t hr = static_cast<uint16_t>(data[offset]) |
                            (static_cast<uint16_t>(data[offset + 1]) << 8);
        pkt.bpm = static_cast<float>(hr);
        offset += 2;
    } else {
        pkt.bpm = static_cast<float>(data[offset]);
        offset += 1;
    }

    if (energy_present) {
        if (offset + 2 > size) return pkt;
        offset += 2; // Skip Energy Expended (kJ)
    }

    if (rr_present) {
        while (offset + 2 <= size) {
            const uint16_t rr_raw = static_cast<uint16_t>(data[offset]) |
                                    (static_cast<uint16_t>(data[offset + 1]) << 8);
            // BLE SIG unit is 1/1024 seconds -> convert to milliseconds
            const float rr_ms = static_cast<float>(rr_raw) * (1000.0f / 1024.0f);
            pkt.rr_intervals_ms.push_back(rr_ms);
            offset += 2;
        }
    }

    pkt.valid = true;
    return pkt;
}

PmdEcgPacket parsePmdEcgFrame(const uint8_t* data, size_t size) {
    PmdEcgPacket pkt;
    // Polar PMD header: [0]=type (0x00=ECG), [1..8]=timestamp_ns (uint64 LE), [9]=frame_type (0x00=raw)
    if (!data || size < 10 || data[0] != 0x00) {
        return pkt;
    }

    uint64_t ts = 0;
    for (int i = 0; i < 8; ++i) {
        ts |= static_cast<uint64_t>(data[1 + i]) << (8 * i);
    }
    pkt.timestamp_ns = ts;

    // Raw ECG samples: 3 bytes (24-bit signed integer, little-endian) in microvolts
    for (size_t offset = 10; offset + 3 <= size; offset += 3) {
        int32_t raw = static_cast<int32_t>(data[offset]) |
                      (static_cast<int32_t>(data[offset + 1]) << 8) |
                      (static_cast<int32_t>(data[offset + 2]) << 16);
        if (raw & 0x00800000) {
            raw |= static_cast<int32_t>(0xFF000000); // Sign-extend 24-bit -> 32-bit
        }
        pkt.samples_uv.push_back(static_cast<float>(raw));
    }

    pkt.valid = !pkt.samples_uv.empty();
    return pkt;
}

} // namespace qdevice_polar
