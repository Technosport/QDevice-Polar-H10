#include "qdevice_polar/PolarPacketParser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace qdevice_polar {

namespace {

int32_t readSignedInt24Le(const uint8_t* p) {
    uint32_t raw = static_cast<uint32_t>(p[0]) |
                   (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16);
    if (raw & 0x00800000u) {
        raw |= 0xFF000000u; // Sign-extend 24-bit two's complement -> 32-bit
    }
    return static_cast<int32_t>(raw);
}

size_t pmdSettingTypeFieldSize(uint8_t type_id) {
    // Matches PmdSetting.PmdSettingType.typeToFieldSize in Polar BLE SDK:
    //   0=SAMPLE_RATE(2), 1=RESOLUTION(2), 2=RANGE(2), 3=RANGE_MILLIUNIT(4),
    //   4=CHANNELS(1), 5=FACTOR(4), 6=SECURITY(16), 7=DERIVED_METHOD(1),
    //   8=SOURCE_MEAS_TYPE(1), 9=SOURCE_MEAS_RATE(2), 10=SOURCE_MEAS_RANGE(4),
    //   11=DERIVED_TIME_WINDOW(4), 12=DERIVED_GROUP_ID(1)
    switch (type_id) {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x09:
            return 2;
        case 0x03:
        case 0x05:
        case 0x0A:
        case 0x0B:
            return 4;
        case 0x04:
        case 0x07:
        case 0x08:
        case 0x0C:
            return 1;
        case 0x06:
            return 16;
        default:
            return 0;
    }
}

bool decodeDeltaCompressedEcgType0(const uint8_t* payload, size_t payload_size,
                                   std::vector<float>& out_samples) {
    // Polar PMD compressed TYPE_0 (1 channel, 24-bit signed reference sample):
    //   [0..2]  : 24-bit signed LE reference sample (uV)
    //   repeat until end of payload:
    //     [0]   : delta_size (bits per signed two's-complement delta, 0..32)
    //     [1]   : sample_count (number of deltas in this block)
    //     [2..] : ceil(delta_size * sample_count / 8) bytes of LSB-first packed deltas
    if (payload_size < 3) {
        return false;
    }

    int32_t current = readSignedInt24Le(payload);
    out_samples.push_back(static_cast<float>(current));

    size_t offset = 3;
    while (offset < payload_size) {
        if (offset + 2 > payload_size) {
            out_samples.clear();
            return false;
        }
        const uint8_t delta_size_bits = payload[offset++];
        const uint8_t sample_count    = payload[offset++];

        if (delta_size_bits > 32) {
            out_samples.clear();
            return false;
        }

        if (sample_count == 0) {
            continue;
        }

        if (delta_size_bits == 0) {
            for (uint8_t s = 0; s < sample_count; ++s) {
                out_samples.push_back(static_cast<float>(current));
            }
            continue;
        }

        const size_t total_bits = static_cast<size_t>(sample_count) * static_cast<size_t>(delta_size_bits);
        const size_t byte_len   = (total_bits + 7) / 8;
        if (offset + byte_len > payload_size) {
            out_samples.clear();
            return false;
        }

        const uint8_t* block = payload + offset;
        size_t bit_pos = 0;
        for (uint8_t s = 0; s < sample_count; ++s) {
            uint32_t val = 0;
            for (uint8_t b = 0; b < delta_size_bits; ++b) {
                const size_t idx = bit_pos + b;
                if ((block[idx >> 3] & (1u << (idx & 7u))) != 0) {
                    val |= (1u << b);
                }
            }
            bit_pos += delta_size_bits;

            // Sign-extend from delta_size_bits to 32-bit signed int
            if (delta_size_bits < 32) {
                const uint32_t sign_bit = 1u << (delta_size_bits - 1);
                if (val & sign_bit) {
                    val |= (~0u << delta_size_bits);
                }
            }

            current += static_cast<int32_t>(val);
            out_samples.push_back(static_cast<float>(current));
        }

        offset += byte_len;
    }

    return !out_samples.empty();
}


int32_t readSignedInt16Le(const uint8_t* p) {
    const uint16_t raw = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    return static_cast<int32_t>(static_cast<int16_t>(raw));
}

bool decodeDeltaCompressedAcc3Ch16Bit(const uint8_t* payload, size_t payload_size,
                                      float scale_factor,
                                      std::vector<AccSampleMg>& out_samples) {
    // Polar PMD compressed 3-channel 16-bit signed ACC frame (TYPE_0 0x80 or TYPE_1 0x81):
    //   [0..5]  : 3 x 16-bit signed LE reference sample (x0, y0, z0)
    //   repeat until end of payload:
    //     [0]   : delta_size_bits (0..32)
    //     [1]   : sample_count (number of 3-axis delta triplets in this block)
    //     [2..] : ceil(sample_count * delta_size_bits * 3 / 8) bytes of LSB-first packed deltas
    if (payload_size < 6) {
        return false;
    }

    int64_t cur_x = readSignedInt16Le(payload);
    int64_t cur_y = readSignedInt16Le(payload + 2);
    int64_t cur_z = readSignedInt16Le(payload + 4);
    out_samples.push_back({
        static_cast<float>(cur_x) * scale_factor,
        static_cast<float>(cur_y) * scale_factor,
        static_cast<float>(cur_z) * scale_factor,
    });

    size_t offset = 6;
    while (offset < payload_size) {
        if (offset + 2 > payload_size) {
            out_samples.clear();
            return false;
        }
        const uint8_t delta_size_bits = payload[offset++];
        const uint8_t sample_count    = payload[offset++];

        if (delta_size_bits > 32) {
            out_samples.clear();
            return false;
        }
        if (sample_count == 0) {
            continue;
        }
        if (delta_size_bits == 0) {
            for (uint8_t s = 0; s < sample_count; ++s) {
                out_samples.push_back({
                    static_cast<float>(cur_x) * scale_factor,
                    static_cast<float>(cur_y) * scale_factor,
                    static_cast<float>(cur_z) * scale_factor,
                });
            }
            continue;
        }

        const size_t total_bits = static_cast<size_t>(sample_count) * static_cast<size_t>(delta_size_bits) * 3u;
        const size_t byte_len   = (total_bits + 7u) / 8u;
        if (offset + byte_len > payload_size) {
            out_samples.clear();
            return false;
        }

        const uint8_t* block = payload + offset;
        size_t bit_pos = 0;
        auto readSignedBits = [&](uint8_t width) -> int32_t {
            uint32_t val = 0;
            for (uint8_t b = 0; b < width; ++b) {
                const size_t idx = bit_pos + b;
                if ((block[idx >> 3] & (1u << (idx & 7u))) != 0) {
                    val |= (1u << b);
                }
            }
            bit_pos += width;
            if (width < 32) {
                const uint32_t sign_bit = 1u << (width - 1);
                if (val & sign_bit) {
                    val |= (~0u << width);
                }
            }
            return static_cast<int32_t>(val);
        };

        for (uint8_t s = 0; s < sample_count; ++s) {
            cur_x = std::clamp<int64_t>(cur_x + readSignedBits(delta_size_bits),
                                        std::numeric_limits<int32_t>::min(),
                                        std::numeric_limits<int32_t>::max());
            cur_y = std::clamp<int64_t>(cur_y + readSignedBits(delta_size_bits),
                                        std::numeric_limits<int32_t>::min(),
                                        std::numeric_limits<int32_t>::max());
            cur_z = std::clamp<int64_t>(cur_z + readSignedBits(delta_size_bits),
                                        std::numeric_limits<int32_t>::min(),
                                        std::numeric_limits<int32_t>::max());
            out_samples.push_back({
                static_cast<float>(cur_x) * scale_factor,
                static_cast<float>(cur_y) * scale_factor,
                static_cast<float>(cur_z) * scale_factor,
            });
        }
        offset += byte_len;
    }

    return !out_samples.empty();
}

} // namespace

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


float parseBatteryLevelPercent(const uint8_t* data, size_t size) {
    if (!data || size < 1) {
        return -1.0f;
    }
    return static_cast<float>(std::min<uint8_t>(data[0], 100u));
}

PmdAccPacket parsePmdAccFrame(const uint8_t* data, size_t size, float factor) {
    PmdAccPacket pkt;
    // Polar PMD ACC header:
    //   [0]    = measurement type (0x02 = ACC)
    //   [1..8] = timestamp_ns (uint64 LE, timestamp of last sample in frame)
    //   [9]    = frame_type (bit 7 = compressed flag, bits 6..0 = sub-type)
    if (!data || size < 10 || data[0] != 0x02) {
        return pkt;
    }
    if (!std::isfinite(factor) || factor <= 0.0f) {
        factor = 1.0f;
    }

    uint64_t ts = 0;
    for (int i = 0; i < 8; ++i) {
        ts |= static_cast<uint64_t>(data[1 + i]) << (8 * i);
    }
    pkt.timestamp_ns  = ts;
    pkt.frame_type    = data[9];
    pkt.is_compressed = (pkt.frame_type & 0x80u) != 0;

    const uint8_t sub_type = pkt.frame_type & 0x7Fu;
    const uint8_t* payload = data + 10;
    const size_t payload_size = size - 10;

    if (!pkt.is_compressed) {
        if (sub_type == 0x00) {
            // Raw TYPE_0: 3 x 1-byte signed int8 (mG)
            if (payload_size == 0 || (payload_size % 3) != 0) return pkt;
            pkt.samples_mg.reserve(payload_size / 3);
            for (size_t off = 0; off + 3 <= payload_size; off += 3) {
                pkt.samples_mg.push_back({
                    static_cast<float>(static_cast<int8_t>(payload[off])) * factor,
                    static_cast<float>(static_cast<int8_t>(payload[off + 1])) * factor,
                    static_cast<float>(static_cast<int8_t>(payload[off + 2])) * factor,
                });
            }
        } else if (sub_type == 0x01) {
            // Raw TYPE_1 (Polar H10 uncompressed 16-bit): 3 x 2-byte signed int16 LE (mG)
            if (payload_size == 0 || (payload_size % 6) != 0) return pkt;
            pkt.samples_mg.reserve(payload_size / 6);
            for (size_t off = 0; off + 6 <= payload_size; off += 6) {
                pkt.samples_mg.push_back({
                    static_cast<float>(readSignedInt16Le(payload + off)) * factor,
                    static_cast<float>(readSignedInt16Le(payload + off + 2)) * factor,
                    static_cast<float>(readSignedInt16Le(payload + off + 4)) * factor,
                });
            }
        } else if (sub_type == 0x02) {
            // Raw TYPE_2: 3 x 3-byte signed int24 LE (mG)
            if (payload_size == 0 || (payload_size % 9) != 0) return pkt;
            pkt.samples_mg.reserve(payload_size / 9);
            for (size_t off = 0; off + 9 <= payload_size; off += 9) {
                pkt.samples_mg.push_back({
                    static_cast<float>(readSignedInt24Le(payload + off)) * factor,
                    static_cast<float>(readSignedInt24Le(payload + off + 3)) * factor,
                    static_cast<float>(readSignedInt24Le(payload + off + 6)) * factor,
                });
            }
        } else {
            return pkt;
        }
        pkt.valid = !pkt.samples_mg.empty();
        return pkt;
    }

    // Compressed delta frame:
    //   TYPE_0 (0x80): arrives in G units -> scaled by factor * 1000.0f to mG
    //   TYPE_1 (0x81, Polar H10 default): arrives in mG -> scaled by factor
    if (sub_type == 0x00) {
        pkt.valid = decodeDeltaCompressedAcc3Ch16Bit(payload, payload_size, factor * 1000.0f, pkt.samples_mg);
    } else if (sub_type == 0x01) {
        pkt.valid = decodeDeltaCompressedAcc3Ch16Bit(payload, payload_size, factor, pkt.samples_mg);
    }
    return pkt;
}

PmdEcgPacket parsePmdEcgFrame(const uint8_t* data, size_t size) {
    PmdEcgPacket pkt;
    // Polar PMD header:
    //   [0]    = measurement type (0x00 = ECG)
    //   [1..8] = timestamp_ns (uint64 LE, timestamp of last sample in frame)
    //   [9]    = frame_type (bit 7 = compressed flag, bits 6..0 = sub-type)
    if (!data || size < 10 || data[0] != 0x00) {
        return pkt;
    }

    uint64_t ts = 0;
    for (int i = 0; i < 8; ++i) {
        ts |= static_cast<uint64_t>(data[1 + i]) << (8 * i);
    }
    pkt.timestamp_ns  = ts;
    pkt.frame_type    = data[9];
    pkt.is_compressed = (pkt.frame_type & 0x80u) != 0;

    const uint8_t sub_type = pkt.frame_type & 0x7Fu;
    const uint8_t* payload = data + 10;
    const size_t payload_size = size - 10;

    if (!pkt.is_compressed) {
        // Polar H10 raw ECG is TYPE_0 (frame_type == 0x00): 3-byte signed LE microvolts.
        if (sub_type != 0x00 || payload_size == 0 || (payload_size % 3) != 0) {
            return pkt;
        }
        pkt.samples_uv.reserve(payload_size / 3);
        for (size_t offset = 0; offset + 3 <= payload_size; offset += 3) {
            pkt.samples_uv.push_back(static_cast<float>(readSignedInt24Le(payload + offset)));
        }
        pkt.valid = !pkt.samples_uv.empty();
        return pkt;
    }

    // Compressed delta frame: only TYPE_0 (frame_type == 0x80) is decoded for 1-channel ECG.
    if (sub_type != 0x00) {
        return pkt;
    }
    pkt.valid = decodeDeltaCompressedEcgType0(payload, payload_size, pkt.samples_uv);
    return pkt;
}

PmdControlPointResponse parsePmdControlPointResponse(const uint8_t* data, size_t size) {
    PmdControlPointResponse resp;
    // Polar PMD CP Response:
    //   [0] = 0xF0 (response code)
    //   [1] = op_code (0x01=GetSettings, 0x02=Start, 0x03=Stop, 0x05=GetStatus)
    //   [2] = measurement_type (0x00=ECG)
    //   [3] = error_code (0x00=Success, 0x06=AlreadyInState, 0x0A=InvalidMtu, ...)
    //   [4] = more_frames (optional; present on Success when size > 4)
    //   [5..] = response parameters
    if (!data || size < 4 || data[0] != 0xF0) {
        return resp;
    }

    resp.op_code          = data[1];
    resp.measurement_type = data[2];

    const uint8_t raw_err = data[3];
    if (raw_err <= static_cast<uint8_t>(PmdControlPointError::InvalidDerivedMeasurementMethod)) {
        resp.error_code = static_cast<PmdControlPointError>(raw_err);
    } else {
        resp.error_code = PmdControlPointError::Unknown;
    }

    if (resp.error_code == PmdControlPointError::Success) {
        resp.more_frames = (size > 4 && data[4] != 0);
        if (size > 5) {
            resp.parameters.assign(data + 5, data + size);
        }
    }

    resp.valid = true;
    return resp;
}

bool isPmdServerStopNotification(const uint8_t* data, size_t size, uint8_t measurement_type) {
    if (!data || size < 2 || data[0] != 0x01) {
        return false;
    }
    for (size_t i = 1; i < size; ++i) {
        if (data[i] == measurement_type) {
            return true;
        }
    }
    return false;
}

PmdFeatures parsePmdFeatures(const uint8_t* data, size_t size) {
    PmdFeatures f;
    // Polar PMD Feature Read response: [0]=0x0F, [1]=bitmask (bit0=ECG, bit1=PPG, bit2=ACC, bit3=PPI)
    if (!data || size < 2 || data[0] != 0x0F) {
        return f;
    }
    const uint8_t mask = data[1];
    f.ecg_supported = (mask & 0x01u) != 0;
    f.ppg_supported = (mask & 0x02u) != 0;
    f.acc_supported = (mask & 0x04u) != 0;
    f.ppi_supported = (mask & 0x08u) != 0;
    f.valid = true;
    return f;
}

PmdEcgSettings parsePmdEcgSettings(const uint8_t* params, size_t size) {
    PmdEcgSettings settings;
    if (!params || size <= 1) {
        return settings;
    }

    size_t offset = 0;
    while (offset + 2 <= size) {
        const uint8_t type  = params[offset++];
        const uint8_t count = params[offset++];

        const size_t elem_size = pmdSettingTypeFieldSize(type);
        if (elem_size == 0) {
            return {};
        }

        const size_t total_bytes = static_cast<size_t>(count) * elem_size;
        if (offset + total_bytes > size) {
            return {};
        }

        if (type == 0x00 || type == 0x01) {
            auto& target = (type == 0x00) ? settings.sample_rates_hz : settings.resolutions_bits;
            for (uint8_t i = 0; i < count; ++i) {
                const uint16_t val = static_cast<uint16_t>(params[offset]) |
                                     (static_cast<uint16_t>(params[offset + 1]) << 8);
                target.push_back(val);
                offset += 2;
            }
        } else {
            offset += total_bytes;
        }
    }

    if (offset != size) {
        return {};
    }

    settings.valid = !settings.sample_rates_hz.empty() || !settings.resolutions_bits.empty();
    return settings;
}

std::vector<uint8_t> buildPmdStartEcgCommand(uint16_t sample_rate_hz, uint16_t resolution_bits) {
    return {
        0x02, // OpCode: REQUEST_MEASUREMENT_START
        0x00, // Measurement Type: ECG
        0x00, // Setting Type: SAMPLE_RATE
        0x01, // Array length: 1
        static_cast<uint8_t>(sample_rate_hz & 0xFFu),
        static_cast<uint8_t>((sample_rate_hz >> 8) & 0xFFu),
        0x01, // Setting Type: RESOLUTION
        0x01, // Array length: 1
        static_cast<uint8_t>(resolution_bits & 0xFFu),
        static_cast<uint8_t>((resolution_bits >> 8) & 0xFFu),
    };
}


PmdAccSettings parsePmdAccSettings(const uint8_t* params, size_t size) {
    PmdAccSettings settings;
    if (!params || size <= 1) {
        return settings;
    }

    size_t offset = 0;
    while (offset + 2 <= size) {
        const uint8_t type  = params[offset++];
        const uint8_t count = params[offset++];

        const size_t elem_size = pmdSettingTypeFieldSize(type);
        if (elem_size == 0) {
            return {};
        }

        const size_t total_bytes = static_cast<size_t>(count) * elem_size;
        if (offset + total_bytes > size) {
            return {};
        }

        if (type == 0x00 || type == 0x01 || type == 0x02) {
            auto& target = (type == 0x00) ? settings.sample_rates_hz
                         : (type == 0x01) ? settings.resolutions_bits
                                          : settings.ranges_g;
            for (uint8_t i = 0; i < count; ++i) {
                const uint16_t val = static_cast<uint16_t>(params[offset]) |
                                     (static_cast<uint16_t>(params[offset + 1]) << 8);
                target.push_back(val);
                offset += 2;
            }
        } else if (type == 0x04) {
            for (uint8_t i = 0; i < count; ++i) {
                settings.channels.push_back(params[offset++]);
            }
        } else {
            offset += total_bytes;
        }
    }

    if (offset != size) {
        return {};
    }

    settings.valid = !settings.sample_rates_hz.empty() ||
                     !settings.resolutions_bits.empty() ||
                     !settings.ranges_g.empty();
    return settings;
}

float parsePmdStartFactor(const uint8_t* params, size_t size) {
    if (!params || size < 6) {
        return 1.0f;
    }
    size_t offset = 0;
    while (offset + 2 <= size) {
        const uint8_t type  = params[offset++];
        const uint8_t count = params[offset++];
        const size_t elem_size = pmdSettingTypeFieldSize(type);
        if (elem_size == 0) break;
        const size_t total_bytes = static_cast<size_t>(count) * elem_size;
        if (offset + total_bytes > size) break;
        if (type == 0x05 && count >= 1 && elem_size == 4) {
            uint32_t bits = static_cast<uint32_t>(params[offset]) |
                            (static_cast<uint32_t>(params[offset + 1]) << 8) |
                            (static_cast<uint32_t>(params[offset + 2]) << 16) |
                            (static_cast<uint32_t>(params[offset + 3]) << 24);
            float f = 1.0f;
            std::memcpy(&f, &bits, sizeof(float));
            return (std::isfinite(f) && f > 0.0f) ? f : 1.0f;
        }
        offset += total_bytes;
    }
    return 1.0f;
}

std::vector<uint8_t> buildPmdStartAccCommand(uint16_t sample_rate_hz,
                                             uint16_t resolution_bits,
                                             uint16_t range_g,
                                             bool include_channels,
                                             uint8_t channels) {
    std::vector<uint8_t> cmd = {
        0x02, // OpCode: REQUEST_MEASUREMENT_START
        0x02, // Measurement Type: ACC
        0x00, // Setting Type: SAMPLE_RATE
        0x01,
        static_cast<uint8_t>(sample_rate_hz & 0xFFu),
        static_cast<uint8_t>((sample_rate_hz >> 8) & 0xFFu),
        0x01, // Setting Type: RESOLUTION
        0x01,
        static_cast<uint8_t>(resolution_bits & 0xFFu),
        static_cast<uint8_t>((resolution_bits >> 8) & 0xFFu),
        0x02, // Setting Type: RANGE (G)
        0x01,
        static_cast<uint8_t>(range_g & 0xFFu),
        static_cast<uint8_t>((range_g >> 8) & 0xFFu),
    };
    if (include_channels) {
        cmd.push_back(0x04); // Setting Type: CHANNELS
        cmd.push_back(0x01);
        cmd.push_back(channels);
    }
    return cmd;
}

const char* pmdErrorToString(PmdControlPointError err) {
    switch (err) {
        case PmdControlPointError::Success:                                return "SUCCESS";
        case PmdControlPointError::InvalidOpCode:                          return "INVALID_OP_CODE";
        case PmdControlPointError::InvalidMeasurementType:                 return "INVALID_MEASUREMENT_TYPE";
        case PmdControlPointError::NotSupported:                           return "NOT_SUPPORTED";
        case PmdControlPointError::InvalidLength:                          return "INVALID_LENGTH";
        case PmdControlPointError::InvalidParameter:                       return "INVALID_PARAMETER";
        case PmdControlPointError::AlreadyInState:                         return "ALREADY_IN_STATE";
        case PmdControlPointError::InvalidResolution:                      return "INVALID_RESOLUTION";
        case PmdControlPointError::InvalidSampleRate:                      return "INVALID_SAMPLE_RATE";
        case PmdControlPointError::InvalidRange:                           return "INVALID_RANGE";
        case PmdControlPointError::InvalidMtu:                             return "INVALID_MTU";
        case PmdControlPointError::InvalidNumberOfChannels:                return "INVALID_NUMBER_OF_CHANNELS";
        case PmdControlPointError::InvalidState:                           return "INVALID_STATE";
        case PmdControlPointError::DeviceInCharger:                        return "DEVICE_IN_CHARGER";
        case PmdControlPointError::DiskFull:                               return "DISK_FULL";
        case PmdControlPointError::InvalidSourceMeasurementType:           return "INVALID_SOURCE_MEASUREMENT_TYPE";
        case PmdControlPointError::InvalidSourceMeasurementRate:           return "INVALID_SOURCE_MEASUREMENT_RATE";
        case PmdControlPointError::InvalidDerivedMeasurementSettingsGroup: return "INVALID_DERIVED_MEASUREMENT_SETTINGS_GROUP";
        case PmdControlPointError::InvalidDerivedMeasurementMethod:        return "INVALID_DERIVED_MEASUREMENT_METHOD";
        default:                                                           return "UNKNOWN";
    }
}

} // namespace qdevice_polar
