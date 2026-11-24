#pragma once

#include "qdevice_polar/PolarPacketParser.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace qdevice_polar {

class PolarSampleBuffer {
public:
    explicit PolarSampleBuffer(size_t ecg_prebuffer_samples = 1);

    void pushHeartRate(const HeartRatePacket& pkt);
    void pushEcg(const PmdEcgPacket& pkt);
    void clear();

    float latestBpm() const;
    float latestRrMs() const;
    float latestContact() const;

    /// Pops `count` ECG samples in microvolts. Waits until `ecg_prebuffer_samples_`
    /// have arrived on initial start before draining, then holds latest on underrun.
    std::vector<float> popEcgSamples(size_t count);

private:
    mutable std::mutex mutex_;
    size_t ecg_prebuffer_samples_ = 10;
    bool ecg_primed_ = false;
    float latest_bpm_ = 0.0f;
    float latest_rr_ms_ = 0.0f;
    float latest_contact_ = 0.0f;
    float latest_ecg_uv_ = 0.0f;
    std::deque<float> ecg_fifo_;
};

} // namespace qdevice_polar
