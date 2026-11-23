#include "qdevice_polar/PolarSampleBuffer.hpp"

namespace qdevice_polar {

PolarSampleBuffer::PolarSampleBuffer(size_t ecg_prebuffer_samples)
    : ecg_prebuffer_samples_(ecg_prebuffer_samples) {}

void PolarSampleBuffer::pushHeartRate(const HeartRatePacket& pkt) {
    if (!pkt.valid) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_bpm_ = pkt.bpm;
    latest_contact_ = pkt.contact_detected ? 1.0f : 0.0f;
    if (!pkt.rr_intervals_ms.empty()) {
        latest_rr_ms_ = pkt.rr_intervals_ms.back();
    }
}

void PolarSampleBuffer::pushEcg(const PmdEcgPacket& pkt) {
    if (!pkt.valid) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (float s : pkt.samples_uv) {
        ecg_fifo_.push_back(s);
        latest_ecg_uv_ = s;
    }
    if (!ecg_primed_ && ecg_fifo_.size() >= ecg_prebuffer_samples_) {
        ecg_primed_ = true;
    }
    while (ecg_fifo_.size() > 13000) { // Cap ~100s @ 130 Hz
        ecg_fifo_.pop_front();
    }
}

void PolarSampleBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    ecg_primed_ = false;
    latest_bpm_ = 0.0f;
    latest_rr_ms_ = 0.0f;
    latest_contact_ = 0.0f;
    latest_ecg_uv_ = 0.0f;
    ecg_fifo_.clear();
}

float PolarSampleBuffer::latestBpm() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_bpm_;
}

float PolarSampleBuffer::latestRrMs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_rr_ms_;
}

float PolarSampleBuffer::latestContact() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_contact_;
}

std::vector<float> PolarSampleBuffer::popEcgSamples(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<float> out;
    out.reserve(count);

    if (!ecg_primed_) {
        out.assign(count, 0.0f);
        return out;
    }

    for (size_t i = 0; i < count; ++i) {
        if (!ecg_fifo_.empty()) {
            out.push_back(ecg_fifo_.front());
            ecg_fifo_.pop_front();
        } else {
            out.push_back(latest_ecg_uv_);
        }
    }
    return out;
}

} // namespace qdevice_polar
