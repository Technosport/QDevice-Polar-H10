#include "qdevice_polar/PolarSampleBuffer.hpp"

#include <algorithm>
#include <cmath>

namespace qdevice_polar {

namespace {

constexpr double kPolarEcgNominalRateHz = 130.0;
constexpr double kNsPerSecond = 1e9;
// >= 10 missing samples (~77 ms) before reconstructing a gap: well above PMD timestamp
// rounding jitter (< 1 sample) and below one lost 73-sample H10 notification.
constexpr size_t kMinGapFillSamples = 10;

float sanitizeFloat(float v, float fallback = 0.0f) {
    return std::isfinite(v) ? v : fallback;
}

} // namespace

PolarSampleBuffer::PolarSampleBuffer(size_t ecg_prebuffer_samples)
    : ecg_prebuffer_samples_(std::max<size_t>(1, ecg_prebuffer_samples)) {
    updateWatermarksLocked();
    updateAccWatermarksLocked();
}

void PolarSampleBuffer::updateAccWatermarksLocked() {
    // Polar H10 MEMS accelerometer runs off an internal oscillator (~204.5 Hz for 200 Hz nominal,
    // i.e. +2.3% fast). Allow up to 10% soft drift compensation (1 blend per 10 samples at 200 Hz)
    // and keep high_watermark tight (~prebuffer + 50 samples = +250 ms) so ACC latency never grows
    // during long captures.
    acc_max_fifo_samples_ = std::max<size_t>(acc_prebuffer_samples_ * 3, 400);
    acc_high_watermark_   = acc_prebuffer_samples_ + std::max<size_t>(30, acc_nominal_rate_hz_ / 4);
    acc_low_watermark_    = (acc_prebuffer_samples_ >= 40) ? 16 : 0;
    acc_drift_adjust_interval_ = std::max<size_t>(4, acc_nominal_rate_hz_ / 20);
}

void PolarSampleBuffer::setAccPrebufferSamples(size_t prebuffer_samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    acc_prebuffer_samples_ = std::max<size_t>(1, prebuffer_samples);
    updateAccWatermarksLocked();
}

void PolarSampleBuffer::setAccNominalRateHz(uint16_t rate_hz) {
    std::lock_guard<std::mutex> lock(mutex_);
    acc_nominal_rate_hz_ = (rate_hz > 0) ? rate_hz : 200;
    // Scale default prebuffer (~850 ms) with the configured ACC sampling rate
    acc_prebuffer_samples_ = std::max<size_t>(20, (static_cast<size_t>(acc_nominal_rate_hz_) * 17) / 20);
    updateAccWatermarksLocked();
}

void PolarSampleBuffer::updateWatermarksLocked() {
    // Keep high_watermark tight (~prebuffer + 90 samples ≈ +0.69 s) so live latency never creeps
    // up during long previews while leaving enough headroom for a 73-sample burst + BLE jitter.
    max_fifo_samples_ = std::max<size_t>(ecg_prebuffer_samples_ * 3, 390);
    high_watermark_   = ecg_prebuffer_samples_ + 90;
    low_watermark_    = (ecg_prebuffer_samples_ >= 40) ? 12 : 0;
}

void PolarSampleBuffer::setEcgPrebufferSamples(size_t prebuffer_samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    ecg_prebuffer_samples_ = std::max<size_t>(1, prebuffer_samples);
    updateWatermarksLocked();
}

void PolarSampleBuffer::setPacketTraceCallback(EcgPacketTraceCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    packet_trace_cb_ = std::move(cb);
}

void PolarSampleBuffer::pushBatteryLevel(float battery_pct) {
    if (!std::isfinite(battery_pct) || battery_pct < 0.0f) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_battery_pct_ = std::clamp(battery_pct, 0.0f, 100.0f);
}

void PolarSampleBuffer::pushHeartRate(const HeartRatePacket& pkt) {
    if (!pkt.valid) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_bpm_ = sanitizeFloat(pkt.bpm, latest_bpm_);
    latest_contact_ = pkt.contact_detected ? 1.0f : 0.0f;
    if (!pkt.rr_intervals_ms.empty()) {
        latest_rr_ms_ = sanitizeFloat(pkt.rr_intervals_ms.back(), latest_rr_ms_);
    }
}

void PolarSampleBuffer::pushEcg(const PmdEcgPacket& pkt) {
    if (!pkt.valid || pkt.samples_uv.empty()) return;

    EcgPacketTrace trace{};
    EcgPacketTraceCallback cb_copy;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = clock_t::now();

        double dt_pc_ms = 0.0;
        double dt_polar_ms = 0.0;

        if (has_received_ecg_) {
            dt_pc_ms = std::chrono::duration<double, std::milli>(now - last_ecg_arrival_time_).count();
            if (burst_interval_count_ == 0 || dt_pc_ms < burst_interval_min_ms_) {
                burst_interval_min_ms_ = dt_pc_ms;
            }
            if (burst_interval_count_ == 0 || dt_pc_ms > burst_interval_max_ms_) {
                burst_interval_max_ms_ = dt_pc_ms;
            }
            burst_interval_sum_ms_ += dt_pc_ms;
            ++burst_interval_count_;
        } else {
            first_ecg_arrival_time_ = now;
        }

        size_t gap_fill = 0;
        if (pkt.timestamp_ns != 0) {
            if (last_ecg_timestamp_ns_ != 0 && pkt.timestamp_ns > last_ecg_timestamp_ns_) {
                const uint64_t delta_ns = pkt.timestamp_ns - last_ecg_timestamp_ns_;
                dt_polar_ms = static_cast<double>(delta_ns) / 1e6;
                const double expected_ns =
                    (static_cast<double>(pkt.samples_uv.size()) / kPolarEcgNominalRateHz) * kNsPerSecond;
                const bool plausible = static_cast<double>(delta_ns) < 10.0 * kNsPerSecond;
                // Flag over-the-air packet gaps (> 1.5x expected frame interval, < 10 s restart jump)
                if (static_cast<double>(delta_ns) > expected_ns * 1.5 && plausible) {
                    ++timestamp_gap_packets_;
                }
                // Polar PMD timestamps mark the LAST sample of each frame, so the number of
                // samples elapsed since the previous frame is round(delta * fs). Anything beyond
                // this frame's own sample count was lost over the air (BLE notifications are
                // unacknowledged). Re-insert them (linear interpolation) so the ECG timeline is
                // not compressed by ~561 ms per lost packet relative to QTM / mocap time.
                if (plausible && has_received_ecg_) {
                    const auto total = static_cast<int64_t>(std::llround(
                        static_cast<double>(delta_ns) * kPolarEcgNominalRateHz / kNsPerSecond));
                    const int64_t missing = total - static_cast<int64_t>(pkt.samples_uv.size());
                    if (missing >= static_cast<int64_t>(kMinGapFillSamples)) {
                        gap_fill = std::min<size_t>(static_cast<size_t>(missing), kMaxGapFillSamples);
                    }
                }
            }

            if (first_ecg_timestamp_ns_ == 0 || pkt.timestamp_ns <= last_ecg_timestamp_ns_) {
                // Initial packet or device timestamp counter reset across PMD restart
                first_ecg_timestamp_ns_ = pkt.timestamp_ns;
                polar_rate_sample_count_ = 0;
            } else {
                polar_rate_sample_count_ += pkt.samples_uv.size() + gap_fill;
            }
            last_ecg_timestamp_ns_ = pkt.timestamp_ns;
        }
        last_ecg_arrival_time_ = now;

        has_received_ecg_ = true;
        ++packets_received_;
        last_frame_type_ = pkt.frame_type;
        last_packet_samples_ = pkt.samples_uv.size();

        if (gap_fill > 0) {
            // Hold samples already emitted while the FIFO was empty during this dropout have
            // occupied part of the missing timeline; only reconstruct the remainder so the
            // dropout does not add permanent latency on top of the timeline it already used.
            const size_t absorbed = std::min(gap_fill, underrun_debt_samples_);
            gap_fill -= absorbed;
        }
        underrun_debt_samples_ = 0;

        if (gap_fill > 0) {
            const float a = last_pushed_ecg_uv_;
            const float b = sanitizeFloat(pkt.samples_uv.front(), a);
            for (size_t k = 0; k < gap_fill; ++k) {
                const float t = static_cast<float>(k + 1) / static_cast<float>(gap_fill + 1);
                ecg_fifo_.push_back(a + (b - a) * t);
            }
            gap_filled_samples_ += gap_fill;
            samples_pushed_ += gap_fill;
        }

        for (float s : pkt.samples_uv) {
            const float clean = sanitizeFloat(s, last_pushed_ecg_uv_);
            ecg_fifo_.push_back(clean);
            last_pushed_ecg_uv_ = clean;
            ++samples_pushed_;
        }

        if (!ecg_primed_ && ecg_fifo_.size() >= ecg_prebuffer_samples_) {
            // Trim any pre-priming excess so initial playback latency always starts anchored
            // at `ecg_prebuffer_samples_` instead of carrying extra accumulated samples.
            while (ecg_fifo_.size() > ecg_prebuffer_samples_) {
                ecg_fifo_.pop_front();
            }
            if (!ecg_fifo_.empty()) {
                last_emitted_ecg_uv_ = ecg_fifo_.front();
            }
            ecg_primed_ = true;
        }

        while (ecg_fifo_.size() > max_fifo_samples_) {
            ecg_fifo_.pop_front();
            ++overflow_dropped_samples_;
        }

        if (packet_trace_cb_) {
            cb_copy = packet_trace_cb_;
            trace.packet_index = packets_received_;
            trace.pc_elapsed_s = std::chrono::duration<double>(now - first_ecg_arrival_time_).count();
            trace.polar_timestamp_ns = pkt.timestamp_ns;
            trace.dt_pc_ms = dt_pc_ms;
            trace.dt_polar_ms = dt_polar_ms;
            trace.frame_type = pkt.frame_type;
            trace.sample_count = pkt.samples_uv.size();
            trace.fifo_after = ecg_fifo_.size();
        }
    }

    ecg_cv_.notify_all();
    if (cb_copy) {
        cb_copy(trace);
    }
}


void PolarSampleBuffer::pushAcc(const PmdAccPacket& pkt) {
    if (!pkt.valid || pkt.samples_mg.empty()) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = clock_t::now();
        const double nominal_hz = static_cast<double>(acc_nominal_rate_hz_ > 0 ? acc_nominal_rate_hz_ : 200);

        if (has_received_acc_) {
            const double dt_pc_ms = std::chrono::duration<double, std::milli>(now - last_acc_arrival_time_).count();
            if (acc_burst_interval_count_ == 0 || dt_pc_ms < acc_burst_interval_min_ms_) {
                acc_burst_interval_min_ms_ = dt_pc_ms;
            }
            if (acc_burst_interval_count_ == 0 || dt_pc_ms > acc_burst_interval_max_ms_) {
                acc_burst_interval_max_ms_ = dt_pc_ms;
            }
            acc_burst_interval_sum_ms_ += dt_pc_ms;
            ++acc_burst_interval_count_;
        } else {
            first_acc_arrival_time_ = now;
        }

        size_t gap_fill = 0;
        if (pkt.timestamp_ns != 0) {
            if (last_acc_timestamp_ns_ != 0 && pkt.timestamp_ns > last_acc_timestamp_ns_) {
                const uint64_t delta_ns = pkt.timestamp_ns - last_acc_timestamp_ns_;
                const double expected_ns =
                    (static_cast<double>(pkt.samples_mg.size()) / nominal_hz) * kNsPerSecond;
                const bool plausible = static_cast<double>(delta_ns) < 10.0 * kNsPerSecond;
                if (static_cast<double>(delta_ns) > expected_ns * 1.5 && plausible) {
                    ++acc_timestamp_gap_packets_;
                }
                if (plausible && has_received_acc_) {
                    const auto total = static_cast<int64_t>(std::llround(
                        static_cast<double>(delta_ns) * nominal_hz / kNsPerSecond));
                    const int64_t missing = total - static_cast<int64_t>(pkt.samples_mg.size());
                    if (missing >= static_cast<int64_t>(kMinGapFillSamples)) {
                        gap_fill = std::min<size_t>(static_cast<size_t>(missing),
                                                    static_cast<size_t>(acc_nominal_rate_hz_) * 2);
                    }
                }
            }

            if (first_acc_timestamp_ns_ == 0 || pkt.timestamp_ns <= last_acc_timestamp_ns_) {
                first_acc_timestamp_ns_ = pkt.timestamp_ns;
                acc_polar_rate_sample_count_ = 0;
            } else {
                acc_polar_rate_sample_count_ += pkt.samples_mg.size() + gap_fill;
            }
            last_acc_timestamp_ns_ = pkt.timestamp_ns;
        }
        last_acc_arrival_time_ = now;

        has_received_acc_ = true;
        ++acc_packets_received_;
        last_acc_frame_type_ = pkt.frame_type;
        last_acc_packet_samples_ = pkt.samples_mg.size();

        if (gap_fill > 0) {
            const size_t absorbed = std::min(gap_fill, acc_underrun_debt_samples_);
            gap_fill -= absorbed;
        }
        acc_underrun_debt_samples_ = 0;

        if (gap_fill > 0) {
            const AccSampleMg a = last_pushed_acc_mg_;
            const AccSampleMg b = {
                sanitizeFloat(pkt.samples_mg.front().x_mg, a.x_mg),
                sanitizeFloat(pkt.samples_mg.front().y_mg, a.y_mg),
                sanitizeFloat(pkt.samples_mg.front().z_mg, a.z_mg),
            };
            for (size_t k = 0; k < gap_fill; ++k) {
                const float t = static_cast<float>(k + 1) / static_cast<float>(gap_fill + 1);
                acc_fifo_.push_back({
                    a.x_mg + (b.x_mg - a.x_mg) * t,
                    a.y_mg + (b.y_mg - a.y_mg) * t,
                    a.z_mg + (b.z_mg - a.z_mg) * t,
                });
            }
            acc_gap_filled_samples_ += gap_fill;
            acc_samples_pushed_ += gap_fill;
        }

        for (const auto& s : pkt.samples_mg) {
            const AccSampleMg clean = {
                sanitizeFloat(s.x_mg, last_pushed_acc_mg_.x_mg),
                sanitizeFloat(s.y_mg, last_pushed_acc_mg_.y_mg),
                sanitizeFloat(s.z_mg, last_pushed_acc_mg_.z_mg),
            };
            acc_fifo_.push_back(clean);
            last_pushed_acc_mg_ = clean;
            ++acc_samples_pushed_;
        }

        if (!acc_primed_ && acc_fifo_.size() >= acc_prebuffer_samples_) {
            while (acc_fifo_.size() > acc_prebuffer_samples_) {
                acc_fifo_.pop_front();
            }
            if (!acc_fifo_.empty()) {
                last_emitted_acc_mg_ = acc_fifo_.front();
            }
            acc_primed_ = true;
        }

        while (acc_fifo_.size() > acc_max_fifo_samples_) {
            acc_fifo_.pop_front();
            ++acc_overflow_dropped_samples_;
        }
    }

    acc_cv_.notify_all();
}

void PolarSampleBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    ecg_primed_ = false;
    ecg_rebuffering_ = false;
    has_emitted_ecg_ = false;
    underrun_debt_samples_ = 0;
    ecg_fifo_.clear();
    samples_since_drift_adjust_ = 0;
    ecg_resamp_init_ = false;
    ecg_resamp_phase_ = 0.0;

    acc_primed_ = false;
    acc_rebuffering_ = false;
    has_emitted_acc_ = false;
    acc_underrun_debt_samples_ = 0;
    acc_fifo_.clear();
    acc_samples_since_drift_adjust_ = 0;
    acc_resamp_init_ = false;
    acc_resamp_phase_ = 0.0;
}

void PolarSampleBuffer::prepareForStreaming(uint32_t fresh_window_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = clock_t::now();
    const bool fresh = has_received_ecg_ &&
                       (std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_ecg_arrival_time_).count() <= static_cast<int64_t>(fresh_window_ms));

    underrun_samples_ = 0;
    overflow_dropped_samples_ = 0;
    drift_dropped_samples_ = 0;
    drift_interpolated_samples_ = 0;
    timestamp_gap_packets_ = 0;
    gap_filled_samples_ = 0;
    samples_popped_ = 0;
    samples_since_drift_adjust_ = 0;
    has_emitted_ecg_ = false;
    ecg_rebuffering_ = false;
    underrun_debt_samples_ = 0;

    burst_interval_count_ = 0;
    burst_interval_min_ms_ = 0.0;
    burst_interval_sum_ms_ = 0.0;
    burst_interval_max_ms_ = 0.0;
    first_ecg_timestamp_ns_ = last_ecg_timestamp_ns_;
    polar_rate_sample_count_ = 0;

    ecg_resamp_init_ = false;
    ecg_resamp_phase_ = 0.0;
    if (fresh && !ecg_fifo_.empty()) {
        while (ecg_fifo_.size() > ecg_prebuffer_samples_) {
            ecg_fifo_.pop_front();
        }
        last_emitted_ecg_uv_ = ecg_fifo_.front();
        ecg_primed_ = true;
    } else {
        ecg_fifo_.clear();
        ecg_primed_ = false;
    }

    const bool acc_fresh = has_received_acc_ &&
                           (std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - last_acc_arrival_time_).count() <= static_cast<int64_t>(fresh_window_ms));
    acc_underrun_samples_ = 0;
    acc_overflow_dropped_samples_ = 0;
    acc_drift_dropped_samples_ = 0;
    acc_drift_interpolated_samples_ = 0;
    acc_timestamp_gap_packets_ = 0;
    acc_gap_filled_samples_ = 0;
    acc_samples_popped_ = 0;
    acc_samples_since_drift_adjust_ = 0;
    has_emitted_acc_ = false;
    acc_rebuffering_ = false;
    acc_underrun_debt_samples_ = 0;
    acc_burst_interval_count_ = 0;
    acc_burst_interval_min_ms_ = 0.0;
    acc_burst_interval_sum_ms_ = 0.0;
    acc_burst_interval_max_ms_ = 0.0;
    first_acc_timestamp_ns_ = last_acc_timestamp_ns_;
    acc_polar_rate_sample_count_ = 0;
    acc_resamp_init_ = false;
    acc_resamp_phase_ = 0.0;

    if (acc_fresh && !acc_fifo_.empty()) {
        while (acc_fifo_.size() > acc_prebuffer_samples_) {
            acc_fifo_.pop_front();
        }
        last_emitted_acc_mg_ = acc_fifo_.front();
        acc_primed_ = true;
    } else {
        acc_fifo_.clear();
        acc_primed_ = false;
    }
}

void PolarSampleBuffer::resetConnectionState() {
    std::lock_guard<std::mutex> lock(mutex_);
    resetEcgStreamStateLocked();
    resetAccStreamStateLocked();
    latest_bpm_ = 0.0f;
    latest_rr_ms_ = 0.0f;
    latest_contact_ = 0.0f;
    latest_battery_pct_ = 0.0f;
}

void PolarSampleBuffer::resetEcgStreamState() {
    std::lock_guard<std::mutex> lock(mutex_);
    resetEcgStreamStateLocked();
}

void PolarSampleBuffer::resetAccStreamState() {
    std::lock_guard<std::mutex> lock(mutex_);
    resetAccStreamStateLocked();
}

void PolarSampleBuffer::resetEcgStreamStateLocked() {
    ecg_primed_ = false;
    ecg_rebuffering_ = false;
    has_received_ecg_ = false;
    has_emitted_ecg_ = false;
    last_emitted_ecg_uv_ = 0.0f;
    last_pushed_ecg_uv_ = 0.0f;
    first_ecg_timestamp_ns_ = 0;
    last_ecg_timestamp_ns_ = 0;
    polar_rate_sample_count_ = 0;
    first_ecg_arrival_time_ = {};
    last_ecg_arrival_time_ = {};
    packets_received_ = 0;
    samples_pushed_ = 0;
    samples_popped_ = 0;
    underrun_samples_ = 0;
    underrun_debt_samples_ = 0;
    overflow_dropped_samples_ = 0;
    drift_dropped_samples_ = 0;
    drift_interpolated_samples_ = 0;
    timestamp_gap_packets_ = 0;
    gap_filled_samples_ = 0;
    samples_since_drift_adjust_ = 0;
    last_frame_type_ = 0;
    last_packet_samples_ = 0;
    burst_interval_count_ = 0;
    burst_interval_min_ms_ = 0.0;
    burst_interval_sum_ms_ = 0.0;
    burst_interval_max_ms_ = 0.0;
    ecg_fifo_.clear();
    ecg_resamp_init_ = false;
    ecg_resamp_phase_ = 0.0;
    ecg_resamp_prev_ = 0.0f;
    ecg_resamp_curr_ = 0.0f;
}

void PolarSampleBuffer::resetAccStreamStateLocked() {
    acc_primed_ = false;
    acc_rebuffering_ = false;
    has_received_acc_ = false;
    has_emitted_acc_ = false;
    last_emitted_acc_mg_ = {};
    last_pushed_acc_mg_ = {};
    first_acc_timestamp_ns_ = 0;
    last_acc_timestamp_ns_ = 0;
    acc_polar_rate_sample_count_ = 0;
    first_acc_arrival_time_ = {};
    last_acc_arrival_time_ = {};
    acc_packets_received_ = 0;
    acc_samples_pushed_ = 0;
    acc_samples_popped_ = 0;
    acc_underrun_samples_ = 0;
    acc_underrun_debt_samples_ = 0;
    acc_overflow_dropped_samples_ = 0;
    acc_drift_dropped_samples_ = 0;
    acc_drift_interpolated_samples_ = 0;
    acc_timestamp_gap_packets_ = 0;
    acc_gap_filled_samples_ = 0;
    acc_samples_since_drift_adjust_ = 0;
    last_acc_frame_type_ = 0;
    last_acc_packet_samples_ = 0;
    acc_burst_interval_count_ = 0;
    acc_burst_interval_min_ms_ = 0.0;
    acc_burst_interval_sum_ms_ = 0.0;
    acc_burst_interval_max_ms_ = 0.0;
    acc_fifo_.clear();
    acc_resamp_init_ = false;
    acc_resamp_phase_ = 0.0;
    acc_resamp_prev_ = {};
    acc_resamp_curr_ = {};
}

bool PolarSampleBuffer::waitForEcg(uint32_t timeout_ms, uint64_t min_packets) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return ecg_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [this, min_packets]() { return packets_received_ >= min_packets; });
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

bool PolarSampleBuffer::hasReceivedEcg() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_received_ecg_;
}

bool PolarSampleBuffer::isPrimed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ecg_primed_;
}

EcgBufferStats PolarSampleBuffer::ecgStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    double avg_ms = 0.0;
    if (burst_interval_count_ > 0) {
        avg_ms = burst_interval_sum_ms_ / static_cast<double>(burst_interval_count_);
    }
    double polar_hz = 0.0;
    if (first_ecg_timestamp_ns_ != 0 &&
        last_ecg_timestamp_ns_ > first_ecg_timestamp_ns_ &&
        polar_rate_sample_count_ > 0) {
        const double span_s = static_cast<double>(last_ecg_timestamp_ns_ - first_ecg_timestamp_ns_) / kNsPerSecond;
        if (span_s > 0.0) {
            polar_hz = static_cast<double>(polar_rate_sample_count_) / span_s;
        }
    }
    EcgBufferStats st;
    st.queued_samples = ecg_fifo_.size();
    st.prebuffer_samples = ecg_prebuffer_samples_;
    st.primed = ecg_primed_;
    st.has_received_ecg = has_received_ecg_;
    st.packets_received = packets_received_;
    st.samples_pushed = samples_pushed_;
    st.samples_popped = samples_popped_;
    st.underrun_samples = underrun_samples_;
    st.overflow_dropped_samples = overflow_dropped_samples_;
    st.drift_dropped_samples = drift_dropped_samples_;
    st.drift_interpolated_samples = drift_interpolated_samples_;
    st.timestamp_gap_packets = timestamp_gap_packets_;
    st.gap_filled_samples = gap_filled_samples_;
    st.last_frame_type = last_frame_type_;
    st.last_packet_samples = last_packet_samples_;
    st.burst_interval_min_ms = burst_interval_min_ms_;
    st.burst_interval_avg_ms = avg_ms;
    st.burst_interval_max_ms = burst_interval_max_ms_;
    st.effective_polar_hz = polar_hz;
    return st;
}


bool PolarSampleBuffer::waitForAcc(uint32_t timeout_ms, uint64_t min_packets) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return acc_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [this, min_packets]() { return acc_packets_received_ >= min_packets; });
}

float PolarSampleBuffer::latestBatteryPct() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_battery_pct_;
}

bool PolarSampleBuffer::hasReceivedAcc() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_received_acc_;
}

bool PolarSampleBuffer::isAccPrimed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return acc_primed_;
}

AccBufferStats PolarSampleBuffer::accStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    double avg_ms = 0.0;
    if (acc_burst_interval_count_ > 0) {
        avg_ms = acc_burst_interval_sum_ms_ / static_cast<double>(acc_burst_interval_count_);
    }
    double polar_hz = 0.0;
    if (first_acc_timestamp_ns_ != 0 &&
        last_acc_timestamp_ns_ > first_acc_timestamp_ns_ &&
        acc_polar_rate_sample_count_ > 0) {
        const double span_s = static_cast<double>(last_acc_timestamp_ns_ - first_acc_timestamp_ns_) / kNsPerSecond;
        if (span_s > 0.0) {
            polar_hz = static_cast<double>(acc_polar_rate_sample_count_) / span_s;
        }
    }
    AccBufferStats st;
    st.queued_samples = acc_fifo_.size();
    st.prebuffer_samples = acc_prebuffer_samples_;
    st.nominal_rate_hz = acc_nominal_rate_hz_;
    st.primed = acc_primed_;
    st.has_received_acc = has_received_acc_;
    st.packets_received = acc_packets_received_;
    st.samples_pushed = acc_samples_pushed_;
    st.samples_popped = acc_samples_popped_;
    st.underrun_samples = acc_underrun_samples_;
    st.overflow_dropped_samples = acc_overflow_dropped_samples_;
    st.drift_dropped_samples = acc_drift_dropped_samples_;
    st.drift_interpolated_samples = acc_drift_interpolated_samples_;
    st.timestamp_gap_packets = acc_timestamp_gap_packets_;
    st.gap_filled_samples = acc_gap_filled_samples_;
    st.last_frame_type = last_acc_frame_type_;
    st.last_packet_samples = last_acc_packet_samples_;
    st.burst_interval_min_ms = acc_burst_interval_min_ms_;
    st.burst_interval_avg_ms = avg_ms;
    st.burst_interval_max_ms = acc_burst_interval_max_ms_;
    st.effective_polar_hz = polar_hz;
    return st;
}

std::vector<float> PolarSampleBuffer::popEcgSamples(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    return popEcgSamplesLocked(count);
}

std::vector<float> PolarSampleBuffer::popEcgSamplesLocked(size_t count) {
    std::vector<float> out;
    out.reserve(count);

    if (!ecg_primed_) {
        out.assign(count, 0.0f);
        samples_popped_ += count;
        return out;
    }

    for (size_t i = 0; i < count; ++i) {
        // Jitter-buffer refill after a genuine underrun (BLE dropout / lost packets that could
        // not be reconstructed): hold the last value until the FIFO is back at the pre-buffer
        // depth. Without this, playback restarts with ~0-73 samples of margin and every
        // subsequent 561 ms burst boundary produces a new micro-plateau.
        if (ecg_rebuffering_) {
            if (ecg_fifo_.size() >= ecg_prebuffer_samples_) {
                ecg_rebuffering_ = false;
                samples_since_drift_adjust_ = 0;
            } else {
                out.push_back(last_emitted_ecg_uv_);
                ++underrun_samples_;
                ++samples_popped_;
                if (ecg_fifo_.empty()) {
                    ++underrun_debt_samples_;
                }
                continue;
            }
        }

        // Soft low-watermark clock drift compensation (Polar clock slightly slower than PC):
        // gently insert a single linearly-interpolated midpoint sample before hitting empty.
        if (low_watermark_ > 0 &&
            has_emitted_ecg_ &&
            !ecg_fifo_.empty() &&
            ecg_fifo_.size() <= low_watermark_ &&
            samples_since_drift_adjust_ >= drift_adjust_interval_) {
            const float interp = 0.5f * (last_emitted_ecg_uv_ + ecg_fifo_.front());
            out.push_back(interp);
            last_emitted_ecg_uv_ = interp;
            samples_since_drift_adjust_ = 0;
            ++drift_interpolated_samples_;
            ++samples_popped_;
            continue;
        }

        if (!ecg_fifo_.empty()) {
            const float s = ecg_fifo_.front();
            ecg_fifo_.pop_front();
            out.push_back(s);
            last_emitted_ecg_uv_ = s;
            has_emitted_ecg_ = true;
            ++samples_since_drift_adjust_;
            ++samples_popped_;

            // Soft high-watermark clock drift compensation (Polar clock slightly faster than PC):
            // blend 1 dropped sample with its neighbor at most once per drift_adjust_interval_
            // so latency stays bounded without clipping sharp QRS R-peaks.
            if (ecg_fifo_.size() > high_watermark_ &&
                samples_since_drift_adjust_ >= drift_adjust_interval_) {
                const float dropped = ecg_fifo_.front();
                ecg_fifo_.pop_front();
                if (!ecg_fifo_.empty()) {
                    ecg_fifo_.front() = 0.5f * (dropped + ecg_fifo_.front());
                }
                samples_since_drift_adjust_ = 0;
                ++drift_dropped_samples_;
            }
        } else {
            out.push_back(last_emitted_ecg_uv_);
            ++underrun_samples_;
            ++samples_popped_;
            ++underrun_debt_samples_;
            ecg_rebuffering_ = true;
        }
    }
    return out;
}


std::vector<float> PolarSampleBuffer::popEcgSamplesResampled(size_t count, double output_rate_hz) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count == 0) return {};
    if (output_rate_hz <= 0.0 || std::fabs(output_rate_hz - kPolarEcgNominalRateHz) < 1e-3) {
        return popEcgSamplesLocked(count);
    }

    const double step = kPolarEcgNominalRateHz / output_rate_hz;
    std::vector<float> out;
    out.reserve(count);

    if (!ecg_resamp_init_) {
        const auto init_s = popEcgSamplesLocked(1);
        ecg_resamp_prev_ = init_s.empty() ? 0.0f : init_s[0];
        ecg_resamp_curr_ = ecg_resamp_prev_;
        ecg_resamp_phase_ = 1.0;
        ecg_resamp_init_ = true;
    }

    for (size_t i = 0; i < count; ++i) {
        while (ecg_resamp_phase_ >= 1.0) {
            ecg_resamp_prev_ = ecg_resamp_curr_;
            const auto next_s = popEcgSamplesLocked(1);
            ecg_resamp_curr_ = next_s.empty() ? ecg_resamp_prev_ : next_s[0];
            ecg_resamp_phase_ -= 1.0;
        }
        const float t = static_cast<float>(std::clamp(ecg_resamp_phase_, 0.0, 1.0));
        out.push_back(ecg_resamp_prev_ + (ecg_resamp_curr_ - ecg_resamp_prev_) * t);
        ecg_resamp_phase_ += step;
    }
    return out;
}

AccPlanarArrays PolarSampleBuffer::popAccSamples(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    return popAccSamplesLocked(count);
}

AccPlanarArrays PolarSampleBuffer::popAccSamplesLocked(size_t count) {
    AccPlanarArrays out;
    out.x_mg.reserve(count);
    out.y_mg.reserve(count);
    out.z_mg.reserve(count);

    if (!acc_primed_) {
        out.x_mg.assign(count, 0.0f);
        out.y_mg.assign(count, 0.0f);
        out.z_mg.assign(count, 0.0f);
        acc_samples_popped_ += count;
        return out;
    }

    auto emit = [&](const AccSampleMg& s) {
        out.x_mg.push_back(s.x_mg);
        out.y_mg.push_back(s.y_mg);
        out.z_mg.push_back(s.z_mg);
    };

    for (size_t i = 0; i < count; ++i) {
        if (acc_rebuffering_) {
            if (acc_fifo_.size() >= acc_prebuffer_samples_) {
                acc_rebuffering_ = false;
                acc_samples_since_drift_adjust_ = 0;
            } else {
                emit(last_emitted_acc_mg_);
                ++acc_underrun_samples_;
                ++acc_samples_popped_;
                if (acc_fifo_.empty()) {
                    ++acc_underrun_debt_samples_;
                }
                continue;
            }
        }

        if (acc_low_watermark_ > 0 &&
            has_emitted_acc_ &&
            !acc_fifo_.empty() &&
            acc_fifo_.size() <= acc_low_watermark_ &&
            acc_samples_since_drift_adjust_ >= acc_drift_adjust_interval_) {
            const auto& f = acc_fifo_.front();
            const AccSampleMg interp = {
                0.5f * (last_emitted_acc_mg_.x_mg + f.x_mg),
                0.5f * (last_emitted_acc_mg_.y_mg + f.y_mg),
                0.5f * (last_emitted_acc_mg_.z_mg + f.z_mg),
            };
            emit(interp);
            last_emitted_acc_mg_ = interp;
            acc_samples_since_drift_adjust_ = 0;
            ++acc_drift_interpolated_samples_;
            ++acc_samples_popped_;
            continue;
        }

        if (!acc_fifo_.empty()) {
            const AccSampleMg s = acc_fifo_.front();
            acc_fifo_.pop_front();
            emit(s);
            last_emitted_acc_mg_ = s;
            has_emitted_acc_ = true;
            ++acc_samples_since_drift_adjust_;
            ++acc_samples_popped_;

            if (acc_fifo_.size() > acc_high_watermark_ &&
                acc_samples_since_drift_adjust_ >= acc_drift_adjust_interval_) {
                const AccSampleMg dropped = acc_fifo_.front();
                acc_fifo_.pop_front();
                if (!acc_fifo_.empty()) {
                    acc_fifo_.front().x_mg = 0.5f * (dropped.x_mg + acc_fifo_.front().x_mg);
                    acc_fifo_.front().y_mg = 0.5f * (dropped.y_mg + acc_fifo_.front().y_mg);
                    acc_fifo_.front().z_mg = 0.5f * (dropped.z_mg + acc_fifo_.front().z_mg);
                }
                acc_samples_since_drift_adjust_ = 0;
                ++acc_drift_dropped_samples_;
            }
        } else {
            emit(last_emitted_acc_mg_);
            ++acc_underrun_samples_;
            ++acc_samples_popped_;
            ++acc_underrun_debt_samples_;
            acc_rebuffering_ = true;
        }
    }
    return out;
}

AccPlanarArrays PolarSampleBuffer::popAccSamplesResampled(size_t count, double output_rate_hz) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count == 0) return {};
    const double native_hz = static_cast<double>(acc_nominal_rate_hz_ > 0 ? acc_nominal_rate_hz_ : 200);
    if (output_rate_hz <= 0.0 || std::fabs(output_rate_hz - native_hz) < 1e-3) {
        return popAccSamplesLocked(count);
    }

    const double step = native_hz / output_rate_hz;
    AccPlanarArrays out;
    out.x_mg.reserve(count);
    out.y_mg.reserve(count);
    out.z_mg.reserve(count);

    auto popOne = [&]() -> AccSampleMg {
        const auto a = popAccSamplesLocked(1);
        if (a.x_mg.empty()) return acc_resamp_prev_;
        return { a.x_mg[0], a.y_mg[0], a.z_mg[0] };
    };

    if (!acc_resamp_init_) {
        acc_resamp_prev_ = popOne();
        acc_resamp_curr_ = acc_resamp_prev_;
        acc_resamp_phase_ = 1.0;
        acc_resamp_init_ = true;
    }

    for (size_t i = 0; i < count; ++i) {
        while (acc_resamp_phase_ >= 1.0) {
            acc_resamp_prev_ = acc_resamp_curr_;
            acc_resamp_curr_ = popOne();
            acc_resamp_phase_ -= 1.0;
        }
        const float t = static_cast<float>(std::clamp(acc_resamp_phase_, 0.0, 1.0));
        out.x_mg.push_back(acc_resamp_prev_.x_mg + (acc_resamp_curr_.x_mg - acc_resamp_prev_.x_mg) * t);
        out.y_mg.push_back(acc_resamp_prev_.y_mg + (acc_resamp_curr_.y_mg - acc_resamp_prev_.y_mg) * t);
        out.z_mg.push_back(acc_resamp_prev_.z_mg + (acc_resamp_curr_.z_mg - acc_resamp_prev_.z_mg) * t);
        acc_resamp_phase_ += step;
    }
    return out;
}

} // namespace qdevice_polar
