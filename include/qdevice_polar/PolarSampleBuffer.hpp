#pragma once

#include "qdevice_polar/PolarPacketParser.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>

namespace qdevice_polar {

/// Default ECG pre-buffer size in samples at 130 Hz (~846 ms).
/// Matches the default 850 ms ACC pre-buffer so ECG and ACC stay time-aligned with each other
/// while absorbing Polar H10 561.5 ms bursts + Windows BLE jitter.
inline constexpr size_t kDefaultEcgPrebufferSamples = 110;
/// Default Accelerometer pre-buffer size in samples at 200 Hz (~850 ms).
inline constexpr size_t kDefaultAccPrebufferSamples = 170;

struct AccPlanarArrays {
    std::vector<float> x_mg;
    std::vector<float> y_mg;
    std::vector<float> z_mg;
};

struct AccBufferStats {
    size_t queued_samples = 0;
    size_t prebuffer_samples = 0;
    uint16_t nominal_rate_hz = 200;
    bool primed = false;
    bool has_received_acc = false;
    uint64_t packets_received = 0;
    uint64_t samples_pushed = 0;
    uint64_t samples_popped = 0;
    uint64_t underrun_samples = 0;
    uint64_t overflow_dropped_samples = 0;
    uint64_t drift_dropped_samples = 0;
    uint64_t drift_interpolated_samples = 0;
    uint64_t timestamp_gap_packets = 0;
    uint64_t gap_filled_samples = 0;
    uint8_t last_frame_type = 0;
    size_t last_packet_samples = 0;
    double burst_interval_min_ms = 0.0;
    double burst_interval_avg_ms = 0.0;
    double burst_interval_max_ms = 0.0;
    double effective_polar_hz = 0.0;
};

struct EcgPacketTrace {
    uint64_t packet_index = 0;
    double pc_elapsed_s = 0.0;
    uint64_t polar_timestamp_ns = 0;
    double dt_pc_ms = 0.0;
    double dt_polar_ms = 0.0;
    uint8_t frame_type = 0;
    size_t sample_count = 0;
    size_t fifo_after = 0;
};

using EcgPacketTraceCallback = std::function<void(const EcgPacketTrace&)>;

struct EcgBufferStats {
    size_t queued_samples = 0;
    size_t prebuffer_samples = 0;
    bool primed = false;
    bool has_received_ecg = false;
    uint64_t packets_received = 0;
    uint64_t samples_pushed = 0;
    uint64_t samples_popped = 0;
    uint64_t underrun_samples = 0;
    uint64_t overflow_dropped_samples = 0;
    uint64_t drift_dropped_samples = 0;
    uint64_t drift_interpolated_samples = 0;
    uint64_t timestamp_gap_packets = 0;
    uint64_t gap_filled_samples = 0;
    uint8_t last_frame_type = 0;
    size_t last_packet_samples = 0;
    double burst_interval_min_ms = 0.0;
    double burst_interval_avg_ms = 0.0;
    double burst_interval_max_ms = 0.0;
    double effective_polar_hz = 0.0;
};

class PolarSampleBuffer {
public:
    explicit PolarSampleBuffer(size_t ecg_prebuffer_samples = kDefaultEcgPrebufferSamples);

    /// Dynamically updates the ECG pre-buffer size and derived drift watermarks.
    void setEcgPrebufferSamples(size_t prebuffer_samples);
    void setAccPrebufferSamples(size_t prebuffer_samples);
    void setAccNominalRateHz(uint16_t rate_hz);

    /// Optional per-packet telemetry callback (used by diagnostic CLI tools).
    void setPacketTraceCallback(EcgPacketTraceCallback cb);

    void pushHeartRate(const HeartRatePacket& pkt);
    void pushBatteryLevel(float battery_pct);
    void pushEcg(const PmdEcgPacket& pkt);
    void pushAcc(const PmdAccPacket& pkt);

    /// Resets the ECG FIFO and priming state while preserving latest HR/RR/Contact.
    void clear();

    /// Prepares the buffer for a new QTM capture:
    /// - Preserves latest HR/RR/Contact state.
    /// - If fresh live ECG packets arrived within `fresh_window_ms`, trims the FIFO to at most
    ///   `ecg_prebuffer_samples_` most recent samples and keeps `ecg_primed_ = true` so capture
    ///   starts immediately at t=0 with live ECG and zero leading zeros (even across rapid
    ///   Stop -> Start cycles in QTM).
    /// - Otherwise clears the FIFO and waits for `ecg_prebuffer_samples_` to arrive.
    void prepareForStreaming(uint32_t fresh_window_ms = 2000);

    /// Full reset on BLE disconnect / reconnect: ECG stream state AND latest HR/RR/Contact
    /// (prevents a stale BPM from a previous strap/session being emitted after reconnect).
    void resetConnectionState();

    /// Resets only the ECG stream state (FIFO, timestamps, counters) while preserving the
    /// latest HR/RR/Contact/Battery/ACC values. Used between PMD Stop/Start retries inside connect().
    void resetEcgStreamState();

    /// Resets only the Accelerometer stream state (FIFO, timestamps, counters).
    void resetAccStreamState();

    /// Blocks until at least `min_packets` ECG frames have been received or `timeout_ms` elapses.
    bool waitForEcg(uint32_t timeout_ms, uint64_t min_packets = 1) const;

    /// Blocks until at least `min_packets` ACC frames have been received or `timeout_ms` elapses.
    bool waitForAcc(uint32_t timeout_ms, uint64_t min_packets = 1) const;

    float latestBpm() const;
    float latestRrMs() const;
    float latestContact() const;
    float latestBatteryPct() const;
    bool hasReceivedEcg() const;
    bool isPrimed() const;
    bool hasReceivedAcc() const;
    bool isAccPrimed() const;
    EcgBufferStats ecgStats() const;
    AccBufferStats accStats() const;

    /// Pops `count` ECG samples in microvolts.
    /// Waits until `ecg_prebuffer_samples_` have arrived on initial start before draining,
    /// applies gentle clock-drift compensation (PC steady_clock vs Polar 130 Hz oscillator),
    /// and holds the last emitted sample on genuine BLE underrun. After an underrun the
    /// buffer re-primes (holds) until `ecg_prebuffer_samples_` are queued again, so a
    /// single BLE dropout produces ONE bounded plateau instead of a cascade of micro-plateaus.
    std::vector<float> popEcgSamples(size_t count);

    /// Pops `count` ECG samples resampled to `output_rate_hz` (e.g. 200 Hz when co-streaming
    /// with the 200 Hz accelerometer on a unified QTM board). If `output_rate_hz == 130`,
    /// delegates directly to `popEcgSamples(count)` with zero resampling overhead.
    std::vector<float> popEcgSamplesResampled(size_t count, double output_rate_hz);

    /// Pops `count` 3-axis Accelerometer samples in milli-G (mG) at the native ACC rate.
    AccPlanarArrays popAccSamples(size_t count);

    /// Pops `count` 3-axis Accelerometer samples resampled to `output_rate_hz`.
    AccPlanarArrays popAccSamplesResampled(size_t count, double output_rate_hz);

    /// Maximum number of missing samples reconstructed from a Polar timestamp gap (2 s).
    static constexpr size_t kMaxGapFillSamples = 260;

private:
    using clock_t = std::chrono::steady_clock;

    void updateWatermarksLocked();
    void updateAccWatermarksLocked();
    void resetEcgStreamStateLocked();
    void resetAccStreamStateLocked();
    std::vector<float> popEcgSamplesLocked(size_t count);
    AccPlanarArrays popAccSamplesLocked(size_t count);

    mutable std::mutex mutex_;
    mutable std::condition_variable ecg_cv_;

    size_t ecg_prebuffer_samples_ = kDefaultEcgPrebufferSamples;
    size_t max_fifo_samples_ = 390;
    size_t high_watermark_ = 210;
    size_t low_watermark_ = 12;
    size_t drift_adjust_interval_ = 65; // Max ~1.5% clock correction (1 sample per 0.5 s)

    bool ecg_primed_ = false;
    bool ecg_rebuffering_ = false;
    bool has_received_ecg_ = false;
    bool has_emitted_ecg_ = false;

    float latest_bpm_ = 0.0f;
    float latest_rr_ms_ = 0.0f;
    float latest_contact_ = 0.0f;
    float last_emitted_ecg_uv_ = 0.0f;
    float last_pushed_ecg_uv_ = 0.0f;

    uint64_t first_ecg_timestamp_ns_ = 0;
    uint64_t last_ecg_timestamp_ns_ = 0;
    uint64_t polar_rate_sample_count_ = 0;
    clock_t::time_point first_ecg_arrival_time_{};
    clock_t::time_point last_ecg_arrival_time_{};

    uint64_t packets_received_ = 0;
    uint64_t samples_pushed_ = 0;
    uint64_t samples_popped_ = 0;
    uint64_t underrun_samples_ = 0;
    uint64_t overflow_dropped_samples_ = 0;
    uint64_t drift_dropped_samples_ = 0;
    uint64_t drift_interpolated_samples_ = 0;
    uint64_t timestamp_gap_packets_ = 0;
    uint64_t gap_filled_samples_ = 0;
    size_t underrun_debt_samples_ = 0; // holds emitted with an empty FIFO since last packet
    size_t samples_since_drift_adjust_ = 0;

    uint8_t last_frame_type_ = 0;
    size_t last_packet_samples_ = 0;
    uint64_t burst_interval_count_ = 0;
    double burst_interval_min_ms_ = 0.0;
    double burst_interval_sum_ms_ = 0.0;
    double burst_interval_max_ms_ = 0.0;

    EcgPacketTraceCallback packet_trace_cb_;
    std::deque<float> ecg_fifo_;

    float latest_battery_pct_ = 0.0f;

    // Fractional phase & endpoint state for rate-conversion when ECG (130 Hz) and ACC (e.g. 200 Hz)
    // share a single unified QTM board rate.
    bool ecg_resamp_init_ = false;
    double ecg_resamp_phase_ = 0.0;
    float ecg_resamp_prev_ = 0.0f;
    float ecg_resamp_curr_ = 0.0f;

    // Accelerometer FIFO & drift/gap/telemetry state
    mutable std::condition_variable acc_cv_;
    uint16_t acc_nominal_rate_hz_ = 200;
    size_t acc_prebuffer_samples_ = kDefaultAccPrebufferSamples;
    size_t acc_max_fifo_samples_ = 600;
    size_t acc_high_watermark_ = 315;
    size_t acc_low_watermark_ = 16;
    size_t acc_drift_adjust_interval_ = 100;

    bool acc_primed_ = false;
    bool acc_rebuffering_ = false;
    bool has_received_acc_ = false;
    bool has_emitted_acc_ = false;

    AccSampleMg last_emitted_acc_mg_{};
    AccSampleMg last_pushed_acc_mg_{};

    uint64_t first_acc_timestamp_ns_ = 0;
    uint64_t last_acc_timestamp_ns_ = 0;
    uint64_t acc_polar_rate_sample_count_ = 0;
    clock_t::time_point first_acc_arrival_time_{};
    clock_t::time_point last_acc_arrival_time_{};

    uint64_t acc_packets_received_ = 0;
    uint64_t acc_samples_pushed_ = 0;
    uint64_t acc_samples_popped_ = 0;
    uint64_t acc_underrun_samples_ = 0;
    uint64_t acc_overflow_dropped_samples_ = 0;
    uint64_t acc_drift_dropped_samples_ = 0;
    uint64_t acc_drift_interpolated_samples_ = 0;
    uint64_t acc_timestamp_gap_packets_ = 0;
    uint64_t acc_gap_filled_samples_ = 0;
    size_t acc_underrun_debt_samples_ = 0;
    size_t acc_samples_since_drift_adjust_ = 0;

    uint8_t last_acc_frame_type_ = 0;
    size_t last_acc_packet_samples_ = 0;
    uint64_t acc_burst_interval_count_ = 0;
    double acc_burst_interval_min_ms_ = 0.0;
    double acc_burst_interval_sum_ms_ = 0.0;
    double acc_burst_interval_max_ms_ = 0.0;

    std::deque<AccSampleMg> acc_fifo_;

    bool acc_resamp_init_ = false;
    double acc_resamp_phase_ = 0.0;
    AccSampleMg acc_resamp_prev_{};
    AccSampleMg acc_resamp_curr_{};
};

} // namespace qdevice_polar
