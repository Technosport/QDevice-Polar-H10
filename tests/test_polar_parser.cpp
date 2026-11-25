#include "qdevice_polar/PolarPacketParser.hpp"
#include "qdevice_polar/PolarSampleBuffer.hpp"

#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

using namespace qdevice_polar;

namespace {

#define CHECK(cond)                                                                      \
    do {                                                                                 \
        if (!(cond)) {                                                                   \
            std::cerr << "CHECK FAILED at " << __FILE__ << ":" << __LINE__ << ": "       \
                      << #cond << "\n";                                                 \
            std::abort();                                                                \
        }                                                                                \
    } while (0)

bool approx(float a, float b, float eps = 1e-2f) {
    return std::fabs(a - b) <= eps;
}

std::vector<uint8_t> makeRawEcgFrame(uint64_t ts_ns, const std::vector<int32_t>& samples, uint8_t frame_type = 0x00) {
    std::vector<uint8_t> frame;
    frame.reserve(10 + samples.size() * 3);
    frame.push_back(0x00); // ECG measurement type
    for (int i = 0; i < 8; ++i) {
        frame.push_back(static_cast<uint8_t>((ts_ns >> (8 * i)) & 0xFFu));
    }
    frame.push_back(frame_type);
    for (int32_t s : samples) {
        const uint32_t u = static_cast<uint32_t>(s);
        frame.push_back(static_cast<uint8_t>(u & 0xFFu));
        frame.push_back(static_cast<uint8_t>((u >> 8) & 0xFFu));
        frame.push_back(static_cast<uint8_t>((u >> 16) & 0xFFu));
    }
    return frame;
}

void appendSignedBitsLsb(std::vector<uint8_t>& out, size_t& bit_pos, int32_t value, uint8_t bit_width) {
    const uint32_t u = static_cast<uint32_t>(value);
    for (uint8_t b = 0; b < bit_width; ++b) {
        const size_t byte_idx = bit_pos >> 3;
        const size_t bit_idx  = bit_pos & 7u;
        if (byte_idx >= out.size()) {
            out.push_back(0);
        }
        if ((u & (1u << b)) != 0) {
            out[byte_idx] |= static_cast<uint8_t>(1u << bit_idx);
        }
        ++bit_pos;
    }
}

void testHeartRateUint8WithContactAndRr() {
    // Flags = 0x16: bit0=0 (uint8), bit1-2=11 (contact supported+detected), bit3=0, bit4=1 (RR present)
    // HR = 72 bpm
    // RR = 853 (0x0355 -> LE: 0x55, 0x03) -> 853 * 1000 / 1024 = 833.0078 ms
    const uint8_t payload[] = { 0x16, 72, 0x55, 0x03 };
    const auto pkt = parseHeartRateMeasurement(payload, sizeof(payload));
    CHECK(pkt.valid);
    CHECK(approx(pkt.bpm, 72.0f));
    CHECK(pkt.contact_supported);
    CHECK(pkt.contact_detected);
    CHECK(pkt.rr_intervals_ms.size() == 1);
    CHECK(approx(pkt.rr_intervals_ms[0], 833.0078f, 0.05f));
}

void testHeartRateUint16WithEnergyExpended() {
    // Flags = 0x19: bit0=1 (uint16), bit3=1 (energy expended 2 bytes), bit4=1 (RR present)
    // HR = 260 (0x04, 0x01), Energy = 500 kJ (0xF4, 0x01), RR = 1024 -> 1000.0 ms
    const uint8_t payload[] = { 0x19, 0x04, 0x01, 0xF4, 0x01, 0x00, 0x04 };
    const auto pkt = parseHeartRateMeasurement(payload, sizeof(payload));
    CHECK(pkt.valid);
    CHECK(approx(pkt.bpm, 260.0f));
    CHECK(pkt.rr_intervals_ms.size() == 1);
    CHECK(approx(pkt.rr_intervals_ms[0], 1000.0f));

    // Truncated payload must be rejected
    CHECK(!parseHeartRateMeasurement(payload, 1).valid);
    CHECK(!parseHeartRateMeasurement(nullptr, 4).valid);
}

void testPmdEcgRawType0AndRejections() {
    // Standard 24-bit signed raw TYPE_0 frame (frame_type = 0x00)
    const auto frame = makeRawEcgFrame(0x0100000000000000ULL, { 100, -100, 0, 8388607, -8388608 });
    const auto ecg = parsePmdEcgFrame(frame.data(), frame.size());
    CHECK(ecg.valid);
    CHECK(!ecg.is_compressed);
    CHECK(ecg.frame_type == 0x00);
    CHECK(ecg.timestamp_ns == 0x0100000000000000ULL);
    CHECK(ecg.samples_uv.size() == 5);
    CHECK(approx(ecg.samples_uv[0], 100.0f));
    CHECK(approx(ecg.samples_uv[1], -100.0f));
    CHECK(approx(ecg.samples_uv[2], 0.0f));
    CHECK(approx(ecg.samples_uv[3], 8388607.0f, 1.0f));
    CHECK(approx(ecg.samples_uv[4], -8388608.0f, 1.0f));

    // Typical Polar H10 burst: 73 samples => 10 + 73 * 3 = 229 bytes
    std::vector<int32_t> burst(73);
    for (int i = 0; i < 73; ++i) burst[i] = (i - 36) * 25;
    const auto burst_frame = makeRawEcgFrame(1'000'000'000ULL, burst);
    CHECK(burst_frame.size() == 229);
    const auto parsed_burst = parsePmdEcgFrame(burst_frame.data(), burst_frame.size());
    CHECK(parsed_burst.valid);
    CHECK(parsed_burst.samples_uv.size() == 73);
    CHECK(approx(parsed_burst.samples_uv.front(), -900.0f));
    CHECK(approx(parsed_burst.samples_uv.back(), 900.0f));

    // Misaligned payload (not a multiple of 3 bytes) must be rejected
    auto truncated = burst_frame;
    truncated.pop_back();
    CHECK(!parsePmdEcgFrame(truncated.data(), truncated.size()).valid);

    // Unsupported raw frame_type (e.g. 0x01, 0x03) must be rejected instead of corrupting ECG
    const auto bad_ft = makeRawEcgFrame(1234ULL, { 10, 20 }, 0x01);
    CHECK(!parsePmdEcgFrame(bad_ft.data(), bad_ft.size()).valid);

    // Wrong measurement type (e.g. ACC = 0x02) must be rejected
    auto acc_frame = makeRawEcgFrame(1234ULL, { 10, 20 }, 0x00);
    acc_frame[0] = 0x02;
    CHECK(!parsePmdEcgFrame(acc_frame.data(), acc_frame.size()).valid);
}

void testPmdEcgCompressedDeltaType0() {
    // Build a Polar PMD compressed TYPE_0 frame (frame_type = 0x80):
    //   ref = +500 uV (24-bit LE: 0xF4, 0x01, 0x00)
    //   block 1: delta_size = 5 bits, count = 4, deltas = [+10, -12, +15, -16]
    //     expected samples: 500, 510, 498, 513, 497
    //   block 2: delta_size = 0 bits, count = 2 (2 repeated samples -> 497, 497)
    //   block 3: delta_size = 8 bits, count = 2, deltas = [+100, -50] -> 597, 547
    std::vector<uint8_t> frame = {
        0x00, // ECG
        0x10, 0x20, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, // timestamp
        0x80, // frame_type: compressed TYPE_0
        0xF4, 0x01, 0x00 // ref = +500
    };

    // Block 1: 5-bit signed deltas
    frame.push_back(5); // delta_size_bits
    frame.push_back(4); // sample_count
    std::vector<uint8_t> packed;
    size_t bit_pos = 0;
    appendSignedBitsLsb(packed, bit_pos, +10, 5);
    appendSignedBitsLsb(packed, bit_pos, -12, 5);
    appendSignedBitsLsb(packed, bit_pos, +15, 5);
    appendSignedBitsLsb(packed, bit_pos, -16, 5);
    frame.insert(frame.end(), packed.begin(), packed.end());

    // Block 2: 0-bit deltas (repeat)
    frame.push_back(0);
    frame.push_back(2);

    // Block 3: 8-bit signed deltas
    frame.push_back(8);
    frame.push_back(2);
    frame.push_back(static_cast<uint8_t>(100));
    frame.push_back(static_cast<uint8_t>(static_cast<int8_t>(-50)));

    const auto ecg = parsePmdEcgFrame(frame.data(), frame.size());
    CHECK(ecg.valid);
    CHECK(ecg.is_compressed);
    CHECK(ecg.frame_type == 0x80);
    const std::vector<float> expected = { 500.f, 510.f, 498.f, 513.f, 497.f, 497.f, 497.f, 597.f, 547.f };
    CHECK(ecg.samples_uv.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        CHECK(approx(ecg.samples_uv[i], expected[i]));
    }

    // Truncated compressed block must be rejected cleanly
    auto truncated = frame;
    truncated.pop_back();
    CHECK(!parsePmdEcgFrame(truncated.data(), truncated.size()).valid);

    // Unsupported compressed sub-type (0x81) must be rejected
    auto bad_comp = frame;
    bad_comp[9] = 0x81;
    CHECK(!parsePmdEcgFrame(bad_comp.data(), bad_comp.size()).valid);
}

void testPmdControlPointResponsesAndSettings() {
    // 0xF0 Start ECG SUCCESS
    const uint8_t ok_start[] = { 0xF0, 0x02, 0x00, 0x00, 0x00 };
    const auto r1 = parsePmdControlPointResponse(ok_start, sizeof(ok_start));
    CHECK(r1.valid);
    CHECK(r1.op_code == 0x02);
    CHECK(r1.measurement_type == 0x00);
    CHECK(r1.error_code == PmdControlPointError::Success);
    CHECK(!r1.more_frames);

    // 0xF0 Start ECG INVALID_MTU (0x0A)
    const uint8_t mtu_err[] = { 0xF0, 0x02, 0x00, 0x0A };
    const auto r2 = parsePmdControlPointResponse(mtu_err, sizeof(mtu_err));
    CHECK(r2.valid);
    CHECK(r2.error_code == PmdControlPointError::InvalidMtu);

    // 0xF0 Get ECG Settings response with 130 Hz (0x0082) & 14-bit (0x000E)
    const uint8_t get_settings_resp[] = {
        0xF0, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x01, 0x82, 0x00, // SAMPLE_RATE: 130 Hz
        0x01, 0x01, 0x0E, 0x00  // RESOLUTION:  14 bit
    };
    const auto r3 = parsePmdControlPointResponse(get_settings_resp, sizeof(get_settings_resp));
    CHECK(r3.valid);
    CHECK(r3.error_code == PmdControlPointError::Success);
    const auto settings = parsePmdEcgSettings(r3.parameters.data(), r3.parameters.size());
    CHECK(settings.valid);
    CHECK(settings.sample_rates_hz.size() == 1 && settings.sample_rates_hz[0] == 130);
    CHECK(settings.resolutions_bits.size() == 1 && settings.resolutions_bits[0] == 14);

    // Start command builder matches Polar H10 spec exactly
    const auto cmd = buildPmdStartEcgCommand(130, 14);
    const std::vector<uint8_t> expected_cmd = { 0x02, 0x00, 0x00, 0x01, 0x82, 0x00, 0x01, 0x01, 0x0E, 0x00 };
    CHECK(cmd == expected_cmd);

    // Unsolicited server stop notification (0x01 0x00 = ECG stopped)
    const uint8_t server_stop[] = { 0x01, 0x00 };
    CHECK(isPmdServerStopNotification(server_stop, sizeof(server_stop), 0x00));
    CHECK(!isPmdServerStopNotification(server_stop, sizeof(server_stop), 0x02));
    CHECK(!parsePmdControlPointResponse(server_stop, sizeof(server_stop)).valid);

    // PMD Feature read: H10 (0x0F 0x05 = ECG + ACC) vs OH1 (0x0F 0x0E = PPG+ACC+PPI, no ECG)
    const uint8_t h10_feat[] = { 0x0F, 0x05, 0x00 };
    const auto f_h10 = parsePmdFeatures(h10_feat, sizeof(h10_feat));
    CHECK(f_h10.valid && f_h10.ecg_supported && f_h10.acc_supported && !f_h10.ppg_supported);

    const uint8_t oh1_feat[] = { 0x0F, 0x0E, 0x00 };
    const auto f_oh1 = parsePmdFeatures(oh1_feat, sizeof(oh1_feat));
    CHECK(f_oh1.valid && !f_oh1.ecg_supported && f_oh1.ppg_supported);
}

void testSampleBufferBasicAndPrebuffer() {
    const auto frame = makeRawEcgFrame(1'000'000'000ULL, { 100, -100 });
    const auto ecg = parsePmdEcgFrame(frame.data(), frame.size());
    CHECK(ecg.valid);

    PolarSampleBuffer buf(2);
    // Before 2 samples arrive, popEcgSamples returns zeros
    const auto pre = buf.popEcgSamples(2);
    CHECK(approx(pre[0], 0.0f));
    CHECK(approx(pre[1], 0.0f));

    buf.pushEcg(ecg);
    const auto popped = buf.popEcgSamples(3);
    CHECK(popped.size() == 3);
    CHECK(approx(popped[0], 100.0f));
    CHECK(approx(popped[1], -100.0f));
    CHECK(approx(popped[2], -100.0f)); // held last sample on underrun
    CHECK(buf.ecgStats().underrun_samples == 1);
}

void testBurstArrivalWithJitterZeroUnderruns() {
    // Simulate real Polar H10 130 Hz streaming:
    // - 73-sample bursts arriving every 561.54 ms with +/-80 ms BLE jitter
    // - QTM polling every ~23 ms (3 samples/poll) for 60 seconds
    PolarSampleBuffer buf; // Default kDefaultEcgPrebufferSamples
    CHECK(buf.ecgStats().prebuffer_samples == kDefaultEcgPrebufferSamples);

    const double burst_period_s = 73.0 / 130.0;
    const double jitter_pattern_s[] = { 0.0, +0.060, -0.045, +0.080, -0.070, +0.030, -0.020 };

    uint64_t polar_ts_ns = 1'000'000'000ULL;
    size_t sample_seq = 0;
    size_t burst_idx = 0;

    auto pushBurst = [&]() {
        PmdEcgPacket pkt;
        pkt.valid = true;
        pkt.timestamp_ns = polar_ts_ns;
        pkt.samples_uv.resize(73);
        for (size_t i = 0; i < 73; ++i) {
            pkt.samples_uv[i] = static_cast<float>(sample_seq++);
        }
        polar_ts_ns += static_cast<uint64_t>(burst_period_s * 1e9);
        buf.pushEcg(pkt);
        ++burst_idx;
    };

    // Pre-connect idle phase: 3 bursts arrive before QTM clicks Start Capture
    pushBurst();
    pushBurst();
    pushBurst();
    buf.prepareForStreaming();
    CHECK(buf.ecgStats().primed);
    CHECK(buf.ecgStats().queued_samples == kDefaultEcgPrebufferSamples);

    // 60 seconds of QTM capture at 130 Hz (7800 samples)
    double t = 0.0;
    double next_burst_nominal = burst_period_s;
    const double poll_dt = 3.0 / 130.0; // ~23.08 ms per poll

    for (int poll = 0; poll < 2600; ++poll) {
        t += poll_dt;
        const double jitter = jitter_pattern_s[burst_idx % 7];
        if (t >= next_burst_nominal + jitter) {
            pushBurst();
            next_burst_nominal += burst_period_s;
        }
        const auto out = buf.popEcgSamples(3);
        CHECK(out.size() == 3);
    }

    const auto stats = buf.ecgStats();
    CHECK(stats.underrun_samples == 0);
    CHECK(stats.overflow_dropped_samples == 0);
    CHECK(stats.drift_dropped_samples == 0);
    CHECK(stats.drift_interpolated_samples == 0);
    CHECK(stats.timestamp_gap_packets == 0);
    CHECK(std::fabs(stats.effective_polar_hz - 130.0) < 0.05);
}

void testClockDriftCompensationSlowAndFastPolar() {
    // 1. Slow Polar clock: Polar at 129.0 Hz (-0.77%) vs PC pulling at 130.0 Hz over 120 s
    {
        PolarSampleBuffer buf;
        PmdEcgPacket pkt;
        pkt.valid = true;
        pkt.samples_uv.resize(73);
        int seq = 0;
        auto pushNext = [&]() {
            for (int i = 0; i < 73; ++i) pkt.samples_uv[i] = static_cast<float>(seq++);
            buf.pushEcg(pkt);
        };

        pushNext();
        pushNext();
        buf.prepareForStreaming();

        const double slow_burst_period = 73.0 / 129.0;
        double next_burst = slow_burst_period;
        double t = 0.0;
        for (int s = 0; s < 130 * 120; ++s) {
            t += 1.0 / 130.0;
            if (t >= next_burst) {
                pushNext();
                next_burst += slow_burst_period;
            }
            (void)buf.popEcgSamples(1);
        }

        const auto st = buf.ecgStats();
        CHECK(st.drift_interpolated_samples > 0);
        CHECK(st.underrun_samples == 0); // Soft interpolation prevented any flat-line plateau!
    }

    // 2. Fast Polar clock: Polar at 131.0 Hz (+0.77%) vs PC pulling at 130.0 Hz over 120 s
    {
        PolarSampleBuffer buf;
        PmdEcgPacket pkt;
        pkt.valid = true;
        pkt.samples_uv.resize(73);
        int seq = 0;
        auto pushNext = [&]() {
            for (int i = 0; i < 73; ++i) pkt.samples_uv[i] = static_cast<float>(seq++);
            buf.pushEcg(pkt);
        };

        pushNext();
        pushNext();
        pushNext();
        buf.prepareForStreaming();

        const double fast_burst_period = 73.0 / 131.0;
        double next_burst = fast_burst_period;
        double t = 0.0;
        for (int s = 0; s < 130 * 180; ++s) {
            t += 1.0 / 130.0;
            if (t >= next_burst) {
                pushNext();
                next_burst += fast_burst_period;
            }
            (void)buf.popEcgSamples(1);
        }

        const auto st = buf.ecgStats();
        CHECK(st.drift_dropped_samples > 0);
        CHECK(st.overflow_dropped_samples == 0); // Soft drift blend kept latency bounded without hard overflow!
        CHECK(st.queued_samples <= 310);
    }
}

void testPrepareForStreamingPreservesHrAndDetectsGapsAndWait() {
    PolarSampleBuffer buf(100);

    HeartRatePacket hr;
    hr.valid = true;
    hr.bpm = 68.0f;
    hr.contact_supported = true;
    hr.contact_detected = true;
    hr.rr_intervals_ms = { 880.0f };
    buf.pushHeartRate(hr);

    // Push two packets with a missing-packet timestamp gap (3x expected interval)
    PmdEcgPacket p1;
    p1.valid = true;
    p1.timestamp_ns = 1'000'000'000ULL;
    p1.samples_uv.assign(73, 10.0f);

    PmdEcgPacket p2;
    p2.valid = true;
    p2.timestamp_ns = 1'000'000'000ULL + 1'700'000'000ULL; // ~3x 561ms -> gap detected
    p2.samples_uv.assign(73, 20.0f);

    buf.pushEcg(p1);
    buf.pushEcg(p2);
    CHECK(buf.ecgStats().timestamp_gap_packets == 1);
    // Cold-priming transition trimmed 146 -> 100 samples automatically
    CHECK(buf.ecgStats().queued_samples == 100);
    CHECK(buf.isPrimed());

    buf.prepareForStreaming();
    CHECK(approx(buf.latestBpm(), 68.0f));
    CHECK(approx(buf.latestRrMs(), 880.0f));
    CHECK(approx(buf.latestContact(), 1.0f));
    CHECK(buf.ecgStats().queued_samples == 100);
    CHECK(buf.ecgStats().primed);

    // Pop 55 samples (leaving 45 samples < 100 prebuffer), then simulate rapid QTM Stop -> Start:
    // prepareForStreaming() must stay primed with 0 leading zeros!
    (void)buf.popEcgSamples(55);
    CHECK(buf.ecgStats().queued_samples == 45);
    buf.prepareForStreaming();
    CHECK(buf.isPrimed());
    CHECK(buf.ecgStats().queued_samples == 45);
    const auto first_after_restart = buf.popEcgSamples(1);
    CHECK(approx(first_after_restart[0], 20.0f)); // Non-zero live sample at t=0.000 s!

    // Test waitForEcg wakes asynchronously
    buf.resetConnectionState();
    CHECK(!buf.hasReceivedEcg());
    std::thread producer([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        buf.pushEcg(p1);
    });
    CHECK(buf.waitForEcg(1000, 1));
    producer.join();
    CHECK(buf.hasReceivedEcg());
}


void testGapFillReconstructsLostBlePacketAndRebufferAvoidsStutter() {
    // 1. Over-the-air lost packet BEFORE FIFO empties:
    //    Packet 1 ends at t = 1.000 s (samples 0..72, last = 100 uV).
    //    Packet 2 is lost over BLE (73 samples = 561,538,461 ns).
    //    Packet 3 arrives with timestamp t = 1.000 s + 2 * 561,538,462 ns (first = 246 uV).
    //    PolarSampleBuffer must detect the 1-packet gap and reconstruct the 73 missing samples
    //    so the ECG timeline is not compressed by 561 ms relative to QTM.
    {
        PolarSampleBuffer buf(100);
        const uint64_t dt_ns = 561'538'462ULL; // 73 / 130 s

        PmdEcgPacket p1;
        p1.valid = true;
        p1.timestamp_ns = 1'000'000'000ULL;
        p1.samples_uv.assign(73, 100.0f);
        buf.pushEcg(p1);

        PmdEcgPacket p2;
        p2.valid = true;
        p2.timestamp_ns = p1.timestamp_ns + dt_ns;
        p2.samples_uv.assign(73, 100.0f);
        buf.pushEcg(p2);
        buf.prepareForStreaming();
        // Pop 80 samples so FIFO has 20 left when the gap occurs
        (void)buf.popEcgSamples(80);
        CHECK(buf.ecgStats().queued_samples == 20);

        // Next packet skips ONE 73-sample notification (2 * dt_ns elapsed)
        PmdEcgPacket p4;
        p4.valid = true;
        p4.timestamp_ns = p2.timestamp_ns + 2 * dt_ns;
        p4.samples_uv.assign(73, 246.0f);
        buf.pushEcg(p4);

        const auto st = buf.ecgStats();
        CHECK(st.timestamp_gap_packets == 1);
        CHECK(st.gap_filled_samples == 73);
        // 20 remaining + 73 reconstructed + 73 from p4 = 166 queued (zero underrun!)
        CHECK(st.queued_samples == 166);
        CHECK(st.underrun_samples == 0);
        CHECK(std::fabs(st.effective_polar_hz - 130.0) < 0.05);
    }

    // 2. Long dropout that drains the FIFO -> underrun triggers a SINGLE re-prime hold
    //    (instead of a cascade of micro-plateaus on every subsequent burst), and the
    //    already-emitted hold samples are subtracted from gap_fill so no permanent latency is added.
    {
        PolarSampleBuffer buf(100);
        PmdEcgPacket pkt;
        pkt.valid = true;
        pkt.samples_uv.assign(73, 50.0f);
        buf.pushEcg(pkt);
        buf.pushEcg(pkt);
        buf.prepareForStreaming();

        // Drain all 100 samples + 1 soft low-watermark interpolation + 9 underrun samples -> enters rebuffering
        const auto drained = buf.popEcgSamples(110);
        CHECK(drained.size() == 110);
        CHECK(buf.ecgStats().drift_interpolated_samples == 1);
        CHECK(buf.ecgStats().underrun_samples == 9);

        // One 73-sample burst arrives (< 100 prebuffer): pop must KEEP holding during refill
        // instead of draining the 73 samples and underrunning again 561 ms later!
        pkt.samples_uv.assign(73, 120.0f);
        buf.pushEcg(pkt);
        const auto during_refill = buf.popEcgSamples(5);
        for (float v : during_refill) {
            CHECK(approx(v, 50.0f)); // still holding cleanly
        }
        CHECK(buf.ecgStats().queued_samples == 73); // untouched until >= 100

        // Second burst arrives (73 + 73 = 146 >= 100) -> exits rebuffering and resumes live ECG
        buf.pushEcg(pkt);
        const auto resumed = buf.popEcgSamples(5);
        for (float v : resumed) {
            CHECK(approx(v, 120.0f));
        }
    }

    // 3. resetEcgStreamState() preserves HR/RR/Contact while resetConnectionState() clears all
    {
        PolarSampleBuffer buf;
        HeartRatePacket hr;
        hr.valid = true;
        hr.bpm = 77.0f;
        hr.contact_detected = true;
        hr.rr_intervals_ms = { 780.0f };
        buf.pushHeartRate(hr);

        buf.resetEcgStreamState();
        CHECK(approx(buf.latestBpm(), 77.0f));
        CHECK(approx(buf.latestRrMs(), 780.0f));
        CHECK(approx(buf.latestContact(), 1.0f));

        buf.resetConnectionState();
        CHECK(approx(buf.latestBpm(), 0.0f));
        CHECK(approx(buf.latestRrMs(), 0.0f));
        CHECK(approx(buf.latestContact(), 0.0f));
    }
}


void testAccelerometerRawAndCompressedAndBattery() {
    // 1. Battery level (0x2A19)
    const uint8_t batt_ok[] = { 87 };
    CHECK(approx(parseBatteryLevelPercent(batt_ok, sizeof(batt_ok)), 87.0f));
    const uint8_t batt_over[] = { 120 };
    CHECK(approx(parseBatteryLevelPercent(batt_over, sizeof(batt_over)), 100.0f));
    CHECK(parseBatteryLevelPercent(nullptr, 0) < 0.0f);

    // 2. Raw TYPE_1 (frame_type = 0x01): 3 x 16-bit signed LE (mG)
    //    Sample 0: X = +100 mG, Y = -200 mG, Z = +1000 mG (1G)
    //    Sample 1: X = -500 mG, Y = +250 mG, Z = -980 mG
    std::vector<uint8_t> raw_t1 = {
        0x02, // ACC
        0x00, 0x10, 0x20, 0x30, 0x00, 0x00, 0x00, 0x00, // timestamp
        0x01, // raw TYPE_1 (3 x int16 LE)
    };
    auto pushInt16 = [&](int16_t v) {
        const uint16_t u = static_cast<uint16_t>(v);
        raw_t1.push_back(static_cast<uint8_t>(u & 0xFFu));
        raw_t1.push_back(static_cast<uint8_t>((u >> 8) & 0xFFu));
    };
    pushInt16(100);  pushInt16(-200); pushInt16(1000);
    pushInt16(-500); pushInt16(250);  pushInt16(-980);

    const auto parsed_raw = parsePmdAccFrame(raw_t1.data(), raw_t1.size());
    CHECK(parsed_raw.valid);
    CHECK(!parsed_raw.is_compressed);
    CHECK(parsed_raw.frame_type == 0x01);
    CHECK(parsed_raw.samples_mg.size() == 2);
    CHECK(approx(parsed_raw.samples_mg[0].x_mg, 100.0f));
    CHECK(approx(parsed_raw.samples_mg[0].y_mg, -200.0f));
    CHECK(approx(parsed_raw.samples_mg[0].z_mg, 1000.0f));
    CHECK(approx(parsed_raw.samples_mg[1].x_mg, -500.0f));
    CHECK(approx(parsed_raw.samples_mg[1].y_mg, 250.0f));
    CHECK(approx(parsed_raw.samples_mg[1].z_mg, -980.0f));

    // 3. Compressed TYPE_1 (frame_type = 0x81, Polar H10 default):
    //    Ref sample (3 x int16 LE): (10, -20, 1000) mG
    //    Block 1: delta_size = 6 bits, sample_count = 2 triplets:
    //      Triplet 1 deltas: (+5, -10, -15) -> (15, -30, 985)
    //      Triplet 2 deltas: (-20, +12, +8) -> (-5, -18, 993)
    std::vector<uint8_t> comp_t1 = {
        0x02, // ACC
        0x40, 0x50, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, // timestamp
        0x81, // compressed TYPE_1
    };
    auto pushRef16 = [&](int16_t v) {
        const uint16_t u = static_cast<uint16_t>(v);
        comp_t1.push_back(static_cast<uint8_t>(u & 0xFFu));
        comp_t1.push_back(static_cast<uint8_t>((u >> 8) & 0xFFu));
    };
    pushRef16(10); pushRef16(-20); pushRef16(1000);
    comp_t1.push_back(6); // delta_size_bits = 6
    comp_t1.push_back(2); // sample_count = 2 triplets (6 values * 6 bits = 36 bits -> 5 bytes)
    std::vector<uint8_t> packed;
    size_t bit_pos = 0;
    appendSignedBitsLsb(packed, bit_pos, +5, 6);
    appendSignedBitsLsb(packed, bit_pos, -10, 6);
    appendSignedBitsLsb(packed, bit_pos, -15, 6);
    appendSignedBitsLsb(packed, bit_pos, -20, 6);
    appendSignedBitsLsb(packed, bit_pos, +12, 6);
    appendSignedBitsLsb(packed, bit_pos, +8, 6);
    comp_t1.insert(comp_t1.end(), packed.begin(), packed.end());

    const auto parsed_comp = parsePmdAccFrame(comp_t1.data(), comp_t1.size());
    CHECK(parsed_comp.valid);
    CHECK(parsed_comp.is_compressed);
    CHECK(parsed_comp.frame_type == 0x81);
    CHECK(parsed_comp.samples_mg.size() == 3);
    CHECK(approx(parsed_comp.samples_mg[0].x_mg, 10.0f));
    CHECK(approx(parsed_comp.samples_mg[0].y_mg, -20.0f));
    CHECK(approx(parsed_comp.samples_mg[0].z_mg, 1000.0f));
    CHECK(approx(parsed_comp.samples_mg[1].x_mg, 15.0f));
    CHECK(approx(parsed_comp.samples_mg[1].y_mg, -30.0f));
    CHECK(approx(parsed_comp.samples_mg[1].z_mg, 985.0f));
    CHECK(approx(parsed_comp.samples_mg[2].x_mg, -5.0f));
    CHECK(approx(parsed_comp.samples_mg[2].y_mg, -18.0f));
    CHECK(approx(parsed_comp.samples_mg[2].z_mg, 993.0f));

    // 4. Parse ACC settings (25/50/100/200 Hz, 16-bit, 2/4/8 G, 3 channels) & build start command
    const uint8_t acc_settings_bytes[] = {
        0x00, 0x04, 25, 0, 50, 0, 100, 0, 200, 0, // SAMPLE_RATE
        0x01, 0x01, 16, 0,                        // RESOLUTION = 16
        0x02, 0x03, 2, 0, 4, 0, 8, 0,             // RANGE = 2, 4, 8 G
        0x04, 0x01, 3                             // CHANNELS = 3
    };
    const auto as = parsePmdAccSettings(acc_settings_bytes, sizeof(acc_settings_bytes));
    CHECK(as.valid);
    CHECK(as.sample_rates_hz.size() == 4 && as.sample_rates_hz.back() == 200);
    CHECK(as.resolutions_bits.size() == 1 && as.resolutions_bits[0] == 16);
    CHECK(as.ranges_g.size() == 3 && as.ranges_g.back() == 8);
    CHECK(as.channels.size() == 1 && as.channels[0] == 3);

    const auto start_acc = buildPmdStartAccCommand(200, 16, 8, true, 3);
    const std::vector<uint8_t> expected_acc_cmd = {
        0x02, 0x02,
        0x00, 0x01, 200, 0x00,
        0x01, 0x01, 16, 0x00,
        0x02, 0x01, 8, 0x00,
        0x04, 0x01, 3
    };
    CHECK(start_acc == expected_acc_cmd);

    // 5. Buffer co-streaming ECG (130 Hz) + ACC (200 Hz) resampled to unified 200 Hz board rate
    PolarSampleBuffer buf(100);
    buf.setAccNominalRateHz(200);
    buf.pushBatteryLevel(92.0f);
    CHECK(approx(buf.latestBatteryPct(), 92.0f));

    PmdEcgPacket ecg_pkt;
    ecg_pkt.valid = true;
    ecg_pkt.samples_uv.assign(130, 250.0f);
    buf.pushEcg(ecg_pkt);

    PmdAccPacket acc_pkt;
    acc_pkt.valid = true;
    acc_pkt.samples_mg.assign(250, { 10.0f, -20.0f, 981.0f });
    buf.pushAcc(acc_pkt);

    CHECK(buf.isPrimed());
    CHECK(buf.isAccPrimed());

    const auto ecg_200 = buf.popEcgSamplesResampled(20, 200.0);
    const auto acc_200 = buf.popAccSamplesResampled(20, 200.0);
    CHECK(ecg_200.size() == 20);
    CHECK(acc_200.x_mg.size() == 20);
    for (size_t i = 0; i < 20; ++i) {
        CHECK(approx(ecg_200[i], 250.0f));
        CHECK(approx(acc_200.x_mg[i], 10.0f));
        CHECK(approx(acc_200.y_mg[i], -20.0f));
        CHECK(approx(acc_200.z_mg[i], 981.0f));
    }

    // 6. Fast Polar H10 MEMS ACC clock (204.55 Hz real vs 200.0 Hz nominal = +2.27%) over 60 s:
    //    must keep FIFO strictly bounded with zero overflow and zero progressive lag!
    {
        PolarSampleBuffer acc_buf;
        acc_buf.setAccNominalRateHz(200);
        PmdAccPacket p;
        p.valid = true;
        p.samples_mg.assign(36, { 10.0f, 20.0f, 1000.0f });
        for (int k = 0; k < 6; ++k) acc_buf.pushAcc(p);
        acc_buf.prepareForStreaming();

        const double burst_period = 0.176; // 36 samples / 204.55 Hz
        double next_burst = burst_period;
        double t = 0.0;
        for (int s = 0; s < 200 * 60; ++s) {
            t += 1.0 / 200.0;
            if (t >= next_burst) {
                acc_buf.pushAcc(p);
                next_burst += burst_period;
            }
            (void)acc_buf.popAccSamples(1);
        }
        const auto st = acc_buf.accStats();
        CHECK(st.drift_dropped_samples > 200);
        CHECK(st.overflow_dropped_samples == 0);
        CHECK(st.queued_samples <= st.prebuffer_samples + 90);
    }
}

} // namespace

int main() {
    testHeartRateUint8WithContactAndRr();
    testHeartRateUint16WithEnergyExpended();
    testPmdEcgRawType0AndRejections();
    testPmdEcgCompressedDeltaType0();
    testPmdControlPointResponsesAndSettings();
    testSampleBufferBasicAndPrebuffer();
    testBurstArrivalWithJitterZeroUnderruns();
    testClockDriftCompensationSlowAndFastPolar();
    testPrepareForStreamingPreservesHrAndDetectsGapsAndWait();
    testGapFillReconstructsLostBlePacketAndRebufferAvoidsStutter();
    testAccelerometerRawAndCompressedAndBattery();
    std::cout << "All qdevice_polar_h10 unit tests passed.\n";
    return 0;
}
