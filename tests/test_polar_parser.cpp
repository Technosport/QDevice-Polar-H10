#include "qdevice_polar/PolarPacketParser.hpp"
#include "qdevice_polar/PolarSampleBuffer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace qdevice_polar;

namespace {

bool approx(float a, float b, float eps = 1e-2f) {
    return std::fabs(a - b) <= eps;
}

void testHeartRateUint8WithContactAndRr() {
    // Flags = 0x16: bit0=0 (uint8), bit1-2=11 (contact supported+detected), bit3=0, bit4=1 (RR present)
    // HR = 72 bpm
    // RR = 853 (0x0355 -> LE: 0x55, 0x03) -> 853 * 1000 / 1024 = 833.0078 ms
    const uint8_t payload[] = { 0x16, 72, 0x55, 0x03 };
    const auto pkt = parseHeartRateMeasurement(payload, sizeof(payload));
    assert(pkt.valid);
    assert(approx(pkt.bpm, 72.0f));
    assert(pkt.contact_supported);
    assert(pkt.contact_detected);
    assert(pkt.rr_intervals_ms.size() == 1);
    assert(approx(pkt.rr_intervals_ms[0], 833.0078f, 0.05f));
}

void testHeartRateUint16WithEnergyExpended() {
    // Flags = 0x19: bit0=1 (uint16), bit3=1 (energy expended 2 bytes), bit4=1 (RR present)
    // HR = 260 (0x04, 0x01)
    // Energy = 500 kJ (0xF4, 0x01)
    // RR = 1024 (0x00, 0x04) -> 1000.0 ms
    const uint8_t payload[] = { 0x19, 0x04, 0x01, 0xF4, 0x01, 0x00, 0x04 };
    const auto pkt = parseHeartRateMeasurement(payload, sizeof(payload));
    assert(pkt.valid);
    assert(approx(pkt.bpm, 260.0f));
    assert(pkt.rr_intervals_ms.size() == 1);
    assert(approx(pkt.rr_intervals_ms[0], 1000.0f));
}

void testPmdEcg24BitSigned() {
    // Header: type=0x00, ts=0x0100000000000000, frame_type=0x00
    // Sample 0: +100 uV (0x64, 0x00, 0x00)
    // Sample 1: -100 uV (0x9C, 0xFF, 0xFF)
    const uint8_t frame[] = {
        0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x00,
        0x64, 0x00, 0x00,
        0x9C, 0xFF, 0xFF
    };
    const auto ecg = parsePmdEcgFrame(frame, sizeof(frame));
    assert(ecg.valid);
    assert(ecg.samples_uv.size() == 2);
    assert(approx(ecg.samples_uv[0], 100.0f));
    assert(approx(ecg.samples_uv[1], -100.0f));

    PolarSampleBuffer buf(2);
    // Before 2 samples arrive, popEcgSamples returns zeros
    const auto pre = buf.popEcgSamples(2);
    assert(approx(pre[0], 0.0f));

    buf.pushEcg(ecg);
    const auto popped = buf.popEcgSamples(3);
    assert(popped.size() == 3);
    assert(approx(popped[0], 100.0f));
    assert(approx(popped[1], -100.0f));
    assert(approx(popped[2], -100.0f)); // held last sample
}

} // namespace

int main() {
    testHeartRateUint8WithContactAndRr();
    testHeartRateUint16WithEnergyExpended();
    testPmdEcg24BitSigned();
    std::cout << "All qdevice_polar_h10 unit tests passed.\n";
    return 0;
}
