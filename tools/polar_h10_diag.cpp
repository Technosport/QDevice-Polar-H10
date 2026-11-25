#include "qdevice_polar/PolarBleClient.hpp"
#include "qdevice_polar/PolarDiagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace qdevice_polar;

namespace {

struct DiagOptions {
    int duration_sec = 30;
    int scan_ms = 3000;
    int cycles = 1;
    int warmup_ms = 1200;
    size_t prebuffer_samples = kDefaultEcgPrebufferSamples;
    bool enable_ecg = true;
    bool enable_acc = false;
    uint16_t acc_rate_hz = 200;
    uint16_t acc_range_g = 8;
    bool show_help = false;
    std::string filter;
    std::string csv_prefix;
};

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  --duration <sec>     Capture duration per cycle in seconds (default: 30)\n"
        << "  --cycles <N>         Number of Connect -> Stream -> Disconnect cycles (default: 1)\n"
        << "  --scan-ms <ms>       BLE discovery duration in ms (default: 3000)\n"
        << "  --warmup-ms <ms>     Wait between connect and stream start in ms (default: 1200)\n"
        << "  --prebuffer <N>      ECG pre-buffer size in samples (default: " << kDefaultEcgPrebufferSamples << ")\n"
        << "  --filter <str>       Optional Polar device name or MAC substring\n"
        << "  --csv <prefix>       Write <prefix>_c<k>_ecg.csv and <prefix>_c<k>_packets.csv\n"
        << "  --hr-only            Disable ECG (test 1 Hz HR-only mode)\n"
        << "  --help               Show this help\n";
}

bool parseArgs(int argc, char** argv, DiagOptions& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto needVal = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            opt.show_help = true;
            return true;
        }
        try {
        if (arg == "--duration") {
            const char* v = needVal("--duration");
            if (!v) return false;
            opt.duration_sec = std::max(1, std::stoi(v));
        } else if (arg == "--cycles") {
            const char* v = needVal("--cycles");
            if (!v) return false;
            opt.cycles = std::max(1, std::stoi(v));
        } else if (arg == "--scan-ms") {
            const char* v = needVal("--scan-ms");
            if (!v) return false;
            opt.scan_ms = std::clamp(std::stoi(v), 500, 15000);
        } else if (arg == "--warmup-ms") {
            const char* v = needVal("--warmup-ms");
            if (!v) return false;
            opt.warmup_ms = std::max(0, std::stoi(v));
        } else if (arg == "--prebuffer") {
            const char* v = needVal("--prebuffer");
            if (!v) return false;
            opt.prebuffer_samples = static_cast<size_t>(std::max(1, std::stoi(v)));
        } else if (arg == "--filter") {
            const char* v = needVal("--filter");
            if (!v) return false;
            opt.filter = v;
        } else if (arg == "--csv") {
            const char* v = needVal("--csv");
            if (!v) return false;
            opt.csv_prefix = v;
        } else if (arg == "--acc") {
            opt.enable_acc = true;
        } else if (arg == "--acc-only") {
            opt.enable_ecg = false;
            opt.enable_acc = true;
        } else if (arg == "--acc-rate") {
            const char* v = needVal("--acc-rate");
            if (!v) return false;
            opt.acc_rate_hz = static_cast<uint16_t>(std::clamp(std::stoi(v), 25, 200));
        } else if (arg == "--acc-range") {
            const char* v = needVal("--acc-range");
            if (!v) return false;
            opt.acc_range_g = static_cast<uint16_t>(std::clamp(std::stoi(v), 2, 8));
        } else if (arg == "--hr-only") {
            opt.enable_ecg = false;
            opt.enable_acc = false;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
        } catch (const std::exception&) {
            std::cerr << "Invalid numeric value for " << arg << "\n";
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    DiagOptions opt;
    if (!parseArgs(argc, argv, opt)) {
        return 1;
    }
    if (opt.show_help) {
        return 0;
    }

    std::mutex console_mtx;
    setDiagnosticLogCallback([&](const std::string& line) {
        std::lock_guard<std::mutex> lock(console_mtx);
        std::cout << line << '\n';
    });

    std::cout << "=== QDevice Polar H10 BLE/PMD Diagnostic Tool (v0.2.9) ===\n";
    std::cout << "Mode: " << (opt.enable_ecg ? "HR + 130 Hz ECG" : "HR only (1 Hz)")
              << " | duration=" << opt.duration_sec << "s"
              << " | cycles=" << opt.cycles
              << " | prebuffer=" << opt.prebuffer_samples << " samples\n";

    std::cout << "\n[Scan] Discovering Polar BLE devices (" << opt.scan_ms << " ms)...\n";
    const auto devices = PolarBleClient::discoverAll(static_cast<uint32_t>(opt.scan_ms), opt.filter);
    if (devices.empty()) {
        std::cerr << "[FAIL] No Polar BLE devices found. Make sure the H10 strap is worn/awake and not locked by another app.\n";
        return 2;
    }

    for (size_t i = 0; i < devices.size(); ++i) {
        std::cout << "  [" << i << "] " << devices[i].name << " (" << devices[i].address_str << ")\n";
    }
    const auto target = devices.front();
    std::cout << "Using target: " << target.name << " (" << target.address_str << ")\n";

    int failures = 0;
    for (int cycle = 1; cycle <= opt.cycles; ++cycle) {
        std::cout << "\n---------------- Cycle " << cycle << "/" << opt.cycles << " ----------------\n";

        PolarBleClient client;
        client.buffer().setEcgPrebufferSamples(opt.prebuffer_samples);

        std::ofstream pkt_csv;
        std::mutex pkt_csv_mtx;
        if (!opt.csv_prefix.empty() && opt.enable_ecg) {
            const std::string pkt_path = opt.csv_prefix + "_c" + std::to_string(cycle) + "_packets.csv";
            pkt_csv.open(pkt_path);
            if (pkt_csv) {
                pkt_csv << "packet_idx,pc_elapsed_s,polar_ts_ns,dt_pc_ms,dt_polar_ms,frame_type,sample_count,fifo_after\n";
                client.buffer().setPacketTraceCallback([&](const EcgPacketTrace& tr) {
                    std::lock_guard<std::mutex> lock(pkt_csv_mtx);
                    pkt_csv << tr.packet_index << ','
                            << std::fixed << std::setprecision(4) << tr.pc_elapsed_s << ','
                            << tr.polar_timestamp_ns << ','
                            << std::setprecision(2) << tr.dt_pc_ms << ','
                            << tr.dt_polar_ms << ','
                            << "0x" << std::hex << static_cast<int>(tr.frame_type) << std::dec << ','
                            << tr.sample_count << ','
                            << tr.fifo_after << '\n';
                });
                std::cout << "Recording BLE packet trace to: " << pkt_path << "\n";
            }
        }

        const auto t_conn_start = std::chrono::steady_clock::now();
        const bool ok = client.connect(target.bluetooth_address, opt.enable_ecg, opt.enable_acc, opt.acc_rate_hz, opt.acc_range_g);
        const double conn_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_conn_start).count();

        if (!ok) {
            std::cerr << "[FAIL] Cycle " << cycle << " connect failed after "
                      << std::fixed << std::setprecision(0) << conn_ms
                      << " ms: " << client.lastError() << "\n";
            ++failures;
            continue;
        }

        std::cout << "[OK] Connected in " << std::fixed << std::setprecision(0) << conn_ms
                  << " ms | MTU=" << client.negotiatedMtu()
                  << " | ECG active=" << (client.isEcgActive() ? "yes" : "no") << "\n";

        if (opt.warmup_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(opt.warmup_ms));
        }

        client.buffer().prepareForStreaming();
        const auto start_stats = client.buffer().ecgStats();
        std::cout << "[Stream Start] primed=" << (start_stats.primed ? "yes" : "no")
                  << " initial_fifo=" << start_stats.queued_samples << "/" << start_stats.prebuffer_samples
                  << " HR=" << client.buffer().latestBpm() << " bpm\n";

        std::ofstream ecg_csv;
        if (!opt.csv_prefix.empty()) {
            const std::string ecg_path = opt.csv_prefix + "_c" + std::to_string(cycle) + "_ecg.csv";
            ecg_csv.open(ecg_path);
            if (ecg_csv) {
                ecg_csv << "sample_idx,time_s,heart_rate_bpm,rr_interval_ms,sensor_contact,ecg_uv\n";
                std::cout << "Recording QTM-simulated stream to: " << ecg_path << "\n";
            }
        }

        const double rate_hz = opt.enable_ecg ? 130.0 : (opt.enable_acc ? static_cast<double>(client.activeAccRateHz()) : 1.0);
        const double acc_rate_hz = static_cast<double>(client.activeAccRateHz());
        const auto t_stream_start = std::chrono::steady_clock::now();
        uint64_t samples_read = 0;
        uint64_t acc_samples_read = 0;
        int next_print_sec = 1;

        uint64_t leading_zeros = 0;
        bool seen_nonzero_ecg = false;
        uint64_t plateau_events = 0;
        uint64_t longest_plateau = 0;
        uint64_t current_run = 0;
        float prev_ecg = 0.0f;
        bool has_prev_ecg = false;

        std::cout << "  t(s) |  HR  |  RR(ms) | Cnt | FIFO | Pkts | PolarHz | Burst(min/avg/max ms) | Underrun | Drift(D/I) | Gaps\n";

        while (true) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed_s = std::chrono::duration<double>(now - t_stream_start).count();
            const auto target_samples = static_cast<uint64_t>(std::floor(std::max(0.0, elapsed_s * rate_hz)));

            if (target_samples > samples_read) {
                const size_t count = static_cast<size_t>(target_samples - samples_read);
                const float hr = client.buffer().latestBpm();
                const float rr = client.buffer().latestRrMs();
                const float cnt = client.buffer().latestContact();
                const auto ecg = opt.enable_ecg ? client.buffer().popEcgSamples(count) : std::vector<float>(count, 0.0f);
                if (opt.enable_acc) {
                    const auto target_acc = static_cast<uint64_t>(std::floor(std::max(0.0, elapsed_s * acc_rate_hz)));
                    if (target_acc > acc_samples_read) {
                        (void)client.buffer().popAccSamples(static_cast<size_t>(target_acc - acc_samples_read));
                        acc_samples_read = target_acc;
                    }
                }

                for (size_t i = 0; i < count; ++i) {
                    const uint64_t idx = samples_read + i;
                    const float v = ecg[i];
                    if (opt.enable_ecg) {
                        if (!seen_nonzero_ecg) {
                            if (std::fabs(v) > 1e-6f) {
                                seen_nonzero_ecg = true;
                            } else {
                                ++leading_zeros;
                            }
                        }
                        if (has_prev_ecg && std::fabs(v - prev_ecg) < 1e-6f) {
                            ++current_run;
                            if (current_run == 3) {
                                ++plateau_events;
                            }
                            if (current_run > longest_plateau) {
                                longest_plateau = current_run;
                            }
                        } else {
                            current_run = 1;
                            prev_ecg = v;
                            has_prev_ecg = true;
                        }
                    }
                    if (ecg_csv) {
                        ecg_csv << idx << ','
                                << std::fixed << std::setprecision(6) << (static_cast<double>(idx) / rate_hz) << ','
                                << std::setprecision(2) << hr << ','
                                << rr << ','
                                << cnt << ','
                                << v << '\n';
                    }
                }
                samples_read += count;
            }

            if (elapsed_s >= static_cast<double>(next_print_sec)) {
                const auto s = client.buffer().ecgStats();
                std::lock_guard<std::mutex> lock(console_mtx);
                std::cout << std::setw(5) << next_print_sec << "s | "
                          << std::setw(4) << std::fixed << std::setprecision(0) << client.buffer().latestBpm() << " | "
                          << std::setw(7) << std::setprecision(1) << client.buffer().latestRrMs() << " | "
                          << std::setw(3) << std::setprecision(0) << client.buffer().latestContact() << " | "
                          << std::setw(4) << s.queued_samples << " | "
                          << std::setw(4) << s.packets_received << " | "
                          << std::setw(7) << std::setprecision(2) << s.effective_polar_hz << " | "
                          << std::setw(6) << std::setprecision(1) << s.burst_interval_min_ms << "/"
                          << std::setw(6) << s.burst_interval_avg_ms << "/"
                          << std::setw(6) << s.burst_interval_max_ms << " | "
                          << std::setw(8) << s.underrun_samples << " | "
                          << std::setw(4) << s.drift_dropped_samples << "/"
                          << std::setw(4) << s.drift_interpolated_samples << " | "
                          << std::setw(4) << s.timestamp_gap_packets << "\n";
                ++next_print_sec;
            }

            if (elapsed_s >= static_cast<double>(opt.duration_sec)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }

        const auto end_stats = client.buffer().ecgStats();
        const uint16_t cycle_mtu = client.negotiatedMtu();
        const auto t_disc_start = std::chrono::steady_clock::now();
        client.buffer().setPacketTraceCallback(nullptr);
        client.disconnect();
        const double disc_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_disc_start).count();

        std::cout << "\n[Cycle " << cycle << " Summary]\n"
                  << "  Connect time     : " << std::fixed << std::setprecision(0) << conn_ms << " ms\n"
                  << "  Disconnect time  : " << disc_ms << " ms\n"
                  << "  Negotiated MTU   : " << cycle_mtu << " bytes\n"
                  << "  Emitted samples  : " << samples_read << " (" << std::setprecision(2)
                  << (static_cast<double>(samples_read) / rate_hz) << " s @ " << rate_hz << " Hz)\n";

        if (opt.enable_ecg) {
            std::cout << "  PMD frame_type   : 0x" << std::hex << static_cast<int>(end_stats.last_frame_type)
                      << std::dec << " (" << end_stats.last_packet_samples << " samples/pkt)\n"
                      << "  Polar real rate  : " << std::setprecision(3) << end_stats.effective_polar_hz << " Hz\n"
                      << "  Burst interval   : min=" << std::setprecision(1) << end_stats.burst_interval_min_ms
                      << " ms, avg=" << end_stats.burst_interval_avg_ms
                      << " ms, max=" << end_stats.burst_interval_max_ms << " ms\n"
                      << "  Leading zeros    : " << leading_zeros << " samples ("
                      << std::setprecision(1) << (1000.0 * static_cast<double>(leading_zeros) / 130.0) << " ms)\n"
                      << "  Underruns        : " << end_stats.underrun_samples << " samples\n"
                      << "  Overflows        : " << end_stats.overflow_dropped_samples << " samples\n"
                      << "  Drift (drop/int) : " << end_stats.drift_dropped_samples
                      << " / " << end_stats.drift_interpolated_samples << "\n"
                      << "  BLE TS gaps      : " << end_stats.timestamp_gap_packets << " packet(s)\n"
                      << "  Flat plateaus>=3 : " << plateau_events << " (longest=" << longest_plateau << " samples)\n";

            const bool cycle_ok = (end_stats.packets_received > 0) &&
                                  (end_stats.underrun_samples == 0) &&
                                  (end_stats.overflow_dropped_samples == 0) &&
                                  (end_stats.timestamp_gap_packets == 0);
            std::cout << "  Verdict          : " << (cycle_ok ? "PASS" : "WARN/CHECK") << "\n";
            if (end_stats.packets_received == 0) {
                ++failures;
            }
        }
    }

    setDiagnosticLogCallback(nullptr);
    return (failures == 0) ? 0 : 3;
}
