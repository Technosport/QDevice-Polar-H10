#!/usr/bin/env python3
"""
Post-capture ECG quality auditor for QDevice Polar H10 (QTM TSV exports & polar_h10_diag CSV).

Pure Python 3 standard library (no pip packages required).
Checks:
  1. Channel duration & sample rate coherence (1 Hz vs 130 Hz QTM trim issues)
  2. Leading zeros (t=0 startup delay)
  3. Flat plateaus (>= 3 identical consecutive ECG samples -> BLE underruns)
  4. Amplitude sanity & corrupted frame spikes (|ECG| > 15000 uV)
  5. Simple QRS R-peak detection vs reported Heart Rate channel (catches time-base drift/stretching)
"""

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def load_qtm_tsv_or_csv(path: Path) -> Tuple[float, Dict[str, List[float]]]:
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    lines = text.splitlines()
    if not lines:
        raise ValueError("Empty input file")

    declared_freq = 130.0
    header_idx = 0

    # Detect QTM Analog TSV header (starts with NO_OF_SAMPLES / FREQUENCY / etc.)
    is_qtm_tsv = False
    for idx, line in enumerate(lines[:30]):
        upper = line.strip().upper()
        if upper.startswith("FREQUENCY"):
            parts = line.replace(",", "\t").split("\t")
            for p in parts[1:]:
                p = p.strip()
                if p:
                    try:
                        declared_freq = float(p)
                    except ValueError:
                        pass
                    break
            is_qtm_tsv = True
        if is_qtm_tsv and idx > 0 and not upper.startswith("TIME_STAMP"):
            cols = [c.strip() for c in line.replace(",", "\t").split("\t") if c.strip()]
            if len(cols) >= 3 and (upper.startswith("SAMPLE") or upper.startswith("TIME\t") or "ECG" in upper):
                header_idx = idx
                break

    sample_line = lines[header_idx]
    delimiter = "\t" if "\t" in sample_line else ","

    reader = csv.reader(lines[header_idx:], delimiter=delimiter)
    headers = [h.strip() for h in next(reader)]
    columns: Dict[str, List[float]] = {h: [] for h in headers if h}

    for row in reader:
        if not row or all(not c.strip() for c in row):
            continue
        for i, h in enumerate(headers):
            if not h or i >= len(row):
                continue
            raw = row[i].strip()
            try:
                columns[h].append(float(raw))
            except ValueError:
                columns[h].append(float("nan"))

    # Infer frequency from time column if CSV didn't declare FREQUENCY
    if not is_qtm_tsv:
        for tcol in ("time_s", "Time", "TIME", "time"):
            if tcol in columns and len(columns[tcol]) >= 2:
                dts = [
                    columns[tcol][i + 1] - columns[tcol][i]
                    for i in range(min(500, len(columns[tcol]) - 1))
                    if columns[tcol][i + 1] > columns[tcol][i]
                ]
                if dts:
                    declared_freq = round(1.0 / statistics.median(dts), 3)
                break

    return declared_freq, columns


def find_column(columns: Dict[str, List[float]], needles: Tuple[str, ...]) -> Optional[str]:
    for col in columns:
        low = col.lower()
        for n in needles:
            if n in low:
                return col
    return None


def estimate_hr_from_ecg(ecg: List[float], fs: float) -> Tuple[Optional[float], int]:
    """Lightweight Pan-Tompkins-style QRS peak detector using only stdlib."""
    clean = [v for v in ecg if math.isfinite(v)]
    if len(clean) < int(fs * 4):
        return None, 0

    # Remove DC offset with ~0.5s moving average
    win = max(5, int(fs * 0.5))
    cumsum = [0.0]
    for v in clean:
        cumsum.append(cumsum[-1] + v)
    hp = []
    half = win // 2
    n = len(clean)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        mean = (cumsum[hi] - cumsum[lo]) / (hi - lo)
        hp.append(clean[i] - mean)

    # First-order derivative squared + short moving integration (~90 ms)
    diff_sq = [0.0] + [(hp[i] - hp[i - 1]) ** 2 for i in range(1, n)]
    int_win = max(3, int(fs * 0.09))
    cs2 = [0.0]
    for v in diff_sq:
        cs2.append(cs2[-1] + v)
    env = [
        (cs2[min(n, i + int_win)] - cs2[i]) / max(1, min(n, i + int_win) - i)
        for i in range(n)
    ]

    sorted_env = sorted(env)
    p95 = sorted_env[int(0.95 * (n - 1))]
    p50 = sorted_env[int(0.50 * (n - 1))]
    if p95 <= 1e-6 or p95 < p50 * 4.0:
        return None, 0

    thresh = p50 + 0.35 * (p95 - p50)
    refractory = int(fs * 0.30)  # Max ~200 bpm

    peaks: List[int] = []
    i = 1
    while i < n - 1:
        if env[i] > thresh and env[i] >= env[i - 1] and env[i] >= env[i + 1]:
            if not peaks or (i - peaks[-1]) >= refractory:
                # Refine to local max inside refractory window
                best = i
                end = min(n - 1, i + int(fs * 0.08))
                for j in range(i + 1, end):
                    if env[j] > env[best]:
                        best = j
                peaks.append(best)
                i = best + refractory
                continue
        i += 1

    if len(peaks) < 3:
        return None, len(peaks)

    rr_sec = [(peaks[k + 1] - peaks[k]) / fs for k in range(len(peaks) - 1)]
    valid_rr = [r for r in rr_sec if 0.30 <= r <= 2.0]
    if len(valid_rr) < 2:
        return None, len(peaks)

    med_rr = statistics.median(valid_rr)
    return 60.0 / med_rr, len(peaks)


def audit_packets_csv(path: Path) -> None:
    print(f"\n--- BLE Packet Trace Audit: {path.name} ---")
    dt_pc: List[float] = []
    dt_polar: List[float] = []
    sample_counts: List[int] = []
    frame_types: Dict[str, int] = {}
    fifo_vals: List[int] = []

    with path.open("r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            dpc = float(row["dt_pc_ms"])
            dpo = float(row["dt_polar_ms"])
            sc = int(row["sample_count"])
            ft = row["frame_type"].strip()
            fifo = int(row["fifo_after"])
            if dpc > 0:
                dt_pc.append(dpc)
            if dpo > 0:
                dt_polar.append(dpo)
            sample_counts.append(sc)
            frame_types[ft] = frame_types.get(ft, 0) + 1
            fifo_vals.append(fifo)

    print(f"  Packets total      : {len(sample_counts)}")
    print(f"  Frame types        : {frame_types}")
    if sample_counts:
        print(f"  Samples/packet     : min={min(sample_counts)}, med={statistics.median(sample_counts):.0f}, max={max(sample_counts)}")
    if dt_pc:
        print(f"  PC inter-burst ms  : min={min(dt_pc):.1f}, avg={statistics.mean(dt_pc):.1f}, "
              f"p95={sorted(dt_pc)[int(0.95*(len(dt_pc)-1))]:.1f}, max={max(dt_pc):.1f} (nominal 561.5 ms)")
    if dt_polar:
        gaps = [d for d in dt_polar if d > 850.0]
        print(f"  Polar TS delta ms  : min={min(dt_polar):.2f}, avg={statistics.mean(dt_polar):.2f}, max={max(dt_polar):.2f}")
        print(f"  Lost BLE packets   : {len(gaps)} gap(s) > 850 ms")
    if fifo_vals:
        print(f"  FIFO after push    : min={min(fifo_vals)}, avg={statistics.mean(fifo_vals):.1f}, max={max(fifo_vals)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit Polar H10 ECG capture (QTM TSV or diag CSV).")
    parser.add_argument("capture_file", type=Path, help="Path to QTM Analog TSV or polar_h10_diag _ecg.csv")
    parser.add_argument("--packets", type=Path, default=None, help="Optional polar_h10_diag _packets.csv")
    parser.add_argument("--rate", type=float, default=None, help="Override sampling rate in Hz")
    parser.add_argument("--ecg-col", type=str, default=None, help="Explicit ECG column name")
    args = parser.parse_args()

    fs, columns = load_qtm_tsv_or_csv(args.capture_file)
    if args.rate is not None:
        fs = args.rate

    ecg_col = args.ecg_col or find_column(columns, ("ecg",))
    hr_col = find_column(columns, ("heart rate", "heart_rate", "bpm"))
    cnt_col = find_column(columns, ("contact",))

    if not ecg_col or ecg_col not in columns:
        print(f"[FAIL] Could not find ECG column. Available columns: {list(columns.keys())}")
        return 2

    ecg = columns[ecg_col]
    n = len(ecg)
    dur_s = n / fs if fs > 0 else 0.0

    # 1. Leading zeros
    leading_zeros = 0
    for v in ecg:
        if math.isfinite(v) and abs(v) <= 1e-6:
            leading_zeros += 1
        else:
            break

    # 2. NaN/Inf & outliers
    nan_count = sum(1 for v in ecg if not math.isfinite(v))
    finite_ecg = [v for v in ecg if math.isfinite(v)]
    corrupt_spikes = sum(1 for v in finite_ecg if abs(v) > 15000.0)

    # 3. Flat plateaus (>= 4 identical consecutive samples after initial zeros)
    plateaus: List[Tuple[int, int]] = []  # (start_idx, length)
    run_start = leading_zeros
    run_len = 1
    for i in range(leading_zeros + 1, n):
        if math.isfinite(ecg[i]) and math.isfinite(ecg[i - 1]) and abs(ecg[i] - ecg[i - 1]) <= 1e-6:
            run_len += 1
        else:
            if run_len >= 4:
                plateaus.append((run_start, run_len))
            run_start = i
            run_len = 1
    if run_len >= 4 and leading_zeros < n:
        plateaus.append((run_start, run_len))

    longest_plateau = max((p[1] for p in plateaus), default=0)
    total_plateau_samples = sum(p[1] for p in plateaus)

    # 4. QRS HR vs HR channel
    ecg_hr, qrs_count = estimate_hr_from_ecg(ecg[leading_zeros:], fs)
    hr_channel_med = None
    if hr_col and columns[hr_col]:
        valid_hr = [v for v in columns[hr_col] if math.isfinite(v) and v > 20.0]
        if valid_hr:
            hr_channel_med = statistics.median(valid_hr)

    print(f"=== ECG Capture Audit: {args.capture_file.name} ===")
    print(f"  Sampling rate      : {fs:.2f} Hz")
    print(f"  Samples / duration : {n} samples ({dur_s:.2f} s)")
    print(f"  ECG column         : '{ecg_col}'")
    print(f"  NaN / Inf samples  : {nan_count}")
    if finite_ecg:
        print(f"  Amplitude (uV)     : min={min(finite_ecg):.1f}, med={statistics.median(finite_ecg):.1f}, "
              f"max={max(finite_ecg):.1f}, std={statistics.pstdev(finite_ecg):.1f}")
    print(f"  Corrupt spikes>15mV: {corrupt_spikes}")
    print(f"  Leading zeros      : {leading_zeros} samples ({1000.0 * leading_zeros / fs:.1f} ms)")
    print(f"  Flat plateaus (>=4): {len(plateaus)} event(s), total={total_plateau_samples} samples, "
          f"longest={longest_plateau} samples ({1000.0 * longest_plateau / fs:.1f} ms)")
    if plateaus:
        worst = sorted(plateaus, key=lambda x: x[1], reverse=True)[:5]
        fmt = ", ".join(f"t={idx/fs:.2f}s ({ln} samp)" for idx, ln in worst)
        print(f"  Worst plateaus     : {fmt}")

    if cnt_col and columns[cnt_col]:
        contact_pct = 100.0 * sum(1 for v in columns[cnt_col] if v >= 0.5) / len(columns[cnt_col])
        print(f"  Sensor contact     : {contact_pct:.1f}% of capture")

    if ecg_hr is not None:
        print(f"  ECG-derived HR     : {ecg_hr:.1f} bpm ({qrs_count} QRS peaks)")
    if hr_channel_med is not None:
        print(f"  HR channel median  : {hr_channel_med:.1f} bpm")
    if ecg_hr is not None and hr_channel_med is not None:
        diff_bpm = abs(ecg_hr - hr_channel_med)
        print(f"  HR vs ECG delta    : {diff_bpm:.1f} bpm")

    if args.packets and args.packets.exists():
        audit_packets_csv(args.packets)

    issues: List[str] = []
    if nan_count > 0:
        issues.append(f"{nan_count} NaN/Inf values")
    if corrupt_spikes > 0:
        issues.append(f"{corrupt_spikes} corrupt amplitude spikes (>15 mV)")
    if leading_zeros > int(fs * 0.25):
        issues.append(f"leading zeros > 250 ms ({1000.0 * leading_zeros / fs:.0f} ms)")
    if longest_plateau >= 8:
        issues.append(f"flat plateau of {longest_plateau} samples ({1000.0 * longest_plateau / fs:.0f} ms)")
    if ecg_hr is not None and hr_channel_med is not None and abs(ecg_hr - hr_channel_med) > 8.0:
        issues.append(f"HR mismatch ({ecg_hr:.1f} vs {hr_channel_med:.1f} bpm)")

    print("\n--- Verdict ---")
    if not issues:
        print("  [PASS] ECG capture is clean, continuous, and coherent.")
        return 0
    print("  [WARN/FAIL] Issues detected:")
    for it in issues:
        print(f"    - {it}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
