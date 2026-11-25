$ErrorActionPreference = "Stop"

$RootDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "[1/3] Configuring CMake for Windows (MSVC x64 + WinRT BLE)..."
cmake -S $RootDir -B (Join-Path $RootDir "build") -A x64

Write-Host "[2/3] Building qdevice_polar_h10.dll, polar_h10_diag.exe and unit tests..."
cmake --build (Join-Path $RootDir "build") --config Release

Write-Host "[3/3] Running unit tests..."
ctest --test-dir (Join-Path $RootDir "build") -C Release --output-on-failure

$DllPath  = Join-Path $RootDir "build\Release\qdevice_polar_h10.dll"
$DiagPath = Join-Path $RootDir "build\Release\polar_h10_diag.exe"

Write-Host ""
Write-Host "Build complete:"
Write-Host "  Plugin : $DllPath"
Write-Host "  Diag   : $DiagPath"
Write-Host ""
Write-Host "1) Standalone BLE/PMD ECG diagnostic & reconnect test (outside QTM):"
Write-Host "   & `"$DiagPath`" --duration 30 --cycles 2 --csv diag_run"
Write-Host "   python tools\audit_ecg_capture.py diag_run_c1_ecg.csv --packets diag_run_c1_packets.csv"
Write-Host ""
Write-Host "2) Validate QDevice plugin with qdevice_validator.exe (130 Hz ECG mode enabled):"
Write-Host "   `$env:QDEVICE_POLAR_LOG = '1'"
Write-Host "   `$env:QDEVICE_POLAR_ECG = '1'"
Write-Host "   ..\qdevice-ximu3\third_party\qdevice_sdk\validator\qdevice_validator.exe --lib `"$DllPath`""
Write-Host "   (Log file written to `$env:TEMP\qdevice_polar_h10.log)"
