$ErrorActionPreference = "Stop"

$RootDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "[1/3] Configuring CMake for Windows (MSVC x64 + WinRT BLE)..."
cmake -S $RootDir -B (Join-Path $RootDir "build") -A x64

Write-Host "[2/3] Building qdevice_polar_h10.dll and unit tests..."
cmake --build (Join-Path $RootDir "build") --config Release

Write-Host "[3/3] Running unit tests..."
ctest --test-dir (Join-Path $RootDir "build") -C Release --output-on-failure

$DllPath = Join-Path $RootDir "build\Release\qdevice_polar_h10.dll"
Write-Host ""
Write-Host "Build complete: $DllPath"
Write-Host "Validate with Polar H10 paired/on:"
Write-Host "  ..\qdevice-ximu3\third_party\qdevice_sdk\validator\qdevice_validator.exe --lib `"$DllPath`""
