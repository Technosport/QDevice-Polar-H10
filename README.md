# QDevice Polar H10

Native **C++20 / WinRT BLE** QDevice plugin (`0.10.x` API) for **Qualisys Track Manager (QTM)** to stream **Polar H10** heart rate, R-R intervals, skin contact, and optional **130 Hz ECG** directly into QTM without any paid Polar SDK.

## Features

- **Direct Windows BLE (WinRT GATT)** on dedicated MTA worker threads (no conflict with QTM's STA UI thread)
- **Standard Heart Rate Service (`0x180D` / `0x2A37`)**:
  - `Heart Rate` (`bpm`, 1 Hz)
  - `RR Interval Ms` (`ms`, 1 Hz)
  - `Sensor Contact` (`0` / `1`, 1 Hz)
- **Optional 130 Hz ECG** via Polar PMD (`FB005C80-...` with `Indicate` on Control Point):
  - `ECG` (`µV`, 130 Hz)
- **Multi-strap support** + `Device Filter` (filter by Polar ID such as `0DA808` or Bluetooth MAC address)
- **Validated** against Qualisys `qdevice_validator.exe` (`23 run, 19 passed, 4 skipped`) and live QTM 2026.2

## Installation (Windows 10/11 x64)

1. Close **Qualisys Track Manager (QTM)**.
2. Download **`QDevice-Polar-H10-Setup-v0.2.0.exe`** from [Releases](https://github.com/Technosport/QDevice-polar-h10/releases) and run it (installs `qdevice_polar_h10.dll` and `qdevice_polar_h10.png` into `C:\Program Files\Qualisys\Qualisys Track Manager\Input_Devices\`).
   - *(Alternatively, download the `.zip` and copy both files manually into `Input_Devices\`.)*
3. Open **QTM > Project Options > Input Devices > Add Device...** and enable **Polar H10 Heart Rate Monitor**.

## Usage in QTM

1. Wear your Polar H10 strap so it starts advertising BLE.
2. In **Project Options > Input Devices > Generics > Polar H10 Heart Rate Monitor**:
   - Optionally enable **`Enable ECG`** (for 130 Hz ECG) or set **`Device Filter`** (e.g. your strap ID).
   - Click **`Locate Devices`**, then **`Synchronize Settings`** if you changed options, then **`OK`**.
3. Start **Preview** (`Ctrl+N`) or **Record** and open **View > Data Info > Analog data**.

## Building from Source

### Prerequisites
- Windows 10/11 x64
- Visual Studio 2022 / Build Tools (MSVC C++20 + Windows 10/11 SDK with `windowsapp.lib`)
- CMake >= 3.20
- **Qualisys QDevice SDK 0.10.x** (download from your [Qualisys account](https://www.qualisys.com/downloads/) — *not included in this repository per Qualisys SDK license*)

### Build
```powershell
cmake -S . -B build -A x64 -DQDEVICE_SDK_DIR="C:\path\to\qdevice_sdk"
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

### Build Installer
Requires [Inno Setup 6](https://jrsoftware.org/isinfo.php):
```powershell
& "$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe" .\installer\qdevice_polar_h10.iss
```
