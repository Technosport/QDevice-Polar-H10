# QDevice Polar H10

[![Technosport amU](https://img.shields.io/badge/Technosport-amU%20%7C%20ISM-002D62?style=flat-square)](https://github.com/Technosport)
[![QDevice](https://img.shields.io/badge/QTM-QDevice%200.10-009EE2?style=flat-square)](../../releases)
[![Windows](https://img.shields.io/badge/Windows-x64%20BLE-26A669?style=flat-square)](#)

> **Qualisys Track Manager integration plugin for Polar H10 sensors.**  
> *Developed at **Technosport / ISM** (UMR 7287 CNRS — Aix-Marseille Université) • [**Releases**](../../releases)*

---

## 🚀 Getting Started

1. Close **Qualisys Track Manager (QTM)**.
2. Download and run the latest installer from **[Releases](../../releases)**.
3. Open **QTM > Project Options > Input Devices > Add Device...** and enable **Polar H10 Heart Rate Monitor**.
4. Put on the Polar H10 strap, then go to **Generics > Polar H10 Heart Rate Monitor**, click **Locate Devices**, and click **OK**.

---

## 🛠️ Channels & Options

| Channel / Option | Description |
| :--- | :--- |
| **`Heart Rate`** (`1 Hz`, `bpm`) | Instantaneous heart rate (BLE GATT `0x180D` / `0x2A37`) |
| **`RR Interval Ms`** (`1 Hz`, `ms`) | Latest R-R interval |
| **`Sensor Contact`** (`1 Hz`, `0 / 1`) | Skin electrode contact status |
| **`ECG`** (`130 Hz`, `µV`, optional) | Raw ECG stream via Polar PMD (`Enable ECG`) |
| **`Device Filter`** | Optional filter by Polar ID or MAC address |
