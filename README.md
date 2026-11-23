# QDevice Polar H10

[![Technosport amU](https://img.shields.io/badge/Technosport-amU%20%7C%20ISM-002D62?style=flat-square)](https://github.com/Technosport)
[![QDevice](https://img.shields.io/badge/QTM-QDevice%200.10-009EE2?style=flat-square)](../../releases)
[![Windows](https://img.shields.io/badge/Windows-x64%20BLE-26A669?style=flat-square)](#)

> **Plugin QDevice C++20 / WinRT BLE intégrant les ceintures Polar H10 (fréquence cardiaque, intervalles RR, contact et ECG 130 Hz optionnel) en temps réel dans Qualisys Track Manager (QTM), sans SDK Polar payant.**  
> *Développé au sein du **Technosport / ISM** (UMR 7287 CNRS — Aix-Marseille Université) • [**Releases**](../../releases)*

---

## 🚀 Démarrage

1. Fermer **Qualisys Track Manager (QTM)**.
2. Télécharger et exécuter **`QDevice-Polar-H10-Setup-v0.2.0.exe`** depuis les **[Releases](../../releases)** *(installe `qdevice_polar_h10.dll` et `qdevice_polar_h10.png` dans `C:\Program Files\Qualisys\Qualisys Track Manager\Input_Devices\`)*.
3. Ouvrir **QTM > Project Options > Input Devices > Add Device...** et activer **Polar H10 Heart Rate Monitor**.
4. Porter la ceinture Polar H10, puis dans **Generics > Polar H10 Heart Rate Monitor** cliquer sur **Locate Devices** et **OK**.

---

## 🛠️ Fonctionnalités & Contenu

| Canal / Fichier | Rôle / Description |
| :--- | :--- |
| **`Heart Rate`** (`1 Hz`, `bpm`) | Fréquence cardiaque instantanée via GATT `0x180D` / `0x2A37` |
| **`RR Interval Ms`** (`1 Hz`, `ms`) | Dernier intervalle R-R mesuré par la ceinture |
| **`Sensor Contact`** (`1 Hz`, `0 / 1`) | État du contact peau des électrodes |
| **`ECG`** (`130 Hz`, `µV`, optionnel) | Flux ECG brut via Polar PMD (`Enable ECG`) |
| **`Device Filter`** | Filtre optionnel par ID Polar (ex. `0DA808`) ou adresse MAC |
| **`src/PolarBleClientWinrt.cpp`** | Client BLE WinRT sur threads MTA dédiés |
| **`src/QDevicePolarPlugin.cpp`** | Point d'entrée QDevice API `0.10.6` |

<details>
<summary><b>Compilation depuis les sources (développeurs)</b></summary>

> [!NOTE]
> Le **Qualisys QDevice SDK 0.10.x** n'est pas redistribué dans ce dépôt ; téléchargez-le depuis votre compte Qualisys.

```powershell
cmake -S . -B build -A x64 -DQDEVICE_SDK_DIR="C:\chemin\vers\qdevice_sdk"
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Pour générer l'installeur `.exe` avec Inno Setup 6 :

```powershell
& "$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe" .\installer\qdevice_polar_h10.iss
```

</details>
