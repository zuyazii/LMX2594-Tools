# LibreVNA CLI & GUI Workspace

This repository packages two entry points for working with LibreVNA hardware:

- **Headless CLI (`cli.py` + `librevna-cli`)** – Drive the analyser from scripts, capture JSON/CSV traces, and automate calibration.
- **Qt GUI (`vna-spoofer-gui`)** – Visual front-end that wraps the same headless core with interactive charts and device management.

The project is designed for Windows developers who want to iterate on either interface while using the same C++/libusb back end.

---

## Repository Layout

| Path | Purpose |
| --- | --- |
| `cpp/` | CMake project that builds the native host core, CLI binary, and Qt GUI. |
| `cli.py` | Click-based wrapper that invokes the native CLI and formats the results. |
| `src/librevna/` | Python helpers (calibration, device adapters) used by the CLI. |
| `third_party/librevna_protocol/` | Protocol sources mirrored from the original LibreVNA project. |
| `builds/` | Out-of-source build trees (created by CMake). |
| `.vscode/` | Pre-configured VS Code settings for CMake, Qt, and debugging. |

---

## Prerequisites

| Tool | Notes |
| --- | --- |
| Windows 10 or newer | 64-bit required. |
| [Python 3.10+](https://www.python.org/downloads/windows/) | Used for the CLI wrapper and tooling. |
| [Git](https://git-scm.com/download/win) | Clone and update the repository. |
| [CMake ≥ 3.20](https://cmake.org/download/) | Configure and build the C++ targets. |
| [Ninja](https://ninja-build.org/) or MSBuild | CMake will default to Ninja with the supplied VS Code setup. |
| Qt 6.9 (MinGW 13.1 toolchain) | Required to build and run the Qt GUI (`vna-spoofer-gui`). Install via the Qt Online Installer. |
| [vcpkg](https://github.com/microsoft/vcpkg) | Used here to fetch `libusb`. Clone vcpkg somewhere on disk, e.g. `C:\Users\<you>\vcpkg`. |

---

## Quick Start

1. **Clone the repository**
   ```powershell
   git clone https://github.com/zuyazii/vna-spoofer.git
   cd vna-spoofer
   ```

2. **Install Python packages**
   ```powershell
   python -m venv .venv
   .\.venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. **Install libusb with vcpkg**
   ```powershell
   cd C:\Users\<you>\vcpkg
   .\vcpkg install libusb:x64-windows
   ```

   Keep the install prefix handy: vcpkg drops headers under `installed\x64-windows\include` and the import library under `installed\x64-windows\lib\libusb-1.0.lib`. The runtime DLL lives in `installed\x64-windows\bin`.

4. **Configure CMake (from the repo root)**
   ```powershell
   cmake -S . -B builds/Qt-6.9.2-mingw_64/Debug ^
     -DLIBUSB_INCLUDE_DIR="C:/Users/<you>/vcpkg/installed/x64-windows/include" ^
     -DLIBUSB_LIBRARY="C:/Users/<you>/vcpkg/installed/x64-windows/lib/libusb-1.0.lib"
   ```

   The `.vscode/settings.json` file mirrors these definitions so the CMake Tools extension will reuse them when you debug inside VS Code.

5. **Build the binaries**
   ```powershell
   cmake --build builds/Qt-6.9.2-mingw_64/Debug --target librevna-cli
   cmake --build builds/Qt-6.9.2-mingw_64/Debug --target vna-spoofer-gui
   ```

6. **Expose libusb at runtime**
   Add `C:\Users\<you>\vcpkg\installed\x64-windows\bin` to your PATH _or_ copy `libusb-1.0.dll` next to the generated executables (for the GUI this is typically `builds\Qt-6.9.2-mingw_64\Debug\vna-spoofer-gui.exe`).

---

## Binding the LibreVNA USB Driver (Windows)

LibreVNA devices must use the WinUSB driver so libusb can access them. Install the driver once per PC:

1. Download [Zadig](https://zadig.akeo.ie/).
2. Launch Zadig → **Options → List All Devices**.
3. Select the LibreVNA device (`VID:PID 0483:564E`, `0483:4121`, or `1209:4121`).
4. Choose **WinUSB (v6.x)** as the new driver and click **Replace Driver**.

After this, both the CLI and GUI will enumerate the analyser without further configuration.

---

## Running the Tools

### Headless CLI

```powershell
.\.venv\Scripts\activate
python cli.py --list                # enumerate connected devices
python cli.py headless-sweep `
  --start-freq 2.5 --stop-freq 6.0 `
  --points 801 --ifbw 1000 `
  --cal calibration\SOLT_12_1_00M-6_00G_500pt.cal
```

Frequencies are specified in GHz; outputs (JSON + CSV) are written into `output/`.

Set `LIBREVNA_CLI_BIN` if you want the Python wrapper to launch a non-default `librevna-cli.exe`.

### Qt GUI

Launch from VS Code using **Run ▸ Start Debugging** (uses the supplied launch configuration), or start the binary manually:

```powershell
set PATH=C:\Qt\6.9.2\mingw_64\bin;C:\Users\<you>\vcpkg\installed\x64-windows\bin;%PATH%
.\builds\Qt-6.9.2-mingw_64\Debug\vna-spoofer-gui.exe
```

The GUI exposes device scanning, live charts, and sweep controls. Use the chart toggles to hide/show magnitude or phase and the gestures (left-drag to pan, right-drag to zoom) to navigate traces.

---

## Recommended VS Code Setup

1. Open the repository folder in VS Code.
2. Install extensions:
   - *CMake Tools* (`ms-vscode.cmake-tools`)
   - *C/C++* (`ms-vscode.cpptools`)
   - *Qt* (`ms-vscode.qt-creator-cmake-tools`, optional but helpful)
3. Press `Ctrl+Shift+P → CMake: Configure`. The settings under `.vscode/` pre-fill the libusb paths and use `builds/Qt-6.9.2-mingw_64/<config>` as the build tree.
4. Use the status bar target picker to switch between `librevna-cli` and `vna-spoofer-gui` targets, or run the provided launch configuration for the GUI.

---

## Troubleshooting

- **CMake reports `libusb` not found** – Re-run configure with the correct `LIBUSB_INCLUDE_DIR` and `LIBUSB_LIBRARY` paths from your vcpkg installation. Make sure you installed the `x64-windows` triplet.
- **GUI starts but detects no devices** – Confirm the WinUSB driver is bound via Zadig and that `libusb-1.0.dll` is on your PATH. The device enumeration logic also requires the executable to be built with libusb support (`LIBREVNA_HEADLESS_HAS_REAL_DRIVER=1` is printed in the compile commands).
- **Linker cannot open `vna-spoofer-gui.exe`** – Close any running instance/debugger before rebuilding; Windows locks the output file while the process is active.
- **Qt DLL errors when launching manually** – Ensure the Qt `bin` directory for your chosen kit is on PATH (for MinGW 13.1 that is `C:\Qt\6.9.2\mingw_64\bin`). Alternatively, run the Qt-supplied `windeployqt` on the executable.

