# LMX2594 Tools

Python tools for the LMX2594 synthesizer with a tinySA Ultra calibration GUI and CLI utilities.

## Hardware

- LMX2594 evaluation board (5V)
- USB2ANY programmer (SPI)
- 50 MHz reference (default)
- tinySA Ultra (optional, for calibration)

## Install

64-bit GUI environment:

```bash
uv python install 3.13.3
uv venv --python 3.13.3 .venv-x64
.\.venv-x64\Scripts\activate
pip install -r requirements.txt
```

32-bit worker environment (USB2ANY + LMX only):

```bash
uv python install cpython-3.10.11-windows-x86-none
uv venv --python cpython-3.10.11-windows-x86-none .venv-x86
.\.venv-x86\Scripts\activate
pip install -r requirements-x86.txt
```

Notes:

- `USB2ANY.dll` must stay in the repo root.
- tinySA Ultra must be in `CONFIG -> Connection -> USB` when using the USB cable.

## Requirements and Dependencies

Python dependencies:

- `requirements.txt` (64-bit GUI):
  - `pyserial>=3.5`
  - `tsapython`
  - `PySide6>=6.5` (preferred GUI backend)
  - `PyQt5>=5.15` (fallback GUI backend)
- `requirements-x86.txt` (32-bit worker):
  - `pyserial>=3.5`

VNA (LibreVNA IPC/CLI) build dependencies (`scripts/vna/cpp`):

- CMake `3.20+`
- C++17 toolchain (`MSVC x64` recommended; `MinGW64` supported)
- Qt 6.x modules: `Core`, `Widgets`, `Network`, `Gui`, `Svg`, `OpenGL`
- `libusb-1.0` (headers + library + runtime DLL)

Windows build prerequisites for VNA tools:

- Visual Studio Build Tools with "Desktop development with C++"
- Qt 6 MSVC kit matching your compiler (set `Qt6_DIR`)
- `libusb` (for example via `vcpkg install libusb:x64-windows`)

Example VNA configure/build:

```powershell
cmake -S scripts\vna\cpp -B scripts\vna\cpp\build -G Ninja `
  -DQt6_DIR="C:/Qt/6.6.3/msvc2022_64/lib/cmake/Qt6" `
  -DLIBUSB_INCLUDE_DIR="C:/Users/<you>/vcpkg/installed/x64-windows/include" `
  -DLIBUSB_LIBRARY="C:/Users/<you>/vcpkg/installed/x64-windows/lib/libusb-1.0.lib"

cmake --build scripts\vna\cpp\build --target librevna-ipc librevna-cli
```

VNA runtime requirements:

- `librevna-ipc.exe` and `librevna-cli.exe` need Qt runtime DLLs and (if enabled) `libusb-1.0.dll`
- Keep those DLLs on `PATH` or next to the executable
- `windeployqt` can be used to collect Qt runtime DLLs
- In GUI Preferences, set **LibreVNA IPC binary** to your built `librevna-ipc.exe`

Platform-specific USB notes:

- Linux: udev rule template is at `scripts/vna/cpp/LibreVNA-GUI_Source/51-vna.rules`
- Windows: if USB access fails with libusb, bind device to WinUSB (Zadig)

For full LibreVNA build/deploy details, see `scripts/vna/cpp/README.md`.

## Quick Start

GUI calibration (64-bit Python):

```bash
python scripts/lmx2594_calibration_gui.py
```

In the GUI, set the 32-bit worker Python path under Hidden settings (saved to `.lmx_gui_config.json`).

CLI frequency generator:

```bash
python scripts/frequency_generator.py --freq 3.6GHz
python scripts/frequency_generator.py --start-freq 3.2GHz --end-freq 3.6GHz --points 10 --dwell-ms 1000 --delta-update
```

tinySA automation (CLI):

```bash
python scripts/tinysa_antenna_test.py list-ports
python scripts/tinysa_antenna_test.py golden --start-freq 3.4GHz --stop-freq 3.6GHz --step 10MHz
```

## Troubleshooting

- Run scripts from the repo root: `python scripts/...`
- "No USB2ANY controllers found": check USB2ANY drivers/cable and SPI wiring.
- tinySA timeouts: confirm `CONFIG -> Connection -> USB`, then replug the device.

## Files

- `scripts/lmx2594_calibration_gui.py`: 64-bit GUI (tinySA + IPC to worker)
- `scripts/lmx2594_worker.py`: 32-bit USB2ANY/LMX worker
- `scripts/frequency_generator.py`: LMX2594 CLI generator
- `scripts/tinysa_antenna_test.py`: tinySA CLI automation
