# LibreVNA C++ Core + IPC

This folder hosts the refactored C++ sources and new headless entrypoints used by the 64-bit Python GUI.

## Targets

- `librevna-core` (static library): LibreVNA driver + calibration + GUI dependencies (unmodified sources).
- `librevna-cli` (console): headless sweep CLI that outputs JSON (used by Python CLI tooling).
- `librevna-ipc` (console): QtCore IPC server for the 64-bit Python GUI (QLocalServer JSON protocol).

## Build prerequisites (Windows)

- CMake 3.20+
- C++ toolchain (MSVC or MinGW 64-bit)
- Qt 6.x (Core, Widgets, Network, Gui, Svg, OpenGL)
- libusb-1.0 (x64) for USB access to LibreVNA

### libusb (vcpkg example)

```
powershell
cd C:\Users\<you>\vcpkg
.\vcpkg install libusb:x64-windows
```

## Configure + build

```
powershell
cmake -S scripts\vna\cpp -B scripts\vna\cpp\build -G Ninja `
  -DQt6_DIR="C:/Qt/6.9.2/msvc2019_64/lib/cmake/Qt6" `
  -DLIBUSB_INCLUDE_DIR="C:/Users/<you>/vcpkg/installed/x64-windows/include" `
  -DLIBUSB_LIBRARY="C:/Users/<you>/vcpkg/installed/x64-windows/lib/libusb-1.0.lib"

cmake --build scripts\vna\cpp\build --target librevna-ipc librevna-cli
```

## Runtime DLLs

`librevna-ipc.exe` and `librevna-cli.exe` require:

- Qt6Core.dll, Qt6Widgets.dll, Qt6Network.dll, Qt6Gui.dll, Qt6Svg.dll, Qt6OpenGL.dll
- libusb-1.0.dll (if built with libusb)
- MSVC runtime (or MinGW runtime DLLs if using MinGW)

Place the Qt + libusb DLLs next to the EXE or add their folders to PATH.

## Python GUI integration

In the 64-bit GUI (Hidden settings):

- **LibreVNA IPC binary**: point to `scripts\vna\cpp\build\librevna-ipc.exe`
- **LibreVNA IPC name**: leave as `librevna-ipc` unless you want a custom name

The GUI will launch the IPC server and use it for `list_devices` and `run_sweep`.

## Prebuilt GUI bundle

A prebuilt LibreVNA GUI (Qt + DLLs) is stored at:

`prebuilt\LibreVNA-GUI_Windows-v1.6.4`
