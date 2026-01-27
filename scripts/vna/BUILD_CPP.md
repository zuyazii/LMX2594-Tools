# LibreVNA C++ Build (clean method)

This folder mirrors the original VNA-CLI sources. The C++ targets live in `scripts/vna/cpp`.

## Prereqs (Windows)

- CMake 3.16+
- C++17 toolchain (MSVC or MinGW)
- Qt 6 or 5 (Widgets + Charts; Core + Network for `librevna-ipc`)
- Optional: libusb-1.0 (for real LibreVNA USB driver)

## Clean build (recommended)

From the repo root:

```powershell
.\scripts\vna\cpp\build.ps1 -Clean
```

Outputs go to: `scripts/vna/cpp/build`

## Common options

```powershell
.\scripts\vna\cpp\build.ps1 -BuildType Debug
.\scripts\vna\cpp\build.ps1 -ConfigureOnly
.\scripts\vna\cpp\build.ps1 -BuildOnly
```

If Qt is not auto-detected, pass `Qt6_DIR` (or set `CMAKE_PREFIX_PATH`):

```powershell
.\scripts\vna\cpp\build.ps1 -QtDir "C:\Qt\6.6.2\msvc2019_64\lib\cmake\Qt6"
```

If libusb is installed in a custom location:

```powershell
.\scripts\vna\cpp\build.ps1 -LibUsbInclude "C:\libusb\include" -LibUsbLib "C:\libusb\lib\libusb-1.0.lib"
```

## Expected targets

- `librevna-cli` (headless CLI)
- `librevna-ipc` (Qt Core/Network IPC server)
- `vna-spoofer-gui` (Qt Widgets GUI)

If Qt Widgets/Charts are missing, the GUI target will be skipped. If Qt Core/Network are missing, the IPC target will be skipped.
