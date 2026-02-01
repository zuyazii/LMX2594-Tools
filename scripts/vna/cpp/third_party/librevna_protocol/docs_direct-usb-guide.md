# LibreVNA Direct-USB Control (No GUI, No Qt)

This guide gives you a practical, end-to-end playbook to control a LibreVNA directly over USB using libusb:
- Instructions, rules, and workflows
- Protocol details (framing, packet types, CRC32)
- How to encode/decode packets
- How to parse VNADatapoint and compute S11 (magnitude/phase)
- A plan to port GUI calibration math (optional)
- Build/run skeleton (CMake), and Copilot/Cursor prompts to scaffold code

Works well with Cursor (VSCode-based). Windows/Linux/macOS.

---

## Goals

- Configure a one-shot S11 sweep (frequency range, points, IFBW, level) directly over USB.
- Stream VNADatapoint packets from the device and compute S11 per point.
- Optionally apply full VNA error-term calibration (OSL/SOLT/TRL) by porting GUI code.

---

## Prerequisites

- libusb (WinUSB backend on Windows) — recommended via vcpkg:
  - Windows: `vcpkg install libusb:x64-windows`
  - Linux: `vcpkg install libusb:x64-linux`
  - macOS: `vcpkg install libusb:x64-osx`
- Bind WinUSB to the LibreVNA device on Windows via Zadig (https://zadig.akeo.ie/):
  - Options → List All Devices → Select LibreVNA (VID:PID 0483:564E or 0483:4121 or 1209:4121) → Replace driver with WinUSB
- A local clone of LibreVNA to copy protocol files from (optional but strongly recommended).

---

## Rules and Best Practices

1. Only one client can claim the USB interface at a time. Ensure the GUI is closed when you run your tool.
2. Always use the official protocol encoder/decoder from LibreVNA (Protocol.hpp/.cpp + PacketConstants.h) to avoid bitfield and CRC mistakes.
3. Memory ownership: DecodeBuffer() allocates VNADatapoint on the heap. You must `delete` it after use.
4. Respect device limits (min/max frequency, max points, IFBW). Query them by sending RequestDeviceInfo and parsing DeviceInfo.
5. S11 for a single-port excitation is approximately S11 = P1 / Ref (complex ratio) for stage 0.
6. Start with instrument-level corrections only. If you need full OSL/SOLT/TRL calibration, port the GUI calibration code.
7. Handle NACKs and timeouts gracefully; retry or fail fast with useful diagnostics.
8. Keep buffers bounded; if your accumulator grows too large, clear it on desync.

---

## USB Protocol Overview

- Interfaces/Endpoints:
  - Bulk OUT 0x01 (PC → device) — commands/settings
  - Bulk IN  0x81 (device → PC) — data (VNADatapoint, status, acks)
  - Bulk IN  0x82 (device → PC) — logs (optional)

- Packet framing (little-endian):
  - Header: 0x5A (1 byte)
  - Length: total packet length incl. header and CRC (2 bytes LE)
  - Type: PacketType enum (1 byte)
  - Payload: variable
  - CRC32: over [header..payload] (4 bytes LE)

- Key PacketType values you’ll use most:
  - SweepSettings = 2
  - Ack = 7, Nack = 10
  - RequestDeviceInfo = 15, DeviceInfo = 5
  - DeviceStatus = 25
  - VNADatapoint = 27

Tip: These are defined in Protocol.hpp in LibreVNA’s firmware source.

---

## Encoding Packets

Use `EncodePacket(const PacketInfo&, uint8_t* dest, uint16_t destsize) -> uint16_t`:

Workflow:
1. Fill a `Protocol::PacketInfo` with `type` and the matching union fields.
2. Call `EncodePacket(...)` to get a framed buffer with header/length/type/payload/CRC.
3. Send via `libusb_bulk_transfer` to EP 0x01.

Example (conceptual) for SweepSettings:
- Set f_start/f_stop (Hz), points, IFBW (Hz)
- Set cdbm_excitation_start/stop in centi-dBm, e.g. -10 dBm → -1000
- For S11: one excited port (port 1) → stages = 0; port1Stage = 0; others “unused”
- logSweep (0/1), dwell_time (us) as needed
- syncMode (0 for internal ref), optional suppressPeaks/fixedPowerSetting

---

## Decoding Packets

Use `DecodeBuffer(uint8_t* buf, uint16_t len, PacketInfo* out) -> uint16_t`:

Workflow:
1. Append received bytes into an accumulator vector.
2. Call `DecodeBuffer` repeatedly; if it returns:
   - 0: need more bytes.
   - >0: it consumed N bytes from start; erase N bytes from accumulator and handle `out`.
3. Switch on `out.type`:
   - Ack/Nack: update state
   - DeviceStatus: optional status/temps/lock info
   - VNADatapoint: parse and compute S11; `delete out.VNAdatapoint` when done

---

## CRC32 Details

- CRC32 used is standard IEEE 802.3 (polynomial 0xEDB88320 in reflected form).
- `CRC32()` is provided with the protocol sources and used by the encoder/decoder.
- Always trust the provided `EncodePacket`/`DecodeBuffer` to do CRC work correctly.

---

## VNADatapoint Parsing and S11

VNADatapoint contains per-point receiver I/Q for several “sources” and stages:
- Sources: Port1, Port2, Reference (bitmask)
- Stages: correspond to excited ports; for S11 with excitedPorts={1}, there’s only stage 0.

S11 computation (single excited port):
- Extract complex P1 = I + jQ for stage 0, Source::Port1
- Extract complex Ref for stage 0, Source::Reference (often also flagged with Port1|Port2|Reference bitmask)
- S11 = P1 / Ref
- Magnitude (dB): 20 log10(|S11|)
- Phase (deg): atan2(Im, Re) × 180/π
- Frequency: dp->frequency (non-zero-span) or dp->us timestamp (zero-span)

You’ll need to consult Protocol.hpp for the exact VNADatapoint accessors or internal layout to iterate stage/source values.

---

## Device Limits (Recommended Flow)

- Before configuring a sweep, send `RequestDeviceInfo` and decode `DeviceInfo`:
  - min/max frequency
  - min/max IFBW
  - max points
  - min/max dBm
- Constrain your sweep settings to these limits.

---

## Calibration Porting (Optional)

Full error-term calibration (OSL/SOLT/TRL) is implemented in the GUI. To apply it headlessly:

- Copy and adapt:
  - `Software/PC_Application/LibreVNA-GUI/Calibration/calibration.h`
  - `Software/PC_Application/LibreVNA-GUI/Calibration/calibration.cpp`
  - `Software/PC_Application/LibreVNA-GUI/Calibration/calkit.*`
- Remove/replace Qt dialogs/signals (pure C++ wrappers).
- Load a `.cal` file or compute from captured standards.
- Convert your raw per-point result into the expected `VNAMeasurement`-like struct and call `Calibration::correctMeasurement(...)` to produce calibrated S-parameters.
- Start with deactivated calibration and add this only if required.

---

## Build/Run Skeleton (CMake + libusb)

- See [CMakeLists.txt](../CMakeLists.txt) and [src/main.cpp](../src/main.cpp) skeletons in this project.
- Copy the protocol files from LibreVNA into `third_party/librevna_protocol/`:
  - PacketConstants.h
  - Protocol.hpp
  - Protocol.cpp

Build:
```bash
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build
```

Run (Windows requires WinUSB bound with Zadig):

```bash
./build/direct_s11
```

---

## Test Plan (Minimal)

1. DeviceInfo sanity:
   - Send RequestDeviceInfo, verify reasonable ranges.
2. Single S11 sweep:
   - S11 over 2–3.5 GHz, 501 points, IFBW 1 kHz, -10 dBm.
   - Confirm point count and monotonically increasing frequencies.
3. Data integrity:
   - For a 50 Ω load, S11 magnitude ≈ near 0 dB? (reflection -∞ dB ideally; practical ~-20..-40 dB). Verify plausibility with known standards (open/short/load).
4. Stability:
   - Repeat sweeps with varying IFBW/points; check no buffer overrun or decode errors.
5. Calibration (optional):
   - Apply known calibration and measure a thru/known DUT to verify expected response.

---

## Common Errors and Remedies

- bulk OUT returns error:
  - Interface not claimed, wrong endpoint, or device is busy (GUI still running).
- No VNADatapoint packets:
  - SweepSettings invalid/out-of-range; send RequestDeviceInfo to constrain.
  - Stages/port mapping wrong; for S11 use one excited port, port1Stage=0, stages=0.
- Decoder never finds a complete packet:
  - Accumulator cleared too early or you’re not preserving partial bytes across reads.
  - Wrong CRC/type/length due to hand-rolled encoder. Use `EncodePacket`.
- Bad S11 values (NaN/Inf):
  - Ref too close to zero (divide-by-zero). Clamp or skip; verify IFBW and level.
  - No Reference channel captured → wrong “Source” selection.

---

## Prompts for Cursor/Copilot (Copy/Paste)

Use these targeted prompts to scaffold code rapidly.

1) Project bootstrap
- “Create a CMakeLists.txt for a C++17 console app named direct_s11 that finds libusb-1.0 and builds src/main.cpp, linking libusb.”

2) USB device open/close helpers
- “Implement a class USBDevice that opens a LibreVNA (VID:PID 0x0483:0x564E or 0x0483:0x4121 or 0x1209:0x4121) using libusb, claims interface 0, and exposes bulkRead/bulkWrite for EP 0x81/0x01.”

3) Protocol integration
- “Add third_party/librevna_protocol/PacketConstants.h, Protocol.hpp, Protocol.cpp from the LibreVNA repo and expose EncodePacket/DecodeBuffer/PacketInfo.”

4) SweepSettings builder
- “Write a function buildSweepSettingsS11(startHz, stopHz, points, ifbwHz, dBm) that returns a Protocol::PacketInfo pre-filled for S11 with excitedPorts={1}, stages=0, port1Stage=0, logSweep=false, dwell_time=0.”

5) Read/parse loop
- “Implement a receive loop that accumulates bytes from EP 0x81, repeatedly calls DecodeBuffer on the accumulator, handles Ack/Nack/DeviceStatus, and for VNADatapoint computes S11=P1/Ref at stage 0, printing freq, |S11| dB, angle deg.”

6) DeviceInfo query
- “Implement sendRequestDeviceInfo() and parse DeviceInfo to constrain sweep parameters.”

7) CSV export
- “After finishing a sweep, write frequency, |S11| dB, ∠S11 deg to results.csv with headers.”

8) Calibration (optional)
- “Port Calibration class from GUI: remove Qt dependencies, load a .cal file, and add a function correctVNAMeasurement(...) that transforms raw S-parameters into calibrated ones.”

9) Robustness
- “Add error handling: timeouts, NACKs, invalid packets, and graceful shutdown.”

---

## Security and Safety Notes

- Don’t send out-of-range RF power levels; respect device min/max dBm.
- Avoid long, continuous sweeps at high power to prevent DUT or device overheating.
- Validate all inputs and handle divide-by-zero when constructing ratios.

---

## Success Criteria

- A single executable that:
  - Enumerates/connects to LibreVNA over USB
  - Starts a one-shot S11 sweep
  - Prints frequency, magnitude (dB), and phase (deg) for each point
  - Optionally exports CSV
  - Exits cleanly

If you want a pre-baked `src/main.cpp` tuned to your desired frequency range/points, ask and I’ll generate it with the accessors matching the exact VNADatapoint layout from Protocol.hpp you import.