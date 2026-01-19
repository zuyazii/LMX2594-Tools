#!/usr/bin/env python3
"""
tinySA + LMX2594 antenna test automation.

Implements:
- Serial port discovery
- tinySA connect/disconnect with error handling
- Single-point scan measurements
- Golden sample capture and test comparison
- CSV/JSON export of scan data
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import sys
import threading
import time
from typing import Callable, Dict, Iterable, List, Optional, Tuple


class TinySAError(RuntimeError):
    pass


class LMXError(RuntimeError):
    pass


def parse_frequency(freq_str: str) -> float:
    """Parse frequency string with optional units (Hz, kHz, MHz, GHz)."""
    freq_str = freq_str.strip().lower()

    if freq_str.endswith("ghz"):
        return float(freq_str[:-3]) * 1e9
    if freq_str.endswith("mhz"):
        return float(freq_str[:-3]) * 1e6
    if freq_str.endswith("khz"):
        return float(freq_str[:-3]) * 1e3
    if freq_str.endswith("hz"):
        return float(freq_str[:-2])
    return float(freq_str)


def build_frequency_points(start_hz: float, stop_hz: float, step_hz: Optional[float], points: Optional[int]) -> List[float]:
    if start_hz > stop_hz:
        raise ValueError("start frequency must be <= stop frequency")

    if step_hz is None and points is None:
        raise ValueError("Provide either --step or --points")

    if step_hz is not None and step_hz <= 0:
        raise ValueError("--step must be > 0")

    if points is not None and points <= 0:
        raise ValueError("--points must be > 0")

    if points is not None:
        if points == 1:
            return [start_hz]
        step = (stop_hz - start_hz) / (points - 1)
        return [start_hz + step * i for i in range(points)]

    # step_hz path
    points = int((stop_hz - start_hz) / step_hz) + 1
    return [start_hz + step_hz * i for i in range(points)]


def list_serial_ports() -> List[Tuple[str, str, str]]:
    try:
        import serial.tools.list_ports
    except ImportError as exc:
        raise TinySAError("pyserial is required for port discovery. Install with: pip install pyserial") from exc

    ports = []
    for port, desc, hwid in serial.tools.list_ports.comports():
        ports.append((port, desc, hwid))
    return ports


def _clean_scan_bytes(data_bytes: bytes) -> bytes:
    # Fix tinySA scan error tokens that look like "-:.0"
    return bytes(data_bytes).replace(b"-:.0", b"-10.0")


def parse_scan_values(data_bytes: bytes) -> List[float]:
    cleaned = _clean_scan_bytes(data_bytes)
    values: List[float] = []
    for raw_line in cleaned.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split()
        if not parts:
            continue
        try:
            values.append(float(parts[0]))
        except ValueError:
            continue
    return values


class TinySAController:
    def __init__(
        self,
        port: Optional[str],
        verbose: bool,
        error_byte: bool,
        baudrate: int = 115200,
        timeout_s: float = 1.0,
        connect_timeout_s: float = 6.0,
        command_timeout_s: float = 3.0,
        debug_cb: Optional[Callable[[str], None]] = None,
    ):
        try:
            from tsapython import tinySA
        except ImportError as exc:
            raise TinySAError(
                "tsapython is required. Install with: pip install tsapython"
            ) from exc

        self._tsa = tinySA()
        self._tsa.set_verbose(bool(verbose))
        self._tsa.set_error_byte_return(bool(error_byte))
        self._port = port
        self._connected = False
        self._baudrate = int(baudrate)
        self._timeout_s = float(timeout_s)
        self._connect_timeout_s = float(connect_timeout_s)
        self._command_timeout_s = float(command_timeout_s)
        self._debug_cb = debug_cb
        self._connect_cancelled = False

    def _log(self, message: str) -> None:
        if self._debug_cb:
            self._debug_cb(message)

    def _log_bytes(self, label: str, data_bytes: object, limit: int = 160) -> None:
        if not self._debug_cb:
            return
        if isinstance(data_bytes, (bytes, bytearray)):
            preview = bytes(data_bytes[:limit])
            self._log(f"{label} bytes={len(data_bytes)} preview={preview!r}")
        else:
            self._log(f"{label} value={data_bytes!r}")

    def _serial_command(
        self, command: str, timeout_s: Optional[float] = None, idle_s: float = 0.2
    ) -> bytes:
        ser = getattr(self._tsa, "ser", None)
        if not ser:
            raise TinySAError("tinySA serial handle not available")
        clean = command.strip().replace("\r", "\\r").replace("\n", "\\n")
        self._log(f"tinySA cmd -> {clean}")
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(command.encode("utf-8"))
        except Exception as exc:
            raise TinySAError(f"tinySA serial write failed: {exc}") from exc
        deadline = time.time() + float(timeout_s or self._command_timeout_s)
        last_data = time.time()
        buffer = bytearray()
        while time.time() < deadline:
            try:
                waiting = int(getattr(ser, "in_waiting", 0))
            except Exception:
                waiting = 0
            if waiting > 0:
                try:
                    chunk = ser.read(waiting)
                except Exception as exc:
                    raise TinySAError(f"tinySA serial read failed: {exc}") from exc
                if chunk:
                    buffer.extend(chunk)
                    last_data = time.time()
                    if b"ch>" in buffer:
                        break
            else:
                if time.time() - last_data >= idle_s:
                    break
                time.sleep(0.01)
        if time.time() >= deadline:
            self._log("tinySA cmd timed out waiting for prompt")
        self._log_bytes("tinySA cmd <-", buffer)
        return bytes(buffer)

    def _tsa_command(self, command: str, timeout_s: Optional[float] = None) -> bytes:
        return self._serial_command(command, timeout_s=timeout_s)

    def connect(self) -> None:
        ports_tried: List[str] = []
        self._log(f"tinySA connect start (port={self._port or 'Auto'})")
        if self._port:
            ports_tried.append(self._port)
            self._log(f"tinySA connect: attempting {self._port}")
            connected = self._connect_with_port_timeout(self._port)
        else:
            connected = self._autoconnect()
            if not connected:
                try:
                    ports = list_serial_ports()
                except TinySAError:
                    ports = []
                if ports:
                    port_list = ", ".join([p for p, _d, _h in ports])
                    self._log(f"tinySA connect: ports={port_list}")
                for port, _desc, _hwid in ports:
                    ports_tried.append(port)
                    self._log(f"tinySA connect: attempting {port}")
                    try:
                        connected = self._connect_with_port_timeout(port)
                    except TinySAError:
                        connected = False
                    if connected:
                        self._port = port
                        break

        if not connected:
            if ports_tried:
                port_list = ", ".join(ports_tried)
                raise TinySAError(f"Failed to connect to tinySA (ports tried: {port_list})")
            raise TinySAError("Failed to connect to tinySA (no serial ports found)")
        self._connected = True

    def _autoconnect(self) -> bool:
        result = self._tsa.autoconnect()
        self._log(f"tinySA autoconnect result={result!r}")
        if isinstance(result, tuple) and len(result) >= 2:
            connected = bool(result[1])
        else:
            connected = bool(result)
        if connected:
            self._apply_serial_settings()
            self._log("tinySA autoconnect succeeded")
        return connected

    def _connect_with_port(self, port: str) -> bool:
        self._log(f"tinySA connect: open {port} (timeout={self._timeout_s})")
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise TinySAError("pyserial is required for tinySA connection") from exc
        try:
            start = time.time()
            self._tsa.ser = serial.Serial(
                port=port,
                baudrate=self._baudrate,
                timeout=self._timeout_s,
                write_timeout=self._timeout_s,
            )
            elapsed = time.time() - start
            self._log(f"tinySA serial open elapsed={elapsed:.2f}s")
            try:
                self._tsa.ser.dtr = True
                self._tsa.ser.rts = False
            except Exception:
                pass
            try:
                self._tsa.ser.reset_input_buffer()
                self._tsa.ser.reset_output_buffer()
            except Exception:
                pass
            self._apply_serial_settings()
            if self._connect_cancelled:
                try:
                    self._tsa.ser.close()
                except Exception:
                    pass
                self._log("tinySA connect cancelled after open")
                return False
            self._log(f"tinySA serial open ok: {port}")
            return True
        except Exception as exc:
            raise TinySAError(f"tinySA serial open failed on {port}: {exc}") from exc

    def _connect_with_port_timeout(self, port: str) -> bool:
        result: Dict[str, object] = {"connected": False, "error": None}
        self._connect_cancelled = False

        def _attempt() -> None:
            try:
                result["connected"] = self._connect_with_port(port)
            except Exception as exc:
                result["error"] = exc

        thread = threading.Thread(target=_attempt, daemon=True)
        thread.start()
        thread.join(self._connect_timeout_s)
        if thread.is_alive():
            self._log(f"tinySA connect timed out: {port}")
            self._connect_cancelled = True
            ser = getattr(self._tsa, "ser", None)
            if ser:
                try:
                    ser.close()
                    self._log("tinySA serial handle closed after timeout")
                except Exception as exc:
                    self._log(f"tinySA serial close failed after timeout: {exc}")
            raise TinySAError(f"tinySA connect timed out on {port}")
        if result["error"]:
            err = result["error"]
            if isinstance(err, TinySAError):
                raise err
            raise TinySAError(str(err))
        return bool(result["connected"])

    def disconnect(self) -> None:
        if self._connected:
            try:
                self._tsa.disconnect()
            finally:
                self._connected = False
                self._log("tinySA disconnected")

    def resume(self) -> None:
        if hasattr(self._tsa, "resume"):
            self._tsa.resume()
            self._log("tinySA resume")

    def _apply_serial_settings(self) -> None:
        ser = getattr(self._tsa, "ser", None)
        if not ser:
            self._log("tinySA serial settings: no serial handle")
            return
        try:
            ser.baudrate = self._baudrate
            ser.timeout = self._timeout_s
        except Exception:
            pass
        self._log(f"tinySA serial settings: baud={self._baudrate} timeout={self._timeout_s}")

    def send_raw(self, command: str) -> bytes:
        return self._tsa_command(command)

    def scan_point(self, freq_hz: float, outmask: int = 2, timeout_s: float = 5.0, retries: int = 1) -> float:
        last_exc: Optional[Exception] = None
        for _attempt in range(max(0, retries) + 1):
            try:
                return self._scan_point_with_timeout(freq_hz, outmask, timeout_s)
            except TinySAError as exc:
                last_exc = exc
                # Attempt to recover from timeouts by reconnecting
                if "timed out" in str(exc).lower():
                    try:
                        self.disconnect()
                    except Exception:
                        pass
                    try:
                        self.connect()
                    except Exception as reconnect_exc:
                        last_exc = reconnect_exc
                        break
                time.sleep(0.1)
        raise TinySAError(str(last_exc))

    def _scan_point_with_timeout(self, freq_hz: float, outmask: int, timeout_s: float) -> float:
        result: Dict[str, object] = {"value": None, "error": None}
        self._log(f"tinySA scan start: freq={freq_hz} timeout={timeout_s}s")

        def _run() -> None:
            try:
                result["value"] = self._scan_point_once(freq_hz, outmask, timeout_s)
            except Exception as exc:
                result["error"] = exc

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()
        thread.join(max(0.1, float(timeout_s)))
        if thread.is_alive():
            self._log("tinySA scan timed out")
            try:
                self._tsa.disconnect()
            except Exception:
                pass
            raise TinySAError("tinySA scan timed out")
        if result["error"]:
            err = result["error"]
            if isinstance(err, TinySAError):
                raise err
            raise TinySAError(str(err))
        return float(result["value"])

    def _scan_point_once(self, freq_hz: float, outmask: int, timeout_s: float) -> float:
        cmd_timeout = max(1.0, float(timeout_s) * 0.8)
        self._ensure_input_mode()
        hop_value = self._hop_measure(freq_hz, cmd_timeout)
        if hop_value is not None:
            self._log(f"tinySA hop result: {hop_value:.2f} dBm")
            return hop_value

        # Use a minimal span to satisfy start < stop requirement.
        start_hz = max(0.0, freq_hz - 1.0)
        stop_hz = freq_hz + 1.0
        self._log(f"tinySA scan: start={start_hz} stop={stop_hz} outmask={outmask}")
        cmd = f"scan {int(start_hz)} {int(stop_hz)} 1 {int(outmask)}\r\n"
        data_bytes = self._serial_command(cmd, timeout_s=cmd_timeout)
        self._log_bytes("tinySA scan raw", data_bytes)
        if not data_bytes:
            raise TinySAError("tinySA scan returned no data")
        if isinstance(data_bytes, (bytes, bytearray)) and data_bytes.strip() in (b"ERROR", b"ERR"):
            raise TinySAError("tinySA scan returned ERROR")

        values = parse_scan_values(data_bytes)
        self._log(f"tinySA scan parsed values={len(values)}")
        if not values:
            self._log_bytes("tinySA scan raw (no parseable values)", data_bytes)
            raise TinySAError("tinySA scan returned no parseable values")

        self.resume()
        return values[0]

    def _ensure_input_mode(self) -> None:
        if hasattr(self._tsa, "tinySA_serial"):
            try:
                self._log("tinySA set mode: low input")
                self._tsa_command("mode low input\r\n")
            except Exception as exc:
                self._log(f"tinySA set mode failed: {exc}")

    def _hop_measure(self, freq_hz: float, timeout_s: float) -> Optional[float]:
        if not hasattr(self._tsa, "tinySA_serial"):
            return None
        try:
            self._log(f"tinySA hop: freq={freq_hz}")
            data_bytes = self._serial_command(f"hop {int(freq_hz)} 2\r\n", timeout_s=timeout_s)
        except Exception:
            return None
        if not data_bytes:
            return None
        values = parse_scan_values(data_bytes)
        if not values:
            return None
        return float(values[0])


def parse_register_values(path: str) -> Tuple[List[Tuple[int, int]], Optional[int], Dict[int, int]]:
    """
    Parse register values file with lines like: R112 0x700000
    Returns (register_list, r0_value, reg_map)
    """
    import re

    register_list: List[Tuple[int, int]] = []
    reg_map: Dict[int, int] = {}
    r0_value: Optional[int] = None

    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            match = re.match(r"^R(\d+)\s+0x([0-9A-Fa-f]+)$", line)
            if not match:
                continue
            addr = int(match.group(1))
            full = int(match.group(2), 16)
            data = full & 0xFFFF
            register_list.append((addr, data))
            reg_map[addr] = data
            if addr == 0:
                r0_value = data

    if not register_list:
        raise ValueError(f"No register values found in {path}")

    return register_list, r0_value, reg_map


def reg_map_to_list(reg_map: Dict[int, int]) -> List[Tuple[int, int]]:
    return [(addr, data & 0xFFFF) for addr, data in reg_map.items()]


class LMXController:
    def __init__(
        self,
        serial_number: Optional[str],
        f_osc: float,
        template_path: str,
        spi_clock: float,
        debug: bool,
        delta_update: bool,
        outa_pwr: int = 50,
    ):
        try:
            from usb2anyapi import USB2ANYInterface
            from lmx2594 import LMX2594Driver, build_registers_from_template
        except ImportError as exc:
            raise LMXError("LMX2594 scripts not found. Run from project root.") from exc

        self._USB2ANYInterface = USB2ANYInterface
        self._LMX2594Driver = LMX2594Driver
        self._build_registers_from_template = build_registers_from_template
        self._serial_number = serial_number
        self._f_osc = f_osc
        self._template_path = template_path
        self._spi_clock = spi_clock
        self._debug = debug
        self._delta_update = delta_update
        self._outa_pwr = outa_pwr

        self._usb2any = None
        self._lmx = None
        self._template_map: Dict[int, int] = {}
        self._r0_value: Optional[int] = None

    def connect(self) -> None:
        if not os.path.exists(self._template_path):
            raise LMXError(f"LMX template file not found: {self._template_path}")

        _, r0_value, template_map = parse_register_values(self._template_path)
        self._template_map = template_map
        self._r0_value = r0_value

        self._usb2any = self._USB2ANYInterface(
            self._serial_number, debug=self._debug, clock_freq=self._spi_clock
        )
        self._usb2any.connect()
        self._lmx = self._LMX2594Driver(
            self._usb2any, readback_enabled=False, debug=self._debug, f_osc=self._f_osc
        )

    def disconnect(self) -> None:
        if self._usb2any:
            try:
                self._usb2any.disconnect()
            finally:
                self._usb2any = None
                self._lmx = None

    def program_initial(self, freq_hz: float) -> Dict[int, int]:
        if not self._lmx:
            raise LMXError("LMX not connected")

        reg_map, _plan = self._build_registers_from_template(
            freq_hz, self._f_osc, self._template_map, outa_pwr=self._outa_pwr
        )
        register_list = reg_map_to_list(reg_map)
        self._lmx.program_registers(register_list, r0_value=self._r0_value)
        self._lmx.configure_muxout_lock_detect()
        return reg_map

    def update_frequency(self, freq_hz: float, prev_reg_map: Optional[Dict[int, int]]) -> Dict[int, int]:
        if not self._lmx:
            raise LMXError("LMX not connected")

        reg_map, _plan = self._build_registers_from_template(
            freq_hz, self._f_osc, self._template_map, outa_pwr=self._outa_pwr
        )
        if self._delta_update and prev_reg_map is not None:
            update_addrs = (31, 34, 36, 38, 39, 42, 43, 44, 75)
            write_addrs = [addr for addr in update_addrs if reg_map.get(addr) != prev_reg_map.get(addr)]
            self._lmx.fast_update_frequency(reg_map, write_addrs=write_addrs)
        else:
            self._lmx.fast_update_frequency(reg_map)
        return reg_map


def write_csv(path: str, rows: Iterable[List[object]], header: List[str]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


def write_json(path: str, payload: Dict[str, object]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


def measure_band(
    freqs: List[float],
    tinysa: TinySAController,
    lmx: Optional[LMXController],
    settle_s: float,
    outmask: int,
) -> List[Dict[str, float]]:
    results: List[Dict[str, float]] = []
    reg_map: Optional[Dict[int, int]] = None

    for idx, freq in enumerate(freqs):
        if lmx:
            if idx == 0:
                reg_map = lmx.program_initial(freq)
            else:
                reg_map = lmx.update_frequency(freq, reg_map)

        if settle_s > 0:
            time.sleep(settle_s)

        dbm = tinysa.scan_point(freq, outmask=outmask)
        results.append({"frequency_hz": float(freq), "dbm": float(dbm)})

    return results


def load_golden(path: str) -> Dict[str, object]:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="tinySA + LMX2594 antenna test automation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list-ports", help="List serial ports")

    golden = subparsers.add_parser("golden", help="Capture golden sample data")
    test = subparsers.add_parser("test", help="Test antenna against golden sample")
    direct = subparsers.add_parser("direct", help="Direct LMX->tinySA sweep capture (no pass/fail)")

    for sub in (golden, test, direct):
        sub.add_argument("--port", help="tinySA serial port (e.g., COM4)")
        sub.add_argument("--tinysa-verbose", action="store_true", help="Enable verbose tinySA output")
        sub.add_argument("--tinysa-error-byte", action="store_true", help="Return b'ERROR' on tinySA errors")
        sub.add_argument("--outmask", type=int, default=2, help="tinySA scan outmask (default: 2)")
        sub.add_argument("--settle-us", type=float, default=1000.0, help="LMX settle delay in microseconds")
        sub.add_argument("--no-lmx", action="store_true", help="Skip LMX control (external signal source)")
        sub.add_argument("--lmx-serial", help="USB2ANY serial number")
        sub.add_argument("--lmx-fosc", default="50MHz", help="LMX reference oscillator (default: 50MHz)")
        sub.add_argument("--lmx-template", default=os.path.join("examples", "register-values", "HexRegisterValues3600.txt"))
        sub.add_argument("--lmx-spi-clock", default="400kHz", help="USB2ANY SPI clock (default: 400kHz)")
        sub.add_argument("--lmx-debug", action="store_true", help="Enable LMX debug output")
        sub.add_argument("--delta-update", action="store_true", help="Only write changed LMX registers between points")

    for sub in (golden, direct):
        sub.add_argument("--start-freq", required=True, help="Start frequency (e.g., 3.4GHz)")
        sub.add_argument("--stop-freq", required=True, help="Stop frequency (e.g., 3.6GHz)")
        group = sub.add_mutually_exclusive_group(required=False)
        group.add_argument("--step", default="10MHz", help="Step frequency (default: 10MHz)")
        group.add_argument("--points", type=int, help="Number of points (inclusive)")
        sub.add_argument("--label", default="golden-sample", help="Label for this capture")
        if sub is golden:
            sub.add_argument("--out-json", default=os.path.join("output", "golden_sample.json"))
            sub.add_argument("--out-csv", default=os.path.join("output", "golden_sample.csv"))
        else:
            sub.add_argument("--out-json", default=os.path.join("output", "direct_capture.json"))
            sub.add_argument("--out-csv", default=os.path.join("output", "direct_capture.csv"))

    test.add_argument("--golden-json", required=True, help="Golden sample JSON file")
    test.add_argument("--tolerance-db", type=float, default=5.0, help="Allowed loss beyond golden sample (dB)")
    test.add_argument("--out-json", default=os.path.join("output", "test_result.json"))
    test.add_argument("--out-csv", default=os.path.join("output", "test_result.csv"))

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "list-ports":
        try:
            ports = list_serial_ports()
        except TinySAError as exc:
            print(f"ERROR: {exc}")
            return 1
        if not ports:
            print("No serial ports found.")
            return 0
        for port, desc, hwid in ports:
            print(f"{port}: {desc} ({hwid})")
        return 0

    tinysa = TinySAController(args.port, args.tinysa_verbose, args.tinysa_error_byte)
    lmx = None
    settle_s = max(0.0, args.settle_us / 1e6)

    try:
        tinysa.connect()

        if not args.no_lmx:
            f_osc = parse_frequency(args.lmx_fosc)
            spi_clock = parse_frequency(args.lmx_spi_clock)
            lmx = LMXController(
                args.lmx_serial,
                f_osc=f_osc,
                template_path=args.lmx_template,
                spi_clock=spi_clock,
                debug=args.lmx_debug,
                delta_update=args.delta_update,
            )
            lmx.connect()

        if args.command == "golden":
            start_hz = parse_frequency(args.start_freq)
            stop_hz = parse_frequency(args.stop_freq)
            step_hz = parse_frequency(args.step) if args.step and args.points is None else None
            points = args.points
            freqs = build_frequency_points(start_hz, stop_hz, step_hz, points)

            measurements = measure_band(
                freqs=freqs,
                tinysa=tinysa,
                lmx=lmx,
                settle_s=settle_s,
                outmask=args.outmask,
            )

            payload = {
                "metadata": {
                    "type": "golden",
                    "label": args.label,
                    "created_at": dt.datetime.utcnow().isoformat() + "Z",
                    "start_hz": start_hz,
                    "stop_hz": stop_hz,
                    "step_hz": step_hz,
                    "points": len(freqs),
                    "outmask": args.outmask,
                },
                "measurements": measurements,
            }

            write_json(args.out_json, payload)
            write_csv(
                args.out_csv,
                [[m["frequency_hz"], m["dbm"]] for m in measurements],
                header=["Frequency_Hz", "Signal_dBm"],
            )

            print(f"Golden sample saved to {args.out_json} and {args.out_csv}")
            return 0

        if args.command == "direct":
            start_hz = parse_frequency(args.start_freq)
            stop_hz = parse_frequency(args.stop_freq)
            step_hz = parse_frequency(args.step) if args.step and args.points is None else None
            points = args.points
            freqs = build_frequency_points(start_hz, stop_hz, step_hz, points)

            measurements = measure_band(
                freqs=freqs,
                tinysa=tinysa,
                lmx=lmx,
                settle_s=settle_s,
                outmask=args.outmask,
            )

            payload = {
                "metadata": {
                    "type": "direct",
                    "label": args.label,
                    "created_at": dt.datetime.utcnow().isoformat() + "Z",
                    "start_hz": start_hz,
                    "stop_hz": stop_hz,
                    "step_hz": step_hz,
                    "points": len(freqs),
                    "outmask": args.outmask,
                },
                "measurements": measurements,
            }

            write_json(args.out_json, payload)
            write_csv(
                args.out_csv,
                [[m["frequency_hz"], m["dbm"]] for m in measurements],
                header=["Frequency_Hz", "Signal_dBm"],
            )

            print(f"Direct capture saved to {args.out_json} and {args.out_csv}")
            return 0

        if args.command == "test":
            golden = load_golden(args.golden_json)
            golden_measurements = golden.get("measurements", [])
            if not golden_measurements:
                raise TinySAError("Golden sample file contains no measurements")

            freqs = [m["frequency_hz"] for m in golden_measurements]
            measurements = measure_band(
                freqs=freqs,
                tinysa=tinysa,
                lmx=lmx,
                settle_s=settle_s,
                outmask=args.outmask,
            )

            tolerance = float(args.tolerance_db)
            results = []
            failures = 0
            for golden_m, test_m in zip(golden_measurements, measurements):
                delta = test_m["dbm"] - golden_m["dbm"]
                passed = delta >= -tolerance
                if not passed:
                    failures += 1
                results.append(
                    {
                        "frequency_hz": test_m["frequency_hz"],
                        "golden_dbm": golden_m["dbm"],
                        "measured_dbm": test_m["dbm"],
                        "delta_db": delta,
                        "pass": passed,
                    }
                )

            payload = {
                "metadata": {
                    "type": "test",
                    "created_at": dt.datetime.utcnow().isoformat() + "Z",
                    "golden_source": args.golden_json,
                    "tolerance_db": tolerance,
                    "points": len(results),
                    "failures": failures,
                },
                "results": results,
            }

            write_json(args.out_json, payload)
            write_csv(
                args.out_csv,
                [
                    [
                        r["frequency_hz"],
                        r["golden_dbm"],
                        r["measured_dbm"],
                        r["delta_db"],
                        r["pass"],
                    ]
                    for r in results
                ],
                header=["Frequency_Hz", "Golden_dBm", "Measured_dBm", "Delta_dB", "Pass"],
            )

            status = "PASS" if failures == 0 else "FAIL"
            print(f"Test {status}: {failures} failures. Results saved to {args.out_json} and {args.out_csv}")
            return 0

    except (TinySAError, LMXError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        if lmx:
            lmx.disconnect()
        tinysa.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
