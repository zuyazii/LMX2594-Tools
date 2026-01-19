#!/usr/bin/env python3
"""
LMX2594 + tinySA calibration runner.

Captures tinySA measured levels for each programmed LMX2594 frequency,
recording timestamp, requested output level, measured level, and offset.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from tinysa_antenna_test import (  # noqa: E402
    LMXController,
    LMXError,
    TinySAController,
    TinySAError,
    build_frequency_points,
    list_serial_ports,
    parse_frequency,
)


def _utc_now() -> str:
    return dt.datetime.utcnow().isoformat() + "Z"


def _default_template_path() -> str:
    return os.path.join(
        os.path.dirname(script_dir),
        "examples",
        "register-values",
        "HexRegisterValues3600.txt",
    )


def _write_csv(path: str, rows: Iterable[List[object]], header: List[str]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


def _write_json(path: str, payload: Dict[str, object]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


def load_calibration_file(path: str) -> Dict[str, object]:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


@dataclass
class CalibrationPoint:
    timestamp: str
    frequency_hz: float
    requested_dbm: float
    measured_dbm: float
    offset_db: float

    def as_row(self) -> List[object]:
        return [
            self.timestamp,
            self.frequency_hz,
            self.requested_dbm,
            self.measured_dbm,
            self.offset_db,
        ]

    def as_dict(self) -> Dict[str, object]:
        return {
            "timestamp": self.timestamp,
            "frequency_hz": self.frequency_hz,
            "requested_dbm": self.requested_dbm,
            "measured_dbm": self.measured_dbm,
            "offset_db": self.offset_db,
        }


@dataclass
class CalibrationConfig:
    start_hz: float
    stop_hz: float
    step_hz: Optional[float]
    points: Optional[int]
    requested_dbm: float
    port: Optional[str]
    lmx_serial: Optional[str]
    lmx_template: str
    lmx_fosc: float
    lmx_spi_clock: float
    outa_pwr: int = 50
    outmask: int = 2
    wait_lock: bool = False
    delta_update: bool = True
    dwell_ms: float = 0.0
    auto_recal: bool = True
    lock_timeout_s: float = 0.0
    capture_timeout_s: float = 2.0
    capture_retry_s: float = 0.2
    calibrate_tinysa: bool = False


def _try_tinysa_calibrate(tinysa: TinySAController) -> bool:
    tsa = getattr(tinysa, "_tsa", None)
    if tsa is None:
        return False

    for method_name, args in (
        ("calibrate", ()),
        ("calibrate_device", ()),
        ("cal", ()),
        ("run_cmd", ("cal",)),
        ("cmd", ("cal",)),
        ("command", ("cal",)),
        ("send_command", ("cal",)),
    ):
        method = getattr(tsa, method_name, None)
        if callable(method):
            method(*args)
            return True
    return False


def calibrate_tinysa(port: Optional[str], verbose: bool = False, error_byte: bool = False) -> bool:
    tinysa = TinySAController(port, verbose, error_byte)
    try:
        tinysa.connect()
        return _try_tinysa_calibrate(tinysa)
    finally:
        tinysa.disconnect()


class CalibrationRunner:
    def __init__(self, config: CalibrationConfig):
        self.config = config
        self._tinysa: Optional[TinySAController] = None
        self._lmx: Optional[LMXController] = None

    def _log(self, on_log: Optional[Callable[[str], None]], message: str) -> None:
        if on_log:
            on_log(message)

    def _connect(self, on_log: Optional[Callable[[str], None]]) -> None:
        self._tinysa = TinySAController(self.config.port, verbose=False, error_byte=False)
        self._tinysa.connect()
        self._log(on_log, "tinySA connected")

        self._lmx = LMXController(
            self.config.lmx_serial,
            f_osc=self.config.lmx_fosc,
            template_path=self.config.lmx_template,
            spi_clock=self.config.lmx_spi_clock,
            debug=False,
            delta_update=self.config.delta_update,
            outa_pwr=self.config.outa_pwr,
        )
        self._lmx.connect()
        self._log(on_log, "LMX2594 connected")

    def _disconnect(self) -> None:
        if self._lmx:
            self._lmx.disconnect()
            self._lmx = None
        if self._tinysa:
            self._tinysa.disconnect()
            self._tinysa = None

    def _wait_for_lock(self, on_log: Optional[Callable[[str], None]]) -> None:
        if not self.config.wait_lock:
            return
        if self.config.lock_timeout_s <= 0:
            self._log(on_log, "Lock wait enabled but lock timeout is 0; skipping lock check")
            return
        if not self._lmx:
            return
        driver = getattr(self._lmx, "_lmx", None)
        if not driver or not hasattr(driver, "wait_for_lock"):
            self._log(on_log, "LMX lock wait not available in driver")
            return
        locked = bool(driver.wait_for_lock(self.config.lock_timeout_s))
        if not locked and self.config.auto_recal and hasattr(driver, "recalibrate_vco"):
            self._log(on_log, "Lock failed; attempting VCO recalibration")
            locked = bool(driver.recalibrate_vco())
        if not locked:
            raise LMXError("LMX2594 failed to lock within timeout")

    def _capture_dbm(self, freq_hz: float) -> float:
        if not self._tinysa:
            raise TinySAError("tinySA not connected")
        deadline = time.time() + max(0.1, self.config.capture_timeout_s)
        last_exc: Optional[Exception] = None
        while time.time() <= deadline:
            try:
                return self._tinysa.scan_point(freq_hz, outmask=self.config.outmask)
            except TinySAError as exc:
                last_exc = exc
                time.sleep(max(0.01, self.config.capture_retry_s))
        raise TinySAError(f"tinySA capture timed out: {last_exc}")

    def run(
        self,
        on_log: Optional[Callable[[str], None]] = None,
        on_point: Optional[Callable[[CalibrationPoint], None]] = None,
    ) -> List[CalibrationPoint]:
        freqs = build_frequency_points(
            self.config.start_hz, self.config.stop_hz, self.config.step_hz, self.config.points
        )
        results: List[CalibrationPoint] = []
        self._connect(on_log)
        try:
            if self.config.calibrate_tinysa:
                self._log(on_log, "Triggering tinySA calibration")
                supported = _try_tinysa_calibrate(self._tinysa)
                if not supported:
                    self._log(on_log, "tinySA calibration command not available in tsapython")

            self._log(on_log, "Connect LMX2594 output to tinySA input before starting sweep")
            self._log(
                on_log,
                "Requested dBm is recorded for calibration; output power control is not applied in this script",
            )
            reg_map: Optional[Dict[int, int]] = None
            for idx, freq in enumerate(freqs):
                if self._lmx:
                    if idx == 0:
                        reg_map = self._lmx.program_initial(freq)
                    else:
                        reg_map = self._lmx.update_frequency(freq, reg_map)

                self._wait_for_lock(on_log)
                if self.config.dwell_ms > 0:
                    time.sleep(self.config.dwell_ms / 1000.0)

                measured_dbm = self._capture_dbm(freq)
                offset_db = measured_dbm - self.config.requested_dbm
                point = CalibrationPoint(
                    timestamp=_utc_now(),
                    frequency_hz=float(freq),
                    requested_dbm=float(self.config.requested_dbm),
                    measured_dbm=float(measured_dbm),
                    offset_db=float(offset_db),
                )
                results.append(point)
                if on_point:
                    on_point(point)
        finally:
            self._disconnect()

        return results


def save_calibration_files(
    config: CalibrationConfig,
    points: List[CalibrationPoint],
    out_cal: str,
    out_csv: Optional[str],
) -> None:
    metadata: Dict[str, object] = {
        "type": "lmx2594_calibration",
        "created_at": _utc_now(),
        "start_hz": config.start_hz,
        "stop_hz": config.stop_hz,
        "step_hz": config.step_hz,
        "points": len(points),
        "requested_dbm": config.requested_dbm,
        "outa_pwr": config.outa_pwr,
        "port": config.port,
        "lmx_serial": config.lmx_serial,
        "lmx_template": config.lmx_template,
        "lmx_fosc": config.lmx_fosc,
        "lmx_spi_clock": config.lmx_spi_clock,
        "wait_lock": config.wait_lock,
        "delta_update": config.delta_update,
        "dwell_ms": config.dwell_ms,
        "auto_recal": config.auto_recal,
        "lock_timeout_s": config.lock_timeout_s,
        "outmask": config.outmask,
    }
    payload = {
        "metadata": metadata,
        "measurements": [p.as_dict() for p in points],
    }
    _write_json(out_cal, payload)
    if out_csv:
        _write_csv(
            out_csv,
            [p.as_row() for p in points],
            header=["Timestamp", "Frequency_Hz", "Requested_dBm", "Measured_dBm", "Offset_dB"],
        )


def _default_output_paths() -> Dict[str, str]:
    stamp = dt.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    base = os.path.join("output", f"calibration_{stamp}")
    return {"cal": f"{base}.cal", "csv": f"{base}.csv"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="LMX2594 + tinySA calibration sweep",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", help="tinySA serial port (e.g., COM4)")
    parser.add_argument("--lmx-serial", help="USB2ANY serial number")
    parser.add_argument("--start-freq", required=True, help="Start frequency (e.g., 3.4GHz)")
    parser.add_argument("--stop-freq", required=True, help="Stop frequency (e.g., 3.6GHz)")
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument("--step", default="10MHz", help="Step frequency (default: 10MHz)")
    group.add_argument("--points", type=int, help="Number of points (inclusive)")
    parser.add_argument("--requested-db", type=float, default=0.0, help="Requested output level in dBm")
    parser.add_argument("--outa-pwr", type=int, default=50, help="OUTA_PWR code (0-63, default: 50)")
    parser.add_argument("--calibrate-tinysa", action="store_true", help="Trigger tinySA calibration")

    parser.add_argument("--wait-lock", action="store_true", help="Wait for PLL lock after each update")
    parser.add_argument("--delta-update", action="store_true", help="Enable delta register updates (default)")
    parser.add_argument("--no-delta-update", dest="delta_update", action="store_false", help="Disable delta updates")
    parser.set_defaults(delta_update=True)

    parser.add_argument("--lmx-fosc", default="50MHz", help="LMX reference oscillator (default: 50MHz)")
    parser.add_argument(
        "--lmx-template",
        default=_default_template_path(),
        help="LMX template register values file",
    )
    parser.add_argument("--lmx-spi-clock", default="400kHz", help="USB2ANY SPI clock (default: 400kHz)")
    parser.add_argument("--dwell-ms", type=float, default=0.0, help="Dwell time after each update (ms)")
    parser.add_argument("--auto-recal", action="store_true", default=True, help="Enable auto recalibration")
    parser.add_argument("--no-auto-recal", dest="auto_recal", action="store_false", help="Disable auto recalibration")
    parser.add_argument("--lock-timeout", type=float, default=0.0, help="Lock timeout in seconds")

    parser.add_argument("--capture-timeout", type=float, default=2.0, help="tinySA capture timeout (s)")
    parser.add_argument("--capture-retry", type=float, default=0.2, help="tinySA capture retry delay (s)")

    parser.add_argument("--out-cal", help="Output .cal file (JSON)")
    parser.add_argument("--out-csv", help="Output CSV file")
    parser.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.list_ports:
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

    start_hz = parse_frequency(args.start_freq)
    stop_hz = parse_frequency(args.stop_freq)
    step_hz = parse_frequency(args.step) if args.step and args.points is None else None
    points = args.points
    lmx_fosc = parse_frequency(args.lmx_fosc)
    lmx_spi_clock = parse_frequency(args.lmx_spi_clock)

    config = CalibrationConfig(
        start_hz=start_hz,
        stop_hz=stop_hz,
        step_hz=step_hz,
        points=points,
        requested_dbm=float(args.requested_db),
        outa_pwr=int(args.outa_pwr),
        port=args.port,
        lmx_serial=args.lmx_serial,
        lmx_template=args.lmx_template,
        lmx_fosc=lmx_fosc,
        lmx_spi_clock=lmx_spi_clock,
        wait_lock=bool(args.wait_lock),
        delta_update=bool(args.delta_update),
        dwell_ms=float(args.dwell_ms),
        auto_recal=bool(args.auto_recal),
        lock_timeout_s=float(args.lock_timeout),
        capture_timeout_s=float(args.capture_timeout),
        capture_retry_s=float(args.capture_retry),
        calibrate_tinysa=bool(args.calibrate_tinysa),
    )

    out_paths = _default_output_paths()
    out_cal = args.out_cal or out_paths["cal"]
    out_csv = args.out_csv or out_paths["csv"]

    runner = CalibrationRunner(config)
    try:
        points = runner.run(on_log=lambda msg: print(msg))
        save_calibration_files(config, points, out_cal, out_csv)
    except (TinySAError, LMXError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1

    print(f"Calibration complete: {len(points)} points saved to {out_cal}")
    if out_csv:
        print(f"CSV saved to {out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
