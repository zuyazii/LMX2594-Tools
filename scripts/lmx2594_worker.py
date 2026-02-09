#!/usr/bin/env python3
"""
32-bit worker process for USB2ANY + LMX2594 control.

Communicates over stdin/stdout using one JSON object per line.
All non-JSON output is redirected to stderr to keep IPC clean.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict, Optional

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from tinysa_antenna_test import LMXController  # noqa: E402
from usb2anyapi import GetSerialNumberFromHandle, u2aFindControllers  # noqa: E402


class WorkerError(RuntimeError):
    pass


class LMXWorker:
    def __init__(self) -> None:
        self._lmx: Optional[LMXController] = None
        self._reg_map: Optional[Dict[int, int]] = None

    def list_usb2any(self) -> Dict[str, Any]:
        count = u2aFindControllers()
        serials = []
        for idx in range(count):
            ret, serial = GetSerialNumberFromHandle(idx)
            if ret >= 0 and serial:
                serials.append(serial.decode("ascii", errors="ignore"))
        return {"count": int(count), "serials": serials}

    def connect(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if self._lmx:
            self.disconnect()

        self._lmx = LMXController(
            payload.get("serial"),
            f_osc=float(payload["f_osc"]),
            template_path=str(payload["template_path"]),
            spi_clock=float(payload["spi_clock"]),
            debug=bool(payload.get("debug", False)),
            delta_update=bool(payload.get("delta_update", True)),
            outa_pwr=int(payload.get("outa_pwr", 50)),
        )
        self._lmx.connect()
        self._reg_map = None
        return {"status": "connected"}

    def disconnect(self) -> Dict[str, Any]:
        if self._lmx:
            self._lmx.disconnect()
            self._lmx = None
        self._reg_map = None
        return {"status": "disconnected"}

    def program_initial(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not self._lmx:
            raise WorkerError("LMX not connected")
        freq_hz = float(payload["freq_hz"])
        self._reg_map = self._lmx.program_initial(freq_hz)
        return self._build_freq_response(freq_hz)

    def update_frequency(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not self._lmx:
            raise WorkerError("LMX not connected")
        freq_hz = float(payload["freq_hz"])
        self._reg_map = self._lmx.update_frequency(freq_hz, self._reg_map)
        return self._build_freq_response(freq_hz)

    def _build_freq_response(self, freq_hz: float) -> Dict[str, Any]:
        """Build response with frequency plan details."""
        result: Dict[str, Any] = {"status": "ok", "freq_hz": freq_hz}
        plan = self._lmx.get_last_plan() if self._lmx else None
        if plan:
            result["vco_hz"] = plan.get("f_vco")
            result["chdiv"] = plan.get("chdiv_value")
            result["n_int"] = plan.get("n_int")
            result["num"] = plan.get("num")
            result["den"] = plan.get("den")
            result["pfd_hz"] = plan.get("f_pd")
            # Get OUTA_PWR from reg_map R44[13:8]
            if self._reg_map:
                r44 = self._reg_map.get(44, 0)
                result["outa_pwr"] = (r44 >> 8) & 0x3F
        return result

    def wait_lock(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not self._lmx:
            raise WorkerError("LMX not connected")
        timeout_s = float(payload.get("timeout_s", 0))
        auto_recal = bool(payload.get("auto_recal", True))

        driver = getattr(self._lmx, "_lmx", None)
        if not driver or not hasattr(driver, "wait_for_lock"):
            raise WorkerError("LMX lock check not available")
        if timeout_s <= 0:
            return {"status": "skipped"}

        locked = bool(driver.wait_for_lock(timeout_s))
        if not locked and auto_recal and hasattr(driver, "recalibrate_vco"):
            locked = bool(driver.recalibrate_vco())
        return {"locked": bool(locked)}


def _handle(worker: LMXWorker, payload: Dict[str, Any]) -> Dict[str, Any]:
    cmd = payload.get("cmd")
    if cmd == "ping":
        return {"status": "pong"}
    if cmd == "list_usb2any":
        return worker.list_usb2any()
    if cmd == "connect":
        return worker.connect(payload)
    if cmd == "disconnect":
        return worker.disconnect()
    if cmd == "program_initial":
        return worker.program_initial(payload)
    if cmd == "update_frequency":
        return worker.update_frequency(payload)
    if cmd == "wait_lock":
        return worker.wait_lock(payload)
    if cmd == "shutdown":
        worker.disconnect()
        sys.exit(0)
    raise WorkerError(f"Unknown command: {cmd}")


def main() -> int:
    worker = LMXWorker()
    json_out = sys.stdout
    sys.stdout = sys.stderr
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            response = {"ok": False, "error": f"Invalid JSON: {exc}", "type": "JSONDecodeError"}
            json_out.write(json.dumps(response) + "\n")
            json_out.flush()
            continue
        try:
            result = _handle(worker, payload)
            response = {"ok": True, "result": result}
        except Exception as exc:
            response = {"ok": False, "error": str(exc), "type": exc.__class__.__name__}
        json_out.write(json.dumps(response) + "\n")
        json_out.flush()
    worker.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
