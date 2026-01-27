#!/usr/bin/env python3
"""
64-bit GUI for LMX2594 calibration using tinySA (local) + USB2ANY worker (32-bit).
"""
from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)
root_dir = os.path.dirname(script_dir)
CONFIG_PATH = os.path.join(root_dir, ".lmx_gui_config.json")

if struct.calcsize("P") * 8 != 64:
    raise RuntimeError("This GUI must be run with 64-bit Python.")

try:
    from PySide6 import QtCore, QtGui, QtWidgets, QtNetwork
    Signal = QtCore.Signal
except ImportError:  # pragma: no cover - fallback only
    from PyQt5 import QtCore, QtGui, QtWidgets, QtNetwork  # type: ignore
    Signal = QtCore.pyqtSignal  # type: ignore

from tinysa_antenna_test import (  # noqa: E402
    TinySAController,
    TinySAError,
    build_frequency_points,
    list_serial_ports,
    parse_frequency,
)


def _utc_now() -> str:
    return QtCore.QDateTime.currentDateTimeUtc().toString(QtCore.Qt.ISODate) + "Z"


@dataclass
class CalibrationPoint:
    timestamp: str
    frequency_hz: float
    requested_dbm: float
    measured_dbm: float
    offset_db: float

    def as_dict(self) -> Dict[str, object]:
        return {
            "timestamp": self.timestamp,
            "frequency_hz": self.frequency_hz,
            "requested_dbm": self.requested_dbm,
            "measured_dbm": self.measured_dbm,
            "offset_db": self.offset_db,
        }


@dataclass
class HiddenSettings:
    fosc: str = "50MHz"
    pfd: str = "50MHz"
    spi_clock: str = "400kHz"
    outa_pwr: int = 50
    dwell_ms: float = 0.0
    auto_recal: bool = True
    lock_timeout_s: float = 0.0
    worker_python: str = os.environ.get("LMX_WORKER_PYTHON", "python")
    tinysa_debug: bool = False
    tinysa_port: Optional[str] = None
    lmx_serial: Optional[str] = None
    librevna_ipc_path: str = ""
    librevna_ipc_name: str = "librevna-ipc"
    librevna_serial: Optional[str] = None


@dataclass
class CalibrationConfig:
    start_hz: float
    stop_hz: float
    step_hz: Optional[float]
    points: Optional[int]
    requested_dbm: float
    outa_pwr: int
    wait_lock: bool
    delta_update: bool
    dwell_ms: float
    auto_recal: bool
    lock_timeout_s: float
    fosc_hz: float
    spi_clock_hz: float
    template_path: str
    tinysa_debug: bool


@dataclass
class LibreVNAConfig:
    cal_path: str
    f_start_hz: float
    f_stop_hz: float
    points: int
    ifbw_hz: float
    power_dbm: float
    threshold_db: float
    timeout_ms: float
    excited_ports: List[int]
    serial: Optional[str]


class LibreVNAIpcError(RuntimeError):
    pass


class LibreVNAIpcClient:
    def __init__(self, server_name: str) -> None:
        self._server_name = server_name

    def request(self, payload: Dict[str, object], timeout_ms: int = 5000) -> Dict[str, object]:
        socket = QtNetwork.QLocalSocket()
        socket.connectToServer(self._server_name)
        if not socket.waitForConnected(timeout_ms):
            raise LibreVNAIpcError(f"IPC connect failed: {socket.errorString()}")

        data = json.dumps(payload) + "\n"
        socket.write(data.encode("utf-8"))
        socket.flush()

        buffer = bytearray()
        timer = QtCore.QElapsedTimer()
        timer.start()
        while True:
            index = buffer.find(b"\n")
            if index >= 0:
                line = bytes(buffer[:index]).strip()
                break
            if not socket.waitForReadyRead(200):
                if timer.elapsed() > timeout_ms:
                    raise LibreVNAIpcError("IPC response timeout")
                continue
            buffer.extend(bytes(socket.readAll()))

        if not line:
            raise LibreVNAIpcError("IPC returned empty response")

        try:
            response = json.loads(line.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise LibreVNAIpcError(f"IPC JSON parse error: {exc}") from exc

        if not response.get("ok", False):
            raise LibreVNAIpcError(str(response.get("error", "IPC error")))
        result = response.get("result")
        if not isinstance(result, dict):
            raise LibreVNAIpcError("IPC returned invalid result payload")
        return result


class LibreVNARequestWorker(QtCore.QThread):
    log = Signal(str)
    done = Signal(object)
    failed = Signal(str)

    def __init__(
        self,
        server_name: str,
        cmd: str,
        payload: Dict[str, object],
        timeout_ms: int = 15000,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._server_name = server_name
        self._cmd = cmd
        self._payload = payload
        self._timeout_ms = timeout_ms

    def run(self) -> None:
        client = LibreVNAIpcClient(self._server_name)
        try:
            request = dict(self._payload)
            request["cmd"] = self._cmd
            self.log.emit(f"LibreVNA IPC -> {self._cmd}")
            result = client.request(request, timeout_ms=self._timeout_ms)
            self.log.emit(f"LibreVNA IPC <- {self._cmd} ok")
            self.done.emit(result)
        except Exception as exc:
            self.failed.emit(str(exc))


class GainPlotWidget(QtWidgets.QWidget):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._points: List[Tuple[float, float]] = []
        self.setMinimumHeight(200)

    def clear(self) -> None:
        self._points = []
        self.update()

    def add_point(self, x: float, y: float) -> None:
        self._points.append((x, y))
        self.update()

    def set_points(self, points: List[Tuple[float, float]]) -> None:
        self._points = list(points)
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), self.palette().window())

        margin_left = 50
        margin_top = 20
        margin_right = 20
        margin_bottom = 40
        plot_rect = self.rect().adjusted(
            margin_left, margin_top, -margin_right, -margin_bottom
        )

        painter.setPen(self.palette().text().color())
        painter.drawRect(plot_rect)

        if not self._points:
            painter.drawText(plot_rect, QtCore.Qt.AlignCenter, "No data")
            return

        xs = [p[0] for p in self._points]
        ys = [p[1] for p in self._points]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        if x_min == x_max:
            x_max += 1.0
        if y_min == y_max:
            y_max += 1.0

        x_padding = (x_max - x_min) * 0.02
        y_padding = (y_max - y_min) * 0.1
        x_min -= x_padding
        x_max += x_padding
        y_min -= y_padding
        y_max += y_padding

        def map_x(val: float) -> float:
            return plot_rect.left() + (val - x_min) * plot_rect.width() / (x_max - x_min)

        def map_y(val: float) -> float:
            return plot_rect.bottom() - (val - y_min) * plot_rect.height() / (y_max - y_min)

        painter.setPen(QtGui.QPen(QtGui.QColor("#1f77b4"), 2))
        last_point = None
        for x_val, y_val in self._points:
            pt = QtCore.QPointF(map_x(x_val), map_y(y_val))
            if last_point is not None:
                painter.drawLine(last_point, pt)
            last_point = pt

        painter.setPen(self.palette().text().color())
        painter.drawText(
            QtCore.QRectF(plot_rect.left(), plot_rect.bottom() + 5, plot_rect.width(), 20),
            QtCore.Qt.AlignCenter,
            "Frequency (Hz)",
        )
        painter.save()
        painter.translate(10, plot_rect.center().y())
        painter.rotate(-90)
        painter.drawText(QtCore.QRectF(0, 0, plot_rect.height(), 20), QtCore.Qt.AlignCenter, "Offset (dB)")
        painter.restore()


class SParameterPlotWidget(QtWidgets.QWidget):
    def __init__(
        self,
        x_label: str,
        y_label: str,
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._x_label = x_label
        self._y_label = y_label
        self._series: Dict[str, List[Tuple[float, float]]] = {}
        self._colors: Dict[str, QtGui.QColor] = {}
        self.setMinimumHeight(220)

    def clear(self) -> None:
        self._series = {}
        self.update()

    def set_series(
        self,
        series: Dict[str, List[Tuple[float, float]]],
        colors: Dict[str, QtGui.QColor],
    ) -> None:
        self._series = {key: list(points) for key, points in series.items()}
        self._colors = dict(colors)
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), self.palette().window())

        margin_left = 60
        margin_top = 20
        margin_right = 20
        margin_bottom = 40
        plot_rect = self.rect().adjusted(
            margin_left, margin_top, -margin_right, -margin_bottom
        )

        painter.setPen(self.palette().text().color())
        painter.drawRect(plot_rect)

        if not self._series:
            painter.drawText(plot_rect, QtCore.Qt.AlignCenter, "No data")
            return

        xs: List[float] = []
        ys: List[float] = []
        for points in self._series.values():
            xs.extend([p[0] for p in points])
            ys.extend([p[1] for p in points])
        if not xs or not ys:
            painter.drawText(plot_rect, QtCore.Qt.AlignCenter, "No data")
            return

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        if x_min == x_max:
            x_max += 1.0
        if y_min == y_max:
            y_max += 1.0

        x_padding = (x_max - x_min) * 0.02
        y_padding = (y_max - y_min) * 0.1
        x_min -= x_padding
        x_max += x_padding
        y_min -= y_padding
        y_max += y_padding

        def map_x(val: float) -> float:
            return plot_rect.left() + (val - x_min) * plot_rect.width() / (x_max - x_min)

        def map_y(val: float) -> float:
            return plot_rect.bottom() - (val - y_min) * plot_rect.height() / (y_max - y_min)

        for name, points in self._series.items():
            color = self._colors.get(name, self.palette().text().color())
            painter.setPen(QtGui.QPen(color, 2))
            last_point = None
            for x_val, y_val in points:
                pt = QtCore.QPointF(map_x(x_val), map_y(y_val))
                if last_point is not None:
                    painter.drawLine(last_point, pt)
                last_point = pt

        painter.setPen(self.palette().text().color())
        painter.drawText(
            QtCore.QRectF(plot_rect.left(), plot_rect.bottom() + 5, plot_rect.width(), 20),
            QtCore.Qt.AlignCenter,
            self._x_label,
        )
        painter.save()
        painter.translate(10, plot_rect.center().y())
        painter.rotate(-90)
        painter.drawText(
            QtCore.QRectF(0, 0, plot_rect.height(), 20),
            QtCore.Qt.AlignCenter,
            self._y_label,
        )
        painter.restore()

        legend_x = plot_rect.left() + 6
        legend_y = plot_rect.top() + 6
        for name in self._series.keys():
            color = self._colors.get(name, self.palette().text().color())
            painter.setPen(QtGui.QPen(color, 2))
            painter.drawLine(legend_x, legend_y + 6, legend_x + 16, legend_y + 6)
            painter.setPen(self.palette().text().color())
            painter.drawText(legend_x + 22, legend_y + 10, name)
            legend_y += 16


class SettingsDialog(QtWidgets.QDialog):
    def __init__(self, settings: HiddenSettings, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Hidden Settings")
        self._settings = settings

        layout = QtWidgets.QFormLayout(self)
        self.fosc_edit = QtWidgets.QLineEdit(settings.fosc)
        self.pfd_edit = QtWidgets.QLineEdit(settings.pfd)
        self.spi_edit = QtWidgets.QLineEdit(settings.spi_clock)
        self.outa_pwr_spin = QtWidgets.QSpinBox()
        self.outa_pwr_spin.setRange(0, 63)
        self.outa_pwr_spin.setValue(settings.outa_pwr)
        self.dwell_spin = QtWidgets.QDoubleSpinBox()
        self.dwell_spin.setRange(0.0, 10000.0)
        self.dwell_spin.setDecimals(1)
        self.dwell_spin.setValue(settings.dwell_ms)
        self.auto_recal_check = QtWidgets.QCheckBox()
        self.auto_recal_check.setChecked(settings.auto_recal)
        self.tinysa_debug_check = QtWidgets.QCheckBox()
        self.tinysa_debug_check.setChecked(settings.tinysa_debug)
        self.lock_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.lock_timeout_spin.setRange(0.0, 60.0)
        self.lock_timeout_spin.setDecimals(2)
        self.lock_timeout_spin.setValue(settings.lock_timeout_s)
        self.worker_python_edit = QtWidgets.QLineEdit(settings.worker_python)
        worker_browse = QtWidgets.QPushButton("Browse")
        worker_browse.clicked.connect(self._browse_worker_python)
        self.librevna_ipc_edit = QtWidgets.QLineEdit(settings.librevna_ipc_path)
        librevna_browse = QtWidgets.QPushButton("Browse")
        librevna_browse.clicked.connect(self._browse_librevna_ipc)
        self.librevna_ipc_name_edit = QtWidgets.QLineEdit(settings.librevna_ipc_name)

        worker_layout = QtWidgets.QHBoxLayout()
        worker_layout.addWidget(self.worker_python_edit)
        worker_layout.addWidget(worker_browse)
        librevna_layout = QtWidgets.QHBoxLayout()
        librevna_layout.addWidget(self.librevna_ipc_edit)
        librevna_layout.addWidget(librevna_browse)

        layout.addRow("Fosc", self.fosc_edit)
        layout.addRow("PFD", self.pfd_edit)
        layout.addRow("USB2ANY SPI clock", self.spi_edit)
        layout.addRow("OUTA_PWR (0-63)", self.outa_pwr_spin)
        layout.addRow("Dwell ms", self.dwell_spin)
        layout.addRow("Auto recal", self.auto_recal_check)
        layout.addRow("tinySA debug", self.tinysa_debug_check)
        layout.addRow("Lock timeout (s)", self.lock_timeout_spin)
        layout.addRow("Worker Python (32-bit)", worker_layout)
        layout.addRow("LibreVNA IPC binary", librevna_layout)
        layout.addRow("LibreVNA IPC name", self.librevna_ipc_name_edit)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def _browse_worker_python(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select 32-bit Python", "", "Python (python.exe)"
        )
        if path:
            self.worker_python_edit.setText(path)

    def _browse_librevna_ipc(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select LibreVNA IPC binary", "", "Executable (*.exe);;All Files (*)"
        )
        if path:
            self.librevna_ipc_edit.setText(path)

    def apply(self) -> HiddenSettings:
        self._settings.fosc = self.fosc_edit.text().strip()
        self._settings.pfd = self.pfd_edit.text().strip()
        self._settings.spi_clock = self.spi_edit.text().strip()
        self._settings.outa_pwr = int(self.outa_pwr_spin.value())
        self._settings.dwell_ms = float(self.dwell_spin.value())
        self._settings.auto_recal = bool(self.auto_recal_check.isChecked())
        self._settings.tinysa_debug = bool(self.tinysa_debug_check.isChecked())
        self._settings.lock_timeout_s = float(self.lock_timeout_spin.value())
        self._settings.worker_python = self.worker_python_edit.text().strip()
        self._settings.librevna_ipc_path = self.librevna_ipc_edit.text().strip()
        self._settings.librevna_ipc_name = self.librevna_ipc_name_edit.text().strip() or "librevna-ipc"
        return self._settings


class LMXWorkerClient:
    def __init__(self, python_path: str, on_log: Optional[Callable[[str], None]] = None) -> None:
        self._python_path = python_path or "python"
        self._worker_script = os.path.join(script_dir, "lmx2594_worker.py")
        self._worker_root = os.path.dirname(script_dir)
        self._proc: Optional[subprocess.Popen[str]] = None
        self._lock = threading.Lock()
        self._on_log = on_log
        self._stderr_thread: Optional[threading.Thread] = None
        self._checked = False

    def _log(self, message: str) -> None:
        if self._on_log:
            self._on_log(message)

    def _check_worker_python(self) -> None:
        if self._checked:
            return
        if not os.path.exists(self._python_path):
            raise RuntimeError(f"Worker Python not found: {self._python_path}")
        self._log(f"Checking worker Python: {self._python_path}")
        try:
            output = subprocess.check_output(
                [self._python_path, "-c", "import struct; print(struct.calcsize('P')*8)"],
                text=True,
            ).strip()
        except Exception as exc:
            raise RuntimeError(f"Failed to run worker Python: {exc}") from exc
        self._log(f"Worker Python bitness: {output}")
        if output != "32":
            raise RuntimeError(
                f"Worker Python must be 32-bit (got {output}-bit). Set it in Hidden settings."
            )
        self._checked = True

    def start(self) -> None:
        if self._proc and self._proc.poll() is None:
            return
        if not os.path.exists(self._worker_script):
            raise RuntimeError(f"Worker script not found: {self._worker_script}")
        self._check_worker_python()
        self._log("Starting LMX worker process")
        self._proc = subprocess.Popen(
            [self._python_path, self._worker_script],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            cwd=self._worker_root,
        )
        if self._proc.stderr:
            self._stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
            self._stderr_thread.start()

    def _drain_stderr(self) -> None:
        if not self._proc or not self._proc.stderr:
            return
        for line in self._proc.stderr:
            cleaned = line.strip()
            if cleaned:
                self._log(f"[LMX worker] {cleaned}")

    def request(self, payload: Dict[str, object]) -> Dict[str, object]:
        with self._lock:
            self.start()
            if not self._proc or not self._proc.stdin or not self._proc.stdout:
                raise RuntimeError("Worker process not ready")
            cmd = payload.get("cmd", "unknown")
            self._log(f"IPC -> {cmd}")
            self._proc.stdin.write(json.dumps(payload) + "\n")
            self._proc.stdin.flush()
            response_line = self._proc.stdout.readline()
            if not response_line:
                raise RuntimeError("Worker did not respond")
            response = json.loads(response_line)
            if not response.get("ok"):
                raise RuntimeError(response.get("error", "Worker error"))
            self._log(f"IPC <- {cmd} ok")
            return response.get("result", {})

    def stop(self) -> None:
        with self._lock:
            if not self._proc:
                return
            self._log("Stopping LMX worker process")
            try:
                if self._proc.poll() is None:
                    if self._proc.stdin:
                        self._proc.stdin.write(json.dumps({"cmd": "shutdown"}) + "\n")
                        self._proc.stdin.flush()
            except Exception:
                pass
            try:
                self._proc.terminate()
            except Exception:
                pass
            self._proc = None


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


class CalibrationWorker(QtCore.QThread):
    log = Signal(str)
    point = Signal(object)
    done = Signal(list)
    failed = Signal(str)

    def __init__(
        self,
        config: CalibrationConfig,
        port: Optional[str],
        lmx_serial: Optional[str],
        worker_python: str,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._config = config
        self._port = port
        self._lmx_serial = lmx_serial
        self._worker_python = worker_python
        self._stop_event = threading.Event()
        self._suppress_signals = False
        self._client: Optional[LMXWorkerClient] = None
        self._tinysa: Optional[TinySAController] = None

    def request_stop(self) -> None:
        self._suppress_signals = True
        self._stop_event.set()
        client = self._client
        if client:
            try:
                client.request({"cmd": "disconnect"})
            except Exception:
                pass
            client.stop()
        tinysa = self._tinysa
        if tinysa:
            try:
                tinysa.disconnect()
            except Exception:
                pass

    def run(self) -> None:
        client = LMXWorkerClient(self._worker_python, on_log=self.log.emit)
        tinysa = TinySAController(
            self._port,
            verbose=False,
            error_byte=False,
            debug_cb=self.log.emit if self._config.tinysa_debug else None,
        )
        self._client = client
        self._tinysa = tinysa
        points: List[CalibrationPoint] = []
        try:
            self.log.emit(f"Connecting to tinySA (port={self._port or 'Auto'})")
            tinysa.connect()
            self.log.emit("tinySA connected")

            self.log.emit(f"Connecting to LMX worker (serial={self._lmx_serial or 'Auto'})")
            client.request(
                {
                    "cmd": "connect",
                    "serial": self._lmx_serial,
                    "f_osc": self._config.fosc_hz,
                    "template_path": self._config.template_path,
                    "spi_clock": self._config.spi_clock_hz,
                    "delta_update": self._config.delta_update,
                    "outa_pwr": self._config.outa_pwr,
                }
            )
            self.log.emit("LMX worker connected")

            freqs = build_frequency_points(
                self._config.start_hz,
                self._config.stop_hz,
                self._config.step_hz,
                self._config.points,
            )
            total = len(freqs)
            for idx, freq in enumerate(freqs):
                if self._stop_event.is_set():
                    self.log.emit("Calibration stop requested")
                    break
                self.log.emit(f"Programming frequency {idx + 1}/{total}: {freq:.3f} Hz")
                if idx == 0:
                    client.request({"cmd": "program_initial", "freq_hz": freq})
                else:
                    client.request({"cmd": "update_frequency", "freq_hz": freq})

                if self._config.wait_lock:
                    self.log.emit("Waiting for LMX lock...")
                    result = client.request(
                        {
                            "cmd": "wait_lock",
                            "timeout_s": self._config.lock_timeout_s,
                            "auto_recal": self._config.auto_recal,
                        }
                    )
                    if result.get("locked") is False:
                        raise RuntimeError("LMX lock failed")

                if self._config.dwell_ms > 0:
                    self.log.emit(f"Dwell {self._config.dwell_ms:.1f} ms")
                    time.sleep(self._config.dwell_ms / 1000.0)

                if self._stop_event.is_set():
                    self.log.emit("Calibration stop requested")
                    break

                self.log.emit("Reading tinySA measurement (timeout 5s)...")
                measured_dbm = tinysa.scan_point(freq, outmask=2, timeout_s=5.0, retries=1)
                offset_db = measured_dbm - self._config.requested_dbm
                point = CalibrationPoint(
                    timestamp=_utc_now(),
                    frequency_hz=float(freq),
                    requested_dbm=float(self._config.requested_dbm),
                    measured_dbm=float(measured_dbm),
                    offset_db=float(offset_db),
                )
                points.append(point)
                self.point.emit(point)

            if not self._stop_event.is_set() and not self._suppress_signals:
                self.done.emit(points)
        except Exception as exc:
            if not self._stop_event.is_set() and not self._suppress_signals:
                self.failed.emit(str(exc))
        finally:
            try:
                client.request({"cmd": "disconnect"})
            except Exception:
                pass
            client.stop()
            try:
                tinysa.disconnect()
            except Exception:
                pass


class CalibrateTinySAWorker(QtCore.QThread):
    log = Signal(str)
    done = Signal(bool)

    def __init__(self, port: Optional[str], debug: bool, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._port = port
        self._debug = debug
        self._stop_event = threading.Event()

    def request_stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        self.log.emit("Connecting to tinySA for calibration")
        tinysa = TinySAController(
            self._port,
            verbose=False,
            error_byte=False,
            debug_cb=self.log.emit if self._debug else None,
        )
        try:
            if self._stop_event.is_set():
                return
            tinysa.connect()
            if self._stop_event.is_set():
                return
            supported = _try_tinysa_calibrate(tinysa)
        except Exception as exc:
            self.log.emit(f"tinySA calibration failed: {exc}")
            self.done.emit(False)
            return
        finally:
            tinysa.disconnect()

        if not supported:
            self.log.emit("tinySA calibration command not available in tsapython")
        self.done.emit(bool(supported))


class CalibrationWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("LMX2594 + LibreVNA Calibration (64-bit GUI)")
        self.resize(1400, 900)

        self._settings = HiddenSettings()
        self._load_settings_from_disk()
        self._worker: Optional[CalibrationWorker] = None
        self._cal_worker: Optional[CalibrateTinySAWorker] = None
        self._librevna_device_worker: Optional[LibreVNARequestWorker] = None
        self._librevna_sweep_worker: Optional[LibreVNARequestWorker] = None
        self._last_config: Optional[CalibrationConfig] = None
        self._busy = False
        self._librevna_busy = False
        self._suppress_device_save = False
        self._librevna_process: Optional[QtCore.QProcess] = None
        self._librevna_trace: List[Dict[str, object]] = []

        central = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(central)

        top_layout = QtWidgets.QHBoxLayout()
        top_layout.addWidget(self._build_device_group())
        top_layout.addWidget(self._build_calibration_group())
        top_layout.addWidget(self._build_librevna_calibration_group())
        main_layout.addLayout(top_layout)

        params_layout = QtWidgets.QGridLayout()
        params_layout.addWidget(self._build_global_params_group(), 0, 0)
        params_layout.addWidget(self._build_lmx_params_group(), 0, 1)
        params_layout.addWidget(self._build_tinysa_params_group(), 1, 0)
        params_layout.addWidget(self._build_librevna_params_group(), 1, 1)
        main_layout.addLayout(params_layout)

        results_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        results_split.addWidget(self._build_results_group())
        results_split.addWidget(self._build_log_group())
        results_split.setStretchFactor(0, 3)
        results_split.setStretchFactor(1, 1)
        main_layout.addWidget(results_split)

        self.setCentralWidget(central)
        self._update_librevna_plot_visibility()
        self._update_controls()
        self._refresh_devices()

    def _build_device_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Device Finder and Selector")
        layout = QtWidgets.QGridLayout(group)
        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.refresh_button.clicked.connect(self._refresh_devices)

        self.tinysa_combo = QtWidgets.QComboBox()
        self.lmx_combo = QtWidgets.QComboBox()
        self.librevna_combo = QtWidgets.QComboBox()
        self.tinysa_combo.currentIndexChanged.connect(self._on_device_selection_changed)
        self.lmx_combo.currentIndexChanged.connect(self._on_device_selection_changed)
        self.librevna_combo.currentIndexChanged.connect(self._on_device_selection_changed)

        layout.addWidget(QtWidgets.QLabel("tinySA Port"), 0, 0)
        layout.addWidget(self.tinysa_combo, 0, 1)
        layout.addWidget(QtWidgets.QLabel("USB2ANY Serial"), 1, 0)
        layout.addWidget(self.lmx_combo, 1, 1)
        layout.addWidget(QtWidgets.QLabel("LibreVNA"), 2, 0)
        layout.addWidget(self.librevna_combo, 2, 1)
        layout.addWidget(self.refresh_button, 0, 2, 3, 1)
        return group

    def _build_calibration_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("LMX2594 Calibration")
        layout = QtWidgets.QGridLayout(group)

        self.calibrate_tinysa_button = QtWidgets.QPushButton("Calibrate tinySA")
        self.calibrate_tinysa_button.clicked.connect(self._run_tinysa_calibration)

        self.start_button = QtWidgets.QPushButton("Start Calibration")
        self.start_button.clicked.connect(self._start_calibration)

        self.load_button = QtWidgets.QPushButton("Load .cal")
        self.load_button.clicked.connect(self._load_calibration)

        self.output_edit = QtWidgets.QLineEdit()
        self.output_edit.setPlaceholderText("Output .cal path (optional)")
        browse = QtWidgets.QPushButton("Browse")
        browse.clicked.connect(self._browse_output)

        layout.addWidget(self.calibrate_tinysa_button, 0, 0)
        layout.addWidget(self.start_button, 0, 1)
        layout.addWidget(self.load_button, 0, 2)
        layout.addWidget(QtWidgets.QLabel("Output .cal"), 1, 0)
        layout.addWidget(self.output_edit, 1, 1)
        layout.addWidget(browse, 1, 2)
        return group

    def _build_librevna_calibration_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("LibreVNA Calibration")
        layout = QtWidgets.QGridLayout(group)

        self.librevna_load_button = QtWidgets.QPushButton("Load .cal")
        self.librevna_load_button.clicked.connect(self._load_librevna_calibration)
        self.librevna_guide_button = QtWidgets.QPushButton("Guide Calibration")
        self.librevna_guide_button.clicked.connect(self._guide_librevna_calibration)

        self.librevna_cal_edit = QtWidgets.QLineEdit()
        self.librevna_cal_edit.setPlaceholderText("LibreVNA .cal path")
        browse = QtWidgets.QPushButton("Browse")
        browse.clicked.connect(self._browse_librevna_calibration)

        layout.addWidget(self.librevna_load_button, 0, 0)
        layout.addWidget(self.librevna_guide_button, 0, 1)
        layout.addWidget(QtWidgets.QLabel("Calibration file"), 1, 0)
        layout.addWidget(self.librevna_cal_edit, 1, 1)
        layout.addWidget(browse, 1, 2)
        return group

    def _build_global_params_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Parameter Setting - Global")
        layout = QtWidgets.QFormLayout(group)

        self.start_freq_edit = QtWidgets.QLineEdit("3.4GHz")
        self.stop_freq_edit = QtWidgets.QLineEdit("3.6GHz")
        self.step_edit = QtWidgets.QLineEdit("10MHz")
        self.points_spin = QtWidgets.QSpinBox()
        self.points_spin.setRange(0, 1000000)
        self.points_spin.setValue(0)
        self.requested_dbm_edit = QtWidgets.QDoubleSpinBox()
        self.requested_dbm_edit.setRange(-120.0, 30.0)
        self.requested_dbm_edit.setDecimals(2)
        self.requested_dbm_edit.setValue(0.0)
        self.golden_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.golden_threshold_spin.setRange(-200.0, 200.0)
        self.golden_threshold_spin.setDecimals(2)
        self.golden_threshold_spin.setValue(0.0)
        self.golden_tolerance_spin = QtWidgets.QDoubleSpinBox()
        self.golden_tolerance_spin.setRange(0.0, 200.0)
        self.golden_tolerance_spin.setDecimals(2)
        self.golden_tolerance_spin.setValue(0.0)

        layout.addRow("Frequency start", self.start_freq_edit)
        layout.addRow("Frequency stop", self.stop_freq_edit)
        layout.addRow("Step (if points=0)", self.step_edit)
        layout.addRow("Number of points", self.points_spin)
        layout.addRow("Requested dBm", self.requested_dbm_edit)
        layout.addRow("Golden sample threshold", self.golden_threshold_spin)
        layout.addRow("Golden sample tolerance", self.golden_tolerance_spin)
        return group

    def _build_lmx_params_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Parameter Setting - LMX2594")
        layout = QtWidgets.QVBoxLayout(group)
        self.wait_lock_check = QtWidgets.QCheckBox("Wait lock")
        self.wait_lock_check.setChecked(False)
        self.delta_update_check = QtWidgets.QCheckBox("Delta update")
        self.delta_update_check.setChecked(True)
        self.settings_button = QtWidgets.QPushButton("Hidden settings...")
        self.settings_button.clicked.connect(self._open_settings)

        layout.addWidget(self.wait_lock_check)
        layout.addWidget(self.delta_update_check)
        layout.addStretch(1)
        layout.addWidget(self.settings_button)
        return group

    def _build_tinysa_params_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Parameter Setting - tinySA")
        layout = QtWidgets.QFormLayout(group)
        self.tinysa_gain_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.tinysa_gain_threshold_spin.setRange(-200.0, 200.0)
        self.tinysa_gain_threshold_spin.setDecimals(2)
        self.tinysa_gain_threshold_spin.setValue(0.0)
        layout.addRow("Gain pass threshold (dB)", self.tinysa_gain_threshold_spin)
        return group

    def _build_librevna_params_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Parameter Setting - LibreVNA")
        layout = QtWidgets.QVBoxLayout(group)

        form = QtWidgets.QFormLayout()
        self.librevna_ifbw_spin = QtWidgets.QDoubleSpinBox()
        self.librevna_ifbw_spin.setRange(1.0, 1e7)
        self.librevna_ifbw_spin.setDecimals(0)
        self.librevna_ifbw_spin.setValue(1000.0)
        self.librevna_power_spin = QtWidgets.QDoubleSpinBox()
        self.librevna_power_spin.setRange(-100.0, 20.0)
        self.librevna_power_spin.setDecimals(1)
        self.librevna_power_spin.setValue(-10.0)
        self.librevna_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.librevna_threshold_spin.setRange(-200.0, 50.0)
        self.librevna_threshold_spin.setDecimals(1)
        self.librevna_threshold_spin.setValue(-10.0)
        self.librevna_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.librevna_timeout_spin.setRange(100.0, 60000.0)
        self.librevna_timeout_spin.setDecimals(0)
        self.librevna_timeout_spin.setValue(15000.0)

        form.addRow("IFBW (Hz)", self.librevna_ifbw_spin)
        form.addRow("Power (dBm)", self.librevna_power_spin)
        form.addRow("S-parameter threshold (dB)", self.librevna_threshold_spin)
        form.addRow("Timeout (ms)", self.librevna_timeout_spin)
        layout.addLayout(form)

        ports_layout = QtWidgets.QHBoxLayout()
        self.librevna_port1_check = QtWidgets.QCheckBox("Port 1")
        self.librevna_port1_check.setChecked(True)
        self.librevna_port2_check = QtWidgets.QCheckBox("Port 2")
        self.librevna_port2_check.setChecked(True)
        ports_layout.addWidget(self.librevna_port1_check)
        ports_layout.addWidget(self.librevna_port2_check)
        ports_layout.addStretch(1)
        layout.addLayout(ports_layout)

        sparam_layout = QtWidgets.QHBoxLayout()
        self.librevna_sparam_checks = {
            "S11": QtWidgets.QCheckBox("S11"),
            "S21": QtWidgets.QCheckBox("S21"),
            "S12": QtWidgets.QCheckBox("S12"),
            "S22": QtWidgets.QCheckBox("S22"),
        }
        for check in self.librevna_sparam_checks.values():
            check.setChecked(True)
            check.stateChanged.connect(self._update_librevna_plots)
            sparam_layout.addWidget(check)
        sparam_layout.addStretch(1)
        layout.addLayout(sparam_layout)

        button_layout = QtWidgets.QHBoxLayout()
        self.librevna_start_button = QtWidgets.QPushButton("Run Sweep")
        self.librevna_start_button.clicked.connect(self._start_librevna_sweep)
        self.librevna_stop_button = QtWidgets.QPushButton("Stop")
        self.librevna_stop_button.clicked.connect(self._stop_librevna_sweep)
        self.librevna_reset_button = QtWidgets.QPushButton("Reset")
        self.librevna_reset_button.clicked.connect(self._reset_librevna_params)
        button_layout.addWidget(self.librevna_start_button)
        button_layout.addWidget(self.librevna_stop_button)
        button_layout.addWidget(self.librevna_reset_button)
        layout.addLayout(button_layout)
        return group

    def _build_results_group(self) -> QtWidgets.QWidget:
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._build_lmx_results_tab(), "LMX2594 Results")
        tabs.addTab(self._build_librevna_results_tab(), "LibreVNA Results")
        return tabs

    def _build_lmx_results_tab(self) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(container)

        table_group = QtWidgets.QGroupBox("LMX2594 Calibration Results")
        table_layout = QtWidgets.QVBoxLayout(table_group)
        self.results_table = QtWidgets.QTableWidget(0, 5)
        self.results_table.setHorizontalHeaderLabels(
            ["Timestamp", "Frequency (Hz)", "Requested dBm", "Measured dBm", "Offset dB"]
        )
        self.results_table.horizontalHeader().setStretchLastSection(True)
        table_layout.addWidget(self.results_table)

        plot_group = QtWidgets.QGroupBox("Gain")
        plot_layout = QtWidgets.QVBoxLayout(plot_group)
        self.plot_widget = GainPlotWidget()
        plot_layout.addWidget(self.plot_widget)

        layout.addWidget(table_group, 3)
        layout.addWidget(plot_group, 2)
        return container

    def _build_librevna_results_tab(self) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)

        header = QtWidgets.QHBoxLayout()
        self.librevna_status_label = QtWidgets.QLabel("Status: Idle")
        self.librevna_pass_label = QtWidgets.QLabel("Overall: N/A")
        self.librevna_pass_label.setStyleSheet("font-weight: bold;")
        self.librevna_show_mag_check = QtWidgets.QCheckBox("Magnitude")
        self.librevna_show_mag_check.setChecked(True)
        self.librevna_show_mag_check.stateChanged.connect(self._update_librevna_plot_visibility)
        self.librevna_show_phase_check = QtWidgets.QCheckBox("Phase")
        self.librevna_show_phase_check.setChecked(True)
        self.librevna_show_phase_check.stateChanged.connect(self._update_librevna_plot_visibility)
        header.addWidget(self.librevna_status_label)
        header.addStretch(1)
        header.addWidget(self.librevna_pass_label)
        header.addSpacing(20)
        header.addWidget(self.librevna_show_mag_check)
        header.addWidget(self.librevna_show_phase_check)
        layout.addLayout(header)

        self.librevna_results_table = QtWidgets.QTableWidget(0, 4)
        self.librevna_results_table.setHorizontalHeaderLabels(
            ["S-Parameter", "Pass", "Worst dB", "Fail @ Hz"]
        )
        self.librevna_results_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.librevna_results_table)

        plots = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.librevna_mag_group = QtWidgets.QGroupBox("Magnitude (dB)")
        mag_layout = QtWidgets.QVBoxLayout(self.librevna_mag_group)
        self.librevna_mag_plot = SParameterPlotWidget("Frequency (Hz)", "Magnitude (dB)")
        mag_layout.addWidget(self.librevna_mag_plot)
        self.librevna_phase_group = QtWidgets.QGroupBox("Phase (deg)")
        phase_layout = QtWidgets.QVBoxLayout(self.librevna_phase_group)
        self.librevna_phase_plot = SParameterPlotWidget("Frequency (Hz)", "Phase (deg)")
        phase_layout.addWidget(self.librevna_phase_plot)
        plots.addWidget(self.librevna_mag_group)
        plots.addWidget(self.librevna_phase_group)
        plots.setStretchFactor(0, 1)
        plots.setStretchFactor(1, 1)
        layout.addWidget(plots)
        return container

    def _build_log_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Event Log Monitoring")
        layout = QtWidgets.QVBoxLayout(group)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view)
        return group

    def _append_log(self, message: str) -> None:
        stamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log_view.appendPlainText(f"[{stamp}] {message}")

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        self._update_controls()

    def _set_librevna_busy(self, busy: bool) -> None:
        self._librevna_busy = busy
        self._update_controls()

    def _update_controls(self) -> None:
        busy = self._busy or self._librevna_busy
        self.start_button.setEnabled(not busy)
        self.calibrate_tinysa_button.setEnabled(not busy)
        self.refresh_button.setEnabled(not busy)
        self.librevna_start_button.setEnabled(not busy)
        self.librevna_stop_button.setEnabled(self._librevna_busy)
        self.librevna_load_button.setEnabled(not self._librevna_busy)
        self.librevna_guide_button.setEnabled(not self._librevna_busy)
        self.librevna_reset_button.setEnabled(not self._librevna_busy)

    def _refresh_devices(self) -> None:
        self._append_log("Refresh device list pressed")
        self._suppress_device_save = True
        self.tinysa_combo.clear()
        self.lmx_combo.clear()
        self.librevna_combo.clear()
        self.tinysa_combo.addItem("Auto", None)
        self.lmx_combo.addItem("Auto", None)
        self.librevna_combo.addItem("Auto", None)

        try:
            ports = list_serial_ports()
            for port, desc, _hwid in ports:
                self.tinysa_combo.addItem(f"{port} ({desc})", port)
            if ports:
                port_list = ", ".join([p for p, _d, _h in ports])
                self._append_log(f"tinySA ports detected: {port_list}")
            else:
                self._append_log("No serial ports detected for tinySA")
        except Exception as exc:
            self._append_log(f"Port scan failed: {exc}")

        try:
            serials, count = self._list_usb2any()
            for serial in serials:
                if serial:
                    self.lmx_combo.addItem(serial, serial)
            if serials:
                self._append_log(f"USB2ANY devices detected: {', '.join(serials)}")
            elif count > 0:
                self._append_log(
                    "USB2ANY controllers detected but serial lookup failed; use Auto selection"
                )
            else:
                self._append_log("No USB2ANY devices detected")
        except Exception as exc:
            self._append_log(f"USB2ANY scan failed: {exc}")
        self._refresh_librevna_devices()
        self._apply_saved_device_selection()
        self._suppress_device_save = False

    def _apply_saved_device_selection(self) -> None:
        if self._settings.tinysa_port:
            index = self.tinysa_combo.findData(self._settings.tinysa_port)
            if index == -1:
                index = self.tinysa_combo.findText(self._settings.tinysa_port, QtCore.Qt.MatchStartsWith)
            if index >= 0:
                self.tinysa_combo.setCurrentIndex(index)
        if self._settings.lmx_serial:
            index = self.lmx_combo.findData(self._settings.lmx_serial)
            if index == -1:
                index = self.lmx_combo.findText(self._settings.lmx_serial, QtCore.Qt.MatchStartsWith)
            if index >= 0:
                self.lmx_combo.setCurrentIndex(index)
        if self._settings.librevna_serial:
            index = self.librevna_combo.findData(self._settings.librevna_serial)
            if index == -1:
                index = self.librevna_combo.findText(
                    self._settings.librevna_serial, QtCore.Qt.MatchStartsWith
                )
            if index >= 0:
                self.librevna_combo.setCurrentIndex(index)

    def _on_device_selection_changed(self) -> None:
        if self._suppress_device_save:
            return
        self._settings.tinysa_port = self._get_selected_port()
        self._settings.lmx_serial = self._get_selected_lmx()
        self._settings.librevna_serial = self._get_selected_librevna()
        self._save_settings_to_disk()

    def _list_usb2any(self) -> Tuple[List[str], int]:
        client = LMXWorkerClient(self._settings.worker_python, on_log=self._append_log)
        try:
            result = client.request({"cmd": "list_usb2any"})
            serials = [str(x) for x in result.get("serials", [])]
            count = int(result.get("count", 0))
            return serials, count
        finally:
            client.stop()

    def _ensure_librevna_ipc(self) -> bool:
        if self._librevna_process and self._librevna_process.state() != QtCore.QProcess.NotRunning:
            return True

        binary = self._settings.librevna_ipc_path
        if not binary:
            self._append_log("LibreVNA IPC binary not set. Configure in Hidden settings.")
            return False
        if not os.path.exists(binary):
            self._append_log(f"LibreVNA IPC binary not found: {binary}")
            return False

        process = QtCore.QProcess(self)
        env = QtCore.QProcessEnvironment.systemEnvironment()
        if self._settings.librevna_ipc_name:
            env.insert("LIBREVNA_IPC_NAME", self._settings.librevna_ipc_name)
        process.setProcessEnvironment(env)
        process.setProgram(binary)
        process.setArguments([])
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._drain_librevna_process)
        process.readyReadStandardError.connect(self._drain_librevna_process)
        process.start()
        if not process.waitForStarted(3000):
            self._append_log("Failed to start LibreVNA IPC process")
            return False
        self._librevna_process = process
        self._append_log("LibreVNA IPC started")
        return True

    def _drain_librevna_process(self) -> None:
        if not self._librevna_process:
            return
        data = bytes(self._librevna_process.readAllStandardOutput())
        if not data:
            data = bytes(self._librevna_process.readAllStandardError())
        text = data.decode("utf-8", errors="ignore").strip()
        if text:
            self._append_log(f"[LibreVNA IPC] {text}")

    def _refresh_librevna_devices(self) -> None:
        if self._librevna_device_worker and self._librevna_device_worker.isRunning():
            self._append_log("LibreVNA device scan already running")
            return

        self._append_log("Scanning LibreVNA devices")
        self._ensure_librevna_ipc()
        worker = LibreVNARequestWorker(
            server_name=self._settings.librevna_ipc_name,
            cmd="list_devices",
            payload={},
            timeout_ms=5000,
        )
        worker.log.connect(self._append_log)
        worker.done.connect(self._handle_librevna_devices)
        worker.failed.connect(self._handle_librevna_device_error)
        self._librevna_device_worker = worker
        worker.start()

    def _handle_librevna_devices(self, payload: Dict[str, object]) -> None:
        self._suppress_device_save = True
        self.librevna_combo.blockSignals(True)
        self.librevna_combo.clear()
        self.librevna_combo.addItem("Auto", None)

        devices = payload.get("devices", [])
        if isinstance(devices, list):
            for entry in devices:
                if not isinstance(entry, dict):
                    continue
                label = str(entry.get("label", "LibreVNA"))
                serial = str(entry.get("serial", "")).strip()
                display = f"{label} ({serial})" if serial else label
                self.librevna_combo.addItem(display, serial or label)

        warning = payload.get("warning")
        if isinstance(warning, str) and warning:
            self._append_log(f"LibreVNA device warning: {warning}")
        real_driver = payload.get("real_driver")
        if real_driver is False:
            self._append_log("LibreVNA: running with stub driver (real driver unavailable)")

        self.librevna_combo.blockSignals(False)
        self._apply_saved_device_selection()
        self._suppress_device_save = False

    def _handle_librevna_device_error(self, message: str) -> None:
        self._append_log(f"LibreVNA device scan failed: {message}")

    def _open_settings(self) -> None:
        self._append_log("Hidden settings opened")
        dialog = SettingsDialog(self._settings, self)
        if dialog.exec() == QtWidgets.QDialog.Accepted:
            self._settings = dialog.apply()
            self._append_log("Hidden settings updated")
            self._save_settings_to_disk()
            self._refresh_devices()
        else:
            self._append_log("Hidden settings canceled")

    def _load_settings_from_disk(self) -> None:
        if not os.path.exists(CONFIG_PATH):
            return
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception:
            return
        worker_python = data.get("worker_python")
        if isinstance(worker_python, str) and worker_python.strip():
            self._settings.worker_python = worker_python.strip()
        tinysa_debug = data.get("tinysa_debug")
        if isinstance(tinysa_debug, bool):
            self._settings.tinysa_debug = tinysa_debug
        tinysa_port = data.get("tinysa_port")
        if isinstance(tinysa_port, str) and tinysa_port.strip():
            self._settings.tinysa_port = tinysa_port.strip()
        lmx_serial = data.get("lmx_serial")
        if isinstance(lmx_serial, str) and lmx_serial.strip():
            self._settings.lmx_serial = lmx_serial.strip()
        librevna_ipc_path = data.get("librevna_ipc_path")
        if isinstance(librevna_ipc_path, str) and librevna_ipc_path.strip():
            self._settings.librevna_ipc_path = librevna_ipc_path.strip()
        librevna_ipc_name = data.get("librevna_ipc_name")
        if isinstance(librevna_ipc_name, str) and librevna_ipc_name.strip():
            self._settings.librevna_ipc_name = librevna_ipc_name.strip()
        librevna_serial = data.get("librevna_serial")
        if isinstance(librevna_serial, str) and librevna_serial.strip():
            self._settings.librevna_serial = librevna_serial.strip()

    def _save_settings_to_disk(self) -> None:
        payload = {
            "worker_python": self._settings.worker_python,
            "tinysa_debug": self._settings.tinysa_debug,
            "tinysa_port": self._settings.tinysa_port,
            "lmx_serial": self._settings.lmx_serial,
            "librevna_ipc_path": self._settings.librevna_ipc_path,
            "librevna_ipc_name": self._settings.librevna_ipc_name,
            "librevna_serial": self._settings.librevna_serial,
        }
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
            self._append_log(f"Saved settings: {CONFIG_PATH}")
        except Exception as exc:
            self._append_log(f"Failed to save settings: {exc}")

    def _get_selected_port(self) -> Optional[str]:
        data = self.tinysa_combo.currentData()
        if data:
            return str(data)
        return None

    def _get_selected_lmx(self) -> Optional[str]:
        data = self.lmx_combo.currentData()
        if data:
            return str(data)
        return None

    def _get_selected_librevna(self) -> Optional[str]:
        data = self.librevna_combo.currentData()
        if data:
            return str(data)
        return None

    def _browse_output(self) -> None:
        self._append_log("Browse output path pressed")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Select calibration file", "", "Calibration Files (*.cal);;JSON Files (*.json)"
        )
        if path:
            self.output_edit.setText(path)
            self._append_log(f"Output path set: {path}")

    def _browse_librevna_calibration(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select LibreVNA calibration file", "", "Calibration Files (*.cal *.json)"
        )
        if path:
            self.librevna_cal_edit.setText(path)
            self._append_log(f"LibreVNA calibration file set: {path}")

    def _load_librevna_calibration(self) -> None:
        path = self.librevna_cal_edit.text().strip()
        if not path:
            self._browse_librevna_calibration()
            path = self.librevna_cal_edit.text().strip()
        if not path:
            self._append_log("LibreVNA calibration load canceled")
            return
        if not os.path.exists(path):
            self._append_log(f"LibreVNA calibration file not found: {path}")
            return
        self.librevna_cal_edit.setText(path)
        self._append_log(f"LibreVNA calibration loaded: {path}")

    def _guide_librevna_calibration(self) -> None:
        message = (
            "LibreVNA calibration workflow is not automated yet.\n"
            "Use the device's UI to perform Open/Short/Load (and Through if needed), "
            "then load the resulting .cal file here."
        )
        QtWidgets.QMessageBox.information(self, "LibreVNA Calibration Guide", message)

    def _run_tinysa_calibration(self) -> None:
        if self._worker and self._worker.isRunning():
            self._append_log("Cannot calibrate tinySA while calibration is running")
            return
        if self._cal_worker and self._cal_worker.isRunning():
            self._append_log("tinySA calibration already running")
            return
        port = self._get_selected_port()
        self._append_log(f"Calibrate tinySA pressed (port={port or 'Auto'})")
        self._cal_worker = CalibrateTinySAWorker(port, self._settings.tinysa_debug)
        self._cal_worker.log.connect(self._append_log)
        self._cal_worker.done.connect(
            lambda ok: self._append_log("tinySA calibration complete" if ok else "tinySA calibration finished")
        )
        self._set_busy(True)
        self._cal_worker.done.connect(lambda _ok: self._set_busy(False))
        self._cal_worker.start()

    def _reset_librevna_params(self) -> None:
        self.librevna_ifbw_spin.setValue(1000.0)
        self.librevna_power_spin.setValue(-10.0)
        self.librevna_threshold_spin.setValue(-10.0)
        self.librevna_timeout_spin.setValue(15000.0)
        self.librevna_port1_check.setChecked(True)
        self.librevna_port2_check.setChecked(True)
        for check in self.librevna_sparam_checks.values():
            check.setChecked(True)
        self._append_log("LibreVNA parameters reset")

    def _build_librevna_config(self) -> LibreVNAConfig:
        start_hz = parse_frequency(self.start_freq_edit.text())
        stop_hz = parse_frequency(self.stop_freq_edit.text())
        points = int(self.points_spin.value())
        if points <= 0:
            step_hz = parse_frequency(self.step_edit.text())
            points = len(build_frequency_points(start_hz, stop_hz, step_hz, None))
        if points <= 0:
            raise ValueError("LibreVNA requires at least 1 point")

        cal_path = self.librevna_cal_edit.text().strip()
        if not cal_path:
            raise ValueError("LibreVNA calibration file is required")
        if not os.path.exists(cal_path):
            raise ValueError(f"LibreVNA calibration file not found: {cal_path}")

        excited_ports: List[int] = []
        if self.librevna_port1_check.isChecked():
            excited_ports.append(1)
        if self.librevna_port2_check.isChecked():
            excited_ports.append(2)
        if not excited_ports:
            raise ValueError("Select at least one LibreVNA port")

        return LibreVNAConfig(
            cal_path=cal_path,
            f_start_hz=start_hz,
            f_stop_hz=stop_hz,
            points=points,
            ifbw_hz=float(self.librevna_ifbw_spin.value()),
            power_dbm=float(self.librevna_power_spin.value()),
            threshold_db=float(self.librevna_threshold_spin.value()),
            timeout_ms=float(self.librevna_timeout_spin.value()),
            excited_ports=excited_ports,
            serial=self._get_selected_librevna(),
        )

    def _start_librevna_sweep(self) -> None:
        if self._librevna_sweep_worker and self._librevna_sweep_worker.isRunning():
            self._append_log("LibreVNA sweep already running")
            return

        try:
            config = self._build_librevna_config()
        except Exception as exc:
            self._append_log(f"LibreVNA config invalid: {exc}")
            return

        self._ensure_librevna_ipc()
        payload: Dict[str, object] = {
            "cal_path": config.cal_path,
            "f_start_hz": config.f_start_hz,
            "f_stop_hz": config.f_stop_hz,
            "points": config.points,
            "ifbw_hz": config.ifbw_hz,
            "power_dbm": config.power_dbm,
            "threshold_db": config.threshold_db,
            "timeout_ms": config.timeout_ms,
            "excited_ports": config.excited_ports,
        }
        if config.serial and config.serial.lower() != "auto":
            payload["serial"] = config.serial

        self.librevna_results_table.setRowCount(0)
        self.librevna_mag_plot.clear()
        self.librevna_phase_plot.clear()
        self._librevna_trace = []

        self._append_log(
            "LibreVNA sweep start: "
            f"start={config.f_start_hz} stop={config.f_stop_hz} points={config.points} "
            f"ifbw={config.ifbw_hz} power={config.power_dbm} threshold={config.threshold_db}"
        )
        self.librevna_status_label.setText("Status: Running")
        self._set_librevna_busy(True)

        worker = LibreVNARequestWorker(
            server_name=self._settings.librevna_ipc_name,
            cmd="run_sweep",
            payload=payload,
            timeout_ms=int(config.timeout_ms) + 5000,
        )
        worker.log.connect(self._append_log)
        worker.done.connect(self._handle_librevna_sweep_done)
        worker.failed.connect(self._handle_librevna_sweep_failed)
        self._librevna_sweep_worker = worker
        worker.start()

    def _stop_librevna_sweep(self) -> None:
        if not (self._librevna_sweep_worker and self._librevna_sweep_worker.isRunning()):
            self._append_log("LibreVNA sweep is not running")
            return
        self._append_log("LibreVNA stop requested (not yet supported)")

    def _handle_librevna_sweep_done(self, payload: Dict[str, object]) -> None:
        self._set_librevna_busy(False)
        self.librevna_status_label.setText("Status: Complete")
        self._update_librevna_results(payload)

    def _handle_librevna_sweep_failed(self, message: str) -> None:
        self._set_librevna_busy(False)
        self.librevna_status_label.setText("Status: Failed")
        self._append_log(f"LibreVNA sweep failed: {message}")

    def _update_librevna_plot_visibility(self) -> None:
        self.librevna_mag_group.setVisible(self.librevna_show_mag_check.isChecked())
        self.librevna_phase_group.setVisible(self.librevna_show_phase_check.isChecked())

    def _update_librevna_results(self, payload: Dict[str, object]) -> None:
        overall_pass = bool(payload.get("overall_pass", False))
        self.librevna_pass_label.setText("Overall: PASS" if overall_pass else "Overall: FAIL")
        self.librevna_pass_label.setStyleSheet(
            "font-weight: bold; color: #0a7f2e;" if overall_pass else "font-weight: bold; color: #c1121f;"
        )

        results = payload.get("results", {})
        self.librevna_results_table.setRowCount(0)
        if isinstance(results, dict):
            for param, entry in results.items():
                row = self.librevna_results_table.rowCount()
                self.librevna_results_table.insertRow(row)
                self.librevna_results_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(param)))
                pass_val = bool(entry.get("pass", False)) if isinstance(entry, dict) else False
                self.librevna_results_table.setItem(row, 1, QtWidgets.QTableWidgetItem("PASS" if pass_val else "FAIL"))
                worst_db = entry.get("worst_db") if isinstance(entry, dict) else None
                self.librevna_results_table.setItem(
                    row,
                    2,
                    QtWidgets.QTableWidgetItem(f"{worst_db:.2f}" if isinstance(worst_db, (int, float)) else ""),
                )
                fail_at = entry.get("fail_at_hz") if isinstance(entry, dict) else None
                self.librevna_results_table.setItem(
                    row,
                    3,
                    QtWidgets.QTableWidgetItem(f"{fail_at:.3f}" if isinstance(fail_at, (int, float)) else ""),
                )

        trace = payload.get("trace", [])
        if isinstance(trace, list):
            self._librevna_trace = trace
        else:
            self._librevna_trace = []
        self._update_librevna_plots()

    def _update_librevna_plots(self) -> None:
        if not self._librevna_trace:
            self.librevna_mag_plot.clear()
            self.librevna_phase_plot.clear()
            return

        colors = {
            "S11": QtGui.QColor("#d62728"),
            "S21": QtGui.QColor("#1f77b4"),
            "S12": QtGui.QColor("#2ca02c"),
            "S22": QtGui.QColor("#ff7f0e"),
        }
        selected = [key for key, check in self.librevna_sparam_checks.items() if check.isChecked()]
        if not selected:
            self.librevna_mag_plot.clear()
            self.librevna_phase_plot.clear()
            return

        mag_series: Dict[str, List[Tuple[float, float]]] = {}
        phase_series: Dict[str, List[Tuple[float, float]]] = {}
        for param in selected:
            mag_points: List[Tuple[float, float]] = []
            phase_points: List[Tuple[float, float]] = []
            for entry in self._librevna_trace:
                if not isinstance(entry, dict):
                    continue
                freq = entry.get("frequency")
                data = entry.get(param)
                if not isinstance(data, dict):
                    continue
                real = data.get("real")
                imag = data.get("imag")
                if not isinstance(freq, (int, float)) or not isinstance(real, (int, float)) or not isinstance(imag, (int, float)):
                    continue
                value = complex(real, imag)
                mag = -300.0 if value == 0 else 20.0 * math.log10(abs(value))
                phase = math.degrees(math.atan2(value.imag, value.real))
                mag_points.append((float(freq), float(mag)))
                phase_points.append((float(freq), float(phase)))
            if mag_points:
                mag_series[param] = mag_points
                phase_series[param] = phase_points

        self.librevna_mag_plot.set_series(mag_series, colors)
        self.librevna_phase_plot.set_series(phase_series, colors)

    def _build_config(self) -> CalibrationConfig:
        start_hz = parse_frequency(self.start_freq_edit.text())
        stop_hz = parse_frequency(self.stop_freq_edit.text())
        points = int(self.points_spin.value())
        step_hz = parse_frequency(self.step_edit.text()) if points == 0 else None

        template_path = os.path.join(
            os.path.dirname(script_dir),
            "examples",
            "register-values",
            "HexRegisterValues3600.txt",
        )

        return CalibrationConfig(
            start_hz=start_hz,
            stop_hz=stop_hz,
            step_hz=step_hz,
            points=points if points > 0 else None,
            requested_dbm=float(self.requested_dbm_edit.value()),
            outa_pwr=int(self._settings.outa_pwr),
            wait_lock=self.wait_lock_check.isChecked(),
            delta_update=self.delta_update_check.isChecked(),
            dwell_ms=self._settings.dwell_ms,
            auto_recal=self._settings.auto_recal,
            lock_timeout_s=self._settings.lock_timeout_s,
            fosc_hz=parse_frequency(self._settings.fosc),
            spi_clock_hz=parse_frequency(self._settings.spi_clock),
            template_path=template_path,
            tinysa_debug=self._settings.tinysa_debug,
        )

    def _start_calibration(self) -> None:
        if self._worker and self._worker.isRunning():
            self._append_log("Calibration already running")
            return
        if self._cal_worker and self._cal_worker.isRunning():
            self._append_log("Calibration blocked: tinySA calibration in progress")
            return

        try:
            config = self._build_config()
        except Exception as exc:
            self._append_log(f"Invalid settings: {exc}")
            return

        if not self._settings.worker_python:
            self._append_log("Worker Python path is empty. Set it in Hidden settings.")
            return

        self.results_table.setRowCount(0)
        self.plot_widget.clear()
        self._append_log("Start calibration pressed")
        self._append_log(
            f"Config: start={config.start_hz} stop={config.stop_hz} step={config.step_hz} "
            f"points={config.points} requested_dbm={config.requested_dbm} outa_pwr={config.outa_pwr}"
        )
        self._append_log(
            f"LMX: wait_lock={config.wait_lock} delta_update={config.delta_update} "
            f"dwell_ms={config.dwell_ms} auto_recal={config.auto_recal} lock_timeout={config.lock_timeout_s}"
        )
        self._append_log(f"tinySA debug: {config.tinysa_debug}")
        self._append_log(
            f"Worker Python: {self._settings.worker_python} tinySA port={self._get_selected_port() or 'Auto'} "
            f"USB2ANY serial={self._get_selected_lmx() or 'Auto'}"
        )
        self._append_log("Starting calibration sweep")
        self._set_busy(True)

        self._last_config = config
        self._worker = CalibrationWorker(
            config=config,
            port=self._get_selected_port(),
            lmx_serial=self._get_selected_lmx(),
            worker_python=self._settings.worker_python,
        )
        self._worker.log.connect(self._append_log)
        self._worker.point.connect(self._handle_point)
        self._worker.done.connect(self._handle_done)
        self._worker.failed.connect(self._handle_failed)
        self._worker.start()

    def _handle_point(self, point: CalibrationPoint) -> None:
        row = self.results_table.rowCount()
        self.results_table.insertRow(row)
        self.results_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(point.timestamp)))
        self.results_table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{point.frequency_hz:.3f}"))
        self.results_table.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{point.requested_dbm:.2f}"))
        self.results_table.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{point.measured_dbm:.2f}"))
        self.results_table.setItem(row, 4, QtWidgets.QTableWidgetItem(f"{point.offset_db:.2f}"))
        self.plot_widget.add_point(point.frequency_hz, point.offset_db)

    def _handle_done(self, points: List[CalibrationPoint]) -> None:
        self._append_log(f"Calibration complete: {len(points)} points")
        self._set_busy(False)
        out_path = self.output_edit.text().strip()
        if not out_path:
            stamp = QtCore.QDateTime.currentDateTimeUtc().toString("yyyyMMdd_HHmmss")
            out_path = os.path.join("output", f"calibration_{stamp}.cal")
        try:
            self._save_calibration(out_path, points)
            self._append_log(f"Saved calibration file: {out_path}")
        except Exception as exc:
            self._append_log(f"Failed to save calibration file: {exc}")

    def _handle_failed(self, message: str) -> None:
        self._append_log(f"Calibration failed: {message}")
        self._set_busy(False)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._append_log("GUI close requested, stopping workers")
        self._shutdown_workers()
        event.accept()

    def _shutdown_workers(self) -> None:
        if self._worker and self._worker.isRunning():
            self._append_log("Stopping calibration worker")
            self._worker.request_stop()
            self._worker.wait(1000)
            if self._worker.isRunning():
                self._append_log("Forcing calibration worker shutdown")
                self._worker.terminate()
                self._worker.wait(1000)
        if self._cal_worker and self._cal_worker.isRunning():
            self._append_log("Stopping tinySA calibration worker")
            self._cal_worker.request_stop()
            self._cal_worker.wait(500)
            if self._cal_worker.isRunning():
                self._append_log("Forcing tinySA calibration worker shutdown")
                self._cal_worker.terminate()
                self._cal_worker.wait(500)
        if self._librevna_sweep_worker and self._librevna_sweep_worker.isRunning():
            self._append_log("Stopping LibreVNA sweep worker")
            self._librevna_sweep_worker.terminate()
            self._librevna_sweep_worker.wait(500)
        if self._librevna_process and self._librevna_process.state() != QtCore.QProcess.NotRunning:
            self._append_log("Stopping LibreVNA IPC process")
            try:
                LibreVNAIpcClient(self._settings.librevna_ipc_name).request({"cmd": "shutdown"}, timeout_ms=2000)
            except Exception:
                pass
            self._librevna_process.terminate()
            self._librevna_process.waitForFinished(2000)

    def _save_calibration(self, path: str, points: List[CalibrationPoint]) -> None:
        config = self._last_config or self._build_config()
        metadata = {
            "type": "lmx2594_calibration",
            "created_at": _utc_now(),
            "start_hz": config.start_hz,
            "stop_hz": config.stop_hz,
            "step_hz": config.step_hz,
            "points": len(points),
            "requested_dbm": config.requested_dbm,
            "outa_pwr": config.outa_pwr,
            "fosc_hz": config.fosc_hz,
            "spi_clock_hz": config.spi_clock_hz,
            "wait_lock": config.wait_lock,
            "delta_update": config.delta_update,
            "dwell_ms": config.dwell_ms,
            "auto_recal": config.auto_recal,
            "lock_timeout_s": config.lock_timeout_s,
        }
        payload = {
            "metadata": metadata,
            "measurements": [p.as_dict() for p in points],
        }
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)

    def _load_calibration(self) -> None:
        self._append_log("Load calibration file pressed")
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open calibration file", "", "Calibration Files (*.cal *.json)"
        )
        if not path:
            self._append_log("Load calibration canceled")
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception as exc:
            self._append_log(f"Failed to load calibration file: {exc}")
            return

        measurements = data.get("measurements", [])
        self.results_table.setRowCount(0)
        self.plot_widget.clear()
        for entry in measurements:
            try:
                point = CalibrationPoint(
                    timestamp=str(entry.get("timestamp", "")),
                    frequency_hz=float(entry.get("frequency_hz", 0.0)),
                    requested_dbm=float(entry.get("requested_dbm", 0.0)),
                    measured_dbm=float(entry.get("measured_dbm", 0.0)),
                    offset_db=float(entry.get("offset_db", 0.0)),
                )
            except Exception:
                continue
            self._handle_point(point)
        self._append_log(f"Loaded calibration file: {path}")


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = CalibrationWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
