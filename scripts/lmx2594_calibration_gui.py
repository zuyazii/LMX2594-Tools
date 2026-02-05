#!/usr/bin/env python3
"""
64-bit GUI for LMX2594 calibration using tinySA (local) + USB2ANY worker (32-bit).
"""
from __future__ import annotations

import json
import math
import os
import re
import struct
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)
root_dir = os.path.dirname(script_dir)
CONFIG_PATH = os.path.join(root_dir, ".lmx_gui_config.json")
DEFAULT_LIBREVNA_IPC_CANDIDATES = [
    os.path.join(root_dir, "scripts", "vna", "cpp", "build", "librevna-ipc.exe"),
    os.path.join(root_dir, "scripts", "vna", "cpp", "build", "Release", "librevna-ipc.exe"),
    os.path.join(root_dir, "scripts", "vna", "cpp", "build", "RelWithDebInfo", "librevna-ipc.exe"),
    os.path.join(root_dir, "scripts", "vna", "cpp", "build", "Debug", "librevna-ipc.exe"),
]
MSYS2_UCRT64_BIN = os.environ.get("MSYS2_UCRT64_BIN", r"C:\msys64\ucrt64\bin")
MSYS2_QT_PLUGIN_PATH = os.environ.get("MSYS2_QT_PLUGIN_PATH", r"C:\msys64\ucrt64\share\qt6\plugins")
SYSTEM_ROOT = os.environ.get("SystemRoot", r"C:\Windows")
WINDOWS_SYSTEM32 = os.path.join(SYSTEM_ROOT, "System32")
IPC_SANITIZE_ENV = os.environ.get("LIBREVNA_IPC_SANITIZE_PATH", "1")
IPC_NAME_DEFAULT = "librevna-ipc"
_IPC_NAME_INVALID_CHARS = re.compile(r"[^A-Za-z0-9.-]+")


def normalize_ipc_name(value: str) -> str:
    name = (value or "").strip()
    if not name:
        return IPC_NAME_DEFAULT
    normalized = _IPC_NAME_INVALID_CHARS.sub("-", name)
    normalized = re.sub(r"-{2,}", "-", normalized).strip("-")
    return normalized or IPC_NAME_DEFAULT

if struct.calcsize("P") * 8 != 64:
    raise RuntimeError("This GUI must be run with 64-bit Python.")

try:
    from PySide6 import QtCore, QtGui, QtWidgets, QtNetwork
    Signal = QtCore.Signal
except ImportError:  # pragma: no cover - fallback only
    from PyQt5 import QtCore, QtGui, QtWidgets, QtNetwork  # type: ignore
    Signal = QtCore.pyqtSignal  # type: ignore

from enum import Enum, auto

from tinysa_antenna_test import (  # noqa: E402
    TinySAController,
    TinySAError,
    build_frequency_points,
    list_serial_ports,
    parse_frequency,
)


class WorkflowState(Enum):
    """State machine for guided calibration workflow."""
    IDLE = auto()
    DEVICE_SELECTION = auto()
    # Calibration Phase
    LIBREVNA_CALIBRATION = auto()
    TINYSA_CALIBRATION = auto()
    REFERENCE_ANTENNA_MEASUREMENT = auto()
    LMX_CALIBRATION = auto()
    LMX_CALIBRATION_COMPLETE = auto()
    # Golden Sample Phase
    ANTENNA_MEASUREMENT_LOOP = auto()
    GOLDEN_SAMPLE_SELECTION = auto()
    # Legacy states (kept for compatibility)
    ANTENNA_MEASUREMENT = auto()
    ANTENNA_COMPLETE = auto()
    GOLDEN_SAMPLE_CHECK = auto()
    # Production Testing Phase
    PRODUCTION_TESTING = auto()
    # Terminal states
    WORKFLOW_COMPLETE = auto()
    WORKFLOW_CANCELLED = auto()


class DeviceMode(Enum):
    """Mutually exclusive device mode selection."""
    NONE = auto()
    VNA = auto()
    SA = auto()


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
class AntennaMeasurement:
    """Single antenna measurement with extended tinySA characteristics."""
    id: str
    timestamp: str
    label: str
    frequency_hz: float
    gain_db: float
    transmitted_power_dbm: float
    received_power_dbm: float
    peak_frequency_hz: Optional[float] = None
    bandwidth_hz: Optional[float] = None
    noise_floor_dbm: Optional[float] = None
    is_golden_sample: bool = False
    pass_fail: Optional[str] = None
    offset_from_golden_db: Optional[float] = None

    def as_dict(self) -> Dict[str, object]:
        return {
            "id": self.id,
            "timestamp": self.timestamp,
            "label": self.label,
            "frequency_hz": self.frequency_hz,
            "gain_db": self.gain_db,
            "transmitted_power_dbm": self.transmitted_power_dbm,
            "received_power_dbm": self.received_power_dbm,
            "peak_frequency_hz": self.peak_frequency_hz,
            "bandwidth_hz": self.bandwidth_hz,
            "noise_floor_dbm": self.noise_floor_dbm,
            "is_golden_sample": self.is_golden_sample,
            "pass_fail": self.pass_fail,
            "offset_from_golden_db": self.offset_from_golden_db,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "AntennaMeasurement":
        return cls(
            id=str(data.get("id", "")),
            timestamp=str(data.get("timestamp", "")),
            label=str(data.get("label", "")),
            frequency_hz=float(data.get("frequency_hz", 0.0)),
            gain_db=float(data.get("gain_db", 0.0)),
            transmitted_power_dbm=float(data.get("transmitted_power_dbm", 0.0)),
            received_power_dbm=float(data.get("received_power_dbm", 0.0)),
            peak_frequency_hz=float(data["peak_frequency_hz"]) if data.get("peak_frequency_hz") else None,
            bandwidth_hz=float(data["bandwidth_hz"]) if data.get("bandwidth_hz") else None,
            noise_floor_dbm=float(data["noise_floor_dbm"]) if data.get("noise_floor_dbm") else None,
            is_golden_sample=bool(data.get("is_golden_sample", False)),
            pass_fail=str(data["pass_fail"]) if data.get("pass_fail") else None,
            offset_from_golden_db=float(data["offset_from_golden_db"]) if data.get("offset_from_golden_db") is not None else None,
        )


@dataclass
class GoldenSampleConfig:
    """Configuration for golden sample comparison."""
    target_gain_db: float = -60.0
    tolerance_db: float = 3.0
    selected_sample_id: Optional[str] = None
    selected_sample: Optional[AntennaMeasurement] = None


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
    # Workflow calibration paths
    tinysa_cal_path: str = ""
    librevna_cal_path: str = ""
    golden_gain_threshold: float = 60.0


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
        self._server_name = normalize_ipc_name(server_name)

    def request(self, payload: Dict[str, object], timeout_ms: int = 5000) -> Dict[str, object]:
        if not self._server_name:
            raise LibreVNAIpcError("IPC server name not set")
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
            self.log.emit(f"LibreVNA IPC -> {self._cmd} (name={self._server_name!r})")
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


class GoldenSamplePanel(QtWidgets.QWidget):
    """Panel for displaying and selecting golden samples."""
    
    sample_selected = Signal(str)  # Emits sample ID when selected
    sample_confirmed = Signal(object)  # Emits AntennaMeasurement when confirmed
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._candidates: List[AntennaMeasurement] = []
        self._selected_id: Optional[str] = None
        self._setup_ui()
    
    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        
        # Target gain and tolerance inputs
        config_layout = QtWidgets.QHBoxLayout()
        config_layout.addWidget(QtWidgets.QLabel("Target Gain:"))
        self.target_gain_spin = QtWidgets.QDoubleSpinBox()
        self.target_gain_spin.setRange(-200.0, 50.0)
        self.target_gain_spin.setDecimals(1)
        self.target_gain_spin.setValue(-60.0)
        self.target_gain_spin.setSuffix(" dB")
        config_layout.addWidget(self.target_gain_spin)
        
        config_layout.addSpacing(20)
        config_layout.addWidget(QtWidgets.QLabel("Tolerance:"))
        self.tolerance_spin = QtWidgets.QDoubleSpinBox()
        self.tolerance_spin.setRange(0.1, 50.0)
        self.tolerance_spin.setDecimals(1)
        self.tolerance_spin.setValue(3.0)
        self.tolerance_spin.setPrefix("+/- ")
        self.tolerance_spin.setSuffix(" dB")
        config_layout.addWidget(self.tolerance_spin)
        config_layout.addStretch(1)
        layout.addLayout(config_layout)
        
        # Candidates list
        candidates_group = QtWidgets.QGroupBox("Candidates (gain within threshold)")
        candidates_layout = QtWidgets.QVBoxLayout(candidates_group)
        self.candidates_list = QtWidgets.QListWidget()
        self.candidates_list.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidates_list.itemClicked.connect(self._on_candidate_clicked)
        candidates_layout.addWidget(self.candidates_list)
        layout.addWidget(candidates_group)
        
        # Selected sample characteristics
        char_group = QtWidgets.QGroupBox("Selected Sample Characteristics")
        char_layout = QtWidgets.QFormLayout(char_group)
        self.char_gain_label = QtWidgets.QLabel("-")
        self.char_tx_power_label = QtWidgets.QLabel("-")
        self.char_rx_power_label = QtWidgets.QLabel("-")
        self.char_peak_freq_label = QtWidgets.QLabel("-")
        self.char_bandwidth_label = QtWidgets.QLabel("-")
        self.char_noise_floor_label = QtWidgets.QLabel("-")
        char_layout.addRow("Gain:", self.char_gain_label)
        char_layout.addRow("Tx Power:", self.char_tx_power_label)
        char_layout.addRow("Rx Power:", self.char_rx_power_label)
        char_layout.addRow("Peak Freq:", self.char_peak_freq_label)
        char_layout.addRow("Bandwidth:", self.char_bandwidth_label)
        char_layout.addRow("Noise Floor:", self.char_noise_floor_label)
        layout.addWidget(char_group)
        
        # Confirm button
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addStretch(1)
        self.confirm_button = QtWidgets.QPushButton("Confirm Golden Sample")
        self.confirm_button.setEnabled(False)
        self.confirm_button.clicked.connect(self._on_confirm_clicked)
        button_layout.addWidget(self.confirm_button)
        layout.addLayout(button_layout)
    
    def add_candidate(self, measurement: AntennaMeasurement) -> None:
        """Add a measurement as a candidate if within tolerance."""
        target = self.target_gain_spin.value()
        tolerance = self.tolerance_spin.value()
        
        if abs(measurement.gain_db - target) <= tolerance:
            self._candidates.append(measurement)
            self._update_candidates_list()
    
    def clear_candidates(self) -> None:
        """Clear all candidates."""
        self._candidates = []
        self._selected_id = None
        self.candidates_list.clear()
        self._clear_characteristics()
        self.confirm_button.setEnabled(False)
    
    def _update_candidates_list(self) -> None:
        self.candidates_list.clear()
        for m in self._candidates:
            text = f"{m.label} | Gain: {m.gain_db:.1f} dB | {m.timestamp}"
            item = QtWidgets.QListWidgetItem(text)
            item.setData(QtCore.Qt.UserRole, m.id)
            if m.id == self._selected_id:
                item.setText(text + "  [SELECTED]")
                font = item.font()
                font.setBold(True)
                item.setFont(font)
            self.candidates_list.addItem(item)
    
    def _on_candidate_clicked(self, item: QtWidgets.QListWidgetItem) -> None:
        sample_id = item.data(QtCore.Qt.UserRole)
        self._selected_id = sample_id
        self._update_candidates_list()
        self._update_characteristics()
        self.confirm_button.setEnabled(True)
        self.sample_selected.emit(sample_id)
    
    def _update_characteristics(self) -> None:
        measurement = self._get_selected_measurement()
        if not measurement:
            self._clear_characteristics()
            return
        
        self.char_gain_label.setText(f"{measurement.gain_db:.2f} dB")
        self.char_tx_power_label.setText(f"{measurement.transmitted_power_dbm:.2f} dBm")
        self.char_rx_power_label.setText(f"{measurement.received_power_dbm:.2f} dBm")
        
        if measurement.peak_frequency_hz:
            self.char_peak_freq_label.setText(f"{measurement.peak_frequency_hz / 1e9:.4f} GHz")
        else:
            self.char_peak_freq_label.setText("-")
        
        if measurement.bandwidth_hz:
            self.char_bandwidth_label.setText(f"{measurement.bandwidth_hz / 1e6:.2f} MHz")
        else:
            self.char_bandwidth_label.setText("-")
        
        if measurement.noise_floor_dbm:
            self.char_noise_floor_label.setText(f"{measurement.noise_floor_dbm:.2f} dBm")
        else:
            self.char_noise_floor_label.setText("-")
    
    def _clear_characteristics(self) -> None:
        self.char_gain_label.setText("-")
        self.char_tx_power_label.setText("-")
        self.char_rx_power_label.setText("-")
        self.char_peak_freq_label.setText("-")
        self.char_bandwidth_label.setText("-")
        self.char_noise_floor_label.setText("-")
    
    def _get_selected_measurement(self) -> Optional[AntennaMeasurement]:
        if not self._selected_id:
            return None
        for m in self._candidates:
            if m.id == self._selected_id:
                return m
        return None
    
    def _on_confirm_clicked(self) -> None:
        measurement = self._get_selected_measurement()
        if measurement:
            measurement.is_golden_sample = True
            self.sample_confirmed.emit(measurement)


class AntennaResultsWidget(QtWidgets.QWidget):
    """Scrollable widget for displaying antenna test results with PASS/FAIL status."""
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._measurements: List[AntennaMeasurement] = []
        self._golden_sample: Optional[AntennaMeasurement] = None
        self._tolerance_db: float = 3.0
        self._setup_ui()
    
    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        
        # Header with golden sample info
        header_layout = QtWidgets.QHBoxLayout()
        self.golden_info_label = QtWidgets.QLabel("Golden Sample: Not set")
        self.golden_info_label.setStyleSheet("font-weight: bold;")
        header_layout.addWidget(self.golden_info_label)
        header_layout.addStretch(1)
        layout.addLayout(header_layout)
        
        # Results table
        self.results_table = QtWidgets.QTableWidget(0, 6)
        self.results_table.setHorizontalHeaderLabels([
            "#", "Label", "Gain (dB)", "Offset", "Status", "Timestamp"
        ])
        self.results_table.horizontalHeader().setStretchLastSection(True)
        self.results_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.results_table.setAlternatingRowColors(True)
        layout.addWidget(self.results_table)
        
        # Summary row
        self.summary_label = QtWidgets.QLabel("Summary: 0 tested | 0 PASS | 0 FAIL")
        layout.addWidget(self.summary_label)
        
        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        self.export_button = QtWidgets.QPushButton("Export Results")
        self.export_button.clicked.connect(self._on_export_clicked)
        self.clear_button = QtWidgets.QPushButton("Clear")
        self.clear_button.clicked.connect(self._on_clear_clicked)
        button_layout.addStretch(1)
        button_layout.addWidget(self.export_button)
        button_layout.addWidget(self.clear_button)
        layout.addLayout(button_layout)
    
    def set_golden_sample(self, sample: AntennaMeasurement, tolerance_db: float) -> None:
        """Set the golden sample for comparison."""
        self._golden_sample = sample
        self._tolerance_db = tolerance_db
        self.golden_info_label.setText(
            f"Golden Sample: {sample.label} ({sample.gain_db:.1f} dB) | Tolerance: +/- {tolerance_db:.1f} dB"
        )
        # Re-evaluate all existing measurements
        self._reevaluate_all()
    
    def add_measurement(self, measurement: AntennaMeasurement) -> None:
        """Add a measurement and evaluate against golden sample."""
        if self._golden_sample:
            offset = measurement.gain_db - self._golden_sample.gain_db
            measurement.offset_from_golden_db = offset
            measurement.pass_fail = "PASS" if abs(offset) <= self._tolerance_db else "FAIL"
        
        self._measurements.append(measurement)
        self._add_table_row(measurement, len(self._measurements))
        self._update_summary()
    
    def _add_table_row(self, m: AntennaMeasurement, row_num: int) -> None:
        row = self.results_table.rowCount()
        self.results_table.insertRow(row)
        
        self.results_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(row_num)))
        self.results_table.setItem(row, 1, QtWidgets.QTableWidgetItem(m.label))
        self.results_table.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{m.gain_db:.2f}"))
        
        offset_text = f"{m.offset_from_golden_db:+.2f}" if m.offset_from_golden_db is not None else "-"
        self.results_table.setItem(row, 3, QtWidgets.QTableWidgetItem(offset_text))
        
        status_item = QtWidgets.QTableWidgetItem(m.pass_fail or "-")
        if m.pass_fail == "PASS":
            status_item.setForeground(QtGui.QColor("#0a7f2e"))
        elif m.pass_fail == "FAIL":
            status_item.setForeground(QtGui.QColor("#c1121f"))
        self.results_table.setItem(row, 4, status_item)
        
        self.results_table.setItem(row, 5, QtWidgets.QTableWidgetItem(m.timestamp))
    
    def _reevaluate_all(self) -> None:
        """Re-evaluate all measurements against current golden sample."""
        if not self._golden_sample:
            return
        
        self.results_table.setRowCount(0)
        for i, m in enumerate(self._measurements):
            offset = m.gain_db - self._golden_sample.gain_db
            m.offset_from_golden_db = offset
            m.pass_fail = "PASS" if abs(offset) <= self._tolerance_db else "FAIL"
            self._add_table_row(m, i + 1)
        self._update_summary()
    
    def _update_summary(self) -> None:
        total = len(self._measurements)
        passed = sum(1 for m in self._measurements if m.pass_fail == "PASS")
        failed = sum(1 for m in self._measurements if m.pass_fail == "FAIL")
        rate = (passed / total * 100) if total > 0 else 0
        self.summary_label.setText(
            f"Summary: {total} tested | {passed} PASS ({rate:.1f}%) | {failed} FAIL"
        )
    
    def clear(self) -> None:
        """Clear all measurements."""
        self._measurements = []
        self.results_table.setRowCount(0)
        self._update_summary()
    
    def _on_clear_clicked(self) -> None:
        reply = QtWidgets.QMessageBox.question(
            self, "Clear Results",
            "Are you sure you want to clear all test results?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.clear()
    
    def _on_export_clicked(self) -> None:
        if not self._measurements:
            QtWidgets.QMessageBox.information(self, "Export", "No measurements to export.")
            return
        
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Results", "", "JSON Files (*.json);;CSV Files (*.csv)"
        )
        if not path:
            return
        
        if path.endswith(".csv"):
            self._export_csv(path)
        else:
            self._export_json(path)
    
    def _export_json(self, path: str) -> None:
        data = {
            "type": "antenna_test_session",
            "created_at": _utc_now(),
            "golden_sample": self._golden_sample.as_dict() if self._golden_sample else None,
            "tolerance_db": self._tolerance_db,
            "measurements": [m.as_dict() for m in self._measurements],
            "summary": {
                "total": len(self._measurements),
                "passed": sum(1 for m in self._measurements if m.pass_fail == "PASS"),
                "failed": sum(1 for m in self._measurements if m.pass_fail == "FAIL"),
            }
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    
    def _export_csv(self, path: str) -> None:
        import csv
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["#", "Label", "Gain (dB)", "Offset", "Status", "Timestamp"])
            for i, m in enumerate(self._measurements):
                offset = f"{m.offset_from_golden_db:+.2f}" if m.offset_from_golden_db is not None else ""
                writer.writerow([i + 1, m.label, f"{m.gain_db:.2f}", offset, m.pass_fail or "", m.timestamp])
    
    def get_measurements(self) -> List[AntennaMeasurement]:
        return list(self._measurements)


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
        self._settings.librevna_ipc_name = normalize_ipc_name(self.librevna_ipc_name_edit.text())
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


class AntennaMeasurementWorker(QtCore.QThread):
    """Worker thread for single antenna measurement with extended characteristics."""
    log = Signal(str)
    done = Signal(object)  # Emits AntennaMeasurement
    failed = Signal(str)

    def __init__(
        self,
        port: Optional[str],
        frequency_hz: float,
        tx_power_dbm: float,
        label: str,
        debug: bool = False,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._port = port
        self._frequency_hz = frequency_hz
        self._tx_power_dbm = tx_power_dbm
        self._label = label
        self._debug = debug

    def run(self) -> None:
        self.log.emit(f"Connecting to tinySA for antenna measurement")
        tinysa = TinySAController(
            self._port,
            verbose=False,
            error_byte=False,
            debug_cb=self.log.emit if self._debug else None,
        )
        try:
            tinysa.connect()
            self.log.emit(f"Measuring at {self._frequency_hz/1e9:.3f} GHz")
            
            # Try extended measurement first
            try:
                ext_data = tinysa.measure_extended(
                    self._frequency_hz,
                    span_hz=10e6,
                    points=51,
                    timeout_s=10.0,
                )
                rx_power = ext_data["power_dbm"]
                peak_freq = ext_data.get("peak_frequency_hz")
                bandwidth = ext_data.get("bandwidth_hz")
                noise_floor = ext_data.get("noise_floor_dbm")
            except Exception as exc:
                self.log.emit(f"Extended measurement failed, using simple: {exc}")
                rx_power = tinysa.scan_point(self._frequency_hz, timeout_s=5.0)
                peak_freq = None
                bandwidth = None
                noise_floor = None
            
            gain_db = rx_power - self._tx_power_dbm
            
            measurement = AntennaMeasurement(
                id=str(uuid.uuid4()),
                timestamp=_utc_now(),
                label=self._label,
                frequency_hz=self._frequency_hz,
                gain_db=gain_db,
                transmitted_power_dbm=self._tx_power_dbm,
                received_power_dbm=rx_power,
                peak_frequency_hz=peak_freq,
                bandwidth_hz=bandwidth,
                noise_floor_dbm=noise_floor,
            )
            
            self.log.emit(f"Measurement complete: {self._label} = {gain_db:.2f} dB")
            self.done.emit(measurement)
            
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            try:
                tinysa.disconnect()
            except Exception:
                pass


class CalibrationWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("LMX2594 + LibreVNA Calibration (64-bit GUI)")
        self.resize(1400, 900)

        self._settings = HiddenSettings()
        self._load_settings_from_disk()
        self._worker: Optional[CalibrationWorker] = None
        self._cal_worker: Optional[CalibrateTinySAWorker] = None
        self._antenna_measure_worker: Optional[AntennaMeasurementWorker] = None
        self._librevna_device_worker: Optional[LibreVNARequestWorker] = None
        self._librevna_sweep_worker: Optional[LibreVNARequestWorker] = None
        self._last_config: Optional[CalibrationConfig] = None
        self._busy = False
        self._librevna_busy = False
        self._suppress_device_save = False
        self._librevna_process: Optional[QtCore.QProcess] = None
        self._librevna_trace: List[Dict[str, object]] = []
        self._closing = False
        self._workflow_state = WorkflowState.IDLE
        self._device_mode = DeviceMode.NONE
        self._workflow_results: Dict[str, object] = {}

        central = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(central)

        main_layout.addWidget(self._build_workflow_status_bar())

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
        self._update_device_panel_visibility()
        self._update_controls()
        self._apply_default_librevna_ipc_path()
        self._apply_saved_calibration_paths()
        self._refresh_devices()

    def _build_workflow_status_bar(self) -> QtWidgets.QFrame:
        frame = QtWidgets.QFrame()
        frame.setFrameStyle(QtWidgets.QFrame.StyledPanel | QtWidgets.QFrame.Raised)
        main_layout = QtWidgets.QVBoxLayout(frame)
        main_layout.setContentsMargins(8, 4, 8, 4)
        main_layout.setSpacing(4)

        # Top row: Phase and step label
        top_layout = QtWidgets.QHBoxLayout()
        self.workflow_step_label = QtWidgets.QLabel("Workflow: Idle")
        font = self.workflow_step_label.font()
        font.setBold(True)
        font.setPointSize(font.pointSize() + 1)
        self.workflow_step_label.setFont(font)
        top_layout.addWidget(self.workflow_step_label)
        top_layout.addStretch(1)
        main_layout.addLayout(top_layout)

        # Middle row: Checklist indicators
        checklist_layout = QtWidgets.QHBoxLayout()
        self.check_librevna_cal = QtWidgets.QLabel("[ ] LibreVNA Cal")
        self.check_tinysa_cal = QtWidgets.QLabel("[ ] tinySA Cal")
        self.check_ref_antenna = QtWidgets.QLabel("[ ] Reference Antenna")
        self.check_golden_sample = QtWidgets.QLabel("[ ] Golden Sample")
        for check in [self.check_librevna_cal, self.check_tinysa_cal, 
                      self.check_ref_antenna, self.check_golden_sample]:
            check.setStyleSheet("color: gray;")
            checklist_layout.addWidget(check)
        checklist_layout.addStretch(1)
        main_layout.addLayout(checklist_layout)

        # Bottom row: Instruction and buttons
        bottom_layout = QtWidgets.QHBoxLayout()
        self.workflow_instruction_label = QtWidgets.QLabel("")
        self.workflow_instruction_label.setStyleSheet("color: gray; font-style: italic;")

        self.start_workflow_button = QtWidgets.QPushButton("Start Workflow")
        self.start_workflow_button.clicked.connect(self._start_guided_workflow)
        self.next_step_button = QtWidgets.QPushButton("Next Step")
        self.next_step_button.setEnabled(False)
        self.next_step_button.clicked.connect(self._advance_workflow)
        self.cancel_workflow_button = QtWidgets.QPushButton("Cancel Workflow")
        self.cancel_workflow_button.setEnabled(False)
        self.cancel_workflow_button.clicked.connect(self._cancel_workflow)

        bottom_layout.addWidget(self.workflow_instruction_label, 1)
        bottom_layout.addWidget(self.start_workflow_button)
        bottom_layout.addWidget(self.next_step_button)
        bottom_layout.addWidget(self.cancel_workflow_button)
        main_layout.addLayout(bottom_layout)
        
        return frame

    def _build_device_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Device Finder and Selector")
        layout = QtWidgets.QGridLayout(group)

        # Device mode selection (VNA or SA)
        mode_layout = QtWidgets.QHBoxLayout()
        self.vna_radio = QtWidgets.QRadioButton("VNA Mode")
        self.sa_radio = QtWidgets.QRadioButton("SA Mode")
        self.device_mode_group = QtWidgets.QButtonGroup(self)
        self.device_mode_group.addButton(self.vna_radio, 1)
        self.device_mode_group.addButton(self.sa_radio, 2)
        self.device_mode_group.buttonClicked.connect(self._on_device_mode_changed)
        mode_layout.addWidget(QtWidgets.QLabel("Measurement Mode:"))
        mode_layout.addWidget(self.vna_radio)
        mode_layout.addWidget(self.sa_radio)
        mode_layout.addStretch(1)
        layout.addLayout(mode_layout, 0, 0, 1, 3)

        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.refresh_button.clicked.connect(self._refresh_devices)

        self.tinysa_combo = QtWidgets.QComboBox()
        self.lmx_combo = QtWidgets.QComboBox()
        self.librevna_combo = QtWidgets.QComboBox()
        self.tinysa_combo.currentIndexChanged.connect(self._on_device_selection_changed)
        self.lmx_combo.currentIndexChanged.connect(self._on_device_selection_changed)
        self.librevna_combo.currentIndexChanged.connect(self._on_device_selection_changed)

        # tinySA row (for SA mode)
        self.tinysa_label = QtWidgets.QLabel("tinySA Port")
        layout.addWidget(self.tinysa_label, 1, 0)
        layout.addWidget(self.tinysa_combo, 1, 1)

        # USB2ANY row (always shown for LMX calibration)
        layout.addWidget(QtWidgets.QLabel("USB2ANY Serial"), 2, 0)
        layout.addWidget(self.lmx_combo, 2, 1)

        # LibreVNA row (for VNA mode)
        self.librevna_label = QtWidgets.QLabel("LibreVNA")
        layout.addWidget(self.librevna_label, 3, 0)
        layout.addWidget(self.librevna_combo, 3, 1)

        layout.addWidget(self.refresh_button, 1, 2, 3, 1)
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

        self.lmx_workflow_hint = QtWidgets.QLabel("")
        self.lmx_workflow_hint.setStyleSheet("color: #b58900; font-style: italic;")
        self.lmx_workflow_hint.setVisible(False)

        layout.addWidget(self.calibrate_tinysa_button, 0, 0)
        layout.addWidget(self.start_button, 0, 1)
        layout.addWidget(self.load_button, 0, 2)
        layout.addWidget(QtWidgets.QLabel("Output .cal"), 1, 0)
        layout.addWidget(self.output_edit, 1, 1)
        layout.addWidget(browse, 1, 2)
        layout.addWidget(self.lmx_workflow_hint, 2, 0, 1, 3)
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
        self.librevna_timeout_spin.setValue(60000.0)

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

        self.librevna_workflow_hint = QtWidgets.QLabel("")
        self.librevna_workflow_hint.setStyleSheet("color: #b58900; font-style: italic;")
        self.librevna_workflow_hint.setVisible(False)
        layout.addWidget(self.librevna_workflow_hint)
        return group

    def _build_results_group(self) -> QtWidgets.QWidget:
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._build_lmx_results_tab(), "LMX2594 Results")
        tabs.addTab(self._build_librevna_results_tab(), "LibreVNA Results")
        tabs.addTab(self._build_antenna_testing_tab(), "Antenna Testing")
        self.results_tabs = tabs
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

    def _build_antenna_testing_tab(self) -> QtWidgets.QWidget:
        """Build the Antenna Testing tab with golden sample panel and results."""
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        
        # Splitter for golden sample panel and results
        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        
        # Golden sample panel (top)
        golden_group = QtWidgets.QGroupBox("Golden Sample Selection")
        golden_layout = QtWidgets.QVBoxLayout(golden_group)
        self.golden_sample_panel = GoldenSamplePanel()
        self.golden_sample_panel.sample_confirmed.connect(self._on_golden_sample_confirmed)
        golden_layout.addWidget(self.golden_sample_panel)
        splitter.addWidget(golden_group)
        
        # Antenna results (bottom)
        results_group = QtWidgets.QGroupBox("Antenna Test Results")
        results_layout = QtWidgets.QVBoxLayout(results_group)
        self.antenna_results_widget = AntennaResultsWidget()
        results_layout.addWidget(self.antenna_results_widget)
        splitter.addWidget(results_group)
        
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter)
        
        # Measurement controls
        controls_layout = QtWidgets.QHBoxLayout()
        
        self.antenna_label_edit = QtWidgets.QLineEdit()
        self.antenna_label_edit.setPlaceholderText("Antenna label (e.g., Antenna #1)")
        controls_layout.addWidget(QtWidgets.QLabel("Label:"))
        controls_layout.addWidget(self.antenna_label_edit)
        
        self.measure_antenna_button = QtWidgets.QPushButton("Measure Antenna")
        self.measure_antenna_button.clicked.connect(self._on_measure_antenna_clicked)
        controls_layout.addWidget(self.measure_antenna_button)
        
        self.save_session_button = QtWidgets.QPushButton("Save Session")
        self.save_session_button.clicked.connect(self._on_save_session_clicked)
        controls_layout.addWidget(self.save_session_button)
        
        self.load_session_button = QtWidgets.QPushButton("Load Session")
        self.load_session_button.clicked.connect(self._on_load_session_clicked)
        controls_layout.addWidget(self.load_session_button)
        
        layout.addLayout(controls_layout)
        
        return container

    def _build_log_group(self) -> QtWidgets.QWidget:
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        event_group = QtWidgets.QGroupBox("Event Log")
        event_layout = QtWidgets.QVBoxLayout(event_group)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        event_layout.addWidget(self.log_view)

        ipc_group = QtWidgets.QGroupBox("LibreVNA IPC Log")
        ipc_layout = QtWidgets.QVBoxLayout(ipc_group)
        self.ipc_log_view = QtWidgets.QPlainTextEdit()
        self.ipc_log_view.setReadOnly(True)
        ipc_layout.addWidget(self.ipc_log_view)

        splitter.addWidget(event_group)
        splitter.addWidget(ipc_group)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        return splitter

    def _append_log(self, message: str) -> None:
        stamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss")
        if self._closing or not hasattr(self, "log_view"):
            return
        try:
            self.log_view.appendPlainText(f"[{stamp}] {message}")
        except RuntimeError:
            return

    def _append_ipc_log(self, message: str) -> None:
        if self._closing or not hasattr(self, "ipc_log_view"):
            return
        try:
            self.ipc_log_view.appendPlainText(message)
        except RuntimeError:
            return

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

    def _on_device_mode_changed(self, button: QtWidgets.QAbstractButton) -> None:
        if button == self.vna_radio:
            self._device_mode = DeviceMode.VNA
            self._append_log("Device mode changed to VNA")
            self._ensure_librevna_ipc()
        elif button == self.sa_radio:
            self._device_mode = DeviceMode.SA
            self._append_log("Device mode changed to SA")
        else:
            self._device_mode = DeviceMode.NONE
        self._update_device_panel_visibility()

    def _update_device_panel_visibility(self) -> None:
        vna_mode = self._device_mode == DeviceMode.VNA
        sa_mode = self._device_mode == DeviceMode.SA
        # tinySA is used for SA mode and for LMX calibration
        self.tinysa_label.setVisible(True)
        self.tinysa_combo.setVisible(True)
        # LibreVNA only visible in VNA mode
        self.librevna_label.setVisible(vna_mode)
        self.librevna_combo.setVisible(vna_mode)
        # Update parameter panels visibility
        if hasattr(self, "librevna_mag_group"):
            self.librevna_mag_group.setVisible(vna_mode and self.librevna_show_mag_check.isChecked())
            self.librevna_phase_group.setVisible(vna_mode and self.librevna_show_phase_check.isChecked())

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

        requested_name = self._settings.librevna_ipc_name
        sanitized_name = normalize_ipc_name(requested_name)
        if sanitized_name != requested_name:
            self._append_log(
                f"Adjusted LibreVNA IPC name from {requested_name!r} to {sanitized_name!r}"
            )
            self._settings.librevna_ipc_name = sanitized_name
        process = QtCore.QProcess(self)
        env = QtCore.QProcessEnvironment.systemEnvironment()
        sanitize = IPC_SANITIZE_ENV.strip() != "0"
        if sanitize:
            path_parts = [MSYS2_UCRT64_BIN, os.path.dirname(binary), WINDOWS_SYSTEM32, SYSTEM_ROOT]
            path = ";".join([p for p in path_parts if p and os.path.isdir(p)])
            env.insert("PATH", path)
            self._append_log(f"LibreVNA IPC PATH sanitized: {path}")
        else:
            self._prepend_env_path(env, os.path.dirname(binary))
            self._prepend_env_path(env, MSYS2_UCRT64_BIN)
            self._append_log("LibreVNA IPC PATH inherited")
        if MSYS2_QT_PLUGIN_PATH and os.path.isdir(MSYS2_QT_PLUGIN_PATH):
            env.insert("QT_PLUGIN_PATH", MSYS2_QT_PLUGIN_PATH)
            env.insert("QT_QPA_PLATFORM_PLUGIN_PATH", MSYS2_QT_PLUGIN_PATH)
        if self._settings.librevna_ipc_name:
            env.insert("LIBREVNA_IPC_NAME", self._settings.librevna_ipc_name)
        env.insert("LIBREVNA_USB_BULK_TIMEOUT_MS", "3000")
        env.insert("LIBREVNA_IPC_LOG", os.path.join(root_dir, "librevna-ipc.log"))
        env.insert("LIBREVNA_DISABLE_PACKET_LOG", "1")
        self._append_log(
            f"Launching LibreVNA IPC: {binary} (name={self._settings.librevna_ipc_name!r})"
        )
        process.setProcessEnvironment(env)
        process.setProgram(binary)
        process.setArguments([])
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.errorOccurred.connect(
            lambda err: self._append_log(f"LibreVNA IPC process error: {err}")
        )
        process.finished.connect(
            lambda code, status: self._append_log(
                f"LibreVNA IPC exited: code={code} status={status}"
            )
        )
        process.stateChanged.connect(
            lambda state: self._append_log(f"LibreVNA IPC state: {state}")
        )
        process.readyReadStandardOutput.connect(self._drain_librevna_process)
        process.readyReadStandardError.connect(self._drain_librevna_process)
        process.start()
        if not process.waitForStarted(3000):
            self._append_log("Failed to start LibreVNA IPC process")
            return False
        self._librevna_process = process
        self._append_log("LibreVNA IPC started")
        return True

    def _apply_default_librevna_ipc_path(self) -> None:
        if self._settings.librevna_ipc_path:
            return
        for candidate in DEFAULT_LIBREVNA_IPC_CANDIDATES:
            if os.path.exists(candidate):
                self._settings.librevna_ipc_path = candidate
                self._append_log(f"Auto-detected LibreVNA IPC binary: {candidate}")
                return

    def _apply_saved_calibration_paths(self) -> None:
        if self._settings.librevna_cal_path:
            self.librevna_cal_edit.setText(self._settings.librevna_cal_path)
            self._append_log(f"Restored LibreVNA calibration path: {self._settings.librevna_cal_path}")
        if self._settings.golden_gain_threshold != 0.0:
            self.golden_threshold_spin.setValue(self._settings.golden_gain_threshold)

    @staticmethod
    def _prepend_env_path(env: QtCore.QProcessEnvironment, path: str) -> None:
        if not path or not os.path.isdir(path):
            return
        existing = env.value("PATH", "")
        parts = [p for p in existing.split(";") if p]
        if path in parts:
            return
        env.insert("PATH", f"{path};{existing}" if existing else path)

    def _drain_librevna_process(self) -> None:
        if not self._librevna_process:
            return
        data = bytes(self._librevna_process.readAllStandardOutput())
        if not data:
            data = bytes(self._librevna_process.readAllStandardError())
        text = data.decode("utf-8", errors="ignore").strip()
        if text:
            for line in text.split("\n"):
                self._append_ipc_log(line)

    def _refresh_librevna_devices(self) -> None:
        if self._librevna_device_worker and self._librevna_device_worker.isRunning():
            self._append_log("LibreVNA device scan already running")
            return

        self._append_log("Scanning LibreVNA devices")
        if not self._ensure_librevna_ipc():
            self._append_log("LibreVNA IPC is not running, aborting device scan")
            return
        server_name = normalize_ipc_name(self._settings.librevna_ipc_name)
        worker = LibreVNARequestWorker(
            server_name=server_name,
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
            self._settings.librevna_ipc_name = normalize_ipc_name(librevna_ipc_name)
        librevna_serial = data.get("librevna_serial")
        if isinstance(librevna_serial, str) and librevna_serial.strip():
            self._settings.librevna_serial = librevna_serial.strip()
        tinysa_cal_path = data.get("tinysa_cal_path")
        if isinstance(tinysa_cal_path, str) and tinysa_cal_path.strip():
            self._settings.tinysa_cal_path = tinysa_cal_path.strip()
        librevna_cal_path = data.get("librevna_cal_path")
        if isinstance(librevna_cal_path, str) and librevna_cal_path.strip():
            self._settings.librevna_cal_path = librevna_cal_path.strip()
        golden_gain_threshold = data.get("golden_gain_threshold")
        if isinstance(golden_gain_threshold, (int, float)):
            self._settings.golden_gain_threshold = float(golden_gain_threshold)

    def _save_settings_to_disk(self) -> None:
        payload = {
            "worker_python": self._settings.worker_python,
            "tinysa_debug": self._settings.tinysa_debug,
            "tinysa_port": self._settings.tinysa_port,
            "lmx_serial": self._settings.lmx_serial,
            "librevna_ipc_path": self._settings.librevna_ipc_path,
            "librevna_ipc_name": self._settings.librevna_ipc_name,
            "librevna_serial": self._settings.librevna_serial,
            "tinysa_cal_path": self._settings.tinysa_cal_path,
            "librevna_cal_path": self._settings.librevna_cal_path,
            "golden_gain_threshold": self._settings.golden_gain_threshold,
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
        self.librevna_timeout_spin.setValue(60000.0)
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
            f"ifbw={config.ifbw_hz} power={config.power_dbm} threshold={config.threshold_db} "
            f"timeout_ms={config.timeout_ms}"
        )
        self.librevna_status_label.setText("Status: Running")
        self._set_librevna_busy(True)

        worker = LibreVNARequestWorker(
            server_name=normalize_ipc_name(self._settings.librevna_ipc_name),
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

    # --- Workflow Methods ---

    def _update_workflow_ui(self) -> None:
        state = self._workflow_state
        step_text = {
            WorkflowState.IDLE: "Workflow: Idle",
            WorkflowState.DEVICE_SELECTION: "Phase 1: Select Device Mode",
            WorkflowState.LIBREVNA_CALIBRATION: "Phase 1: LibreVNA Calibration",
            WorkflowState.TINYSA_CALIBRATION: "Phase 1: tinySA Calibration",
            WorkflowState.REFERENCE_ANTENNA_MEASUREMENT: "Phase 1: Reference Antenna",
            WorkflowState.LMX_CALIBRATION: "Phase 1: LMX2594 Calibration",
            WorkflowState.LMX_CALIBRATION_COMPLETE: "Phase 1: Calibration Complete",
            WorkflowState.ANTENNA_MEASUREMENT_LOOP: "Phase 2: Antenna Measurement",
            WorkflowState.GOLDEN_SAMPLE_SELECTION: "Phase 2: Select Golden Sample",
            WorkflowState.ANTENNA_MEASUREMENT: "Phase 2: Antenna Measurement",
            WorkflowState.ANTENNA_COMPLETE: "Phase 2: Measurement Complete",
            WorkflowState.GOLDEN_SAMPLE_CHECK: "Phase 2: Golden Sample Check",
            WorkflowState.PRODUCTION_TESTING: "Phase 3: Production Testing",
            WorkflowState.WORKFLOW_COMPLETE: "Workflow Complete",
            WorkflowState.WORKFLOW_CANCELLED: "Workflow Cancelled",
        }.get(state, "Workflow: Unknown")

        instruction_text = {
            WorkflowState.IDLE: "Click 'Start Workflow' to begin guided calibration",
            WorkflowState.DEVICE_SELECTION: "Select VNA or SA mode, then click 'Next Step'",
            WorkflowState.LIBREVNA_CALIBRATION: "Perform LibreVNA open/short/load calibration",
            WorkflowState.TINYSA_CALIBRATION: "Click 'Calibrate tinySA' to calibrate",
            WorkflowState.REFERENCE_ANTENNA_MEASUREMENT: "Connect reference antenna and measure",
            WorkflowState.LMX_CALIBRATION: "Running LMX2594 calibration...",
            WorkflowState.LMX_CALIBRATION_COMPLETE: "Calibration complete. Click 'Next Step'",
            WorkflowState.ANTENNA_MEASUREMENT_LOOP: "Measure antennas to find golden sample candidates",
            WorkflowState.GOLDEN_SAMPLE_SELECTION: "Select a golden sample from candidates",
            WorkflowState.ANTENNA_MEASUREMENT: "Running antenna measurement...",
            WorkflowState.ANTENNA_COMPLETE: "Click 'Next Step' to continue",
            WorkflowState.GOLDEN_SAMPLE_CHECK: "Checking against golden sample...",
            WorkflowState.PRODUCTION_TESTING: "Measure antennas for PASS/FAIL testing",
            WorkflowState.WORKFLOW_COMPLETE: "All steps complete. Check results below.",
            WorkflowState.WORKFLOW_CANCELLED: "Workflow was cancelled.",
        }.get(state, "")

        self.workflow_step_label.setText(step_text)
        self.workflow_instruction_label.setText(instruction_text)

        # Update checklist indicators
        self._update_checklist_indicators()

        # Enable/disable workflow buttons
        in_progress = state in (
            WorkflowState.LMX_CALIBRATION,
            WorkflowState.ANTENNA_MEASUREMENT,
            WorkflowState.GOLDEN_SAMPLE_CHECK,
            WorkflowState.LIBREVNA_CALIBRATION,
            WorkflowState.TINYSA_CALIBRATION,
            WorkflowState.REFERENCE_ANTENNA_MEASUREMENT,
        )
        awaiting_next = state in (
            WorkflowState.DEVICE_SELECTION,
            WorkflowState.LMX_CALIBRATION_COMPLETE,
            WorkflowState.ANTENNA_COMPLETE,
            WorkflowState.ANTENNA_MEASUREMENT_LOOP,
            WorkflowState.GOLDEN_SAMPLE_SELECTION,
        )
        workflow_active = state not in (
            WorkflowState.IDLE,
            WorkflowState.WORKFLOW_COMPLETE,
            WorkflowState.WORKFLOW_CANCELLED,
        )

        self.start_workflow_button.setEnabled(not workflow_active)
        self.next_step_button.setEnabled(awaiting_next)
        self.cancel_workflow_button.setEnabled(workflow_active)

        # Color-code status bar
        if state == WorkflowState.WORKFLOW_COMPLETE:
            self.workflow_step_label.setStyleSheet("color: #0a7f2e; font-weight: bold;")
        elif state == WorkflowState.WORKFLOW_CANCELLED:
            self.workflow_step_label.setStyleSheet("color: #c1121f; font-weight: bold;")
        elif in_progress:
            self.workflow_step_label.setStyleSheet("color: #b58900; font-weight: bold;")
        else:
            self.workflow_step_label.setStyleSheet("font-weight: bold;")

        # Update inline hints
        lmx_hint_text = ""
        librevna_hint_text = ""
        if state == WorkflowState.DEVICE_SELECTION:
            lmx_hint_text = "Waiting: Select device mode above to proceed"
        elif state == WorkflowState.LMX_CALIBRATION:
            lmx_hint_text = "Running: LMX2594 calibration in progress..."
        elif state == WorkflowState.LMX_CALIBRATION_COMPLETE:
            lmx_hint_text = "Complete: Click 'Next Step' to continue"
        elif state == WorkflowState.ANTENNA_MEASUREMENT:
            if self._device_mode == DeviceMode.VNA:
                librevna_hint_text = "Running: VNA antenna measurement in progress..."
            else:
                lmx_hint_text = "Running: SA antenna measurement in progress..."
        elif state == WorkflowState.ANTENNA_COMPLETE:
            librevna_hint_text = "Complete: Click 'Next Step' to check golden sample"

        self.lmx_workflow_hint.setText(lmx_hint_text)
        self.lmx_workflow_hint.setVisible(bool(lmx_hint_text))
        self.librevna_workflow_hint.setText(librevna_hint_text)
        self.librevna_workflow_hint.setVisible(bool(librevna_hint_text))

    def _update_checklist_indicators(self) -> None:
        """Update the checklist indicators based on workflow results."""
        results = self._workflow_results
        
        # LibreVNA calibration
        if results.get("librevna_calibrated"):
            self.check_librevna_cal.setText("[X] LibreVNA Cal")
            self.check_librevna_cal.setStyleSheet("color: #0a7f2e;")
        else:
            self.check_librevna_cal.setText("[ ] LibreVNA Cal")
            self.check_librevna_cal.setStyleSheet("color: gray;")
        
        # tinySA calibration
        if results.get("tinysa_calibrated"):
            self.check_tinysa_cal.setText("[X] tinySA Cal")
            self.check_tinysa_cal.setStyleSheet("color: #0a7f2e;")
        else:
            self.check_tinysa_cal.setText("[ ] tinySA Cal")
            self.check_tinysa_cal.setStyleSheet("color: gray;")
        
        # Reference antenna
        if results.get("reference_antenna_measured"):
            self.check_ref_antenna.setText("[X] Reference Antenna")
            self.check_ref_antenna.setStyleSheet("color: #0a7f2e;")
        else:
            self.check_ref_antenna.setText("[ ] Reference Antenna")
            self.check_ref_antenna.setStyleSheet("color: gray;")
        
        # Golden sample
        if results.get("golden_sample_selected"):
            self.check_golden_sample.setText("[X] Golden Sample")
            self.check_golden_sample.setStyleSheet("color: #0a7f2e;")
        else:
            self.check_golden_sample.setText("[ ] Golden Sample")
            self.check_golden_sample.setStyleSheet("color: gray;")

    def _transition_workflow(self, new_state: WorkflowState) -> None:
        old_state = self._workflow_state
        self._workflow_state = new_state
        self._append_log(f"Workflow: {old_state.name} -> {new_state.name}")
        self._update_workflow_ui()

    def _start_guided_workflow(self) -> None:
        self._append_log("Starting guided calibration workflow")
        self._workflow_results = {}
        self._transition_workflow(WorkflowState.DEVICE_SELECTION)

    def _advance_workflow(self) -> None:
        state = self._workflow_state
        if state == WorkflowState.DEVICE_SELECTION:
            if self._device_mode == DeviceMode.NONE:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Device Mode Required",
                    "Please select VNA or SA mode before continuing.",
                )
                return
            # Start calibration phase
            if self._device_mode == DeviceMode.VNA:
                self._transition_workflow(WorkflowState.LIBREVNA_CALIBRATION)
                self._show_librevna_calibration_guide()
            else:
                self._transition_workflow(WorkflowState.TINYSA_CALIBRATION)
                self._run_tinysa_calibration()
        elif state == WorkflowState.LIBREVNA_CALIBRATION:
            self._workflow_results["librevna_calibrated"] = True
            self._transition_workflow(WorkflowState.TINYSA_CALIBRATION)
            self._run_tinysa_calibration()
        elif state == WorkflowState.TINYSA_CALIBRATION:
            self._workflow_results["tinysa_calibrated"] = True
            self._transition_workflow(WorkflowState.LMX_CALIBRATION)
            self._run_lmx_calibration_step()
        elif state == WorkflowState.LMX_CALIBRATION_COMPLETE:
            # Move to antenna measurement loop for golden sample selection
            self._transition_workflow(WorkflowState.ANTENNA_MEASUREMENT_LOOP)
            # Switch to Antenna Testing tab
            if hasattr(self, 'results_tabs'):
                self.results_tabs.setCurrentIndex(2)
        elif state == WorkflowState.ANTENNA_MEASUREMENT_LOOP:
            # Check if we have candidates
            if not self.golden_sample_panel._candidates:
                QtWidgets.QMessageBox.warning(
                    self, "No Candidates",
                    "Measure at least one antenna before selecting golden sample."
                )
                return
            self._transition_workflow(WorkflowState.GOLDEN_SAMPLE_SELECTION)
        elif state == WorkflowState.GOLDEN_SAMPLE_SELECTION:
            # Wait for user to confirm golden sample via panel
            if not self._workflow_results.get("golden_sample_selected"):
                QtWidgets.QMessageBox.information(
                    self, "Select Golden Sample",
                    "Click on a candidate and press 'Confirm Golden Sample'."
                )
                return
            self._transition_workflow(WorkflowState.PRODUCTION_TESTING)
        elif state == WorkflowState.ANTENNA_COMPLETE:
            self._transition_workflow(WorkflowState.GOLDEN_SAMPLE_CHECK)
            self._run_golden_sample_check()

    def _show_librevna_calibration_guide(self) -> None:
        """Show LibreVNA calibration instructions."""
        msg = QtWidgets.QMessageBox(self)
        msg.setWindowTitle("LibreVNA Calibration")
        msg.setText("Perform LibreVNA Open/Short/Load calibration:")
        msg.setInformativeText(
            "1. Connect OPEN standard and click OK\n"
            "2. Connect SHORT standard and click OK\n"
            "3. Connect LOAD standard and click OK\n\n"
            "When complete, click 'Next Step' to continue."
        )
        msg.setIcon(QtWidgets.QMessageBox.Information)
        msg.exec()

    def _cancel_workflow(self) -> None:
        reply = QtWidgets.QMessageBox.question(
            self,
            "Cancel Workflow",
            "Are you sure you want to cancel the workflow?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self._append_log("Workflow cancelled by user")
            # Stop any running workers
            if self._worker and self._worker.isRunning():
                self._worker.request_stop()
            if self._librevna_sweep_worker and self._librevna_sweep_worker.isRunning():
                self._librevna_sweep_worker.terminate()
            self._transition_workflow(WorkflowState.WORKFLOW_CANCELLED)

    # --- Workflow Step Execution ---

    def _run_lmx_calibration_step(self) -> None:
        if self._worker and self._worker.isRunning():
            self._append_log("LMX calibration already running")
            return

        try:
            config = self._build_config()
        except Exception as exc:
            self._append_log(f"Invalid LMX settings: {exc}")
            self._transition_workflow(WorkflowState.WORKFLOW_CANCELLED)
            return

        self.results_table.setRowCount(0)
        self.plot_widget.clear()
        self._set_busy(True)

        self._worker = CalibrationWorker(
            config=config,
            port=self._get_selected_port(),
            lmx_serial=self._get_selected_lmx(),
            worker_python=self._settings.worker_python,
        )
        self._worker.log.connect(self._append_log)
        self._worker.point.connect(self._handle_point)
        self._worker.done.connect(self._handle_lmx_workflow_done)
        self._worker.failed.connect(self._handle_lmx_workflow_failed)
        self._worker.start()

    def _handle_lmx_workflow_done(self, points: List[CalibrationPoint]) -> None:
        self._set_busy(False)
        self._append_log(f"LMX calibration complete: {len(points)} points")
        self._workflow_results["lmx_points"] = points

        # Save calibration file
        stamp = QtCore.QDateTime.currentDateTimeUtc().toString("yyyyMMdd_HHmmss")
        out_path = self.output_edit.text().strip()
        if not out_path:
            out_path = os.path.join("calibration", f"lmx_workflow_{stamp}.cal")
        try:
            self._save_calibration(out_path, points)
            self._append_log(f"Saved LMX calibration: {out_path}")
            self._settings.tinysa_cal_path = out_path
            self._save_settings_to_disk()
        except Exception as exc:
            self._append_log(f"Failed to save LMX calibration: {exc}")

        self._transition_workflow(WorkflowState.LMX_CALIBRATION_COMPLETE)

    def _handle_lmx_workflow_failed(self, message: str) -> None:
        self._set_busy(False)
        self._append_log(f"LMX calibration failed: {message}")
        self._transition_workflow(WorkflowState.WORKFLOW_CANCELLED)

    def _run_antenna_measurement_step(self) -> None:
        if self._device_mode == DeviceMode.VNA:
            self._run_antenna_vna_measurement()
        elif self._device_mode == DeviceMode.SA:
            self._run_antenna_sa_measurement()
        else:
            self._append_log("No device mode selected")
            self._transition_workflow(WorkflowState.WORKFLOW_CANCELLED)

    def _run_antenna_vna_measurement(self) -> None:
        if self._librevna_sweep_worker and self._librevna_sweep_worker.isRunning():
            self._append_log("LibreVNA sweep already running")
            return

        try:
            config = self._build_librevna_config()
        except Exception as exc:
            self._append_log(f"LibreVNA config invalid: {exc}")
            self._transition_workflow(WorkflowState.WORKFLOW_CANCELLED)
            return

        self._ensure_librevna_ipc()
        self.librevna_results_table.setRowCount(0)
        self.librevna_mag_plot.clear()
        self.librevna_phase_plot.clear()
        self._librevna_trace = []
        self._set_librevna_busy(True)
        self.librevna_status_label.setText("Status: Running (Workflow)")

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

        worker = LibreVNARequestWorker(
            server_name=normalize_ipc_name(self._settings.librevna_ipc_name),
            cmd="run_sweep",
            payload=payload,
            timeout_ms=int(config.timeout_ms) + 5000,
        )
        worker.log.connect(self._append_log)
        worker.done.connect(self._handle_antenna_vna_done)
        worker.failed.connect(self._handle_antenna_vna_failed)
        self._librevna_sweep_worker = worker
        worker.start()

    def _handle_antenna_vna_done(self, payload: Dict[str, object]) -> None:
        self._set_librevna_busy(False)
        self.librevna_status_label.setText("Status: Complete (Workflow)")
        self._update_librevna_results(payload)

        # Extract S21 gain for golden sample comparison
        trace = payload.get("trace", [])
        s21_gains: List[float] = []
        for entry in trace:
            if not isinstance(entry, dict):
                continue
            s21 = entry.get("S21")
            if isinstance(s21, dict):
                real = s21.get("real", 0)
                imag = s21.get("imag", 0)
                if isinstance(real, (int, float)) and isinstance(imag, (int, float)):
                    value = complex(real, imag)
                    mag_db = -300.0 if value == 0 else 20.0 * math.log10(abs(value))
                    s21_gains.append(mag_db)

        if s21_gains:
            avg_gain = sum(s21_gains) / len(s21_gains)
            max_gain = max(s21_gains)
            self._workflow_results["antenna_gain_avg"] = avg_gain
            self._workflow_results["antenna_gain_max"] = max_gain
            self._append_log(f"VNA S21 gain: avg={avg_gain:.2f} dB, max={max_gain:.2f} dB")

        self._workflow_results["vna_payload"] = payload
        self._transition_workflow(WorkflowState.ANTENNA_COMPLETE)

    def _handle_antenna_vna_failed(self, message: str) -> None:
        self._set_librevna_busy(False)
        self.librevna_status_label.setText("Status: Failed (Workflow)")
        self._append_log(f"VNA antenna measurement failed: {message}")
        self._transition_workflow(WorkflowState.WORKFLOW_CANCELLED)

    def _run_antenna_sa_measurement(self) -> None:
        # SA mode reuses the tinySA for a single-point gain check
        # This is a simplified implementation - could be expanded
        self._append_log("SA mode antenna measurement: using tinySA gain data from LMX calibration")
        lmx_points = self._workflow_results.get("lmx_points", [])
        if lmx_points:
            offsets = [p.offset_db for p in lmx_points]
            avg_offset = sum(offsets) / len(offsets) if offsets else 0.0
            self._workflow_results["antenna_gain_avg"] = avg_offset
            self._workflow_results["antenna_gain_max"] = max(offsets) if offsets else 0.0
            self._append_log(f"SA gain offset: avg={avg_offset:.2f} dB")
        self._transition_workflow(WorkflowState.ANTENNA_COMPLETE)

    def _run_golden_sample_check(self) -> None:
        threshold = float(self.golden_threshold_spin.value())
        tolerance = float(self.golden_tolerance_spin.value())
        measured_gain = self._workflow_results.get("antenna_gain_avg", 0.0)

        self._append_log(
            f"Golden sample check: measured={measured_gain:.2f} dB, "
            f"threshold={threshold:.2f} dB, tolerance=±{tolerance:.2f} dB"
        )

        lower_bound = threshold - tolerance
        upper_bound = threshold + tolerance
        passed = lower_bound <= measured_gain <= upper_bound

        self._workflow_results["golden_passed"] = passed
        self._workflow_results["golden_threshold"] = threshold
        self._workflow_results["golden_tolerance"] = tolerance

        if passed:
            self._append_log(f"PASS: Measured gain {measured_gain:.2f} dB within [{lower_bound:.2f}, {upper_bound:.2f}] dB")
            result_msg = f"PASS - Gain: {measured_gain:.2f} dB"
            color = "#0a7f2e"
        else:
            self._append_log(f"FAIL: Measured gain {measured_gain:.2f} dB outside [{lower_bound:.2f}, {upper_bound:.2f}] dB")
            result_msg = f"FAIL - Gain: {measured_gain:.2f} dB"
            color = "#c1121f"

        QtWidgets.QMessageBox.information(
            self,
            "Golden Sample Result",
            f"Result: {result_msg}\n\n"
            f"Threshold: {threshold:.2f} ± {tolerance:.2f} dB\n"
            f"Measured: {measured_gain:.2f} dB",
        )

        self._transition_workflow(WorkflowState.WORKFLOW_COMPLETE)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._closing = True
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
                LibreVNAIpcClient(normalize_ipc_name(self._settings.librevna_ipc_name)).request(
                    {"cmd": "shutdown"}, timeout_ms=2000
                )
            except Exception:
                pass
            try:
                self._librevna_process.disconnect()
            except Exception:
                pass
            self._librevna_process.terminate()
            self._librevna_process.waitForFinished(2000)
            self._librevna_process = None

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

    # --- Antenna Testing Methods ---

    def _on_golden_sample_confirmed(self, sample: AntennaMeasurement) -> None:
        """Handle golden sample confirmation from the panel."""
        self._append_log(f"Golden sample confirmed: {sample.label} ({sample.gain_db:.2f} dB)")
        self._workflow_results["golden_sample_selected"] = True
        self._workflow_results["golden_sample"] = sample
        
        tolerance = self.golden_sample_panel.tolerance_spin.value()
        self.antenna_results_widget.set_golden_sample(sample, tolerance)
        
        self._update_checklist_indicators()
        
        # Transition to production testing if in workflow
        if self._workflow_state == WorkflowState.GOLDEN_SAMPLE_SELECTION:
            self._transition_workflow(WorkflowState.PRODUCTION_TESTING)
            # Switch to Antenna Testing tab
            if hasattr(self, 'results_tabs'):
                self.results_tabs.setCurrentIndex(2)

    def _on_measure_antenna_clicked(self) -> None:
        """Handle measure antenna button click."""
        label = self.antenna_label_edit.text().strip()
        if not label:
            # Auto-generate label
            count = len(self.antenna_results_widget.get_measurements()) + 1
            count += len(self.golden_sample_panel._candidates)
            label = f"Antenna #{count}"
            self.antenna_label_edit.setText(label)
        
        self._append_log(f"Measuring antenna: {label}")
        self._run_single_antenna_measurement(label)

    def _run_single_antenna_measurement(self, label: str) -> None:
        """Run a single antenna measurement using tinySA."""
        # Get frequency from settings
        try:
            freq_hz = parse_frequency(self.start_freq_edit.text())
        except Exception:
            freq_hz = 3.5e9
        
        tx_power = float(self.requested_dbm_edit.value())
        
        # Create measurement worker
        self._antenna_measure_worker = AntennaMeasurementWorker(
            port=self._get_selected_port(),
            frequency_hz=freq_hz,
            tx_power_dbm=tx_power,
            label=label,
            debug=self._settings.tinysa_debug,
        )
        self._antenna_measure_worker.log.connect(self._append_log)
        self._antenna_measure_worker.done.connect(self._handle_antenna_measurement_done)
        self._antenna_measure_worker.failed.connect(self._handle_antenna_measurement_failed)
        
        self.measure_antenna_button.setEnabled(False)
        self._antenna_measure_worker.start()

    def _handle_antenna_measurement_done(self, measurement: AntennaMeasurement) -> None:
        """Handle completed antenna measurement."""
        self.measure_antenna_button.setEnabled(True)
        self._append_log(f"Antenna measured: {measurement.label} = {measurement.gain_db:.2f} dB")
        
        # Clear the label field for next measurement
        self.antenna_label_edit.clear()
        
        # Check workflow state to determine where to add the measurement
        state = self._workflow_state
        if state in (WorkflowState.ANTENNA_MEASUREMENT_LOOP, WorkflowState.GOLDEN_SAMPLE_SELECTION):
            # Add to golden sample candidates
            self.golden_sample_panel.add_candidate(measurement)
            self._append_log(f"Added to golden sample candidates")
        elif state == WorkflowState.PRODUCTION_TESTING:
            # Add to production test results
            self.antenna_results_widget.add_measurement(measurement)
        else:
            # Default: add to both if golden sample not yet selected
            if not self._workflow_results.get("golden_sample_selected"):
                self.golden_sample_panel.add_candidate(measurement)
            else:
                self.antenna_results_widget.add_measurement(measurement)

    def _handle_antenna_measurement_failed(self, message: str) -> None:
        """Handle failed antenna measurement."""
        self.measure_antenna_button.setEnabled(True)
        self._append_log(f"Antenna measurement failed: {message}")
        QtWidgets.QMessageBox.warning(
            self, "Measurement Failed",
            f"Failed to measure antenna:\n{message}"
        )

    def _on_save_session_clicked(self) -> None:
        """Save the current antenna testing session to JSON."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Session", "", "JSON Files (*.json)"
        )
        if not path:
            return
        
        self._save_antenna_session(path)

    def _save_antenna_session(self, path: str) -> None:
        """Save antenna testing session to file."""
        golden = self._workflow_results.get("golden_sample")
        tolerance = self.golden_sample_panel.tolerance_spin.value()
        
        data = {
            "type": "antenna_test_session",
            "created_at": _utc_now(),
            "golden_sample": golden.as_dict() if golden else None,
            "tolerance_db": tolerance,
            "candidates": [m.as_dict() for m in self.golden_sample_panel._candidates],
            "measurements": [m.as_dict() for m in self.antenna_results_widget.get_measurements()],
        }
        
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self._append_log(f"Session saved: {path}")
        except Exception as exc:
            self._append_log(f"Failed to save session: {exc}")

    def _on_load_session_clicked(self) -> None:
        """Load an antenna testing session from JSON."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Session", "", "JSON Files (*.json)"
        )
        if not path:
            return
        
        self._load_antenna_session(path)

    def _load_antenna_session(self, path: str) -> None:
        """Load antenna testing session from file."""
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self._append_log(f"Failed to load session: {exc}")
            return
        
        # Clear existing data
        self.golden_sample_panel.clear_candidates()
        self.antenna_results_widget.clear()
        
        # Load tolerance
        tolerance = data.get("tolerance_db", 3.0)
        self.golden_sample_panel.tolerance_spin.setValue(tolerance)
        
        # Load candidates
        for entry in data.get("candidates", []):
            try:
                m = AntennaMeasurement.from_dict(entry)
                self.golden_sample_panel._candidates.append(m)
            except Exception:
                continue
        self.golden_sample_panel._update_candidates_list()
        
        # Load golden sample
        golden_data = data.get("golden_sample")
        if golden_data:
            golden = AntennaMeasurement.from_dict(golden_data)
            self._workflow_results["golden_sample"] = golden
            self._workflow_results["golden_sample_selected"] = True
            self.antenna_results_widget.set_golden_sample(golden, tolerance)
            self._update_checklist_indicators()
        
        # Load measurements
        for entry in data.get("measurements", []):
            try:
                m = AntennaMeasurement.from_dict(entry)
                self.antenna_results_widget.add_measurement(m)
            except Exception:
                continue
        
        self._append_log(f"Session loaded: {path}")


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = CalibrationWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
