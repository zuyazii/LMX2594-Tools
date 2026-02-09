"""
LMX Monitor tab widget for displaying LMX2594 frequency emission properties
and device status (USB2ANY, LMX2594).
"""
from __future__ import annotations

from typing import Optional, Dict, Any
import datetime as dt

try:
    from PySide6 import QtCore, QtWidgets
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtWidgets
    Signal = QtCore.pyqtSignal

from gui.styles import COLORS


class LMXMonitorTab(QtWidgets.QWidget):
    """Tab for monitoring LMX2594 frequency emission and device status."""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._setup_ui()
        self._init_state()

    def _init_state(self):
        """Initialize internal state."""
        self._last_freq_hz: Optional[float] = None
        self._last_vco_hz: Optional[float] = None
        self._last_chdiv: Optional[int] = None
        self._last_n_int: Optional[int] = None
        self._last_num: Optional[int] = None
        self._last_den: Optional[int] = None
        self._last_outa_pwr: Optional[int] = None
        self._last_pfd_hz: Optional[float] = None
        self._last_update_time: Optional[dt.datetime] = None
        self._last_fcal_ms: Optional[float] = None
        self._last_lock_time_ms: Optional[float] = None
        self._sweep_point: int = 0
        self._sweep_total: int = 0

    def _setup_ui(self):
        """Setup the tab UI."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(12)

        # Top row: Frequency Properties + Timing Info side by side
        top_row = QtWidgets.QHBoxLayout()
        top_row.addWidget(self._create_frequency_section(), stretch=2)
        top_row.addWidget(self._create_timing_section(), stretch=1)
        layout.addLayout(top_row)

        # Bottom row: Device Status
        layout.addWidget(self._create_device_status_section())

        layout.addStretch()

    def _create_frequency_section(self) -> QtWidgets.QGroupBox:
        """Create the frequency properties section."""
        group = QtWidgets.QGroupBox("Frequency Properties")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(6)

        # Output Frequency
        self.lbl_output_freq = QtWidgets.QLabel("--")
        self.lbl_output_freq.setStyleSheet(f"font-weight: bold; color: {COLORS['accent']};")
        layout.addRow("Output Frequency:", self.lbl_output_freq)

        # VCO Frequency
        self.lbl_vco_freq = QtWidgets.QLabel("--")
        layout.addRow("VCO Frequency:", self.lbl_vco_freq)

        # Channel Divider
        self.lbl_chdiv = QtWidgets.QLabel("--")
        layout.addRow("Channel Divider:", self.lbl_chdiv)

        # N (integer)
        self.lbl_n_int = QtWidgets.QLabel("--")
        layout.addRow("N (integer):", self.lbl_n_int)

        # NUM (fractional numerator)
        self.lbl_num = QtWidgets.QLabel("--")
        layout.addRow("NUM (numerator):", self.lbl_num)

        # DEN (fractional denominator)
        self.lbl_den = QtWidgets.QLabel("--")
        layout.addRow("DEN (denominator):", self.lbl_den)

        # OUTA_PWR
        self.lbl_outa_pwr = QtWidgets.QLabel("--")
        layout.addRow("OUTA_PWR:", self.lbl_outa_pwr)

        # PFD Frequency
        self.lbl_pfd_freq = QtWidgets.QLabel("--")
        layout.addRow("PFD Frequency:", self.lbl_pfd_freq)

        return group

    def _create_timing_section(self) -> QtWidgets.QGroupBox:
        """Create the timing info section."""
        group = QtWidgets.QGroupBox("Timing Info")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(6)

        # Last Update
        self.lbl_last_update = QtWidgets.QLabel("--")
        layout.addRow("Last Update:", self.lbl_last_update)

        # FCAL Duration
        self.lbl_fcal_duration = QtWidgets.QLabel("--")
        layout.addRow("FCAL Duration:", self.lbl_fcal_duration)

        # Lock Acquisition Time
        self.lbl_lock_time = QtWidgets.QLabel("--")
        layout.addRow("Lock Acq. Time:", self.lbl_lock_time)

        # Sweep Progress
        self.lbl_sweep_progress = QtWidgets.QLabel("Idle")
        self.lbl_sweep_progress.setStyleSheet(f"color: {COLORS['text_dim']};")
        layout.addRow("Sweep Progress:", self.lbl_sweep_progress)

        return group

    def _create_device_status_section(self) -> QtWidgets.QGroupBox:
        """Create the device status section."""
        group = QtWidgets.QGroupBox("Device Status")
        layout = QtWidgets.QHBoxLayout(group)
        layout.setSpacing(24)

        # USB2ANY status
        usb2any_layout = QtWidgets.QVBoxLayout()
        self.lbl_usb2any_status = QtWidgets.QLabel()
        self._set_status_indicator(self.lbl_usb2any_status, "USB2ANY", "disconnected")
        self.lbl_usb2any_serial = QtWidgets.QLabel("Serial: --")
        self.lbl_usb2any_serial.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px;")
        usb2any_layout.addWidget(self.lbl_usb2any_status)
        usb2any_layout.addWidget(self.lbl_usb2any_serial)
        layout.addLayout(usb2any_layout)

        # LMX2594 status
        lmx_layout = QtWidgets.QVBoxLayout()
        self.lbl_lmx_status = QtWidgets.QLabel()
        self._set_status_indicator(self.lbl_lmx_status, "LMX2594", "not_programmed")
        self.lbl_lmx_lock = QtWidgets.QLabel("Lock: --")
        self.lbl_lmx_lock.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px;")
        lmx_layout.addWidget(self.lbl_lmx_status)
        lmx_layout.addWidget(self.lbl_lmx_lock)
        layout.addLayout(lmx_layout)

        layout.addStretch()

        return group

    def _set_status_indicator(self, label: QtWidgets.QLabel, name: str, status: str):
        """Set a status indicator label with colored dot."""
        dot = "\u25cf"  # Filled circle
        if status == "connected" or status == "locked":
            color = COLORS["success"]
            status_text = "Connected" if status == "connected" else "Locked"
        elif status == "unlocked":
            color = COLORS.get("warning", "#FFA500")
            status_text = "Unlocked"
        elif status == "not_programmed":
            color = COLORS["text_dim"]
            status_text = "Not Programmed"
        else:  # disconnected
            color = COLORS.get("error", "#FF4444")
            status_text = "Disconnected"

        label.setText(f"{dot} {name}: {status_text}")
        label.setStyleSheet(f"color: {color}; font-weight: bold;")

    @staticmethod
    def _format_frequency(hz: Optional[float]) -> str:
        """Format frequency in appropriate units."""
        if hz is None:
            return "--"
        if hz >= 1e9:
            return f"{hz / 1e9:.6f} GHz"
        elif hz >= 1e6:
            return f"{hz / 1e6:.3f} MHz"
        elif hz >= 1e3:
            return f"{hz / 1e3:.3f} kHz"
        else:
            return f"{hz:.1f} Hz"

    def update_frequency_info(self, data: Dict[str, Any]):
        """Update frequency properties from sweep point data."""
        self._last_freq_hz = data.get("freq_hz")
        self._last_vco_hz = data.get("vco_hz")
        self._last_chdiv = data.get("chdiv")
        self._last_n_int = data.get("n_int")
        self._last_num = data.get("num")
        self._last_den = data.get("den")
        self._last_outa_pwr = data.get("outa_pwr")
        self._last_pfd_hz = data.get("pfd_hz")
        self._last_fcal_ms = data.get("fcal_ms")
        self._last_lock_time_ms = data.get("lock_time_ms")
        self._last_update_time = dt.datetime.now()

        # Update labels
        self.lbl_output_freq.setText(self._format_frequency(self._last_freq_hz))
        self.lbl_vco_freq.setText(self._format_frequency(self._last_vco_hz))
        self.lbl_chdiv.setText(str(self._last_chdiv) if self._last_chdiv is not None else "--")
        self.lbl_n_int.setText(str(self._last_n_int) if self._last_n_int is not None else "--")
        self.lbl_num.setText(str(self._last_num) if self._last_num is not None else "--")
        self.lbl_den.setText(str(self._last_den) if self._last_den is not None else "--")
        self.lbl_outa_pwr.setText(str(self._last_outa_pwr) if self._last_outa_pwr is not None else "--")
        self.lbl_pfd_freq.setText(self._format_frequency(self._last_pfd_hz))

        # Update timing
        self.lbl_last_update.setText(self._last_update_time.strftime("%H:%M:%S.%f")[:-3])
        if self._last_fcal_ms is not None:
            self.lbl_fcal_duration.setText(f"{self._last_fcal_ms:.1f} ms")
        if self._last_lock_time_ms is not None:
            self.lbl_lock_time.setText(f"{self._last_lock_time_ms:.1f} ms")

        # Update LMX status to locked if we got valid data
        if self._last_freq_hz is not None:
            locked = data.get("locked", True)
            self._set_status_indicator(
                self.lbl_lmx_status, "LMX2594", "locked" if locked else "unlocked"
            )
            self.lbl_lmx_lock.setText(f"Lock: {'Yes' if locked else 'No'}")

    def update_sweep_progress(self, current: int, total: int):
        """Update sweep progress display."""
        self._sweep_point = current
        self._sweep_total = total
        if total > 0:
            self.lbl_sweep_progress.setText(f"Point {current} of {total}")
            self.lbl_sweep_progress.setStyleSheet(f"color: {COLORS['accent']};")
        else:
            self.lbl_sweep_progress.setText("Idle")
            self.lbl_sweep_progress.setStyleSheet(f"color: {COLORS['text_dim']};")

    def update_usb2any_status(self, connected: bool, serial: Optional[str] = None):
        """Update USB2ANY connection status."""
        self._set_status_indicator(
            self.lbl_usb2any_status,
            "USB2ANY",
            "connected" if connected else "disconnected"
        )
        if serial:
            self.lbl_usb2any_serial.setText(f"Serial: {serial}")
        else:
            self.lbl_usb2any_serial.setText("Serial: --")

    def update_lmx_status(self, status: str, locked: Optional[bool] = None):
        """Update LMX2594 status.
        
        Args:
            status: One of 'locked', 'unlocked', 'not_programmed', 'disconnected'
            locked: Optional explicit lock state
        """
        self._set_status_indicator(self.lbl_lmx_status, "LMX2594", status)
        if locked is not None:
            self.lbl_lmx_lock.setText(f"Lock: {'Yes' if locked else 'No'}")
        elif status == "not_programmed":
            self.lbl_lmx_lock.setText("Lock: --")

    def clear_all(self):
        """Reset all displays to default state."""
        self._init_state()
        self.lbl_output_freq.setText("--")
        self.lbl_vco_freq.setText("--")
        self.lbl_chdiv.setText("--")
        self.lbl_n_int.setText("--")
        self.lbl_num.setText("--")
        self.lbl_den.setText("--")
        self.lbl_outa_pwr.setText("--")
        self.lbl_pfd_freq.setText("--")
        self.lbl_last_update.setText("--")
        self.lbl_fcal_duration.setText("--")
        self.lbl_lock_time.setText("--")
        self.lbl_sweep_progress.setText("Idle")
        self.lbl_sweep_progress.setStyleSheet(f"color: {COLORS['text_dim']};")
        self._set_status_indicator(self.lbl_usb2any_status, "USB2ANY", "disconnected")
        self.lbl_usb2any_serial.setText("Serial: --")
        self._set_status_indicator(self.lbl_lmx_status, "LMX2594", "not_programmed")
        self.lbl_lmx_lock.setText("Lock: --")

    def export_state(self) -> Dict[str, Any]:
        """Export current state for session save."""
        return {
            "freq_hz": self._last_freq_hz,
            "vco_hz": self._last_vco_hz,
            "chdiv": self._last_chdiv,
            "n_int": self._last_n_int,
            "num": self._last_num,
            "den": self._last_den,
            "outa_pwr": self._last_outa_pwr,
            "pfd_hz": self._last_pfd_hz,
        }

    def apply_state(self, state: Dict[str, Any]):
        """Apply saved state from session load."""
        if state:
            self.update_frequency_info(state)
