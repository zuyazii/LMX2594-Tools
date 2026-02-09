"""
LibreVNA tab - thresholds, visibility, and S-parameter summary table.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

try:
    from PySide6 import QtCore, QtWidgets
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtWidgets
    Signal = QtCore.pyqtSignal


class LibreVNATab(QtWidgets.QWidget):
    """LibreVNA results tab."""

    plot_changed = Signal(dict)

    S_PARAMS = ("S11", "S21", "S12", "S22")

    COLORS_BY_PARAM = {
        "S11": "#d62728",
        "S21": "#1f77b4",
        "S12": "#2ca02c",
        "S22": "#ff7f0e",
    }
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._trace: List[Dict[str, object]] = []
        self._series_mag: Dict[str, List[Tuple[float, float]]] = {}
        self._series_phase: Dict[str, List[Tuple[float, float]]] = {}
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the LibreVNA tab UI."""
        layout = QtWidgets.QVBoxLayout(self)

        # Status row
        status_row = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("Status: Idle")
        self.pass_label = QtWidgets.QLabel("Overall: N/A")
        self.pass_label.setStyleSheet("font-weight: bold;")
        status_row.addWidget(self.status_label)
        status_row.addStretch(1)
        status_row.addWidget(self.pass_label)
        layout.addLayout(status_row)

        body_layout = QtWidgets.QHBoxLayout()

        left_col = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_col)

        threshold_group = QtWidgets.QGroupBox("Pass Threshold per S-Parameter")
        threshold_layout = QtWidgets.QFormLayout(threshold_group)
        self.threshold_spins: Dict[str, QtWidgets.QDoubleSpinBox] = {}
        for param in self.S_PARAMS:
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-200.0, 20.0)
            spin.setDecimals(1)
            spin.setValue(-60.0)
            spin.setSuffix(" dB")
            spin.valueChanged.connect(self._update_results_and_emit)
            self.threshold_spins[param] = spin
            threshold_layout.addRow(f"{param} threshold", spin)
        left_layout.addWidget(threshold_group)

        controls_group = QtWidgets.QGroupBox("Plot Visibility")
        controls_layout = QtWidgets.QVBoxLayout(controls_group)

        mag_row = QtWidgets.QHBoxLayout()
        mag_row.addWidget(QtWidgets.QLabel("Magnitude:"))
        self.mag_checks: Dict[str, QtWidgets.QCheckBox] = {}
        for param in self.S_PARAMS:
            check = QtWidgets.QCheckBox(param)
            check.setChecked(True)
            check.stateChanged.connect(self._emit_plot_payload)
            self.mag_checks[param] = check
            mag_row.addWidget(check)
        mag_row.addStretch(1)
        controls_layout.addLayout(mag_row)

        phase_row = QtWidgets.QHBoxLayout()
        phase_row.addWidget(QtWidgets.QLabel("Phase:"))
        self.phase_checks: Dict[str, QtWidgets.QCheckBox] = {}
        for param in self.S_PARAMS:
            check = QtWidgets.QCheckBox(param)
            check.setChecked(True)
            check.stateChanged.connect(self._emit_plot_payload)
            self.phase_checks[param] = check
            phase_row.addWidget(check)
        phase_row.addStretch(1)
        controls_layout.addLayout(phase_row)

        left_layout.addWidget(controls_group)
        left_layout.addStretch(1)

        right_col = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_col)
        self.results_table = QtWidgets.QTableWidget(0, 4)
        self.results_table.setHorizontalHeaderLabels([
            "S-Parameter", "Pass", "Worst dB", "Fail @ Hz"
        ])
        self.results_table.horizontalHeader().setStretchLastSection(True)
        right_layout.addWidget(self.results_table)

        body_layout.addWidget(left_col, 0)
        body_layout.addWidget(right_col, 1)
        layout.addLayout(body_layout)

    def set_status(self, text: str):
        self.status_label.setText(text)

    def set_sweep_result(self, payload: Dict[str, object]):
        trace = payload.get("trace", [])
        self._trace = trace if isinstance(trace, list) else []
        self._build_series()
        self._update_results_and_emit()

    def emit_plot_payload(self):
        self._emit_plot_payload()

    def export_state(self) -> Dict[str, object]:
        return {
            "thresholds": {param: float(self.threshold_spins[param].value()) for param in self.S_PARAMS},
            "show_mag": [param for param in self.S_PARAMS if self.mag_checks[param].isChecked()],
            "show_phase": [param for param in self.S_PARAMS if self.phase_checks[param].isChecked()],
        }

    def apply_state(self, state: Dict[str, object]):
        if not isinstance(state, dict):
            return
        thresholds = state.get("thresholds", {})
        if isinstance(thresholds, dict):
            for param in self.S_PARAMS:
                value = thresholds.get(param)
                if isinstance(value, (int, float)):
                    self.threshold_spins[param].setValue(float(value))

        show_mag = set(state.get("show_mag", []))
        show_phase = set(state.get("show_phase", []))
        for param in self.S_PARAMS:
            self.mag_checks[param].setChecked(param in show_mag if show_mag else True)
            self.phase_checks[param].setChecked(param in show_phase if show_phase else True)

        self._emit_plot_payload()

    def clear_all(self):
        self._trace = []
        self._series_mag = {}
        self._series_phase = {}
        self.results_table.setRowCount(0)
        self.status_label.setText("Status: Idle")
        self.pass_label.setText("Overall: N/A")
        self.pass_label.setStyleSheet("font-weight: bold;")
        self._emit_plot_payload()

    def _build_series(self):
        self._series_mag = {k: [] for k in self.S_PARAMS}
        self._series_phase = {k: [] for k in self.S_PARAMS}

        for entry in self._trace:
            if not isinstance(entry, dict):
                continue
            freq = entry.get("frequency")
            if not isinstance(freq, (int, float)):
                continue
            for param in self.S_PARAMS:
                data = entry.get(param)
                if not isinstance(data, dict):
                    continue
                real = data.get("real")
                imag = data.get("imag")
                if not isinstance(real, (int, float)) or not isinstance(imag, (int, float)):
                    continue
                mag_linear = math.sqrt(real * real + imag * imag)
                mag_db = -200.0 if mag_linear <= 0.0 else 20.0 * math.log10(mag_linear)
                phase_deg = math.degrees(math.atan2(imag, real))
                self._series_mag[param].append((float(freq), float(mag_db)))
                self._series_phase[param].append((float(freq), float(phase_deg)))

    def _update_results_and_emit(self):
        self.results_table.setRowCount(0)
        overall_pass = True

        for param in self.S_PARAMS:
            points = self._series_mag.get(param, [])
            threshold = float(self.threshold_spins[param].value())

            row = self.results_table.rowCount()
            self.results_table.insertRow(row)
            self.results_table.setItem(row, 0, QtWidgets.QTableWidgetItem(param))

            if not points:
                self.results_table.setItem(row, 1, QtWidgets.QTableWidgetItem("N/A"))
                self.results_table.setItem(row, 2, QtWidgets.QTableWidgetItem(""))
                self.results_table.setItem(row, 3, QtWidgets.QTableWidgetItem(""))
                continue

            mags = [v for _f, v in points]
            worst_db = max(mags)
            pass_ok = worst_db <= threshold
            if not pass_ok:
                overall_pass = False

            fail_at = None
            for freq_hz, mag_db in points:
                if mag_db > threshold:
                    fail_at = freq_hz
                    break

            self.results_table.setItem(row, 1, QtWidgets.QTableWidgetItem("PASS" if pass_ok else "FAIL"))
            self.results_table.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{worst_db:.2f}"))
            self.results_table.setItem(
                row,
                3,
                QtWidgets.QTableWidgetItem(f"{fail_at:.3f}" if isinstance(fail_at, (int, float)) else ""),
            )

        if any(self._series_mag.get(p) for p in self.S_PARAMS):
            self.pass_label.setText("Overall: PASS" if overall_pass else "Overall: FAIL")
            self.pass_label.setStyleSheet(
                "font-weight: bold; color: #0a7f2e;" if overall_pass else "font-weight: bold; color: #c1121f;"
            )
        else:
            self.pass_label.setText("Overall: N/A")
            self.pass_label.setStyleSheet("font-weight: bold;")

        self._emit_plot_payload()

    def _emit_plot_payload(self):
        payload = {
            "mag": {k: list(v) for k, v in self._series_mag.items()},
            "phase": {k: list(v) for k, v in self._series_phase.items()},
            "show_mag": [k for k, chk in self.mag_checks.items() if chk.isChecked()],
            "show_phase": [k for k, chk in self.phase_checks.items() if chk.isChecked()],
            "colors": dict(self.COLORS_BY_PARAM),
        }
        self.plot_changed.emit(payload)
