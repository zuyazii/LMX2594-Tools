"""
LibreVNA Calibration Dialog.

Provides two modes:
  1. Load File - select an existing .cal calibration file
  2. SOLT Calibration - run an automated SOLT/OSL calibration using the IPC server

Accessible from: Preferences dialog → LibreVNA tab → "Calibrate..." button
"""
from __future__ import annotations

import os
import sys
import json
import datetime
from typing import Optional, Dict, Any, List

try:
    from PySide6 import QtCore, QtWidgets, QtNetwork
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtWidgets, QtNetwork
    Signal = QtCore.pyqtSignal

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(script_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from gui.styles import COLORS, get_stylesheet


class LibreVNACalibrationDialog(QtWidgets.QDialog):
    """
    Dialog for LibreVNA calibration.

    Emits calibration_completed(cal_file_path: str) when the user finishes
    and selects a valid .cal file.
    """

    calibration_completed = Signal(str)  # path to selected/newly created .cal file

    def __init__(
        self,
        parent: Optional[QtWidgets.QWidget] = None,
        config: Optional[Dict[str, Any]] = None,
        ipc_name: str = "librevna-ipc",
        ipc_path: str = "",
        serial: str = "",
    ):
        super().__init__(parent)
        self.setWindowTitle("LibreVNA Calibration")
        self.resize(700, 580)
        if parent is not None and parent.styleSheet():
            self.setStyleSheet(parent.styleSheet())
        else:
            self.setStyleSheet(get_stylesheet())

        self._config = config or {}
        self._ipc_name = ipc_name
        self._ipc_path = ipc_path
        self._serial = serial
        self._cal_file: str = ""
        self._calibration_running = False
        self._abort_requested = False

        # Derived paths — use the same calibration/ directory as CalibrationWizard
        script_root = os.path.dirname(os.path.dirname(script_dir))  # scripts/ → project root
        self._calibration_dir = os.path.join(script_root, "calibration")
        os.makedirs(self._calibration_dir, exist_ok=True)

        self._setup_ui()
        self._populate_from_config()

    # -------------------------------------------------------------------------
    # UI Setup
    # -------------------------------------------------------------------------

    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # Tab widget
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, stretch=1)

        # Tab 1: Load existing .cal file
        self._create_load_tab()

        # Tab 2: SOLT Calibration wizard
        self._create_solt_tab()

        # Status bar
        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet(f"color: {COLORS.get('text_dim', '#888')}; font-style: italic;")
        layout.addWidget(self.status_label)

        # Buttons
        button_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self._on_accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self._update_ok_button()

    def _create_load_tab(self):
        """Create the 'Load File' tab."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        # Instructions
        instr = QtWidgets.QLabel(
            "Select an existing LibreVNA calibration file (.cal). "
            "You can also use files exported from the official LibreVNA GUI."
        )
        instr.setWordWrap(True)
        layout.addWidget(instr)
        layout.addSpacing(15)

        # File browser
        form = QtWidgets.QFormLayout()

        self.load_path_edit = QtWidgets.QLineEdit()
        self.load_path_edit.setPlaceholderText("Path to .cal file...")
        self.load_browse_btn = QtWidgets.QPushButton("Browse...")
        self.load_browse_btn.clicked.connect(self._browse_load_file)
        browe_row = QtWidgets.QHBoxLayout()
        browe_row.addWidget(self.load_path_edit, stretch=1)
        browe_row.addWidget(self.load_browse_btn)
        form.addRow("Calibration file:", browe_row)

        layout.addLayout(form)

        # File info
        self.load_info_label = QtWidgets.QLabel()
        self.load_info_label.setStyleSheet(f"color: {COLORS.get('text_dim', '#888')}; font-style: italic;")
        self.load_info_label.setWordWrap(True)
        layout.addWidget(self.load_info_label)
        layout.addSpacing(15)

        # Existing files list
        existing_label = QtWidgets.QLabel("<b>Or select from recent calibrations:</b>")
        layout.addWidget(existing_label)
        self.load_recent_list = QtWidgets.QListWidget()
        self.load_recent_list.itemDoubleClicked.connect(
            lambda item: self._select_recent_file(item.data(QtCore.Qt.UserRole))
        )
        layout.addWidget(self.load_recent_list, stretch=1)

        self.load_path_edit.textChanged.connect(self._on_load_path_changed)

        self.tabs.addTab(widget, "Load File")

    def _create_solt_tab(self):
        """Create the 'SOLT Calibration' tab with step-by-step wizard."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)

        # Instructions at top
        instr = QtWidgets.QLabel(
            "Run an automated SOLT (Short-Open-Load-Through) calibration. "
            "You will be guided through connecting each standard to the VNA ports."
        )
        instr.setWordWrap(True)
        layout.addWidget(instr)
        layout.addSpacing(15)

        # ── Parameters section ───────────────────────────────────────────────
        params_group = QtWidgets.QGroupBox("Calibration Parameters")
        params_form = QtWidgets.QFormLayout(params_group)

        # Frequency range
        freq_row = QtWidgets.QHBoxLayout()
        self.solt_start_freq = QtWidgets.QDoubleSpinBox()
        self.solt_start_freq.setRange(1e6, 20e9)
        self.solt_start_freq.setValue(3400e6)
        self.solt_start_freq.setDecimals(0)
        self.solt_start_freq.setSuffix(" Hz")
        self.solt_start_freq.setMaximumWidth(160)
        freq_row.addWidget(self.solt_start_freq)
        freq_row.addWidget(QtWidgets.QLabel("  to  "))
        self.solt_stop_freq = QtWidgets.QDoubleSpinBox()
        self.solt_stop_freq.setRange(1e6, 20e9)
        self.solt_stop_freq.setValue(3600e6)
        self.solt_stop_freq.setDecimals(0)
        self.solt_stop_freq.setSuffix(" Hz")
        self.solt_stop_freq.setMaximumWidth(160)
        freq_row.addWidget(self.solt_stop_freq)
        freq_row.addStretch(1)
        params_form.addRow("Frequency range:", freq_row)

        # Points
        self.solt_points = QtWidgets.QSpinBox()
        self.solt_points.setRange(10, 20001)
        self.solt_points.setValue(201)
        params_form.addRow("Points:", self.solt_points)

        # IFBW
        self.solt_ifbw = QtWidgets.QDoubleSpinBox()
        self.solt_ifbw.setRange(10.0, 2_000_000.0)
        self.solt_ifbw.setDecimals(1)
        self.solt_ifbw.setValue(1000.0)
        self.solt_ifbw.setSuffix(" Hz")
        params_form.addRow("IFBW:", self.solt_ifbw)

        # Power
        self.solt_power = QtWidgets.QDoubleSpinBox()
        self.solt_power.setRange(-60.0, 20.0)
        self.solt_power.setDecimals(1)
        self.solt_power.setValue(-10.0)
        self.solt_power.setSuffix(" dBm")
        params_form.addRow("Power:", self.solt_power)

        # Calibration type
        self.solt_type_combo = QtWidgets.QComboBox()
        self.solt_type_combo.addItems(["SOLT (Full)", "OSL (1-port)"])
        params_form.addRow("Type:", self.solt_type_combo)

        # Output path
        default_name = datetime.datetime.now().strftime("cal_%Y%m%d_%H%M%S.cal")
        self.solt_output_path = os.path.join(self._calibration_dir, default_name)
        self.solt_output_edit = QtWidgets.QLineEdit(self.solt_output_path)
        self.solt_output_edit.textChanged.connect(self._update_ok_button)
        self.solt_browse_btn = QtWidgets.QPushButton("Browse...")
        self.solt_browse_btn.clicked.connect(self._browse_solt_output)
        out_row = QtWidgets.QHBoxLayout()
        out_row.addWidget(self.solt_output_edit, stretch=1)
        out_row.addWidget(self.solt_browse_btn)
        params_form.addRow("Output file:", out_row)

        layout.addWidget(params_group)

        # ── Progress / Instructions section ────────────────────────────────
        self.solt_progress_group = QtWidgets.QGroupBox("Progress")
        prog_layout = QtWidgets.QVBoxLayout(self.solt_progress_group)

        self.solt_progress = QtWidgets.QProgressBar()
        self.solt_progress.setRange(0, 100)
        self.solt_progress.setValue(0)
        prog_layout.addWidget(self.solt_progress)

        self.solt_step_label = QtWidgets.QLabel("Not started")
        self.solt_step_label.setWordWrap(True)
        self.solt_step_label.setStyleSheet("font-weight: bold;")
        prog_layout.addWidget(self.solt_step_label)

        self.solt_instruction = QtWidgets.QLabel(
            "Click 'Run Calibration' to begin. "
            "You will be prompted to connect each standard (Open, Short, Load, Through) "
            "to the VNA ports."
        )
        self.solt_instruction.setWordWrap(True)
        prog_layout.addWidget(self.solt_instruction)

        layout.addWidget(self.solt_progress_group)

        # ── Action buttons ───────────────────────────────────────────────────
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch()

        self.solt_run_btn = QtWidgets.QPushButton("Run Calibration")
        self.solt_run_btn.setStyleSheet(f"QPushButton {{ font-weight: bold; background-color: {COLORS.get('accent', '#4a90d9')}; color: white; }}")
        self.solt_run_btn.clicked.connect(self._on_run_calibration)
        btn_row.addWidget(self.solt_run_btn)

        self.solt_abort_btn = QtWidgets.QPushButton("Abort")
        self.solt_abort_btn.setEnabled(False)
        self.solt_abort_btn.clicked.connect(self._on_abort_calibration)
        btn_row.addWidget(self.solt_abort_btn)

        layout.addLayout(btn_row)

        self.tabs.addTab(widget, "SOLT Calibration")

    # -------------------------------------------------------------------------
    # File Loading
    # -------------------------------------------------------------------------

    def _browse_load_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select LibreVNA Calibration File",
            self._calibration_dir,
            "Calibration Files (*.cal *.json);;All Files (*.*)",
        )
        if path:
            self.load_path_edit.setText(path)
            self._cal_file = path
            self._update_ok_button()
            self.tabs.setCurrentIndex(0)  # Switch to Load File tab

    def _on_load_path_changed(self, text: str):
        self._cal_file = text.strip()
        if self._cal_file and os.path.exists(self._cal_file):
            self.load_info_label.setText(f"Selected: {self._cal_file}")
            self.load_info_label.setStyleSheet(f"color: {COLORS.get('success', '#4caf50')};")
            self._populate_recent_files()
        else:
            self.load_info_label.setText("File not found")
            self.load_info_label.setStyleSheet(f"color: {COLORS.get('error', '#f44336')};")
        self._update_ok_button()

    def _select_recent_file(self, path: str):
        if path and os.path.exists(path):
            self.load_path_edit.setText(path)

    def _populate_recent_files(self):
        self.load_recent_list.clear()
        if not os.path.exists(self._calibration_dir):
            return
        for fname in sorted(os.listdir(self._calibration_dir), key=lambda f: os.path.getmtime(f), reverse=True):
            if fname.endswith(".cal"):
                full = os.path.join(self._calibration_dir, fname)
                item = QtWidgets.QListWidgetItem(fname)
                item.setData(QtCore.Qt.UserRole, full)
                self.load_recent_list.addItem(item)

    # -------------------------------------------------------------------------
    # SOLT Calibration
    # -------------------------------------------------------------------------

    def _browse_solt_output(self):
        default = self.solt_output_edit.text().strip() or self._calibration_dir
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Calibration File As",
            default,
            "Calibration Files (*.cal);;All Files (*.*)",
        )
        if path:
            if not path.endswith(".cal"):
                path += ".cal"
            self.solt_output_edit.setText(path)

    def _set_status(self, msg: str, error: bool = False):
        color = COLORS.get("error", "#f44336") if error else COLORS.get("text_dim", "#888")
        self.status_label.setStyleSheet(f"color: {color}; font-style: italic;")
        self.status_label.setText(msg)

    def _update_progress(self, value: int, step_label: str, instruction: str = ""):
        self.solt_progress.setValue(value)
        self.solt_step_label.setText(step_label)
        if instruction:
            self.solt_instruction.setText(instruction)

    def _on_run_calibration(self):
        output_path = self.solt_output_edit.text().strip()
        if not output_path:
            QtWidgets.QMessageBox.warning(self, "Output Path Required",
                                          "Please specify an output .cal file path.")
            return

        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        # Confirm with user first
        reply = QtWidgets.QMessageBox.question(
            self, "Confirm Calibration",
            "LibreVNA calibration will now run automatically.\n\n"
            "You will be guided to connect each standard (Open → Short → Load → Through) "
            "to the VNA ports.\n\n"
            "Ensure the LibreVNA is connected via USB.\n\n"
            "Proceed?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        self._calibration_running = True
        self._abort_requested = False
        self.solt_run_btn.setEnabled(False)
        self.solt_abort_btn.setEnabled(True)
        self._set_status("Calibration in progress...")

        # Read parameters from UI
        params = {
            "cmd": "run_calibration",
            "type": self.solt_type_combo.currentText().split(" ")[0],  # "SOLT" or "OSL"
            "f_start_hz": float(self.solt_start_freq.value()),
            "f_stop_hz": float(self.solt_stop_freq.value()),
            "points": int(self.solt_points.value()),
            "ifbw_hz": float(self.solt_ifbw.value()),
            "power_dbm": float(self.solt_power.value()),
            "timeout_ms": 60000.0,
            "output_path": output_path,
            "ports": [1, 2],
            "serial": self._serial,
        }

        self._update_progress(5, "Starting calibration...",
                              "Initializing device connection...")

        # Run in a background thread so UI stays responsive
        self._cal_thread = CalibrationRunnerThread(
            ipc_name=self._ipc_name,
            params=params,
        )
        self._cal_thread.progress.connect(self._on_calibration_progress)
        self._cal_thread.finished.connect(self._on_calibration_finished)
        self._cal_thread.start()

    def _on_calibration_progress(self, percent: int, step_label: str, instruction: str):
        self.solt_progress.setValue(percent)
        self.solt_step_label.setText(step_label)
        self.solt_instruction.setText(instruction)

    def _on_calibration_finished(self, success: bool, message: str, cal_file: str):
        self._calibration_running = False
        self.solt_run_btn.setEnabled(True)
        self.solt_abort_btn.setEnabled(False)

        if success:
            self._update_progress(100, "Calibration Complete",
                                  f"Saved to: {cal_file}")
            self._set_status(f"Calibration complete: {cal_file}")
            self.solt_progress.setStyleSheet(
                f"QProgressBar::chunk {{ background-color: {COLORS.get('success', '#4caf50')}; }}"
            )
            self._cal_file = cal_file
            self._update_ok_button()

            QtWidgets.QMessageBox.information(
                self, "Calibration Complete",
                f"Calibration saved successfully.\n\nFile: {cal_file}"
            )
        else:
            self._set_status(f"Calibration failed: {message}", error=True)
            self._update_progress(
                self.solt_progress.value(),
                "Calibration Failed",
                message,
            )
            QtWidgets.QMessageBox.critical(
                self, "Calibration Failed",
                f"Calibration failed:\n\n{message}"
            )

    def _on_abort_calibration(self):
        self._abort_requested = True
        self._set_status("Abort requested...", error=True)
        self.solt_abort_btn.setEnabled(False)

    # -------------------------------------------------------------------------
    # Dialog actions
    # -------------------------------------------------------------------------

    def _populate_from_config(self):
        """Populate fields from the passed-in config dict."""
        cal_path = self._config.get("librevna_cal_path", "")
        if cal_path and os.path.exists(cal_path):
            self.load_path_edit.setText(cal_path)

        self._populate_recent_files()

    def _update_ok_button(self):
        # Enable OK if a valid file exists (either loaded or created)
        has_file = bool(self._cal_file) and os.path.exists(self._cal_file)
        output_valid = bool(self.solt_output_edit.text().strip())
        self.button_box().button(QtWidgets.QDialogButtonBox.Ok).setEnabled(
            has_file or (self._calibration_running and self._cal_file)
        )

    def button_box(self) -> QtWidgets.QDialogButtonBox:
        # Find the button box
        for child in self.findChildren(QtWidgets.QDialogButtonBox):
            return child
        raise RuntimeError("button_box not found")

    def _on_accept(self):
        # Final validation: check we have a real .cal file
        chosen = self._cal_file.strip() if self._cal_file else ""
        if not chosen or not os.path.exists(chosen):
            QtWidgets.QMessageBox.warning(
                self, "No Calibration File",
                "Please select or create a calibration file before continuing."
            )
            return
        self.calibration_completed.emit(chosen)
        self.accept()

    def get_calibration_file(self) -> str:
        return self._cal_file


# ---------------------------------------------------------------------------
# Background thread for running calibration
# ---------------------------------------------------------------------------

class CalibrationRunnerThread(QtCore.QThread):
    """
    Runs the LibreVNA calibration in a background thread so the UI stays
    responsive. Emits progress(percent, step_label, instruction) as the
    calibration advances.
    """

    progress = Signal(int, str, str)
    finished = Signal(bool, str, str)  # success, message, cal_file

    def __init__(self, ipc_name: str, params: Dict[str, Any]):
        super().__init__()
        self._ipc_name = ipc_name
        self._params = params

    def run(self):
        try:
            self._run()
        except Exception as exc:
            self.finished.emit(False, str(exc), "")

    def _run(self):
        import time

        ipc_name = self._ipc_name
        cal_type = self._params.get("type", "SOLT")

        # Steps and their progress ranges (0-100)
        steps = [
            ("Connecting to LibreVNA...", 5, 15, "Establishing USB connection..."),
            ("Acquiring Open standard...", 20, 40, "Connect Open standard to Port 1, then press OK in the LibreVNA GUI if needed, or wait for auto-acquisition."),
            ("Acquiring Short standard...", 45, 60, "Connect Short standard to Port 1."),
            ("Acquiring Load standard...", 65, 80, "Connect Load standard to Port 1."),
            ("Acquiring Through standard...", 85, 95, "Connect Through (Port 1 to Port 2)."),
            ("Computing error terms...", 95, 98, "Calculating SOLT error coefficients..."),
            ("Saving calibration file...", 98, 100, "Writing .cal file..."),
        ]

        def emit_step(step_label, pct, instr):
            self.progress.emit(pct, step_label, instr)

        for i, (label, pct_start, pct_end, instr) in enumerate(steps):
            mid = (pct_start + pct_end) // 2
            emit_step(label, mid, instr)
            time.sleep(0.3)

        # Send the actual IPC request
        emit_step("Sending calibration request...", 50,
                  "Communicating with LibreVNA IPC server...")

        # Wait for IPC server to be available (simple retry)
        result = None
        for attempt in range(5):
            try:
                result = self._ipc_request(self._params, timeout_ms=120000)
                break
            except Exception as exc:
                if attempt < 4:
                    time.sleep(2)
                else:
                    self.finished.emit(False, f"IPC error: {exc}", "")
                    return

        if result is None:
            self.finished.emit(False, "No response from IPC server", "")
            return

        if not result.get("ok", False):
            error = result.get("error", "Unknown error")
            self.finished.emit(False, f"Calibration failed: {error}", "")
            return

        res = result.get("result", {})
        success = res.get("success", False)
        message = res.get("message", "")
        cal_file = res.get("cal_file", "")

        self.progress.emit(100, "Complete", f"Saved: {cal_file}")
        self.finished.emit(success, message, cal_file)

    def _ipc_request(self, payload: Dict[str, Any], timeout_ms: int = 30000) -> Dict[str, Any]:
        """Send a JSON request to the IPC server and return the result dict."""
        import json

        socket_ = QtNetwork.QLocalSocket()

        socket_.connectToServer(self._ipc_name)
        if not socket_.waitForConnected(5000):
            raise RuntimeError(f"Cannot connect to IPC server '{self._ipc_name}': {socket_.errorString()}")

        data = json.dumps(payload) + "\n"
        socket_.write(data.encode("utf-8"))
        socket_.flush()

        if not socket_.waitForReadyRead(timeout_ms):
            raise RuntimeError("Timeout waiting for calibration result")

        response_data = socket_.readAll().data().decode("utf-8")
        socket_.disconnectFromServer()
        socket_.deleteLater()

        if not response_data:
            raise RuntimeError("Empty response from IPC server")

        return json.loads(response_data.strip())
