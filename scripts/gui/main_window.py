"""
Main window for the LMX2594 calibration GUI.
Modern dark theme with sidebar, toolbar, and dynamic plots.
"""
from __future__ import annotations

import datetime
import json
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional

# Add parent directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

MSYS2_UCRT64_BIN = os.environ.get("MSYS2_UCRT64_BIN", r"C:\msys64\ucrt64\bin")
MSYS2_QT_PLUGIN_PATH = os.environ.get("MSYS2_QT_PLUGIN_PATH", r"C:\msys64\ucrt64\share\qt6\plugins")
SYSTEM_ROOT = os.environ.get("SystemRoot", r"C:\Windows")
WINDOWS_SYSTEM32 = os.path.join(SYSTEM_ROOT, "System32")

try:
    from PySide6 import QtCore, QtGui, QtWidgets, QtNetwork
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets, QtNetwork
    Signal = QtCore.pyqtSignal

from gui.styles import get_stylesheet, COLORS
from gui.sidebar import Sidebar
from gui.plots import DynamicPlotWidget
from gui.tabs.librevna_tab import LibreVNATab
from gui.tabs.tinysa_tab import TinySATab
from gui.tabs.gain_measure_tab import GainMeasureTab
from gui.tabs.logs_tab import LogsTab
from gui.tabs.lmx_monitor_tab import LMXMonitorTab
from gui.dialogs.calibration_wizard import CalibrationWizard


class SweepWorker(QtCore.QThread):
    """Background worker for tinySA and/or LibreVNA sweep."""

    log = Signal(str)
    failed = Signal(str)
    tiny_done = Signal(list)
    libre_done = Signal(dict)
    point_update = Signal(dict)  # Emits frequency plan details per sweep point
    progress_update = Signal(int, int)  # current, total
    run_finished = Signal()

    class LMXWorkerClient:
        """Small JSON-line client for lmx2594_worker.py."""

        def __init__(self, python_path: str, on_log=None):
            self._python_path = python_path or sys.executable
            self._worker_script = os.path.join(parent_dir, "lmx2594_worker.py")
            self._proc = None
            self._on_log = on_log

        def _log(self, msg: str):
            if self._on_log:
                self._on_log(msg)

        def start(self):
            if self._proc and self._proc.poll() is None:
                return
            if not os.path.exists(self._worker_script):
                raise RuntimeError(f"Worker script not found: {self._worker_script}")

            try:
                bits = subprocess.check_output(
                    [self._python_path, "-c", "import struct;print(struct.calcsize('P')*8)"],
                    text=True,
                    timeout=5,
                ).strip()
            except Exception as exc:
                raise RuntimeError(f"Failed to run worker Python: {exc}") from exc
            if bits != "32":
                raise RuntimeError(
                    f"Worker Python must be 32-bit for USB2ANY (current: {bits}-bit): {self._python_path}"
                )

            self._proc = subprocess.Popen(
                [self._python_path, self._worker_script],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                cwd=parent_dir,
            )

        def request(self, payload: Dict[str, object]) -> Dict[str, object]:
            self.start()
            if not self._proc or not self._proc.stdin or not self._proc.stdout:
                raise RuntimeError("Worker process not ready")
            cmd = str(payload.get("cmd", "unknown"))
            self._log(f"LMX worker -> {cmd}")
            self._proc.stdin.write(json.dumps(payload) + "\n")
            self._proc.stdin.flush()
            line = self._proc.stdout.readline()
            if not line:
                err = ""
                if self._proc.stderr:
                    err = self._proc.stderr.read().strip()
                raise RuntimeError(f"Worker did not respond{': ' + err if err else ''}")
            response = json.loads(line)
            if not response.get("ok", False):
                raise RuntimeError(str(response.get("error", "Worker error")))
            self._log(f"LMX worker <- {cmd} ok")
            result = response.get("result", {})
            return result if isinstance(result, dict) else {}

        def stop(self):
            if not self._proc:
                return
            try:
                self.request({"cmd": "shutdown"})
            except Exception:
                pass
            try:
                self._proc.terminate()
                self._proc.wait(timeout=2)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            self._proc = None

    def __init__(
        self,
        mode: str,
        settings: Dict[str, object],
        tinysa_port: Optional[str],
        lmx_serial: Optional[str],
        librevna_serial: Optional[str],
        start_text: str,
        stop_text: str,
        step_mode_per_freq: bool,
        step_value_text: str,
        ipc_name: str,
        parent: Optional[QtCore.QObject] = None,
        use_lmx: bool = True,
    ):
        super().__init__(parent)
        self._mode = mode
        self._settings = dict(settings)
        self._tinysa_port = tinysa_port
        self._lmx_serial = lmx_serial
        self._librevna_serial = librevna_serial
        self._start_text = start_text
        self._stop_text = stop_text
        self._step_mode_per_freq = step_mode_per_freq
        self._step_value_text = step_value_text
        self._ipc_name = ipc_name
        self._stop_requested = False
        # Whether to use USB2ANY/LMX during tinySA sweeps
        self._use_lmx = bool(use_lmx)

    def request_stop(self):
        self._stop_requested = True

    def run(self):
        try:
            start_hz, stop_hz, points = self._build_frequency_points()

            if self._mode in ("Both", "LibreVNA only"):
                payload = self._run_librevna(start_hz, stop_hz, len(points))
                self.libre_done.emit(payload)

            if self._mode in ("Both", "tinySA only"):
                tiny_results = self._run_tinysa(points)
                self.tiny_done.emit(tiny_results)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.run_finished.emit()

    def _build_frequency_points(self):
        from tinysa_antenna_test import build_frequency_points, parse_frequency

        start_hz = parse_frequency(self._start_text)
        stop_hz = parse_frequency(self._stop_text)
        if self._step_mode_per_freq:
            step_hz = parse_frequency(self._step_value_text)
            points = build_frequency_points(start_hz, stop_hz, step_hz, None)
        else:
            count = int(self._step_value_text)
            points = build_frequency_points(start_hz, stop_hz, None, count)
        if not points:
            raise ValueError("No frequency points generated")
        return start_hz, stop_hz, points

    def _run_tinysa(self, points):
        import datetime as dt
        from tinysa_antenna_test import TinySAController, parse_frequency

        self.log.emit(f"tinySA sweep start ({len(points)} points)")
        
        # Load calibration data if available
        self._tinysa_cal_data = None
        tinysa_cal_path = self._settings.get("tinysa_cal_path", "")
        if tinysa_cal_path and os.path.exists(tinysa_cal_path):
            try:
                from lmx2594_calibration import load_calibration_file
                self._tinysa_cal_data = load_calibration_file(tinysa_cal_path)
                self.log.emit(f"Loaded tinySA calibration: {os.path.basename(tinysa_cal_path)}")
            except Exception as e:
                self.log.emit(f"Warning: Failed to load tinySA calibration: {e}")
        
        tinysa = TinySAController(
            self._tinysa_port,
            verbose=False,
            error_byte=False,
            debug_cb=self.log.emit if bool(self._settings.get("tinysa_debug")) else None,
        )
        lmx = None
        if self._use_lmx:
            lmx = self.LMXWorkerClient(
                str(self._settings.get("worker_python", sys.executable)),
                on_log=self.log.emit,
            )
        results = []
        try:
            template_path = str(self._settings.get("template_path", "")).strip()
            if not template_path:
                template_path = os.path.join(
                    os.path.dirname(parent_dir),
                    "examples",
                    "register-values",
                    "HexRegisterValues3600.txt",
                )

            fosc_hz = parse_frequency(str(self._settings.get("fosc", "50MHz")))
            spi_clock_hz = parse_frequency(str(self._settings.get("spi_clock", "400kHz")))

            if self._use_lmx and lmx is not None:
                self.log.emit(f"Connecting LMX worker (USB2ANY serial={self._lmx_serial or 'Auto'})")
                lmx.request(
                    {
                        "cmd": "connect",
                        "serial": self._lmx_serial,
                        "f_osc": float(fosc_hz),
                        "template_path": template_path,
                        "spi_clock": float(spi_clock_hz),
                        "delta_update": bool(self._settings.get("delta_update", True)),
                        "outa_pwr": int(self._settings.get("outa_pwr", 50)),
                    }
                )

            self.log.emit(f"Connecting tinySA (port={self._tinysa_port or 'Auto'})")
            tinysa.connect()

            for idx, freq_hz in enumerate(points):
                if self._stop_requested:
                    self.log.emit("tinySA sweep stop requested")
                    break

                # Emit progress update
                self.progress_update.emit(idx + 1, len(points))

                if self._use_lmx and lmx is not None:
                    if idx == 0:
                        freq_resp = lmx.request({"cmd": "program_initial", "freq_hz": float(freq_hz)})
                    else:
                        freq_resp = lmx.request({"cmd": "update_frequency", "freq_hz": float(freq_hz)})

                    # Emit frequency plan details for LMX Monitor tab
                    if isinstance(freq_resp, dict):
                        self.point_update.emit(freq_resp)

                    if bool(self._settings.get("wait_lock", True)):
                        lock_resp = lmx.request(
                            {
                                "cmd": "wait_lock",
                                "timeout_s": float(self._settings.get("lock_timeout_s", 1.0)),
                                "auto_recal": bool(self._settings.get("auto_recal", True)),
                            }
                        )
                        if lock_resp.get("locked") is False:
                            raise RuntimeError(f"LMX lock failed at {freq_hz:.0f} Hz")

                dwell_ms = float(self._settings.get("dwell_ms", 0.0))
                if dwell_ms > 0:
                    time.sleep(dwell_ms / 1000.0)

                power_dbm = tinysa.scan_point(float(freq_hz), outmask=2, timeout_s=5.0, retries=1)
                
                # Apply calibration offset if available
                calibrated_dbm = power_dbm
                cal_offset = 0.0
                if hasattr(self, '_tinysa_cal_data') and self._tinysa_cal_data:
                    try:
                        from lmx2594_calibration import get_calibration_offset
                        cal_offset = get_calibration_offset(self._tinysa_cal_data, freq_hz)
                        calibrated_dbm = power_dbm - cal_offset
                    except Exception:
                        pass
                
                results.append(
                    {
                        "frequency_hz": float(freq_hz),
                        "power_dbm": float(power_dbm),
                        "power_dbm_calibrated": float(calibrated_dbm),
                        "calibration_offset_db": float(cal_offset),
                        "timestamp": dt.datetime.utcnow().isoformat() + "Z",
                    }
                )
            self.log.emit(f"tinySA sweep complete ({len(results)} points)")
            return results
        finally:
            if self._use_lmx and lmx is not None:
                try:
                    lmx.request({"cmd": "disconnect"})
                except Exception:
                    pass
                lmx.stop()
            try:
                tinysa.disconnect()
            except Exception:
                pass

    def _run_librevna(self, start_hz: float, stop_hz: float, points: int) -> Dict[str, object]:
        self.log.emit(f"LibreVNA sweep start ({points} points)")
        payload: Dict[str, object] = {
            "cmd": "run_sweep",
            "cal_path": str(self._settings.get("librevna_cal_path", "")),
            "f_start_hz": float(start_hz),
            "f_stop_hz": float(stop_hz),
            "points": int(points),
            "ifbw_hz": float(self._settings.get("librevna_ifbw_hz", 10000.0)),
            "power_dbm": float(self._settings.get("librevna_power_dbm", -10.0)),
            "threshold_db": float(self._settings.get("librevna_threshold_db", -60.0)),
            "timeout_ms": float(self._settings.get("librevna_timeout_ms", 15000.0)),
            "excited_ports": list(self._settings.get("librevna_excited_ports", [1, 2])),
        }
        if self._librevna_serial and self._librevna_serial.lower() != "auto":
            payload["serial"] = self._librevna_serial
        response = self._ipc_request(payload, timeout_ms=int(payload["timeout_ms"]) + 5000)
        self.log.emit("LibreVNA sweep complete")
        return response

    def _ipc_request(self, payload: Dict[str, object], timeout_ms: int = 5000) -> Dict[str, object]:
        socket = QtNetwork.QLocalSocket()
        socket.connectToServer(self._ipc_name)
        if not socket.waitForConnected(timeout_ms):
            raise RuntimeError(f"IPC connect failed: {socket.errorString()}")

        socket.write((json.dumps(payload) + "\n").encode("utf-8"))
        socket.flush()

        buffer = bytearray()
        timer = QtCore.QElapsedTimer()
        timer.start()
        while True:
            idx = buffer.find(b"\n")
            if idx >= 0:
                line = bytes(buffer[:idx]).strip()
                break
            if not socket.waitForReadyRead(200):
                if timer.elapsed() > timeout_ms:
                    raise RuntimeError("IPC response timeout")
                continue
            buffer.extend(bytes(socket.readAll()))

        if not line:
            raise RuntimeError("IPC returned empty response")
        response = json.loads(line.decode("utf-8"))
        if not response.get("ok", False):
            raise RuntimeError(str(response.get("error", "IPC error")))
        result = response.get("result")
        if not isinstance(result, dict):
            raise RuntimeError("IPC returned invalid result payload")
        return result


class CalibrationApp(QtWidgets.QMainWindow):
    """Main application window."""
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("LMX2594 Calibration")
        self.resize(1400, 900)
        
        # Apply dark theme
        self.setStyleSheet(get_stylesheet())
        
        # Initialize state
        self._init_state()
        
        # Build UI
        self._create_menu_bar()
        self._create_toolbar()
        self._create_central_widget()
        self._load_persistent_state()
        self._settings["worker_python"] = self._resolve_worker_python_path(prefer_x86=True)
        QtCore.QTimer.singleShot(200, self._on_device_refresh)
        QtCore.QTimer.singleShot(1800, self._on_device_refresh)
        QtCore.QTimer.singleShot(2500, self._check_load_previous_data)
        
        # Start periodic status polling timer (2 second interval)
        self._status_poll_timer = QtCore.QTimer(self)
        self._status_poll_timer.timeout.connect(self._poll_device_status)
        self._status_poll_timer.start(2000)
        
    def _init_state(self):
        """Initialize application state variables."""
        self._devices_connected = {
            "tinysa": False,
            "librevna": False,
            "usb2any": False,
        }
        self._sweep_running = False
        self._polling_paused = False
        self._sweep_worker = None
        self._calibration_results = {}
        self._settings = {
            "fosc": "50MHz",
            "pfd": "50MHz",
            "spi_clock": "400kHz",
            "template_path": "",
            "outa_pwr": 50,
            "dwell_ms": 0.0,
            "wait_lock": True,
            "delta_update": True,
            "auto_recal": True,
            "tinysa_debug": False,
            "lock_timeout_s": 1.0,
            "worker_python": sys.executable,
            "librevna_ipc_path": "",
            "librevna_ipc_name": "librevna-ipc",
            "librevna_cal_path": "",
            "librevna_ifbw_hz": 10_000.0,
            "librevna_power_dbm": -10.0,
            "librevna_threshold_db": -60.0,
            "librevna_timeout_ms": 15_000.0,
            "librevna_excited_ports": [1, 2],
        }
        self._librevna_ipc_name = "librevna-ipc"
        self._librevna_ipc_process: Optional[QtCore.QProcess] = None
        self._librevna_ipc_path_override: Optional[str] = None
        self._session_path: Optional[str] = None
        self._persist_settings = QtCore.QSettings("LMX2594", "CalibrationGUI")
        self._pending_device_selection: Dict[str, object] = {}
        self._pending_golden_samples: List[object] = []
        self._pending_tested_results: List[object] = []
        self._last_librevna_plot_payload: Dict[str, object] = {}
        self._last_tinysa_plot_data: Dict[str, object] = {}  # Store tinySA plot data
        
        # Status polling timer (2 second interval)
        self._status_poll_timer: Optional[QtCore.QTimer] = None
    
    def _create_menu_bar(self):
        """Create the menu bar."""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        file_menu.addAction("New Session", self._on_new_session)
        file_menu.addAction("Open Session", self._on_open_session)
        file_menu.addAction("Save Session", self._on_save_session)
        file_menu.addSeparator()
        file_menu.addAction("Export Results", self._on_export_results)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close)
        
        # Edit menu
        edit_menu = menubar.addMenu("Edit")
        edit_menu.addAction("Preferences", self._on_preferences)
        
        # Help menu
        help_menu = menubar.addMenu("Help")
        help_menu.addAction("Documentation", self._on_documentation)
        help_menu.addAction("About", self._on_about)
    
    def _create_toolbar(self):
        """Create the draggable toolbar with sweep controls."""
        toolbar = QtWidgets.QToolBar("Sweep Controls")
        toolbar.setMovable(True)
        toolbar.setFloatable(True)
        self.addToolBar(QtCore.Qt.TopToolBarArea, toolbar)
        
        # Run button
        self.run_button = QtWidgets.QPushButton("Run")
        self.run_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.run_button.clicked.connect(self._on_run_sweep)
        toolbar.addWidget(self.run_button)
        
        # Stop button
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop))
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self._on_stop_sweep)
        toolbar.addWidget(self.stop_button)
        
        # Measure Golden Sample button (visible only when Antenna Test tab is active)
        self.measure_golden_sample_button = QtWidgets.QPushButton("Measure Golden Sample")
        self.measure_golden_sample_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.measure_golden_sample_button.clicked.connect(self._on_measure_golden_sample)
        self.measure_golden_sample_action = toolbar.addWidget(self.measure_golden_sample_button)
        self.measure_golden_sample_action.setVisible(False)
        
        # Reset button
        self.reset_button = QtWidgets.QPushButton("Reset")
        self.reset_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.reset_button.clicked.connect(self._on_reset)
        toolbar.addWidget(self.reset_button)

        toolbar.addSeparator()
        toolbar.addWidget(QtWidgets.QLabel("Sweep:"))
        self.chk_usb2any = QtWidgets.QCheckBox("USB2ANY/LMX")
        self.chk_usb2any.setChecked(True)
        self.chk_tinysa = QtWidgets.QCheckBox("tinySA")
        self.chk_tinysa.setChecked(True)
        self.chk_librevna = QtWidgets.QCheckBox("LibreVNA")
        self.chk_librevna.setChecked(True)
        toolbar.addWidget(self.chk_usb2any)
        toolbar.addWidget(self.chk_tinysa)
        toolbar.addWidget(self.chk_librevna)
        
        toolbar.addSeparator()
        
        # Device status indicators (read-only)
        self.usb2any_status_label = QtWidgets.QLabel()
        self.tinysa_status_label = QtWidgets.QLabel()
        self.librevna_status_label = QtWidgets.QLabel()
        toolbar.addWidget(self.usb2any_status_label)
        toolbar.addWidget(self.tinysa_status_label)
        toolbar.addWidget(self.librevna_status_label)
        self._set_device_status_indicator(self.usb2any_status_label, "USB2ANY", False)
        self._set_device_status_indicator(self.tinysa_status_label, "tinySA", False)
        self._set_device_status_indicator(self.librevna_status_label, "LibreVNA", False)
    
    def _create_central_widget(self):
        """Create the central widget with sidebar, plot, and tabs."""
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # Left sidebar (will be created in Phase 2)
        self.sidebar = self._create_sidebar()
        main_layout.addWidget(self.sidebar)
        
        # Right side: plot + tabs
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)
        
        # Plot area (will be created in Phase 3)
        self.plot_widget = self._create_plot_area()
        right_layout.addWidget(self.plot_widget, stretch=2)
        
        # Bottom tabs (will be created in Phase 4)
        self.tab_widget = self._create_tabs()
        right_layout.addWidget(self.tab_widget, stretch=1)
        
        main_layout.addWidget(right_widget, stretch=1)
    
    def _create_sidebar(self):
        """Create the left sidebar."""
        sidebar = Sidebar()
        
        # Connect signals
        sidebar.device_refresh_requested.connect(self._on_device_refresh)
        sidebar.calibration_requested.connect(self._on_calibration_wizard)
        
        return sidebar
    
    def _create_plot_area(self):
        """Create the plot area."""
        plot = DynamicPlotWidget()
        return plot
    
    def _create_tabs(self):
        """Create the bottom tabs."""
        tabs = QtWidgets.QTabWidget()
        
        # Create tab widgets
        self.librevna_tab = LibreVNATab()
        self.tinysa_tab = TinySATab()
        self.gain_measure_tab = GainMeasureTab()
        self.lmx_monitor_tab = LMXMonitorTab()
        self.logs_tab = LogsTab()

        self.librevna_tab.plot_changed.connect(self._on_librevna_plot_changed)
        self.gain_measure_tab.plot_requested.connect(self._on_gain_plot_requested)
        
        # Add tabs
        tabs.addTab(self.librevna_tab, "LibreVNA")
        tabs.addTab(self.tinysa_tab, "tinySA")
        tabs.addTab(self.gain_measure_tab, "Antenna Test")
        tabs.addTab(self.lmx_monitor_tab, "LMX Monitor")
        tabs.addTab(self.logs_tab, "Logs")
        
        # Connect tab change to plot mode
        tabs.currentChanged.connect(self._on_tab_changed)
        
        return tabs
    
    # Menu action handlers
    def _on_new_session(self):
        """Handle New Session action."""
        self._session_path = None
        self._calibration_results = {}
        self.sidebar.freq_start_edit.setText("3.52GHz")
        self.sidebar.freq_end_edit.setText("3.6GHz")
        self.sidebar.step_per_freq_radio.setChecked(True)
        self.sidebar.step_value_edit.setText("10MHz")
        self.sidebar.s11_threshold_spin.setValue(-10.0)
        self.sidebar.s11_tolerance_spin.setValue(2.0)
        self.sidebar.gain_tolerance_spin.setValue(1.5)
        self.sidebar.noise_threshold_spin.setValue(-80.0)
        self.librevna_tab.clear_all()
        self.tinysa_tab.results_table.setRowCount(0)
        self.gain_measure_tab.clear_all()
        self.plot_widget.clear()
        self.logs_tab.event_log.clear()
        self.logs_tab.ipc_log.clear()
        self.statusBar().showMessage("Started new session", 3000)
        self.logs_tab.append_event("New session created")
    
    def _on_open_session(self):
        """Handle Open Session action."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open Session",
            self._session_path or "",
            "Session Files (*.json);;All Files (*.*)",
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            self._apply_session_data(data)
            self._session_path = path
            self.statusBar().showMessage(f"Opened session: {path}", 5000)
            self.logs_tab.append_event(f"Session opened: {path}")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Open Session", f"Failed to open session:\n{exc}")
    
    def _on_save_session(self):
        """Handle Save Session action."""
        default_name = self._session_path or os.path.join(parent_dir, "session.json")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Session",
            default_name,
            "Session Files (*.json);;All Files (*.*)",
        )
        if not path:
            return
        try:
            payload = self._collect_session_data()
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
            self._session_path = path
            self.statusBar().showMessage(f"Saved session: {path}", 5000)
            self.logs_tab.append_event(f"Session saved: {path}")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Save Session", f"Failed to save session:\n{exc}")
    
    def _on_export_results(self):
        """Handle Export Results action."""
        index = self.tab_widget.currentIndex()
        table = None
        default_name = "results.csv"
        if index == 0:
            table = self.librevna_tab.results_table
            default_name = "librevna_results.csv"
        elif index == 1:
            table = self.tinysa_tab.results_table
            default_name = "tinysa_results.csv"
        elif index == 2:
            # Gain Measure tab - export all tables as JSON
            self._export_gain_measure_results()
            return

        if table is None:
            QtWidgets.QMessageBox.information(
                self, "Export Results", "Current tab has no exportable table."
            )
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export Results",
            default_name,
            "CSV Files (*.csv);;All Files (*.*)",
        )
        if not path:
            return
        try:
            self._export_table_to_csv(table, path)
            self.statusBar().showMessage(f"Exported results: {path}", 5000)
            self.logs_tab.append_event(f"Results exported: {path}")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Export Results", f"Failed to export results:\n{exc}")

    def _export_gain_measure_results(self):
        """Export Gain Measure tab results as JSON."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export Gain Measure Results",
            "gain_measure_results.json",
            "JSON Files (*.json);;All Files (*.*)",
        )
        if not path:
            return

        try:
            export_data = {
                "type": "gain_measure_export",
                "exported_at": datetime.datetime.now().isoformat(),
                "golden_samples": self.gain_measure_tab.golden_table.get_records(),
                "tested_antennas": self.gain_measure_tab.tested_table.get_records(),
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(export_data, f, indent=2)
            self.statusBar().showMessage(f"Exported gain results: {path}", 5000)
            self.logs_tab.append_event(f"Gain measure results exported: {path}")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self, "Export Error", f"Failed to export:\n{exc}"
            )
    
    def _on_hidden_settings(self):
        """Handle Hidden Settings action."""
        self._on_preferences()

    def _on_preferences(self):
        """Handle Preferences action."""
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Preferences")
        dialog.resize(640, 520)

        root_layout = QtWidgets.QVBoxLayout(dialog)
        tabs = QtWidgets.QTabWidget()
        root_layout.addWidget(tabs)

        device_page = QtWidgets.QWidget()
        device_form = QtWidgets.QFormLayout(device_page)

        ipc_name_edit = QtWidgets.QLineEdit(str(self._settings.get("librevna_ipc_name", "librevna-ipc")))
        ipc_path_edit = QtWidgets.QLineEdit(
            str(self._settings.get("librevna_ipc_path") or self._default_librevna_ipc_path() or "")
        )
        ipc_browse_btn = QtWidgets.QPushButton("Browse")
        ipc_row = QtWidgets.QHBoxLayout()
        ipc_row.addWidget(ipc_path_edit)
        ipc_row.addWidget(ipc_browse_btn)

        worker_python_edit = QtWidgets.QLineEdit(self._resolve_worker_python_path(prefer_x86=True))
        worker_browse_btn = QtWidgets.QPushButton("Browse")
        worker_row = QtWidgets.QHBoxLayout()
        worker_row.addWidget(worker_python_edit)
        worker_row.addWidget(worker_browse_btn)

        tinysa_debug_check = QtWidgets.QCheckBox()
        tinysa_debug_check.setChecked(bool(self._settings.get("tinysa_debug", False)))

        device_form.addRow("LibreVNA IPC name", ipc_name_edit)
        device_form.addRow("LibreVNA IPC binary", ipc_row)
        device_form.addRow("Worker Python (32-bit)", worker_row)
        device_form.addRow("tinySA debug", tinysa_debug_check)
        tabs.addTab(device_page, "Device")

        calib_page = QtWidgets.QWidget()
        calib_form = QtWidgets.QFormLayout(calib_page)

        fosc_edit = QtWidgets.QLineEdit(str(self._settings.get("fosc", "50MHz")))
        pfd_edit = QtWidgets.QLineEdit(str(self._settings.get("pfd", "50MHz")))
        spi_edit = QtWidgets.QLineEdit(str(self._settings.get("spi_clock", "400kHz")))
        template_path_edit = QtWidgets.QLineEdit(str(self._settings.get("template_path", "")))
        template_browse_btn = QtWidgets.QPushButton("Browse")
        template_row = QtWidgets.QHBoxLayout()
        template_row.addWidget(template_path_edit)
        template_row.addWidget(template_browse_btn)

        outa_pwr_spin = QtWidgets.QSpinBox()
        outa_pwr_spin.setRange(0, 63)
        outa_pwr_spin.setValue(int(self._settings.get("outa_pwr", 50)))

        dwell_spin = QtWidgets.QDoubleSpinBox()
        dwell_spin.setRange(0.0, 10000.0)
        dwell_spin.setDecimals(1)
        dwell_spin.setValue(float(self._settings.get("dwell_ms", 0.0)))

        lock_timeout_spin = QtWidgets.QDoubleSpinBox()
        lock_timeout_spin.setRange(0.0, 60.0)
        lock_timeout_spin.setDecimals(2)
        lock_timeout_spin.setValue(float(self._settings.get("lock_timeout_s", 0.0)))

        auto_recal_check = QtWidgets.QCheckBox()
        auto_recal_check.setChecked(bool(self._settings.get("auto_recal", True)))

        wait_lock_check = QtWidgets.QCheckBox()
        wait_lock_check.setChecked(bool(self._settings.get("wait_lock", True)))

        delta_update_check = QtWidgets.QCheckBox()
        delta_update_check.setChecked(bool(self._settings.get("delta_update", True)))

        calib_form.addRow("Fosc", fosc_edit)
        calib_form.addRow("PFD", pfd_edit)
        calib_form.addRow("USB2ANY SPI clock", spi_edit)
        calib_form.addRow("LMX template", template_row)
        calib_form.addRow("OUTA_PWR (0-63)", outa_pwr_spin)
        calib_form.addRow("Dwell ms", dwell_spin)
        calib_form.addRow("Wait lock", wait_lock_check)
        calib_form.addRow("Delta update", delta_update_check)
        calib_form.addRow("Lock timeout (s)", lock_timeout_spin)
        calib_form.addRow("Auto recal", auto_recal_check)
        tabs.addTab(calib_page, "Calibration")

        vna_page = QtWidgets.QWidget()
        vna_form = QtWidgets.QFormLayout(vna_page)

        cal_path_edit = QtWidgets.QLineEdit(str(self._settings.get("librevna_cal_path", "")))
        cal_browse_btn = QtWidgets.QPushButton("Browse")
        cal_row = QtWidgets.QHBoxLayout()
        cal_row.addWidget(cal_path_edit)
        cal_row.addWidget(cal_browse_btn)

        ifbw_spin = QtWidgets.QDoubleSpinBox()
        ifbw_spin.setRange(10.0, 2_000_000.0)
        ifbw_spin.setDecimals(1)
        ifbw_spin.setValue(float(self._settings.get("librevna_ifbw_hz", 10_000.0)))
        ifbw_spin.setSuffix(" Hz")

        power_spin = QtWidgets.QDoubleSpinBox()
        power_spin.setRange(-60.0, 20.0)
        power_spin.setDecimals(1)
        power_spin.setValue(float(self._settings.get("librevna_power_dbm", -10.0)))
        power_spin.setSuffix(" dBm")

        threshold_spin = QtWidgets.QDoubleSpinBox()
        threshold_spin.setRange(-150.0, 20.0)
        threshold_spin.setDecimals(1)
        threshold_spin.setValue(float(self._settings.get("librevna_threshold_db", -60.0)))
        threshold_spin.setSuffix(" dB")

        timeout_spin = QtWidgets.QDoubleSpinBox()
        timeout_spin.setRange(1000.0, 120000.0)
        timeout_spin.setDecimals(0)
        timeout_spin.setValue(float(self._settings.get("librevna_timeout_ms", 15000.0)))
        timeout_spin.setSuffix(" ms")

        ports_widget = QtWidgets.QWidget()
        ports_layout = QtWidgets.QHBoxLayout(ports_widget)
        ports_layout.setContentsMargins(0, 0, 0, 0)
        port1_check = QtWidgets.QCheckBox("Port 1")
        port2_check = QtWidgets.QCheckBox("Port 2")
        existing_ports = list(self._settings.get("librevna_excited_ports", [1, 2]))
        port1_check.setChecked(1 in existing_ports)
        port2_check.setChecked(2 in existing_ports)
        ports_layout.addWidget(port1_check)
        ports_layout.addWidget(port2_check)
        ports_layout.addStretch(1)

        vna_form.addRow("Calibration file", cal_row)
        vna_form.addRow("IFBW", ifbw_spin)
        vna_form.addRow("Power", power_spin)
        vna_form.addRow("Threshold", threshold_spin)
        vna_form.addRow("Timeout", timeout_spin)
        vna_form.addRow("Excited ports", ports_widget)
        tabs.addTab(vna_page, "LibreVNA")

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        root_layout.addWidget(buttons)

        def _browse_ipc():
            selected, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Select LibreVNA IPC binary",
                ipc_path_edit.text(),
                "Executable (*.exe);;All Files (*.*)",
            )
            if selected:
                ipc_path_edit.setText(selected)

        def _browse_worker():
            selected, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Select 32-bit Python",
                worker_python_edit.text(),
                "Python (python.exe);;All Files (*.*)",
            )
            if selected:
                worker_python_edit.setText(selected)

        def _browse_cal():
            selected, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Select LibreVNA calibration file",
                cal_path_edit.text(),
                "Calibration Files (*.cal *.json *.s1p *.s2p);;All Files (*.*)",
            )
            if selected:
                cal_path_edit.setText(selected)

        def _browse_template():
            selected, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Select LMX template file",
                template_path_edit.text(),
                "Text Files (*.txt);;All Files (*.*)",
            )
            if selected:
                template_path_edit.setText(selected)

        ipc_browse_btn.clicked.connect(_browse_ipc)
        worker_browse_btn.clicked.connect(_browse_worker)
        cal_browse_btn.clicked.connect(_browse_cal)
        template_browse_btn.clicked.connect(_browse_template)

        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return

        self._settings.update(
            {
                "librevna_ipc_name": ipc_name_edit.text().strip() or "librevna-ipc",
                "librevna_ipc_path": ipc_path_edit.text().strip(),
                "worker_python": worker_python_edit.text().strip() or sys.executable,
                "tinysa_debug": bool(tinysa_debug_check.isChecked()),
                "fosc": fosc_edit.text().strip() or "50MHz",
                "pfd": pfd_edit.text().strip() or "50MHz",
                "spi_clock": spi_edit.text().strip() or "400kHz",
                "template_path": template_path_edit.text().strip(),
                "outa_pwr": int(outa_pwr_spin.value()),
                "dwell_ms": float(dwell_spin.value()),
                "wait_lock": bool(wait_lock_check.isChecked()),
                "delta_update": bool(delta_update_check.isChecked()),
                "lock_timeout_s": float(lock_timeout_spin.value()),
                "auto_recal": bool(auto_recal_check.isChecked()),
                "librevna_cal_path": cal_path_edit.text().strip(),
                "librevna_ifbw_hz": float(ifbw_spin.value()),
                "librevna_power_dbm": float(power_spin.value()),
                "librevna_threshold_db": float(threshold_spin.value()),
                "librevna_timeout_ms": float(timeout_spin.value()),
                "librevna_excited_ports": [
                    port for port, enabled in ((1, port1_check.isChecked()), (2, port2_check.isChecked())) if enabled
                ] or [1],
            }
        )
        self._librevna_ipc_name = str(self._settings["librevna_ipc_name"])
        self._librevna_ipc_path_override = str(self._settings["librevna_ipc_path"] or "") or None

        if self._librevna_ipc_process and self._librevna_ipc_process.state() != QtCore.QProcess.NotRunning:
            self._librevna_ipc_process.terminate()
            self._librevna_ipc_process.waitForFinished(1000)
            self._librevna_ipc_process = None

        self.logs_tab.append_event("Preferences updated")
        self._on_device_refresh()
    
    def _on_documentation(self):
        """Handle Documentation action."""
        doc_path = os.path.join(os.path.dirname(parent_dir), "README.md")
        if os.path.exists(doc_path):
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(doc_path))
            self.statusBar().showMessage("Opened documentation", 3000)
        else:
            QtWidgets.QMessageBox.information(self, "Documentation", "README.md not found in repository root.")

    def _load_persistent_state(self):
        """Load persisted UI state from previous run."""
        store = self._persist_settings

        self.sidebar.freq_start_edit.setText(str(store.value("sidebar/freq_start", self.sidebar.freq_start_edit.text())))
        self.sidebar.freq_end_edit.setText(str(store.value("sidebar/freq_end", self.sidebar.freq_end_edit.text())))

        step_mode = str(store.value("sidebar/step_mode", "per_freq"))
        self.sidebar.step_per_freq_radio.setChecked(step_mode == "per_freq")
        self.sidebar.step_num_points_radio.setChecked(step_mode == "num_points")
        self.sidebar.step_value_edit.setText(str(store.value("sidebar/step_value", self.sidebar.step_value_edit.text())))

        try:
            self.sidebar.s11_threshold_spin.setValue(float(store.value("sidebar/s11_threshold", -10.0)))
            self.sidebar.s11_tolerance_spin.setValue(float(store.value("sidebar/s11_tolerance", 2.0)))
            self.sidebar.gain_tolerance_spin.setValue(float(store.value("sidebar/gain_tolerance", 1.5)))
            self.sidebar.noise_threshold_spin.setValue(float(store.value("sidebar/noise_threshold", -80.0)))
        except Exception:
            pass

        # Load sweep device checkboxes
        self.chk_usb2any.setChecked(store.value("toolbar/chk_usb2any", True) in (True, "true", "True", 1, "1"))
        self.chk_tinysa.setChecked(store.value("toolbar/chk_tinysa", True) in (True, "true", "True", 1, "1"))
        self.chk_librevna.setChecked(store.value("toolbar/chk_librevna", True) in (True, "true", "True", 1, "1"))

        # Restore antenna test tab splitter widths
        self.gain_measure_tab.restore_state(store)

        # Load golden samples from previous session
        golden_json = store.value("antenna_test/golden_samples", "")
        if isinstance(golden_json, str) and golden_json.strip():
            try:
                loaded_golden = json.loads(golden_json)
                if isinstance(loaded_golden, list) and loaded_golden:
                    self._pending_golden_samples = loaded_golden
            except Exception:
                pass

        # Load tested results from previous session
        tested_json = store.value("antenna_test/tested_results", "")
        if isinstance(tested_json, str) and tested_json.strip():
            try:
                loaded_tested = json.loads(tested_json)
                if isinstance(loaded_tested, list) and loaded_tested:
                    self._pending_tested_results = loaded_tested
            except Exception:
                pass

        self._pending_device_selection = {
            "tinysa": store.value("devices/tinysa", None),
            "usb2any": store.value("devices/usb2any", None),
            "librevna": store.value("devices/librevna", None),
        }

        prefs_json = store.value("prefs/json", "")
        if isinstance(prefs_json, str) and prefs_json.strip():
            try:
                loaded = json.loads(prefs_json)
                if isinstance(loaded, dict):
                    self._settings.update(loaded)
                    self._librevna_ipc_name = str(self._settings.get("librevna_ipc_name", "librevna-ipc"))
            except Exception:
                pass

        libre_json = store.value("librevna/state", "")
        if isinstance(libre_json, str) and libre_json.strip():
            try:
                loaded_state = json.loads(libre_json)
                if isinstance(loaded_state, dict):
                    self.librevna_tab.apply_state(loaded_state)
            except Exception:
                pass

    def _save_persistent_state(self):
        """Persist current UI state for next launch."""
        store = self._persist_settings
        store.setValue("sidebar/freq_start", self.sidebar.freq_start_edit.text().strip())
        store.setValue("sidebar/freq_end", self.sidebar.freq_end_edit.text().strip())
        store.setValue("sidebar/step_mode", "per_freq" if self.sidebar.step_per_freq_radio.isChecked() else "num_points")
        store.setValue("sidebar/step_value", self.sidebar.step_value_edit.text().strip())
        store.setValue("sidebar/s11_threshold", float(self.sidebar.s11_threshold_spin.value()))
        store.setValue("sidebar/s11_tolerance", float(self.sidebar.s11_tolerance_spin.value()))
        store.setValue("sidebar/gain_tolerance", float(self.sidebar.gain_tolerance_spin.value()))
        store.setValue("sidebar/noise_threshold", float(self.sidebar.noise_threshold_spin.value()))
        
        # Save sweep device checkboxes
        store.setValue("toolbar/chk_usb2any", self.chk_usb2any.isChecked())
        store.setValue("toolbar/chk_tinysa", self.chk_tinysa.isChecked())
        store.setValue("toolbar/chk_librevna", self.chk_librevna.isChecked())

        # Save antenna test tab splitter widths
        self.gain_measure_tab.save_state(store)

        # Auto-save golden samples for persistence across sessions
        golden_samples = self.gain_measure_tab.golden_table.get_records()
        try:
            store.setValue("antenna_test/golden_samples", json.dumps(golden_samples))
        except Exception:
            pass

        store.setValue("devices/tinysa", self.sidebar.tinysa_combo.currentData())
        store.setValue("devices/usb2any", self.sidebar.usb2any_combo.currentData())
        store.setValue("devices/librevna", self.sidebar.librevna_combo.currentData())

        try:
            store.setValue("prefs/json", json.dumps(self._settings))
            store.setValue("librevna/state", json.dumps(self.librevna_tab.export_state()))
        except Exception:
            pass

        store.sync()

    def _restore_device_selections(self):
        """Restore saved device selections after combos are repopulated."""
        if not self._pending_device_selection:
            return

        self._restore_combo_by_data(self.sidebar.tinysa_combo, self._pending_device_selection.get("tinysa"))
        self._restore_combo_by_data(self.sidebar.usb2any_combo, self._pending_device_selection.get("usb2any"))
        self._restore_combo_by_data(self.sidebar.librevna_combo, self._pending_device_selection.get("librevna"))

    def _check_load_previous_data(self):
        """Check if there's previous test data and prompt user to load it."""
        # Check for pending golden samples
        if self._pending_golden_samples:
            # Show prompt to load golden samples
            self._prompt_load_golden_samples()
            return

        # Check for pending tested results
        if self._pending_tested_results:
            self._prompt_load_tested_results()

    def _prompt_load_golden_samples(self):
        """Prompt user to load golden samples from previous session."""
        count = len(self._pending_golden_samples)
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Load Golden Samples")
        layout = QtWidgets.QVBoxLayout(dialog)

        label = QtWidgets.QLabel(f"Found {count} golden sample(s) from previous session.\n\nDo you want to load them?")
        layout.addWidget(label)

        btn_layout = QtWidgets.QHBoxLayout()
        yes_btn = QtWidgets.QPushButton("Yes, Load")
        no_btn = QtWidgets.QPushButton("No, Delete")
        btn_layout.addWidget(yes_btn)
        btn_layout.addWidget(no_btn)
        layout.addLayout(btn_layout)

        yes_btn.clicked.connect(dialog.accept)
        no_btn.clicked.connect(dialog.reject)

        result = dialog.exec()

        if result == QtWidgets.QDialog.Accepted:
            for record in self._pending_golden_samples:
                self.gain_measure_tab.golden_table.add_record(record)
            self.logs_tab.append_event(f"Loaded {count} golden sample(s) from previous session")
            self._pending_golden_samples = []
        elif result == QtWidgets.QDialog.Rejected:
            # Confirm deletion
            confirm = QtWidgets.QMessageBox.question(
                self, "Confirm Delete",
                "Are you sure you want to delete these golden samples?\n"
                "This action cannot be undone.",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )
            if confirm == QtWidgets.QMessageBox.Yes:
                self._pending_golden_samples = []
                self._persist_settings.remove("antenna_test/golden_samples")
            else:
                # User cancelled, re-prompt
                QtCore.QTimer.singleShot(100, self._prompt_load_golden_samples)
                return

        # Check for tested results after golden samples
        if self._pending_tested_results:
            self._prompt_load_tested_results()

    def _prompt_load_tested_results(self):
        """Prompt user to load tested results from previous session."""
        count = len(self._pending_tested_results)
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Load Previous Test Results")
        layout = QtWidgets.QVBoxLayout(dialog)

        label = QtWidgets.QLabel(f"Found {count} tested antenna result(s) from previous session.\n\nDo you want to load them?")
        layout.addWidget(label)

        btn_layout = QtWidgets.QHBoxLayout()
        yes_btn = QtWidgets.QPushButton("Yes, Load")
        no_btn = QtWidgets.QPushButton("No, Delete")
        btn_layout.addWidget(yes_btn)
        btn_layout.addWidget(no_btn)
        layout.addLayout(btn_layout)

        yes_btn.clicked.connect(dialog.accept)
        no_btn.clicked.connect(dialog.reject)

        result = dialog.exec()

        if result == QtWidgets.QDialog.Accepted:
            for record in self._pending_tested_results:
                self.gain_measure_tab.tested_table.add_record(record)
            self.logs_tab.append_event(f"Loaded {count} tested antenna result(s) from previous session")
            self._pending_tested_results = []
        elif result == QtWidgets.QDialog.Rejected:
            # Confirm deletion
            confirm = QtWidgets.QMessageBox.question(
                self, "Confirm Delete",
                "Are you sure you want to delete these test results?\n"
                "This action cannot be undone.",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )
            if confirm == QtWidgets.QMessageBox.Yes:
                self._pending_tested_results = []
                self._persist_settings.remove("antenna_test/tested_results")
            else:
                # User cancelled, re-prompt
                QtCore.QTimer.singleShot(100, self._prompt_load_tested_results)
                return

    @staticmethod
    def _restore_combo_by_data(combo: QtWidgets.QComboBox, value: object):
        if value in (None, "", "Auto"):
            combo.setCurrentIndex(0)
            return
        for idx in range(combo.count()):
            if str(combo.itemData(idx)) == str(value):
                combo.setCurrentIndex(idx)
                return

    def _collect_session_data(self) -> Dict[str, object]:
        """Collect current UI/session state for save."""
        return {
            "version": 1,
            "sidebar": {
                "tinysa": self.sidebar.tinysa_combo.currentData(),
                "usb2any": self.sidebar.usb2any_combo.currentData(),
                "librevna": self.sidebar.librevna_combo.currentData(),
                "freq_start": self.sidebar.freq_start_edit.text().strip(),
                "freq_end": self.sidebar.freq_end_edit.text().strip(),
                "step_mode": "per_freq" if self.sidebar.step_per_freq_radio.isChecked() else "num_points",
                "step_value": self.sidebar.step_value_edit.text().strip(),
                "s11_threshold": float(self.sidebar.s11_threshold_spin.value()),
                "s11_tolerance": float(self.sidebar.s11_tolerance_spin.value()),
                "gain_tolerance": float(self.sidebar.gain_tolerance_spin.value()),
                "noise_threshold": float(self.sidebar.noise_threshold_spin.value()),
            },
            "toolbar": {
                "chk_usb2any": self.chk_usb2any.isChecked(),
                "chk_tinysa": self.chk_tinysa.isChecked(),
                "chk_librevna": self.chk_librevna.isChecked(),
            },
            "antenna_test_tab": self.gain_measure_tab.save_state(store),
            "calibration_results": self._calibration_results,
            "preferences": self._settings,
        }

    def _apply_session_data(self, data: Dict[str, object]):
        """Apply a previously saved session state."""
        sidebar = data.get("sidebar", {}) if isinstance(data, dict) else {}
        if not isinstance(sidebar, dict):
            sidebar = {}

        self.sidebar.freq_start_edit.setText(str(sidebar.get("freq_start", "3.52GHz")))
        self.sidebar.freq_end_edit.setText(str(sidebar.get("freq_end", "3.6GHz")))
        self.sidebar.step_per_freq_radio.setChecked(sidebar.get("step_mode", "per_freq") == "per_freq")
        self.sidebar.step_num_points_radio.setChecked(sidebar.get("step_mode", "per_freq") != "per_freq")
        self.sidebar.step_value_edit.setText(str(sidebar.get("step_value", "10MHz")))

        try:
            self.sidebar.s11_threshold_spin.setValue(float(sidebar.get("s11_threshold", -10.0)))
            self.sidebar.s11_tolerance_spin.setValue(float(sidebar.get("s11_tolerance", 2.0)))
            self.sidebar.gain_tolerance_spin.setValue(float(sidebar.get("gain_tolerance", 1.5)))
            self.sidebar.noise_threshold_spin.setValue(float(sidebar.get("noise_threshold", -80.0)))
        except Exception:
            self.sidebar.s11_threshold_spin.setValue(-10.0)
            self.sidebar.s11_tolerance_spin.setValue(2.0)
            self.sidebar.gain_tolerance_spin.setValue(1.5)
            self.sidebar.noise_threshold_spin.setValue(-80.0)

        # Restore toolbar checkbox states
        toolbar = data.get("toolbar", {}) if isinstance(data, dict) else {}
        if isinstance(toolbar, dict):
            self.chk_usb2any.setChecked(toolbar.get("chk_usb2any", True))
            self.chk_tinysa.setChecked(toolbar.get("chk_tinysa", True))
            self.chk_librevna.setChecked(toolbar.get("chk_librevna", True))

        # Restore antenna test tab state
        antenna_test_tab = data.get("antenna_test_tab", {}) if isinstance(data, dict) else {}
        if isinstance(antenna_test_tab, dict):
            self.gain_measure_tab.restore_state(antenna_test_tab)

        loaded_results = data.get("calibration_results", {}) if isinstance(data, dict) else {}
        if isinstance(loaded_results, dict):
            self._calibration_results = loaded_results
        loaded_prefs = data.get("preferences", {}) if isinstance(data, dict) else {}
        if isinstance(loaded_prefs, dict):
            self._settings.update(loaded_prefs)
            self._librevna_ipc_name = str(self._settings.get("librevna_ipc_name", "librevna-ipc"))

    @staticmethod
    def _export_table_to_csv(table: QtWidgets.QTableWidget, path: str):
        """Export QTableWidget contents to CSV."""
        import csv

        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            headers = []
            for col in range(table.columnCount()):
                item = table.horizontalHeaderItem(col)
                headers.append(item.text() if item else f"Column {col + 1}")
            writer.writerow(headers)

            for row in range(table.rowCount()):
                values = []
                for col in range(table.columnCount()):
                    item = table.item(row, col)
                    values.append(item.text() if item else "")
                writer.writerow(values)
    
    def _on_about(self):
        """Handle About action."""
        QtWidgets.QMessageBox.about(
            self, "About",
            "LMX2594 Calibration GUI v2.0\n\nModern dark theme interface"
        )
    
    # Toolbar action handlers
    def _get_sweep_mode(self) -> str:
        """Derive sweep mode string from toolbar checkboxes."""
        tinysa = self.chk_tinysa.isChecked()
        librevna = self.chk_librevna.isChecked()
        if tinysa and librevna:
            return "Both"
        elif librevna:
            return "LibreVNA only"
        elif tinysa:
            return "tinySA only"
        else:
            return "None"

    def _on_run_sweep(self):
        """Handle Run Sweep action."""
        if self._sweep_running:
            return

        # Check if on Antenna Test tab - use special antenna test flow
        if self.tab_widget.currentIndex() == 2:
            self._on_run_antenna_test()
            return

        mode = self._get_sweep_mode()
        
        # Validate device selection
        if mode == "None":
            QtWidgets.QMessageBox.warning(
                self,
                "Run Sweep",
                "No spectrum analyzer or VNA is selected.\nCheck at least tinySA or LibreVNA to perform a sweep.",
            )
            return

        # Warn if USB2ANY/LMX is enabled but no SA/VNA is selected
        if self.chk_usb2any.isChecked() and not self.chk_tinysa.isChecked() and not self.chk_librevna.isChecked():
            QtWidgets.QMessageBox.warning(
                self,
                "Run Sweep",
                "USB2ANY/LMX is enabled but no SA/VNA is attached to measure.\nEnable tinySA or LibreVNA.",
            )
            return

        # Warn if tinySA is checked but USB2ANY/LMX is not
        if self.chk_tinysa.isChecked() and not self.chk_usb2any.isChecked():
            if self.chk_librevna.isChecked():
                # Both tinySA and LibreVNA are on, but USB2ANY/LMX is off
                reply = QtWidgets.QMessageBox.warning(
                    self,
                    "Run Sweep",
                    "tinySA sweep requires USB2ANY/LMX to generate the signal.\n\n"
                    "Do you want to disable tinySA and continue with LibreVNA only?",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.No,
                )
                if reply == QtWidgets.QMessageBox.Yes:
                    # Disable tinySA and continue
                    self.chk_tinysa.setChecked(False)
                else:
                    return
            else:
                # Only tinySA is on, USB2ANY/LMX is off - warn but allow to proceed
                reply = QtWidgets.QMessageBox.warning(
                    self,
                    "Run Sweep",
                    "WARNING: tinySA sweep requires USB2ANY/LMX to generate the signal,\n"
                    "but no USB2ANY/LMX is enabled.\n\n"
                    "The tinySA will measure ambient signals only (no LMX frequency control).\n\n"
                    "Do you want to continue anyway?",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.No,
                )
                if reply == QtWidgets.QMessageBox.No:
                    return

        if mode in ("Both", "LibreVNA only"):
            if not self._ensure_librevna_ipc():
                QtWidgets.QMessageBox.warning(
                    self,
                    "Run Sweep",
                    "LibreVNA IPC is not available. Configure it in Preferences.",
                )
                return

        if mode in ("Both", "tinySA only") and self.chk_usb2any.isChecked():
            worker_python = self._resolve_worker_python_path(prefer_x86=True)
            self._settings["worker_python"] = worker_python
            bits = self._python_bitness(worker_python)
            if bits != 32:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Run Sweep",
                    (
                        "USB2ANY requires a 32-bit worker Python.\n"
                        f"Current: {worker_python} ({bits or 'unknown'}-bit).\n"
                        "Set .venv-x86\\Scripts\\python.exe in Preferences."
                    ),
                )
                return

        if mode in ("Both", "tinySA only") and self.chk_usb2any.isChecked() and self.sidebar.usb2any_combo.count() <= 1:
            QtWidgets.QMessageBox.warning(
                self,
                "Run Sweep",
                "No USB2ANY device detected. Check Worker Python in Preferences (32-bit required) and click Refresh.",
            )
            return

        self._sweep_running = True
        self._polling_paused = True  # Pause polling to avoid USB2ANY conflicts
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.logs_tab.append_event(f"Run pressed: mode={mode}")
        if mode in ("Both", "LibreVNA only"):
            self.librevna_tab.set_status("Status: Running")

        worker = SweepWorker(
            mode=mode,
            settings=self._settings,
            tinysa_port=self.sidebar.tinysa_combo.currentData(),
            lmx_serial=self.sidebar.usb2any_combo.currentData(),
            librevna_serial=self.sidebar.librevna_combo.currentData(),
            start_text=self.sidebar.freq_start_edit.text().strip(),
            stop_text=self.sidebar.freq_end_edit.text().strip(),
            step_mode_per_freq=self.sidebar.step_per_freq_radio.isChecked(),
            step_value_text=self.sidebar.step_value_edit.text().strip(),
            ipc_name=self._librevna_ipc_name,
            parent=self,
            use_lmx=self.chk_usb2any.isChecked(),
        )
        worker.log.connect(self.logs_tab.append_event)
        worker.failed.connect(self._on_sweep_failed)
        worker.tiny_done.connect(self._on_tinysa_sweep_done)
        worker.libre_done.connect(self._on_librevna_sweep_done)
        worker.point_update.connect(self._on_lmx_point_update)
        worker.progress_update.connect(self._on_sweep_progress_update)
        worker.run_finished.connect(self._on_sweep_finished)
        self._sweep_worker = worker
        worker.start()
    
    def _on_stop_sweep(self):
        """Handle Stop Sweep action."""
        if self._sweep_worker and self._sweep_worker.isRunning():
            self._sweep_worker.request_stop()
            self.logs_tab.append_event("Stop requested")
            self.statusBar().showMessage("Stop requested", 3000)
        else:
            self.logs_tab.append_event("Stop pressed (no active sweep)")
    
    def _on_reset(self):
        """Handle Reset action."""
        self._sweep_running = False
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.plot_widget.clear()
        self.librevna_tab.clear_all()
        self.tinysa_tab.results_table.setRowCount(0)
        self.gain_measure_tab.clear_all()
        self._last_tinysa_plot_data = {}  # Clear tinySA plot data
        if hasattr(self, 'lmx_monitor_tab'):
            self.lmx_monitor_tab.clear_all()
        self.logs_tab.append_event("UI reset completed")
        self.statusBar().showMessage("Reset complete", 3000)

    def _on_measure_golden_sample(self):
        """Handle Measure Golden Sample button - measure golden sample signal strength."""
        if self._sweep_running:
            QtWidgets.QMessageBox.warning(
                self, "Measure Golden Sample", "A sweep is already running."
            )
            return

        # Check device availability
        librevna_enabled = self.chk_librevna.isChecked()
        tinysa_enabled = self.chk_tinysa.isChecked()

        # Handle single device selection with confirmation
        if librevna_enabled and not tinysa_enabled:
            result = QtWidgets.QMessageBox.question(
                self, "Confirm Measurement",
                "Only LibreVNA is enabled. The S11 test will be performed but no Gain test. Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )
            if result != QtWidgets.QMessageBox.Yes:
                return
        elif tinysa_enabled and not librevna_enabled:
            result = QtWidgets.QMessageBox.question(
                self, "Confirm Measurement",
                "Only tinySA is enabled. Only the Gain test will be performed (no S11 test). Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )
            if result != QtWidgets.QMessageBox.Yes:
                return
        elif not librevna_enabled and not tinysa_enabled:
            QtWidgets.QMessageBox.warning(
                self, "Measure Golden Sample",
                "At least one device (LibreVNA or tinySA) must be enabled."
            )
            return

        # Show dialog to get golden sample antenna info
        dialog = self._create_golden_sample_dialog()
        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return

        sample_label = dialog.findChild(QtWidgets.QLineEdit, "sample_label").text().strip()
        sample_gain = dialog.findChild(QtWidgets.QDoubleSpinBox, "sample_gain").value()

        if not sample_label:
            sample_label = f"Golden_{datetime.datetime.now().strftime('%H%M%S')}"

        self.logs_tab.append_event(f"Starting golden sample measurement: {sample_label}")
        self._run_golden_sample_measurement(sample_label, sample_gain)

    def _create_golden_sample_dialog(self) -> QtWidgets.QDialog:
        """Create dialog for golden sample antenna info."""
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Golden Sample Measurement")
        dialog.resize(350, 150)

        layout = QtWidgets.QFormLayout(dialog)

        label_edit = QtWidgets.QLineEdit()
        label_edit.setObjectName("sample_label")
        label_edit.setPlaceholderText("e.g., Reference Antenna 3.5GHz")

        gain_spin = QtWidgets.QDoubleSpinBox()
        gain_spin.setObjectName("sample_gain")
        gain_spin.setRange(-50.0, 50.0)
        gain_spin.setDecimals(2)
        gain_spin.setSuffix(" dBi")
        gain_spin.setValue(0.0)

        layout.addRow("Label:", label_edit)
        layout.addRow("Reference Gain:", gain_spin)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)

        return dialog

    def _select_golden_sample_for_test(self, records: List[Dict]) -> Optional[Dict]:
        """Show dialog to select a golden sample for antenna test comparison.

        Returns:
            Selected record dict, or None if user cancelled.
        """
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Select Golden Sample")
        dialog.resize(450, 120)

        layout = QtWidgets.QVBoxLayout(dialog)

        layout.addWidget(QtWidgets.QLabel("Select a golden sample for tolerance comparison:"))

        combo = QtWidgets.QComboBox()
        for r in records:
            label = r.get("label", "Unknown")
            ts = r.get("timestamp", "")
            display = f"{label} ({ts[:16]})" if ts else label
            combo.addItem(display, userData=r)
        layout.addWidget(combo)

        btn_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        btn_box.accepted.connect(dialog.accept)
        btn_box.rejected.connect(dialog.reject)
        layout.addWidget(btn_box)

        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return None
        return combo.currentData()

    def _run_golden_sample_measurement(self, label: str, sample_gain_dbi: float):
        """Run golden sample antenna measurement sweep."""
        import datetime

        self._sweep_running = True
        self._polling_paused = True
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.measure_golden_sample_action.setEnabled(False)

        # Store golden sample info for when sweep completes
        self._pending_golden_sample = {
            "label": label,
            "sample_gain_dbi": sample_gain_dbi,
        }

        # Determine which devices are enabled
        librevna_enabled = self.chk_librevna.isChecked()
        tinysa_enabled = self.chk_tinysa.isChecked()

        # Set mode based on device selection
        if librevna_enabled and tinysa_enabled:
            mode = "Both"
        elif librevna_enabled:
            mode = "LibreVNA only"
        else:
            mode = "tinySA only"

        worker = SweepWorker(
            mode=mode,
            settings=self._settings,
            tinysa_port=self.sidebar.tinysa_combo.currentData(),
            lmx_serial=self.sidebar.usb2any_combo.currentData(),
            librevna_serial=None,
            start_text=self.sidebar.freq_start_edit.text().strip(),
            stop_text=self.sidebar.freq_end_edit.text().strip(),
            step_mode_per_freq=self.sidebar.step_per_freq_radio.isChecked(),
            step_value_text=self.sidebar.step_value_edit.text().strip(),
            ipc_name=self._librevna_ipc_name,
            parent=self,
            use_lmx=True,
        )

        worker.log.connect(self.logs_tab.append_event)
        worker.failed.connect(self._on_golden_sample_sweep_failed)
        worker.tiny_done.connect(self._on_golden_sample_sweep_done)
        worker.libre_done.connect(self._on_librevna_golden_sample_sweep_done)
        worker.point_update.connect(self._on_lmx_point_update)
        worker.progress_update.connect(self._on_sweep_progress_update)
        worker.run_finished.connect(self._on_golden_sample_sweep_finished)

        self._sweep_worker = worker
        worker.start()

    def _on_golden_sample_sweep_done(self, points: list):
        """Handle golden sample measurement sweep completion."""
        import datetime

        if not hasattr(self, '_pending_golden_sample') or not self._pending_golden_sample:
            return

        # Build measurement record
        measurements = []
        for entry in points:
            if isinstance(entry, dict):
                measurements.append({
                    "frequency_hz": float(entry.get("frequency_hz", 0)),
                    "power_dbm": float(entry.get("power_dbm", 0)),
                })

        record = {
            "id": datetime.datetime.now().strftime("%Y%m%d%H%M%S"),
            "label": self._pending_golden_sample.get("label", "Golden Sample"),
            "sample_gain_dbi": self._pending_golden_sample.get("sample_gain_dbi", 0.0),
            "measurements": measurements,
            "timestamp": datetime.datetime.now().isoformat(),
        }

        # Add to Antenna Test tab
        self.gain_measure_tab.add_golden_sample(record)
        self.logs_tab.append_event(
            f"Golden sample measurement saved: {len(measurements)} points"
        )

        self._pending_golden_sample = None

    def _on_golden_sample_sweep_failed(self, message: str):
        """Handle golden sample sweep failure."""
        self._polling_paused = False
        self._pending_golden_sample = None
        self.logs_tab.append_event(f"Golden sample measurement failed: {message}")
        QtWidgets.QMessageBox.critical(self, "Golden Sample Measurement Error", message)

    def _on_golden_sample_sweep_finished(self):
        """Handle golden sample sweep finished."""
        self._sweep_running = False
        self._polling_paused = False
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.measure_golden_sample_action.setEnabled(True)
        self.statusBar().showMessage("Golden sample measurement finished", 3000)

    def _on_run_antenna_test(self):
        """Handle Run button on Antenna Test tab - complete antenna test sequence."""
        if self._sweep_running:
            return

        librevna_enabled = self.chk_librevna.isChecked()
        tinysa_enabled = self.chk_tinysa.isChecked()

        # Check device selection
        if not librevna_enabled and not tinysa_enabled:
            QtWidgets.QMessageBox.warning(
                self, "Antenna Test",
                "No device enabled. Enable LibreVNA and/or tinySA to run antenna test."
            )
            return

        # Handle single device selection with confirmation
        if librevna_enabled and not tinysa_enabled:
            result = QtWidgets.QMessageBox.question(
                self, "Confirm Antenna Test",
                "Only LibreVNA is enabled. Only S11 test will be performed (no Gain test). Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )
            if result != QtWidgets.QMessageBox.Yes:
                return
        elif tinysa_enabled and not librevna_enabled:
            result = QtWidgets.QMessageBox.question(
                self, "Confirm Antenna Test",
                "Only tinySA is enabled. Only Gain test will be performed (no S11 test). Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )
            if result != QtWidgets.QMessageBox.Yes:
                return

        # Get antenna label
        label = self.gain_measure_tab.get_next_antenna_label()
        if not label:
            label = f"Antenna_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # Get threshold values from sidebar
        s11_threshold = self.sidebar.s11_threshold_spin.value()
        s11_tolerance = self.sidebar.s11_tolerance_spin.value()
        gain_tolerance = self.sidebar.gain_tolerance_spin.value()
        noise_threshold = self.sidebar.noise_threshold_spin.value()

        self.logs_tab.append_event(f"=== Starting antenna test: {label} ===")
        self.logs_tab.append_event(f"Thresholds: S11<-{s11_threshold}dB, S11 tol:{s11_tolerance}dB, Gain tol:{gain_tolerance}dB, Noise<{noise_threshold}dBm")

        # Prompt user to select a golden sample for comparison
        selected_golden = None
        golden_records = self.gain_measure_tab.golden_table.get_records()
        if golden_records:
            selected_golden = self._select_golden_sample_for_test(golden_records)
            if selected_golden is None:
                return  # User cancelled
            golden_s11_data = selected_golden.get("s11_data")
            if golden_s11_data and isinstance(golden_s11_data, dict):
                golden_s11 = golden_s11_data.get("magnitudes", [])
                golden_freqs = golden_s11_data.get("frequencies", [])
                self.logs_tab.append_event(f"Using golden sample: {selected_golden.get('label', 'Unknown')}")
            else:
                golden_s11 = None
                golden_freqs = None
            golden_gain_data = selected_golden.get("measurements", [])
            if golden_gain_data and isinstance(golden_gain_data, list):
                golden_gain = golden_gain_data
            else:
                golden_gain = None
        else:
            golden_s11 = None
            golden_freqs = None
            golden_gain = None
            self.logs_tab.append_event("WARNING: No golden sample found - tolerance checks disabled")

        # Store pending antenna test info
        self._pending_antenna_test = {
            "label": label,
            "librevna_enabled": librevna_enabled,
            "tinysa_enabled": tinysa_enabled,
            "s11_threshold": s11_threshold,
            "s11_tolerance": s11_tolerance,
            "gain_tolerance": gain_tolerance,
            "noise_threshold": noise_threshold,
            "golden_s11": golden_s11,
            "golden_s11_freqs": golden_freqs if golden_s11 else None,
            "golden_gain": golden_gain,
            "s11_data": None,
            "gain_data": None,
            "dark_scan_data": None,
            "phase": 1,  # 1=VNA, 2=device swap, 3=dark scan, 4=gain
            "fail_reason": None,
        }

        # Phase 1: Run VNA S11 test if enabled
        if librevna_enabled:
            self._run_antenna_test_vna_phase()
        else:
            # Skip to gain phase
            self._run_antenna_test_gain_phase()

    def _run_antenna_test_vna_phase(self):
        """Run the VNA S11 test phase."""
        self.logs_tab.append_event("Phase 1: Running S11 (LibreVNA) test")

        self._sweep_running = True
        self._polling_paused = True
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        worker = SweepWorker(
            mode="LibreVNA only",
            settings=self._settings,
            tinysa_port=self.sidebar.tinysa_combo.currentData(),
            lmx_serial=self.sidebar.usb2any_combo.currentData(),
            librevna_serial=self.sidebar.librevna_combo.currentData(),
            start_text=self.sidebar.freq_start_edit.text().strip(),
            stop_text=self.sidebar.freq_end_edit.text().strip(),
            step_mode_per_freq=self.sidebar.step_per_freq_radio.isChecked(),
            step_value_text=self.sidebar.step_value_edit.text().strip(),
            ipc_name=self._librevna_ipc_name,
            parent=self,
            use_lmx=False,  # VNA doesn't need LMX
        )

        worker.log.connect(self.logs_tab.append_event)
        worker.failed.connect(self._on_antenna_test_vna_failed)
        worker.libre_done.connect(self._on_antenna_test_vna_done)
        worker.run_finished.connect(self._on_antenna_test_vna_finished)

        self._sweep_worker = worker
        worker.start()

    def _on_antenna_test_vna_failed(self, message: str):
        """Handle VNA phase failure."""
        self.logs_tab.append_event(f"VNA S11 test failed: {message}")
        self._finish_antenna_test_with_error(message)

    def _on_antenna_test_vna_done(self, payload: dict):
        """Handle VNA S11 test completion - check for fail-fast condition."""
        import math

        # Parse the trace array format returned by librevna-ipc
        # Format: {"trace": [{"frequency": hz, "S11": {"real": x, "imag": y}}, ...]}
        trace = payload.get("trace", [])
        frequencies = []
        magnitudes = []
        phases = []

        for entry in trace:
            if not isinstance(entry, dict):
                continue
            freq = entry.get("frequency")
            if not isinstance(freq, (int, float)):
                continue
            s11_data = entry.get("S11")
            if not isinstance(s11_data, dict):
                continue
            real = s11_data.get("real")
            imag = s11_data.get("imag")
            if not isinstance(real, (int, float)) or not isinstance(imag, (int, float)):
                continue
            # Calculate magnitude in dB
            mag_linear = math.sqrt(real * real + imag * imag)
            mag_db = -200.0 if mag_linear <= 0.0 else 20.0 * math.log10(mag_linear)
            # Calculate phase in degrees
            phase_deg = math.degrees(math.atan2(imag, real))
            frequencies.append(float(freq))
            magnitudes.append(float(mag_db))
            phases.append(float(phase_deg))

        if not frequencies or not magnitudes:
            self.logs_tab.append_event("ERROR: No S11 data received")
            self._pending_antenna_test["s11_data"] = {"fail": True, "reason": "No data"}
            self._pending_antenna_test["fail_reason"] = "No S11 data"
            self._check_antenna_test_continue()
            return

        # Find worst case S11 (highest magnitude = worst return loss)
        worst_s11 = max(magnitudes)
        worst_freq = frequencies[magnitudes.index(worst_s11)]

        # Calculate VSWR from return loss
        # VSWR = (1 + 10^(-RL/20)) / (1 - 10^(-RL/20))
        if worst_s11 < 0:
            rl = -worst_s11  # Return loss in dB
            vswr = (1 + 10**(-rl/20)) / (1 - 10**(-rl/20))
        else:
            vswr = float('inf')

        s11_threshold = self._pending_antenna_test.get("s11_threshold", -10.0)

        self._pending_antenna_test["s11_data"] = {
            "worst_s11_db": worst_s11,
            "worst_freq_hz": worst_freq,
            "vswr": vswr,
            "frequencies": frequencies,
            "magnitudes": magnitudes,
            "phases": phases,
        }

        self.logs_tab.append_event(f"S11 results: {worst_s11:.2f} dB @ {worst_freq/1e6:.2f} MHz, VSWR: {vswr:.2f}")

        # Fail-fast check: S11 >= S11 Threshold (e.g., -10dB)
        if worst_s11 >= s11_threshold:
            self.logs_tab.append_event(f"FAIL: S11 ({worst_s11:.2f} dB) >= {s11_threshold} dB - Poor Matching")
            self._pending_antenna_test["s11_pass"] = False
            self._pending_antenna_test["fail_reason"] = "S11 above threshold"
            # Don't continue to gain test - _finish_antenna_test will add the result
            self._finish_antenna_test()
            return

        self.logs_tab.append_event(f"PASS: S11 ({worst_s11:.2f} dB) < {s11_threshold} dB")

        # Check S11 tolerance vs golden sample
        s11_tolerance = self._pending_antenna_test.get("s11_tolerance", 2.0)
        golden_s11 = self._pending_antenna_test.get("golden_s11")
        golden_freqs = self._pending_antenna_test.get("golden_s11_freqs")

        if golden_s11 and golden_freqs:
            # Compare each frequency point
            tolerance_fail = False
            max_diff = 0.0

            for i, (freq, mag) in enumerate(zip(frequencies, magnitudes)):
                # Find matching golden sample frequency
                for j, golden_freq in enumerate(golden_freqs):
                    if abs(freq - golden_freq) < 1e6:  # Within 1 MHz
                        golden_mag = golden_s11[j]
                        diff = abs(mag - golden_mag)
                        max_diff = max(max_diff, diff)
                        if diff > s11_tolerance:
                            tolerance_fail = True
                            self.logs_tab.append_event(
                                f"FAIL: S11 variation at {freq/1e6:.2f} MHz: {mag:.2f} vs golden {golden_mag:.2f} dB (diff {diff:.2f} > {s11_tolerance} dB)"
                            )
                        break

            if tolerance_fail:
                self.logs_tab.append_event(f"FAIL: S11 tolerance exceeded (max diff: {max_diff:.2f} dB)")
                self._pending_antenna_test["s11_pass"] = False
                self._pending_antenna_test["fail_reason"] = "S11 variation detected"
                # _finish_antenna_test will add the result
                self._finish_antenna_test()
                return
            else:
                self.logs_tab.append_event(f"PASS: S11 within tolerance (max diff: {max_diff:.2f} dB)")
        else:
            self.logs_tab.append_event("INFO: No golden sample - skipping S11 tolerance check")

        self._pending_antenna_test["s11_pass"] = True
        self._check_antenna_test_continue()

    def _on_antenna_test_vna_finished(self):
        """Handle VNA phase finished."""
        # This is called after VNA is done, check if we should continue
        pass  # Continuation handled in _on_antenna_test_vna_done

    def _check_antenna_test_continue(self):
        """Check if we should continue to next phase of antenna test."""
        pending = self._pending_antenna_test
        tinysa_enabled = pending.get("tinysa_enabled", False)

        if not tinysa_enabled:
            # No more phases
            self._finish_antenna_test()
            return

        # Phase 2: Device swap prompt
        if pending.get("s11_pass", True):
            # Show device swap prompt
            self.logs_tab.append_event("Requesting device swap: disconnect VNA, connect to tinySA fixture")
            result = QtWidgets.QMessageBox.question(
                self, "Swap Device",
                "S11 test passed.\n\n"
                "Please disconnect the antenna from the VNA and connect it to the tinySA fixture.\n\n"
                "Press Yes to continue with Gain test.",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.Yes
            )
            if result != QtWidgets.QMessageBox.Yes:
                self.logs_tab.append_event("Antenna test cancelled by user")
                self._finish_antenna_test()
                return

            # Phase 3: Dark scan
            self._run_antenna_test_dark_scan()
        else:
            # S11 failed, skip gain test
            self._finish_antenna_test()

    def _run_antenna_test_dark_scan(self):
        """Run dark scan (noise floor measurement) before gain test."""
        self.logs_tab.append_event("Phase 2: Running Dark Scan (noise floor)")

        self._pending_antenna_test["phase"] = 3

        # Run a tinySA sweep with LMX OFF (no transmission)
        # We use a shorter sweep for dark scan
        worker = SweepWorker(
            mode="tinySA only",
            settings=self._settings,
            tinysa_port=self.sidebar.tinysa_combo.currentData(),
            lmx_serial=self.sidebar.usb2any_combo.currentData(),
            librevna_serial=None,
            start_text=self.sidebar.freq_start_edit.text().strip(),
            stop_text=self.sidebar.freq_end_edit.text().strip(),
            step_mode_per_freq=self.sidebar.step_per_freq_radio.isChecked(),
            step_value_text=self.sidebar.step_value_edit.text().strip(),
            ipc_name=self._librevna_ipc_name,
            parent=self,
            use_lmx=False,  # Dark scan: LMX OFF
        )

        worker.log.connect(self.logs_tab.append_event)
        worker.failed.connect(self._on_antenna_test_dark_scan_failed)
        worker.tiny_done.connect(self._on_antenna_test_dark_scan_done)
        worker.run_finished.connect(self._on_antenna_test_dark_scan_finished)

        self._sweep_worker = worker
        worker.start()

    def _on_antenna_test_dark_scan_failed(self, message: str):
        """Handle dark scan failure."""
        self.logs_tab.append_event(f"Dark scan failed: {message}")
        # Continue anyway - not critical
        self._run_antenna_test_gain_phase()

    def _on_antenna_test_dark_scan_done(self, points: list):
        """Handle dark scan completion - check noise floor threshold."""
        if not points:
            self.logs_tab.append_event("ERROR: No dark scan data received")
            self._pending_antenna_test["dark_scan_data"] = None
            # Continue anyway
            self._run_antenna_test_gain_phase()
            return

        # Find max noise floor
        powers = [float(p.get("power_dbm", -100)) for p in points if isinstance(p, dict)]
        if not powers:
            self._pending_antenna_test["dark_scan_data"] = None
            self._run_antenna_test_gain_phase()
            return

        max_noise = max(powers)
        noise_threshold = self._pending_antenna_test.get("noise_threshold", -80.0)

        self._pending_antenna_test["dark_scan_data"] = {
            "max_noise_floor_dbm": max_noise,
            "points": points,
        }

        self.logs_tab.append_event(f"Dark scan complete: max noise floor = {max_noise:.2f} dBm (threshold: {noise_threshold} dBm)")

        # Check noise floor threshold
        if max_noise > noise_threshold:
            self.logs_tab.append_event(f"WARNING: High Interference - Noise ({max_noise:.2f} dBm) > {noise_threshold} dBm")
            # Show warning popup with Retry/Abort options
            result = QtWidgets.QMessageBox.warning(
                self, "High Interference Detected",
                f"Noise floor ({max_noise:.2f} dBm) exceeds threshold ({noise_threshold} dBm).\n\n"
                "This may affect measurement accuracy.\n\n"
                "Do you want to retry the dark scan or abort?",
                QtWidgets.QMessageBox.Retry | QtWidgets.QMessageBox.Abort,
                QtWidgets.QMessageBox.Retry
            )
            if result == QtWidgets.QMessageBox.Retry:
                self.logs_tab.append_event("Retrying dark scan...")
                self._run_antenna_test_dark_scan()
                return
            else:
                self.logs_tab.append_event("Continuing despite high noise floor (user accepted)")
                # Continue with measurement but warn

        self.logs_tab.append_event("PASS: Noise floor within acceptable range")
        self._run_antenna_test_gain_phase()

    def _on_antenna_test_dark_scan_finished(self):
        """Handle dark scan finished - proceed to gain phase."""
        self._run_antenna_test_gain_phase()

    def _run_antenna_test_gain_phase(self):
        """Run the Gain test phase using Friis equation."""
        self.logs_tab.append_event("Phase 3: Running Gain (tinySA) test")

        self._pending_antenna_test["phase"] = 4

        # Run tinySA sweep with LMX transmitting
        worker = SweepWorker(
            mode="tinySA only",
            settings=self._settings,
            tinysa_port=self.sidebar.tinysa_combo.currentData(),
            lmx_serial=self.sidebar.usb2any_combo.currentData(),
            librevna_serial=None,
            start_text=self.sidebar.freq_start_edit.text().strip(),
            stop_text=self.sidebar.freq_end_edit.text().strip(),
            step_mode_per_freq=self.sidebar.step_per_freq_radio.isChecked(),
            step_value_text=self.sidebar.step_value_edit.text().strip(),
            ipc_name=self._librevna_ipc_name,
            parent=self,
            use_lmx=True,  # Enable LMX transmission
        )

        worker.log.connect(self.logs_tab.append_event)
        worker.failed.connect(self._on_antenna_test_gain_failed)
        worker.tiny_done.connect(self._on_antenna_test_gain_done)
        worker.run_finished.connect(self._on_antenna_test_gain_finished)

        self._sweep_worker = worker
        worker.start()

    def _on_antenna_test_gain_failed(self, message: str):
        """Handle gain test failure."""
        self.logs_tab.append_event(f"Gain test failed: {message}")
        self._finish_antenna_test_with_error(message)

    def _on_antenna_test_gain_done(self, points: list):
        """Handle gain test completion - calculate gain and check tolerance."""
        if not points:
            self.logs_tab.append_event("ERROR: No gain measurement data received")
            self._pending_antenna_test["gain_data"] = None
            self._pending_antenna_test["fail_reason"] = "No gain data"
            self._check_antenna_test_complete()
            return

        # Calculate received power and estimate gain using Friis equation
        # For now, we'll use the max received power as a relative gain measurement
        powers = [float(p.get("power_dbm", -100)) for p in points if isinstance(p, dict)]
        if not powers:
            self._pending_antenna_test["gain_data"] = None
            self._check_antenna_test_complete()
            return

        max_power = max(powers)
        avg_power = sum(powers) / len(powers)

        # Store gain data
        self._pending_antenna_test["gain_data"] = {
            "points": points,
            "max_power_dbm": max_power,
            "avg_power_dbm": avg_power,
        }

        self.logs_tab.append_event(f"Gain measurement: max={max_power:.2f} dBm, avg={avg_power:.2f} dBm")

        # Update tinySA tab with the gain measurement data
        self.tinysa_tab.results_table.setRowCount(0)
        x_data = []
        y_data = []
        for entry in points:
            if not isinstance(entry, dict):
                continue
            row = self.tinysa_tab.results_table.rowCount()
            self.tinysa_tab.results_table.insertRow(row)
            freq = float(entry.get("frequency_hz", 0.0))
            power = float(entry.get("power_dbm", 0.0))
            timestamp = str(entry.get("timestamp", ""))
            self.tinysa_tab.results_table.setItem(row, 0, QtWidgets.QTableWidgetItem(f"{freq:.3f}"))
            self.tinysa_tab.results_table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{power:.2f}"))
            self.tinysa_tab.results_table.setItem(row, 2, QtWidgets.QTableWidgetItem(timestamp))
            x_data.append(freq / 1e9)
            y_data.append(power)

        # Store the plot data for later redrawing
        self._last_tinysa_plot_data = {"x": x_data, "y": y_data}

        if x_data and y_data:
            self.plot_widget.set_mode("spectrum")
            self.plot_widget.clear()
            self.plot_widget.plot_data(x_data, y_data, name="tinySA")

        # Check gain tolerance vs golden sample
        gain_tolerance = self._pending_antenna_test.get("gain_tolerance", 1.5)
        golden_gain = self._pending_antenna_test.get("golden_gain")

        gain_pass = True
        if golden_gain:
            # Get golden sample max power
            golden_powers = [float(p.get("power_dbm", -100)) for p in golden_gain if isinstance(p, dict)]
            if golden_powers:
                golden_max = max(golden_powers)
                diff = abs(max_power - golden_max)
                self.logs_tab.append_event(f"Golden sample max: {golden_max:.2f} dBm, difference: {diff:.2f} dB")

                if diff > gain_tolerance:
                    if max_power < golden_max:
                        self.logs_tab.append_event(f"FAIL: Gain ({max_power:.2f} dBm) lower than golden ({golden_max:.2f} dBm) by {diff:.2f} dB - Poor Radiation")
                    else:
                        self.logs_tab.append_event(f"FAIL: Gain ({max_power:.2f} dBm) higher than golden ({golden_max:.2f} dBm) by {diff:.2f} dB - Variation Detected")
                    gain_pass = False
                else:
                    self.logs_tab.append_event(f"PASS: Gain within tolerance (diff: {diff:.2f} dB)")
            else:
                self.logs_tab.append_event("INFO: Golden sample has no valid gain data - skipping tolerance check")
        else:
            self.logs_tab.append_event("INFO: No golden sample - skipping gain tolerance check")

        self._pending_antenna_test["gain_pass"] = gain_pass
        if not gain_pass:
            self._pending_antenna_test["fail_reason"] = "Gain outside tolerance"

        self._check_antenna_test_complete()

    def _on_antenna_test_gain_finished(self):
        """Handle gain test finished."""
        pass  # Handled in _on_antenna_test_gain_done

    def _check_antenna_test_complete(self):
        """Check if antenna test is complete."""
        # If we got here with all phases done, finish up
        self._finish_antenna_test()

    def _add_antenna_test_result(self, s11_fail: bool = False, gain_fail: bool = None):
        """Add the antenna test result to the tested antennas table."""
        pending = self._pending_antenna_test
        label = pending.get("label", "Unknown")

        # Determine overall pass/fail
        s11_pass = pending.get("s11_pass", True)
        gain_pass = pending.get("gain_pass", True)
        fail_reason = pending.get("fail_reason")

        if s11_pass and gain_pass:
            overall_pass = True
        else:
            overall_pass = False

        # Build record
        record = {
            "id": datetime.datetime.now().strftime("%Y%m%d%H%M%S"),
            "label": label,
            "timestamp": datetime.datetime.now().isoformat(),
            "s11_pass": s11_pass,
            "gain_pass": gain_pass,
            "overall_pass": overall_pass,
            "fail_reason": fail_reason,
            "s11_data": pending.get("s11_data"),
            "gain_data": pending.get("gain_data"),
            "dark_scan_data": pending.get("dark_scan_data"),
            "thresholds": {
                "s11_threshold": pending.get("s11_threshold"),
                "s11_tolerance": pending.get("s11_tolerance"),
                "gain_tolerance": pending.get("gain_tolerance"),
                "noise_threshold": pending.get("noise_threshold"),
            },
        }

        # Add to tested antennas table
        self.gain_measure_tab.add_tested_measurement(record)

    def _finish_antenna_test(self):
        """Finish the antenna test and reset UI."""
        self._sweep_running = False
        self._polling_paused = False
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        # Log final result
        pending = self._pending_antenna_test
        if pending:
            s11_pass = pending.get("s11_pass", True)
            gain_pass = pending.get("gain_pass", True)
            fail_reason = pending.get("fail_reason")

            if s11_pass and gain_pass:
                self.logs_tab.append_event("=== RESULT: PASS ===")
            else:
                self.logs_tab.append_event(f"=== RESULT: FAIL - {fail_reason} ===")

            # Add result to tested antennas table
            self._add_antenna_test_result()

        self.statusBar().showMessage("Antenna test completed", 3000)

        self._pending_antenna_test = None

    def _finish_antenna_test_with_error(self, message: str):
        """Finish antenna test with error."""
        self._sweep_running = False
        self._polling_paused = False
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        self.logs_tab.append_event(f"Antenna test failed: {message}")
        QtWidgets.QMessageBox.critical(self, "Antenna Test Error", message)

        self._pending_antenna_test = None

    def _on_sweep_failed(self, message: str):
        self._polling_paused = False  # Resume polling on failure
        self.logs_tab.append_event(f"Sweep failed: {message}")
        self.librevna_tab.set_status("Status: Failed")
        QtWidgets.QMessageBox.critical(self, "Sweep Error", message)

    def _on_sweep_finished(self):
        self._sweep_running = False
        self._polling_paused = False  # Resume polling
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.statusBar().showMessage("Sweep finished", 3000)
        # Reset sweep progress in LMX Monitor tab
        if hasattr(self, 'lmx_monitor_tab'):
            self.lmx_monitor_tab.update_sweep_progress(0, 0)

    def _on_tinysa_sweep_done(self, points: list):
        self.tinysa_tab.results_table.setRowCount(0)
        x_data = []
        y_data = []
        for entry in points:
            if not isinstance(entry, dict):
                continue
            row = self.tinysa_tab.results_table.rowCount()
            self.tinysa_tab.results_table.insertRow(row)
            freq = float(entry.get("frequency_hz", 0.0))
            power = float(entry.get("power_dbm", 0.0))
            timestamp = str(entry.get("timestamp", ""))
            self.tinysa_tab.results_table.setItem(row, 0, QtWidgets.QTableWidgetItem(f"{freq:.3f}"))
            self.tinysa_tab.results_table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{power:.2f}"))
            self.tinysa_tab.results_table.setItem(row, 2, QtWidgets.QTableWidgetItem(timestamp))
            x_data.append(freq / 1e9)
            y_data.append(power)

        # Store the plot data for later redrawing
        self._last_tinysa_plot_data = {"x": x_data, "y": y_data}

        if x_data and y_data:
            self.plot_widget.set_mode("spectrum")
            self.plot_widget.clear()
            self.plot_widget.plot_data(x_data, y_data, name="tinySA")

    def _on_librevna_sweep_done(self, payload: dict):
        self.librevna_tab.set_status("Status: Complete")
        self.plot_widget.set_mode("s_parameter")
        self.librevna_tab.set_sweep_result(payload if isinstance(payload, dict) else {})

    def _on_librevna_golden_sample_sweep_done(self, payload: dict):
        """Handle LibreVNA sweep completion for golden sample measurement."""
        if not hasattr(self, '_pending_golden_sample') or not self._pending_golden_sample:
            return

        import datetime
        import math
        
        # Parse the trace array format returned by librevna-ipc
        # Format: {"trace": [{"frequency": hz, "S11": {"real": x, "imag": y}}, ...]}
        trace = payload.get("trace", []) if isinstance(payload, dict) else []
        frequencies = []
        magnitudes = []
        phases = []

        for entry in trace:
            if not isinstance(entry, dict):
                continue
            freq = entry.get("frequency")
            if not isinstance(freq, (int, float)):
                continue
            s11_data = entry.get("S11")
            if not isinstance(s11_data, dict):
                continue
            real = s11_data.get("real")
            imag = s11_data.get("imag")
            if not isinstance(real, (int, float)) or not isinstance(imag, (int, float)):
                continue
            # Calculate magnitude in dB
            mag_linear = math.sqrt(real * real + imag * imag)
            mag_db = -200.0 if mag_linear <= 0.0 else 20.0 * math.log10(mag_linear)
            # Calculate phase in degrees
            phase_deg = math.degrees(math.atan2(imag, real))
            frequencies.append(float(freq))
            magnitudes.append(float(mag_db))
            phases.append(float(phase_deg))

        s11_data = {}
        if frequencies and magnitudes:
            s11_data = {
                "frequencies": frequencies,
                "magnitudes": magnitudes,
                "phases": phases,
            }

        # For LibreVNA-only measurements, don't populate measurements array
        # (measurements should only contain actual gain data from tinySA)
        record = {
            "id": datetime.datetime.now().strftime("%Y%m%d%H%M%S"),
            "label": self._pending_golden_sample.get("label", "Golden Sample"),
            "sample_gain_dbi": self._pending_golden_sample.get("sample_gain_dbi", 0.0),
            "measurements": [],  # Empty - this is S11-only data, not gain
            "s11_data": s11_data,
            "timestamp": datetime.datetime.now().isoformat(),
        }

        self.gain_measure_tab.add_golden_sample(record)
        self.logs_tab.append_event(
            f"Golden sample (LibreVNA) measurement saved: {len(measurements)} points"
        )

        self._pending_golden_sample = None

    def _on_lmx_point_update(self, data: dict):
        """Handle LMX frequency point update from sweep worker."""
        if hasattr(self, 'lmx_monitor_tab'):
            self.lmx_monitor_tab.update_frequency_info(data)

    def _on_sweep_progress_update(self, current: int, total: int):
        """Handle sweep progress update."""
        if hasattr(self, 'lmx_monitor_tab'):
            self.lmx_monitor_tab.update_sweep_progress(current, total)

    def _on_librevna_plot_changed(self, payload: dict):
        self._last_librevna_plot_payload = payload if isinstance(payload, dict) else {}
        if self.tab_widget.currentIndex() == 0:
            self.plot_widget.plot_sparameters(self._last_librevna_plot_payload)

    def _on_gain_plot_requested(self, plot_data: dict):
        """Handle gain plot request from GainMeasureTab."""
        if self.tab_widget.currentIndex() != 2:  # Not on Gain Measure tab
            return

        # Check if this is a comparison plot request
        mode = plot_data.get("mode", "")
        if mode == "comparison":
            golden_record = plot_data.get("golden")
            tested_record = plot_data.get("tested")
            meas_type = plot_data.get("meas_type", "both")
            self.plot_widget.plot_comparison(
                golden_record, tested_record,
                is_single_record=False,
                meas_type=meas_type
            )

            golden_label = plot_data.get("golden_label", "None")
            tested_label = plot_data.get("tested_label", "None")
            self.logs_tab.append_event(
                f"Plotted comparison: Golden={golden_label}, Tested={tested_label}"
            )
            return

        # Single record plot
        record = plot_data.get("record", {})
        source = plot_data.get("source", "")
        label = plot_data.get("label", "Unknown")
        meas_type = plot_data.get("meas_type", "none")

        if not record:
            return

        if meas_type == "none":
            self.logs_tab.append_event(
                f"No plot data available for: {label}"
            )
        else:
            self.plot_widget.plot_comparison(
                record, None,
                is_single_record=True,
                meas_type=meas_type
            )
            type_name = {
                "gain_only": "Gain",
                "sparam_only": "S-parameters",
                "both": "Gain + S-parameters",
            }.get(meas_type, meas_type)
            self.logs_tab.append_event(
                f"Plotted {type_name} ({source}): {label}"
            )
    
    def _on_device_refresh(self):
        """Handle device refresh request."""
        self.logs_tab.append_event("Refreshing device list")

        self.sidebar.tinysa_combo.clear()
        self.sidebar.usb2any_combo.clear()
        self.sidebar.librevna_combo.clear()
        self.sidebar.tinysa_combo.addItem("Auto", None)
        self.sidebar.usb2any_combo.addItem("Auto", None)
        self.sidebar.librevna_combo.addItem("Auto", None)

        self._refresh_tinysa_devices()
        self._refresh_usb2any_devices()
        self._refresh_librevna_devices()

        self._restore_device_selections()

        usb2any_connected = self.sidebar.usb2any_combo.count() > 1
        tinysa_connected = self.sidebar.tinysa_combo.count() > 1
        librevna_connected = self.sidebar.librevna_combo.count() > 1

        self._set_device_status_indicator(
            self.usb2any_status_label,
            "USB2ANY",
            usb2any_connected,
        )
        self._set_device_status_indicator(
            self.tinysa_status_label,
            "tinySA",
            tinysa_connected,
        )
        self._set_device_status_indicator(
            self.librevna_status_label,
            "LibreVNA",
            librevna_connected,
        )

        # Update LMX Monitor tab device status
        if hasattr(self, 'lmx_monitor_tab'):
            serial = self.sidebar.usb2any_combo.currentData() if usb2any_connected else None
            self.lmx_monitor_tab.update_usb2any_status(usb2any_connected, serial)

        # Update checkbox enabled state based on device connection
        self._update_device_checkboxes(usb2any_connected, tinysa_connected, librevna_connected)

    def _update_device_checkboxes(self, usb2any_connected: bool, tinysa_connected: bool, librevna_connected: bool):
        """Update checkbox enabled state based on device connection status."""
        # USB2ANY is needed for LMX, but can work without it for some tests
        # We'll enable it but warn if needed
        self.chk_usb2any.setEnabled(True)

        # tinySA and LibreVNA - disable if not connected
        self.chk_tinysa.setEnabled(tinysa_connected)
        self.chk_librevna.setEnabled(librevna_connected)

        # If a device was enabled but is now disconnected, uncheck it
        if not tinysa_connected and self.chk_tinysa.isChecked():
            self.chk_tinysa.setChecked(False)
        if not librevna_connected and self.chk_librevna.isChecked():
            self.chk_librevna.setChecked(False)

        # Update tooltip to explain why checkbox is disabled
        self.chk_tinysa.setToolTip("tinySA connected" if tinysa_connected else "tinySA not connected - cannot select")
        self.chk_librevna.setToolTip("LibreVNA connected" if librevna_connected else "LibreVNA not connected - cannot select")

    def _set_device_status_indicator(self, label: QtWidgets.QLabel, name: str, connected: bool):
        """Set toolbar device indicator chip text and color."""
        dot = "●"
        color = COLORS["success"] if connected else COLORS["text_dim"]
        label.setText(f"{dot} {name}")
        label.setStyleSheet(
            f"color: {color}; padding: 2px 8px; border: 1px solid {COLORS['border']}; border-radius: 8px;"
        )

    def _poll_device_status(self):
        """Periodically poll device status (called by timer)."""
        # Skip polling during active sweep or when polling is paused
        if self._sweep_running or getattr(self, '_polling_paused', False):
            return

        # Poll USB2ANY status via quick worker subprocess
        self._poll_usb2any_status()

    def _poll_usb2any_status(self):
        """Quick poll of USB2ANY device availability."""
        worker_script = os.path.join(parent_dir, "lmx2594_worker.py")
        if not os.path.exists(worker_script):
            return

        try:
            worker_python = str(self._settings.get("worker_python", sys.executable))
            if not worker_python or not os.path.exists(worker_python):
                return

            result = subprocess.run(
                [worker_python, worker_script],
                input=json.dumps({"cmd": "list_usb2any"}) + "\n",
                text=True,
                capture_output=True,
                timeout=2,
                cwd=parent_dir,
            )
            if result.returncode != 0:
                self._update_usb2any_poll_result(False, None)
                return

            response = json.loads((result.stdout or "").strip() or "{}")
            payload = response.get("result", {}) if response.get("ok") else {}
            count = int(payload.get("count", 0)) if isinstance(payload, dict) else 0
            serials = payload.get("serials", []) if isinstance(payload, dict) else []

            connected = count > 0
            serial = serials[0] if serials else None
            self._update_usb2any_poll_result(connected, serial)

        except Exception:
            self._update_usb2any_poll_result(False, None)

    def _update_usb2any_poll_result(self, connected: bool, serial: Optional[str]):
        """Update UI with USB2ANY poll result."""
        self._set_device_status_indicator(self.usb2any_status_label, "USB2ANY", connected)
        if hasattr(self, 'lmx_monitor_tab'):
            self.lmx_monitor_tab.update_usb2any_status(connected, serial)

    def _candidate_worker_python_paths(self):
        """Return likely x86 worker python locations."""
        repo_root = os.path.dirname(parent_dir)
        return [
            os.path.join(repo_root, ".venv-x86", "Scripts", "python.exe"),
            os.path.join(repo_root, ".venv_x86", "Scripts", "python.exe"),
            os.path.join(repo_root, "venv-x86", "Scripts", "python.exe"),
            os.path.join(repo_root, "venv_x86", "Scripts", "python.exe"),
        ]

    @staticmethod
    def _python_bitness(python_path: str) -> Optional[int]:
        """Return interpreter bitness (32/64), or None on failure."""
        if not python_path or not os.path.exists(python_path):
            return None
        try:
            out = subprocess.check_output(
                [python_path, "-c", "import struct; print(struct.calcsize('P')*8)"],
                text=True,
                timeout=5,
            ).strip()
            return int(out)
        except Exception:
            return None

    def _resolve_worker_python_path(self, prefer_x86: bool = True) -> str:
        """Resolve worker python path, preferring x86 virtual env."""
        configured = str(self._settings.get("worker_python", "")).strip()
        if configured and os.path.exists(configured):
            bits = self._python_bitness(configured)
            if not prefer_x86 or bits == 32:
                return configured

        for candidate in self._candidate_worker_python_paths():
            if os.path.exists(candidate):
                bits = self._python_bitness(candidate)
                if not prefer_x86 or bits == 32:
                    return candidate

        return configured if configured else sys.executable

    def _refresh_tinysa_devices(self):
        """Detect and populate tinySA serial ports."""
        try:
            from tinysa_antenna_test import list_serial_ports

            ports = list_serial_ports()
            for port, desc, _hwid in ports:
                self.sidebar.tinysa_combo.addItem(f"{port} ({desc})", port)

            if ports:
                detected = ", ".join(port for port, _desc, _hwid in ports)
                self.logs_tab.append_event(f"tinySA ports detected: {detected}")
            else:
                self.logs_tab.append_event("No tinySA serial ports detected")
        except Exception as exc:
            self.logs_tab.append_event(f"tinySA scan failed: {exc}")

    def _refresh_usb2any_devices(self):
        """Detect and populate USB2ANY devices."""
        worker_script = os.path.join(parent_dir, "lmx2594_worker.py")
        if not os.path.exists(worker_script):
            self.logs_tab.append_event("USB2ANY scan unavailable: lmx2594_worker.py missing")
            return

        try:
            worker_python = self._resolve_worker_python_path(prefer_x86=True)
            self._settings["worker_python"] = worker_python
            bits = self._python_bitness(worker_python)
            if bits != 32:
                self.logs_tab.append_event(
                    f"USB2ANY scan failed: worker python is not x86 (path={worker_python}, bits={bits})"
                )
                return
            result = subprocess.run(
                [worker_python, worker_script],
                input=json.dumps({"cmd": "list_usb2any"}) + "\n",
                text=True,
                capture_output=True,
                timeout=8,
                cwd=parent_dir,
            )
            if result.returncode != 0:
                err = (result.stderr or "").strip()
                self.logs_tab.append_event(
                    f"USB2ANY scan failed: worker exited with error{': ' + err if err else ''}"
                )
                return
            response = json.loads((result.stdout or "").strip() or "{}")
            payload = response.get("result", {}) if response.get("ok") else {}
            count = int(payload.get("count", 0)) if isinstance(payload, dict) else 0
            serials = payload.get("serials", []) if isinstance(payload, dict) else []
            
            if count > 0:
                if isinstance(serials, list) and serials:
                    for idx, serial in enumerate(serials):
                        text = str(serial).strip()
                        if text:
                            self.sidebar.usb2any_combo.addItem(text, text)
                        else:
                            # No serial - use None to open first available device
                            self.sidebar.usb2any_combo.addItem(f"USB2ANY #{idx}", None)
                else:
                    # Device detected but no serial info - use None to open first available
                    for idx in range(count):
                        self.sidebar.usb2any_combo.addItem(f"USB2ANY #{idx}", None)
                
                self.logs_tab.append_event(f"USB2ANY devices detected: {count}")
            else:
                self.logs_tab.append_event("No USB2ANY devices detected")
        except Exception as exc:
            self.logs_tab.append_event(f"USB2ANY scan failed: {exc}")

    def _refresh_librevna_devices(self):
        """Detect and populate LibreVNA devices via IPC server."""
        if not self._ensure_librevna_ipc():
            self.logs_tab.append_event("LibreVNA scan skipped: IPC server unavailable")
            return

        payload = None
        last_error = None
        for _ in range(4):
            try:
                payload = self._librevna_ipc_request({"cmd": "list_devices"}, timeout_ms=6000)
                break
            except Exception as exc:
                last_error = exc
                time.sleep(0.25)
        if payload is None:
            self.logs_tab.append_event(f"LibreVNA scan failed: {last_error}")
            return

        devices = payload.get("devices", [])
        count = 0
        if isinstance(devices, list):
            for entry in devices:
                if not isinstance(entry, dict):
                    continue
                label = str(entry.get("label", "LibreVNA")).strip() or "LibreVNA"
                serial = str(entry.get("serial", "")).strip()
                display = f"{label} ({serial})" if serial else label
                self.sidebar.librevna_combo.addItem(display, serial or label)
                count += 1

        if count > 0:
            self.logs_tab.append_event(f"LibreVNA devices detected: {count}")
        else:
            warning = payload.get("warning")
            if isinstance(warning, str) and warning.strip():
                self.logs_tab.append_event(f"LibreVNA warning: {warning.strip()}")
            self.logs_tab.append_event("No LibreVNA devices detected")

    def _default_librevna_ipc_path(self) -> Optional[str]:
        """Return first existing librevna-ipc.exe candidate."""
        configured = str(self._settings.get("librevna_ipc_path", "")).strip()
        if configured and os.path.exists(configured):
            return configured
        if self._librevna_ipc_path_override and os.path.exists(self._librevna_ipc_path_override):
            return self._librevna_ipc_path_override
        root_dir = os.path.dirname(parent_dir)
        candidates = [
            os.path.join(root_dir, "scripts", "vna", "cpp", "build", "librevna-ipc.exe"),
            os.path.join(root_dir, "scripts", "vna", "cpp", "build", "Release", "librevna-ipc.exe"),
            os.path.join(root_dir, "scripts", "vna", "cpp", "build", "RelWithDebInfo", "librevna-ipc.exe"),
            os.path.join(root_dir, "scripts", "vna", "cpp", "build", "Debug", "librevna-ipc.exe"),
        ]
        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return None

    def _ensure_librevna_ipc(self) -> bool:
        """Ensure LibreVNA IPC server is running."""
        if self._librevna_ipc_process and self._librevna_ipc_process.state() != QtCore.QProcess.NotRunning:
            return True

        binary = self._default_librevna_ipc_path()
        if not binary:
            self.logs_tab.append_event("LibreVNA IPC not found (build librevna-ipc.exe first)")
            return False

        process = QtCore.QProcess(self)
        env = QtCore.QProcessEnvironment.systemEnvironment()
        env.insert("LIBREVNA_IPC_NAME", self._librevna_ipc_name)
        path_parts = [MSYS2_UCRT64_BIN, os.path.dirname(binary), WINDOWS_SYSTEM32, SYSTEM_ROOT]
        env.insert("PATH", ";".join([p for p in path_parts if p and os.path.isdir(p)]))
        if MSYS2_QT_PLUGIN_PATH and os.path.isdir(MSYS2_QT_PLUGIN_PATH):
            env.insert("QT_PLUGIN_PATH", MSYS2_QT_PLUGIN_PATH)
            env.insert("QT_QPA_PLATFORM_PLUGIN_PATH", MSYS2_QT_PLUGIN_PATH)
        env.insert("LIBREVNA_USB_BULK_TIMEOUT_MS", "3000")
        env.insert("LIBREVNA_IPC_LOG", os.path.join(os.path.dirname(parent_dir), "librevna-ipc.log"))
        env.insert("LIBREVNA_DISABLE_PACKET_LOG", "1")
        env.insert("LIBREVNA_IPC_DEBUG", "1")
        process.setProcessEnvironment(env)
        process.setProgram(binary)
        process.setArguments([])
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._drain_librevna_process)
        process.readyReadStandardError.connect(self._drain_librevna_process)
        process.start()
        if not process.waitForStarted(3000):
            self.logs_tab.append_event("Failed to start LibreVNA IPC process")
            return False

        self._librevna_ipc_process = process
        self.logs_tab.append_event(f"LibreVNA IPC started: {binary}")
        return True

    def _drain_librevna_process(self):
        """Drain LibreVNA IPC stdout/stderr to IPC log tab."""
        if not self._librevna_ipc_process:
            return
        data = bytes(self._librevna_ipc_process.readAllStandardOutput())
        if not data:
            data = bytes(self._librevna_ipc_process.readAllStandardError())
        text = data.decode("utf-8", errors="ignore").strip()
        if text:
            for line in text.split("\n"):
                line = line.strip()
                if line:
                    self.logs_tab.append_ipc(line)

    def _librevna_ipc_request(self, payload: Dict[str, object], timeout_ms: int = 5000) -> Dict[str, object]:
        """Send request to LibreVNA IPC server and return result payload."""
        self.logs_tab.append_ipc(f">> {payload.get('cmd', 'unknown')} ({self._librevna_ipc_name})")
        socket = QtNetwork.QLocalSocket(self)
        socket.connectToServer(self._librevna_ipc_name)
        if not socket.waitForConnected(timeout_ms):
            raise RuntimeError(f"IPC connect failed: {socket.errorString()}")

        socket.write((json.dumps(payload) + "\n").encode("utf-8"))
        socket.flush()

        buffer = bytearray()
        timer = QtCore.QElapsedTimer()
        timer.start()

        while True:
            idx = buffer.find(b"\n")
            if idx >= 0:
                line = bytes(buffer[:idx]).strip()
                break
            if not socket.waitForReadyRead(200):
                if timer.elapsed() > timeout_ms:
                    raise RuntimeError("IPC response timeout")
                continue
            buffer.extend(bytes(socket.readAll()))

        if not line:
            raise RuntimeError("IPC returned empty response")

        response = json.loads(line.decode("utf-8"))
        if not response.get("ok", False):
            raise RuntimeError(str(response.get("error", "IPC error")))
        result = response.get("result")
        if not isinstance(result, dict):
            raise RuntimeError("IPC returned invalid result payload")
        self.logs_tab.append_ipc(f"<< {payload.get('cmd', 'unknown')} ok")
        return result
    
    def _on_calibration_wizard(self):
        """Handle calibration wizard request."""
        # Load existing calibration config
        config = self._settings.copy()
        wizard = CalibrationWizard(self, config)
        
        # Connect signals
        wizard.calibration_completed.connect(self._on_calibration_completed)
        wizard.tinysa_calibration_requested.connect(
            lambda: self._run_tinysa_calibration(wizard)
        )
        
        wizard.exec()
    
    def _on_calibration_completed(self, results: dict):
        """Handle calibration wizard completion."""
        self.logs_tab.append_event("Calibration wizard completed")
        
        # Store calibration results
        self._calibration_results = results
        
        # Update settings with calibration file paths
        if results.get("librevna_cal_file"):
            self._settings["librevna_cal_path"] = results["librevna_cal_file"]
            self.logs_tab.append_event(f"LibreVNA calibration file: {results['librevna_cal_file']}")
        
        if results.get("tinysa_cal_file"):
            self._settings["tinysa_cal_path"] = results["tinysa_cal_file"]
            self.logs_tab.append_event(f"tinySA calibration file: {results['tinysa_cal_file']}")
        
        # Log reference antenna configuration
        if results.get("reference_antenna") and not results.get("reference_skipped"):
            ref_info = (f"Reference antenna: {results['reference_antenna']}, "
                       f"Gain: {results.get('reference_gain_dbi', 0):.2f} dBi, "
                       f"Freq: {results.get('freq_start_mhz', 0):.0f}-{results.get('freq_stop_mhz', 0):.0f} MHz")
            self.logs_tab.append_event(ref_info)
        
        # Persist settings to disk
        self._save_persistent_state()
    
    def _run_tinysa_calibration(self, wizard):
        """Run tinySA calibration sweep in background using worker thread."""
        try:
            import datetime
            
            # Get sweep parameters from wizard
            sweep_params = wizard._calibration_results.get('tinysa_sweep_params', {})
            start_freq_mhz = sweep_params.get('start_freq_mhz', 3400.0)
            stop_freq_mhz = sweep_params.get('stop_freq_mhz', 3600.0)
            step_freq_mhz = sweep_params.get('step_freq_mhz', 10.0)
            
            self.logs_tab.append_event("Starting tinySA calibration sweep...")
            self.logs_tab.append_event(f"Frequency range: {start_freq_mhz:.1f} - {stop_freq_mhz:.1f} MHz")
            self.logs_tab.append_event(f"Step size: {step_freq_mhz:.1f} MHz")
            
            # Create worker thread for calibration
            self._calibration_worker = SweepWorker(
                mode="tinySA only",
                settings=self._settings,
                tinysa_port=self.sidebar.tinysa_combo.currentData(),
                lmx_serial=self.sidebar.usb2any_combo.currentData(),
                librevna_serial=None,
                start_text=f"{start_freq_mhz}MHz",
                stop_text=f"{stop_freq_mhz}MHz",
                step_mode_per_freq=True,
                step_value_text=f"{step_freq_mhz}MHz",
                ipc_name=self._librevna_ipc_name,
                parent=self,
                use_lmx=True,
            )
            
            # Connect signals
            self._calibration_worker.log.connect(self.logs_tab.append_event)
            self._calibration_worker.tiny_done.connect(
                lambda results: self._on_calibration_sweep_done(wizard, results, start_freq_mhz, stop_freq_mhz, step_freq_mhz)
            )
            self._calibration_worker.failed.connect(
                lambda err: self._on_calibration_sweep_failed(wizard, err)
            )
            
            # Start calibration
            self._calibration_worker.start()
                
        except Exception as e:
            wizard.set_tinysa_calibration_result(False, str(e))
            self.logs_tab.append_event(f"tinySA calibration failed: {e}")
    
    def _on_calibration_sweep_done(self, wizard, results, start_freq_mhz, stop_freq_mhz, step_freq_mhz):
        """Handle calibration sweep completion."""
        try:
            import datetime
            import json
            
            # Convert sweep results to calibration format
            cal_points = []
            for point in results:
                freq_hz = point.get("frequency_hz", 0)
                measured_dbm = point.get("power_dbm", 0)
                requested_dbm = 0.0  # USB2ANY should output 0 dBm
                offset_db = measured_dbm - requested_dbm
                
                cal_points.append({
                    "timestamp": point.get("timestamp", ""),
                    "frequency_hz": freq_hz,
                    "requested_dbm": requested_dbm,
                    "measured_dbm": measured_dbm,
                    "offset_db": offset_db
                })
            
            # Generate calibration file path
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            cal_dir = os.path.join(os.path.dirname(parent_dir), "calibration", "tinysa")
            os.makedirs(cal_dir, exist_ok=True)
            cal_file = os.path.join(cal_dir, f"tinysa_cal_{timestamp}.json")
            
            # Save calibration data
            cal_data = {
                "type": "tinysa_usb2any_calibration",
                "created_at": datetime.datetime.now().isoformat(),
                "metadata": {
                    "start_hz": start_freq_mhz * 1e6,
                    "stop_hz": stop_freq_mhz * 1e6,
                    "step_hz": step_freq_mhz * 1e6,
                    "points": len(cal_points),
                    "requested_dbm": 0.0
                },
                "measurements": cal_points
            }
            
            with open(cal_file, 'w') as f:
                json.dump(cal_data, f, indent=2)
            
            wizard.set_tinysa_calibration_result(True, "", cal_file)
            self.logs_tab.append_event(f"tinySA calibration completed: {len(cal_points)} points saved")
            self.logs_tab.append_event(f"Calibration file: {cal_file}")
            
        except Exception as e:
            wizard.set_tinysa_calibration_result(False, str(e))
            self.logs_tab.append_event(f"Failed to save calibration: {e}")
    
    def _on_calibration_sweep_failed(self, wizard, error):
        """Handle calibration sweep failure."""
        wizard.set_tinysa_calibration_result(False, error)
        self.logs_tab.append_event(f"tinySA calibration failed: {error}")
    
    def _save_calibration_results(self, results: dict):
        """Save calibration results to file."""
        import json
        import datetime
        
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Calibration",
            f"calibration_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
            "JSON Files (*.json);;All Files (*.*)"
        )
        if path:
            try:
                with open(path, 'w') as f:
                    json.dump(results, f, indent=2)
                self.logs_tab.append_event(f"Calibration saved to {path}")
            except Exception as e:
                self.logs_tab.append_event(f"Failed to save calibration: {e}")
    
    def _on_tab_changed(self, index: int):
        """Handle tab change - update plot mode."""
        # Show/hide Measure Golden Sample action based on tab
        if index == 2:  # Antenna Test tab
            self.measure_golden_sample_action.setVisible(True)
        else:
            self.measure_golden_sample_action.setVisible(False)
        
        if index == 0:  # LibreVNA
            self.plot_widget.set_mode("s_parameter")
            if self._last_librevna_plot_payload:
                self.plot_widget.plot_sparameters(self._last_librevna_plot_payload)
            else:
                self.librevna_tab.emit_plot_payload()
        elif index == 1:  # tinySA
            self.plot_widget.set_mode("spectrum")
            # Redraw tinySA data if available
            if self._last_tinysa_plot_data.get("x") and self._last_tinysa_plot_data.get("y"):
                self.plot_widget.clear()
                self.plot_widget.plot_data(
                    self._last_tinysa_plot_data["x"],
                    self._last_tinysa_plot_data["y"],
                    name="tinySA"
                )
        elif index == 2:  # Gain Measure
            self.plot_widget.set_mode("gain")
        # index == 3 (LMX Monitor) and index == 4 (Logs) - keep current plot mode

    def closeEvent(self, event):
        """Clean up child processes before closing."""
        # Check if there are tested results to save
        tested_records = self.gain_measure_tab.tested_table.get_records()

        if tested_records:
            # Show dialog asking if user wants to keep the tested results
            dialog = QtWidgets.QDialog(self)
            dialog.setWindowTitle("Save Test Results")
            layout = QtWidgets.QVBoxLayout(dialog)

            count = len(tested_records)
            label = QtWidgets.QLabel(
                f"There { 'is' if count == 1 else 'are' } {count} tested antenna result(s).\n\n"
                f"Do you want to keep these results for the next session?"
            )
            layout.addWidget(label)

            btn_layout = QtWidgets.QHBoxLayout()
            save_btn = QtWidgets.QPushButton("Save")
            discard_btn = QtWidgets.QPushButton("Discard")
            btn_layout.addWidget(save_btn)
            btn_layout.addWidget(discard_btn)
            layout.addLayout(btn_layout)

            save_btn.clicked.connect(dialog.accept)
            discard_btn.clicked.connect(dialog.reject)

            result = dialog.exec()

            if result == QtWidgets.QDialog.Accepted:
                # Save tested results to persist for next session
                store = self._persist_settings
                store.setValue("antenna_test/tested_results", json.dumps(tested_records))
                store.sync()
                self.logs_tab.append_event(f"Saved {count} tested result(s) for next session")
            elif result == QtWidgets.QDialog.Rejected:
                # Double confirm deletion
                confirm = QtWidgets.QMessageBox.question(
                    self, "Confirm Delete",
                    "Are you sure you want to delete the tested results?\n"
                    "This action cannot be undone.",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.No
                )
                if confirm == QtWidgets.QMessageBox.Yes:
                    self._persist_settings.remove("antenna_test/tested_results")
                else:
                    # User cancelled deletion, save instead
                    store = self._persist_settings
                    store.setValue("antenna_test/tested_results", json.dumps(tested_records))
                    store.sync()

        # Save golden samples (always auto-save)
        self._save_persistent_state()

        if self._librevna_ipc_process and self._librevna_ipc_process.state() != QtCore.QProcess.NotRunning:
            self._librevna_ipc_process.terminate()
            self._librevna_ipc_process.waitForFinished(1000)
        super().closeEvent(event)
