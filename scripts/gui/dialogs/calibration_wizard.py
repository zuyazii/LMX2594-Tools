"""
Calibration wizard dialog with step-by-step guidance.
"""
from __future__ import annotations

import os
import sys
import json
from typing import Optional, Callable, Dict, Any
from datetime import datetime

try:
    from PySide6 import QtCore, QtWidgets
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtWidgets
    Signal = QtCore.pyqtSignal

# Add parent directory to path for imports
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(script_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from gui.styles import COLORS, get_stylesheet


class CalibrationWizard(QtWidgets.QDialog):
    """Multi-step calibration wizard."""
    
    # Signals
    calibration_completed = Signal(dict)  # Emits calibration results
    tinysa_calibration_requested = Signal()
    
    STEP_TITLES = [
        "Step 1: LibreVNA Calibration",
        "Step 2: tinySA Calibration", 
        "Step 3: Reference Antenna Setup",
        "Step 4: Calibration Complete",
    ]
    
    # Default calibration paths
    CALIBRATION_DIR = os.path.join(os.path.dirname(parent_dir), "calibration")
    TINYSA_CAL_DIR = os.path.join(CALIBRATION_DIR, "tinysa")
    DEVICES_CAL_FILE = os.path.join(CALIBRATION_DIR, "devices_calibrate.json")
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None, config: Optional[Dict[str, Any]] = None):
        super().__init__(parent)
        self.setWindowTitle("Calibration Wizard")
        self.resize(650, 500)
        if parent is not None and parent.styleSheet():
            self.setStyleSheet(parent.styleSheet())
        else:
            self.setStyleSheet(get_stylesheet())
        
        self._current_step = 0
        self._calibration_results = {}
        self._tinysa_calibrated = False
        self._tinysa_attempted = False
        self._config = config or {}
        
        # Ensure calibration directories exist
        os.makedirs(self.TINYSA_CAL_DIR, exist_ok=True)
        os.makedirs(self.CALIBRATION_DIR, exist_ok=True)
        
        # Load existing calibration if available
        self._load_existing_calibration()
        
        self._setup_ui()
        self._populate_from_config()
    
    def _load_existing_calibration(self):
        """Load existing calibration from devices_calibrate.json."""
        if os.path.exists(self.DEVICES_CAL_FILE):
            try:
                with open(self.DEVICES_CAL_FILE, 'r') as f:
                    self._calibration_results = json.load(f)
            except Exception:
                pass
    
    def _save_calibration_to_file(self):
        """Save calibration results to devices_calibrate.json."""
        try:
            # Add timestamp
            self._calibration_results["last_updated"] = datetime.now().isoformat()
            
            with open(self.DEVICES_CAL_FILE, 'w') as f:
                json.dump(self._calibration_results, f, indent=2)
            return True
        except Exception as e:
            QtWidgets.QMessageBox.warning(
                self,
                "Save Error",
                f"Failed to save calibration: {e}"
            )
            return False
    
    def _setup_ui(self):
        """Setup the wizard UI."""
        layout = QtWidgets.QVBoxLayout(self)
        
        # Title
        self.title_label = QtWidgets.QLabel()
        font = self.title_label.font()
        font.setPointSize(12)
        font.setBold(True)
        self.title_label.setFont(font)
        layout.addWidget(self.title_label)
        
        # Progress bar
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setMaximum(3)
        layout.addWidget(self.progress_bar)
        
        layout.addSpacing(20)
        
        # Stacked widget for steps
        self.stack = QtWidgets.QStackedWidget()
        layout.addWidget(self.stack, stretch=1)
        
        # Create step pages
        self._create_steps()
        
        layout.addSpacing(20)
        
        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addStretch()
        
        self.back_button = QtWidgets.QPushButton("< Back")
        self.back_button.clicked.connect(self._go_back)
        button_layout.addWidget(self.back_button)
        
        self.skip_button = QtWidgets.QPushButton("Skip")
        self.skip_button.clicked.connect(self._skip_step)
        button_layout.addWidget(self.skip_button)
        
        self.next_button = QtWidgets.QPushButton("Next >")
        self.next_button.clicked.connect(self._go_next)
        button_layout.addWidget(self.next_button)
        
        self.cancel_button = QtWidgets.QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_button)
        
        layout.addLayout(button_layout)
        
        self._update_ui()
    
    def _create_steps(self):
        """Create wizard step pages."""
        # Step 1: LibreVNA Calibration
        self.stack.addWidget(self._create_librevna_step())
        
        # Step 2: tinySA Calibration
        self.stack.addWidget(self._create_tinysa_step())
        
        # Step 3: Reference Antenna
        self.stack.addWidget(self._create_reference_step())
        
        # Step 4: Complete
        self.stack.addWidget(self._create_complete_step())
    
    def _create_librevna_step(self):
        """Create LibreVNA calibration step."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        
        # Instruction label
        instruction = QtWidgets.QLabel(
            "<b>Please perform calibration in the LibreVNA GUI first.</b><br><br>"
            "1. Open LibreVNA software<br>"
            "2. Perform Open/Short/Load calibration<br>"
            "3. Save the calibration file (.cal)<br>"
            "4. Select the calibration file below"
        )
        instruction.setWordWrap(True)
        layout.addWidget(instruction)
        
        layout.addSpacing(15)
        
        # Cal file path
        file_layout = QtWidgets.QHBoxLayout()
        file_layout.addWidget(QtWidgets.QLabel("Calibration file:"))
        self.librevna_cal_edit = QtWidgets.QLineEdit()
        self.librevna_cal_edit.setPlaceholderText("Select LibreVNA .cal file...")
        file_layout.addWidget(self.librevna_cal_edit, stretch=1)
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self._browse_librevna_cal)
        file_layout.addWidget(browse_btn)
        layout.addLayout(file_layout)
        
        # Current loaded file display
        layout.addSpacing(10)
        self.librevna_current_label = QtWidgets.QLabel()
        self.librevna_current_label.setStyleSheet(f"color: {COLORS['text_dim']}; font-style: italic;")
        self.librevna_current_label.setWordWrap(True)
        layout.addWidget(self.librevna_current_label)
        
        layout.addStretch()
        return widget
    
    def _browse_librevna_cal(self):
        """Browse for LibreVNA calibration file."""
        start_dir = self.CALIBRATION_DIR if os.path.exists(self.CALIBRATION_DIR) else ""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select LibreVNA Calibration File",
            start_dir,
            "Calibration Files (*.cal *.s1p *.s2p);;All Files (*.*)"
        )
        if path:
            self.librevna_cal_edit.setText(path)
            self._calibration_results["librevna_cal_file"] = path
    
    def _create_tinysa_step(self):
        """Create tinySA calibration step."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        
        # Instructions
        instruction = QtWidgets.QLabel(
            "<b>tinySA Calibration Setup:</b><br><br>"
            "1. Select tinySA and USB2ANY devices below<br>"
            "2. Connect USB2ANY RF output to tinySA RF input with SMA cable<br>"
            "3. Configure sweep parameters<br>"
            "4. Click 'Run Calibration' to perform sweep"
        )
        instruction.setWordWrap(True)
        layout.addWidget(instruction)
        
        layout.addSpacing(15)
        
        # Device selection
        device_group = QtWidgets.QGroupBox("Device Selection")
        device_layout = QtWidgets.QFormLayout()
        
        self.wizard_tinysa_combo = QtWidgets.QComboBox()
        self.wizard_tinysa_combo.currentIndexChanged.connect(self._on_wizard_tinysa_changed)
        device_layout.addRow("tinySA Port:", self.wizard_tinysa_combo)
        
        self.wizard_usb2any_combo = QtWidgets.QComboBox()
        self.wizard_usb2any_combo.currentIndexChanged.connect(self._on_wizard_usb2any_changed)
        device_layout.addRow("USB2ANY Serial:", self.wizard_usb2any_combo)
        
        device_group.setLayout(device_layout)
        layout.addWidget(device_group)
        
        layout.addSpacing(10)
        
        # Device status
        status_group = QtWidgets.QGroupBox("Device Status")
        status_layout = QtWidgets.QVBoxLayout()
        self.tinysa_device_status = QtWidgets.QLabel("tinySA: Not selected")
        self.usb2any_device_status = QtWidgets.QLabel("USB2ANY: Not selected")
        status_layout.addWidget(self.tinysa_device_status)
        status_layout.addWidget(self.usb2any_device_status)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addSpacing(10)
        
        # Sweep parameters
        params_group = QtWidgets.QGroupBox("Sweep Parameters")
        params_layout = QtWidgets.QFormLayout()
        
        self.tinysa_start_freq = QtWidgets.QDoubleSpinBox()
        self.tinysa_start_freq.setRange(10.0, 15000.0)
        self.tinysa_start_freq.setValue(3400.0)
        self.tinysa_start_freq.setSuffix(" MHz")
        self.tinysa_start_freq.setDecimals(1)
        params_layout.addRow("Start Frequency:", self.tinysa_start_freq)
        
        self.tinysa_stop_freq = QtWidgets.QDoubleSpinBox()
        self.tinysa_stop_freq.setRange(10.0, 15000.0)
        self.tinysa_stop_freq.setValue(3600.0)
        self.tinysa_stop_freq.setSuffix(" MHz")
        self.tinysa_stop_freq.setDecimals(1)
        params_layout.addRow("Stop Frequency:", self.tinysa_stop_freq)
        
        self.tinysa_step_freq = QtWidgets.QDoubleSpinBox()
        self.tinysa_step_freq.setRange(0.1, 1000.0)
        self.tinysa_step_freq.setValue(10.0)
        self.tinysa_step_freq.setSuffix(" MHz")
        self.tinysa_step_freq.setDecimals(1)
        params_layout.addRow("Step Size:", self.tinysa_step_freq)
        
        params_group.setLayout(params_layout)
        layout.addWidget(params_group)
        
        layout.addSpacing(10)
        
        self.tinysa_calibrate_btn = QtWidgets.QPushButton("Run Calibration")
        self.tinysa_calibrate_btn.clicked.connect(self._on_tinysa_calibrate)
        layout.addWidget(self.tinysa_calibrate_btn)
        
        layout.addSpacing(10)
        
        self.tinysa_status_label = QtWidgets.QLabel("Status: Ready")
        layout.addWidget(self.tinysa_status_label)
        
        layout.addSpacing(15)
        
        # Cal file path selection
        file_layout = QtWidgets.QHBoxLayout()
        file_layout.addWidget(QtWidgets.QLabel("Calibration file:"))
        self.tinysa_cal_edit = QtWidgets.QLineEdit()
        self.tinysa_cal_edit.setPlaceholderText("Auto-saved after calibration...")
        self.tinysa_cal_edit.setReadOnly(True)
        file_layout.addWidget(self.tinysa_cal_edit, stretch=1)
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self._browse_tinysa_cal)
        file_layout.addWidget(browse_btn)
        layout.addLayout(file_layout)
        
        # Current loaded file display
        layout.addSpacing(10)
        self.tinysa_current_label = QtWidgets.QLabel()
        self.tinysa_current_label.setStyleSheet(f"color: {COLORS['text_dim']}; font-style: italic;")
        self.tinysa_current_label.setWordWrap(True)
        layout.addWidget(self.tinysa_current_label)
        
        layout.addStretch()
        return widget
    
    def _browse_tinysa_cal(self):
        """Browse for tinySA calibration file."""
        start_dir = self.TINYSA_CAL_DIR if os.path.exists(self.TINYSA_CAL_DIR) else ""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select tinySA Calibration File",
            start_dir,
            "JSON Files (*.json);;All Files (*.*)"
        )
        if path:
            self.tinysa_cal_edit.setText(path)
            self._calibration_results["tinysa_cal_file"] = path
            self.tinysa_current_label.setText(f"Selected: {os.path.basename(path)}")
    
    def _populate_device_combos(self):
        """Populate device combo boxes from parent window."""
        parent = self.parent()
        
        # Populate tinySA combo
        if hasattr(parent, 'sidebar') and hasattr(parent.sidebar, 'tinysa_combo'):
            self.wizard_tinysa_combo.blockSignals(True)
            self.wizard_tinysa_combo.clear()
            
            for i in range(parent.sidebar.tinysa_combo.count()):
                text = parent.sidebar.tinysa_combo.itemText(i)
                data = parent.sidebar.tinysa_combo.itemData(i)
                self.wizard_tinysa_combo.addItem(text, data)
            
            # Set current selection to match sidebar
            current_idx = parent.sidebar.tinysa_combo.currentIndex()
            self.wizard_tinysa_combo.setCurrentIndex(current_idx)
            self.wizard_tinysa_combo.blockSignals(False)
        
        # Populate USB2ANY combo
        if hasattr(parent, 'sidebar') and hasattr(parent.sidebar, 'usb2any_combo'):
            self.wizard_usb2any_combo.blockSignals(True)
            self.wizard_usb2any_combo.clear()
            
            for i in range(parent.sidebar.usb2any_combo.count()):
                text = parent.sidebar.usb2any_combo.itemText(i)
                data = parent.sidebar.usb2any_combo.itemData(i)
                self.wizard_usb2any_combo.addItem(text, data)
            
            # Set current selection to match sidebar
            current_idx = parent.sidebar.usb2any_combo.currentIndex()
            self.wizard_usb2any_combo.setCurrentIndex(current_idx)
            self.wizard_usb2any_combo.blockSignals(False)
        
        # Update status after populating
        self._check_device_connections()
    
    def _on_wizard_tinysa_changed(self, index):
        """Handle tinySA selection change in wizard."""
        parent = self.parent()
        if hasattr(parent, 'sidebar') and hasattr(parent.sidebar, 'tinysa_combo'):
            parent.sidebar.tinysa_combo.setCurrentIndex(index)
        self._check_device_connections()
    
    def _on_wizard_usb2any_changed(self, index):
        """Handle USB2ANY selection change in wizard."""
        parent = self.parent()
        if hasattr(parent, 'sidebar') and hasattr(parent.sidebar, 'usb2any_combo'):
            parent.sidebar.usb2any_combo.setCurrentIndex(index)
        self._check_device_connections()
    
    def _check_device_connections(self):
        """Check if tinySA and USB2ANY devices are connected."""
        tinysa_connected = False
        usb2any_connected = False
        
        # Check tinySA from wizard combo
        tinysa_text = self.wizard_tinysa_combo.currentText()
        tinysa_port = self.wizard_tinysa_combo.currentData()
        if tinysa_text and tinysa_text != "Auto":
            tinysa_connected = True
            display_port = tinysa_port if tinysa_port else tinysa_text
            self.tinysa_device_status.setText(f"tinySA: Connected ({display_port})")
            self.tinysa_device_status.setStyleSheet(f"color: {COLORS['success']};")
        else:
            self.tinysa_device_status.setText("tinySA: Not selected")
            self.tinysa_device_status.setStyleSheet(f"color: {COLORS.get('error', '#FF4444')};")
        
        # Check USB2ANY from wizard combo
        usb2any_text = self.wizard_usb2any_combo.currentText()
        usb2any_serial = self.wizard_usb2any_combo.currentData()
        if usb2any_text and usb2any_text != "Auto":
            usb2any_connected = True
            display_serial = usb2any_serial if usb2any_serial else usb2any_text
            self.usb2any_device_status.setText(f"USB2ANY: Connected ({display_serial})")
            self.usb2any_device_status.setStyleSheet(f"color: {COLORS['success']};")
        else:
            self.usb2any_device_status.setText("USB2ANY: Not selected")
            self.usb2any_device_status.setStyleSheet(f"color: {COLORS.get('error', '#FF4444')};")
        
        return tinysa_connected and usb2any_connected
    
    def _on_tinysa_calibrate(self):
        """Handle tinySA calibration button click."""
        # Check device connections first
        if not self._check_device_connections():
            QtWidgets.QMessageBox.warning(
                self,
                "Device Connection Error",
                "Please ensure both tinySA and USB2ANY devices are connected before running calibration."
            )
            return
        
        # Confirm SMA cable connection
        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Setup",
            "Have you connected the USB2ANY RF output to the tinySA RF input with an SMA cable?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )
        
        if reply != QtWidgets.QMessageBox.Yes:
            return
        
        self._tinysa_attempted = True
        self.tinysa_calibrate_btn.setEnabled(False)
        self.tinysa_status_label.setText("Status: Calibrating...")
        
        # Collect sweep parameters
        sweep_params = {
            'start_freq_mhz': self.tinysa_start_freq.value(),
            'stop_freq_mhz': self.tinysa_stop_freq.value(),
            'step_freq_mhz': self.tinysa_step_freq.value()
        }
        
        # Emit signal for external handling with parameters
        self.tinysa_calibration_requested.emit()
        
        # Store parameters in results for later use
        self._calibration_results['tinysa_sweep_params'] = sweep_params
    
    def set_tinysa_calibration_result(self, success: bool, message: str = "", cal_file: str = ""):
        """Set the result of tinySA calibration (called externally)."""
        self.tinysa_calibrate_btn.setEnabled(True)
        if success:
            self._tinysa_calibrated = True
            self.tinysa_status_label.setText("Status: Calibration Complete")
            self.tinysa_status_label.setStyleSheet(f"color: {COLORS['success']};")
            self._calibration_results["tinysa_calibrated"] = True
            
            # Save calibration file path
            if cal_file:
                self._calibration_results["tinysa_cal_file"] = cal_file
                self.tinysa_cal_edit.setText(cal_file)
                self.tinysa_current_label.setText(f"Saved: {os.path.basename(cal_file)}")
        else:
            self._calibration_results["tinysa_calibrated"] = False
            self.tinysa_status_label.setText(f"Status: Failed - {message}")
            self.tinysa_status_label.setStyleSheet(f"color: {COLORS.get('error', '#FF4444')};")
    
    def _create_reference_step(self):
        """Create reference antenna setup step."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        
        layout.addWidget(QtWidgets.QLabel(
            "Configure reference antenna for gain measurements:"
        ))
        layout.addSpacing(10)
        
        # Reference antenna selection
        ref_layout = QtWidgets.QFormLayout()
        
        self.ref_antenna_combo = QtWidgets.QComboBox()
        self.ref_antenna_combo.addItems([
            "Select reference antenna...",
            "Golden Sample A",
            "Golden Sample B", 
            "Custom Reference"
        ])
        ref_layout.addRow("Reference Antenna:", self.ref_antenna_combo)
        
        # Reference gain input
        self.ref_gain_spin = QtWidgets.QDoubleSpinBox()
        self.ref_gain_spin.setRange(-50.0, 50.0)
        self.ref_gain_spin.setValue(0.0)
        self.ref_gain_spin.setSuffix(" dBi")
        self.ref_gain_spin.setDecimals(2)
        ref_layout.addRow("Reference Gain:", self.ref_gain_spin)
        
        # Frequency range
        self.ref_freq_start = QtWidgets.QDoubleSpinBox()
        self.ref_freq_start.setRange(10.0, 15000.0)
        self.ref_freq_start.setValue(3400.0)
        self.ref_freq_start.setSuffix(" MHz")
        ref_layout.addRow("Start Frequency:", self.ref_freq_start)
        
        self.ref_freq_stop = QtWidgets.QDoubleSpinBox()
        self.ref_freq_stop.setRange(10.0, 15000.0)
        self.ref_freq_stop.setValue(3600.0)
        self.ref_freq_stop.setSuffix(" MHz")
        ref_layout.addRow("Stop Frequency:", self.ref_freq_stop)
        
        layout.addLayout(ref_layout)
        
        # Current loaded reference display
        layout.addSpacing(10)
        self.reference_current_label = QtWidgets.QLabel()
        self.reference_current_label.setStyleSheet(f"color: {COLORS['text_dim']}; font-style: italic;")
        self.reference_current_label.setWordWrap(True)
        layout.addWidget(self.reference_current_label)
        
        layout.addStretch()
        return widget
    
    def _create_complete_step(self):
        """Create calibration complete step."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        
        # Success icon/message
        complete_label = QtWidgets.QLabel("Calibration Setup Complete!")
        font = complete_label.font()
        font.setPointSize(14)
        font.setBold(True)
        complete_label.setFont(font)
        complete_label.setStyleSheet(f"color: {COLORS['success']};")
        complete_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(complete_label)
        
        layout.addSpacing(20)
        
        # Summary
        self.summary_text = QtWidgets.QTextEdit()
        self.summary_text.setReadOnly(True)
        self.summary_text.setMaximumHeight(200)
        layout.addWidget(self.summary_text)
        
        layout.addSpacing(10)
        
        # Save calibration checkbox
        self.save_cal_check = QtWidgets.QCheckBox("Save calibration to file")
        self.save_cal_check.setChecked(True)
        layout.addWidget(self.save_cal_check)
        
        layout.addStretch()
        return widget
    
    def _populate_from_config(self):
        """Populate wizard fields from loaded calibration config and settings."""
        # Try to load from config (passed from main window settings)
        config_librevna = self._config.get("librevna_cal_path", "")
        config_tinysa = self._config.get("tinysa_cal_path", "")
        
        # LibreVNA calibration - prefer existing calibration results, fallback to config
        librevna_cal = self._calibration_results.get("librevna_cal_file", "") or config_librevna
        if librevna_cal and os.path.exists(librevna_cal):
            self.librevna_cal_edit.setText(librevna_cal)
            self.librevna_current_label.setText(f"Currently loaded: {os.path.basename(librevna_cal)}")
        
        # tinySA calibration - prefer existing calibration results, fallback to config
        tinysa_cal = self._calibration_results.get("tinysa_cal_file", "") or config_tinysa
        if tinysa_cal and os.path.exists(tinysa_cal):
            self.tinysa_cal_edit.setText(tinysa_cal)
            self.tinysa_current_label.setText(f"Currently loaded: {os.path.basename(tinysa_cal)}")
            self._tinysa_calibrated = True
        
        # Populate sweep parameters from calibration results if available
        sweep_params = self._calibration_results.get("tinysa_sweep_params", {})
        if sweep_params:
            self.tinysa_start_freq.setValue(sweep_params.get('start_freq_mhz', 3400.0))
            self.tinysa_stop_freq.setValue(sweep_params.get('stop_freq_mhz', 3600.0))
            self.tinysa_step_freq.setValue(sweep_params.get('step_freq_mhz', 10.0))
        
        # Reference antenna
        ref_antenna = self._calibration_results.get("reference_antenna", "")
        if ref_antenna:
            idx = self.ref_antenna_combo.findText(ref_antenna)
            if idx >= 0:
                self.ref_antenna_combo.setCurrentIndex(idx)
        
        ref_gain = self._calibration_results.get("reference_gain_dbi", 0.0)
        self.ref_gain_spin.setValue(ref_gain)
        
        freq_start = self._calibration_results.get("freq_start_mhz", 3400.0)
        self.ref_freq_start.setValue(freq_start)
        
        freq_stop = self._calibration_results.get("freq_stop_mhz", 3600.0)
        self.ref_freq_stop.setValue(freq_stop)
        
        if ref_antenna and ref_antenna != "Select reference antenna...":
            self.reference_current_label.setText(
                f"Currently configured: {ref_antenna} ({ref_gain:.2f} dBi, {freq_start:.0f}-{freq_stop:.0f} MHz)"
            )
    
    def _go_back(self):
        """Navigate to previous step."""
        if self._current_step > 0:
            self._current_step -= 1
            self._update_ui()
    
    def _skip_step(self):
        """Skip the current step."""
        if self._current_step == 0:  # LibreVNA
            self._calibration_results["librevna_skipped"] = True
            self._calibration_results["librevna_cal_file"] = ""
        elif self._current_step == 1:  # tinySA
            self._calibration_results["tinysa_skipped"] = True
            self._calibration_results["tinysa_calibrated"] = False
        elif self._current_step == 2:  # Reference
            self._calibration_results["reference_skipped"] = True
        
        self._go_next()
    
    def _go_next(self):
        """Navigate to next step or finish."""
        if self._current_step < len(self.STEP_TITLES) - 1:
            # Collect data from current step before advancing
            self._collect_step_data()
            self._current_step += 1
            self._update_ui()
            
            # Update summary on final step
            if self._current_step == len(self.STEP_TITLES) - 1:
                self._update_summary()
        else:
            # Final step - finish wizard
            self._finish_wizard()
    
    def _collect_step_data(self):
        """Collect data from the current step."""
        if self._current_step == 0:  # LibreVNA step
            cal_file = self.librevna_cal_edit.text().strip()
            if cal_file:
                self._calibration_results["librevna_cal_file"] = cal_file
                self._calibration_results["librevna_completed"] = True
                self._calibration_results["librevna_skipped"] = False
            else:
                self._calibration_results["librevna_skipped"] = True
                self._calibration_results["librevna_completed"] = False
        elif self._current_step == 1:  # tinySA step
            tinysa_cal = self.tinysa_cal_edit.text().strip()
            if tinysa_cal:
                self._calibration_results["tinysa_cal_file"] = tinysa_cal
            self._calibration_results["tinysa_skipped"] = not self._tinysa_calibrated
        elif self._current_step == 2:  # Reference step
            ref_antenna = self.ref_antenna_combo.currentText()
            if ref_antenna and ref_antenna != "Select reference antenna...":
                self._calibration_results["reference_antenna"] = ref_antenna
                self._calibration_results["reference_gain_dbi"] = self.ref_gain_spin.value()
                self._calibration_results["freq_start_mhz"] = self.ref_freq_start.value()
                self._calibration_results["freq_stop_mhz"] = self.ref_freq_stop.value()
                self._calibration_results["reference_skipped"] = False
            else:
                self._calibration_results["reference_skipped"] = True
    
    def _update_ui(self):
        """Update UI based on current step."""
        # Update title
        self.title_label.setText(self.STEP_TITLES[self._current_step])
        
        # Update progress bar
        self.progress_bar.setValue(self._current_step)
        
        # Update stack
        self.stack.setCurrentIndex(self._current_step)
        
        # Update buttons
        self.back_button.setEnabled(self._current_step > 0)
        
        # Show/hide skip button (not on final step)
        self.skip_button.setVisible(self._current_step < len(self.STEP_TITLES) - 1)
        
        if self._current_step == len(self.STEP_TITLES) - 1:
            self.next_button.setText("Finish")
        else:
            self.next_button.setText("Next >")
        
        # Populate device combos and check connections when entering tinySA step
        if self._current_step == 1:  # tinySA step
            self._populate_device_combos()
    
    def _update_summary(self):
        """Update the summary text on the final step."""
        lines = ["Calibration Summary:", ""]
        
        # LibreVNA status
        if self._calibration_results.get("librevna_skipped"):
            lines.append("LibreVNA: Skipped")
        elif self._calibration_results.get("librevna_cal_file"):
            lines.append("LibreVNA: Calibration file provided")
            lines.append(f"  File: {self._calibration_results['librevna_cal_file']}")
        else:
            lines.append("LibreVNA: Not configured")
        
        lines.append("")
        
        # tinySA status
        if self._calibration_results.get("tinysa_skipped"):
            lines.append("tinySA: Skipped")
        elif self._calibration_results.get("tinysa_calibrated"):
            lines.append("tinySA: Calibrated")
            if self._calibration_results.get("tinysa_cal_file"):
                lines.append(f"  File: {self._calibration_results['tinysa_cal_file']}")
        elif self._tinysa_attempted:
            lines.append("tinySA: Calibration failed")
        else:
            lines.append("tinySA: Not configured")
        
        lines.append("")
        
        # Reference antenna
        if self._calibration_results.get("reference_skipped"):
            lines.append("Reference Antenna: Skipped")
        else:
            ref = self._calibration_results.get("reference_antenna", "")
            if ref and ref != "Select reference antenna...":
                lines.append(f"Reference Antenna: {ref}")
                lines.append(f"  Gain: {self._calibration_results.get('reference_gain_dbi', 0):.2f} dBi")
                lines.append(f"  Frequency: {self._calibration_results.get('freq_start_mhz', 0):.0f} - "
                            f"{self._calibration_results.get('freq_stop_mhz', 0):.0f} MHz")
            else:
                lines.append("Reference Antenna: Not configured")
        
        self.summary_text.setText("\n".join(lines))
    
    def _finish_wizard(self):
        """Complete the wizard and emit results."""
        self._collect_step_data()
        
        # Save to file if requested
        if self.save_cal_check.isChecked():
            self._save_calibration_to_file()
        
        # Set status flags
        if self._calibration_results.get("librevna_completed"):
            self._calibration_results["librevna_status"] = "calibrated"
        else:
            self._calibration_results["librevna_status"] = "skipped"

        if self._calibration_results.get("tinysa_calibrated"):
            self._calibration_results["tinysa_status"] = "calibrated"
        elif self._calibration_results.get("tinysa_skipped"):
            self._calibration_results["tinysa_status"] = "skipped"
        elif self._tinysa_attempted:
            self._calibration_results["tinysa_status"] = "failed"
        else:
            self._calibration_results["tinysa_status"] = "skipped"

        ref = self._calibration_results.get("reference_antenna", "")
        self._calibration_results["reference_status"] = (
            "skipped" if self._calibration_results.get("reference_skipped")
            else ("configured" if ref and ref != "Select reference antenna..." else "skipped")
        )
        
        # Emit completion signal
        self.calibration_completed.emit(self._calibration_results)
        self.accept()
    
    def get_calibration_results(self) -> dict:
        """Return the collected calibration results."""
        return self._calibration_results.copy()
