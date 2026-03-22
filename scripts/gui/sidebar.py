"""
Left sidebar widget with device selection and parameter controls.
"""
from __future__ import annotations

from typing import Optional

try:
    from PySide6 import QtCore, QtGui, QtWidgets
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets
    Signal = QtCore.pyqtSignal

from gui.styles import COLORS


class Sidebar(QtWidgets.QWidget):
    """Left sidebar with devices, frequency, step, and golden sample controls."""
    
    # Signals
    device_refresh_requested = Signal()
    frequency_changed = Signal(float, float)  # start, stop
    step_changed = Signal()
    golden_sample_changed = Signal(float, float)  # threshold, tolerance (deprecated)
    test_thresholds_changed = Signal(float, float, float, float)  # s11_threshold, s11_tolerance, gain_tolerance, noise_threshold
    calibration_requested = Signal()
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setFixedWidth(250)
        self.setStyleSheet(f"background-color: {COLORS['sidebar']};")
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the sidebar UI."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(16)
        
        # Devices section
        layout.addWidget(self._create_devices_section())
        
        # Frequency section
        layout.addWidget(self._create_frequency_section())
        
        # Step section
        layout.addWidget(self._create_step_section())
        
        # Golden sample section
        layout.addWidget(self._create_golden_sample_section())
        
        # Calibration button
        layout.addWidget(self._create_calibration_button())
        
        layout.addStretch()
    
    def _create_devices_section(self):
        """Create the devices section."""
        group = QtWidgets.QGroupBox("Devices")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(8)
        
        # USB2ANY combo
        layout.addWidget(QtWidgets.QLabel("USB2ANY"))
        self.usb2any_combo = QtWidgets.QComboBox()
        self.usb2any_combo.addItem("Auto")
        layout.addWidget(self.usb2any_combo)
        
        # tinySA combo
        layout.addWidget(QtWidgets.QLabel("tinySA"))
        self.tinysa_combo = QtWidgets.QComboBox()
        self.tinysa_combo.addItem("Auto")
        layout.addWidget(self.tinysa_combo)
        
        # LibreVNA combo
        layout.addWidget(QtWidgets.QLabel("LibreVNA"))
        self.librevna_combo = QtWidgets.QComboBox()
        self.librevna_combo.addItem("Auto")
        layout.addWidget(self.librevna_combo)
        
        # Refresh button
        refresh_btn = QtWidgets.QPushButton("Refresh")
        refresh_btn.clicked.connect(self.device_refresh_requested.emit)
        layout.addWidget(refresh_btn)
        
        return group
    
    def _create_frequency_section(self):
        """Create the frequency section."""
        group = QtWidgets.QGroupBox("Frequency")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(8)
        
        # Start frequency
        self.freq_start_edit = QtWidgets.QLineEdit("3.52GHz")
        layout.addRow("Start", self.freq_start_edit)
        
        # End frequency
        self.freq_end_edit = QtWidgets.QLineEdit("3.6GHz")
        layout.addRow("End", self.freq_end_edit)
        
        return group
    
    def _create_step_section(self):
        """Create the step section."""
        group = QtWidgets.QGroupBox("Step (Either one)")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setSpacing(8)
        
        # Radio buttons
        self.step_per_freq_radio = QtWidgets.QRadioButton("Per freq.")
        self.step_per_freq_radio.setChecked(True)
        layout.addWidget(self.step_per_freq_radio)
        
        self.step_num_points_radio = QtWidgets.QRadioButton("No. of points")
        layout.addWidget(self.step_num_points_radio)

        self.step_per_freq_radio.toggled.connect(self._on_step_mode_changed)
        self.step_num_points_radio.toggled.connect(self._on_step_mode_changed)
        
        # Value input
        self.step_value_edit = QtWidgets.QLineEdit("10MHz")
        layout.addWidget(self.step_value_edit)

        self._on_step_mode_changed()
        
        return group

    def _on_step_mode_changed(self):
        """Adjust step input format for current step mode."""
        if self.step_num_points_radio.isChecked():
            self.step_value_edit.setPlaceholderText("10")
            self.step_value_edit.setText("10")
            validator = QtGui.QIntValidator(1, 1000000, self)
            self.step_value_edit.setValidator(validator)
        else:
            self.step_value_edit.setPlaceholderText("10MHz")
            self.step_value_edit.setText("10MHz")
            self.step_value_edit.setValidator(None)
    
    def _create_golden_sample_section(self):
        """Create the test thresholds section for antenna testing."""
        group = QtWidgets.QGroupBox("Test Thresholds")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(8)

        # S11 Magnitude Threshold - must be lower than this (default -10 dB)
        self.s11_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.s11_threshold_spin.setRange(-50.0, 0.0)
        self.s11_threshold_spin.setValue(-10.0)
        self.s11_threshold_spin.setSuffix(" dB")
        self.s11_threshold_spin.setToolTip("S11 must be lower than this threshold (e.g., -10 dB)")
        layout.addRow("S11 Threshold", self.s11_threshold_spin)

        # S11 Magnitude Tolerance - vs golden sample (default 2 dB)
        self.s11_tolerance_spin = QtWidgets.QDoubleSpinBox()
        self.s11_tolerance_spin.setRange(0.0, 20.0)
        self.s11_tolerance_spin.setValue(2.0)
        self.s11_tolerance_spin.setSuffix(" dB")
        self.s11_tolerance_spin.setToolTip("Measured S11 must be within this range of golden sample S11")
        layout.addRow("S11 Tolerance", self.s11_tolerance_spin)

        # Antenna Gain Tolerance - vs golden sample (default 1.5 dB)
        self.gain_tolerance_spin = QtWidgets.QDoubleSpinBox()
        self.gain_tolerance_spin.setRange(0.0, 20.0)
        self.gain_tolerance_spin.setValue(1.5)
        self.gain_tolerance_spin.setSuffix(" dB")
        self.gain_tolerance_spin.setToolTip("Measured gain must be within this range of golden sample gain")
        layout.addRow("Gain Tolerance", self.gain_tolerance_spin)

        # Noise Floor Threshold (default -80 dBm)
        self.noise_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.noise_threshold_spin.setRange(-120.0, -40.0)
        self.noise_threshold_spin.setValue(-80.0)
        self.noise_threshold_spin.setSuffix(" dBm")
        self.noise_threshold_spin.setToolTip("Noise floor must be below this threshold")
        layout.addRow("Noise Floor", self.noise_threshold_spin)

        return group
    
    def _create_calibration_button(self):
        """Create the calibration button."""
        btn = QtWidgets.QPushButton("Calibration...")
        btn.clicked.connect(self.calibration_requested.emit)
        return btn
