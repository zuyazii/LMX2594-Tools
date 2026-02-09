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
    golden_sample_changed = Signal(float, float)  # threshold, tolerance
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
        """Create the golden sample section."""
        group = QtWidgets.QGroupBox("Golden sample")
        layout = QtWidgets.QFormLayout(group)
        layout.setSpacing(8)
        
        # Threshold
        self.golden_threshold_spin = QtWidgets.QDoubleSpinBox()
        self.golden_threshold_spin.setRange(-200.0, 50.0)
        self.golden_threshold_spin.setValue(-60.0)
        self.golden_threshold_spin.setSuffix(" dB")
        layout.addRow("Threshold", self.golden_threshold_spin)
        
        # Tolerance
        self.golden_tolerance_spin = QtWidgets.QDoubleSpinBox()
        self.golden_tolerance_spin.setRange(0.0, 50.0)
        self.golden_tolerance_spin.setValue(3.0)
        self.golden_tolerance_spin.setSuffix(" dB")
        layout.addRow("Tolerance", self.golden_tolerance_spin)
        
        return group
    
    def _create_calibration_button(self):
        """Create the calibration button."""
        btn = QtWidgets.QPushButton("Calibration...")
        btn.clicked.connect(self.calibration_requested.emit)
        return btn
