"""
Gain Measure tab - displays antenna test results and gain calculations.

The gain measurement flow:
1. Connect synthesizer with reference antenna, SA with another reference antenna
2. Measure signal strength (SS) of source antenna at each frequency
3. Replace reference antenna connected to SA with target antenna
4. Measure SS of target antenna at each frequency
5. Calculate: Antenna Gain = (Antenna SS - Reference SS) + Reference Gain
"""
from __future__ import annotations

import datetime
import json
import uuid
from typing import Optional, Dict, List

try:
    from PySide6 import QtCore, QtWidgets
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtWidgets
    Signal = QtCore.pyqtSignal


class AntennaRecordDialog(QtWidgets.QDialog):
    """Dialog for adding/editing antenna measurement records."""
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None, 
                 record: Optional[Dict] = None,
                 record_type: str = "reference"):
        super().__init__(parent)
        self._record = record or {}
        self._record_type = record_type
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the dialog UI."""
        self.setWindowTitle("Antenna Measurement Record")
        self.resize(500, 400)
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # Basic info section
        form = QtWidgets.QFormLayout()
        
        self.id_edit = QtWidgets.QLineEdit(
            str(self._record.get("id", str(uuid.uuid4())[:8]))
        )
        self.label_edit = QtWidgets.QLineEdit(
            str(self._record.get("label", ""))
        )
        
        # Reference gain (only for reference antennas)
        self.ref_gain_spin = QtWidgets.QDoubleSpinBox()
        self.ref_gain_spin.setRange(-50.0, 50.0)
        self.ref_gain_spin.setDecimals(2)
        self.ref_gain_spin.setSuffix(" dBi")
        self.ref_gain_spin.setValue(float(self._record.get("reference_gain_dbi", 0.0)))
        
        form.addRow("ID:", self.id_edit)
        form.addRow("Label:", self.label_edit)
        
        if self._record_type == "reference":
            form.addRow("Reference Gain:", self.ref_gain_spin)
        
        layout.addLayout(form)
        
        # Measurement data section
        data_group = QtWidgets.QGroupBox("Measurement Data (Frequency vs Signal Strength)")
        data_layout = QtWidgets.QVBoxLayout(data_group)
        
        # Data table
        self.data_table = QtWidgets.QTableWidget(0, 2)
        self.data_table.setHorizontalHeaderLabels(["Frequency (Hz)", "Power (dBm)"])
        self.data_table.horizontalHeader().setStretchLastSection(True)
        data_layout.addWidget(self.data_table)
        
        # Load existing measurements
        measurements = self._record.get("measurements", [])
        for m in measurements:
            row = self.data_table.rowCount()
            self.data_table.insertRow(row)
            self.data_table.setItem(row, 0, QtWidgets.QTableWidgetItem(
                str(m.get("frequency_hz", ""))
            ))
            self.data_table.setItem(row, 1, QtWidgets.QTableWidgetItem(
                str(m.get("power_dbm", ""))
            ))
        
        # Data buttons
        data_btn_layout = QtWidgets.QHBoxLayout()
        add_point_btn = QtWidgets.QPushButton("Add Point")
        add_point_btn.clicked.connect(self._add_data_point)
        remove_point_btn = QtWidgets.QPushButton("Remove Point")
        remove_point_btn.clicked.connect(self._remove_data_point)
        data_btn_layout.addWidget(add_point_btn)
        data_btn_layout.addWidget(remove_point_btn)
        data_btn_layout.addStretch()
        data_layout.addLayout(data_btn_layout)
        
        layout.addWidget(data_group)
        
        # Dialog buttons
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
    
    def _add_data_point(self):
        """Add a new data point row."""
        row = self.data_table.rowCount()
        self.data_table.insertRow(row)
        self.data_table.setItem(row, 0, QtWidgets.QTableWidgetItem(""))
        self.data_table.setItem(row, 1, QtWidgets.QTableWidgetItem(""))
    
    def _remove_data_point(self):
        """Remove selected data point."""
        current_row = self.data_table.currentRow()
        if current_row >= 0:
            self.data_table.removeRow(current_row)
    
    def get_record(self) -> Dict:
        """Get the record from the dialog."""
        measurements = []
        for row in range(self.data_table.rowCount()):
            freq_item = self.data_table.item(row, 0)
            power_item = self.data_table.item(row, 1)
            if freq_item and power_item:
                try:
                    freq = float(freq_item.text().strip())
                    power = float(power_item.text().strip())
                    measurements.append({
                        "frequency_hz": freq,
                        "power_dbm": power
                    })
                except ValueError:
                    continue
        
        record = {
            "id": self.id_edit.text().strip(),
            "label": self.label_edit.text().strip(),
            "measurements": measurements,
            "timestamp": self._record.get("timestamp", 
                datetime.datetime.now().isoformat()),
        }
        
        if self._record_type == "reference":
            record["reference_gain_dbi"] = self.ref_gain_spin.value()
        
        return record


class AntennaTableWidget(QtWidgets.QWidget):
    """Table widget for antenna measurements with CRUD operations."""
    
    record_selected = Signal(dict)  # Emitted when a record is double-clicked
    records_changed = Signal()  # Emitted when records are modified
    
    def __init__(self, title: str, record_type: str = "reference",
                 parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._title = title
        self._record_type = record_type
        self._records: List[Dict] = []
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the table UI."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Title and buttons
        header_layout = QtWidgets.QHBoxLayout()
        title_label = QtWidgets.QLabel(self._title)
        title_label.setStyleSheet("font-weight: bold;")
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        
        # Action buttons
        self.add_btn = QtWidgets.QPushButton("Add")
        self.add_btn.clicked.connect(self._on_add_row)
        self.edit_btn = QtWidgets.QPushButton("Edit")
        self.edit_btn.clicked.connect(self._on_edit_row)
        self.delete_btn = QtWidgets.QPushButton("Delete")
        self.delete_btn.clicked.connect(self._on_delete_row)
        self.import_btn = QtWidgets.QPushButton("Import")
        self.import_btn.clicked.connect(self._on_import_json)
        self.export_btn = QtWidgets.QPushButton("Export")
        self.export_btn.clicked.connect(self._on_export_json)
        
        header_layout.addWidget(self.add_btn)
        header_layout.addWidget(self.edit_btn)
        header_layout.addWidget(self.delete_btn)
        header_layout.addWidget(self.import_btn)
        header_layout.addWidget(self.export_btn)
        
        layout.addLayout(header_layout)
        
        # Table - shows summary info
        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels([
            "ID", "Label", "Points", "Timestamp"
        ])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.doubleClicked.connect(self._on_row_double_clicked)
        layout.addWidget(self.table)
    
    def _on_add_row(self):
        """Add a new row to the table."""
        dialog = AntennaRecordDialog(self, record_type=self._record_type)
        if dialog.exec() == QtWidgets.QDialog.Accepted:
            record = dialog.get_record()
            self._records.append(record)
            self._refresh_table()
            self.records_changed.emit()
    
    def _on_edit_row(self):
        """Edit the selected row."""
        current_row = self.table.currentRow()
        if current_row < 0 or current_row >= len(self._records):
            QtWidgets.QMessageBox.warning(self, "Edit", "Please select a row to edit.")
            return
        
        record = self._records[current_row]
        dialog = AntennaRecordDialog(self, record, self._record_type)
        if dialog.exec() == QtWidgets.QDialog.Accepted:
            self._records[current_row] = dialog.get_record()
            self._refresh_table()
            self.records_changed.emit()
    
    def _on_delete_row(self):
        """Delete the selected row."""
        current_row = self.table.currentRow()
        if current_row < 0 or current_row >= len(self._records):
            QtWidgets.QMessageBox.warning(self, "Delete", "Please select a row to delete.")
            return
        
        reply = QtWidgets.QMessageBox.question(
            self, "Delete", "Are you sure you want to delete this record?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )
        if reply == QtWidgets.QMessageBox.Yes:
            del self._records[current_row]
            self._refresh_table()
            self.records_changed.emit()
    
    def _on_import_json(self):
        """Import records from JSON file."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import JSON", "", "JSON Files (*.json);;All Files (*.*)"
        )
        if not path:
            return
        
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            if isinstance(data, list):
                self._records = data
            elif isinstance(data, dict) and "records" in data:
                self._records = data["records"]
            else:
                raise ValueError("Invalid JSON format")
            
            self._refresh_table()
            self.records_changed.emit()
            QtWidgets.QMessageBox.information(
                self, "Import", f"Imported {len(self._records)} records."
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Import Error", f"Failed to import:\n{e}")
    
    def _on_export_json(self):
        """Export records to JSON file."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export JSON", "", "JSON Files (*.json);;All Files (*.*)"
        )
        if not path:
            return
        
        try:
            export_data = {
                "type": f"antenna_{self._record_type}_measurements",
                "exported_at": datetime.datetime.now().isoformat(),
                "records": self._records
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(export_data, f, indent=2)
            QtWidgets.QMessageBox.information(
                self, "Export", f"Exported {len(self._records)} records."
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Export Error", f"Failed to export:\n{e}")
    
    def _on_row_double_clicked(self, index):
        """Handle row double-click."""
        row = index.row()
        if 0 <= row < len(self._records):
            self.record_selected.emit(self._records[row])
    
    def _refresh_table(self):
        """Refresh the table display."""
        self.table.setRowCount(0)
        for record in self._records:
            row = self.table.rowCount()
            self.table.insertRow(row)
            
            # ID
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(
                str(record.get("id", ""))
            ))
            # Label
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(
                str(record.get("label", ""))
            ))
            # Points count
            measurements = record.get("measurements", [])
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(
                str(len(measurements))
            ))
            # Timestamp
            ts = record.get("timestamp", "")
            if ts:
                try:
                    dt = datetime.datetime.fromisoformat(ts)
                    ts = dt.strftime("%Y-%m-%d %H:%M")
                except Exception:
                    pass
            self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(str(ts)))
    
    def add_record(self, record: Dict):
        """Add a record programmatically."""
        self._records.append(record)
        self._refresh_table()
        self.records_changed.emit()
    
    def get_records(self) -> List[Dict]:
        """Get all records."""
        return self._records.copy()
    
    def get_selected_record(self) -> Optional[Dict]:
        """Get currently selected record."""
        current_row = self.table.currentRow()
        if 0 <= current_row < len(self._records):
            return self._records[current_row]
        return None
    
    def clear_records(self):
        """Clear all records."""
        self._records.clear()
        self._refresh_table()
        self.records_changed.emit()


class GainMeasureTab(QtWidgets.QWidget):
    """Gain measurement results tab with three-column layout."""
    
    # Signal emitted when a record is selected for plotting
    plot_requested = Signal(dict)  # Emits record with gain data
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._selected_reference: Optional[Dict] = None
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the gain measure tab UI."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        
        # Three-column layout
        columns_layout = QtWidgets.QHBoxLayout()
        
        # Left: Reference antenna measurements
        self.reference_table = AntennaTableWidget(
            "Reference Antenna", 
            record_type="reference",
            parent=self
        )
        self.reference_table.record_selected.connect(self._on_reference_selected)
        columns_layout.addWidget(self.reference_table, 1)
        
        # Middle: Golden samples (antennas meeting threshold)
        self.golden_table = AntennaTableWidget(
            "Golden Samples",
            record_type="golden",
            parent=self
        )
        self.golden_table.record_selected.connect(self._on_golden_selected)
        columns_layout.addWidget(self.golden_table, 1)
        
        # Right: Tested antennas (excluding golden samples)
        self.tested_table = AntennaTableWidget(
            "Tested Antennas",
            record_type="tested",
            parent=self
        )
        self.tested_table.record_selected.connect(self._on_tested_selected)
        columns_layout.addWidget(self.tested_table, 1)
        
        layout.addLayout(columns_layout)
    
    def _on_reference_selected(self, record: Dict):
        """Handle reference antenna selection."""
        self._selected_reference = record
        self._emit_plot_data(record, "Reference")
    
    def _on_golden_selected(self, record: Dict):
        """Handle golden sample selection."""
        self._emit_plot_data(record, "Golden")
    
    def _on_tested_selected(self, record: Dict):
        """Handle tested antenna selection."""
        self._emit_plot_data(record, "Tested")
    
    def _emit_plot_data(self, record: Dict, source: str):
        """Emit plot data for the selected record."""
        plot_data = {
            "source": source,
            "record": record,
            "label": record.get("label", "Unknown"),
            "measurements": record.get("measurements", []),
            "gain_data": record.get("gain_data", []),  # Calculated gain
        }
        self.plot_requested.emit(plot_data)
    
    def get_selected_reference(self) -> Optional[Dict]:
        """Get the currently selected reference antenna."""
        return self._selected_reference
    
    def add_reference_measurement(self, record: Dict):
        """Add a reference antenna measurement."""
        self.reference_table.add_record(record)
    
    def add_tested_measurement(self, record: Dict, is_golden: bool = False):
        """Add a tested antenna measurement."""
        if is_golden:
            self.golden_table.add_record(record)
        else:
            self.tested_table.add_record(record)
    
    def calculate_gain(self, target_measurements: List[Dict],
                       reference_record: Dict) -> List[Dict]:
        """
        Calculate antenna gain using the formula:
        Antenna Gain = (Antenna SS - Reference SS) + Reference Gain
        
        Args:
            target_measurements: List of {frequency_hz, power_dbm} for target
            reference_record: Reference antenna record with measurements
            
        Returns:
            List of {frequency_hz, gain_db} calculated gain values
        """
        ref_measurements = reference_record.get("measurements", [])
        ref_gain_dbi = float(reference_record.get("reference_gain_dbi", 0.0))
        
        # Build lookup for reference measurements
        ref_lookup = {}
        for m in ref_measurements:
            freq = m.get("frequency_hz")
            if freq is not None:
                ref_lookup[float(freq)] = float(m.get("power_dbm", 0.0))
        
        gain_data = []
        for m in target_measurements:
            freq = m.get("frequency_hz")
            target_power = m.get("power_dbm")
            
            if freq is None or target_power is None:
                continue
            
            freq = float(freq)
            target_power = float(target_power)
            
            # Find matching reference measurement
            ref_power = ref_lookup.get(freq)
            if ref_power is not None:
                # Gain = (Target SS - Reference SS) + Reference Gain
                gain_db = (target_power - ref_power) + ref_gain_dbi
                gain_data.append({
                    "frequency_hz": freq,
                    "gain_db": gain_db
                })
        
        return gain_data
    
    def clear_all(self):
        """Clear all tables."""
        self.reference_table.clear_records()
        self.golden_table.clear_records()
        self.tested_table.clear_records()
        self._selected_reference = None
