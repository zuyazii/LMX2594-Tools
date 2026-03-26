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
    from PySide6 import QtCore, QtGui, QtWidgets
    Signal = QtCore.Signal
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets
    Signal = QtCore.pyqtSignal


def format_tested_worst_summary(record: Dict) -> str:
    """One-line worst deviation for tested-antenna table."""
    fr = record.get("failure_report") or {}
    if fr.get("error") == "no_s11_data":
        return "No S11 data"
    parts: List[str] = []
    st = fr.get("s11_threshold")
    if isinstance(st, dict) and st.get("worst"):
        w = st["worst"]
        parts.append(
            f"S11 max +{float(w.get('excess_db', 0)):.1f} dB @ {float(w.get('frequency_hz', 0)) / 1e6:.0f} MHz"
        )
    tol = fr.get("s11_tolerance")
    if isinstance(tol, dict) and tol.get("worst"):
        w = tol["worst"]
        parts.append(
            f"S11 Δ{float(w.get('delta_db', 0)):.1f} dB @ {float(w.get('frequency_mhz', 0)):.0f} MHz"
        )
    g = fr.get("gain_tolerance")
    if isinstance(g, dict) and g.get("worst"):
        w = g["worst"]
        parts.append(
            f"Gain Δ{float(w.get('delta_db', 0)):.1f} dB @ {float(w.get('frequency_mhz', 0)):.0f} MHz"
        )
    return "; ".join(parts) if parts else "-"


class AntennaFailureReportDialog(QtWidgets.QDialog):
    """Table of all failed points with copy to clipboard."""

    def __init__(self, parent: Optional[QtWidgets.QWidget], record: Dict):
        super().__init__(parent)
        self._record = record
        self.setWindowTitle(f"Failure details — {record.get('label', '')}")
        self.resize(780, 460)
        layout = QtWidgets.QVBoxLayout(self)

        self._table = QtWidgets.QTableWidget(0, 6)
        self._table.setHorizontalHeaderLabels(
            ["Category", "F (MHz)", "Measured", "Reference / limit", "|Δ| / excess", "Notes"]
        )
        self._table.horizontalHeader().setStretchLastSection(True)

        fr = record.get("failure_report") or {}
        rows: List[tuple] = []
        if fr.get("error") == "no_s11_data":
            rows.append(("S11 data", "", "", "", "", "No S11 trace received"))
        st = fr.get("s11_threshold")
        if isinstance(st, dict):
            for p in st.get("failed_points") or []:
                rows.append(
                    (
                        "S11 vs max",
                        f"{float(p.get('frequency_mhz', 0)):.3f}",
                        f"{float(p.get('s11_db', 0)):.2f} dB",
                        f"< {float(st.get('limit_db', 0)):.2f} dB",
                        f"+{float(p.get('excess_db', 0)):.2f} dB",
                        "above threshold",
                    )
                )
        tol = fr.get("s11_tolerance")
        if isinstance(tol, dict):
            tdb = float(tol.get("tolerance_db", 0))
            for p in tol.get("failed_points") or []:
                rows.append(
                    (
                        "S11 vs golden",
                        f"{float(p.get('frequency_mhz', 0)):.3f}",
                        f"{float(p.get('measured_db', 0)):.2f} dB",
                        f"{float(p.get('golden_db', 0)):.2f} dB (golden)",
                        f"{float(p.get('delta_db', 0)):.2f} dB",
                        f"limit ±{tdb:.2f} dB",
                    )
                )
        g = fr.get("gain_tolerance")
        if isinstance(g, dict):
            gdb = float(g.get("tolerance_db", 0))
            for p in g.get("failed_points") or []:
                rows.append(
                    (
                        "Gain vs golden",
                        f"{float(p.get('frequency_mhz', 0)):.3f}",
                        f"{float(p.get('measured_dbm', 0)):.2f} dBm",
                        f"{float(p.get('golden_dbm', 0)):.2f} dBm (golden)",
                        f"{float(p.get('delta_db', 0)):.2f} dB",
                        f"limit ±{gdb:.2f} dB",
                    )
                )
        if not rows and record.get("fail_reason"):
            rows.append(
                ("Summary", "", "", "", "", str(record.get("fail_reason"))),
            )

        self._table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            for c, val in enumerate(row):
                self._table.setItem(r, c, QtWidgets.QTableWidgetItem(str(val)))
        layout.addWidget(self._table)

        btn_row = QtWidgets.QHBoxLayout()
        copy_txt = QtWidgets.QPushButton("Copy as text")
        copy_json = QtWidgets.QPushButton("Copy as JSON")
        close_btn = QtWidgets.QPushButton("Close")
        copy_txt.clicked.connect(self._copy_text)
        copy_json.clicked.connect(self._copy_json)
        close_btn.clicked.connect(self.accept)
        btn_row.addWidget(copy_txt)
        btn_row.addWidget(copy_json)
        btn_row.addStretch()
        btn_row.addWidget(close_btn)
        layout.addLayout(btn_row)

    def _copy_text(self) -> None:
        hdr = []
        for c in range(self._table.columnCount()):
            it = self._table.horizontalHeaderItem(c)
            hdr.append(it.text() if it else "")
        lines = ["\t".join(hdr)]
        for r in range(self._table.rowCount()):
            cells = []
            for c in range(self._table.columnCount()):
                it = self._table.item(r, c)
                cells.append(it.text() if it else "")
            lines.append("\t".join(cells))
        QtWidgets.QApplication.clipboard().setText("\n".join(lines))

    def _copy_json(self) -> None:
        payload = {
            "label": self._record.get("label"),
            "fail_reason": self._record.get("fail_reason"),
            "failure_report": self._record.get("failure_report"),
            "thresholds": self._record.get("thresholds"),
        }
        QtWidgets.QApplication.clipboard().setText(json.dumps(payload, indent=2))


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
        
        # Load existing measurements (from measurements or gain_data.points for tested)
        measurements = self._record.get("measurements", [])
        if not measurements and self._record_type == "tested":
            gain_data = self._record.get("gain_data", {})
            if isinstance(gain_data, dict):
                measurements = gain_data.get("points", [])
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
        # Different columns for tested antennas (shows S11 and Gain results)
        if self._record_type == "tested":
            self.table = QtWidgets.QTableWidget(0, 9)
            self.table.setHorizontalHeaderLabels([
                "ID",
                "Label",
                "S11 (dB)",
                "VSWR",
                "Gain (dB)",
                "Worst deviation",
                "Status",
                "Details",
                "Timestamp",
            ])
        else:
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

        # Different display for tested antennas
        if self._record_type == "tested":
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

                # S11 data
                s11_data = record.get("s11_data", {})
                if s11_data and isinstance(s11_data, dict):
                    worst_s11 = s11_data.get("worst_s11_db")
                    if worst_s11 is not None:
                        s11_text = f"{worst_s11:.2f}"
                    else:
                        s11_text = "-"
                else:
                    s11_text = "-"
                self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(s11_text))

                # VSWR
                if s11_data and isinstance(s11_data, dict):
                    vswr = s11_data.get("vswr")
                    if vswr is not None and vswr != float('inf'):
                        vswr_text = f"{vswr:.2f}"
                    else:
                        vswr_text = "-"
                else:
                    vswr_text = "-"
                self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(vswr_text))

                # Gain data - calculate from measurements
                gain_data = record.get("gain_data", {})
                if gain_data and isinstance(gain_data, dict):
                    points = gain_data.get("points", [])
                    if points:
                        powers = [float(p.get("power_dbm", -100)) for p in points if isinstance(p, dict)]
                        if powers:
                            # Use max power as gain proxy
                            max_gain = max(powers)
                            gain_text = f"{max_gain:.2f}"
                        else:
                            gain_text = "-"
                    else:
                        gain_text = "-"
                else:
                    gain_text = "-"
                self.table.setItem(row, 4, QtWidgets.QTableWidgetItem(gain_text))

                # Worst deviation summary (from failure_report when present)
                self.table.setItem(
                    row, 5, QtWidgets.QTableWidgetItem(format_tested_worst_summary(record))
                )

                # Status
                s11_pass = record.get("s11_pass")
                gain_pass = record.get("gain_pass")
                overall_pass = record.get("overall_pass")
                fail_reason = record.get("fail_reason", "")

                if overall_pass is True:
                    status_text = "PASS"
                elif s11_pass is False:
                    fr = (fail_reason or "").lower()
                    if "variation" in fr or "vs golden" in fr:
                        status_text = "FAIL (S11 vs golden)"
                    elif "threshold" in fr or "above" in fr:
                        status_text = "FAIL (S11 max)"
                    elif "no s11" in fr:
                        status_text = "FAIL (S11 data)"
                    else:
                        status_text = "FAIL (S11)"
                elif gain_pass is False:
                    status_text = f"FAIL (Gain)"
                else:
                    status_text = "-"

                status_item = QtWidgets.QTableWidgetItem(status_text)
                if "FAIL" in status_text:
                    status_item.setForeground(QtGui.QColor("#c1121f"))
                elif "PASS" in status_text:
                    status_item.setForeground(QtGui.QColor("#0a7f2e"))
                self.table.setItem(row, 6, status_item)

                # Fail details button
                detail_wrap = QtWidgets.QWidget()
                detail_layout = QtWidgets.QHBoxLayout(detail_wrap)
                detail_layout.setContentsMargins(2, 2, 2, 2)
                detail_btn = QtWidgets.QPushButton("View…")
                fr = record.get("failure_report") or {}
                has_rows = bool(
                    fr.get("s11_threshold")
                    or fr.get("s11_tolerance")
                    or fr.get("gain_tolerance")
                    or fr.get("error")
                )
                detail_btn.setEnabled(
                    has_rows or (overall_pass is not True and bool(record.get("fail_reason")))
                )
                rec_copy = dict(record)
                detail_btn.clicked.connect(
                    lambda _=False, rc=rec_copy: AntennaFailureReportDialog(self, rc).exec()
                )
                detail_layout.addWidget(detail_btn)
                self.table.setCellWidget(row, 7, detail_wrap)

                # Timestamp
                ts = record.get("timestamp", "")
                if ts:
                    try:
                        dt = datetime.datetime.fromisoformat(ts)
                        ts = dt.strftime("%Y-%m-%d %H:%M")
                    except Exception:
                        pass
                self.table.setItem(row, 8, QtWidgets.QTableWidgetItem(str(ts)))
        else:
            # Standard display for reference and golden samples
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
                # Points count (merged golden = S11 + gain counts)
                s11_data = record.get("s11_data", {})
                n_s11 = len(s11_data.get("frequencies", [])) if isinstance(s11_data, dict) else 0
                n_g = len(record.get("measurements", []))
                if n_s11 and n_g:
                    pts_text = f"{n_s11}+{n_g}"
                elif n_s11:
                    pts_text = str(n_s11)
                else:
                    pts_text = str(n_g)
                self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(pts_text))
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
        self._antenna_counter = 0
        self._config_key = "antenna_test_tab"
        self._selected_golden_record: Optional[Dict] = None
        self._selected_tested_record: Optional[Dict] = None
        self._display_mode = "golden"  # "golden", "tested", or "both"
        self._lock_display_mode = False  # When True, radio buttons are disabled
        self._auto_mode = True  # Tracks whether current mode was auto-set
        self._setup_ui()

    def _setup_ui(self):
        """Setup the gain measure tab UI."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Two-column layout using splitter for adjustable widths
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # Left: Golden samples (captured by pressing "Measure Golden Sample")
        self.golden_table = AntennaTableWidget(
            "Golden Samples",
            record_type="golden",
            parent=self
        )
        self.golden_table.record_selected.connect(self._on_golden_selected)
        self.splitter.addWidget(self.golden_table)

        # Right: Tested Antennas (with subcolumns for LibreVNA S11 and tinySA Gain)
        self.tested_table = AntennaTableWidget(
            "Tested Antennas",
            record_type="tested",
            parent=self
        )
        self.tested_table.record_selected.connect(self._on_tested_selected)
        self.splitter.addWidget(self.tested_table)

        # Set stretch factors for equal initial size
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 1)

        # Display mode selection
        mode_layout = QtWidgets.QHBoxLayout()
        mode_label = QtWidgets.QLabel("Display:")
        mode_label.setStyleSheet("font-weight: bold;")
        mode_layout.addWidget(mode_label)

        self.golden_radio = QtWidgets.QRadioButton("Golden Reference Only")
        self.golden_radio.setChecked(True)
        self.golden_radio.toggled.connect(self._on_display_mode_changed)
        mode_layout.addWidget(self.golden_radio)

        self.tested_radio = QtWidgets.QRadioButton("Test Antenna Only")
        self.tested_radio.toggled.connect(self._on_display_mode_changed)
        mode_layout.addWidget(self.tested_radio)

        self.both_radio = QtWidgets.QRadioButton("Show Both")
        self.both_radio.toggled.connect(self._on_display_mode_changed)
        mode_layout.addWidget(self.both_radio)

        mode_layout.addStretch()
        layout.addLayout(mode_layout)

        layout.addWidget(self.splitter)

    def save_state(self, store) -> dict:
        """Save splitter state to config."""
        widths = self.splitter.sizes()
        # Check if store is a QSettings object (has setValue method)
        if hasattr(store, 'setValue'):
            store.setValue(f"{self._config_key}/splitter_widths", widths)
        return {"splitter_widths": widths}

    def restore_state(self, store):
        """Restore splitter state from config.

        Args:
            store: Either a QSettings object or a dict from session data
        """
        # Check if store is a QSettings object or a dict
        if isinstance(store, dict):
            widths = store.get("splitter_widths")
        else:
            widths = store.value(f"{self._config_key}/splitter_widths")

        if widths and isinstance(widths, (list, tuple)) and len(widths) == 2:
            self.splitter.setSizes([int(w) for w in widths])
    
    def _update_radio_enabled_state(self):
        """Enable/disable radio buttons based on selection state."""
        has_golden = self._selected_golden_record is not None
        has_tested = self._selected_tested_record is not None

        if has_golden and has_tested:
            # Both selected - unlock all radios
            self._lock_display_mode = False
            self.golden_radio.setEnabled(True)
            self.tested_radio.setEnabled(True)
            self.both_radio.setEnabled(True)
        elif has_golden and not has_tested:
            # Only golden - lock to golden only
            self._lock_display_mode = True
            self.golden_radio.setEnabled(True)
            self.tested_radio.setEnabled(False)
            self.both_radio.setEnabled(False)
            self.golden_radio.setChecked(True)
            self._display_mode = "golden"
        elif has_tested and not has_golden:
            # Only tested - lock to tested only
            self._lock_display_mode = True
            self.golden_radio.setEnabled(False)
            self.tested_radio.setEnabled(True)
            self.both_radio.setEnabled(False)
            self.tested_radio.setChecked(True)
            self._display_mode = "tested"
        else:
            # Nothing selected - unlock all
            self._lock_display_mode = False
            self.golden_radio.setEnabled(True)
            self.tested_radio.setEnabled(True)
            self.both_radio.setEnabled(True)

    def _on_golden_selected(self, record: Dict):
        """Handle golden sample selection."""
        self._selected_golden_record = record
        self._update_radio_enabled_state()
        self._update_plot_display()

    def _on_tested_selected(self, record: Dict):
        """Handle tested antenna selection."""
        self._selected_tested_record = record
        self._update_radio_enabled_state()
        self._update_plot_display()

    def _on_display_mode_changed(self):
        """Handle display mode radio button change."""
        if self._lock_display_mode:
            # Revert to the locked selection if user tried to change
            self._update_radio_enabled_state()
            return

        if self.golden_radio.isChecked():
            self._display_mode = "golden"
        elif self.tested_radio.isChecked():
            self._display_mode = "tested"
        elif self.both_radio.isChecked():
            self._display_mode = "both"
        self._update_plot_display()

    @staticmethod
    def _record_measurement_type(record: Dict) -> str:
        """Determine the measurement type of a record.

        Returns:
            "gain_only": Only tinySA gain measurements present
            "sparam_only": Only LibreVNA S-parameters present
            "both": Both gain and S-parameters present
            "none": No recognized measurement data
        """
        if not record:
            return "none"

        measurements = record.get("measurements", [])
        gain_data = record.get("gain_data", {})
        s11_data = record.get("s11_data", {})

        has_gain = bool(measurements or (isinstance(gain_data, dict) and gain_data.get("points")))
        has_s11 = bool(
            isinstance(s11_data, dict)
            and s11_data.get("magnitudes")
        )

        if has_gain and has_s11:
            return "both"
        elif has_gain:
            return "gain_only"
        elif has_s11:
            return "sparam_only"
        return "none"

    def _update_plot_display(self):
        """Emit plot data based on current display mode and selections."""
        if self._display_mode == "golden" and self._selected_golden_record:
            self._emit_plot_data(self._selected_golden_record, "golden")
        elif self._display_mode == "tested" and self._selected_tested_record:
            self._emit_plot_data(self._selected_tested_record, "tested")
        elif self._display_mode == "both":
            golden = self._selected_golden_record
            tested = self._selected_tested_record
            if golden or tested:
                self._emit_comparison_plot(golden, tested)

    def _emit_comparison_plot(self, golden: Optional[Dict], tested: Optional[Dict]):
        """Emit comparison plot data for golden vs tested."""
        # Determine measurement type for layout
        golden_type = self._record_measurement_type(golden) if golden else "none"
        tested_type = self._record_measurement_type(tested) if tested else "none"

        # Overall type: use the richer type available
        combined_types = {golden_type, tested_type}
        if "both" in combined_types:
            meas_type = "both"
        elif "gain_only" in combined_types and "sparam_only" in combined_types:
            meas_type = "both"
        elif "gain_only" in combined_types:
            meas_type = "gain_only"
        elif "sparam_only" in combined_types:
            meas_type = "sparam_only"
        else:
            meas_type = "none"

        plot_data = {
            "mode": "comparison",
            "meas_type": meas_type,
            "golden": golden,
            "tested": tested,
            "golden_label": golden.get("label", "Golden") if golden else "None",
            "tested_label": tested.get("label", "Tested") if tested else "None",
        }
        self.plot_requested.emit(plot_data)

    def _emit_plot_data(self, record: Dict, source: str):
        """Emit plot data for the selected record."""
        meas_type = self._record_measurement_type(record)
        plot_data = {
            "mode": "single",
            "source": source,
            "meas_type": meas_type,
            "record": record,
            "label": record.get("label", "Unknown"),
            "measurements": record.get("measurements", []),
            "gain_data": record.get("gain_data", []),
            "s11_data": record.get("s11_data", {}),
        }
        self.plot_requested.emit(plot_data)
    
    def add_golden_sample(self, record: Dict):
        """Add a golden sample measurement."""
        self.golden_table.add_record(record)
    
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
        self.golden_table.clear_records()
        self.tested_table.clear_records()
        self._antenna_counter = 0

    def get_next_antenna_label(self) -> str:
        """Get the next antenna label for testing."""
        self._antenna_counter += 1
        return f"Antenna_{self._antenna_counter:03d}"

    def get_display_mode(self) -> str:
        """Get current display mode."""
        return self._display_mode

    def get_selected_golden(self) -> Optional[Dict]:
        """Get currently selected golden record."""
        return self._selected_golden_record

    def get_selected_tested(self) -> Optional[Dict]:
        """Get currently selected tested record."""
        return self._selected_tested_record
