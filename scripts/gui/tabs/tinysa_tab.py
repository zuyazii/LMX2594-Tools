"""
tinySA tab - displays spectrum analyzer results.
"""
from __future__ import annotations

from typing import Optional

try:
    from PySide6 import QtWidgets
except ImportError:
    from PyQt5 import QtWidgets


class TinySATab(QtWidgets.QWidget):
    """tinySA results tab."""
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the tinySA tab UI."""
        layout = QtWidgets.QVBoxLayout(self)
        
        # Results table
        self.results_table = QtWidgets.QTableWidget(0, 3)
        self.results_table.setHorizontalHeaderLabels([
            "Frequency (Hz)", "Power (dBm)", "Timestamp"
        ])
        layout.addWidget(self.results_table)
