"""
Logs tab - displays event log and IPC log.
"""
from __future__ import annotations

from typing import Optional

try:
    from PySide6 import QtWidgets
except ImportError:
    from PyQt5 import QtWidgets


class LogsTab(QtWidgets.QWidget):
    """Logs tab with event log and IPC log."""
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the logs tab UI."""
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        
        # Event log
        event_group = QtWidgets.QGroupBox("Event Log")
        event_layout = QtWidgets.QVBoxLayout(event_group)
        self.event_log = QtWidgets.QPlainTextEdit()
        self.event_log.setReadOnly(True)
        event_layout.addWidget(self.event_log)
        layout.addWidget(event_group)
        
        # IPC log
        ipc_group = QtWidgets.QGroupBox("IPC Log")
        ipc_layout = QtWidgets.QVBoxLayout(ipc_group)
        self.ipc_log = QtWidgets.QPlainTextEdit()
        self.ipc_log.setReadOnly(True)
        ipc_layout.addWidget(self.ipc_log)
        layout.addWidget(ipc_group)
    
    def append_event(self, message: str):
        """Append message to event log."""
        self.event_log.appendPlainText(message)
    
    def append_ipc(self, message: str):
        """Append message to IPC log."""
        self.ipc_log.appendPlainText(message)
