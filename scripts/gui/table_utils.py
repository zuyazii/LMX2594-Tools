"""Shared QTableWidget helpers for scrollable tables on narrow / tablet layouts."""
from __future__ import annotations

try:
    from PySide6 import QtCore, QtWidgets
except ImportError:
    from PyQt5 import QtCore, QtWidgets


def configure_table_horizontal_scroll(table: QtWidgets.QTableWidget, min_section_width: int = 72) -> None:
    """Use interactive column widths and horizontal scrollbar instead of squashing columns."""
    table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
    table.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
    table.setWordWrap(False)
    hdr = table.horizontalHeader()
    hdr.setStretchLastSection(False)
    hdr.setMinimumSectionSize(min_section_width)
    for col in range(table.columnCount()):
        hdr.setSectionResizeMode(col, QtWidgets.QHeaderView.Interactive)
    table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
