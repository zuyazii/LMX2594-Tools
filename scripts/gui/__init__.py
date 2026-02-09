#!/usr/bin/env python3
"""
Modern GUI package for LMX2594 calibration.
Dark theme, clean layout with sidebar, toolbar, and dynamic plots.
"""
from __future__ import annotations

__version__ = "2.0.0"

def main() -> int:
    """Main entry point for the GUI application."""
    import sys
    from gui.main_window import CalibrationApp
    
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        from PyQt5.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    app.setApplicationName("LMX2594 Calibration")
    app.setOrganizationName("LMX2594")
    
    window = CalibrationApp()
    window.show()
    
    return app.exec()

if __name__ == "__main__":
    raise SystemExit(main())
