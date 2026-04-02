"""
Dark theme stylesheet for the LMX2594 calibration GUI.
Modern, minimalistic design inspired by VS Code.
"""

# Color palette
COLORS = {
    "background": "#1e1e1e",
    "sidebar": "#252526",
    "panel": "#2d2d30",
    "input": "#3c3c3c",
    "border": "#3c3c3c",
    "text": "#cccccc",
    "text_dim": "#808080",
    "accent": "#0078d4",
    "accent_hover": "#1a86d9",
    "accent2": "#4ec9b0",  # Secondary color for comparison plots
    "success": "#4ec9b0",
    "error": "#f14c4c",
    "warning": "#cca700",
    "selection": "#264f78",
}


def get_stylesheet(is_tablet: bool = False) -> str:
    """Return the complete dark theme stylesheet.

    Args:
        is_tablet: When True, enlarges touch targets and fonts for tablet/touch use.
    """
    # Tablet overrides
    btn_pad = "10px 20px" if is_tablet else "6px 16px"
    inp_pad = "8px 12px" if is_tablet else "4px 8px"
    font_sz = "11pt" if is_tablet else "9pt"
    tab_pad = "10px" if is_tablet else "8px"
    min_h = "36px" if is_tablet else ""
    cb_sz = "20px" if is_tablet else "16px"
    cb_rad = "4px" if is_tablet else "2px"

    return f"""
    /* Main Window */
    QMainWindow {{
        background-color: {COLORS['background']};
        color: {COLORS['text']};
        font-size: {font_sz};
    }}

    QDialog {{
        background-color: {COLORS['background']};
        color: {COLORS['text']};
        font-size: {font_sz};
    }}

    /* Menu Bar */
    QMenuBar {{
        background-color: {COLORS['panel']};
        color: {COLORS['text']};
        border-bottom: 1px solid {COLORS['border']};
        padding: 2px;
    }}

    QMenuBar::item {{
        background-color: transparent;
        padding: 4px 12px;
    }}

    QMenuBar::item:selected {{
        background-color: {COLORS['selection']};
    }}

    QMenuBar::item:pressed {{
        background-color: {COLORS['accent']};
    }}

    /* Menu */
    QMenu {{
        background-color: {COLORS['panel']};
        color: {COLORS['text']};
        border: 1px solid {COLORS['border']};
    }}

    QMenu::item {{
        padding: 6px 24px 6px 12px;
    }}

    QMenu::item:selected {{
        background-color: {COLORS['selection']};
    }}

    QMenu::separator {{
        height: 1px;
        background-color: {COLORS['border']};
        margin: 4px 0px;
    }}

    /* Toolbar */
    QToolBar {{
        background-color: {COLORS['panel']};
        border-bottom: 1px solid {COLORS['border']};
        spacing: 8px;
        padding: 4px;
    }}

    QToolBar::separator {{
        background-color: {COLORS['border']};
        width: 1px;
        margin: 4px 8px;
    }}

    /* Buttons */
    QPushButton {{
        background-color: {COLORS['input']};
        color: {COLORS['text']};
        border: 1px solid {COLORS['border']};
        padding: {btn_pad};
        border-radius: 4px;
        min-height: {min_h};
    }}

    QPushButton:hover {{
        background-color: {COLORS['accent']};
        border-color: {COLORS['accent']};
    }}

    QPushButton:pressed {{
        background-color: {COLORS['accent_hover']};
    }}

    QPushButton:disabled {{
        background-color: {COLORS['panel']};
        color: {COLORS['text_dim']};
    }}

    /* Input Fields */
    QLineEdit, QSpinBox, QDoubleSpinBox {{
        background-color: {COLORS['input']};
        color: {COLORS['text']};
        border: 1px solid {COLORS['border']};
        padding: {inp_pad};
        border-radius: 4px;
        min-height: {min_h};
    }}

    QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
        border-color: {COLORS['accent']};
    }}

    QTextEdit, QPlainTextEdit {{
        background-color: {COLORS['panel']};
        color: {COLORS['text']};
        border: 1px solid {COLORS['border']};
        selection-background-color: {COLORS['selection']};
    }}

    /* ComboBox */
    QComboBox {{
        background-color: {COLORS['input']};
        color: {COLORS['text']};
        border: 1px solid {COLORS['border']};
        padding: {inp_pad};
        border-radius: 4px;
        min-height: {min_h};
    }}

    QComboBox:hover {{
        border-color: {COLORS['accent']};
    }}

    QComboBox::drop-down {{
        border: none;
        width: 20px;
    }}

    QComboBox QAbstractItemView {{
        background-color: {COLORS['panel']};
        color: {COLORS['text']};
        selection-background-color: {COLORS['selection']};
        border: 1px solid {COLORS['border']};
    }}

    /* Labels */
    QLabel {{
        color: {COLORS['text']};
        background-color: transparent;
    }}

    /* GroupBox */
    QGroupBox {{
        color: {COLORS['text']};
        border: 1px solid {COLORS['border']};
        border-radius: 4px;
        margin-top: 8px;
        padding-top: 8px;
    }}

    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        padding: 0 4px;
        color: {COLORS['text']};
    }}

    /* TabWidget */
    QTabWidget::pane {{
        border: 1px solid {COLORS['border']};
        background-color: {COLORS['panel']};
    }}

    QTabBar::tab {{
        background-color: {COLORS['panel']};
        color: {COLORS['text_dim']};
        border: 1px solid {COLORS['border']};
        padding: {tab_pad} 16px;
        margin-right: 2px;
    }}

    QTabBar::tab:selected {{
        background-color: {COLORS['background']};
        color: {COLORS['text']};
        border-bottom-color: {COLORS['background']};
    }}

    QTabBar::tab:hover {{
        color: {COLORS['text']};
    }}

    /* Tables */
    QTableWidget {{
        background-color: {COLORS['background']};
        color: {COLORS['text']};
        gridline-color: {COLORS['border']};
        border: 1px solid {COLORS['border']};
    }}

    QTableWidget::item {{
        padding: 4px;
    }}

    QTableWidget::item:selected {{
        background-color: {COLORS['selection']};
    }}

    QHeaderView::section {{
        background-color: {COLORS['panel']};
        color: {COLORS['text']};
        padding: 6px;
        border: 1px solid {COLORS['border']};
    }}

    /* Scrollbars */
    QScrollBar:vertical {{
        background-color: {COLORS['background']};
        width: 12px;
        margin: 0px;
    }}

    QScrollBar::handle:vertical {{
        background-color: {COLORS['input']};
        min-height: 20px;
        border-radius: 6px;
    }}

    QScrollBar::handle:vertical:hover {{
        background-color: {COLORS['border']};
    }}

    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0px;
    }}

    /* CheckBox */
    QCheckBox {{
        color: {COLORS['text']};
        spacing: 8px;
    }}

    QCheckBox::indicator {{
        width: {cb_sz};
        height: {cb_sz};
        border: 1px solid {COLORS['border']};
        border-radius: {cb_rad};
        background-color: {COLORS['input']};
    }}

    QCheckBox::indicator:checked {{
        background-color: {COLORS['accent']};
        border-color: {COLORS['accent']};
    }}

    /* RadioButton */
    QRadioButton {{
        color: {COLORS['text']};
        spacing: 8px;
    }}

    QRadioButton::indicator {{
        width: {cb_sz};
        height: {cb_sz};
        border: 1px solid {COLORS['border']};
        border-radius: {int(cb_sz[:-2]) // 2}px;
        background-color: {COLORS['input']};
    }}

    QRadioButton::indicator:checked {{
        background-color: {COLORS['accent']};
        border-color: {COLORS['accent']};
    }}

    QProgressBar {{
        border: 1px solid {COLORS['border']};
        border-radius: 2px;
        background-color: {COLORS['panel']};
        color: {COLORS['text']};
        text-align: right;
        padding-right: 4px;
    }}

    QProgressBar::chunk {{
        background-color: {COLORS['accent']};
    }}
    """
