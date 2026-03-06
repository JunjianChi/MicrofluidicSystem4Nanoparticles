"""
Fluent / macOS-soft theme for the Microfluidic Control GUI.

Applies a modern, card-based look with soft rounded corners,
Segoe UI typography, and dark chart backgrounds — all on top of
standard Qt widgets (QGroupBox, QPushButton, etc.) for compact sizing.
"""
from __future__ import annotations

from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg


# ── Palette ───────────────────────────────────────────────────────────────

ACCENT       = "#0078D4"
ACCENT_HOVER = "#1A86D9"
ACCENT_PRESS = "#006CBE"

BG_WINDOW    = "#E4E4E4"      # soft gray backdrop
BG_CARD      = "#FFFFFF"
BORDER       = "#CACACA"
BORDER_FOCUS = ACCENT

TEXT_PRI     = "#1A1A1A"
TEXT_SEC     = "#5A5A5A"
TEXT_DIS     = "#A0A0A0"

# Status
GREEN   = "#107C10"
RED     = "#C42B1C"
ORANGE  = "#CA5010"
PURPLE  = "#881798"

# Chart (dark)
BG_CHART   = "#1E1E1E"
TEXT_CHART  = "#D0D0D0"
GRID_CHART  = "#3A3A3A"
CURVE_FLOW  = "#60CDFF"
CURVE_TGT   = "#FF6B6B"
CURVE_PRES  = "#FFB900"


# ── Global stylesheet ─────────────────────────────────────────────────────

_SS = f"""
/* ── window ────────────────────────────────────────────────── */
QMainWindow, QWidget {{
    font-family: "Segoe UI", "SF Pro Display", "Helvetica Neue", sans-serif;
    font-size: 9pt;
    color: {TEXT_PRI};
}}
QMainWindow {{
    background: {BG_WINDOW};
}}

/* ── card-style group box ──────────────────────────────────── */
QGroupBox {{
    background: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 8px;
    margin-top: 10px;
    padding: 8px 10px 6px 10px;
    font-size: 9pt;
    font-weight: 600;
    color: {TEXT_PRI};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 8px;
    background: transparent;
    color: {TEXT_PRI};
    font-size: 9pt;
    font-weight: 600;
}}

/* ── labels ────────────────────────────────────────────────── */
QLabel {{
    color: {TEXT_PRI};
    background: transparent;
    font-size: 9pt;
}}

/* ── push buttons ──────────────────────────────────────────── */
QPushButton {{
    font-family: "Segoe UI", sans-serif;
    font-size: 9pt;
    font-weight: 500;
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #FAFAFA, stop:1 #EFEFEF);
    border: 1px solid {BORDER};
    border-radius: 5px;
    padding: 4px 14px;
    color: {TEXT_PRI};
    min-height: 22px;
}}
QPushButton:hover {{
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #F0F0F0, stop:1 #E4E4E4);
    border-color: #B0B0B0;
}}
QPushButton:pressed {{
    background: #D8D8D8;
    border-color: #A0A0A0;
}}
QPushButton:disabled {{
    background: #F5F5F5;
    color: {TEXT_DIS};
    border-color: #E0E0E0;
}}

/* ── inputs (combo, spin, line edit) ───────────────────────── */
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    font-family: "Segoe UI", sans-serif;
    font-size: 9pt;
    background: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 5px;
    padding: 3px 8px;
    color: {TEXT_PRI};
    min-height: 20px;
    selection-background-color: {ACCENT};
    selection-color: white;
}}
QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover, QLineEdit:hover {{
    border-color: #A0A0A0;
}}
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{
    border: 1.5px solid {ACCENT};
}}
QComboBox::drop-down {{
    border: none;
    width: 22px;
    subcontrol-position: center right;
}}
QComboBox::down-arrow {{
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid {TEXT_SEC};
    margin-right: 6px;
}}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {{
    width: 16px;
    border: none;
    background: transparent;
}}

/* ── slider ────────────────────────────────────────────────── */
QSlider::groove:horizontal {{
    height: 4px;
    background: #D0D0D0;
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    width: 14px;
    height: 14px;
    margin: -5px 0;
    background: {ACCENT};
    border: 2px solid white;
    border-radius: 8px;
}}
QSlider::sub-page:horizontal {{
    background: {ACCENT};
    border-radius: 2px;
}}

/* ── text edit (log area) ──────────────────────────────────── */
QTextEdit {{
    font-family: "Consolas", "SF Mono", monospace;
    font-size: 8pt;
    background: #F6F6F6;
    border: 1px solid {BORDER};
    border-radius: 5px;
    color: {TEXT_PRI};
    padding: 4px;
}}

/* ── tab widget ────────────────────────────────────────────── */
QTabWidget::pane {{
    border: 1px solid {BORDER};
    border-radius: 6px;
    background: {BG_CHART};
    top: -1px;
}}
QTabBar::tab {{
    font-family: "Segoe UI", sans-serif;
    font-size: 9pt;
    background: #F0F0F0;
    border: 1px solid {BORDER};
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 5px 16px;
    color: {TEXT_SEC};
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background: {BG_CARD};
    color: {ACCENT};
    font-weight: 600;
    border-bottom: 2px solid {ACCENT};
}}
QTabBar::tab:hover:!selected {{
    background: #E8E8E8;
}}

/* ── progress bar ──────────────────────────────────────────── */
QProgressBar {{
    border: 1px solid {BORDER};
    border-radius: 4px;
    background: #EBEBEB;
    text-align: center;
    max-height: 14px;
    font-size: 7pt;
    color: {TEXT_SEC};
}}
QProgressBar::chunk {{
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 {ACCENT}, stop:1 {ACCENT_HOVER});
    border-radius: 3px;
}}

/* ── splitter ──────────────────────────────────────────────── */
QSplitter::handle {{
    background: {BORDER};
    width: 1px;
}}
QSplitter::handle:hover {{
    background: {ACCENT};
}}

/* ── scrollbar (thin, macOS-like) ──────────────────────────── */
QScrollBar:vertical {{
    width: 6px;
    background: transparent;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: rgba(0,0,0,0.2);
    border-radius: 3px;
    min-height: 30px;
}}
QScrollBar::handle:vertical:hover {{
    background: rgba(0,0,0,0.35);
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
    height: 0; background: transparent;
}}
QScrollBar:horizontal {{
    height: 6px;
    background: transparent;
}}
QScrollBar::handle:horizontal {{
    background: rgba(0,0,0,0.2);
    border-radius: 3px;
    min-width: 30px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal,
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
    width: 0; background: transparent;
}}
"""


# ── Theme init ────────────────────────────────────────────────────────────

def init_fluent_theme() -> None:
    """Apply the macOS-soft Fluent theme. Call after QApplication + setStyle('Fusion')."""
    app = QApplication.instance()
    if not app:
        return

    app.setFont(QFont("Segoe UI", 9))

    pal = app.palette()
    pal.setColor(QPalette.Window,          QColor(BG_WINDOW))
    pal.setColor(QPalette.WindowText,      QColor(TEXT_PRI))
    pal.setColor(QPalette.Base,            QColor(BG_CARD))
    pal.setColor(QPalette.AlternateBase,   QColor("#F4F4F4"))
    pal.setColor(QPalette.Text,            QColor(TEXT_PRI))
    pal.setColor(QPalette.Button,          QColor("#F0F0F0"))
    pal.setColor(QPalette.ButtonText,      QColor(TEXT_PRI))
    pal.setColor(QPalette.Highlight,       QColor(ACCENT))
    pal.setColor(QPalette.HighlightedText, QColor("#FFFFFF"))
    pal.setColor(QPalette.ToolTipBase,     QColor(BG_CARD))
    pal.setColor(QPalette.ToolTipText,     QColor(TEXT_PRI))
    pal.setColor(QPalette.Disabled, QPalette.WindowText, QColor(TEXT_DIS))
    pal.setColor(QPalette.Disabled, QPalette.Text,       QColor(TEXT_DIS))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(TEXT_DIS))
    app.setPalette(pal)

    app.setStyleSheet(_SS)


# ── pyqtgraph chart styling ──────────────────────────────────────────────

def apply_chart_theme() -> None:
    """Set pyqtgraph defaults for dark charts."""
    pg.setConfigOptions(
        antialias=True,
        background=QColor(BG_CHART),
        foreground=QColor(TEXT_CHART),
    )


def style_plot_widget(pw: pg.PlotWidget) -> None:
    """Apply dark chart style with subtle grid."""
    pw.setBackground(QColor(BG_CHART))
    for axis_name in ("left", "bottom"):
        ax = pw.getAxis(axis_name)
        ax.setTextPen(pg.mkPen(color=TEXT_CHART))
        ax.setPen(pg.mkPen(color=GRID_CHART))
    pw.showGrid(x=True, y=True, alpha=0.15)
