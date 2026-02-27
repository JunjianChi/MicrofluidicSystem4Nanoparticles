"""
Microfluidic Control System - PyQt5 GUI

Connects to ESP32 via UART using MicrofluidicController API.
Provides manual pump control, PID mode, real-time flow charting,
data recording with CSV export, and event notifications.

Usage:
    python main_gui.py
"""
from __future__ import annotations

import math
import sys
import csv
import time
import threading
from collections import deque
from datetime import datetime
from typing import Optional, List, Tuple

from PyQt5.QtCore import (
    Qt, QTimer, pyqtSignal, QObject,
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout, QVBoxLayout, QGridLayout, QFormLayout,
    QGroupBox, QLabel, QPushButton, QComboBox,
    QSlider, QSpinBox, QDoubleSpinBox, QLineEdit,
    QTextEdit, QFileDialog, QMessageBox, QSplitter,
    QSizePolicy, QStackedWidget, QTabWidget, QProgressBar,
)
from PyQt5.QtGui import QFont, QColor, QTextCursor
import pyqtgraph as pg
import serial.tools.list_ports

from microfluidic_api import MicrofluidicController, CommError, TimeoutError
from tube_calibration import TubeCalibrationManager, CalibrationResult


# ---------------------------------------------------------------------------
# Signal bridge: API callbacks (background thread) -> Qt GUI thread
# ---------------------------------------------------------------------------

class SignalBridge(QObject):
    """Bridges MicrofluidicController callbacks to Qt signals.

    MicrofluidicController callbacks fire on a background listener thread.
    Qt widgets must only be updated from the main GUI thread.  SignalBridge
    converts each callback into a pyqtSignal emission, which Qt delivers
    to connected slots on the GUI thread via the event loop.

    Usage: controller.on_data = lambda f, t: bridge.data_received.emit(f, t)
    """
    data_received = pyqtSignal(float, float)   # flow, temperature
    pid_done = pyqtSignal()
    flow_err = pyqtSignal(float, float)        # target, actual
    air_in_line = pyqtSignal()                 # air bubble detected
    air_clear = pyqtSignal()                   # air-in-line condition resolved
    high_flow = pyqtSignal()                   # flow exceeds sensor range
    high_flow_clear = pyqtSignal()             # high-flow condition resolved
    log_message = pyqtSignal(str)              # raw log line
    connection_lost = pyqtSignal()             # serial port disconnected
    status_received = pyqtSignal(object)       # SystemStatus from background poll
    hw_scan_result = pyqtSignal(object)        # list of I2C addresses from background scan
    cmd_result = pyqtSignal(str, bool, str)    # (cmd_id, success, error_msg)
    cal_progress = pyqtSignal(int, str)         # (percent, message)
    cal_completed = pyqtSignal(object)          # CalibrationResult
    cal_failed = pyqtSignal(str)                # error message


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Microfluidic Control System")
        self.setMinimumSize(1100, 700)

        # Controller & signal bridge
        self._ctrl: Optional[MicrofluidicController] = None
        self._bridge = SignalBridge()

        # Data buffers for plotting (60s window at 10Hz = 600 points)
        self._time_data = deque(maxlen=600)
        self._flow_data = deque(maxlen=600)
        self._pressure_time_data = deque(maxlen=600)
        self._pressure_data = deque(maxlen=600)
        self._t0: Optional[float] = None  # stream start timestamp

        # Recording state
        self._recording = False
        self._record_buf: List[Tuple[float, float, float]] = []  # (timestamp, flow, temperature)
        self._record_start: float = 0.0

        # Hardware availability (updated after connect)
        self._pump_available = True
        self._sensor_available = True
        self._pressure_available = True

        # PID target for chart overlay
        self._pid_target: Optional[float] = None
        self._pid_running = False
        self._pending_pid_target: float = 0.0

        # Plot pause state
        self._plot_paused = False

        # Tube calibration state
        self._cal_manager: Optional[TubeCalibrationManager] = None
        self._calibrating = False
        self._cal_min_flow: Optional[float] = None
        self._cal_max_flow: Optional[float] = None

        # Guards to prevent overlapping background polls
        self._status_busy = False
        self._hw_scan_busy = False

        # --- Build UI ---
        self._build_ui()
        self._connect_signals()

        # ---- Timers overview ----
        # chart    100ms  - refresh pyqtgraph flow/pressure curves
        # status   1s     - poll STATUS command for elapsed time, mode sync
        # conn     2s     - detect unexpected serial disconnect (USB unplug)
        # hw_scan  5s     - re-scan I2C bus for hot-plugged devices
        # debounce 150ms  - delay slider AMP/FREQ commands to avoid flooding

        # Chart refresh timer (100ms = 10Hz)
        self._chart_timer = QTimer(self)
        self._chart_timer.timeout.connect(self._refresh_chart)
        self._chart_timer.start(100)

        # Status poll timer (1s) - updates elapsed time etc.
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._poll_status)

        # Connection health check timer (2s)
        self._conn_check_timer = QTimer(self)
        self._conn_check_timer.timeout.connect(self._check_connection)

        # Hardware hot-plug re-scan timer (5s)
        self._hw_scan_timer = QTimer(self)
        self._hw_scan_timer.timeout.connect(self._periodic_hw_scan)

        # Debounce timers for live slider updates (150ms)
        self._amp_debounce = QTimer(self)
        self._amp_debounce.setSingleShot(True)
        self._amp_debounce.setInterval(150)
        self._amp_debounce.timeout.connect(self._cmd_set_amplitude)

        self._freq_debounce = QTimer(self)
        self._freq_debounce.setSingleShot(True)
        self._freq_debounce.setInterval(150)
        self._freq_debounce.timeout.connect(self._cmd_set_frequency)

        # Debounce timers for live PID tuning (300ms - slower to avoid flooding)
        self._pid_gains_debounce = QTimer(self)
        self._pid_gains_debounce.setSingleShot(True)
        self._pid_gains_debounce.setInterval(300)
        self._pid_gains_debounce.timeout.connect(self._cmd_live_pid_tune)

        self._pid_target_debounce = QTimer(self)
        self._pid_target_debounce.setSingleShot(True)
        self._pid_target_debounce.setInterval(300)
        self._pid_target_debounce.timeout.connect(self._cmd_live_pid_target)

        # Initial UI state
        self._set_controls_enabled(False)

    # ==================================================================
    # UI Construction
    # ==================================================================

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)

        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # --- Left: Control Panel ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)

        left_layout.addWidget(self._build_connection_group())
        left_layout.addWidget(self._build_tube_params_group())
        left_layout.addWidget(self._build_mode_selector_group())
        left_layout.addWidget(self._build_mode_stack())
        left_layout.addWidget(self._build_tools_group())
        left_layout.addWidget(self._build_calibration_group())
        left_layout.addStretch()

        # --- Right: Dashboard ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        right_layout.addWidget(self._build_chart_group(), stretch=3)
        right_layout.addWidget(self._build_status_bar_group())
        right_layout.addWidget(self._build_recording_group())
        right_layout.addWidget(self._build_alerts_group())
        right_layout.addWidget(self._build_log_group(), stretch=1)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([320, 700])

    # --- Connection Group ---
    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("Connection")
        layout = QGridLayout(grp)

        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(100)
        self.btn_refresh_ports = QPushButton("Refresh")
        self.btn_connect = QPushButton("Connect")
        self.lbl_conn_status = QLabel("Disconnected")
        self.lbl_conn_status.setStyleSheet("color: red; font-weight: bold;")

        layout.addWidget(QLabel("COM Port:"), 0, 0)
        layout.addWidget(self.combo_port, 0, 1)
        layout.addWidget(self.btn_refresh_ports, 0, 2)
        layout.addWidget(self.btn_connect, 1, 0, 1, 2)
        layout.addWidget(self.lbl_conn_status, 1, 2)

        self._refresh_ports()
        return grp

    # --- Tube Parameters Group ---
    def _build_tube_params_group(self) -> QGroupBox:
        grp = QGroupBox("Tube Parameters")
        form = QFormLayout(grp)

        self.spin_tube_diameter = QDoubleSpinBox()
        self.spin_tube_diameter.setRange(1, 10000)
        self.spin_tube_diameter.setDecimals(1)
        self.spin_tube_diameter.setSuffix(" um")
        self.spin_tube_diameter.setValue(500.0)
        form.addRow("Diameter:", self.spin_tube_diameter)

        self.spin_tube_length = QDoubleSpinBox()
        self.spin_tube_length.setRange(0.1, 1000)
        self.spin_tube_length.setDecimals(1)
        self.spin_tube_length.setSuffix(" cm")
        self.spin_tube_length.setValue(10.0)
        form.addRow("Length:", self.spin_tube_length)

        self.spin_viscosity = QDoubleSpinBox()
        self.spin_viscosity.setRange(0.01, 1000)
        self.spin_viscosity.setDecimals(2)
        self.spin_viscosity.setSuffix(" mPa.s")
        self.spin_viscosity.setValue(1.00)  # water at ~20C
        form.addRow("Viscosity:", self.spin_viscosity)

        # Live computed values (updated on each flow reading)
        self.lbl_v_avg = QLabel("V_avg: -- mm/s")
        self.lbl_shear_rate = QLabel("Shear rate: -- 1/s")
        self.lbl_shear_stress = QLabel("Shear stress: -- mPa")
        form.addRow(self.lbl_v_avg)
        form.addRow(self.lbl_shear_rate)
        form.addRow(self.lbl_shear_stress)

        return grp

    # --- Mode Selector Group ---
    def _build_mode_selector_group(self) -> QGroupBox:
        grp = QGroupBox("Mode")
        layout = QHBoxLayout(grp)

        self.combo_mode = QComboBox()
        self.combo_mode.addItem("Manual", "manual")
        self.combo_mode.addItem("PID", "pid")
        self.combo_mode.setCurrentIndex(0)
        self.combo_mode.setEnabled(False)

        layout.addWidget(QLabel("Control Mode:"))
        layout.addWidget(self.combo_mode, stretch=1)
        return grp

    # --- Mode Stack (Manual / PID pages) ---
    def _build_mode_stack(self) -> QWidget:
        self.mode_stack = QStackedWidget()
        self.mode_stack.addWidget(self._build_manual_page())  # index 0
        self.mode_stack.addWidget(self._build_pid_page())     # index 1
        self.mode_stack.setCurrentIndex(0)
        return self.mode_stack

    # --- Manual Mode Page ---
    def _build_manual_page(self) -> QWidget:
        page = QGroupBox("Manual Control")
        layout = QVBoxLayout(page)

        # Amplitude
        amp_layout = QHBoxLayout()
        amp_layout.addWidget(QLabel("Amplitude:"))
        self.slider_amp = QSlider(Qt.Horizontal)
        self.slider_amp.setRange(80, 250)
        self.slider_amp.setValue(200)
        self.spin_amp = QSpinBox()
        self.spin_amp.setRange(80, 250)
        self.spin_amp.setValue(200)
        amp_layout.addWidget(self.slider_amp, stretch=1)
        amp_layout.addWidget(self.spin_amp)
        layout.addLayout(amp_layout)

        # Frequency
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Frequency:"))
        self.slider_freq = QSlider(Qt.Horizontal)
        self.slider_freq.setRange(25, 300)
        self.slider_freq.setValue(100)
        self.spin_freq = QSpinBox()
        self.spin_freq.setRange(25, 300)
        self.spin_freq.setValue(100)
        freq_layout.addWidget(self.slider_freq, stretch=1)
        freq_layout.addWidget(self.spin_freq)
        layout.addLayout(freq_layout)

        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_pump_on = QPushButton("PUMP ON")
        self.btn_pump_on.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;")
        self.btn_pump_off = QPushButton("PUMP OFF")
        self.btn_pump_off.setStyleSheet(
            "background-color: #f44336; color: white; font-weight: bold; padding: 6px;")
        btn_layout.addWidget(self.btn_pump_on)
        btn_layout.addWidget(self.btn_pump_off)
        layout.addLayout(btn_layout)

        # Pump feedback label
        self.lbl_pump_feedback = QLabel("Pump: idle")
        self.lbl_pump_feedback.setStyleSheet("color: gray; font-weight: bold;")
        self.lbl_pump_feedback.setWordWrap(True)
        layout.addWidget(self.lbl_pump_feedback)

        # Sync slider <-> spinbox
        self.slider_amp.valueChanged.connect(self.spin_amp.setValue)
        self.spin_amp.valueChanged.connect(self.slider_amp.setValue)
        self.slider_freq.valueChanged.connect(self.spin_freq.setValue)
        self.spin_freq.valueChanged.connect(self.slider_freq.setValue)

        return page

    # --- PID Mode Page ---
    def _build_pid_page(self) -> QWidget:
        page = QGroupBox("PID Control")
        layout = QVBoxLayout(page)

        # Target flow rate — slider + spinbox, range set by calibration
        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("Target:"))
        self.slider_pid_target = QSlider(Qt.Horizontal)
        self.slider_pid_target.setRange(0, 0)  # Updated after calibration
        self.slider_pid_target.setValue(0)
        self.spin_pid_target = QSpinBox()
        self.spin_pid_target.setRange(0, 0)
        self.spin_pid_target.setValue(0)
        self.spin_pid_target.setSuffix(" ul/min")
        target_layout.addWidget(self.slider_pid_target, stretch=1)
        target_layout.addWidget(self.spin_pid_target)
        layout.addLayout(target_layout)

        # Sync slider <-> spinbox
        self.slider_pid_target.valueChanged.connect(self.spin_pid_target.setValue)
        self.spin_pid_target.valueChanged.connect(self.slider_pid_target.setValue)

        # Calibration hint label (shown when not calibrated)
        self.lbl_pid_cal_hint = QLabel("Calibrate tube first to enable PID")
        self.lbl_pid_cal_hint.setStyleSheet("color: #9C27B0; font-style: italic;")
        layout.addWidget(self.lbl_pid_cal_hint)

        form = QFormLayout()
        self.spin_pid_duration = QSpinBox()
        self.spin_pid_duration.setRange(0, 99999)
        self.spin_pid_duration.setSuffix(" sec")
        self.spin_pid_duration.setSpecialValueText("Infinite")
        self.spin_pid_duration.setValue(0)
        form.addRow("Duration:", self.spin_pid_duration)
        layout.addLayout(form)

        # PID gains
        gains_label = QLabel("PID Gains:")
        gains_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(gains_label)

        gains_layout = QHBoxLayout()
        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setRange(0, 9999)
        self.spin_kp.setDecimals(4)
        self.spin_kp.setValue(1.0)
        self.spin_ki = QDoubleSpinBox()
        self.spin_ki.setRange(0, 9999)
        self.spin_ki.setDecimals(4)
        self.spin_ki.setValue(0.1)
        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setRange(0, 9999)
        self.spin_kd.setDecimals(4)
        self.spin_kd.setValue(0.01)

        gains_layout.addWidget(QLabel("Kp"))
        gains_layout.addWidget(self.spin_kp)
        gains_layout.addWidget(QLabel("Ki"))
        gains_layout.addWidget(self.spin_ki)
        gains_layout.addWidget(QLabel("Kd"))
        gains_layout.addWidget(self.spin_kd)
        layout.addLayout(gains_layout)

        # Start / Stop
        pid_btn_layout = QHBoxLayout()
        self.btn_pid_start = QPushButton("START PID")
        self.btn_pid_start.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold; padding: 6px;")
        self.btn_pid_stop = QPushButton("STOP PID")
        self.btn_pid_stop.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold; padding: 6px;")
        self.btn_pid_stop.setEnabled(False)
        pid_btn_layout.addWidget(self.btn_pid_start)
        pid_btn_layout.addWidget(self.btn_pid_stop)
        layout.addLayout(pid_btn_layout)

        # PID feedback / elapsed display
        self.lbl_pid_feedback = QLabel("PID: idle")
        self.lbl_pid_feedback.setStyleSheet("color: gray; font-weight: bold;")
        self.lbl_pid_feedback.setWordWrap(True)
        layout.addWidget(self.lbl_pid_feedback)

        self.lbl_pid_elapsed = QLabel("")
        layout.addWidget(self.lbl_pid_elapsed)

        return page

    # --- Tools Group ---
    def _build_tools_group(self) -> QGroupBox:
        grp = QGroupBox("Tools")
        layout = QVBoxLayout(grp)

        btn_row = QHBoxLayout()
        self.btn_i2c_scan = QPushButton("I2C Scan")
        self.btn_status = QPushButton("STATUS")
        btn_row.addWidget(self.btn_i2c_scan)
        btn_row.addWidget(self.btn_status)
        layout.addLayout(btn_row)

        cal_row = QHBoxLayout()
        cal_row.addWidget(QLabel("Calibration:"))
        self.combo_calibration = QComboBox()
        self.combo_calibration.addItem("Water", "WATER")
        self.combo_calibration.addItem("IPA", "IPA")
        self.combo_calibration.setCurrentIndex(0)
        cal_row.addWidget(self.combo_calibration, stretch=1)
        layout.addLayout(cal_row)

        return grp

    # --- Tube Calibration Group ---
    def _build_calibration_group(self) -> QGroupBox:
        grp = QGroupBox("Tube Calibration")
        layout = QVBoxLayout(grp)

        # Calibrate / Cancel buttons
        btn_row = QHBoxLayout()
        self.btn_calibrate = QPushButton("Calibrate Tube")
        self.btn_calibrate.setStyleSheet(
            "background-color: #9C27B0; color: white; font-weight: bold; padding: 6px;")
        self.btn_cal_cancel = QPushButton("Cancel")
        self.btn_cal_cancel.setEnabled(False)
        btn_row.addWidget(self.btn_calibrate)
        btn_row.addWidget(self.btn_cal_cancel)
        layout.addLayout(btn_row)

        # Progress bar
        self.cal_progress_bar = QProgressBar()
        self.cal_progress_bar.setRange(0, 100)
        self.cal_progress_bar.setValue(0)
        self.cal_progress_bar.setTextVisible(True)
        layout.addWidget(self.cal_progress_bar)

        # Status label
        self.lbl_cal_status = QLabel("Not calibrated")
        self.lbl_cal_status.setWordWrap(True)
        self.lbl_cal_status.setStyleSheet("color: gray;")
        layout.addWidget(self.lbl_cal_status)

        # Range result label
        self.lbl_cal_range = QLabel("")
        self.lbl_cal_range.setWordWrap(True)
        self.lbl_cal_range.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.lbl_cal_range)

        return grp

    # --- Chart Group ---
    def _build_chart_group(self) -> QGroupBox:
        grp = QGroupBox("Sensor Data")
        layout = QVBoxLayout(grp)

        # Toolbar with pause/resume and clear
        toolbar = QHBoxLayout()
        self.btn_plot_clear = QPushButton("Clear")
        self.btn_plot_clear.setStyleSheet("padding: 4px 12px;")
        self.btn_plot_pause = QPushButton("Pause Plot")
        self.btn_plot_pause.setCheckable(True)
        self.btn_plot_pause.setStyleSheet("padding: 4px 12px;")
        toolbar.addStretch()
        toolbar.addWidget(self.btn_plot_clear)
        toolbar.addWidget(self.btn_plot_pause)
        layout.addLayout(toolbar)

        pg.setConfigOptions(antialias=True)

        # Tab widget for switching between sensor charts
        self.chart_tabs = QTabWidget()

        # --- Flow Rate chart ---
        self.flow_plot = pg.PlotWidget()
        self.flow_plot.setLabel("left", "Flow Rate (ul/min)")
        self.flow_plot.setLabel("bottom", "Time (s)")
        self.flow_plot.getAxis("left").enableAutoSIPrefix(False)
        self.flow_plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.flow_plot.showGrid(x=True, y=True, alpha=0.3)
        self.flow_plot.enableAutoRange(axis='y')

        self.flow_curve = self.flow_plot.plot(
            pen=pg.mkPen(color="#2196F3", width=2), name="Flow"
        )
        self.target_line = pg.InfiniteLine(
            angle=0, pen=pg.mkPen(color="#f44336", width=1, style=Qt.DashLine),
            label="Target", labelOpts={"color": "#f44336", "position": 0.95}
        )
        self.target_line.setVisible(False)
        self.flow_plot.addItem(self.target_line)

        # --- Pressure chart ---
        self.pressure_plot = pg.PlotWidget()
        self.pressure_plot.setLabel("left", "Pressure (mbar)")
        self.pressure_plot.setLabel("bottom", "Time (s)")
        self.pressure_plot.getAxis("left").enableAutoSIPrefix(False)
        self.pressure_plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.pressure_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pressure_plot.enableAutoRange(axis='y')

        self.pressure_curve = self.pressure_plot.plot(
            pen=pg.mkPen(color="#FF9800", width=2), name="Pressure"
        )

        self.chart_tabs.addTab(self.flow_plot, "Flow Rate")
        self.chart_tabs.addTab(self.pressure_plot, "Pressure")

        layout.addWidget(self.chart_tabs)
        return grp

    # --- Status Bar Group ---
    def _build_status_bar_group(self) -> QGroupBox:
        grp = QGroupBox("Status")
        layout = QHBoxLayout(grp)
        self.lbl_mode = QLabel("Mode: --")
        self.lbl_mode.setStyleSheet("font-weight: bold;")
        self.lbl_pump_status = QLabel("Pump: --")
        self.lbl_flow_reading = QLabel("Flow: -- ul/min")
        self.lbl_temp_reading = QLabel("Temp: -- C")
        self.lbl_pump_hw = QLabel("Driver: --")
        self.lbl_sensor_hw = QLabel("Flow Sensor: --")
        self.lbl_pressure_hw = QLabel("Pressure: --")
        layout.addWidget(self.lbl_mode)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_pump_status)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_flow_reading)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_temp_reading)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_pump_hw)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_sensor_hw)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_pressure_hw)
        layout.addStretch()
        return grp

    # --- Recording Group ---
    def _build_recording_group(self) -> QGroupBox:
        grp = QGroupBox("Data Recording")
        layout = QHBoxLayout(grp)
        self.btn_record = QPushButton("Record")
        self.btn_record_stop = QPushButton("Stop")
        self.btn_record_stop.setEnabled(False)
        self.btn_export_csv = QPushButton("Export CSV")
        self.btn_export_csv.setEnabled(False)
        self.lbl_record_info = QLabel("Samples: 0  Duration: 00:00")

        layout.addWidget(self.btn_record)
        layout.addWidget(self.btn_record_stop)
        layout.addWidget(self.btn_export_csv)
        layout.addWidget(self.lbl_record_info)
        layout.addStretch()
        return grp

    # --- Alerts Group ---
    def _build_alerts_group(self) -> QGroupBox:
        grp = QGroupBox("Alerts")
        layout = QHBoxLayout(grp)
        self.lbl_alert = QLabel("No alerts")
        self.lbl_alert.setWordWrap(True)
        layout.addWidget(self.lbl_alert, stretch=1)
        self.btn_dismiss_alert = QPushButton("Dismiss")
        self.btn_dismiss_alert.setFixedWidth(70)
        self.btn_dismiss_alert.setVisible(False)
        layout.addWidget(self.btn_dismiss_alert)
        return grp

    # --- Log Group ---
    def _build_log_group(self) -> QGroupBox:
        grp = QGroupBox("Communication Log")
        layout = QVBoxLayout(grp)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFont(QFont("Consolas", 9))
        self.txt_log.setMaximumHeight(150)
        layout.addWidget(self.txt_log)
        return grp

    # ==================================================================
    # Signal / Slot connections
    # ==================================================================

    def _connect_signals(self):
        # Connection
        self.btn_refresh_ports.clicked.connect(self._refresh_ports)
        self.btn_connect.clicked.connect(self._toggle_connection)

        # Mode selector
        self.combo_mode.currentIndexChanged.connect(self._on_mode_changed)

        # Manual mode
        self.btn_pump_on.clicked.connect(self._cmd_pump_on)
        self.btn_pump_off.clicked.connect(self._cmd_pump_off)
        self.slider_amp.valueChanged.connect(self._on_amp_changed)
        self.spin_amp.valueChanged.connect(self._on_amp_changed)
        self.slider_freq.valueChanged.connect(self._on_freq_changed)
        self.spin_freq.valueChanged.connect(self._on_freq_changed)

        # PID mode
        self.btn_pid_start.clicked.connect(self._cmd_pid_start)
        self.btn_pid_stop.clicked.connect(self._cmd_pid_stop)
        self.spin_kp.valueChanged.connect(self._on_pid_gains_changed)
        self.spin_ki.valueChanged.connect(self._on_pid_gains_changed)
        self.spin_kd.valueChanged.connect(self._on_pid_gains_changed)
        self.slider_pid_target.valueChanged.connect(self._on_pid_target_changed)
        self.spin_pid_target.valueChanged.connect(self._on_pid_target_changed)

        # Tools
        self.btn_i2c_scan.clicked.connect(self._cmd_i2c_scan)
        self.btn_status.clicked.connect(self._cmd_status)
        self.combo_calibration.currentIndexChanged.connect(self._cmd_set_calibration)

        # Plot pause/resume
        self.btn_plot_pause.toggled.connect(self._on_plot_pause_toggled)
        self.btn_plot_clear.clicked.connect(self._on_plot_clear)

        # Alerts
        self.btn_dismiss_alert.clicked.connect(self._dismiss_alert)

        # Recording
        self.btn_record.clicked.connect(self._start_recording)
        self.btn_record_stop.clicked.connect(self._stop_recording)
        self.btn_export_csv.clicked.connect(self._export_csv)

        # Calibration
        self.btn_calibrate.clicked.connect(self._cmd_calibrate)
        self.btn_cal_cancel.clicked.connect(self._cmd_cal_cancel)

        # Bridge signals (thread-safe)
        self._bridge.data_received.connect(self._on_data)
        self._bridge.pid_done.connect(self._on_pid_done)
        self._bridge.flow_err.connect(self._on_flow_err)
        self._bridge.air_in_line.connect(self._on_air_in_line)
        self._bridge.air_clear.connect(self._on_air_clear)
        self._bridge.high_flow.connect(self._on_high_flow)
        self._bridge.high_flow_clear.connect(self._on_high_flow_clear)
        self._bridge.log_message.connect(self._append_log)
        self._bridge.connection_lost.connect(self._on_connection_lost)
        self._bridge.status_received.connect(self._update_status_display)
        self._bridge.hw_scan_result.connect(self._on_hw_scan_result)
        self._bridge.cmd_result.connect(self._on_cmd_result)
        self._bridge.cal_progress.connect(self._on_cal_progress)
        self._bridge.cal_completed.connect(self._on_cal_completed)
        self._bridge.cal_failed.connect(self._on_cal_failed)

    # ==================================================================
    # Mode switching
    # ==================================================================

    def _on_mode_changed(self, index: int):
        self.mode_stack.setCurrentIndex(index)

    # ==================================================================
    # Connection management
    # ==================================================================

    def _refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(f"{p.device} - {p.description}", p.device)
        if not ports:
            self.combo_port.addItem("No ports found", "")

    def _toggle_connection(self):
        if self._ctrl and self._ctrl.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        # Clean up any previous controller first
        if self._ctrl is not None:
            self._disconnect()

        port = self.combo_port.currentData()
        if not port:
            QMessageBox.warning(self, "Error", "No COM port selected.")
            return

        # --- Step 1: open serial port ---
        try:
            ctrl = MicrofluidicController(port)
            ctrl.connect()
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Cannot open {port}:\n{e}")
            return

        # Serial port is open - keep the reference
        self._ctrl = ctrl

        # Install callbacks that emit Qt signals
        self._ctrl.on_data = lambda flow, temp: self._bridge.data_received.emit(flow, temp)
        self._ctrl.on_pid_done = lambda: self._bridge.pid_done.emit()
        self._ctrl.on_flow_err = lambda t, a: self._bridge.flow_err.emit(t, a)
        self._ctrl.on_air_in_line = lambda: self._bridge.air_in_line.emit()
        self._ctrl.on_air_clear = lambda: self._bridge.air_clear.emit()
        self._ctrl.on_high_flow = lambda: self._bridge.high_flow.emit()
        self._ctrl.on_high_flow_clear = lambda: self._bridge.high_flow_clear.emit()

        # Wrap _send_cmd_raw to log TX/RX traffic
        original_send = self._ctrl._send_cmd_raw
        def logged_send(cmd):
            self._bridge.log_message.emit(f">> {cmd}")
            resp = original_send(cmd)
            self._bridge.log_message.emit(f"<< {resp}")
            return resp
        self._ctrl._send_cmd_raw = logged_send

        # Wrap _dispatch to log async messages (D / EVENT lines)
        original_dispatch = self._ctrl._dispatch
        def logged_dispatch(line):
            if line.startswith("D ") or line.startswith("EVENT"):
                self._bridge.log_message.emit(f"<< {line}")
            original_dispatch(line)
        self._ctrl._dispatch = logged_dispatch

        # --- Step 2: update UI (connection is already good) ---
        self.lbl_conn_status.setText("Connected")
        self.lbl_conn_status.setStyleSheet("color: green; font-weight: bold;")
        self.btn_connect.setText("Disconnect")
        self._set_controls_enabled(True)
        self._status_timer.start(1000)
        self._conn_check_timer.start(2000)
        self._hw_scan_timer.start(5000)

        # Reset chart time reference
        self._t0 = time.monotonic()
        self._time_data.clear()
        self._flow_data.clear()
        self._pressure_time_data.clear()
        self._pressure_data.clear()

        self._append_log(f"Connected to {port}")

        # --- Step 3: flush stale data and wait for firmware ready ---
        time.sleep(0.3)
        if self._ctrl._ser:
            self._ctrl._ser.reset_input_buffer()

        # --- Step 4: detect hardware via I2C SCAN (independent per device) ---
        MCP4726_ADDR = 0x61   # MP6 pump driver DAC
        SLF3S_ADDR = 0x08     # Flow sensor
        PRESSURE_ADDR = 0x76  # Pressure sensor (default)
        try:
            devices = self._ctrl.scan_i2c()
            self._pump_available = MCP4726_ADDR in devices
            self._sensor_available = SLF3S_ADDR in devices
            self._pressure_available = PRESSURE_ADDR in devices
            addr_str = ", ".join(f"0x{d:02X}" for d in devices) if devices else "none"
            self._append_log(f"I2C scan: [{addr_str}]")
            if not self._pump_available:
                self._append_log("[WARN] Pump driver not detected (no 0x61)")
            if not self._sensor_available:
                self._append_log("[WARN] Flow sensor not detected (no 0x08)")
            if not self._pressure_available:
                self._append_log("[WARN] Pressure sensor not detected (no 0x76)")
        except (CommError, TimeoutError, ConnectionError) as e:
            self._append_log(f"[WARN] I2C scan failed: {e}")
            self._pump_available = False
            self._sensor_available = False
            self._pressure_available = False

        self._update_hw_labels()
        self._apply_hw_availability()

        # --- Step 5: try to enable data stream (non-fatal if it fails) ---
        try:
            self._ctrl.stream_on()
        except (CommError, TimeoutError, ConnectionError) as e:
            self._append_log(f"[WARN] STREAM ON failed: {e} (data stream disabled)")

    def _disconnect(self):
        # Cancel any active calibration
        if self._calibrating and self._cal_manager:
            self._cal_manager.cancel()
        self._calibrating = False
        self._cal_manager = None
        self._cal_min_flow = None
        self._cal_max_flow = None

        if self._ctrl:
            try:
                self._ctrl.stream_off()
            except Exception:
                pass
            self._ctrl.disconnect()
            self._ctrl = None

        self._status_timer.stop()
        self._conn_check_timer.stop()
        self._hw_scan_timer.stop()

        # Reset calibration UI
        self.cal_progress_bar.setValue(0)
        self.lbl_cal_status.setText("Not calibrated")
        self.lbl_cal_status.setStyleSheet("color: gray;")
        self.lbl_cal_range.setText("")
        self.btn_cal_cancel.setEnabled(False)

        # Reset PID target slider (no calibration = no range)
        self.slider_pid_target.setRange(0, 0)
        self.spin_pid_target.setRange(0, 0)
        self.lbl_pid_cal_hint.setText("Calibrate tube first to enable PID")
        self.lbl_pid_cal_hint.setStyleSheet("color: #9C27B0; font-style: italic;")

        # Reset recording state
        if self._recording:
            self._recording = False
            self.btn_record_stop.setEnabled(False)
            self.btn_export_csv.setEnabled(len(self._record_buf) > 0)

        # Reset PID state
        self._pid_running = False
        self._pid_target = None
        self.target_line.setVisible(False)
        self.btn_pid_start.setEnabled(True)
        self.btn_pid_stop.setEnabled(False)
        self.lbl_pid_feedback.setText("PID: idle")
        self.lbl_pid_feedback.setStyleSheet("color: gray; font-weight: bold;")
        self.lbl_pid_elapsed.setText("")

        self.lbl_conn_status.setText("Disconnected")
        self.lbl_conn_status.setStyleSheet("color: red; font-weight: bold;")
        self.btn_connect.setText("Connect")
        self._set_controls_enabled(False)
        self.lbl_pump_feedback.setText("Pump: idle")
        self.lbl_pump_feedback.setStyleSheet("color: gray; font-weight: bold;")

        # Reset hardware availability
        self._pump_available = True
        self._sensor_available = True
        self._pressure_available = True
        self.lbl_pump_hw.setText("Driver: --")
        self.lbl_pump_hw.setStyleSheet("")
        self.lbl_sensor_hw.setText("Flow Sensor: --")
        self.lbl_sensor_hw.setStyleSheet("")
        self.lbl_pressure_hw.setText("Pressure: --")
        self.lbl_pressure_hw.setStyleSheet("")

        self._append_log("Disconnected")

    def _check_connection(self):
        """Periodic check: detect if serial port was unplugged."""
        if self._ctrl and not self._ctrl.is_connected:
            self._bridge.connection_lost.emit()

    def _on_connection_lost(self):
        """Handle unexpected disconnection (USB unplug etc.)."""
        self._append_log("[WARN] Connection lost unexpectedly!")
        self._disconnect()

    def _set_controls_enabled(self, enabled: bool):
        self.combo_mode.setEnabled(enabled)
        for w in [
            self.btn_pump_on, self.btn_pump_off,
            self.slider_amp, self.spin_amp,
            self.slider_freq, self.spin_freq,
            self.btn_i2c_scan, self.btn_status,
            self.combo_calibration,
            self.btn_record,
            self.btn_calibrate,
        ]:
            w.setEnabled(enabled)
        # PID controls: only enable if calibrated
        pid_enabled = enabled and self._cal_min_flow is not None
        for w in [
            self.btn_pid_start,
            self.slider_pid_target, self.spin_pid_target,
            self.spin_pid_duration,
            self.spin_kp, self.spin_ki, self.spin_kd,
        ]:
            w.setEnabled(pid_enabled)
        if not enabled:
            self.btn_pid_stop.setEnabled(False)
            self.btn_cal_cancel.setEnabled(False)

    def _update_hw_labels(self):
        """Update hardware availability labels in status bar."""
        if self._pump_available:
            self.lbl_pump_hw.setText("Driver: OK")
            self.lbl_pump_hw.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.lbl_pump_hw.setText("Driver: N/A")
            self.lbl_pump_hw.setStyleSheet("color: red; font-weight: bold;")

        if self._sensor_available:
            self.lbl_sensor_hw.setText("Flow Sensor: OK")
            self.lbl_sensor_hw.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.lbl_sensor_hw.setText("Flow Sensor: N/A")
            self.lbl_sensor_hw.setStyleSheet("color: red; font-weight: bold;")

        if self._pressure_available:
            self.lbl_pressure_hw.setText("Pressure: OK")
            self.lbl_pressure_hw.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.lbl_pressure_hw.setText("Pressure: N/A")
            self.lbl_pressure_hw.setStyleSheet("color: red; font-weight: bold;")

    def _apply_hw_availability(self):
        """Disable controls for unavailable hardware."""
        if not self._pump_available:
            for w in [self.btn_pump_on, self.btn_pump_off,
                       self.slider_amp, self.spin_amp,
                       self.slider_freq, self.spin_freq]:
                w.setEnabled(False)
            self.lbl_pump_feedback.setText("Pump: driver not detected")
            self.lbl_pump_feedback.setStyleSheet("color: red; font-weight: bold;")

        if not self._sensor_available:
            self.combo_calibration.setEnabled(False)

        # PID requires: pump + sensor + calibration
        if not self._sensor_available or not self._pump_available or self._cal_min_flow is None:
            for w in [self.btn_pid_start,
                       self.slider_pid_target, self.spin_pid_target,
                       self.spin_pid_duration,
                       self.spin_kp, self.spin_ki, self.spin_kd]:
                w.setEnabled(False)
            if not self._pump_available or not self._sensor_available:
                reason = []
                if not self._pump_available:
                    reason.append("pump")
                if not self._sensor_available:
                    reason.append("sensor")
                self.lbl_pid_feedback.setText(f"PID: unavailable ({', '.join(reason)} not detected)")
                self.lbl_pid_feedback.setStyleSheet("color: red; font-weight: bold;")
            elif self._cal_min_flow is None:
                self.lbl_pid_feedback.setText("PID: calibrate tube first")
                self.lbl_pid_feedback.setStyleSheet("color: #9C27B0; font-weight: bold;")

    # ==================================================================
    # Hot-plug hardware re-scan (every 5s)
    # ==================================================================

    def _periodic_hw_scan(self):
        """Re-scan I2C bus to detect newly connected/disconnected sensors."""
        if not self._ctrl or not self._ctrl.is_connected or self._hw_scan_busy:
            return
        self._hw_scan_busy = True
        threading.Thread(target=self._hw_scan_bg, daemon=True).start()

    def _hw_scan_bg(self):
        """Background thread: run I2C scan without blocking GUI."""
        try:
            ctrl = self._ctrl
            if ctrl and ctrl.is_connected:
                devices = ctrl.scan_i2c()
                self._bridge.hw_scan_result.emit(devices)
            else:
                self._hw_scan_busy = False
        except (CommError, TimeoutError, ConnectionError):
            self._hw_scan_busy = False

    def _on_hw_scan_result(self, devices):
        """Process I2C scan results on GUI thread (slot for hw_scan_result signal)."""
        self._hw_scan_busy = False

        MCP4726_ADDR = 0x61
        SLF3S_ADDR = 0x08
        PRESSURE_ADDR = 0x76

        pump_now = MCP4726_ADDR in devices
        sensor_now = SLF3S_ADDR in devices
        pressure_now = PRESSURE_ADDR in devices

        changed = (pump_now != self._pump_available or
                   sensor_now != self._sensor_available or
                   pressure_now != self._pressure_available)

        if changed:
            # Log changes
            if pump_now and not self._pump_available:
                self._append_log("[HOT-PLUG] Pump driver detected")
            elif not pump_now and self._pump_available:
                self._append_log("[HOT-PLUG] Pump driver disconnected")
            if sensor_now and not self._sensor_available:
                self._append_log("[HOT-PLUG] Flow sensor detected")
            elif not sensor_now and self._sensor_available:
                self._append_log("[HOT-PLUG] Flow sensor disconnected")
            if pressure_now and not self._pressure_available:
                self._append_log("[HOT-PLUG] Pressure sensor detected")
            elif not pressure_now and self._pressure_available:
                self._append_log("[HOT-PLUG] Pressure sensor disconnected")

            self._pump_available = pump_now
            self._sensor_available = sensor_now
            self._pressure_available = pressure_now

            self._update_hw_labels()
            self._apply_hw_availability()

            # Re-enable controls if hardware came back
            if self._pump_available:
                for w in [self.btn_pump_on, self.btn_pump_off,
                           self.slider_amp, self.spin_amp,
                           self.slider_freq, self.spin_freq]:
                    w.setEnabled(True)
                self.lbl_pump_feedback.setText("Pump: idle")
                self.lbl_pump_feedback.setStyleSheet("color: gray; font-weight: bold;")

            if self._sensor_available:
                self.combo_calibration.setEnabled(True)

            if (self._pump_available and self._sensor_available
                    and not self._pid_running and self._cal_min_flow is not None):
                for w in [self.btn_pid_start,
                           self.slider_pid_target, self.spin_pid_target,
                           self.spin_pid_duration,
                           self.spin_kp, self.spin_ki, self.spin_kd]:
                    w.setEnabled(True)
                self.lbl_pid_feedback.setText("PID: idle")
                self.lbl_pid_feedback.setStyleSheet("color: gray; font-weight: bold;")

    # ==================================================================
    # Command handlers (run API calls, catch errors)
    # ==================================================================

    def _run_cmd(self, func, *args):
        """Execute an API call with error handling. Returns (success, result)."""
        if not self._ctrl or not self._ctrl.is_connected:
            return False, None
        try:
            result = func(*args)
            return True, result
        except (CommError, TimeoutError, ConnectionError, ValueError) as e:
            err_str = str(e)
            self._append_log(f"[ERROR] {err_str}")
            self.lbl_alert.setText(f"Error: {err_str}")
            self.lbl_alert.setStyleSheet("color: red;")
            # Auto-disable controls on hardware unavailability errors
            if "PUMP_UNAVAIL" in err_str:
                self._pump_available = False
                self._update_hw_labels()
                self._apply_hw_availability()
            if "SENSOR_UNAVAIL" in err_str:
                self._sensor_available = False
                self._update_hw_labels()
                self._apply_hw_availability()
            return False, err_str

    def _cmd_pump_on(self):
        if not self._ctrl or not self._ctrl.is_connected:
            return
        self.btn_pump_on.setEnabled(False)
        self.lbl_pump_feedback.setText("Pump: starting...")
        self.lbl_pump_feedback.setStyleSheet("color: orange; font-weight: bold;")
        amp = self.spin_amp.value()
        freq = self.spin_freq.value()
        self._bg_cmd_seq("pump_on", [
            (self._ctrl.set_amplitude, amp),
            (self._ctrl.set_frequency, freq),
            (self._ctrl.pump_on,),
        ])

    def _cmd_pump_off(self):
        if not self._ctrl or not self._ctrl.is_connected:
            return
        self.btn_pump_off.setEnabled(False)
        self.lbl_pump_feedback.setText("Pump: stopping...")
        self.lbl_pump_feedback.setStyleSheet("color: orange; font-weight: bold;")
        self._bg_cmd_seq("pump_off", [(self._ctrl.pump_off,)])

    def _on_amp_changed(self, _value):
        """Slider/spinbox amplitude changed — debounce then send if pump is on."""
        if self._ctrl and self._ctrl.is_connected and self._pump_available:
            self._amp_debounce.start()  # restart timer on each change

    def _on_freq_changed(self, _value):
        """Slider/spinbox frequency changed — debounce then send if pump is on."""
        if self._ctrl and self._ctrl.is_connected and self._pump_available:
            self._freq_debounce.start()

    def _cmd_set_amplitude(self):
        if not self._ctrl or not self._ctrl.is_connected:
            return
        val = self.spin_amp.value()
        threading.Thread(target=self._bg_cmd, args=(self._ctrl.set_amplitude, val), daemon=True).start()

    def _cmd_set_frequency(self):
        if not self._ctrl or not self._ctrl.is_connected:
            return
        val = self.spin_freq.value()
        threading.Thread(target=self._bg_cmd, args=(self._ctrl.set_frequency, val), daemon=True).start()

    def _on_pid_gains_changed(self, _value):
        """Kp/Ki/Kd spinbox changed — debounce then send PID TUNE if PID is running."""
        if self._pid_running and self._ctrl and self._ctrl.is_connected:
            self._pid_gains_debounce.start()

    def _on_pid_target_changed(self, _value):
        """Target spinbox changed — debounce then send PID TARGET if PID is running."""
        if self._pid_running and self._ctrl and self._ctrl.is_connected:
            self._pid_target_debounce.start()

    def _cmd_live_pid_tune(self):
        """Send PID TUNE with current gains (called after debounce)."""
        if not self._pid_running or not self._ctrl or not self._ctrl.is_connected:
            return
        kp = self.spin_kp.value()
        ki = self.spin_ki.value()
        kd = self.spin_kd.value()
        threading.Thread(
            target=self._bg_cmd, args=(self._ctrl.pid_tune, kp, ki, kd), daemon=True
        ).start()
        self._append_log(f"[PID] Live tune: Kp={kp}, Ki={ki}, Kd={kd}")

    def _cmd_live_pid_target(self):
        """Send PID TARGET with current target value (called after debounce)."""
        if not self._pid_running or not self._ctrl or not self._ctrl.is_connected:
            return
        target = float(self.spin_pid_target.value())
        threading.Thread(
            target=self._bg_cmd, args=(self._ctrl.pid_set_target, target), daemon=True
        ).start()
        # Update chart target line
        self._pid_target = target
        self.target_line.setValue(target)
        self.lbl_pid_feedback.setText(f"PID: running (target={target:.0f} ul/min, FREQ=200)")
        self._append_log(f"[PID] Live target: {target:.0f} ul/min")

    def _bg_cmd(self, func, *args):
        """Run a controller command in background (fire-and-forget)."""
        try:
            func(*args)
        except (CommError, TimeoutError, ConnectionError, ValueError) as e:
            self._bridge.log_message.emit(f"[ERROR] {e}")

    def _bg_cmd_seq(self, cmd_id: str, steps):
        """Run a sequence of API calls in a background thread.

        *steps* is a list of (callable, args...) tuples executed in order.
        On first failure the sequence stops.  Result is delivered via
        cmd_result signal so the GUI thread can update widgets safely.
        """
        def _worker():
            for step in steps:
                func, *args = step
                try:
                    func(*args)
                except (CommError, TimeoutError, ConnectionError, ValueError) as e:
                    self._bridge.cmd_result.emit(cmd_id, False, str(e))
                    return
            self._bridge.cmd_result.emit(cmd_id, True, "")
        threading.Thread(target=_worker, daemon=True).start()

    def _on_cmd_result(self, cmd_id: str, ok: bool, err: str):
        """Handle background command completion on GUI thread."""
        if not ok:
            self._append_log(f"[ERROR] {cmd_id}: {err}")
            if "PUMP_UNAVAIL" in err:
                self._pump_available = False
                self._update_hw_labels()
                self._apply_hw_availability()
            if "SENSOR_UNAVAIL" in err:
                self._sensor_available = False
                self._update_hw_labels()
                self._apply_hw_availability()

        if cmd_id == "pump_on":
            self.btn_pump_on.setEnabled(True)
            if ok:
                self.lbl_pump_feedback.setText("Pump: ON")
                self.lbl_pump_feedback.setStyleSheet("color: green; font-weight: bold;")
            else:
                self.lbl_pump_feedback.setText(f"Pump start failed: {err}")
                self.lbl_pump_feedback.setStyleSheet("color: red; font-weight: bold;")

        elif cmd_id == "pump_off":
            self.btn_pump_off.setEnabled(True)
            if ok:
                self.lbl_pump_feedback.setText("Pump: OFF")
                self.lbl_pump_feedback.setStyleSheet("color: gray; font-weight: bold;")
            else:
                self.lbl_pump_feedback.setText(f"Pump stop failed: {err}")
                self.lbl_pump_feedback.setStyleSheet("color: red; font-weight: bold;")

        elif cmd_id == "pid_start":
            if ok:
                self._pid_running = True
                self._pid_target = self._pending_pid_target
                self.target_line.setValue(self._pid_target)
                self.target_line.setVisible(True)
                self.btn_pid_start.setEnabled(False)
                self.btn_pid_stop.setEnabled(True)
                self.slider_pid_target.setEnabled(True)
                self.spin_pid_target.setEnabled(True)
                self.spin_pid_duration.setEnabled(False)
                self.combo_mode.setEnabled(False)
                self.lbl_pid_feedback.setText(
                    f"PID: running (target={self._pid_target:.0f} ul/min, FREQ=200)")
                self.lbl_pid_feedback.setStyleSheet("color: #2196F3; font-weight: bold;")
            else:
                self.btn_pid_start.setEnabled(True)
                self.lbl_pid_feedback.setText(f"PID start failed: {err}")
                self.lbl_pid_feedback.setStyleSheet("color: red; font-weight: bold;")

        elif cmd_id == "pid_stop":
            if ok:
                self._pid_stopped_cleanup()
                self.lbl_pid_feedback.setText("PID: stopped by user")
                self.lbl_pid_feedback.setStyleSheet("color: gray; font-weight: bold;")
            else:
                self.btn_pid_stop.setEnabled(True)
                self.lbl_pid_feedback.setText(f"PID stop failed: {err}")
                self.lbl_pid_feedback.setStyleSheet("color: red; font-weight: bold;")

        elif cmd_id == "calibration":
            self.combo_calibration.setEnabled(True)
            if ok:
                liquid = self.combo_calibration.currentData()
                self.lbl_alert.setText(f"Calibration set to {liquid}")
                self.lbl_alert.setStyleSheet("color: #2196F3; font-weight: bold;")
                self._append_log(f"Calibration changed to {liquid}")
            else:
                self._append_log(f"Calibration change failed: {err}")

        elif cmd_id == "i2c_scan":
            self.btn_i2c_scan.setEnabled(True)
            self.btn_i2c_scan.setText("I2C Scan")
            if ok:
                if err:  # reused err field to carry address list
                    self._append_log(f"I2C devices found: {err}")
                    QMessageBox.information(self, "I2C Scan", f"Devices found:\n{err}")
                else:
                    self._append_log("I2C scan: no devices found")
                    QMessageBox.information(self, "I2C Scan", "No devices found.")
            else:
                self._append_log(f"I2C scan failed: {err}")

    def _cmd_pid_start(self):
        if self._cal_min_flow is None or self._cal_max_flow is None:
            return  # PID requires calibration
        target = float(self.spin_pid_target.value())
        duration = self.spin_pid_duration.value()

        # Confirmation dialog
        if duration > 0:
            dur_str = f"{duration} seconds"
        else:
            dur_str = "indefinitely (no time limit)"
        range_info = f"\n  Calibrated range: [{self._cal_min_flow:.1f}, {self._cal_max_flow:.1f}] ul/min"
        reply = QMessageBox.question(
            self, "Start PID",
            f"Start PID control?\n\n"
            f"  Target: {target:.0f} ul/min\n"
            f"  Duration: {dur_str}\n"
            f"  FREQ: 200 Hz (fixed)"
            f"{range_info}\n"
            f"  Gains: Kp={self.spin_kp.value()}, Ki={self.spin_ki.value()}, Kd={self.spin_kd.value()}\n\n"
            f"Pump will start automatically (adjusts amplitude at FREQ=200).",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        # Disable controls and run freq+tune+start in background
        self.btn_pid_start.setEnabled(False)
        self.slider_pid_target.setEnabled(False)
        self.spin_pid_target.setEnabled(False)
        self.btn_calibrate.setEnabled(False)  # Disable calibration during PID
        self.lbl_pid_feedback.setText("PID: starting...")
        self.lbl_pid_feedback.setStyleSheet("color: orange; font-weight: bold;")
        self._pending_pid_target = target
        kp = self.spin_kp.value()
        ki = self.spin_ki.value()
        kd = self.spin_kd.value()
        self._bg_cmd_seq("pid_start", [
            (self._ctrl.set_frequency, 200),   # Always FREQ=200 for PID
            (self._ctrl.pid_tune, kp, ki, kd),
            (self._ctrl.pid_start, target, duration),
        ])

    def _cmd_pid_stop(self):
        # Confirmation dialog
        reply = QMessageBox.question(
            self, "Stop PID",
            "Stop PID control early?\n\nPump will be turned off.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        self.btn_pid_stop.setEnabled(False)
        self.lbl_pid_feedback.setText("PID: stopping...")
        self.lbl_pid_feedback.setStyleSheet("color: orange; font-weight: bold;")
        self._bg_cmd_seq("pid_stop", [(self._ctrl.pid_stop,)])

    def _pid_stopped_cleanup(self):
        """Reset PID-related UI state after PID stops (by user or auto-done)."""
        self._pid_running = False
        self._pid_target = None
        self.target_line.setVisible(False)

        calibrated = self._cal_min_flow is not None
        hw_ok = self._pump_available and self._sensor_available
        pid_enabled = calibrated and hw_ok
        self.btn_pid_start.setEnabled(pid_enabled)
        self.btn_pid_stop.setEnabled(False)
        self.slider_pid_target.setEnabled(pid_enabled)
        self.spin_pid_target.setEnabled(pid_enabled)
        self.spin_pid_duration.setEnabled(pid_enabled)
        self.spin_kp.setEnabled(pid_enabled)
        self.spin_ki.setEnabled(pid_enabled)
        self.spin_kd.setEnabled(pid_enabled)
        self.combo_mode.setEnabled(True)
        self.btn_calibrate.setEnabled(True)  # Re-enable calibration

        self.lbl_pid_elapsed.setText("")

    def _cmd_set_calibration(self, _index):
        """Send CAL command when calibration combo box changes."""
        if not self._ctrl or not self._ctrl.is_connected:
            return
        liquid = self.combo_calibration.currentData()
        self.combo_calibration.setEnabled(False)
        self._bg_cmd_seq("calibration", [(self._ctrl.set_calibration, liquid)])

    def _cmd_i2c_scan(self):
        if not self._ctrl or not self._ctrl.is_connected:
            return
        self.btn_i2c_scan.setEnabled(False)
        self.btn_i2c_scan.setText("Scanning...")

        def _worker():
            try:
                devices = self._ctrl.scan_i2c()
                self._bridge.cmd_result.emit(
                    "i2c_scan", True,
                    " ".join(f"0x{d:02X}" for d in devices) if devices else "")
            except (CommError, TimeoutError, ConnectionError, ValueError) as e:
                self._bridge.cmd_result.emit("i2c_scan", False, str(e))
        threading.Thread(target=_worker, daemon=True).start()

    def _cmd_status(self):
        ok, status = self._run_cmd(self._ctrl.get_status)
        if ok and status:
            self._update_status_display(status)
            self._append_log(
                f"STATUS: mode={status.mode} pump={'ON' if status.pump_on else 'OFF'} "
                f"amp={status.amplitude} freq={status.frequency} "
                f"flow={status.current_flow:.2f} target={status.pid_target:.2f} "
                f"elapsed={status.pid_elapsed_s}/{status.pid_duration_s}s"
            )

    # ==================================================================
    # Tube Calibration
    # ==================================================================

    def _cmd_calibrate(self):
        """Start tube calibration sequence."""
        if not self._ctrl or not self._ctrl.is_connected:
            return
        if not self._pump_available or not self._sensor_available:
            QMessageBox.warning(
                self, "Calibration",
                "Calibration requires both pump driver and flow sensor.")
            return

        # Disable controls during calibration
        self._calibrating = True
        self.btn_calibrate.setEnabled(False)
        self.btn_cal_cancel.setEnabled(True)
        self.cal_progress_bar.setValue(0)
        self.lbl_cal_status.setText("Calibrating...")
        self.lbl_cal_status.setStyleSheet("color: #9C27B0; font-weight: bold;")
        self.lbl_cal_range.setText("")

        # Disable pump/PID controls during calibration
        for w in [self.btn_pump_on, self.btn_pump_off,
                   self.slider_amp, self.spin_amp,
                   self.slider_freq, self.spin_freq,
                   self.btn_pid_start, self.combo_mode]:
            w.setEnabled(False)

        # Create and start calibration manager
        self._cal_manager = TubeCalibrationManager(self._ctrl)
        self._cal_manager.on_progress = (
            lambda pct, msg: self._bridge.cal_progress.emit(pct, msg))
        self._cal_manager.on_completed = (
            lambda result: self._bridge.cal_completed.emit(result))
        self._cal_manager.on_failed = (
            lambda err: self._bridge.cal_failed.emit(err))

        threading.Thread(target=self._cal_manager.run, daemon=True).start()
        self._append_log("[CAL] Tube calibration started")

    def _cmd_cal_cancel(self):
        """Cancel ongoing calibration."""
        if self._cal_manager:
            self._cal_manager.cancel()
            self.btn_cal_cancel.setEnabled(False)
            self.lbl_cal_status.setText("Cancelling...")
            self.lbl_cal_status.setStyleSheet("color: orange; font-weight: bold;")
            self._append_log("[CAL] Calibration cancel requested")
            # Cleanup will happen when the thread finishes — use a short timer
            QTimer.singleShot(1000, self._cal_cleanup_after_cancel)

    def _cal_cleanup_after_cancel(self):
        """Restore UI after calibration cancel."""
        if self._calibrating:
            self._calibrating = False
            self._cal_manager = None
            self._cal_restore_controls()
            self.cal_progress_bar.setValue(0)
            self.lbl_cal_status.setText("Cancelled")
            self.lbl_cal_status.setStyleSheet("color: gray;")
            self._append_log("[CAL] Calibration cancelled")

    def _on_cal_progress(self, percent: int, message: str):
        """Update calibration progress (GUI thread)."""
        self.cal_progress_bar.setValue(percent)
        self.lbl_cal_status.setText(message)

    def _on_cal_completed(self, result: CalibrationResult):
        """Handle successful calibration (GUI thread)."""
        self._calibrating = False
        self._cal_manager = None
        self._cal_min_flow = result.min_flow
        self._cal_max_flow = result.max_flow

        self.cal_progress_bar.setValue(100)
        self.lbl_cal_status.setText("Calibration complete")
        self.lbl_cal_status.setStyleSheet("color: green; font-weight: bold;")

        range_text = f"Range: {result.min_flow:.1f} - {result.max_flow:.1f} ul/min"
        if result.sensor_saturated:
            range_text += " (sensor saturated at max)"
        self.lbl_cal_range.setText(range_text)
        self.lbl_cal_range.setStyleSheet("color: #2196F3; font-weight: bold;")

        # Update PID target slider range to calibrated range
        lo = int(result.min_flow)
        hi = int(result.max_flow)
        mid = (lo + hi) // 2
        self.slider_pid_target.setRange(lo, hi)
        self.spin_pid_target.setRange(lo, hi)
        self.slider_pid_target.setValue(mid)
        self.spin_pid_target.setValue(mid)
        self.lbl_pid_cal_hint.setText(
            f"Range: {result.min_flow:.1f} - {result.max_flow:.1f} ul/min")
        self.lbl_pid_cal_hint.setStyleSheet("color: green; font-style: italic;")

        self._cal_restore_controls()
        self._append_log(
            f"[CAL] Complete: range=[{result.min_flow:.1f}, {result.max_flow:.1f}] ul/min"
            + (" (sensor saturated)" if result.sensor_saturated else ""))

    def _on_cal_failed(self, error: str):
        """Handle calibration failure (GUI thread)."""
        self._calibrating = False
        self._cal_manager = None

        self.cal_progress_bar.setValue(0)
        self.lbl_cal_status.setText(f"Failed: {error}")
        self.lbl_cal_status.setStyleSheet("color: red; font-weight: bold;")

        self._cal_restore_controls()
        self._append_log(f"[CAL] Failed: {error}")

    def _cal_restore_controls(self):
        """Re-enable controls after calibration ends."""
        self.btn_calibrate.setEnabled(True)
        self.btn_cal_cancel.setEnabled(False)

        # Restore pump controls based on hardware availability
        if self._pump_available:
            for w in [self.btn_pump_on, self.btn_pump_off,
                       self.slider_amp, self.spin_amp,
                       self.slider_freq, self.spin_freq]:
                w.setEnabled(True)
        self.combo_mode.setEnabled(True)

        # PID: only enable if calibrated + hardware available
        if (self._pump_available and self._sensor_available
                and self._cal_min_flow is not None):
            for w in [self.btn_pid_start,
                       self.slider_pid_target, self.spin_pid_target,
                       self.spin_pid_duration,
                       self.spin_kp, self.spin_ki, self.spin_kd]:
                w.setEnabled(True)
            self.lbl_pid_feedback.setText("PID: ready")
            self.lbl_pid_feedback.setStyleSheet("color: green; font-weight: bold;")

    # ==================================================================
    # Data callbacks (from SignalBridge, runs on GUI thread)
    # ==================================================================

    def _on_data(self, flow: float, temperature: float):
        # Feed flow data to calibration manager if active
        if self._calibrating and self._cal_manager:
            self._cal_manager.feed_flow_data(flow)

        # Skip flow data if sensor is not available
        if not self._sensor_available:
            return

        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        self._time_data.append(t)
        self._flow_data.append(flow)

        # Update flow and temperature readings
        self.lbl_flow_reading.setText(f"Flow: {flow:.2f} ul/min")
        self.lbl_temp_reading.setText(f"Temp: {temperature:.1f} C")

        # Compute fluid dynamics from tube parameters
        # Q: ul/min -> m^3/s  (1 ul = 1e-9 m^3, 1 min = 60 s)
        # r: um -> m  (1 um = 1e-6 m)
        d_um = self.spin_tube_diameter.value()
        r_m = (d_um / 2.0) * 1e-6               # radius in metres
        Q_m3s = flow * 1e-9 / 60.0               # flow in m^3/s
        mu_Pa_s = self.spin_viscosity.value() * 1e-3  # mPa.s -> Pa.s

        if r_m > 0:
            area = math.pi * r_m ** 2
            v_avg = Q_m3s / area                  # m/s
            v_avg_mm = v_avg * 1000.0             # mm/s
            shear_rate = 4.0 * Q_m3s / (math.pi * r_m ** 3)  # 1/s
            shear_stress = mu_Pa_s * shear_rate   # Pa
            shear_stress_mPa = shear_stress * 1000.0  # mPa

            self.lbl_v_avg.setText(f"V_avg: {v_avg_mm:.2f} mm/s")
            self.lbl_shear_rate.setText(f"Shear rate: {shear_rate:.1f} 1/s")
            self.lbl_shear_stress.setText(f"Shear stress: {shear_stress_mPa:.2f} mPa")

        # Recording
        if self._recording:
            self._record_buf.append((now - self._record_start, flow, temperature))
            n = len(self._record_buf)
            dur = now - self._record_start
            mins, secs = divmod(int(dur), 60)
            self.lbl_record_info.setText(f"Samples: {n}  Duration: {mins:02d}:{secs:02d}")

    def _on_pid_done(self):
        self.lbl_alert.setText("PID_DONE: PID control completed.")
        self.lbl_alert.setStyleSheet("color: #2196F3; font-weight: bold;")
        self._pid_stopped_cleanup()
        self.lbl_pid_feedback.setText("PID: completed (duration expired)")
        self.lbl_pid_feedback.setStyleSheet("color: #4CAF50; font-weight: bold;")
        self._append_log("[EVENT] PID_DONE")
        QMessageBox.information(self, "PID Done", "PID control duration has completed.")

    def _on_flow_err(self, target: float, actual: float):
        msg = f"FLOW_ERR: target={target:.2f}, actual={actual:.2f} ul/min"
        self.lbl_alert.setText(msg)
        self.lbl_alert.setStyleSheet("color: red; font-weight: bold;")
        self.btn_dismiss_alert.setVisible(True)
        self._append_log(f"[EVENT] {msg}")

    def _on_air_in_line(self):
        self.lbl_alert.setText("AIR_IN_LINE: Air bubble detected in flow path!")
        self.lbl_alert.setStyleSheet("color: #FF9800; font-weight: bold;")
        self.btn_dismiss_alert.setVisible(False)
        self._append_log("[EVENT] AIR_IN_LINE: Air bubble detected")

    def _on_air_clear(self):
        if self.lbl_alert.text().startswith("AIR_IN_LINE:"):
            self.lbl_alert.setText("No alerts")
            self.lbl_alert.setStyleSheet("")
            self.btn_dismiss_alert.setVisible(False)
        self._append_log("[EVENT] AIR_CLEAR: Air-in-line condition resolved")

    def _on_high_flow(self):
        # Forward to calibration manager
        if self._calibrating and self._cal_manager:
            self._cal_manager.notify_high_flow()
        self.lbl_alert.setText("HIGH_FLOW: Flow rate exceeds sensor range!")
        self.lbl_alert.setStyleSheet("color: red; font-weight: bold;")
        self.btn_dismiss_alert.setVisible(False)
        self._append_log("[EVENT] HIGH_FLOW: Flow rate exceeds sensor range")

    def _on_high_flow_clear(self):
        # Forward to calibration manager
        if self._calibrating and self._cal_manager:
            self._cal_manager.notify_high_flow_clear()
        if self.lbl_alert.text().startswith("HIGH_FLOW:"):
            self.lbl_alert.setText("No alerts")
            self.lbl_alert.setStyleSheet("")
            self.btn_dismiss_alert.setVisible(False)
        self._append_log("[EVENT] HIGH_FLOW_CLEAR: High-flow condition resolved")

    def _dismiss_alert(self):
        self.lbl_alert.setText("No alerts")
        self.lbl_alert.setStyleSheet("")
        self.btn_dismiss_alert.setVisible(False)

    # ==================================================================
    # Chart refresh & plot pause
    # ==================================================================

    def _on_plot_pause_toggled(self, checked: bool):
        self._plot_paused = checked
        self.btn_plot_pause.setText("Resume Plot" if checked else "Pause Plot")

    def _on_plot_clear(self):
        """Clear all chart data and reset time reference."""
        self._time_data.clear()
        self._flow_data.clear()
        self._pressure_time_data.clear()
        self._pressure_data.clear()
        self._t0 = time.monotonic()
        self.flow_curve.setData([], [])
        self.pressure_curve.setData([], [])

    def _refresh_chart(self):
        if self._plot_paused:
            return

        # Flow chart: only update when sensor is available and has data
        if self._sensor_available and self._time_data:
            self.flow_curve.setData(list(self._time_data), list(self._flow_data))

        # Pressure chart: only update when pressure sensor is available and has data
        if self._pressure_available and self._pressure_time_data:
            self.pressure_curve.setData(
                list(self._pressure_time_data), list(self._pressure_data)
            )

    # ==================================================================
    # Status polling
    # ==================================================================

    def _poll_status(self):
        if not self._ctrl or not self._ctrl.is_connected or self._status_busy:
            return
        self._status_busy = True
        threading.Thread(target=self._poll_status_bg, daemon=True).start()

    def _poll_status_bg(self):
        """Background thread: fetch STATUS without blocking GUI."""
        try:
            ctrl = self._ctrl
            if ctrl and ctrl.is_connected:
                status = ctrl.get_status()
                self._bridge.status_received.emit(status)
        except (CommError, TimeoutError, ConnectionError):
            pass
        finally:
            self._status_busy = False

    def _update_status_display(self, status):
        self.lbl_mode.setText(f"Mode: {status.mode}")
        pump_str = "ON" if status.pump_on else "OFF"
        color = "green" if status.pump_on else "gray"
        self.lbl_pump_status.setText(f"Pump: {pump_str}")
        self.lbl_pump_status.setStyleSheet(f"color: {color}; font-weight: bold;")
        self.lbl_flow_reading.setText(f"Flow: {status.current_flow:.2f} ul/min")
        self.lbl_temp_reading.setText(f"Temp: {status.current_temperature:.1f} C")

        # PID elapsed
        if status.mode == "PID":
            if status.pid_duration_s > 0:
                self.lbl_pid_elapsed.setText(
                    f"Elapsed: {status.pid_elapsed_s}/{status.pid_duration_s}s"
                )
            else:
                self.lbl_pid_elapsed.setText(f"Elapsed: {status.pid_elapsed_s}s (infinite)")
            # Sync target line if PID became active externally
            if not self._pid_running:
                self._pid_running = True
                self._pid_target = status.pid_target
                self.target_line.setValue(status.pid_target)
                self.target_line.setVisible(True)
                self.btn_pid_start.setEnabled(False)
                self.btn_pid_stop.setEnabled(True)
                self.combo_mode.setEnabled(False)
        else:
            if self._pid_running:
                self._pid_stopped_cleanup()

        # Safety-net: clear stale alerts based on STATUS flags
        alert_text = self.lbl_alert.text()
        if alert_text.startswith("AIR_IN_LINE:") and not status.air_in_line:
            self.lbl_alert.setText("No alerts")
            self.lbl_alert.setStyleSheet("")
            self.btn_dismiss_alert.setVisible(False)
        if alert_text.startswith("HIGH_FLOW:") and not status.high_flow:
            self.lbl_alert.setText("No alerts")
            self.lbl_alert.setStyleSheet("")
            self.btn_dismiss_alert.setVisible(False)

    # ==================================================================
    # Data recording & CSV export
    # ==================================================================

    def _start_recording(self):
        self._recording = True
        self._record_buf.clear()
        self._record_start = time.monotonic()
        self.btn_record.setEnabled(False)
        self.btn_record_stop.setEnabled(True)
        self.btn_export_csv.setEnabled(False)
        self.lbl_record_info.setText("Samples: 0  Duration: 00:00")
        self._append_log("Recording started")

    def _stop_recording(self):
        self._recording = False
        self.btn_record.setEnabled(True)
        self.btn_record_stop.setEnabled(False)
        self.btn_export_csv.setEnabled(len(self._record_buf) > 0)
        self._append_log(f"Recording stopped ({len(self._record_buf)} samples)")

    def _export_csv(self):
        if not self._record_buf:
            return
        default_name = f"flow_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "Export CSV", default_name, "CSV Files (*.csv)"
        )
        if not path:
            return
        try:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "flow_ul_min", "temperature_c"])
                writer.writerows(self._record_buf)
            self._append_log(f"Exported {len(self._record_buf)} samples to {path}")
            QMessageBox.information(self, "Export", f"Saved {len(self._record_buf)} samples.")
        except OSError as e:
            QMessageBox.critical(self, "Export Error", str(e))

    # ==================================================================
    # Log
    # ==================================================================

    def _append_log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.txt_log.append(f"[{ts}] {text}")
        # Auto-scroll to bottom
        self.txt_log.moveCursor(QTextCursor.End)

    # ==================================================================
    # Cleanup
    # ==================================================================

    def closeEvent(self, event):
        if self._ctrl and self._ctrl.is_connected:
            self._disconnect()
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
