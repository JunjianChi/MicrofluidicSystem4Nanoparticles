"""
Microfluidic Control System - PyQt5 GUI

Connects to ESP32 via UART using MicrofluidicController API.
Provides manual pump control, PID mode, real-time flow charting,
data recording with CSV export, and event notifications.

Usage:
    python main_gui.py
"""
from __future__ import annotations

import sys
import csv
import time
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
    QSizePolicy, QStackedWidget,
)
from PyQt5.QtGui import QFont, QColor, QTextCursor
import pyqtgraph as pg
import serial.tools.list_ports

from microfluidic_api import MicrofluidicController, CommError, TimeoutError


# ---------------------------------------------------------------------------
# Signal bridge: API callbacks (background thread) -> Qt GUI thread
# ---------------------------------------------------------------------------

class SignalBridge(QObject):
    """Bridges MicrofluidicController callbacks to Qt signals."""
    data_received = pyqtSignal(float)          # flow value
    pid_done = pyqtSignal()
    flow_err = pyqtSignal(float, float)        # target, actual
    log_message = pyqtSignal(str)              # raw log line
    connection_lost = pyqtSignal()             # serial port disconnected


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
        self._t0: Optional[float] = None  # stream start timestamp

        # Recording state
        self._recording = False
        self._record_buf: List[Tuple[float, float]] = []  # (timestamp, flow)
        self._record_start: float = 0.0

        # PID target for chart overlay
        self._pid_target: Optional[float] = None
        self._pid_running = False

        # Plot pause state
        self._plot_paused = False

        # --- Build UI ---
        self._build_ui()
        self._connect_signals()

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
        left_layout.addWidget(self._build_mode_selector_group())
        left_layout.addWidget(self._build_mode_stack())
        left_layout.addWidget(self._build_tools_group())
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
        self.slider_freq.setRange(25, 226)
        self.slider_freq.setValue(100)
        self.spin_freq = QSpinBox()
        self.spin_freq.setRange(25, 226)
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

        form = QFormLayout()
        self.spin_pid_target = QDoubleSpinBox()
        self.spin_pid_target.setRange(0.01, 500.0)
        self.spin_pid_target.setDecimals(2)
        self.spin_pid_target.setSuffix(" ul/min")
        self.spin_pid_target.setValue(10.0)
        form.addRow("Target:", self.spin_pid_target)

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
        layout = QHBoxLayout(grp)
        self.btn_i2c_scan = QPushButton("I2C Scan")
        self.btn_status = QPushButton("STATUS")
        layout.addWidget(self.btn_i2c_scan)
        layout.addWidget(self.btn_status)
        return grp

    # --- Chart Group ---
    def _build_chart_group(self) -> QGroupBox:
        grp = QGroupBox("Flow Rate vs Time")
        layout = QVBoxLayout(grp)

        # Toolbar with pause/resume
        toolbar = QHBoxLayout()
        self.btn_plot_pause = QPushButton("Pause Plot")
        self.btn_plot_pause.setCheckable(True)
        self.btn_plot_pause.setStyleSheet("padding: 4px 12px;")
        toolbar.addStretch()
        toolbar.addWidget(self.btn_plot_pause)
        layout.addLayout(toolbar)

        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel("left", "Flow Rate", units="ul/min")
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setYRange(0, 50)

        self.flow_curve = self.plot_widget.plot(
            pen=pg.mkPen(color="#2196F3", width=2), name="Flow"
        )
        self.target_line = pg.InfiniteLine(
            angle=0, pen=pg.mkPen(color="#f44336", width=1, style=Qt.DashLine),
            label="Target", labelOpts={"color": "#f44336", "position": 0.95}
        )
        self.target_line.setVisible(False)
        self.plot_widget.addItem(self.target_line)

        layout.addWidget(self.plot_widget)
        return grp

    # --- Status Bar Group ---
    def _build_status_bar_group(self) -> QGroupBox:
        grp = QGroupBox("Status")
        layout = QHBoxLayout(grp)
        self.lbl_mode = QLabel("Mode: --")
        self.lbl_mode.setStyleSheet("font-weight: bold;")
        self.lbl_pump_status = QLabel("Pump: --")
        self.lbl_flow_reading = QLabel("Flow: -- ul/min")
        layout.addWidget(self.lbl_mode)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_pump_status)
        layout.addWidget(QLabel(" | "))
        layout.addWidget(self.lbl_flow_reading)
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
        layout = QVBoxLayout(grp)
        self.lbl_alert = QLabel("No alerts")
        self.lbl_alert.setWordWrap(True)
        layout.addWidget(self.lbl_alert)
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
        self.spin_amp.editingFinished.connect(self._cmd_set_amplitude)
        self.spin_freq.editingFinished.connect(self._cmd_set_frequency)

        # PID mode
        self.btn_pid_start.clicked.connect(self._cmd_pid_start)
        self.btn_pid_stop.clicked.connect(self._cmd_pid_stop)

        # Tools
        self.btn_i2c_scan.clicked.connect(self._cmd_i2c_scan)
        self.btn_status.clicked.connect(self._cmd_status)

        # Plot pause/resume
        self.btn_plot_pause.toggled.connect(self._on_plot_pause_toggled)

        # Recording
        self.btn_record.clicked.connect(self._start_recording)
        self.btn_record_stop.clicked.connect(self._stop_recording)
        self.btn_export_csv.clicked.connect(self._export_csv)

        # Bridge signals (thread-safe)
        self._bridge.data_received.connect(self._on_data)
        self._bridge.pid_done.connect(self._on_pid_done)
        self._bridge.flow_err.connect(self._on_flow_err)
        self._bridge.log_message.connect(self._append_log)
        self._bridge.connection_lost.connect(self._on_connection_lost)

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
        self._ctrl.on_data = lambda flow: self._bridge.data_received.emit(flow)
        self._ctrl.on_pid_done = lambda: self._bridge.pid_done.emit()
        self._ctrl.on_flow_err = lambda t, a: self._bridge.flow_err.emit(t, a)

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

        # Reset chart time reference
        self._t0 = time.monotonic()
        self._time_data.clear()
        self._flow_data.clear()

        self._append_log(f"Connected to {port}")

        # --- Step 3: try to enable data stream (non-fatal if it fails) ---
        try:
            self._ctrl.stream_on()
        except (CommError, TimeoutError, ConnectionError) as e:
            self._append_log(f"[WARN] STREAM ON failed: {e} (data stream disabled)")

    def _disconnect(self):
        if self._ctrl:
            try:
                self._ctrl.stream_off()
            except Exception:
                pass
            self._ctrl.disconnect()
            self._ctrl = None

        self._status_timer.stop()
        self._conn_check_timer.stop()

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
            self.btn_pid_start,
            self.spin_pid_target, self.spin_pid_duration,
            self.spin_kp, self.spin_ki, self.spin_kd,
            self.btn_i2c_scan, self.btn_status,
            self.btn_record,
        ]:
            w.setEnabled(enabled)
        if not enabled:
            self.btn_pid_stop.setEnabled(False)

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
            self._append_log(f"[ERROR] {e}")
            self.lbl_alert.setText(f"Error: {e}")
            self.lbl_alert.setStyleSheet("color: red;")
            return False, str(e)

    def _cmd_pump_on(self):
        ok, err = self._run_cmd(self._ctrl.pump_on)
        if ok:
            self.lbl_pump_feedback.setText("Pump: ON")
            self.lbl_pump_feedback.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.lbl_pump_feedback.setText(f"Pump start failed: {err}")
            self.lbl_pump_feedback.setStyleSheet("color: red; font-weight: bold;")

    def _cmd_pump_off(self):
        ok, err = self._run_cmd(self._ctrl.pump_off)
        if ok:
            self.lbl_pump_feedback.setText("Pump: OFF")
            self.lbl_pump_feedback.setStyleSheet("color: gray; font-weight: bold;")
        else:
            self.lbl_pump_feedback.setText(f"Pump stop failed: {err}")
            self.lbl_pump_feedback.setStyleSheet("color: red; font-weight: bold;")

    def _cmd_set_amplitude(self):
        self._run_cmd(self._ctrl.set_amplitude, self.spin_amp.value())

    def _cmd_set_frequency(self):
        self._run_cmd(self._ctrl.set_frequency, self.spin_freq.value())

    def _cmd_pid_start(self):
        target = self.spin_pid_target.value()
        duration = self.spin_pid_duration.value()

        # Confirmation dialog
        if duration > 0:
            dur_str = f"{duration} seconds"
        else:
            dur_str = "indefinitely (no time limit)"
        reply = QMessageBox.question(
            self, "Start PID",
            f"Start PID control?\n\n"
            f"  Target: {target:.2f} ul/min\n"
            f"  Duration: {dur_str}\n"
            f"  Gains: Kp={self.spin_kp.value()}, Ki={self.spin_ki.value()}, Kd={self.spin_kd.value()}\n\n"
            f"Pump will start automatically.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        # Send PID TUNE first (set gains before starting)
        ok, err = self._run_cmd(
            self._ctrl.pid_tune,
            self.spin_kp.value(),
            self.spin_ki.value(),
            self.spin_kd.value(),
        )
        if not ok:
            self.lbl_pid_feedback.setText(f"PID tune failed: {err}")
            self.lbl_pid_feedback.setStyleSheet("color: red; font-weight: bold;")
            return

        # Send PID START
        ok, err = self._run_cmd(self._ctrl.pid_start, target, duration)
        if ok:
            self._pid_running = True
            self._pid_target = target
            self.target_line.setValue(target)
            self.target_line.setVisible(True)

            self.btn_pid_start.setEnabled(False)
            self.btn_pid_stop.setEnabled(True)
            self.spin_pid_target.setEnabled(False)
            self.spin_pid_duration.setEnabled(False)
            self.spin_kp.setEnabled(False)
            self.spin_ki.setEnabled(False)
            self.spin_kd.setEnabled(False)

            # Prevent switching to manual during PID
            self.combo_mode.setEnabled(False)

            self.lbl_pid_feedback.setText(f"PID: running (target={target:.2f} ul/min)")
            self.lbl_pid_feedback.setStyleSheet("color: #2196F3; font-weight: bold;")
        else:
            self.lbl_pid_feedback.setText(f"PID start failed: {err}")
            self.lbl_pid_feedback.setStyleSheet("color: red; font-weight: bold;")

    def _cmd_pid_stop(self):
        # Confirmation dialog
        reply = QMessageBox.question(
            self, "Stop PID",
            "Stop PID control early?\n\nPump will be turned off.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        ok, err = self._run_cmd(self._ctrl.pid_stop)
        if ok:
            self._pid_stopped_cleanup()
            self.lbl_pid_feedback.setText("PID: stopped by user")
            self.lbl_pid_feedback.setStyleSheet("color: gray; font-weight: bold;")
        else:
            self.lbl_pid_feedback.setText(f"PID stop failed: {err}")
            self.lbl_pid_feedback.setStyleSheet("color: red; font-weight: bold;")

    def _pid_stopped_cleanup(self):
        """Reset PID-related UI state after PID stops (by user or auto-done)."""
        self._pid_running = False
        self._pid_target = None
        self.target_line.setVisible(False)

        self.btn_pid_start.setEnabled(True)
        self.btn_pid_stop.setEnabled(False)
        self.spin_pid_target.setEnabled(True)
        self.spin_pid_duration.setEnabled(True)
        self.spin_kp.setEnabled(True)
        self.spin_ki.setEnabled(True)
        self.spin_kd.setEnabled(True)
        self.combo_mode.setEnabled(True)

        self.lbl_pid_elapsed.setText("")

    def _cmd_i2c_scan(self):
        ok, devices = self._run_cmd(self._ctrl.scan_i2c)
        if ok and devices is not None:
            if devices:
                addr_str = ", ".join(f"0x{d:02X}" for d in devices)
                self._append_log(f"I2C devices found: {addr_str}")
                QMessageBox.information(self, "I2C Scan", f"Devices found:\n{addr_str}")
            else:
                self._append_log("I2C scan: no devices found")
                QMessageBox.information(self, "I2C Scan", "No devices found.")

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
    # Data callbacks (from SignalBridge, runs on GUI thread)
    # ==================================================================

    def _on_data(self, flow: float):
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        self._time_data.append(t)
        self._flow_data.append(flow)

        # Update flow reading
        self.lbl_flow_reading.setText(f"Flow: {flow:.2f} ul/min")

        # Recording
        if self._recording:
            self._record_buf.append((now - self._record_start, flow))
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
        self._append_log(f"[EVENT] {msg}")

    # ==================================================================
    # Chart refresh & plot pause
    # ==================================================================

    def _on_plot_pause_toggled(self, checked: bool):
        self._plot_paused = checked
        self.btn_plot_pause.setText("Resume Plot" if checked else "Pause Plot")

    def _refresh_chart(self):
        if not self._plot_paused and self._time_data:
            self.flow_curve.setData(list(self._time_data), list(self._flow_data))

    # ==================================================================
    # Status polling
    # ==================================================================

    def _poll_status(self):
        if not self._ctrl or not self._ctrl.is_connected:
            return
        try:
            status = self._ctrl.get_status()
            self._update_status_display(status)
        except (CommError, TimeoutError, ConnectionError):
            pass

    def _update_status_display(self, status):
        self.lbl_mode.setText(f"Mode: {status.mode}")
        pump_str = "ON" if status.pump_on else "OFF"
        color = "green" if status.pump_on else "gray"
        self.lbl_pump_status.setText(f"Pump: {pump_str}")
        self.lbl_pump_status.setStyleSheet(f"color: {color}; font-weight: bold;")

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
                writer.writerow(["time_s", "flow_ul_min"])
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
