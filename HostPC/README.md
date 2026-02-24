# Microfluidic Control System - GUI User Guide

> **GUI V1.0.0** | PyQt5 + pyqtgraph | Python 3.8+

This document covers every feature of the Host PC graphical user interface for the ESP32 microfluidic control system. For system-level documentation (hardware, firmware, wiring), see the [root README](../README.md).

---

## Table of Contents

1. [Installation](#1-installation)
2. [Connection Setup](#2-connection-setup)
3. [Manual Mode](#3-manual-mode)
4. [PID Mode](#4-pid-mode)
5. [Real-Time Charts](#5-real-time-charts)
6. [Data Recording and CSV Export](#6-data-recording-and-csv-export)
7. [Alerts](#7-alerts)
8. [Hardware Status Monitoring](#8-hardware-status-monitoring)
9. [Tools](#9-tools)
10. [Communication Log](#10-communication-log)
11. [Python API Scripting](#11-python-api-scripting)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Installation

### Option A: Run from Source (Recommended)

```bash
cd HostPC
pip install -r requirements.txt
python main_gui.py
```

**Dependencies** (`requirements.txt`):
```
PyQt5>=5.15
pyqtgraph>=0.13
pyserial>=3.5
```

### Option B: Standalone Executable

Build with PyInstaller:

```bash
cd HostPC
pip install pyinstaller
pyinstaller --onefile --windowed --name MicrofluidicGUI main_gui.py
```

The executable is created at `dist/MicrofluidicGUI.exe`. No Python installation is needed on the target machine.

**USB driver prerequisite:** The target machine must have the correct USB-to-serial driver installed for the ESP32 board:
- **CP2102** (Silicon Labs): [download](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- **CH340** (WCH): [download](http://www.wch-ic.com/downloads/CH341SER_EXE.html)

---

## 2. Connection Setup

### GUI Layout

The connection panel is in the top-left corner of the window:

```
┌─ Connection ───────────────────────────┐
│ COM Port: [COM3 - USB Serial ▼] [Refresh] │
│ [  Connect  ]              Disconnected    │
└────────────────────────────────────────┘
```

### Connecting

1. Plug the ESP32 into USB.
2. Click **Refresh** to scan for available COM ports.
3. Select the correct port from the dropdown (shown as `COMx - device description`).
4. Click **Connect**.

### What Happens on Connect

The GUI performs these steps automatically:

1. **Open serial port** at 115200 baud (8N1).
2. **Flush stale data** -- waits 300 ms and clears the input buffer to discard ESP32 boot logs.
3. **I2C Scan** -- sends `SCAN` to detect connected hardware:
   - `0x61` (MCP4726 DAC) -> pump driver available
   - `0x08` (SLF3S) -> flow sensor available
   - `0x76` -> pressure sensor available
4. **Update hardware status labels** in the status bar.
5. **Enable data stream** -- sends `STREAM ON` to start receiving 10 Hz flow data.
6. **Start timers**:
   - Status poll: every 1 second
   - Connection health check: every 2 seconds
   - Hardware hot-plug re-scan: every 5 seconds

### Auto-Disconnect Detection

The GUI checks the serial port health every 2 seconds. If the ESP32 is unplugged or the connection is lost:

- An automatic disconnect is triggered.
- A warning appears in the Communication Log: `[WARN] Connection lost unexpectedly!`
- All controls are disabled until you reconnect.

### Disconnecting

Click **Disconnect** (the button changes text when connected). The GUI will:
1. Send `STREAM OFF` to stop the data stream.
2. Close the serial port.
3. Stop all background timers.
4. Reset the UI to its disconnected state.

---

## 3. Manual Mode

Manual mode provides direct, open-loop control of the MP6 micropump.

### Controls

```
┌─ Manual Control ──────────────────────┐
│ Amplitude:  [====|=========] [200]    │
│ Frequency:  [==|===========] [100]    │
│                                       │
│ [ PUMP ON  ]  [ PUMP OFF ]           │
│                                       │
│ Pump: idle                            │
└───────────────────────────────────────┘
```

### Amplitude Slider (80-250)

Controls the pump drive voltage via the MCP4726 DAC.

| Value | DAC Voltage | Effect |
|-------|-------------|--------|
| 80 | 0.35 V | Minimum drive (lowest flow) |
| 200 | ~1.02 V | Medium drive |
| 250 | 1.30 V | Maximum drive (highest flow) |

The slider and spinbox are synchronized -- changing one updates the other.

### Frequency Slider (25-300 Hz)

Controls the pump drive frequency via ESP32 PWM (GPIO 27, 95% duty cycle).

Higher frequency generally increases flow rate, but the relationship depends on the specific fluid and tubing configuration.

### Live Slider Updates

When the pump is running and you move a slider, the GUI sends the updated value to the firmware **in real-time** with a **150 ms debounce** delay. This means:
- Rapid slider movements are batched -- only the final value is sent.
- The pump parameters update smoothly without flooding the UART.

### PUMP ON Sequence

When you click **PUMP ON**, the GUI sends three commands in order:
1. `AMP <current_slider_value>` -- ensure firmware has the displayed amplitude
2. `FREQ <current_slider_value>` -- ensure firmware has the displayed frequency
3. `PUMP ON` -- enable the pump

This guarantees the pump starts with the parameters shown in the GUI, even if they were changed while the pump was off.

### PUMP OFF

Sends `PUMP OFF`. If PID mode is active, this also stops the PID controller automatically.

### Feedback Label

The "Pump: idle/ON/OFF" label below the buttons shows the current state. Error messages are displayed here in red if a command fails (e.g., `ERR PID_ACTIVE`).

---

## 4. PID Mode

PID mode enables closed-loop flow rate control. The firmware's PID controller automatically adjusts the pump amplitude to maintain the target flow rate.

### Controls

```
┌─ PID Control ─────────────────────────┐
│ Target:    [  10.00  ] ul/min         │
│ Duration:  [     0   ] sec (Infinite) │
│                                       │
│ PID Gains:                            │
│ Kp [1.0000]  Ki [0.1000]  Kd [0.0100]│
│                                       │
│ [ START PID ]  [ STOP PID ]           │
│                                       │
│ PID: idle                             │
│                                       │
└───────────────────────────────────────┘
```

### Parameters

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| Target | 0.01 - 3250.00 ul/min | 10.00 | Desired flow rate |
| Duration | 0 - 99999 seconds | 0 (Infinite) | How long to run PID. 0 = no time limit. |
| Kp | 0 - 9999 | 1.0 | Proportional gain |
| Ki | 0 - 9999 | 0.1 | Integral gain |
| Kd | 0 - 9999 | 0.01 | Derivative gain |

### START PID Sequence

1. A **confirmation dialog** appears showing the target, duration, and gains.
2. If confirmed, the GUI sends:
   - `PID TUNE <Kp> <Ki> <Kd>` -- set gains
   - `PID START <target> <duration>` -- start PID control
3. The pump starts automatically (firmware handles this).
4. The GUI:
   - Disables parameter inputs and mode selector (prevents accidental changes)
   - Shows a **red dashed target line** on the flow chart
   - Updates the PID feedback label: `PID: running (target=10.00 ul/min)`
   - Enables the **STOP PID** button

### While PID is Running

- The **Elapsed** timer shows `Elapsed: <current>/<total>s` or `Elapsed: <current>s (infinite)`.
- Manual controls (amplitude, frequency, PUMP ON) are locked -- the firmware rejects them with `ERR PID_ACTIVE`.
- The mode selector is disabled.
- You can still use **Tools** (I2C Scan, STATUS) and **Data Recording**.

### STOP PID

1. A **confirmation dialog** asks if you want to stop early.
2. If confirmed, sends `PID STOP`.
3. The pump stops, and the system returns to MANUAL mode.
4. All controls are re-enabled.

### Auto-Completion (PID_DONE)

When the PID duration expires:
1. The firmware sends `EVENT PID_DONE`.
2. The GUI shows an **information dialog**: "PID control duration has completed."
3. The Alerts panel shows: `PID_DONE: PID control completed.`
4. All PID controls are re-enabled.
5. The target line is hidden from the chart.

---

## 5. Real-Time Charts

### Chart Tabs

The GUI has two chart tabs powered by `pyqtgraph`:

| Tab | Y-Axis | Color | Data Source |
|-----|--------|-------|-------------|
| **Flow Rate** | Flow Rate (ul/min) | Blue (#2196F3) | `D <flow>` data stream |
| **Pressure** | Pressure (mbar) | Orange (#FF9800) | Pressure sensor (when available) |

Both charts have auto-scaling Y-axis and show approximately 60 seconds of data (600 points at 10 Hz).

### Flow Rate Chart Features

- **Blue curve**: Real-time flow rate data.
- **Red dashed target line**: Appears during PID mode, showing the setpoint.
- **X-axis**: Time in seconds since connection.
- **Auto-range**: Y-axis adjusts to fit the data.
- **Grid**: Semi-transparent grid for readability.

### Pause / Resume

Click **Pause Plot** (top-right of chart area) to freeze the chart display. Data continues to be collected in the background. Click **Resume Plot** to update the chart with the latest data.

### Refresh Rate

The chart refreshes every **100 ms** (10 Hz), matching the data stream rate.

---

## 6. Data Recording and CSV Export

### Recording Controls

```
┌─ Data Recording ──────────────────────────────────┐
│ [Record] [Stop] [Export CSV]  Samples: 0  00:00   │
└───────────────────────────────────────────────────┘
```

### How to Record Data

1. Click **Record** to start recording. The GUI captures every data point from the stream.
2. The info label updates in real-time: `Samples: 150  Duration: 00:15`
3. Click **Stop** to end recording.
4. Click **Export CSV** to save the data to a file.

### CSV Format

The exported CSV file has three columns:

```csv
time_s,flow_ul_min,temperature_c
0.000,12.50,23.1
0.100,12.48,23.1
0.200,12.55,23.1
...
```

| Column | Type | Description |
|--------|------|-------------|
| `time_s` | float | Time in seconds since recording started |
| `flow_ul_min` | float | Flow rate in ul/min |
| `temperature_c` | float | Temperature in degrees Celsius |

### Default Filename

The suggested filename is `flow_data_YYYYMMDD_HHMMSS.csv` based on the current date/time.

### Notes

- Recording and playback are independent -- you can record while the chart is paused.
- The record buffer persists after stopping. You can export multiple times.
- Starting a new recording clears the previous buffer.
- If the connection is lost during recording, recording stops automatically.

---

## 7. Alerts

### Alert Panel

```
┌─ Alerts ──────────────────────────────────────────┐
│ No alerts                                         │
└───────────────────────────────────────────────────┘
```

The alert panel shows the most recent event notification. Alerts are **edge-triggered** -- the firmware only sends each alert once when the condition first occurs (on the 0->1 transition), not repeatedly.

### Alert Types

| Alert | Color | Trigger | Meaning |
|-------|-------|---------|---------|
| `AIR_IN_LINE` | Orange | Sensor flag bit 0 transitions 0->1 | Air bubble detected in the flow path. Likely causes: inadequate priming, tubing leak, empty reservoir. |
| `HIGH_FLOW` | Red | Sensor flag bit 1 transitions 0->1 | Flow rate exceeds the sensor measurement range. Reduce pump amplitude/frequency. |
| `FLOW_ERR` | Red | Flow deviates >20% from PID target for >10 seconds | PID cannot maintain the target. Possible causes: bubble, blockage, target unachievable. Shows both target and actual values. |
| `PID_DONE` | Blue | PID duration expires | PID control completed normally. Also triggers a popup dialog. |

### Behavior

- Only the **most recent** alert is displayed. Previous alerts are visible in the Communication Log.
- Alerts do not auto-clear -- they remain until replaced by a new alert or a calibration confirmation.
- `FLOW_ERR` shows quantitative data: `FLOW_ERR: target=10.00, actual=15.50 ul/min`
- `PID_DONE` also triggers a **popup dialog** for visibility.

---

## 8. Hardware Status Monitoring

### Status Bar

The status bar runs across the bottom of the dashboard:

```
┌─ Status ──────────────────────────────────────────────────────────────────────┐
│ Mode: MANUAL | Pump: OFF | Flow: 12.50 ul/min | Temp: 23.1 C | Driver: OK | Flow Sensor: OK | Pressure: N/A │
└───────────────────────────────────────────────────────────────────────────────┘
```

### Fields

| Field | Update Source | Description |
|-------|--------------|-------------|
| Mode | STATUS poll (1s) | `MANUAL` or `PID` |
| Pump | STATUS poll (1s) | `ON` (green) or `OFF` (gray) |
| Flow | Data stream (10 Hz) | Current flow reading in ul/min |
| Temp | Data stream (10 Hz) | Current temperature in degrees C |
| Driver | I2C scan (5s) | `OK` (green) if MCP4726 at 0x61 detected, `N/A` (red) otherwise |
| Flow Sensor | I2C scan (5s) | `OK` (green) if SLF3S at 0x08 detected, `N/A` (red) otherwise |
| Pressure | I2C scan (5s) | `OK` (green) if sensor at 0x76 detected, `N/A` (red) otherwise |

### Hot-Plug Detection (5-Second Re-Scan)

Every 5 seconds, the GUI sends a `SCAN` command and checks which I2C addresses are present:

- If a device **appears** (was N/A, now detected): the GUI logs `[HOT-PLUG] <device> detected` and re-enables the associated controls.
- If a device **disappears** (was OK, now gone): the GUI logs `[HOT-PLUG] <device> disconnected` and disables the associated controls.

**Control availability based on hardware:**

| Hardware Missing | Controls Disabled |
|-----------------|-------------------|
| Pump driver (0x61) | PUMP ON, PUMP OFF, Amplitude slider, Frequency slider |
| Flow sensor (0x08) | Calibration dropdown |
| Pump + Sensor | All PID controls (START, Target, Duration, Gains) |

When missing hardware is reconnected, the controls are automatically re-enabled.

---

## 9. Tools

### Tools Panel

```
┌─ Tools ───────────────────────────────┐
│ [I2C Scan] [STATUS]                   │
│ Calibration: [Water ▼]               │
└───────────────────────────────────────┘
```

### I2C Scan

Sends the `SCAN` command to the ESP32. The firmware scans the I2C bus and returns all detected device addresses.

Results are shown in:
- A **popup dialog**: `Devices found: 0x08, 0x61`
- The **Communication Log**: `I2C devices found: 0x08, 0x61`

Known addresses:
| Address | Device |
|---------|--------|
| 0x08 | SLF3S flow sensor |
| 0x61 | MCP4726 DAC (pump amplitude control) |
| 0x76 | Pressure sensor (reserved) |
| 0x48 | Temperature sensor (reserved) |

### STATUS Query

Sends the `STATUS` command. The response updates the status bar and is logged:

```
STATUS: mode=MANUAL pump=OFF amp=200 freq=100 flow=12.50 target=0.00 elapsed=0/0s
```

### Calibration (Water / IPA)

The calibration dropdown switches the flow sensor's internal calibration between:

| Liquid | Calibration Byte | Use Case |
|--------|-----------------|----------|
| **Water** | 0x08 | Deionized water, aqueous solutions |
| **IPA** | 0x15 | Isopropanol |

When you change the selection, the GUI sends `CAL WATER` or `CAL IPA`. The firmware stops the flow measurement, reconfigures the sensor, and restarts.

**Important:** Always match the calibration to your actual working liquid. Mismatched calibration produces inaccurate flow readings.

---

## 10. Communication Log

### Log Panel

```
┌─ Communication Log ───────────────────────────────┐
│ [14:30:15.123] Connected to COM3                  │
│ [14:30:15.456] I2C scan: [0x08, 0x61]            │
│ [14:30:15.789] >> STREAM ON                       │
│ [14:30:15.812] << OK                              │
│ [14:30:16.100] << D 12.50 23.1                    │
│ [14:30:16.200] << D 12.48 23.1                    │
└───────────────────────────────────────────────────┘
```

### Log Format

Every log entry has a millisecond-precision timestamp:

| Prefix | Meaning |
|--------|---------|
| `>>` | Command sent to ESP32 (TX) |
| `<<` | Response/data received from ESP32 (RX) |
| `[WARN]` | Warning (connection issues, failed commands) |
| `[ERROR]` | Error from a command response |
| `[EVENT]` | Async event (PID_DONE, AIR_IN_LINE, etc.) |
| `[HOT-PLUG]` | Hardware connect/disconnect detection |

### Auto-Scroll

The log auto-scrolls to show the latest entry. The log panel is read-only.

### Logged Items

- All TX/RX serial traffic (commands and responses)
- Data stream lines (`D <flow> <temp>`) and events (`EVENT ...`)
- Connection and disconnection events
- I2C scan results
- Hardware hot-plug changes
- Calibration changes
- Recording start/stop with sample counts
- CSV export results
- Error messages

---

## 11. Python API Scripting

You can use `microfluidic_api.py` directly for headless (no GUI) experiments or automated testing.

### Basic Usage

```python
import time
from microfluidic_api import MicrofluidicController

# Connect
ctrl = MicrofluidicController("COM3")
ctrl.connect()

# Query status
status = ctrl.get_status()
print(f"Mode: {status.mode}, Pump: {'ON' if status.pump_on else 'OFF'}")
print(f"Hardware: pump={status.pump_available}, sensor={status.sensor_available}")

# Scan I2C bus
devices = ctrl.scan_i2c()
print(f"I2C devices: {[f'0x{d:02X}' for d in devices]}")

# Manual pump control
ctrl.set_amplitude(200)
ctrl.set_frequency(100)
ctrl.pump_on()
time.sleep(5)
ctrl.pump_off()

# Disconnect
ctrl.disconnect()
```

### Using Context Manager

```python
from microfluidic_api import MicrofluidicController

with MicrofluidicController("COM3") as ctrl:
    ctrl.pump_on()
    # ... do experiment ...
    ctrl.pump_off()
# Automatically disconnected on exit
```

### Data Stream with Callbacks

```python
import time
from microfluidic_api import MicrofluidicController

def on_data(flow, temperature):
    print(f"Flow: {flow:.2f} ul/min, Temp: {temperature:.1f} C")

def on_pid_done():
    print("[EVENT] PID completed!")

def on_flow_err(target, actual):
    print(f"[EVENT] FLOW_ERR: target={target:.2f}, actual={actual:.2f}")

def on_air_in_line():
    print("[EVENT] Air bubble detected!")

def on_high_flow():
    print("[EVENT] Flow exceeds sensor range!")

with MicrofluidicController("COM3") as ctrl:
    # Set callbacks
    ctrl.on_data = on_data
    ctrl.on_pid_done = on_pid_done
    ctrl.on_flow_err = on_flow_err
    ctrl.on_air_in_line = on_air_in_line
    ctrl.on_high_flow = on_high_flow

    # Start data stream
    ctrl.stream_on()
    time.sleep(5)   # on_data fires at 10 Hz
    ctrl.stream_off()
```

### PID Experiment Script

```python
import time
from microfluidic_api import MicrofluidicController

results = []

def on_data(flow, temperature):
    results.append((time.time(), flow, temperature))

with MicrofluidicController("COM3") as ctrl:
    ctrl.on_data = on_data

    # Set PID gains
    ctrl.pid_tune(kp=1.0, ki=0.1, kd=0.01)

    # Start PID: target 15 ul/min for 300 seconds
    ctrl.pid_start(target_flow=15.0, duration_s=300)
    ctrl.stream_on()

    # Wait for completion
    time.sleep(300)

    ctrl.stream_off()

# Save results
import csv
with open("pid_experiment.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "flow_ul_min", "temperature_c"])
    writer.writerows(results)
```

### API Reference

**Connection:**

| Method | Description |
|--------|-------------|
| `MicrofluidicController(port, baudrate=115200, timeout=2.0)` | Create controller instance |
| `connect()` | Open serial port, start listener thread |
| `disconnect()` | Stop listener, close serial port |
| `is_connected` | Property: True if serial port is open |

**Pump Control (Manual Mode):**

| Method | Description |
|--------|-------------|
| `pump_on()` | Start pump. Raises `CommError` if PID active or pump unavailable. |
| `pump_off()` | Stop pump. Auto-stops PID if active. |
| `set_amplitude(value)` | Set amplitude 80-250. Raises `ValueError` if out of range. |
| `set_frequency(freq_hz)` | Set frequency 25-300 Hz. Raises `ValueError` if out of range. |

**PID Control:**

| Method | Description |
|--------|-------------|
| `pid_start(target_flow, duration_s)` | Start PID. target_flow > 0 (ul/min). duration_s = 0 for infinite. |
| `pid_stop()` | Stop PID, return to MANUAL, stop pump. |
| `pid_set_target(target_flow)` | Update target while PID running. Raises `CommError` if not in PID mode. |
| `pid_tune(kp, ki, kd)` | Set PID gains. Works in any mode. |

**Data Stream:**

| Method | Description |
|--------|-------------|
| `stream_on()` | Enable 10 Hz data stream. Data arrives via `on_data` callback. |
| `stream_off()` | Disable data stream. |

**Sensor Calibration:**

| Method | Description |
|--------|-------------|
| `set_calibration(liquid)` | Set calibration. liquid: `"WATER"` or `"IPA"`. |

**Query:**

| Method | Returns | Description |
|--------|---------|-------------|
| `get_status()` | `SystemStatus` | Full system status (mode, pump, flow, PID, hardware availability) |
| `scan_i2c()` | `List[int]` | List of detected I2C addresses |

**Callbacks:**

| Callback | Signature | Trigger |
|----------|-----------|---------|
| `on_data` | `(flow: float, temperature: float)` | Every 100 ms when stream is ON |
| `on_pid_done` | `()` | PID duration expired |
| `on_flow_err` | `(target: float, actual: float)` | Sustained flow deviation |
| `on_air_in_line` | `()` | Air bubble detected (edge-triggered) |
| `on_high_flow` | `()` | Flow exceeds sensor range (edge-triggered) |

**Exceptions:**

| Exception | Cause |
|-----------|-------|
| `CommError` | ESP32 returned `ERR <reason>` |
| `TimeoutError` | No response within timeout (default 2s) |
| `ConnectionError` | Serial port not connected |
| `ValueError` | Parameter out of range (client-side validation) |

### SystemStatus Fields

```python
@dataclass
class SystemStatus:
    mode: str               # "MANUAL" or "PID"
    pump_on: bool           # True if pump is running
    amplitude: int          # Current amplitude (80-250)
    frequency: int          # Current frequency (25-300 Hz)
    current_flow: float     # Flow reading (ul/min)
    pid_target: float       # PID target (0.0 in MANUAL)
    pid_elapsed_s: int      # PID elapsed seconds
    pid_duration_s: int     # PID total duration (0=infinite)
    pump_available: bool    # Pump driver detected
    sensor_available: bool  # Flow sensor detected
    pressure_available: bool # Pressure sensor detected
    current_temperature: float # Temperature (C)
```

---

## 12. Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| No COM ports in dropdown | USB driver not installed | Install CP2102 or CH340 driver. Click **Refresh**. |
| "Cannot open COMx" error | Port in use by another app | Close serial monitors (PuTTY, Arduino IDE, etc.). |
| "No COM port selected" error | No port chosen in dropdown | Select a valid COM port before clicking Connect. |
| Connection succeeds but no data | `STREAM ON` failed | Check Communication Log for `[WARN] STREAM ON failed`. Reconnect. |
| Flow reading stuck at `-- ul/min` | Not connected or sensor unavailable | Connect to ESP32. Check that Flow Sensor shows `OK` in status bar. |
| Chart not updating | Plot is paused | Click **Resume Plot** (check if button says "Resume Plot"). |
| Chart shows flat line at 0 | Pump not running or tubing dry | Start the pump. Prime the system with liquid. |
| Sliders not updating firmware | Pump is off (by design) | Sliders send live updates only when connected. Values are sent on PUMP ON. |
| PUMP ON fails: `ERR PID_ACTIVE` | PID mode is running | Stop PID first (STOP PID button), then use manual controls. |
| PUMP ON fails: `ERR PUMP_UNAVAIL` | DAC not detected at 0x61 | Check I2C wiring. Run I2C Scan. Wait for hot-plug detection. |
| PID START fails: hardware unavailable | Pump or sensor not detected | Both pump (0x61) and sensor (0x08) must be detected for PID. Check wiring. |
| PID not converging | Gains too aggressive or too conservative | Start with defaults (Kp=1.0, Ki=0.1, Kd=0.01). Adjust incrementally. |
| `FLOW_ERR` alert during PID | Flow >20% off target for >10s | Check for air bubbles, blockages, or unreachable targets. |
| `AIR_IN_LINE` alert | Air bubble in flow path | Re-prime the system at max amplitude (250) and frequency (300 Hz). |
| `HIGH_FLOW` alert | Flow exceeds sensor range | Reduce pump amplitude and/or frequency. |
| Hardware status shows N/A | I2C device not detected | Check physical connections. Verify 4.7k pull-ups on SDA/SCL. |
| Controls grayed out after disconnect | Hardware not re-detected on reconnect | Click Connect again. GUI will re-scan hardware. |
| CSV export is empty | Forgot to Record before experiment | Click **Record** before starting, **Stop** when done, then **Export CSV**. |
| GUI freezes momentarily on connect | ESP32 boot log processing | Normal -- the GUI flushes stale data (300 ms). Wait for "Connected" status. |
| Calibration change fails: `SENSOR_UNAVAIL` | Flow sensor not detected | Calibration requires the flow sensor. Check wiring and run I2C Scan. |
| Mode selector is disabled | PID is running | Stop PID to switch modes. Mode is locked during PID to prevent conflicts. |

---

## GUI Architecture Overview

For developers who want to modify the GUI:

```
main_gui.py
├── SignalBridge (QObject)          # Thread-safe callback -> Qt signal bridge
│   ├── data_received(float, float) # flow, temperature
│   ├── pid_done()
│   ├── flow_err(float, float)      # target, actual
│   ├── air_in_line()
│   ├── high_flow()
│   ├── log_message(str)
│   └── connection_lost()
│
└── MainWindow (QMainWindow)
    ├── Connection Group             # COM port selection, connect/disconnect
    ├── Mode Selector                # Manual / PID combo box
    ├── Mode Stack (QStackedWidget)
    │   ├── Manual Page              # Amplitude/Frequency sliders, PUMP ON/OFF
    │   └── PID Page                 # Target, Duration, Gains, START/STOP
    ├── Tools Group                  # I2C Scan, STATUS, Calibration
    ├── Chart Group (QTabWidget)
    │   ├── Flow Rate (pyqtgraph)   # Blue curve + red target line
    │   └── Pressure (pyqtgraph)    # Orange curve
    ├── Status Bar Group             # Mode, Pump, Flow, Temp, HW status
    ├── Recording Group              # Record, Stop, Export CSV
    ├── Alerts Group                 # Latest event notification
    └── Communication Log            # Timestamped TX/RX log
```

### Timer Summary

| Timer | Interval | Purpose |
|-------|----------|---------|
| `_chart_timer` | 100 ms | Refresh pyqtgraph charts |
| `_status_timer` | 1000 ms | Poll STATUS for mode/pump/elapsed updates |
| `_conn_check_timer` | 2000 ms | Detect unexpected serial disconnection |
| `_hw_scan_timer` | 5000 ms | Re-scan I2C bus for hot-plug detection |
| `_amp_debounce` | 150 ms (single-shot) | Debounce amplitude slider changes |
| `_freq_debounce` | 150 ms (single-shot) | Debounce frequency slider changes |
