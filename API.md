# Serial Communication API V2.0.0

## Changelog (V1 -> V2)

- **Boot robustness**: UART initializes first; pump/sensor init is non-fatal. System always starts even if no I2C devices are connected.
- **Hardware availability flags**: STATUS response extended with 3 new fields (`pump_hw`, `sensor_hw`, `pressure_hw`).
- **New error codes**: `PUMP_UNAVAIL` and `SENSOR_UNAVAIL` — commands are rejected if required hardware is not detected.
- **Hot-plug support (firmware)**: `sensor_read_task` re-probes I2C bus every 5 seconds for unavailable devices. Newly connected sensors are auto-initialized.
- **Hot-plug support (GUI)**: GUI re-scans I2C every 5 seconds to detect device connect/disconnect, updates UI controls and status labels in real-time.
- **Pressure sensor detection**: Added pressure sensor (default I2C 0x76) to hardware detection and STATUS reporting.
- **PUMP ON fix**: GUI now sends `AMP` + `FREQ` before `PUMP ON` to ensure pump starts with correct parameters.
- **Live slider updates**: Amplitude and frequency sliders update the firmware in real-time (150ms debounce) while pump is running.
- **Tabbed sensor charts**: GUI has separate tabs for Flow Rate and Pressure, with auto-scaling Y axis.
- **Response filtering**: Host-side dispatch only accepts lines starting with `OK`, `ERR`, `S `, `SCAN` as command responses. ESP-IDF boot log and garbage lines are ignored.
- **I2C SCAN-based detection**: GUI detects hardware by I2C address from SCAN results (0x61=pump, 0x08=flow sensor, 0x76=pressure), independent per device.

## System Architecture

```
+------------------------------+       UART 115200 8N1       +------------------------------+
|       Host PC (Python)       | <======== \n term ========> |       ESP32 Firmware          |
|                              |                              |                              |
|  main_gui.py (PyQt5 GUI)    |   >> PUMP ON                 |  serial_comm.c               |
|       |                      |   << OK                      |    serial_comm_process_cmd() |
|  microfluidic_api.py         |   >> AMP 200                 |       |                      |
|    MicrofluidicController    |   << OK                      |       +-> mp6_driver          |
|    +- pump_on/off()          |   >> STATUS                  |       |    +- amplitude (DAC)  |
|    +- set_amplitude/freq()   |   << S MANUAL 1 200 100 ... |       |    +- frequency (LEDC) |
|    +- pid_start/stop/tune()  |   << D 12.50                |       |    +- enable (GPIO)    |
|    +- stream_on/off()        |   << EVENT PID_DONE         |       +-> SLF3S_sensor         |
|    +- get_status()           |                              |       |    +- flow_read()      |
|    +- scan_i2c()             |                              |       +-> pid_controller       |
|                              |                              |            +- setpoint         |
|  Callbacks:                  |                              |            +- compute()        |
|    on_data(flow)             |                              |                              |
|    on_pid_done()             |                              |  sensor_read_task (10Hz)      |
|    on_flow_err(tgt, act)     |                              |    +-> serial_comm_send_data()|
+------------------------------+                              +------------------------------+
```

## Communication Configuration

| Parameter | Value |
|-----------|-------|
| Interface | UART0 (USB serial) |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Encoding | ASCII |
| Line terminator | `\n` |
| Max command length | 128 bytes |
| Default response timeout | 2 seconds |

## Operating Modes

Two mutually exclusive modes:

| Mode | Identifier | Description |
|------|------------|-------------|
| **MANUAL** | `MODE_MANUAL` | Direct control of pump amplitude/frequency from host PC |
| **PID** | `MODE_PID` | Closed-loop PID control; firmware auto-adjusts amplitude to track target flow |

- Default on boot: MANUAL
- Enter PID: `PID START`
- Exit PID: `PID STOP`, `PUMP OFF`, or PID duration expired
- During PID mode: `AMP`, `FREQ`, `PUMP ON` are rejected (`ERR PID_ACTIVE`)

## I2C Device Addresses

| Device | Address | Description |
|--------|---------|-------------|
| MCP4726 DAC | `0x61` | Pump driver amplitude control |
| SLF3S Flow Sensor | `0x08` | Liquid flow measurement |
| Pressure Sensor | `0x76` | Pressure measurement (default, detection only) |

## PC -> ESP32 Commands

### Pump Control (Manual Mode)

| Command | Format | Response | Notes |
|---------|--------|----------|-------|
| Pump ON | `PUMP ON\n` | `OK\n` | Enable GPIO + clock start. Returns `ERR PID_ACTIVE` in PID mode. Returns `ERR PUMP_UNAVAIL` if no pump hardware. |
| Pump OFF | `PUMP OFF\n` | `OK\n` | DAC -> 0V, enable -> LOW, clock stop. If in PID mode, auto-stops PID first. |
| Set amplitude | `AMP <value>\n` | `OK\n` | value: 80-250. Returns `ERR PID_ACTIVE` in PID mode. |
| Set frequency | `FREQ <value>\n` | `OK\n` | value: 25-226 Hz. Returns `ERR PID_ACTIVE` in PID mode. |

### PID Control

| Command | Format | Response | Notes |
|---------|--------|----------|-------|
| PID start | `PID START <target> <duration>\n` | `OK\n` | target: flow rate in ul/min (>0); duration: seconds (0=infinite). Requires both pump and sensor. |
| PID stop | `PID STOP\n` | `OK\n` | Stops PID, returns to MANUAL, pump stops. |
| Update target | `PID TARGET <value>\n` | `OK\n` | Modify target flow while running. Only works in PID mode. |
| Set gains | `PID TUNE <Kp> <Ki> <Kd>\n` | `OK\n` | Set PID gains. Can be called in any mode. |

### Data Stream Control

| Command | Format | Response | Notes |
|---------|--------|----------|-------|
| Stream ON | `STREAM ON\n` | `OK\n` | Enable 10Hz flow data reporting (`D <flow>`) |
| Stream OFF | `STREAM OFF\n` | `OK\n` | Disable flow data reporting |

### Query Commands

| Command | Format | Response | Notes |
|---------|--------|----------|-------|
| Status | `STATUS\n` | `S ...` (see below) | Returns full system status |
| I2C scan | `SCAN\n` | `SCAN ...` (see below) | Scan I2C bus, returns detected device addresses |

## ESP32 -> PC Responses

### Command Responses (synchronous)

Each command returns exactly one response line:

| Type | Format | Example |
|------|--------|---------|
| Success | `OK\n` | `OK\n` |
| Error | `ERR <reason>\n` | `ERR INVALID_ARG\n`, `ERR PID_ACTIVE\n` |
| Status | `S <fields...>\n` | `S MANUAL 1 200 100 12.50 0.00 0 0 1 1 0\n` |
| I2C scan | `SCAN [<addr> ...]\n` | `SCAN 08 61\n` or `SCAN\n` (no devices) |

### Error Codes

| Code | Meaning |
|------|---------|
| `UNKNOWN_CMD` | Unrecognized command |
| `INVALID_ARG` | Argument out of range or wrong format |
| `PID_ACTIVE` | Command blocked during PID mode (AMP, FREQ, PUMP ON) |
| `NOT_PID_MODE` | PID TARGET sent when not in PID mode |
| `PUMP_UNAVAIL` | Pump hardware (MCP4726 DAC at 0x61) not detected |
| `SENSOR_UNAVAIL` | Flow sensor (SLF3S at 0x08) not detected |

### STATUS Response Fields (V2)

`S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration> <pump_hw> <sensor_hw> <pressure_hw>`

| Field | Position | Type | Description |
|-------|----------|------|-------------|
| `S` | 0 | fixed | Status response identifier |
| mode | 1 | string | `MANUAL` or `PID` |
| pump | 2 | 0/1 | Pump state (0=OFF, 1=ON) |
| amp | 3 | int | Current amplitude (80-250) |
| freq | 4 | int | Current frequency (25-226 Hz) |
| flow | 5 | float | Current flow reading (ul/min) |
| target | 6 | float | PID target flow (0.00 in MANUAL mode) |
| elapsed | 7 | int | PID elapsed time (seconds) |
| duration | 8 | int | PID set duration (seconds, 0=infinite) |
| pump_hw | 9 | 0/1 | Pump driver available (1=detected, 0=not found) |
| sensor_hw | 10 | 0/1 | Flow sensor available (1=detected, 0=not found) |
| pressure_hw | 11 | 0/1 | Pressure sensor available (1=detected, 0=not found) |

**Note:** Fields 9-11 are new in V2. The host-side parser is backward-compatible (defaults to 1/1/0 if fields are missing).

### SCAN Response

`SCAN [<addr1> <addr2> ...]`

- Addresses are two-digit uppercase hex, space-separated
- Empty result: just `SCAN` with no addresses
- Example: `SCAN 08 61 76` -> devices at 0x08, 0x61, 0x76

### Async Messages (ESP32 -> PC, unsolicited)

| Type | Format | Trigger |
|------|--------|---------|
| Data stream | `D <flow>\n` | After `STREAM ON`, at 10Hz |
| PID done | `EVENT PID_DONE\n` | PID duration expired, auto-stops |
| Flow error | `EVENT FLOW_ERR <target> <actual>\n` | Sustained flow deviation from target |

## Message Classification (Host-side Dispatch)

The host listener thread classifies each incoming line:

```
Incoming line
  +-- matches ESP-IDF log regex [EWIDV] (\d+) ... -> discard (boot/debug log)
  +-- starts with "D "                            -> data stream, on_data(flow) callback
  +-- == "EVENT PID_DONE"                         -> PID done event, on_pid_done() callback
  +-- starts with "EVENT FLOW_ERR"                -> flow error event, on_flow_err(tgt, act)
  +-- starts with "OK" / "ERR" / "S " / "SCAN"   -> command response (wakes waiting caller)
  +-- anything else                               -> discard (boot garbage, stale output)
```

## Boot Sequence (V2)

```
app_main()
  +-- memset(&g_state, 0)            # Zero-init all state
  +-- pid_init(&g_state.pid)          # PID controller defaults
  +-- xSemaphoreCreateMutex()         # State mutex
  |
  +-- hw_init()                       # ** NON-FATAL - always continues **
  |     +-- serial_comm_init()        # UART0 - must succeed (fatal if fails)
  |     +-- i2c_interface_init()      # I2C bus
  |     +-- i2c_interface_scan()      # Enumerate bus
  |     +-- probe 0x61 -> mp6_init()           -> pump_available = true/false
  |     +-- probe 0x08 -> slf3s_init/start()   -> sensor_available = true/false
  |     +-- probe 0x76                         -> pressure_available = true/false
  |     +-- return ESP_OK             # Always OK (individual failures logged)
  |
  +-- xTaskCreate(serial_cmd_task)    # Always created
  +-- xTaskCreate(sensor_read_task)   # Always created
```

**Key change from V1:** Tasks are always created regardless of hardware detection results. The system is fully functional for serial communication even with no I2C devices connected.

## Hot-Plug Detection (V2)

### Firmware Side (sensor_read_task)

Every 5 seconds, the sensor_read_task re-probes I2C addresses for unavailable devices:

```
Every 5s in sensor_read_task loop:
  if !pump_available:
    probe 0x61 -> mp6_init() -> set pump_available = true
  if !sensor_available:
    probe 0x08 -> slf3s_init() + slf3s_start_measurement() -> set sensor_available = true
  if !pressure_available:
    probe 0x76 -> set pressure_available = true
```

Once a device is detected and initialized, its `*_available` flag is set. This flag is included in subsequent STATUS responses.

### GUI Side (periodic I2C SCAN)

Every 5 seconds, the GUI sends a `SCAN` command and checks addresses:

- `0x61` present -> pump available
- `0x08` present -> flow sensor available
- `0x76` present -> pressure sensor available

On change: UI labels update, controls enable/disable, log messages emitted.

## Hardware Control Chain

```
PC sends AMP 200
  -> serial_comm_process_cmd() parse
    -> check pump_available (ERR PUMP_UNAVAIL if false)
    -> check mode != PID (ERR PID_ACTIVE if PID)
    -> mp6_set_amplitude(&pump, 200)
      -> voltage = 0.35 + (200-80) * 0.95/170 = 1.02V
        -> mcp4726_set_voltage(1.02)
          -> DAC I2C write
            -> MP-Driver amplitude input
              -> Pump ~200V piezoelectric drive

PC sends FREQ 100
  -> serial_comm_process_cmd() parse
    -> mp6_set_frequency(&pump, 100)
      -> LEDC PWM 100Hz 95% duty (GPIO27)
        -> MP-Driver clock input

PC sends PUMP OFF
  -> serial_comm_process_cmd() parse
    -> mp6_stop(&pump)
      -> mcp4726_set_voltage(0)    -> DAC output 0V
      -> gpio_set_level(14, LOW)   -> MP-Driver enable low
      -> clock PWM duty = 0        -> Clock stop

PC sends PID START 15.00 600
  -> serial_comm_process_cmd() parse
    -> check pump_available AND sensor_available
    -> mode switch MANUAL -> PID
    -> pid_start(target=15.00, duration=600)
    -> mp6_start(&pump)
    -> sensor_read_task: pid_compute() -> mp6_set_amplitude() auto-adjust
    -> 600s later -> EVENT PID_DONE
```

## Python Host API Usage

### Direct API Call

```python
from microfluidic_api import MicrofluidicController

def on_data(flow):
    print(f"Flow: {flow:.2f} ul/min")

def on_pid_done():
    print("[EVENT] PID_DONE")

def on_flow_err(target, actual):
    print(f"[EVENT] FLOW_ERR target={target:.2f} actual={actual:.2f}")

with MicrofluidicController("COM3") as ctrl:
    ctrl.on_data = on_data
    ctrl.on_pid_done = on_pid_done
    ctrl.on_flow_err = on_flow_err

    # Query status (V2: includes hardware availability)
    status = ctrl.get_status()
    # -> SystemStatus(mode='MANUAL', pump_on=False, amplitude=200, ...,
    #                 pump_available=True, sensor_available=True, pressure_available=False)

    # I2C scan
    devices = ctrl.scan_i2c()
    # -> [0x08, 0x61]

    # Manual mode
    ctrl.set_amplitude(200)
    ctrl.set_frequency(100)
    ctrl.pump_on()
    ctrl.stream_on()       # Enable 10Hz data stream
    time.sleep(5)           # on_data callback receives data
    ctrl.stream_off()
    ctrl.pump_off()

    # PID mode
    ctrl.pid_tune(1.0, 0.1, 0.01)
    ctrl.pid_start(target_flow=15.0, duration_s=600)
    ctrl.stream_on()
    time.sleep(600)         # on_data + on_pid_done callbacks
    # PID auto-stops on expiry, or manually:
    ctrl.pid_stop()
```

### GUI Application

```bash
cd HostPC
pip install -r requirements.txt
python main_gui.py
```

## File Structure

```
HostPC/
+-- microfluidic_api.py    # Serial communication API (MicrofluidicController)
+-- main_gui.py            # PyQt5 GUI main window
+-- requirements.txt       # Python dependencies (PyQt5, pyqtgraph, pyserial)

Firmware/main/
+-- serial_comm.h/.c       # UART command parsing + data/event reporting
+-- mp6_driver.h/.c        # Micropump driver (amplitude/frequency/enable)
+-- mcp4726_dac.h/.c       # DAC driver (I2C, amplitude -> voltage)
+-- SLF3S_flow_sensor.h/.c # Flow sensor driver (I2C, 10Hz read)
+-- pid_controller.h/.c    # PID closed-loop controller
+-- sensor_config.h        # I2C addresses, pin config, sampling rates
+-- i2c_interface.h/.c     # I2C bus driver (init, probe, scan)
+-- main.h                 # System state struct, command handler declarations
+-- main.c                 # FreeRTOS task orchestration, hardware init
```

## Firmware Interface Functions (serial_comm.h)

| Function | Description |
|----------|-------------|
| `serial_comm_init()` | Initialize UART0 |
| `serial_comm_read_line(buf, max_len)` | Blocking read one command line |
| `serial_comm_send(fmt, ...)` | Formatted send |
| `serial_comm_send_ok()` | Send `OK\n` |
| `serial_comm_send_err(reason)` | Send `ERR <reason>\n` |
| `serial_comm_send_data(flow)` | Send `D <flow>\n` |
| `serial_comm_send_status(state)` | Send `S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration> <pump_hw> <sensor_hw> <pressure_hw>\n` |
| `serial_comm_send_scan(devices, count)` | Send `SCAN <addr1> <addr2> ...\n` |
| `serial_comm_send_event_pid_done()` | Send `EVENT PID_DONE\n` |
| `serial_comm_send_event_flow_err(target, actual)` | Send `EVENT FLOW_ERR <target> <actual>\n` |
| `serial_comm_process_cmd(state, cmd_line)` | Parse and execute one command |
