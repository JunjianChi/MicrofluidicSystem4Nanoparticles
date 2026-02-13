# Microfluidic System - Serial Communication Protocol

## System Architecture

```
+--------------------------------------------------------------+
|  App (Python + PyQt5)                                        |
|                                                              |
|  +---------------------+   +-----------------------------+  |
|  |   Control Panel      |   |   Sensor Dashboard          |  |
|  |                      |   |   (always visible)           |  |
|  |  [PID Mode]          |   |   Flow Rate vs Time          |  |
|  |  [Manual Mode]       |   |   Pressure vs Time           |  |
|  |                      |   |   [Record] [Export CSV]       |  |
|  |  (mutually exclusive)|   |   Safety Alerts               |  |
|  +---------------------+   +-----------------------------+  |
|                USB-UART 115200                               |
+-----------------------------+--------------------------------+
                              |
+-----------------------------+--------------------------------+
|  ESP32 (Firmware)                                            |
|                                                              |
|  serial_comm <-> main.c (FreeRTOS tasks)                     |
|                    |                                         |
|         +----------+----------+                              |
|         |          |          |                              |
|    pid_controller  |    mp6_driver                            |
|         |          |     - amplitude (DAC)                    |
|         |    SLF3S_sensor  - frequency (PWM)                  |
|         |     (flow)   - enable (GPIO)                       |
|         +---------------------+                              |
+--------------------------------------------------------------+
```

## Communication Configuration

- Interface: UART0 (USB)
- Baud rate: 115200, 8N1
- Command terminator: `\n`
- Encoding: ASCII text

## MCU Operating Modes

| Mode | Description | Pump Control |
|------|-------------|-------------|
| MANUAL | Manual mode (power-on default) | App directly controls amplitude/frequency/on-off |
| PID | Constant-flow experiment mode | PID auto-controls pump; App sets target flow and duration |

## Mode Switching Rules

- Power-on default: MANUAL
- `PID START <target> <duration>` → enter PID mode, auto-start pump
- `PID STOP` → stop PID, stop pump, return to MANUAL
- Duration expires → MCU auto-stops PID, sends `EVENT PID_DONE`, stops pump, returns to MANUAL
- `AMP`/`FREQ` during PID mode → `ERR PID_ACTIVE`
- `PUMP OFF` during PID mode → auto PID STOP

---

## PC → ESP32 Commands

### Pump Control (MANUAL mode)

| Command | Format | Parameters | Description |
|---------|--------|-----------|-------------|
| Pump on | `PUMP ON\n` | - | enable HIGH, clock start |
| Pump off | `PUMP OFF\n` | - | DAC→0V, enable LOW, clock stop |
| Set amplitude | `AMP <value>\n` | 80-250 | amplitude → DAC → MP-Driver |
| Set frequency | `FREQ <value>\n` | 25-226 (Hz) | clock PWM frequency |

### PID Constant-Flow Experiment

| Command | Format | Parameters | Description |
|---------|--------|-----------|-------------|
| Start experiment | `PID START <target> <duration>\n` | target: ul/min, duration: seconds (0=infinite) | Start pump + PID closed-loop |
| Stop experiment | `PID STOP\n` | - | Stop pump, return to MANUAL |
| Update target | `PID TARGET <value>\n` | ul/min | Adjust target flow while running |
| Set PID params | `PID TUNE <Kp> <Ki> <Kd>\n` | float | Set PID gains |

### Query & Data Stream

| Command | Format | Description |
|---------|--------|-------------|
| Query status | `STATUS\n` | Return full system status |
| Scan sensors | `SCAN\n` | Scan I2C bus |
| Enable data stream | `STREAM ON\n` | 10Hz periodic sensor data reporting |
| Disable data stream | `STREAM OFF\n` | Stop reporting |

---

## ESP32 → PC Responses

### Command Acknowledgement

| Type | Format | Example |
|------|--------|---------|
| Success | `OK\n` | `OK\n` |
| Failure | `ERR <reason>\n` | `ERR PID_ACTIVE\n` / `ERR INVALID_ARG\n` |

### Status (STATUS response)

Format: `S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration>\n`

| Field | Type | Description |
|-------|------|-------------|
| mode | MANUAL/PID | Current mode |
| pump | 0/1 | Pump state |
| amp | int | Current amplitude (0 or 80-250) |
| freq | int | Frequency (25-226 Hz) |
| flow | float | Current flow rate ul/min |
| target | float | PID target (0.00 in MANUAL) |
| elapsed | int | PID elapsed seconds (0 in MANUAL) |
| duration | int | PID total duration seconds (0=infinite, 0 in MANUAL) |

Example: `S PID 1 185 100 14.80 15.00 323 600\n` (PID mode, target 15 ul/min, elapsed 323s/600s)

### I2C Scan

Format: `SCAN <addr1> <addr2> ...\n`

Example: `SCAN 08 61\n`

### Data Stream (10Hz)

Format: `D <flow>\n`

Example: `D 14.80\n`

> Future extension (when pressure/temperature sensors are ready): `D <flow> <pressure> <temp>\n`

### Event Notifications (MCU-initiated)

| Event | Format | Description |
|-------|--------|-------------|
| Experiment done | `EVENT PID_DONE\n` | Duration expired, pump auto-stopped |
| Flow deviation | `EVENT FLOW_ERR <target> <actual>\n` | Flow deviates from target (>20% for 10s) |

> Future extension: `EVENT PRESSURE_HIGH <value>\n` (when pressure sensor is ready)

---

## Typical Interaction: Nanoparticle Circulation Experiment

```
App                              ESP32
 |                                 |
 |-- SCAN ----------------------->|
 |<---- SCAN 08 61 ---------------|  (found sensor + DAC)
 |                                 |
 |-- STREAM ON ------------------>|
 |<---- OK -----------------------|
 |<---- D 0.00 -------------------|  (data stream started)
 |                                 |
 |-- PID TUNE 2.0 0.5 0.1 ------>|  (set PID parameters)
 |<---- OK -----------------------|
 |                                 |
 |-- PID START 15.0 600 --------->|  (target 15ul/min, 600s = 10min)
 |<---- OK -----------------------|
 |<---- D 2.30 -------------------|  (PID adjusting...)
 |<---- D 8.70 -------------------|
 |<---- D 14.80 ------------------|  (approaching target)
 |<---- D 15.10 ------------------|  (stable)
 |                                 |
 |  ... App recording data to CSV  |
 |  ... App checking thresholds    |
 |                                 |
 |  (if flow deviates persistently)|
 |<---- EVENT FLOW_ERR 15.00 8.50 |  (App shows warning)
 |                                 |
 |  (600 seconds elapsed)          |
 |<---- EVENT PID_DONE ------------|  (experiment done, pump auto-stop)
 |<---- D 0.00 -------------------|
 |                                 |
 |-- STREAM OFF ------------------>|
 |<---- OK -----------------------|
 |                                 |
 |  App: export experiment CSV     |
```
