"""
Microfluidic Control System - Host PC Serial API

Communicates with ESP32 firmware over UART (115200, 8N1).
Protocol: ASCII text commands, newline-terminated.

Usage:
    from microfluidic_api import MicrofluidicController

    ctrl = MicrofluidicController("COM3")
    ctrl.connect()
    ctrl.pump_on()
    ctrl.set_amplitude(200)
    ctrl.set_frequency(100)
    ctrl.disconnect()
"""

import re
import serial
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, List

# ESP-IDF log lines: "I (12345) TAG: message", "W (12345) TAG: ..."
# Levels: E(rror), W(arning), I(nfo), D(ebug), V(erbose)
_ESP_LOG_RE = re.compile(r'^[EWIDV] \(\d+\) ')


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class SystemStatus:
    """Parsed STATUS response: S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration>"""
    mode: str           # "MANUAL" or "PID"
    pump_on: bool
    amplitude: int
    frequency: int
    current_flow: float
    pid_target: float
    pid_elapsed_s: int
    pid_duration_s: int


class CommError(Exception):
    """Raised when ESP32 returns ERR <reason>."""
    pass


class TimeoutError(Exception):
    """Raised when no response received within timeout."""
    pass


# ---------------------------------------------------------------------------
# Main controller class
# ---------------------------------------------------------------------------

class MicrofluidicController:
    """
    Host PC API for the ESP32 microfluidic control system.

    Commands (PC -> ESP32):
        PUMP ON / PUMP OFF
        AMP <80-250>
        FREQ <25-226>
        PID START <target> <duration>
        PID STOP
        PID TARGET <value>
        PID TUNE <Kp> <Ki> <Kd>
        STATUS
        SCAN
        STREAM ON / STREAM OFF

    Async messages (ESP32 -> PC):
        D <flow>                    - 10Hz data stream
        EVENT PID_DONE              - PID duration expired
        EVENT FLOW_ERR <tgt> <act>  - Sustained flow deviation
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

        # Background listener
        self._listener_thread: Optional[threading.Thread] = None
        self._running = False

        # Callbacks (user-settable)
        self.on_data: Optional[Callable[[float], None]] = None
        self.on_pid_done: Optional[Callable[[], None]] = None
        self.on_flow_err: Optional[Callable[[float, float], None]] = None

        # Response queue for synchronous command/response
        self._response_event = threading.Event()
        self._response_line: Optional[str] = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self):
        """Open serial port and start background listener."""
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,  # Short timeout for non-blocking reads in listener
        )
        self._running = True
        self._listener_thread = threading.Thread(
            target=self._listener_loop, daemon=True
        )
        self._listener_thread.start()

    def disconnect(self):
        """Stop listener and close serial port."""
        self._running = False
        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ------------------------------------------------------------------
    # Pump control (Manual mode)
    # ------------------------------------------------------------------

    def pump_on(self):
        """Start the micropump. Blocked during PID mode."""
        self._send_cmd("PUMP ON")

    def pump_off(self):
        """Stop the micropump. Auto-stops PID if in PID mode."""
        self._send_cmd("PUMP OFF")

    def set_amplitude(self, value: int):
        """Set pump amplitude (80-250). Blocked during PID mode."""
        if not 80 <= value <= 250:
            raise ValueError(f"Amplitude must be 80-250, got {value}")
        self._send_cmd(f"AMP {value}")

    def set_frequency(self, freq_hz: int):
        """Set pump frequency (25-226 Hz). Blocked during PID mode."""
        if not 25 <= freq_hz <= 226:
            raise ValueError(f"Frequency must be 25-226 Hz, got {freq_hz}")
        self._send_cmd(f"FREQ {freq_hz}")

    # ------------------------------------------------------------------
    # PID control
    # ------------------------------------------------------------------

    def pid_start(self, target_flow: float, duration_s: int):
        """
        Start PID closed-loop control.

        Args:
            target_flow: Target flow rate in ul/min (must be > 0)
            duration_s:  Duration in seconds (0 = run indefinitely)
        """
        if target_flow <= 0:
            raise ValueError(f"Target flow must be > 0, got {target_flow}")
        self._send_cmd(f"PID START {target_flow:.2f} {duration_s}")

    def pid_stop(self):
        """Stop PID control. Returns to MANUAL mode, pump stops."""
        self._send_cmd("PID STOP")

    def pid_set_target(self, target_flow: float):
        """Update PID target flow rate while running. Only works in PID mode."""
        if target_flow <= 0:
            raise ValueError(f"Target flow must be > 0, got {target_flow}")
        self._send_cmd(f"PID TARGET {target_flow:.2f}")

    def pid_tune(self, kp: float, ki: float, kd: float):
        """Set PID gains. Can be called anytime."""
        self._send_cmd(f"PID TUNE {kp:.4f} {ki:.4f} {kd:.4f}")

    # ------------------------------------------------------------------
    # Data stream
    # ------------------------------------------------------------------

    def stream_on(self):
        """Enable 10Hz flow data stream. Data arrives via on_data callback."""
        self._send_cmd("STREAM ON")

    def stream_off(self):
        """Disable flow data stream."""
        self._send_cmd("STREAM OFF")

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def get_status(self) -> SystemStatus:
        """
        Query full system status.

        Returns:
            SystemStatus with mode, pump_on, amplitude, frequency,
            current_flow, pid_target, pid_elapsed_s, pid_duration_s
        """
        resp = self._send_cmd_raw("STATUS")
        return self._parse_status(resp)

    def scan_i2c(self) -> List[int]:
        """
        Scan I2C bus.

        Returns:
            List of detected device addresses (as integers).
        """
        resp = self._send_cmd_raw("SCAN")
        # Response: "SCAN 08 61" or "SCAN" (no devices)
        parts = resp.split()
        if len(parts) <= 1:
            return []
        return [int(addr, 16) for addr in parts[1:]]

    # ------------------------------------------------------------------
    # Low-level communication
    # ------------------------------------------------------------------

    def _send_cmd(self, cmd: str):
        """Send command and expect OK response. Raises CommError on ERR."""
        resp = self._send_cmd_raw(cmd)
        if resp != "OK":
            if resp.startswith("ERR "):
                raise CommError(resp[4:])
            raise CommError(f"Unexpected response: {resp}")

    def _send_cmd_raw(self, cmd: str) -> str:
        """Send command and return raw response line."""
        if not self.is_connected:
            raise ConnectionError("Not connected")

        with self._lock:
            self._response_event.clear()
            self._response_line = None

            # Send command
            self._ser.write(f"{cmd}\n".encode("ascii"))

            # Wait for response from listener thread
            if not self._response_event.wait(timeout=self.timeout):
                raise TimeoutError(f"No response for: {cmd}")

            return self._response_line

    def _listener_loop(self):
        """Background thread: reads lines and dispatches responses/events."""
        while self._running:
            try:
                if not self._ser or not self._ser.is_open:
                    break
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                self._dispatch(line)
            except (serial.SerialException, OSError):
                break

    def _dispatch(self, line: str):
        """Route incoming line to callback or response slot."""
        # Filter ESP-IDF log output that shares UART0 with our protocol.
        # Format: "I (40478) MAIN: Pump ON", "W (40478) SERIAL: ..."
        if _ESP_LOG_RE.match(line):
            return

        # Data stream: D <flow>
        if line.startswith("D "):
            if self.on_data:
                try:
                    flow = float(line.split()[1])
                    self.on_data(flow)
                except (IndexError, ValueError):
                    pass
            return

        # Event: PID_DONE
        if line == "EVENT PID_DONE":
            if self.on_pid_done:
                self.on_pid_done()
            return

        # Event: FLOW_ERR
        if line.startswith("EVENT FLOW_ERR"):
            if self.on_flow_err:
                try:
                    parts = line.split()
                    target = float(parts[2])
                    actual = float(parts[3])
                    self.on_flow_err(target, actual)
                except (IndexError, ValueError):
                    pass
            return

        # Everything else is a command response (OK, ERR, S ..., SCAN ...)
        self._response_line = line
        self._response_event.set()

    # ------------------------------------------------------------------
    # Parsers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_status(line: str) -> SystemStatus:
        """Parse: S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration>"""
        parts = line.split()
        if len(parts) < 9 or parts[0] != "S":
            raise CommError(f"Invalid STATUS response: {line}")
        return SystemStatus(
            mode=parts[1],
            pump_on=(parts[2] == "1"),
            amplitude=int(parts[3]),
            frequency=int(parts[4]),
            current_flow=float(parts[5]),
            pid_target=float(parts[6]),
            pid_elapsed_s=int(parts[7]),
            pid_duration_s=int(parts[8]),
        )

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False


# ---------------------------------------------------------------------------
# Quick demo
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"

    def on_data(flow):
        print(f"  Flow: {flow:.2f} ul/min")

    def on_pid_done():
        print("  [EVENT] PID_DONE")

    def on_flow_err(target, actual):
        print(f"  [EVENT] FLOW_ERR target={target:.2f} actual={actual:.2f}")

    with MicrofluidicController(port) as ctrl:
        ctrl.on_data = on_data
        ctrl.on_pid_done = on_pid_done
        ctrl.on_flow_err = on_flow_err

        # Query status
        status = ctrl.get_status()
        print(f"Status: {status}")

        # Scan I2C bus
        devices = ctrl.scan_i2c()
        print(f"I2C devices: {[f'0x{d:02X}' for d in devices]}")

        # Manual pump control
        ctrl.set_amplitude(200)
        ctrl.set_frequency(100)
        ctrl.pump_on()
        ctrl.stream_on()

        # Collect data for 5 seconds
        time.sleep(5)

        # Adjust parameters while running
        ctrl.set_amplitude(180)
        ctrl.set_frequency(80)
        status = ctrl.get_status()
        print(f"Status: {status}")

        # Stop
        ctrl.stream_off()
        ctrl.pump_off()
