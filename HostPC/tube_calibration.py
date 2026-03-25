"""
Tube Calibration & Flow Range Detection

Runs a multi-phase calibration sequence to determine the usable flow range
for the currently connected tubing.  Executed entirely from the host PC
using existing firmware commands (no firmware changes required).

Phases:
  1. Priming   - gradually increase amplitude until flow is established
  2. Stability - wait for flow to settle within ±7 ul/min over 3 seconds
  3. Range     - measure flow at AMP=250 (max) and AMP=80 (min)

Usage:
    mgr = TubeCalibrationManager(controller)
    mgr.on_progress  = lambda pct, msg: ...
    mgr.on_completed = lambda result: ...
    mgr.on_failed    = lambda err: ...
    threading.Thread(target=mgr.run, daemon=True).start()
"""
from __future__ import annotations

import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

from microfluidic_api import MicrofluidicController, CommError, TimeoutError


# ---------------------------------------------------------------------------
# Result data
# ---------------------------------------------------------------------------

@dataclass
class CalibrationResult:
    """Returned on successful calibration."""
    min_flow: float          # ul/min at AMP=80
    max_flow: float          # ul/min at AMP=250
    sensor_saturated: bool   # True if HIGH_FLOW occurred at AMP=250


# ---------------------------------------------------------------------------
# Manager
# ---------------------------------------------------------------------------

class TubeCalibrationManager:
    """Orchestrates the tube calibration sequence in a background thread."""

    # Tuning constants
    FREQ = 200          # Fixed frequency (Hz) for calibration
    AMP_START = 80      # Starting amplitude
    AMP_MAX = 250       # Maximum amplitude
    AMP_STEP = 10       # Amplitude increment per step
    AMP_STEP_INTERVAL = 2.0   # Seconds between amplitude steps
    PRIME_HOLD_TIME = 10.0    # Seconds to hold after reaching target amp

    STABILITY_WINDOW = 3.0    # Seconds of data for stability check
    STABILITY_SAMPLES = 30    # Expected samples in window (10 Hz * 3 s)
    STABILITY_TOLERANCE = 14.0  # Max (max-min) in window (±7 ul/min)
    STABILITY_TIMEOUT = 30.0  # Max wait for stability

    RANGE_SETTLE_TIME = 5.0   # Seconds to wait at each amplitude for range detection
    RANGE_SAMPLES = 50        # Expected samples in settle window (10 Hz * 5 s)

    def __init__(self, controller: MicrofluidicController):
        self._ctrl = controller

        # Flow data fed from the GUI data callback
        self._flow_buf: deque = deque(maxlen=200)
        self._flow_lock = threading.Lock()

        # HIGH_FLOW event signalling
        self._high_flow_event = threading.Event()
        self._high_flow_active = False

        # Cancellation
        self._cancel_event = threading.Event()

        # Callbacks (called from the calibration thread)
        self.on_progress: Optional[Callable[[int, str], None]] = None
        self.on_completed: Optional[Callable[[CalibrationResult], None]] = None
        self.on_failed: Optional[Callable[[str], None]] = None

    # ------------------------------------------------------------------
    # Public interface (called from GUI thread)
    # ------------------------------------------------------------------

    def feed_flow_data(self, flow: float):
        """Feed a flow reading from the data stream."""
        with self._flow_lock:
            self._flow_buf.append((time.monotonic(), flow))

    def notify_high_flow(self):
        """Called when EVENT HIGH_FLOW is received."""
        self._high_flow_active = True
        self._high_flow_event.set()

    def notify_high_flow_clear(self):
        """Called when EVENT HIGH_FLOW_CLEAR is received."""
        self._high_flow_active = False
        self._high_flow_event.clear()

    def cancel(self):
        """Request cancellation (non-blocking)."""
        self._cancel_event.set()

    # ------------------------------------------------------------------
    # Main calibration sequence (runs in background thread)
    # ------------------------------------------------------------------

    def run(self):
        """Execute the full calibration sequence.  Blocking — run in a thread."""
        try:
            self._progress(0, "Starting calibration...")
            self._setup()
            self._phase_priming()
            if self._cancelled():
                return
            self._phase_stability()
            if self._cancelled():
                return
            result = self._phase_range()
            if self._cancelled():
                return
            self._stop_pump()
            self._progress(100, "Calibration complete")
            if self.on_completed:
                self.on_completed(result)

        except _CancelledError:
            self._safe_stop_pump()
            # on_failed not called for user cancellation — GUI handles it
        except Exception as e:
            self._safe_stop_pump()
            if self.on_failed:
                self.on_failed(str(e))

    # ------------------------------------------------------------------
    # Phases
    # ------------------------------------------------------------------

    def _setup(self):
        """Set frequency, clear buffers, enable stream."""
        self._ctrl.set_frequency(self.FREQ)
        self._clear_flow_buf()
        self._high_flow_active = False
        self._high_flow_event.clear()

    def _phase_priming(self):
        """Gradually increase amplitude; back off on HIGH_FLOW; hold at target."""
        self._progress(5, "Priming: starting pump at AMP=80...")

        amp = self.AMP_START
        self._ctrl.set_amplitude(amp)
        self._ctrl.pump_on()

        target_amp = self.AMP_MAX
        backed_off = False

        while amp < target_amp:
            self._check_cancel()

            # Wait for step interval, but check HIGH_FLOW frequently
            step_end = time.monotonic() + self.AMP_STEP_INTERVAL
            while time.monotonic() < step_end:
                self._check_cancel()
                if self._high_flow_active:
                    # Back off
                    amp = max(self.AMP_START, amp - self.AMP_STEP)
                    self._ctrl.set_amplitude(amp)
                    target_amp = amp  # don't go higher than this
                    backed_off = True
                    self._progress(
                        15, f"Priming: HIGH_FLOW detected, backed off to AMP={amp}")
                    # Wait for HIGH_FLOW to clear
                    clear_deadline = time.monotonic() + 5.0
                    while self._high_flow_active and time.monotonic() < clear_deadline:
                        self._check_cancel()
                        time.sleep(0.1)
                    break
                time.sleep(0.1)

            if backed_off:
                break

            # Increase amplitude
            amp = min(amp + self.AMP_STEP, self.AMP_MAX)
            self._ctrl.set_amplitude(amp)
            pct = 5 + int(15 * (amp - self.AMP_START) / (self.AMP_MAX - self.AMP_START))
            self._progress(pct, f"Priming: AMP={amp}")

        # Check if AMP=80 already triggers HIGH_FLOW
        if amp == self.AMP_START and self._high_flow_active:
            raise RuntimeError(
                "HIGH_FLOW at minimum amplitude (AMP=80). "
                "Check tubing connection or use a higher-resistance tube.")

        # Hold at current amplitude for PRIME_HOLD_TIME
        self._progress(25, f"Priming: holding AMP={amp} for {int(self.PRIME_HOLD_TIME)}s...")
        hold_end = time.monotonic() + self.PRIME_HOLD_TIME
        while time.monotonic() < hold_end:
            self._check_cancel()
            elapsed = self.PRIME_HOLD_TIME - (hold_end - time.monotonic())
            pct = 25 + int(15 * elapsed / self.PRIME_HOLD_TIME)
            self._progress(pct, f"Priming: holding ({int(elapsed)}/{int(self.PRIME_HOLD_TIME)}s)")
            time.sleep(0.5)

    def _phase_stability(self):
        """Wait for flow readings to stabilize within tolerance."""
        self._progress(40, "Stability check: waiting for flow to settle...")
        self._clear_flow_buf()

        deadline = time.monotonic() + self.STABILITY_TIMEOUT
        while time.monotonic() < deadline:
            self._check_cancel()

            with self._flow_lock:
                now = time.monotonic()
                # Get readings from the last STABILITY_WINDOW seconds
                recent = [
                    flow for (t, flow) in self._flow_buf
                    if now - t <= self.STABILITY_WINDOW
                ]

            if len(recent) >= self.STABILITY_SAMPLES // 2:
                spread = max(recent) - min(recent)
                if spread <= self.STABILITY_TOLERANCE:
                    avg = sum(recent) / len(recent)
                    self._progress(
                        55, f"Flow stable: {avg:.1f} ul/min (spread={spread:.1f})")
                    return

            remaining = deadline - time.monotonic()
            pct = 40 + int(15 * (1 - remaining / self.STABILITY_TIMEOUT))
            self._progress(pct, f"Stability: waiting... ({int(remaining)}s left)")
            time.sleep(0.5)

        raise RuntimeError(
            f"Flow did not stabilize within {int(self.STABILITY_TIMEOUT)}s. "
            "Check tubing for leaks or blockages.")

    def _phase_range(self) -> CalibrationResult:
        """Measure flow at AMP_MAX and AMP_START to determine range."""
        sensor_saturated = False

        # --- Measure max flow at AMP=250 ---
        self._progress(60, f"Range: setting AMP={self.AMP_MAX}...")
        self._ctrl.set_amplitude(self.AMP_MAX)
        self._clear_flow_buf()

        settle_end = time.monotonic() + self.RANGE_SETTLE_TIME
        while time.monotonic() < settle_end:
            self._check_cancel()
            remaining = settle_end - time.monotonic()
            pct = 60 + int(10 * (1 - remaining / self.RANGE_SETTLE_TIME))
            self._progress(pct, f"Range: measuring max flow ({int(remaining)}s)...")
            if self._high_flow_active:
                sensor_saturated = True
            time.sleep(0.5)

        with self._flow_lock:
            now = time.monotonic()
            samples = [
                flow for (t, flow) in self._flow_buf
                if now - t <= self.RANGE_SETTLE_TIME
            ]
        if not samples:
            raise RuntimeError("No flow data received during max-flow measurement.")
        max_flow = sum(samples) / len(samples)

        # --- Measure min flow at AMP=80 ---
        self._progress(75, f"Range: setting AMP={self.AMP_START}...")
        self._ctrl.set_amplitude(self.AMP_START)
        self._clear_flow_buf()

        settle_end = time.monotonic() + self.RANGE_SETTLE_TIME
        while time.monotonic() < settle_end:
            self._check_cancel()
            remaining = settle_end - time.monotonic()
            pct = 75 + int(15 * (1 - remaining / self.RANGE_SETTLE_TIME))
            self._progress(pct, f"Range: measuring min flow ({int(remaining)}s)...")
            time.sleep(0.5)

        with self._flow_lock:
            now = time.monotonic()
            samples = [
                flow for (t, flow) in self._flow_buf
                if now - t <= self.RANGE_SETTLE_TIME
            ]
        if not samples:
            raise RuntimeError("No flow data received during min-flow measurement.")
        min_flow = sum(samples) / len(samples)

        return CalibrationResult(
            min_flow=min_flow,
            max_flow=max_flow,
            sensor_saturated=sensor_saturated,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _progress(self, percent: int, message: str):
        if self.on_progress:
            self.on_progress(percent, message)

    def _clear_flow_buf(self):
        with self._flow_lock:
            self._flow_buf.clear()

    def _check_cancel(self):
        if self._cancel_event.is_set():
            raise _CancelledError()

    def _cancelled(self) -> bool:
        return self._cancel_event.is_set()

    def _stop_pump(self):
        try:
            self._ctrl.pump_off()
        except (CommError, TimeoutError, ConnectionError):
            pass

    def _safe_stop_pump(self):
        """Best-effort pump stop — never raises."""
        try:
            self._ctrl.pump_off()
        except Exception:
            pass


class _CancelledError(Exception):
    """Internal: calibration was cancelled by user."""
    pass
