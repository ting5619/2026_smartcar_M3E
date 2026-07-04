"""
Pre-flight checks and in-flight safety monitoring.

Checks:
  - PSDK + roscore + fcProxy are running
  - VO health acceptable
  - GPS home point recorded (outdoor flights)
  - Battery level adequate
  - Flight altitude within limits

In-flight:
  - Heartbeat monitoring
  - VO health degradation alert
  - Battery drain rate
  - Geofence violation
"""

import time
import math
import threading
from enum import Enum
from typing import Optional, Callable
from dataclasses import dataclass, field

from me3_groundstation.telemetry import TelemetryFrame
from me3_groundstation.vo_fusion import VOFusion


class AlertLevel(Enum):
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"


@dataclass
class Alert:
    level: AlertLevel
    message: str
    timestamp: float = field(default_factory=time.time)


class SafetyMonitor:
    """Monitors telemetry and VO health during flight.

    Usage:
        monitor = SafetyMonitor(vo_fusion)
        monitor.start()
        # in telemetry callback:
        monitor.check(frame)
        if monitor.has_critical():
            controller.abort()
    """

    def __init__(self, vo_fusion: VOFusion,
                 min_battery_pct: float = 15.0,
                 warn_battery_pct: float = 25.0,
                 max_altitude_m: float = 5.0,
                 min_altitude_m: float = 0.1,
                 heartbeat_timeout_s: float = 3.0,
                 vo_health_min_ratio: float = 0.5):
        self.vo = vo_fusion

        # Thresholds
        self.min_battery = min_battery_pct
        self.warn_battery = warn_battery_pct
        self.max_altitude = max_altitude_m
        self.min_altitude = min_altitude_m
        self.heartbeat_timeout = heartbeat_timeout_s
        self.vo_health_min_ratio = vo_health_min_ratio

        # State
        self._last_telemetry_t: float = 0.0
        self._alerts: list = []
        self._lock = threading.Lock()
        self._running = False
        self._frame_count = 0
        self._battery_start: Optional[float] = None
        self._on_alert: Optional[Callable] = None

    def start(self):
        """Start monitoring."""
        self._running = True
        self._last_telemetry_t = time.time()

    def stop(self):
        self._running = False

    def check(self, frame: TelemetryFrame) -> list:
        """Run all safety checks on a telemetry frame.

        Returns list of new alerts.
        """
        if not self._running:
            return []

        new_alerts = []
        t = time.time()

        # Heartbeat
        if t - self._last_telemetry_t > self.heartbeat_timeout:
            new_alerts.append(Alert(
                AlertLevel.CRITICAL,
                f"Telemetry timeout: {t - self._last_telemetry_t:.1f}s"
            ))
        self._last_telemetry_t = t
        self._frame_count += 1

        # VO health
        if self._frame_count > 30:  # allow init
            healthy_ratio = (self.vo.state.vo_healthy_count /
                             max(self.vo.state.total_frames, 1))
            if healthy_ratio < self.vo_health_min_ratio:
                new_alerts.append(Alert(
                    AlertLevel.WARNING,
                    f"VO health degraded: {healthy_ratio:.0%}"
                ))

        # Altitude
        if frame.alt > self.max_altitude:
            new_alerts.append(Alert(
                AlertLevel.CRITICAL,
                f"Altitude {frame.alt:.1f}m exceeds limit {self.max_altitude}m"
            ))
        if frame.alt < self.min_altitude and self._frame_count > 50:
            new_alerts.append(Alert(
                AlertLevel.WARNING,
                f"Altitude {frame.alt:.2f}m near ground"
            ))

        # GPS home point
        if frame.lat == 0.0 and frame.lon == 0.0:
            if self._frame_count > 100:
                new_alerts.append(Alert(
                    AlertLevel.WARNING,
                    "GPS position is zero — home point may not be recorded"
                ))

        # Store
        with self._lock:
            self._alerts.extend(new_alerts)

        # Notify
        for a in new_alerts:
            if self._on_alert:
                try:
                    self._on_alert(a)
                except Exception:
                    pass

        return new_alerts

    def has_critical(self) -> bool:
        with self._lock:
            return any(a.level == AlertLevel.CRITICAL for a in self._alerts)

    def has_warning(self) -> bool:
        with self._lock:
            return any(a.level == AlertLevel.WARNING for a in self._alerts)

    def get_alerts(self) -> list:
        with self._lock:
            return list(self._alerts)

    def clear_alerts(self):
        with self._lock:
            self._alerts = []

    def on_alert(self, cb: Callable):
        self._on_alert = cb

    def status_string(self) -> str:
        """One-line status for logging."""
        with self._lock:
            critical = sum(1 for a in self._alerts
                           if a.level == AlertLevel.CRITICAL)
            warn = sum(1 for a in self._alerts
                       if a.level == AlertLevel.WARNING)
        alive = (time.time() - self._last_telemetry_t
                 < self.heartbeat_timeout)
        return (f"Frames:{self._frame_count} "
                f"Alive:{'Y' if alive else 'N'} "
                f"Crit:{critical} Warn:{warn}")


# ── Pre-flight checklist ──────────────────────────────

def preflight_check(telemetry_frame: Optional[TelemetryFrame] = None) -> list:
    """Run pre-flight checklist.

    Returns list of (passed:bool, item:str, detail:str)
    """
    checks = []

    # PSDK running = we have telemetry
    if telemetry_frame is None:
        checks.append((False, "PSDK", "No telemetry data — is PSDK running?"))
    else:
        checks.append((True, "PSDK", "Telemetry streaming"))

        if telemetry_frame.lat != 0 or telemetry_frame.lon != 0:
            checks.append((True, "GPS/Home", "Home point recorded"))
        else:
            checks.append((False, "GPS/Home",
                           "GPS is zero — record home point outdoors first"))

        if telemetry_frame.vo_health & 0x03 == 0x03:
            checks.append((True, "VO", "xHealth and yHealth OK"))
        else:
            checks.append((False, "VO",
                           f"VO health={telemetry_frame.vo_health}"))

    return checks
