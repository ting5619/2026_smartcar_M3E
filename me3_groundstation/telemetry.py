"""
Telemetry subscriber for ME3 drone.

Connects to the onboard box's ROS master and subscribes to /uavdata.
Provides parsed telemetry to the rest of the ground station stack.
"""

import time
import threading
from typing import Callable, Optional
from dataclasses import dataclass, field

import rospy
from tta_m3e_rtsp.msg import uavdata


@dataclass
class TelemetryFrame:
    """Parsed telemetry data from one /uavdata message."""

    # Timestamp (local monotonic seconds)
    timestamp: float = 0.0

    # GPS position (WGS84)
    lat: float = 0.0   # degrees
    lon: float = 0.0   # degrees
    alt: float = 0.0   # meters, WGS84 ellipsoid

    # Velocity (NED, m/s)
    vel_n: float = 0.0
    vel_e: float = 0.0
    vel_d: float = 0.0

    # Attitude (degrees)
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0

    # Angular velocity (rad/s, body frame)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0

    # Acceleration (m/s², raw body frame)
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0

    # Quaternion [w, x, y, z]
    quat: list = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])

    # VO fused position (from PSDK topic 41)
    vo_x: float = 0.0
    vo_y: float = 0.0
    vo_z: float = 0.0
    vo_health: int = 0


class Telemetry:
    """
    Subscribes to /uavdata and provides parsed telemetry.

    Usage:
        tel = Telemetry()
        tel.start()

        frame = tel.get_latest()
        # or
        tel.on_frame(callback)
    """

    def __init__(self, queue_size: int = 100):
        self._queue_size = queue_size
        self._sub = None
        self._latest: Optional[TelemetryFrame] = None
        self._lock = threading.Lock()
        self._callbacks: list = []
        self._running = False
        self._frame_count: int = 0
        self._start_time: float = 0.0

    def start(self):
        """Start subscribing to /uavdata."""
        if self._running:
            return

        self._sub = rospy.Subscriber(
            "/uavdata", uavdata, self._on_uavdata,
            queue_size=self._queue_size
        )
        self._running = True
        self._start_time = time.time()

    def stop(self):
        """Stop subscribing."""
        if self._sub:
            self._sub.unregister()
        self._running = False

    def on_frame(self, callback: Callable[[TelemetryFrame], None]):
        """Register a callback for every telemetry frame."""
        self._callbacks.append(callback)

    def get_latest(self) -> Optional[TelemetryFrame]:
        """Get the most recent telemetry frame (non-blocking)."""
        with self._lock:
            return self._latest

    def get_latest_blocking(self, timeout: float = 5.0) -> Optional[TelemetryFrame]:
        """Wait for a telemetry frame (blocking)."""
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                if self._latest is not None:
                    return self._latest
            time.sleep(0.05)
        return None

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def uptime(self) -> float:
        if not self._running:
            return 0.0
        return time.time() - self._start_time

    def _on_uavdata(self, msg: uavdata):
        """ROS callback: parse uavdata message."""
        frame = TelemetryFrame(
            timestamp=time.time(),
            lat=msg.latit,
            lon=msg.longi,
            alt=msg.altit,
            vel_n=msg.velN,
            vel_e=msg.velE,
            vel_d=msg.velD,
            pitch=msg.atti_pitch,
            roll=msg.atti_roll,
            yaw=msg.atti_yaw,
            gyro_x=msg.gyro_pitch,
            gyro_y=msg.gyro_roll,
            gyro_z=msg.gyro_yaw,
            acc_x=msg.accN,
            acc_y=msg.accE,
            acc_z=msg.accD,
            quat=list(msg.quat) if len(msg.quat) >= 4 else [1.0, 0.0, 0.0, 0.0],
            vo_x=msg.vo_x,
            vo_y=msg.vo_y,
            vo_z=msg.vo_z,
            vo_health=msg.vo_health,
        )

        with self._lock:
            self._latest = frame
            self._frame_count += 1

        for cb in self._callbacks:
            try:
                cb(frame)
            except Exception as e:
                rospy.logwarn(f"Telemetry callback error: {e}")


if __name__ == "__main__":
    rospy.init_node("telemetry_test", anonymous=True)
    tel = Telemetry()

    def print_frame(f: TelemetryFrame):
        print(f"VO: ({f.vo_x:.4f}, {f.vo_y:.4f}, {f.vo_z:.4f}) "
              f"health={f.vo_health} yaw={f.yaw:.1f} alt={f.alt:.1f}")

    tel.on_frame(print_frame)
    tel.start()

    rospy.spin()
