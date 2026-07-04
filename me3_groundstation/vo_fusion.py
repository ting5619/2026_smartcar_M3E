"""
VO + Gyro + AprilTag fusion localization.

Principle:
  POSITION_VO (PSDK topic 41) provides x,y,z from DJI's internal fusion
  of omnidirectional vision + ultrasonic + IMU + barometer + IR ToF.
  The X-Y axes are LABELED as NED by the flight controller, but the
  compass heading used for that label is unreliable indoors.

  Solution:
    1. Take frame-to-frame DELTA (pure visual, independent of compass)
    2. Rotate NED delta -> BODY using quaternion (geometrically valid)
    3. Integrate gyro for heading (no compass needed)
    4. Rotate BODY delta -> MAP using gyro heading
    5. AprilTag periodically resets absolute position

Usage:
    fusion = VOFusion()
    fusion.start()

    # In telemetry callback:
    fusion.feed_vo(vo_x, vo_y, vo_z, vo_health, timestamp)
    fusion.feed_quaternion(q0, q1, q2, q3, timestamp)
    fusion.feed_gyro(gx, gy, gz, timestamp)
    fusion.feed_height(height_fused, timestamp)

    x, y, z = fusion.get_position()
    heading = fusion.get_heading()
"""

import time
import math
import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np


@dataclass
class FusionState:
    """Internal state of the fusion filter."""

    # Position in MAP frame (origin = first valid VO reading)
    pos_x: float = 0.0  # m
    pos_y: float = 0.0  # m
    pos_z: float = 0.0  # m (ENU: up is positive)

    # Heading from gyro integration (rad)
    heading: float = 0.0

    # Quaternion [w, x, y, z] from DJI EKF
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))

    # Gyro bias (rad/s), calibrated during initialization
    gyro_z_bias: float = 0.0

    # Last values for delta computation
    last_vo_x: float = 0.0
    last_vo_y: float = 0.0
    last_vo_z: float = 0.0
    last_timestamp: float = 0.0

    # Initialization
    vo_initialized: bool = False
    gyro_calibrated: bool = False
    init_samples_collected: int = 0
    init_required: int = 30  # samples (~3 seconds at 10Hz)

    # Health tracking
    vo_healthy_count: int = 0
    vo_degraded_count: int = 0
    total_frames: int = 0

    # Drift estimation
    vo_drift_accumulated: float = 0.0  # m, since last AprilTag correction
    time_since_correction: float = 0.0  # seconds

    # Last AprilTag correction
    last_apriltag_t: float = 0.0
    apriltag_corrections: int = 0


@dataclass
class FusionConfig:
    """Configuration for VOFusion."""

    # Minimum health bits: bit0=xHealth, bit1=yHealth, bit2=zHealth
    # Default 3 = xHealth AND yHealth required
    min_health_mask: int = 0x03

    # Initialization: number of samples to collect before starting
    init_samples: int = 30  # ~3 seconds at 10Hz

    # Gyro bias calibration: samples to average during init
    gyro_calib_samples: int = 50

    # Reject vo jumps larger than this (m) — likely compass glitch
    max_vo_jump: float = 2.0

    # Drift warning threshold (m)
    drift_warning: float = 1.0

    # AprilTag correction timeout: warn if no correction for this long (s)
    correction_timeout: float = 30.0


class VOFusion:
    """
    Indoor localization without magnetometer dependence.

    Fuses:
      - POSITION_VO frame delta (visual feature matching, no compass)
      - Quaternion rotation (gravity-referenced pitch/roll, reliable)
      - Gyro z-axis integration (short-term heading, no compass)
      - AprilTag for absolute position reset

    Not fused, but monitored:
      - Height (from HEIGHT_FUSION if available, else VO_z)
    """

    def __init__(self, config: Optional[FusionConfig] = None):
        self.config = config or FusionConfig()
        self.state = FusionState()
        self.init_required = self.config.init_samples

        # Gyro bias calibration buffer
        self._gyro_z_buffer: list = []

        # Lock for thread safety
        self._lock = threading.Lock()

        # Callbacks
        self._on_corrected = None  # called when AprilTag resets position

    def start(self):
        """Start the fusion engine. Call after construction."""
        self._reset_state()

    def reset(self):
        """Reset all state (e.g. after re-localization)."""
        with self._lock:
            self._reset_state()

    def _reset_state(self):
        self.state = FusionState()
        self.state.init_required = self.config.init_samples
        self._gyro_z_buffer = []

    # ─── Feed methods (called from telemetry callback) ───

    def feed_vo(self, vo_x: float, vo_y: float, vo_z: float,
                vo_health: int, timestamp: float):
        """
        Feed a POSITION_VO reading.

        Args:
            vo_x, vo_y, vo_z: VO position in NED frame (m)
            vo_health: bit0=xHealth, bit1=yHealth, bit2=zHealth
            timestamp: monotonic seconds
        """
        with self._lock:
            self.state.total_frames += 1

            # Check health
            if (vo_health & self.config.min_health_mask) != self.config.min_health_mask:
                self.state.vo_degraded_count += 1
                return

            self.state.vo_healthy_count += 1

            # Initialization phase: collect samples, don't integrate yet
            if self.state.init_required > 0:
                self.state.last_vo_x = vo_x
                self.state.last_vo_y = vo_y
                self.state.last_vo_z = vo_z
                self.state.last_timestamp = timestamp
                self.state.init_required -= 1
                if self.state.init_required == 0:
                    self.state.vo_initialized = True
                return

            if not self.state.vo_initialized:
                self.state.last_vo_x = vo_x
                self.state.last_vo_y = vo_y
                self.state.last_vo_z = vo_z
                self.state.last_timestamp = timestamp
                self.state.vo_initialized = True
                return

            # ① Compute NED delta (labeled as NED, but delta is pure visual)
            d_ned = np.array([
                vo_x - self.state.last_vo_x,
                vo_y - self.state.last_vo_y,
                vo_z - self.state.last_vo_z,
            ])

            # Save for next frame
            self.state.last_vo_x = vo_x
            self.state.last_vo_y = vo_y
            self.state.last_vo_z = vo_z

            # Reject jumps (compass glitch or VO tracking loss)
            jump_mag = np.linalg.norm(d_ned)
            if jump_mag > self.config.max_vo_jump:
                return  # discard this frame

            if jump_mag < 1e-6:
                return  # no movement, skip to avoid roundoff

            # ② Rotate NED -> BODY using quaternion
            #     The quaternion is geometrically self-consistent:
            #     pitch/roll from gravity, yaw from compass — but as a
            #     rotation matrix, the NED->BODY transform still works.
            d_body = self._quat_rotate_ned_to_body(self.state.q, d_ned)

            # ③ Rotate BODY -> MAP using gyro-integrated heading
            d_map = self._rotate_body_to_map(d_body, self.state.heading)

            # ④ Accumulate (convert Z: NED Down -> ENU Up)
            self.state.pos_x += d_map[0]
            self.state.pos_y += d_map[1]
            self.state.pos_z -= d_map[2]  # Down -> Up

            # ⑤ Track drift
            dt = timestamp - self.state.last_timestamp if self.state.last_timestamp > 0 else 0.1
            self.state.last_timestamp = timestamp
            self.state.vo_drift_accumulated += math.sqrt(d_map[0]**2 + d_map[1]**2)
            self.state.time_since_correction += dt

    def feed_quaternion(self, q0: float, q1: float, q2: float, q3: float,
                        timestamp: float):
        """Feed attitude quaternion [w, x, y, z] from DJI EKF."""
        with self._lock:
            self.state.q = np.array([q0, q1, q2, q3], dtype=np.float64)

    def feed_gyro(self, gx: float, gy: float, gz: float, timestamp: float):
        """
        Feed gyro angular velocity (rad/s).

        During initialization, collects samples for bias calibration.
        After calibration, integrates heading.
        """
        with self._lock:
            if not self.state.gyro_calibrated:
                self._gyro_z_buffer.append(gz)
                self.state.gyro_calib_samples = len(self._gyro_z_buffer)
                if len(self._gyro_z_buffer) >= self.config.gyro_calib_samples:
                    self.state.gyro_z_bias = np.mean(self._gyro_z_buffer)
                    self.state.gyro_calibrated = True
                    self.state.last_timestamp = timestamp
                return

            dt = timestamp - self.state.last_timestamp
            if dt <= 0 or dt > 1.0:
                self.state.last_timestamp = timestamp
                return

            self.state.heading += (gz - self.state.gyro_z_bias) * dt
            self.state.heading = self._normalize_angle(self.state.heading)
            self.state.last_timestamp = timestamp

    def feed_height(self, height: float):
        """Feed fused height (from HEIGHT_FUSION or barometer). Overrides VO_z."""
        with self._lock:
            self.state.pos_z = height

    # ─── Correction ───

    def correct_by_apriltag(self, tag_world_x: float, tag_world_y: float,
                            tag_heading: Optional[float] = None):
        """
        Reset position to known AprilTag coordinates.

        Args:
            tag_world_x, tag_world_y: tag position in map frame (m)
            tag_heading: optional tag orientation for heading correction
        """
        with self._lock:
            self.state.pos_x = tag_world_x
            self.state.pos_y = tag_world_y
            if tag_heading is not None:
                self.state.heading = tag_heading
            self.state.vo_drift_accumulated = 0.0
            self.state.time_since_correction = 0.0
            self.state.apriltag_corrections += 1

            if self._on_corrected:
                self._on_corrected(self.state)

    def set_on_corrected_callback(self, callback):
        """Set callback called after each AprilTag correction."""
        self._on_corrected = callback

    # ─── Getters ───

    def get_position(self) -> Tuple[float, float, float]:
        """Get current position in map frame (x, y, z) in meters, ENU."""
        with self._lock:
            return (self.state.pos_x, self.state.pos_y, self.state.pos_z)

    def get_heading(self) -> float:
        """Get current heading in radians."""
        with self._lock:
            return self.state.heading

    def is_healthy(self) -> bool:
        """Check if VO is currently healthy."""
        with self._lock:
            if self.state.total_frames == 0:
                return False
            return (self.state.vo_healthy_count /
                    max(self.state.total_frames, 1)) > 0.7

    def needs_correction(self) -> bool:
        """Check if drift has accumulated enough to need AprilTag correction."""
        with self._lock:
            return (self.state.vo_drift_accumulated > self.config.drift_warning
                    or self.state.time_since_correction > self.config.correction_timeout)

    def get_state_dict(self) -> dict:
        """Get full state as dict for logging/debugging."""
        with self._lock:
            return {
                "pos_x": round(self.state.pos_x, 4),
                "pos_y": round(self.state.pos_y, 4),
                "pos_z": round(self.state.pos_z, 4),
                "heading_deg": round(math.degrees(self.state.heading), 2),
                "vo_initialized": self.state.vo_initialized,
                "gyro_calibrated": self.state.gyro_calibrated,
                "gyro_bias": round(self.state.gyro_z_bias, 6),
                "drift_m": round(self.state.vo_drift_accumulated, 3),
                "time_since_correction": round(self.state.time_since_correction, 1),
                "apriltag_corrections": self.state.apriltag_corrections,
                "healthy_ratio": (
                    self.state.vo_healthy_count / max(self.state.total_frames, 1)
                ),
            }

    # ─── Quaternion math ───

    @staticmethod
    def _quat_rotate_ned_to_body(q: np.ndarray, v_ned: np.ndarray) -> np.ndarray:
        """
        Rotate a vector from NED frame to BODY frame using quaternion.

        q = [w, x, y, z] rotates BODY -> NED (DJI convention: Hamilton).
        To rotate NED -> BODY, we use q_conjugate.

        q_conj * [0, v] * q
        """
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        v_quat = np.array([0.0, v_ned[0], v_ned[1], v_ned[2]])

        tmp = VOFusion._quat_mul(q_conj, v_quat)
        result = VOFusion._quat_mul(tmp, q)
        return result[1:]  # drop scalar part

    @staticmethod
    def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Quaternion multiplication (Hamilton)."""
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    @staticmethod
    def _rotate_body_to_map(v_body: np.ndarray, heading: float) -> np.ndarray:
        """Rotate BODY vector to MAP frame using gyro-integrated heading."""
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        return np.array([
            cos_h * v_body[0] - sin_h * v_body[1],
            sin_h * v_body[0] + cos_h * v_body[1],
            v_body[2],
        ])

    @staticmethod
    def _normalize_angle(a: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return ((a + math.pi) % (2 * math.pi)) - math.pi


# ─── Convenience: quick test with recorded data ───

if __name__ == "__main__":
    import json, sys

    if len(sys.argv) < 2:
        print("Usage: python vo_fusion.py <vo_data.json>")
        print("  vo_data.json: recorded from onboard box via vo_rec.py")
        sys.exit(1)

    with open(sys.argv[1]) as f:
        data = json.load(f)

    fusion = VOFusion()
    fusion.start()

    positions = []

    for d in data:
        t = d["t"]
        fusion.feed_vo(d["vx"], d["vy"], d["vz"], d["vh"], t)
        fusion.feed_gyro(d["gx"], d["gy"], d["gz"], t)
        # Note: quaternion not in recorded data, using identity
        # In real use, feed quaternion from /uavdata.quat
        x, y, z = fusion.get_position()
        positions.append([t, x, y, z])

    positions = np.array(positions)
    print(f"Processed {len(data)} frames")
    print(f"State: {json.dumps(fusion.get_state_dict(), indent=2)}")
    print(f"Final position: ({positions[-1,1]:.3f}, {positions[-1,2]:.3f}, {positions[-1,3]:.3f})")
    print(f"Total path: {np.sum(np.sqrt(np.diff(positions[:,1])**2 + np.diff(positions[:,2])**2)):.2f} m")
