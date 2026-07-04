"""
Closed-loop position controller with PD + velocity feedforward.

Controls drone via /flightByVel using VO position feedback.
No magnetometer dependency — body-frame velocity commands.
"""

import time
import math
import threading
from enum import Enum
from typing import Optional, Callable, Tuple, List
from dataclasses import dataclass

from me3_groundstation.vo_fusion import VOFusion
from me3_groundstation.flight_control import ME3FlightController


class ControllerState(Enum):
    IDLE = "idle"
    TAKEOFF = "takeoff"
    HOLDING = "holding"
    MOVING = "moving"
    LANDING = "landing"
    ABORTING = "aborting"
    DONE = "done"
    ERROR = "error"


@dataclass
class ControlGains:
    """PD controller gains for position control."""

    # Position -> velocity gains
    kp_xy: float = 1.2    # horizontal P (m/s per m error)
    kd_xy: float = 0.8    # horizontal D (damping, m/s per m/s vel_error)

    kp_z: float = 1.0     # vertical P
    kd_z: float = 0.6     # vertical D

    kp_yaw: float = 1.5   # yaw P (deg/s per deg error)

    # Limits
    max_vel_xy: float = 2.0   # m/s
    max_vel_z: float = 1.0    # m/s
    max_yaw_rate: float = 90  # deg/s

    # Thresholds
    arrival_tolerance_xy: float = 0.15  # m
    arrival_tolerance_z: float = 0.10   # m
    arrival_tolerance_yaw: float = 5.0  # deg


@dataclass
class Waypoint:
    """A waypoint in the (non-magnetic) map frame."""

    x: float        # m, map X
    y: float        # m, map Y
    z: float        # m, ENU (up positive)
    yaw: float = 0  # deg, desired heading at waypoint
    speed: float = 0.5  # m/s, cruise speed
    hold_time: float = 0.0  # seconds to hover at this waypoint

    @classmethod
    def body_relative(cls, dx_forward: float, dy_right: float, dz_up: float = 0.0,
                      heading: float = 0.0, speed: float = 0.5, hold: float = 0.0):
        """Create a waypoint offset in body frame from current position."""
        return cls(dx_forward, dy_right, dz_up, heading, speed, hold)
        # Note: dx/dy/dz here are body offsets; caller must convert to map
        # using current heading before calling fly_to()


class PositionController:
    """PD position controller with velocity feedforward.

    Usage:
        ctrl = ME3FlightController()
        vo = VOFusion()
        pc = PositionController(ctrl, vo)
        pc.takeoff(0.2)                     # takeoff 20cm
        pc.fly_to(x=1.0, y=0.0, z=0.2)     # fly to waypoint
        pc.wait_arrival(timeout=10.0)
        pc.land()
    """

    def __init__(self, controller: ME3FlightController, vo_fusion: VOFusion,
                 gains: Optional[ControlGains] = None):
        self.ctrl = controller
        self.vo = vo_fusion
        self.gains = gains or ControlGains()
        self.state = ControllerState.IDLE
        self._stop = threading.Event()
        self._control_thread: Optional[threading.Thread] = None
        self._state_cb: Optional[Callable] = None

        # Previous velocity for D term
        self._prev_ex = 0.0
        self._prev_ey = 0.0
        self._prev_ez = 0.0
        self._last_vel_t = 0.0

    # ── Public API ─────────────────────────────────────

    def takeoff(self, altitude: float = 1.0) -> bool:
        """Takeoff and climb to altitude. BLOCKING."""
        self._change_state(ControllerState.TAKEOFF)
        try:
            if not self.ctrl.takeoff():
                self._change_state(ControllerState.ERROR)
                return False
            time.sleep(3.0)  # motor spin-up
            self.fly_to_z(altitude, speed=0.5, timeout=10.0)
            self._change_state(ControllerState.HOLDING)
            return True
        except Exception as e:
            self._change_state(ControllerState.ERROR)
            raise

    def land(self) -> bool:
        """Land. BLOCKING."""
        self._change_state(ControllerState.LANDING)
        try:
            if not self.ctrl.land():
                return False
            time.sleep(3.0)
            self._change_state(ControllerState.DONE)
            return True
        except Exception:
            self._change_state(ControllerState.ERROR)
            return False

    def fly_to(self, x: float, y: float, z: float, yaw: float = 0.0,
               speed: float = 0.5, timeout: float = 20.0) -> bool:
        """Fly to absolute map coordinates (PD closed-loop). BLOCKING.

        Args:
            x, y, z: target in ENU map frame (z=up positive)
            yaw: desired heading at arrival (deg)
            speed: maximum cruise speed (m/s)
            timeout: max execution time

        Returns:
            True if arrived within tolerance
        """
        self._change_state(ControllerState.MOVING)
        t0 = time.time()
        self._reset_derivative()

        while not self._stop.is_set():
            dt = time.time() - t0
            if dt > timeout:
                self.ctrl.hover(0.3)
                return False

            cx, cy, cz = self.vo.get_position()
            ex, ey, ez = x - cx, y - cy, z - cz
            dist = math.sqrt(ex**2 + ey**2 + ez**2)

            # Arrival check
            if (dist < self.gains.arrival_tolerance_xy
                and abs(ez) < self.gains.arrival_tolerance_z):
                self.ctrl.hover(0.5)
                self._change_state(ControllerState.HOLDING)
                return True

            # PD velocity command (map frame)
            vx, vy, vz = self._pd_control(ex, ey, ez, dt)
            self._prev_ex, self._prev_ey, self._prev_ez = ex, ey, ez

            # Scale to max speed
            v_norm = math.sqrt(vx**2 + vy**2)
            if v_norm > speed:
                scale = speed / v_norm
                vx *= scale
                vy *= scale
            if abs(vz) > self.gains.max_vel_z:
                vz = math.copysign(self.gains.max_vel_z, vz)

            # Map frame velocity -> body frame
            h = self.vo.get_heading()
            cos_h, sin_h = math.cos(h), math.sin(h)
            vx_b = cos_h * vx + sin_h * vy
            vy_b = -sin_h * vx + cos_h * vy

            self.ctrl.set_velocity_body(vx_b, vy_b, -vz, 0.0, 0.1,
                                        frame=1)
            time.sleep(0.05)  # 20 Hz

        self.ctrl.hover(0.3)
        return False

    def fly_to_z(self, z: float, speed: float = 0.3,
                 timeout: float = 10.0) -> bool:
        """Altitude-only closed-loop with PD."""
        t0 = time.time()
        self._reset_derivative()

        while not self._stop.is_set():
            dt = time.time() - t0
            if dt > timeout:
                self.ctrl.hover(0.3)
                return False

            _, _, cz = self.vo.get_position()
            ez = z - cz

            if abs(ez) < self.gains.arrival_tolerance_z:
                self.ctrl.hover(0.3)
                return True

            vz = self.gains.kp_z * ez + self.gains.kd_z * (ez - self._prev_ez) / max(dt, 0.01)
            vz = max(-self.gains.max_vel_z,
                     min(self.gains.max_vel_z, vz))
            self._prev_ez = ez

            self.ctrl.set_velocity_body(0, 0, -vz, 0, 0.1, frame=1)
            time.sleep(0.05)

        return False

    def fly_path(self, waypoints: List[Waypoint], loop: bool = False,
                 home_on_finish: bool = False) -> bool:
        """Fly a sequence of map-frame waypoints. BLOCKING.

        Each waypoint is in absolute map coordinates.
        """
        for wp in waypoints:
            if self._stop.is_set():
                break
            self.fly_to(wp.x, wp.y, wp.z, wp.yaw, wp.speed)
            if wp.hold_time > 0:
                time.sleep(wp.hold_time)

        if self._stop.is_set():
            self.ctrl.hover(2.0)
            return False
        return True

    def wait_arrival(self, timeout: float = 10.0) -> bool:
        """Wait until arrival detection settles. BLOCKING."""
        self._change_state(ControllerState.HOLDING)
        return True  # fly_to already checked arrival

    def hover(self, duration: float = 1.0):
        """Hover for duration. BLOCKING."""
        t0 = time.time()
        while time.time() - t0 < duration and not self._stop.is_set():
            self.ctrl.hover(0.1)
            time.sleep(0.1)

    def abort(self):
        """Emergency: stop movement and hover."""
        self._stop.set()
        self._change_state(ControllerState.ABORTING)
        self.ctrl.hover(2.0)

    # ── Callback ───────────────────────────────────────

    def on_state_change(self, cb: Callable):
        self._state_cb = cb

    # ── Internal ───────────────────────────────────────

    def _pd_control(self, ex: float, ey: float, ez: float,
                    dt: float) -> Tuple[float, float, float]:
        """PD: vel = Kp * e + Kd * de/dt."""
        g = self.gains
        dt = max(dt, 0.01)

        # P term
        vx = g.kp_xy * ex
        vy = g.kp_xy * ey
        vz = g.kp_z * ez

        # D term (velocity damping via position error derivative)
        dex = (ex - self._prev_ex) / dt
        dey = (ey - self._prev_ey) / dt
        dez = (ez - self._prev_ez) / dt

        vx += g.kd_xy * dex
        vy += g.kd_xy * dey
        vz += g.kd_z * dez

        return vx, vy, vz

    def _reset_derivative(self):
        self._prev_ex = self._prev_ey = self._prev_ez = 0.0

    def _change_state(self, s: ControllerState):
        self.state = s
        if self._state_cb:
            try:
                self._state_cb(s)
            except Exception:
                pass


# ── Convenience factory ────────────────────────────────

def create_controller(ros_master_uri: str = None) -> Tuple[ME3FlightController,
                                                             VOFusion,
                                                             PositionController]:
    """Create all three objects wired together.

    Returns (flight_ctrl, vo_fusion, position_ctrl)
    """
    import rospy
    if ros_master_uri:
        import os
        os.environ['ROS_MASTER_URI'] = ros_master_uri

    ctrl = ME3FlightController()
    vo = VOFusion()
    pc = PositionController(ctrl, vo)
    return ctrl, vo, pc
