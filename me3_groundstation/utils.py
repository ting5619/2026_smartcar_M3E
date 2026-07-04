"""
Coordinate and math utilities for ME3 ground station.

Conversions between NED, BODY, ENU, GPS, and local map frames.
"""

import math
import numpy as np


# ── Constants ──────────────────────────────────────────

WGS84_A = 6378137.0          # Semi-major axis (m)
WGS84_F = 1.0 / 298.257223563  # Flattening
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2  # First eccentricity squared

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


# ── Angle normalization ────────────────────────────────

def normalize_angle(rad: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return ((rad + math.pi) % (2 * math.pi)) - math.pi


def angle_diff(a: float, b: float) -> float:
    """Shortest angular difference a - b, in [-pi, pi]."""
    return normalize_angle(a - b)


# ── Quaternion operations ──────────────────────────────

def quat_to_euler(q0: float, q1: float, q2: float, q3: float
                  ) -> tuple[float, float, float]:
    """
    Convert Hamilton quaternion [w,x,y,z] to Euler angles (deg).

    Returns (roll, pitch, yaw) in degrees.
    Roll/Pitch are reliable (gravity-referenced).
    Yaw is magnetometer-referenced — unreliable indoors.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def quat_rotate(q_xyzw: tuple, v: np.ndarray) -> np.ndarray:
    """
    Rotate vector v by quaternion q (BODY -> NED).
    q_xyzw = (x, y, z, w) in DJI convention.

    Returns rotated vector in NED.
    """
    qw, qx, qy, qz = q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]

    # Rotation matrix from quaternion
    r00 = 1 - 2*(qy**2 + qz**2)
    r01 = 2*(qx*qy - qz*qw)
    r02 = 2*(qx*qz + qy*qw)
    r10 = 2*(qx*qy + qz*qw)
    r11 = 1 - 2*(qx**2 + qz**2)
    r12 = 2*(qy*qz - qx*qw)
    r20 = 2*(qx*qz - qy*qw)
    r21 = 2*(qy*qz + qx*qw)
    r22 = 1 - 2*(qx**2 + qy**2)

    return np.array([
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ])


# ── Frame conversions ──────────────────────────────────

def ned_to_enu(v_ned: np.ndarray) -> np.ndarray:
    """NED (North-East-Down) -> ENU (East-North-Up)."""
    return np.array([v_ned[1], v_ned[0], -v_ned[2]])


def enu_to_ned(v_enu: np.ndarray) -> np.ndarray:
    """ENU (East-North-Up) -> NED (North-East-Down)."""
    return np.array([v_enu[1], v_enu[0], -v_enu[2]])


def body_to_map_2d(v_body: np.ndarray, heading: float) -> np.ndarray:
    """
    Rotate 2D vector from BODY to MAP frame using heading.
    heading: radians, 0 = map X axis.
    """
    c = math.cos(heading)
    s = math.sin(heading)
    return np.array([
        c * v_body[0] - s * v_body[1],
        s * v_body[0] + c * v_body[1],
    ])


# ── GPS utilities ─────────────────────────────────────

def gps_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple:
    """
    Compute distance and bearing between two GPS coordinates.

    Returns (distance_m, bearing_deg).
    Using Haversine formula.
    """
    lat1_r = lat1 * DEG_TO_RAD
    lat2_r = lat2 * DEG_TO_RAD
    dlat = (lat2 - lat1) * DEG_TO_RAD
    dlon = (lon2 - lon1) * DEG_TO_RAD

    a = (math.sin(dlat/2)**2 +
         math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon/2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = WGS84_A * c

    # Bearing
    y = math.sin(dlon) * math.cos(lat2_r)
    x = (math.cos(lat1_r) * math.sin(lat2_r) -
         math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))
    bearing = math.degrees(math.atan2(y, x))

    return (distance, normalize_angle(bearing * DEG_TO_RAD))


def gps_to_local_origin(ref_lat: float, ref_lon: float, ref_alt: float
                        ) -> tuple:
    """
    Return the local ENU origin for converting GPS to local coordinates.

    Returns (ref_lat_rad, ref_lon_rad, Rn, Re) where Rn,Re are radii
    of curvature for N,E displacement calculations.
    """
    lat_r = ref_lat * DEG_TO_RAD

    # Radius of curvature in prime vertical
    sin_lat = math.sin(lat_r)
    Rn = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)

    # Radius of curvature in meridian
    Re = WGS84_A * (1 - WGS84_E2) / ((1 - WGS84_E2 * sin_lat * sin_lat) ** 1.5)

    return (lat_r, ref_lon * DEG_TO_RAD, ref_alt, Rn, Re)


def gps_to_enu(lat: float, lon: float, alt: float,
               origin: tuple) -> np.ndarray:
    """Convert GPS to ENU relative to origin."""
    lat_r_o, lon_r_o, alt_o, Rn, Re = origin

    dlat = (lat - math.degrees(lat_r_o)) * DEG_TO_RAD
    dlon = (lon - math.degrees(lon_r_o * RAD_TO_DEG)) * DEG_TO_RAD
    dalt = alt - alt_o

    # Approximate local ENU
    e = dlon * Rn * math.cos(lat_r_o)
    n = dlat * Re
    u = dalt

    return np.array([e, n, u])


# ── Signal processing ──────────────────────────────────

class LowPassFilter:
    """Simple first-order low-pass filter."""

    def __init__(self, alpha: float = 0.2):
        self.alpha = alpha
        self._value: float = 0.0
        self._initialized = False

    def update(self, value: float) -> float:
        if not self._initialized:
            self._value = value
            self._initialized = True
        else:
            self._value = self.alpha * value + (1 - self.alpha) * self._value
        return self._value

    def reset(self):
        self._value = 0.0
        self._initialized = False

    @property
    def value(self) -> float:
        return self._value


class MovingAverage:
    """Moving average filter with fixed window."""

    def __init__(self, window_size: int = 10):
        self.window = window_size
        self._buffer: list = []

    def update(self, value: float) -> float:
        self._buffer.append(value)
        if len(self._buffer) > self.window:
            self._buffer.pop(0)
        return sum(self._buffer) / len(self._buffer)

    def reset(self):
        self._buffer = []

    @property
    def value(self) -> float:
        if not self._buffer:
            return 0.0
        return sum(self._buffer) / len(self._buffer)
