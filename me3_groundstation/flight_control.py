"""
Flight control interface for ME3 drone.

Publishes to /flightByVel and calls /takeoffOrLanding, /gimbalControl.
No magnetometer needed — supports BODY frame velocity commands.
"""

import time
import rospy
from tta_m3e_rtsp.msg import flightByVel
from tta_m3e_rtsp.srv import takeoffOrLanding, gimbalControl


class ME3FlightController:
    """Direct flight control via ROS topics/services.

    Usage:
        ctrl = ME3FlightController()
        ctrl.takeoff()
        ctrl.set_velocity_body(1.0, 0, 0)  # forward 1 m/s
        time.sleep(2)
        ctrl.hover()
        ctrl.land()
    """

    def __init__(self):
        self._vel_pub = rospy.Publisher("/flightByVel", flightByVel, queue_size=10)
        time.sleep(0.5)  # wait for publisher to register

        rospy.wait_for_service("/takeoffOrLanding", timeout=10)
        self._takeoff_srv = rospy.ServiceProxy("/takeoffOrLanding", takeoffOrLanding)

        try:
            rospy.wait_for_service("/gimbalControl", timeout=3)
            self._gimbal_srv = rospy.ServiceProxy("/gimbalControl", gimbalControl)
        except:
            self._gimbal_srv = None

    # ── Basic flight ──────────────────────────────────

    def takeoff(self) -> bool:
        """Request takeoff. Blocks until acknowledged."""
        try:
            resp = self._takeoff_srv(1)
            if resp.ack:
                rospy.loginfo("Takeoff command accepted")
                return True
            rospy.logerr("Takeoff rejected")
            return False
        except Exception as e:
            rospy.logerr(f"Takeoff service error: {e}")
            return False

    def land(self) -> bool:
        """Request landing."""
        try:
            resp = self._takeoff_srv(2)
            if resp.ack:
                rospy.loginfo("Land command accepted")
                return True
            return False
        except Exception as e:
            rospy.logerr(f"Land service error: {e}")
            return False

    # ── Velocity control ───────────────────────────────

    def set_velocity_body(self, vx: float, vy: float, vz: float,
                          yaw_rate: float = 0.0, duration: float = 0.1,
                          frame: int = 1):
        """Publish body-frame velocity (frame=1).

        BODY (FRU):
            vx: forward (positive = nose-forward)
            vy: right  (positive = right)
            vz: down   (positive = down, NEGATIVE = up!)
            yaw_rate: deg/s
            duration: command duration in seconds
            frame: always 1 (BODY mode)
        """
        msg = flightByVel()
        msg.vel_n = vx
        msg.vel_e = vy
        msg.vel_d = vz
        msg.targetYaw = yaw_rate
        msg.fly_time = duration
        msg.frame = frame
        self._vel_pub.publish(msg)

    def set_velocity_ned(self, vn: float, ve: float, vd: float,
                         yaw: float = 0.0, duration: float = 0.1):
        """Publish NED velocity (frame=0). Outdoor use."""
        msg = flightByVel()
        msg.vel_n = vn
        msg.vel_e = ve
        msg.vel_d = vd
        msg.targetYaw = yaw
        msg.fly_time = duration
        msg.frame = 0  # NED mode
        self._vel_pub.publish(msg)

    def hover(self, duration: float = 0.5):
        """Send zero velocity to hover."""
        msg = flightByVel()
        msg.vel_n = 0.0
        msg.vel_e = 0.0
        msg.vel_d = 0.0
        msg.targetYaw = 0.0
        msg.fly_time = duration
        msg.frame = 1  # BODY
        self._vel_pub.publish(msg)

    # ── Gimbal ─────────────────────────────────────────

    def set_gimbal(self, pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0):
        """Control gimbal angles (degrees)."""
        if not self._gimbal_srv:
            return
        self._gimbal_srv(pitch, roll, yaw)

    # ── Safety ─────────────────────────────────────────

    def emergency_stop(self):
        """Send repeated zero velocity."""
        for _ in range(10):
            self.hover(0.1)
            time.sleep(0.05)

    def stop_motors(self):
        """Land and stop."""
        self.land()
