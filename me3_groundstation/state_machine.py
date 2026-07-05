"""
无人机端状态机 — 接收车辆 WiFi TCP 指令，执行飞行任务。

架构角色：本工程是执行端（Slave），车辆（YL-R8 Jetson Nano）是主控端（Master）。
车辆通过 WiFi TCP 发 JSON 指令给本机，本机解析后控制 M3E 无人机执行飞行。

通信协议（车辆 → 无人机）：
    {cmd: "takeoff", altitude: 0.5}              → 起飞到指定高度
    {cmd: "fly_to", x: 1.0, y: 0.5, z: 0.5}     → 飞向目标坐标
    {cmd: "return"}                               → 返航
    {cmd: "land"}                                 → 降落
    {cmd: "abort", reason: "manual"}              → 紧急停止

通信协议（无人机 → 车辆）：
    {status: "ok", drone_state: "hovering", position: [x,y,z], vo_healthy: true}
    {event: "arrived", waypoint_id: "wp1"}
    {event: "error", msg: "VO degraded"}

内部飞行控制链：
    状态机 → ME3FlightController.set_velocity_body() → ROS /flightByVel
           → fcProxy → PSDK → Dji_FlightControlVelocityAndYawRateCtrl → 飞控

Reference: YL-R8 rescue_sm.py design pattern (sequential step blocking execution,
timeout/retry/abort on each step, event callbacks).
"""

import time
import enum
import threading
from typing import Optional, Callable, Tuple
from dataclasses import dataclass, field

# These imports assume ROS environment is sourced
# and me3_groundstation is on PYTHONPATH
from me3_groundstation.flight_control import ME3FlightController
from me3_groundstation.vo_fusion import VOFusion
from me3_groundstation.path_executor import PositionController, ControlGains


# ── State definitions ─────────────────────────────────

class DroneState(enum.Enum):
    """无人机自身状态"""
    IDLE             = "idle"
    PRECHECK         = "precheck"
    TAKING_OFF       = "taking_off"
    HOVERING         = "hovering"
    HOVERING_AFTER_TAKEOFF = "hovering_after_takeoff"
    MOVING_TO_TARGET = "moving_to_target"
    HOLDING_POSITION = "holding_position"
    RETURNING        = "returning"
    LANDING          = "landing"
    EMERGENCY        = "emergency"
    DONE             = "done"
    ERROR            = "error"


class MissionState(enum.Enum):
    """车-机联合任务状态"""
    WAITING_FOR_CAR    = "waiting_for_car"      # 等待车辆指令
    CAR_REQUEST_TAKEOFF = "car_request_takeoff"  # 车辆请求起飞
    DRONE_TAKING_OFF   = "drone_taking_off"      # 无人机起飞中
    DRONE_GOING_TO_TARGET = "drone_going_to_target"  # 飞向目标
    DRONE_OVER_TARGET  = "drone_over_target"     # 到达目标上空
    DRONE_RETURNING    = "drone_returning"       # 返航中
    MISSION_COMPLETE   = "mission_complete"      # 任务完成
    MISSION_ABORTED    = "mission_aborted"       # 任务中止


# ── Data structures ───────────────────────────────────

@dataclass
class WaypointTask:
    """车辆发送给无人机的航点任务"""
    waypoint_id: str
    x: float      # map frame ENU, meters
    y: float
    z: float      # altitude above ground, meters
    hold_time: float = 3.0     # hover after arrival
    speed: float = 0.5         # cruise speed m/s
    description: str = ""


@dataclass
class MissionResult:
    """任务执行结果"""
    success: bool
    final_state: DroneState
    total_flight_time: float = 0.0
    error_message: str = ""
    score: int = 0


# ── Callback types ────────────────────────────────────

StateCallback = Callable[[DroneState, MissionState], None]
LogCallback = Callable[[str], None]
CommandCallback = Callable[[str, dict], bool]  # (cmd_name, params) -> success


# ── State Machine ─────────────────────────────────────

class DroneStateMachine:
    """
    Drone state machine — sequential blocking execution.

    Usage:
        sm = DroneStateMachine(flight_ctrl, vo_fusion, position_ctrl)
        sm.configure(waypoint)
        sm.execute()  # blocking until done or error

    Communication protocol (car -> drone):
        {cmd: "takeoff"}                         -> takeoff
        {cmd: "fly_to", x:1.0, y:0.5, z:0.5}    -> fly to waypoint
        {cmd: "return"}                          -> return to home
        {cmd: "land"}                            -> land
        {cmd: "abort"}                           -> emergency stop

    Communication protocol (drone -> car):
        {state: "hovering", pos: [x,y,z], bat:66} -> status heartbeat
        {event: "arrived", waypoint_id: "wp1"}    -> arrival notification
        {event: "error", msg: "VO degraded"}     -> error notification
    """

    def __init__(
        self,
        flight_ctrl: ME3FlightController,
        vo_fusion: VOFusion,
        position_ctrl: Optional[PositionController] = None,
    ):
        self.ctrl = flight_ctrl
        self.vo = vo_fusion
        self.pc = position_ctrl or PositionController(
            flight_ctrl, vo_fusion,
            gains=ControlGains(
                kp_xy=1.0, kd_xy=0.5,
                kp_z=1.0, kd_z=0.4,
                max_vel_xy=1.5, max_vel_z=0.8,
                arrival_tolerance_xy=0.15,
                arrival_tolerance_z=0.10,
            )
        )

        self.drone_state = DroneState.IDLE
        self.mission_state = MissionState.WAITING_FOR_CAR

        # Mission parameters
        self._target_altitude: float = 1.0
        self._waypoints: list = []
        self._home_position: Optional[Tuple[float, float, float]] = None
        self._mission_start_time: float = 0.0

        # Execution control
        self._stop_flag = threading.Event()
        self._pause_flag = threading.Event()
        self._exec_thread: Optional[threading.Thread] = None

        # Callbacks
        self.on_state_changed: Optional[StateCallback] = None
        self.on_log: Optional[LogCallback] = None
        self.on_score_changed: Optional[Callable[[int, str], None]] = None

        # Safety
        self._min_battery: float = 15.0
        self._max_flight_time: float = 300.0  # 5 minutes

    # ── Public API ─────────────────────────────────────

    def configure(
        self,
        target_altitude: float = 1.0,
        waypoints: Optional[list] = None,
    ):
        """Configure mission parameters before execute()."""
        self._target_altitude = target_altitude
        self._waypoints = waypoints or []

    def execute(self, blocking: bool = True) -> MissionResult:
        """Execute the configured mission. Optionally non-blocking."""
        if blocking:
            return self._run_mission()
        else:
            self._stop_flag.clear()
            self._exec_thread = threading.Thread(target=self._run_mission, daemon=True)
            self._exec_thread.start()
            return MissionResult(success=True, final_state=DroneState.IDLE)

    def wait(self, timeout: Optional[float] = None) -> MissionResult:
        """Wait for async mission to complete."""
        if self._exec_thread and self._exec_thread.is_alive():
            self._exec_thread.join(timeout=timeout)
        return MissionResult(
            success=(self.drone_state == DroneState.DONE),
            final_state=self.drone_state,
        )

    def abort(self, reason: str = "user_abort"):
        """Emergency abort. Stops movement and hovers."""
        self._log(f"!!! ABORT: {reason}")
        self._stop_flag.set()
        self._change_state(DroneState.EMERGENCY)
        self.mission_state = MissionState.MISSION_ABORTED
        self.ctrl.emergency_stop()

    def pause(self):
        """Pause mission execution."""
        self._pause_flag.set()
        self.ctrl.hover(1.0)

    def resume(self):
        """Resume paused mission."""
        self._pause_flag.clear()

    # ── Command handlers (for external car control) ───

    def handle_car_command(self, cmd: str, params: Optional[dict] = None) -> dict:
        """Handle a command from the car. Returns response dict."""
        params = params or {}
        try:
            if cmd == "takeoff":
                return self._cmd_takeoff(params)
            elif cmd == "fly_to":
                return self._cmd_fly_to(params)
            elif cmd == "return":
                return self._cmd_return(params)
            elif cmd == "land":
                return self._cmd_land(params)
            elif cmd == "abort":
                self.abort(params.get("reason", "car_abort"))
                return {"status": "aborted"}
            elif cmd == "status":
                return self._cmd_status()
            else:
                return {"status": "error", "msg": f"Unknown command: {cmd}"}
        except Exception as e:
            return {"status": "error", "msg": str(e)}

    def _cmd_takeoff(self, params: dict) -> dict:
        altitude = params.get("altitude", self._target_altitude)
        self._target_altitude = altitude
        self._waypoints = []  # just takeoff + hover

        t = threading.Thread(target=self._run_takeoff_only, daemon=True)
        t.start()
        return {"status": "accepted", "action": "takeoff", "altitude": altitude}

    def _cmd_fly_to(self, params: dict) -> dict:
        x = params.get("x", 0.0)
        y = params.get("y", 0.0)
        z = params.get("z", self._target_altitude)
        hold = params.get("hold", 3.0)
        self._waypoints = [WaypointTask("car_cmd", x, y, z, hold)]

        t = threading.Thread(target=self._run_flyto_only, daemon=True)
        t.start()
        return {"status": "accepted", "target": (x, y, z)}

    def _cmd_return(self, params: dict) -> dict:
        self._change_state(DroneState.RETURNING)
        t = threading.Thread(target=self._run_return_only, daemon=True)
        t.start()
        return {"status": "accepted", "action": "return"}

    def _cmd_land(self, params: dict) -> dict:
        self._change_state(DroneState.LANDING)
        t = threading.Thread(target=self._run_land_only, daemon=True)
        t.start()
        return {"status": "accepted", "action": "land"}

    def _cmd_status(self) -> dict:
        x, y, z = self.vo.get_position()
        return {
            "status": "ok",
            "drone_state": self.drone_state.value,
            "mission_state": self.mission_state.value,
            "position": (round(x, 3), round(y, 3), round(z, 3)),
            "vo_healthy": self.vo.is_healthy(),
        }

    # ── Mission execution (internal) ───────────────────

    def _run_mission(self) -> MissionResult:
        """Full mission: takeoff -> waypoints -> return -> land."""
        self._mission_start_time = time.time()
        try:
            # Step 1: Precheck
            if not self._step_precheck():
                return self._fail("Precheck failed")

            # Step 2: Takeoff
            if not self._step_takeoff():
                return self._fail("Takeoff failed")

            # Step 3: Hover after takeoff
            if not self._step_hover_after_takeoff():
                return self._fail("Hover-stabilize failed")

            # Step 4: Execute waypoints
            for wp in self._waypoints:
                if self._stop_flag.is_set():
                    return self._fail("Aborted during waypoints")
                if not self._step_fly_to_waypoint(wp):
                    return self._fail(f"Waypoint {wp.waypoint_id} failed")

            # Step 5: Return (if waypoints were executed)
            if self._waypoints:
                if not self._step_return():
                    return self._fail("Return failed")

            # Step 6: Land
            if not self._step_land():
                return self._fail("Land failed")

            self._change_state(DroneState.DONE)
            self.mission_state = MissionState.MISSION_COMPLETE
            elapsed = time.time() - self._mission_start_time
            self._log(f"Mission complete in {elapsed:.1f}s")
            return MissionResult(
                success=True,
                final_state=DroneState.DONE,
                total_flight_time=elapsed,
            )

        except Exception as e:
            return self._fail(f"Exception: {e}")

    def _run_takeoff_only(self):
        """Standalone takeoff + hover."""
        try:
            if self._step_precheck() and self._step_takeoff():
                self._step_hover_after_takeoff()
        except Exception as e:
            self._log(f"Takeoff error: {e}")

    def _run_flyto_only(self):
        """Fly to single waypoint."""
        try:
            for wp in self._waypoints:
                self._step_fly_to_waypoint(wp)
        except Exception as e:
            self._log(f"FlyTo error: {e}")

    def _run_return_only(self):
        try:
            self._step_return()
            self._step_land()
        except Exception as e:
            self._log(f"Return error: {e}")

    def _run_land_only(self):
        try:
            self._step_land()
        except Exception as e:
            self._log(f"Land error: {e}")

    # ── Individual steps ───────────────────────────────

    def _step_precheck(self, timeout: float = 10.0) -> bool:
        """Pre-flight checks: telemetry alive, VO healthy."""
        self._change_state(DroneState.PRECHECK)
        self._log("PRECHECK: waiting for telemetry...")

        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._stop_flag.is_set():
                return False

            x, y, z = self.vo.get_position()
            # Check: any VO data received?
            if self.vo.state.total_frames > 5:
                healthy = self.vo.is_healthy()
                self._log(f"PRECHECK: alt={z:.2f}m vo_ok={healthy}")
                if healthy:
                    self._home_position = (x, y, z)
                    self._log(f"Home position recorded: ({x:.2f},{y:.2f},{z:.2f})")
                    return True
                else:
                    self._log("WARN: VO health degraded, continue anyway")
                    return True
            time.sleep(0.2)

        self._log("PRECHECK FAILED: no telemetry")
        return False

    def _step_takeoff(self, timeout: float = 30.0) -> bool:
        """Takeoff with PD altitude control. Currently uses srv(1) + MonitoredTakeoff."""
        self._change_state(DroneState.TAKING_OFF)
        self.mission_state = MissionState.DRONE_TAKING_OFF
        self._log("TAKEOFF: commanding motors...")

        # Use the takeoff service (triggers MonitoredTakeoff in PSDK)
        if not self.ctrl.takeoff():
            self._log("TAKEOFF REJECTED")
            return False

        # Wait for climb to stabilize
        t0 = time.time()
        min_alt = self._home_position[2] + 0.3 if self._home_position else 0.3
        while time.time() - t0 < timeout:
            if self._stop_flag.is_set():
                return False
            _, _, z = self.vo.get_position()
            if z > min_alt:
                self._log(f"TAKEOFF: airborne at {z:.2f}m")
                return True
            time.sleep(0.3)

        self._log("TAKEOFF FAILED: did not reach altitude")
        return False

    def _step_hover_after_takeoff(self, duration: float = 3.0) -> bool:
        """Hover-stabilize after takeoff before proceeding."""
        self._change_state(DroneState.HOVERING_AFTER_TAKEOFF)
        self._log(f"HOVER: stabilizing {duration}s...")

        t0 = time.time()
        while time.time() - t0 < duration:
            if self._stop_flag.is_set():
                return False
            self.ctrl.hover(0.2)
            time.sleep(0.2)

        self._change_state(DroneState.HOVERING)
        return True

    def _step_fly_to_waypoint(self, wp: WaypointTask, timeout: float = 30.0) -> bool:
        """Fly to a waypoint with PD position control."""
        self._change_state(DroneState.MOVING_TO_TARGET)
        self.mission_state = MissionState.DRONE_GOING_TO_TARGET
        self._log(f"FLY_TO: {wp.description or wp.waypoint_id} → ({wp.x:.2f},{wp.y:.2f},{wp.z:.2f})")

        success = self.pc.fly_to(wp.x, wp.y, wp.z, speed=wp.speed, timeout=timeout)

        if success:
            self._change_state(DroneState.HOLDING_POSITION)
            self.mission_state = MissionState.DRONE_OVER_TARGET
            self._log(f"ARRIVED at {wp.waypoint_id}")

            # Hold
            if wp.hold_time > 0:
                self._log(f"Holding {wp.hold_time}s at waypoint")
                self.pc.hover(wp.hold_time)
        else:
            self._log(f"WAYPOINT {wp.waypoint_id} TIMEOUT")

        return success

    def _step_return(self, timeout: float = 30.0) -> bool:
        """Return to home position."""
        self._change_state(DroneState.RETURNING)
        self.mission_state = MissionState.DRONE_RETURNING

        if not self._home_position:
            self._log("RETURN: no home position recorded, landing in place")
            return True

        hx, hy, hz = self._home_position
        target_z = hz + self._target_altitude  # maintain cruise altitude
        self._log(f"RETURN: flying to home ({hx:.2f},{hy:.2f},{target_z:.2f})")
        return self.pc.fly_to(hx, hy, target_z, speed=0.5, timeout=timeout)

    def _step_land(self, timeout: float = 20.0) -> bool:
        """Land."""
        self._change_state(DroneState.LANDING)
        self._log("LAND: commanding landing...")

        if not self.ctrl.land():
            self._log("LAND REJECTED")
            return False

        # Wait for touchdown
        t0 = time.time()
        ground_z = self._home_position[2] if self._home_position else 0.0
        while time.time() - t0 < timeout:
            if self._stop_flag.is_set():
                return False
            _, _, z = self.vo.get_position()
            if abs(z - ground_z) < 0.1:
                self._log(f"LAND: touched down at {z:.2f}m")
                self._change_state(DroneState.DONE)
                return True
            time.sleep(0.3)

        self._log("LAND: completed (timeout)")
        self._change_state(DroneState.DONE)
        return True

    # ── Internal helpers ───────────────────────────────

    def _change_state(self, new_state: DroneState):
        old = self.drone_state
        self.drone_state = new_state
        if self.on_state_changed:
            try:
                self.on_state_changed(new_state, self.mission_state)
            except Exception:
                pass

    def _log(self, msg: str):
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] [{self.drone_state.value}] {msg}"
        print(line)
        if self.on_log:
            try:
                self.on_log(line)
            except Exception:
                pass

    def _fail(self, reason: str) -> MissionResult:
        self._log(f"FAIL: {reason}")
        self._change_state(DroneState.ERROR)
        self.mission_state = MissionState.MISSION_ABORTED
        elapsed = time.time() - self._mission_start_time
        return MissionResult(
            success=False,
            final_state=DroneState.ERROR,
            total_flight_time=elapsed,
            error_message=reason,
        )

    def is_running(self) -> bool:
        return self._exec_thread is not None and self._exec_thread.is_alive()

    def get_status(self) -> dict:
        x, y, z = self.vo.get_position()
        return {
            "drone_state": self.drone_state.value,
            "mission_state": self.mission_state.value,
            "position": (round(x, 3), round(y, 3), round(z, 3)),
            "vo_healthy": self.vo.is_healthy(),
            "heading": round(self.vo.get_heading(), 3),
            "elapsed": round(time.time() - self._mission_start_time, 1),
        }
