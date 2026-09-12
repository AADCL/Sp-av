"""ROS-independent waypoint mission policy."""

from dataclasses import dataclass
import math
import re
import uuid


def _number(value, name, positive=False, nonnegative=False):
    if isinstance(value, bool):
        raise ValueError("{} must be numeric".format(name))
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError("{} must be numeric".format(name))
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    if positive and result <= 0.0:
        raise ValueError("{} must be positive".format(name))
    if nonnegative and result < 0.0:
        raise ValueError("{} must be nonnegative".format(name))
    return result


def _integer(value, name):
    if isinstance(value, bool):
        raise ValueError("{} must be an integer".format(name))
    try:
        result = int(value)
    except (TypeError, ValueError):
        raise ValueError("{} must be an integer".format(name))
    if result != value:
        raise ValueError("{} must be an integer".format(name))
    return result


@dataclass(frozen=True)
class Waypoint:
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float
    xy_tolerance: float
    z_tolerance: float
    yaw_tolerance: float
    speed_tolerance: float
    dwell: float
    timeout: float


@dataclass(frozen=True)
class MissionConfig:
    mission_id: str
    allowed_frames: tuple
    workspace: dict
    waypoints: tuple
    mission_timeout: float
    blocked_timeout: float
    base_timeout: float = 1.5
    telemetry_timeout: float = 0.5

    @classmethod
    def from_dict(cls, raw):
        if not isinstance(raw, dict):
            raise ValueError("mission configuration must be a mapping")
        mission_id = raw.get("mission_id")
        if not isinstance(mission_id, str) or not mission_id.strip():
            raise ValueError("mission_id must be a non-empty string")
        frames = raw.get("allowed_frames", ["map", "odom"])
        if (not isinstance(frames, list) or not frames
                or any(not isinstance(frame, str) or not frame for frame in frames)):
            raise ValueError("allowed_frames must contain frame names")
        if len(set(frames)) != len(frames) or not set(frames).issubset({"map", "odom"}):
            raise ValueError("allowed_frames may contain map and odom only")
        workspace_raw = raw.get("workspace")
        if not isinstance(workspace_raw, dict):
            raise ValueError("workspace is required")
        workspace = {}
        for axis in ("x", "y", "z"):
            bounds = workspace_raw.get(axis)
            if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                raise ValueError("workspace.{} must have two bounds".format(axis))
            low = _number(bounds[0], "workspace.{}.min".format(axis))
            high = _number(bounds[1], "workspace.{}.max".format(axis))
            if low >= high:
                raise ValueError("workspace.{} bounds are reversed".format(axis))
            workspace[axis] = (low, high)

        defaults = raw.get("defaults")
        if not isinstance(defaults, dict):
            raise ValueError("defaults are required")
        required = ("xy_tolerance", "z_tolerance", "yaw_tolerance",
                    "speed_tolerance", "dwell", "timeout")
        for name in required:
            if name not in defaults:
                raise ValueError("defaults.{} is required".format(name))

        waypoint_data = raw.get("waypoints")
        if not isinstance(waypoint_data, list) or not waypoint_data:
            raise ValueError("at least one waypoint is required")
        waypoints = []
        for index, item in enumerate(waypoint_data):
            if not isinstance(item, dict):
                raise ValueError("waypoint {} must be a mapping".format(index))
            frame_id = item.get("frame_id")
            if frame_id not in frames:
                raise ValueError("waypoint {} frame is not allowed".format(index))
            position = item.get("position")
            if not isinstance(position, (list, tuple)) or len(position) != 3:
                raise ValueError("waypoint {} position must contain xyz".format(index))
            coordinates = tuple(
                _number(value, "waypoint {} position".format(index))
                for value in position
            )
            for axis, value in zip(("x", "y", "z"), coordinates):
                low, high = workspace[axis]
                if value < low or value > high:
                    raise ValueError("waypoint {} is outside {} bounds".format(index, axis))
            values = {}
            for name in required:
                value = item.get(name, defaults[name])
                values[name] = _number(
                    value, "waypoint {} {}".format(index, name),
                    positive=name != "dwell", nonnegative=name == "dwell")
            waypoints.append(Waypoint(
                frame_id, coordinates[0], coordinates[1], coordinates[2],
                _number(item.get("yaw"), "waypoint {} yaw".format(index)),
                values["xy_tolerance"], values["z_tolerance"],
                values["yaw_tolerance"], values["speed_tolerance"],
                values["dwell"], values["timeout"]))

        input_timeouts = raw.get("input_timeouts", {})
        if not isinstance(input_timeouts, dict):
            raise ValueError("input_timeouts must be a mapping")
        base_timeout = _number(input_timeouts.get("base", 1.5),
                               "input_timeouts.base", positive=True)
        telemetry_timeout = _number(input_timeouts.get("telemetry", 0.5),
                                    "input_timeouts.telemetry", positive=True)
        return cls(
            mission_id.strip(), tuple(frames), workspace, tuple(waypoints),
            _number(raw.get("mission_timeout"), "mission_timeout", positive=True),
            _number(raw.get("blocked_timeout"), "blocked_timeout", positive=True),
            base_timeout, telemetry_timeout)


@dataclass(frozen=True)
class GateSnapshot:
    now: float
    base_ready: bool
    base_age: float
    rc_valid: bool
    rc_command: bool
    kill: bool
    rc_age: float
    odom_age: float
    terrain_valid: bool
    terrain_ready: bool
    terrain_age: float
    planner_age: float
    controller_ready: bool
    controller_state: str
    controller_age: float
    height_mode: str = 'terrain'


@dataclass(frozen=True)
class OdomObservation:
    x: float
    y: float
    z: float
    yaw: float
    speed: float


@dataclass(frozen=True)
class PlannerObservation:
    request_id: str
    state: str


@dataclass(frozen=True)
class ControllerObservation:
    ready: bool
    state: str
    request_id: str


@dataclass(frozen=True)
class ExecutionTarget:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class MissionAction:
    kind: str
    generation: int
    request_id: str
    waypoint: object = None


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    message: str
    actions: tuple = ()


class StampedInput:
    """Track source and arrival freshness without re-stamping old payloads."""

    def __init__(self, name, source_timeout, arrival_timeout, future_tolerance=0.05):
        self.name = name
        self.source_timeout = _number(source_timeout, "source_timeout", positive=True)
        self.arrival_timeout = _number(arrival_timeout, "arrival_timeout", positive=True)
        self.future_tolerance = _number(future_tolerance, "future_tolerance", nonnegative=True)
        self.seen_stamp = None
        self.accepted_stamp = None
        self.arrival = None
        self.valid = False
        self.reason = "unavailable"

    def invalidate(self, reason):
        self.valid = False
        self.accepted_stamp = None
        self.arrival = None
        self.reason = reason

    def accept(self, source_stamp, arrival, payload_valid, now_source):
        try:
            source_stamp = _number(source_stamp, self.name + " stamp", positive=True)
            arrival = _number(arrival, self.name + " arrival", nonnegative=True)
            now_source = _number(now_source, "source clock", positive=True)
        except ValueError as error:
            self.invalidate(str(error))
            return False
        age = now_source - source_stamp
        if age > self.source_timeout:
            self.invalidate(self.name + " timestamp is stale")
            return False
        if age < -self.future_tolerance:
            self.invalidate(self.name + " timestamp is in the future")
            return False
        if self.seen_stamp is not None and source_stamp <= self.seen_stamp:
            self.invalidate(self.name + " timestamp is duplicate or backwards")
            return False
        self.seen_stamp = source_stamp
        if not payload_valid:
            self.invalidate(self.name + " payload is invalid")
            return False
        self.accepted_stamp = source_stamp
        self.arrival = arrival
        self.valid = True
        self.reason = ""
        return True

    def fresh(self, now_arrival, now_source):
        if not self.valid or self.arrival is None or self.accepted_stamp is None:
            return False
        try:
            arrival_age = _number(now_arrival, "arrival clock", nonnegative=True) - self.arrival
            source_age = _number(now_source, "source clock", positive=True) - self.accepted_stamp
        except ValueError:
            self.invalidate(self.name + " clock is invalid")
            return False
        if (arrival_age < 0.0 or arrival_age > self.arrival_timeout
                or source_age < -self.future_tolerance or source_age > self.source_timeout):
            self.invalidate(self.name + " sample is stale")
            return False
        return True


class MissionCore:
    ACTIVE_STATES = ("RUNNING", "DWELLING")

    def __init__(self, config, enable_commands=False, session_id=None):
        if not isinstance(config, MissionConfig):
            raise TypeError("config must be MissionConfig")
        self.config = config
        self.enable_commands = bool(enable_commands)
        if session_id is None:
            session_id = uuid.uuid4().hex[:12]
        if (not isinstance(session_id, str) or not session_id
                or re.match(r"^[A-Za-z0-9_-]+$", session_id) is None):
            raise ValueError("session_id must contain only letters, digits, _ or -")
        mission_token = re.sub(r"[^A-Za-z0-9_-]+", "-", config.mission_id).strip("-")
        self.session_id = session_id
        self.id_prefix = "{}-{}".format(mission_token or "mission", session_id)
        self.state = "IDLE"
        self.reason = "loaded; explicit start required"
        self.epoch = 0
        self.generation = 0
        self.waypoint_index = 0
        self.retry = 0
        self.request_id = ""
        self.mission_started = None
        self.waypoint_started = None
        self.dwell_started = None
        self.blocked_started = None
        self.stop_generation = None
        self.stop_complete = True
        self.execution_target = None

    def _gate_reason(self, gates):
        if gates.height_mode not in ('terrain','takeoff_relative'):
            return 'unknown mission height mode'
        timeout = self.config.telemetry_timeout
        checks = (
            (gates.base_ready and math.isfinite(gates.base_age)
             and 0.0 <= gates.base_age <= self.config.base_timeout,
             "base readiness unavailable or stale"),
            (gates.rc_valid and gates.rc_command and not gates.kill
             and math.isfinite(gates.rc_age) and 0.0 <= gates.rc_age <= timeout,
             "RC command authority unavailable"),
            (math.isfinite(gates.odom_age) and 0.0 <= gates.odom_age <= timeout,
             "localization unavailable or stale"),
            (gates.height_mode=='takeoff_relative' or (gates.terrain_valid and gates.terrain_ready
             and math.isfinite(gates.terrain_age) and 0.0 <= gates.terrain_age <= timeout),
             "terrain unavailable or stale"),
            (math.isfinite(gates.planner_age) and 0.0 <= gates.planner_age <= timeout,
             "planner unavailable or stale"),
            (gates.controller_ready and gates.controller_state in ("HOLD", "TRACK")
             and math.isfinite(gates.controller_age)
             and 0.0 <= gates.controller_age <= timeout,
             "controller ownership unavailable or stale"),
        )
        for valid, reason in checks:
            if not valid:
                return reason
        return ""

    def _new_request_id(self):
        self.request_id = "{}-e{:06d}-wp-{:03d}-try-{:03d}".format(
            self.id_prefix, self.epoch, self.waypoint_index, self.retry)

    def _goal_action(self):
        return MissionAction("goal", self.generation, self.request_id,
                             self.config.waypoints[self.waypoint_index])

    def _stop_action(self):
        self.generation += 1
        self.stop_generation = self.generation
        self.stop_complete = False
        return MissionAction("stop", self.generation, self.request_id)

    def start(self, now, gates):
        if not self.enable_commands:
            return CommandResult(False, "commands are disabled")
        if self.state in self.ACTIVE_STATES or self.state == "PAUSED":
            return CommandResult(False, "mission is already active")
        if not self.stop_complete:
            return CommandResult(False, "cancel and hold have not completed")
        reason = self._gate_reason(gates)
        if reason:
            return CommandResult(False, reason)
        self.epoch += 1
        self.generation += 1
        self.waypoint_index = 0
        self.retry = 0
        self._new_request_id()
        self.state = "RUNNING"
        self.reason = "executing waypoint"
        self.mission_started = now
        self.waypoint_started = now
        self.dwell_started = None
        self.blocked_started = None
        self.stop_complete = True
        self.execution_target = None
        return CommandResult(True, "mission started", (self._goal_action(),))

    def pause(self, now, reason="operator pause"):
        if self.state not in self.ACTIVE_STATES:
            return CommandResult(False, "mission is not running")
        self.state = "PAUSED"
        self.reason = reason
        self.dwell_started = None
        self.blocked_started = None
        return CommandResult(True, "mission paused", (self._stop_action(),))

    def resume(self, now, gates):
        if self.state != "PAUSED":
            return CommandResult(False, "mission is not paused")
        if not self.stop_complete:
            return CommandResult(False, "cancel and hold have not completed")
        reason = self._gate_reason(gates)
        if reason:
            return CommandResult(False, reason)
        self.retry += 1
        self.generation += 1
        self._new_request_id()
        self.state = "RUNNING"
        self.reason = "executing resumed waypoint"
        self.waypoint_started = now
        self.dwell_started = None
        self.blocked_started = None
        self.execution_target = None
        return CommandResult(True, "mission resumed", (self._goal_action(),))

    def set_execution_target(self, generation, target):
        if (generation != self.generation or self.state not in self.ACTIVE_STATES
                or not isinstance(target, ExecutionTarget)):
            return False
        if not all(math.isfinite(value) for value in
                   (target.x, target.y, target.z, target.yaw)):
            return False
        for axis, value in zip(("x", "y", "z"),
                               (target.x, target.y, target.z)):
            low, high = self.config.workspace[axis]
            if value < low or value > high:
                return False
        self.execution_target = target
        return True

    def cancel(self, now):
        if self.state not in self.ACTIVE_STATES and self.state != "PAUSED":
            return CommandResult(False, "mission is not active")
        self.state = "CANCELED"
        self.reason = "operator canceled"
        self.dwell_started = None
        return CommandResult(True, "mission canceled", (self._stop_action(),))

    def ack_stop(self, generation, accepted):
        if generation != self.stop_generation:
            return False
        self.stop_complete = bool(accepted)
        if accepted and self.state == "COMPLETING":
            self.state = "SUCCEEDED"
            self.reason = "all waypoints complete; holding"
        elif not accepted:
            self.reason = "navigation cancel or controller hold was rejected"
            if self.state == "COMPLETING":
                self.state = "FAILED"
        return self.stop_complete

    def ack_goal(self, generation, accepted):
        if generation != self.generation or self.state not in self.ACTIVE_STATES:
            return []
        if accepted:
            return []
        self.state = "FAILED"
        self.reason = "navigation goal was rejected"
        return [self._stop_action()]

    def _fail(self, reason):
        self.state = "FAILED"
        self.reason = reason
        self.dwell_started = None
        return [self._stop_action()]

    @staticmethod
    def _yaw_error(left, right):
        return abs(math.atan2(math.sin(left - right), math.cos(left - right)))

    def _at_target(self, odom):
        waypoint = self.config.waypoints[self.waypoint_index]
        target = self.execution_target
        if target is None:
            return False
        values = (odom.x, odom.y, odom.z, odom.yaw, odom.speed)
        if not all(math.isfinite(value) for value in values):
            return False
        return (
            math.hypot(odom.x - target.x, odom.y - target.y)
            <= waypoint.xy_tolerance
            and abs(odom.z - target.z) <= waypoint.z_tolerance
            and self._yaw_error(odom.yaw, target.yaw) <= waypoint.yaw_tolerance
            and odom.speed <= waypoint.speed_tolerance
        )

    def update(self, now, gates, odom, planner, controller):
        if self.state not in self.ACTIVE_STATES:
            return []
        gate_reason = self._gate_reason(gates)
        if gate_reason:
            return list(self.pause(now, gate_reason).actions)
        if now - self.mission_started >= self.config.mission_timeout:
            return self._fail("mission timeout")
        waypoint = self.config.waypoints[self.waypoint_index]
        if now - self.waypoint_started >= waypoint.timeout:
            return self._fail("waypoint timeout")
        if planner.request_id == self.request_id and planner.state == "STALE_INPUT":
            return list(self.pause(now, "planner input is stale").actions)
        if planner.request_id == self.request_id and planner.state == "BLOCKED":
            if self.blocked_started is None:
                self.blocked_started = now
            elif now - self.blocked_started >= self.config.blocked_timeout:
                return self._fail("planner blocked timeout")
        else:
            self.blocked_started = None

        complete_inputs = (
            planner.request_id == self.request_id
            and planner.state == "GOAL_REACHED"
            and controller.ready
            and controller.state == "TRACK"
            and controller.request_id == self.request_id
            and self._at_target(odom)
        )
        if not complete_inputs:
            self.state = "RUNNING"
            self.dwell_started = None
            return []
        if self.dwell_started is None:
            self.dwell_started = now
            self.state = "DWELLING"
            return []
        if now - self.dwell_started < waypoint.dwell:
            return []

        self.dwell_started = None
        self.blocked_started = None
        if self.waypoint_index + 1 == len(self.config.waypoints):
            self.state = "COMPLETING"
            self.reason = "waypoints complete; awaiting cancel and hold"
            return [self._stop_action()]
        self.waypoint_index += 1
        self.retry = 0
        self.generation += 1
        self._new_request_id()
        self.waypoint_started = now
        self.state = "RUNNING"
        self.reason = "executing waypoint"
        self.execution_target = None
        return [self._goal_action()]
