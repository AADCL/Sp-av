"""Thread-safe navigation input lifetime and exact-transform helpers."""
from dataclasses import dataclass
from collections import deque
import math
import threading

from .planner import Pose, Snapshot, Terrain, Vec3, finite, wrap


@dataclass(frozen=True)
class RuntimeConfig:
    source_timeout: float = 0.5
    arrival_timeout: float = 0.5
    future_tolerance: float = 0.05
    max_skew: float = 0.08
    tf_wait_timeout: float = 0.06
    readiness_timeout: float = 1.5
    history_limit: int = 128
    max_snapshot_translation: float = 0.05
    max_snapshot_speed_increase: float = 0.1
    max_terrain_variance: float = 0.02

    def __post_init__(self):
        values = (self.source_timeout, self.arrival_timeout, self.max_skew,
                  self.tf_wait_timeout, self.readiness_timeout)
        if any(not finite(value) or value <= 0 for value in values):
            raise ValueError("runtime timeouts must be finite and positive")
        if not finite(self.future_tolerance) or self.future_tolerance < 0:
            raise ValueError("future_tolerance must be finite and nonnegative")
        if type(self.history_limit) is not int or not 2 <= self.history_limit <= 4096:
            raise ValueError("history_limit must be an integer from 2 to 4096")
        for value in (self.max_snapshot_translation, self.max_snapshot_speed_increase,
                      self.max_terrain_variance):
            if not finite(value) or value < 0:
                raise ValueError("snapshot margins and terrain variance must be nonnegative")


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    message: str


@dataclass(frozen=True)
class InputSample:
    name: str
    stamp: float
    wall: float
    value: object


@dataclass(frozen=True)
class SnapshotToken:
    generation: int
    request_id: str
    stamps: tuple
    samples: tuple


@dataclass(frozen=True)
class GoalToken:
    generation: int
    request_id: str
    stamp: float


class NavigationRuntime:
    """Own input watermarks and make stale heavy-work results unpublishable."""

    SENSOR_NAMES = ("odom", "cloud", "terrain", "controller")
    SNAPSHOT_NAMES = ("odom", "cloud", "terrain")
    READINESS_NAMES = ("base_ready", "external_ready", "terrain_ready")
    SOURCE_NAMES = SENSOR_NAMES + READINESS_NAMES

    def __init__(self, config):
        self.config = config
        self._lock = threading.RLock()
        self._records = {name: self._record() for name in self.SOURCE_NAMES}
        self._history = {name: deque(maxlen=config.history_limit)
                         for name in ("odom", "terrain")}
        self._generation = 0
        self._goal_generation = 0
        self._request_id = ""
        self._goal = None
        self._goal_stamp = 0.0
        self._goal_orientation = None
        self._goal_seen = 0.0
        self._pending_goal = None
        self._last_publish_stamp = 0.0

    @staticmethod
    def _record():
        return {"seen": 0.0, "valid": False, "stamp": 0.0,
                "wall": None, "value": None}

    def _accept(self, name, stamp, wall, valid, value, ros_now=None):
        with self._lock:
            record = self._records[name]
            time_valid = finite(stamp, wall) and stamp > 0 and wall >= 0
            plausible = (ros_now is None or
                         (finite(ros_now) and ros_now - stamp >= -self.config.future_tolerance))
            if not time_valid or not plausible or stamp <= record["seen"]:
                record.update(valid=False, stamp=0.0, wall=None, value=None)
                if name in self._history:
                    self._history[name].clear()
                self._generation += 1
                return False
            record["seen"] = float(stamp)
            if not valid:
                record.update(valid=False, stamp=float(stamp), wall=float(wall), value=None)
                if name in self._history:
                    self._history[name].clear()
                self._generation += 1
                return False
            record.update(valid=True, stamp=float(stamp), wall=float(wall), value=value)
            if name in self._history:
                history = self._history[name]
                history.append(InputSample(name, float(stamp), float(wall), value))
                while history and (stamp - history[0].stamp > self.config.source_timeout
                                   or wall - history[0].wall > max(self.config.arrival_timeout,
                                                                   self.config.source_timeout)):
                    history.popleft()
            if name == "cloud":
                self._generation += 1
            return True

    def accept_odom(self, stamp, wall, frame, child_frame, pose, velocity, ros_now=None):
        valid = (frame == "odom" and child_frame == "base_link"
                 and isinstance(pose, Pose) and isinstance(velocity, Vec3)
                 and finite(pose.x, pose.y, pose.z, pose.yaw,
                            velocity.x, velocity.y, velocity.z))
        return self._accept("odom", stamp, wall, valid, (pose, velocity), ros_now)

    def accept_cloud(self, stamp, wall, frame, points, ros_now=None):
        try:
            points = tuple(tuple(float(axis) for axis in point) for point in points)
            points_valid = all(len(point) == 3 and finite(*point) for point in points)
        except (TypeError, ValueError):
            points, points_valid = (), False
        return self._accept("cloud", stamp, wall, frame == "odom" and points_valid,
                            points, ros_now)

    def accept_terrain(self, stamp, wall, frame, terrain, ros_now=None):
        valid = (frame == "odom" and isinstance(terrain, Terrain)
                 and terrain.valid is True
                  and finite(terrain.ground_z, terrain.agl, terrain.variance)
                  and 0 <= terrain.variance <= self.config.max_terrain_variance)
        return self._accept("terrain", stamp, wall, valid, terrain, ros_now)

    def accept_controller(self, stamp, wall, ready, ros_now=None):
        return self._accept("controller", stamp, wall, ready is True,
                            bool(ready), ros_now)

    def accept_readiness(self, name, ready, stamp, wall, ros_now=None):
        if name not in self.READINESS_NAMES:
            raise ValueError("unknown readiness source")
        return self._accept(name, stamp, wall, type(ready) is bool and ready,
                            bool(ready), ros_now)

    def begin_goal(self, request_id, stamp, ros_now):
        valid = (isinstance(request_id, str) and request_id.strip()
                 and finite(stamp, ros_now) and stamp > 0
                 and -self.config.future_tolerance <= ros_now - stamp <= self.config.source_timeout)
        with self._lock:
            if not valid or stamp <= self._goal_seen:
                return None, CommandResult(False, "request ID or goal source stamp is invalid")
            self._goal_seen = float(stamp)
            self._request_id = ""
            self._goal = None
            self._goal_orientation = None
            self._goal_generation += 1
            self._generation += 1
            token = GoalToken(self._goal_generation, request_id.strip(), float(stamp))
            self._pending_goal = token
        return token, CommandResult(True, "navigation goal reserved")

    def complete_goal(self, token, goal, orientation, ros_now):
        if (not isinstance(goal, Pose) or not finite(goal.x, goal.y, goal.z, goal.yaw)
                or len(orientation) != 4 or not finite(*orientation, ros_now)):
            return CommandResult(False, "transformed navigation goal is invalid")
        with self._lock:
            if (token != self._pending_goal or token.generation != self._goal_generation
                    or not (-self.config.future_tolerance <= ros_now - token.stamp
                            <= self.config.source_timeout)):
                return CommandResult(False, "navigation goal was superseded or became stale")
            self._request_id = token.request_id
            self._goal = goal
            self._goal_stamp = token.stamp
            self._goal_orientation = tuple(orientation)
            self._pending_goal = None
            self._goal_generation += 1
            self._generation += 1
        return CommandResult(True, "navigation goal accepted")

    def abort_goal(self, token):
        with self._lock:
            if token == self._pending_goal:
                self._pending_goal = None
                self._goal_generation += 1
                self._generation += 1

    def set_goal(self, request_id, goal, stamp, ros_now=None, orientation=(0, 0, 0, 1)):
        ros_now = stamp if ros_now is None else ros_now
        token, result = self.begin_goal(request_id, stamp, ros_now)
        if not result.accepted:
            return result
        return self.complete_goal(token, goal, orientation, ros_now)

    def cancel(self, request_id):
        with self._lock:
            if not isinstance(request_id, str) or not request_id.strip():
                return CommandResult(False, "cancel request ID is invalid")
            active = self._request_id == request_id
            pending = self._pending_goal is not None and self._pending_goal.request_id == request_id
            if not active and not pending:
                if not self._request_id and self._pending_goal is None:
                    return CommandResult(True, "navigation request is already inactive")
                return CommandResult(False, "request ID is not active")
            self._request_id = ""
            self._goal = None
            self._goal_stamp = 0.0
            self._goal_orientation = None
            self._pending_goal = None
            self._goal_generation += 1
            self._generation += 1
        return CommandResult(True, "navigation goal cancelled")

    def _fresh(self, name, ros_now, wall_now):
        record = self._records[name]
        timeout = (self.config.readiness_timeout if name in self.READINESS_NAMES
                   else self.config.source_timeout)
        return (record["valid"] and record["wall"] is not None
                and -self.config.future_tolerance <= ros_now - record["stamp"] <= timeout
                and 0 <= wall_now - record["wall"] <= max(self.config.arrival_timeout, timeout))

    def _sample_fresh(self, sample, ros_now, wall_now):
        return (-self.config.future_tolerance <= ros_now - sample.stamp
                <= self.config.source_timeout
                and 0 <= wall_now - sample.wall <= max(self.config.arrival_timeout,
                                                       self.config.source_timeout))

    def _matched_samples(self, ros_now, wall_now):
        cloud = self._records["cloud"]
        stamp = cloud["stamp"]
        candidates = {name: [sample for sample in self._history[name]
                             if abs(sample.stamp - stamp) <= self.config.max_skew + 1e-12
                             and self._sample_fresh(sample, ros_now, wall_now)]
                      for name in ("odom", "terrain")}
        best, best_cost = None, None
        for odom in candidates["odom"]:
            for terrain in candidates["terrain"]:
                span = max(stamp, odom.stamp, terrain.stamp) - min(stamp, odom.stamp, terrain.stamp)
                if span > self.config.max_skew + 1e-12:
                    continue
                cost = (abs(odom.stamp - stamp) + abs(terrain.stamp - stamp),
                        span, -odom.stamp, -terrain.stamp)
                if best_cost is None or cost < best_cost:
                    best_cost = cost
                    best = (odom, InputSample("cloud", stamp, cloud["wall"], cloud["value"]),
                            terrain)
        return best

    def _motion_ok(self, samples):
        selected_pose, selected_velocity = samples[0].value
        latest_pose, latest_velocity = self._records["odom"]["value"]
        distance = math.sqrt(sum((getattr(latest_pose, axis) - getattr(selected_pose, axis)) ** 2
                                 for axis in ("x", "y", "z")))
        selected_terrain, latest_terrain = samples[2].value, self._records["terrain"]["value"]
        # A newly observed ground change consumes the same reserved spatial
        # margin as vehicle movement; do not silently keep an old ground model.
        ground_change = max(abs(latest_terrain.ground_z - selected_terrain.ground_z),
                            abs(latest_terrain.agl - selected_terrain.agl
                                - (latest_pose.z - selected_pose.z)))
        selected_speed = math.sqrt(sum(getattr(selected_velocity, axis) ** 2
                                       for axis in ("x", "y", "z")))
        latest_speed = math.sqrt(sum(getattr(latest_velocity, axis) ** 2
                                     for axis in ("x", "y", "z")))
        return (distance + ground_change <= self.config.max_snapshot_translation + 1e-12
                and latest_speed <= selected_speed + self.config.max_snapshot_speed_increase + 1e-12)

    def snapshot(self, ros_now, wall_now, clock=None):
        with self._lock:
            # A callback can accept a newer heartbeat while the caller waits
            # for this lock. Sample here to avoid a negative receipt age.
            if clock is not None:
                ros_now, wall_now = clock()
            if not finite(ros_now, wall_now):
                return None, None, "invalid current time"
            if not self._request_id or self._goal is None:
                return None, None, "no active navigation request"
            for name in self.SOURCE_NAMES:
                if not self._fresh(name, ros_now, wall_now):
                    return None, None, "%s missing or stale" % name
            samples = self._matched_samples(ros_now, wall_now)
            if samples is None:
                return None, None, "input source skew exceeds limit"
            if not self._motion_ok(samples):
                return None, None, "snapshot motion exceeds reserved margin"
            pose, velocity = samples[0].value
            snapshot = Snapshot(samples[1].stamp, pose, velocity, self._goal,
                                samples[1].value, samples[2].value)
            stamps = tuple(sample.stamp for sample in samples)
            token = SnapshotToken(self._generation, self._request_id, stamps, samples)
            return token, snapshot, ""

    def revalidate(self, token, ros_now, wall_now):
        with self._lock:
            return (isinstance(token, SnapshotToken)
                    and token.generation == self._generation
                    and token.request_id == self._request_id
                    and all(self._fresh(name, ros_now, wall_now)
                            for name in self.SOURCE_NAMES)
                    and token.samples[1].stamp == self._records["cloud"]["stamp"]
                    and all(self._sample_fresh(sample, ros_now, wall_now) for sample in token.samples)
                    and max(token.stamps) - min(token.stamps) <= self.config.max_skew + 1e-12
                    and self._motion_ok(token.samples))

    def claim_publish(self, token, stamp, ros_now, wall_now):
        with self._lock:
            if (not self.revalidate(token, ros_now, wall_now) or not finite(stamp)
                    or stamp <= self._last_publish_stamp):
                return False
            self._last_publish_stamp = float(stamp)
            return True

    def commit_observation(self, token, clock, publish):
        """Enqueue snapshot status/preview only while its request is current."""
        with self._lock:
            ros_now, wall_now = clock()
            if not self.revalidate(token, ros_now, wall_now):
                return False
            publish()
            return True

    def commit_publish(self, token, stamp, clock, publish):
        """Validate and enqueue one output without a cancellation race."""
        with self._lock:
            # Sample after acquiring the lock: serialization or another callback
            # may have consumed the remaining source/arrival lifetime.
            ros_now, wall_now = clock()
            if (not self.revalidate(token, ros_now, wall_now) or not finite(stamp)
                    or stamp <= self._last_publish_stamp):
                return False
            publish()
            self._last_publish_stamp = float(stamp)
            return True

    def ages(self, ros_now):
        with self._lock:
            return {name: (math.inf if not record["valid"] else ros_now - record["stamp"])
                    for name, record in self._records.items()}

    @property
    def request_id(self):
        with self._lock:
            return self._request_id

    @property
    def goal_orientation(self):
        with self._lock:
            return self._request_id, self._goal_orientation


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(point, quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not finite(norm) or norm <= 0:
        raise ValueError("transform quaternion is invalid")
    q = (x / norm, y / norm, z / norm, w / norm)
    pure = (point[0], point[1], point[2], 0.0)
    conjugate = (-q[0], -q[1], -q[2], q[3])
    return quaternion_multiply(quaternion_multiply(q, pure), conjugate)[:3]


def transform_points(points, translation, rotation_quaternion):
    if len(translation) != 3 or not finite(*translation, *rotation_quaternion):
        raise ValueError("transform contains non-finite values")
    transformed = []
    for point in points:
        rotated = rotate(point, rotation_quaternion)
        transformed.append(tuple(rotated[i] + translation[i] for i in range(3)))
    return tuple(transformed)


def transform_pose(pose, translation, rotation_quaternion):
    position = transform_points(((pose.x, pose.y, pose.z),), translation,
                                rotation_quaternion)[0]
    yaw_q = (0.0, 0.0, math.sin(pose.yaw / 2.0), math.cos(pose.yaw / 2.0))
    orientation = quaternion_multiply(rotation_quaternion, yaw_q)
    # The planning core steers in gravity-aligned odom, so it consumes odom yaw.
    yaw = wrap(math.atan2(2 * (orientation[3] * orientation[2]
                              + orientation[0] * orientation[1]),
                          1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2)))
    return Pose(position[0], position[1], position[2], yaw)


def transform_pose_full(position, orientation, translation, rotation_quaternion):
    transformed = transform_points((position,), translation, rotation_quaternion)[0]
    transformed_orientation = quaternion_multiply(rotation_quaternion, orientation)
    norm = math.sqrt(sum(value * value for value in transformed_orientation))
    if not finite(norm) or norm <= 0:
        raise ValueError("pose orientation is invalid")
    return transformed, tuple(value / norm for value in transformed_orientation)


def lookup_exact(lookup, target, source, source_stamp, wait_timeout, clock,
                 pause, exceptions):
    """Retry zero-duration exact TF lookups until a monotonic deadline."""
    if not finite(source_stamp) or source_stamp <= 0:
        raise ValueError("source stamp must be finite and positive")
    deadline = clock() + wait_timeout
    last_error = None
    while True:
        try:
            return lookup(target, source, source_stamp, 0.0)
        except exceptions as error:
            last_error = error
            if clock() >= deadline:
                raise last_error
            pause()
