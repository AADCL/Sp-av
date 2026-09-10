"""Deterministic 3D swept-hull local planning without ROS dependencies."""
from dataclasses import dataclass
from functools import cached_property
import math

try:
    import numpy as _np
except ImportError:
    _np = None

try:
    from scipy.spatial import cKDTree as _CKDTree
except ImportError:
    _CKDTree = None


TAU = 2.0 * math.pi


def angle_delta(target, current):
    return (target - current + math.pi) % TAU - math.pi


def wrap(angle):
    return (angle + math.pi) % TAU - math.pi


def finite(*values):
    return all(math.isfinite(float(value)) for value in values)


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class Terrain:
    ground_z: float
    agl: float
    variance: float
    valid: bool


@dataclass(frozen=True)
class Snapshot:
    stamp: float
    current: Pose
    velocity: Vec3
    goal: Pose
    obstacles: tuple
    terrain: Terrain


@dataclass(frozen=True)
class Hull:
    vertices: tuple
    reference: str
    declared_test_geometry: bool = False
    measurement_source: str = ""

    def __post_init__(self):
        if self.reference != "base_link":
            raise ValueError("planner and terrain hull reference must be base_link")
        if not self.declared_test_geometry and not self.measurement_source.strip():
            raise ValueError("hull requires a test declaration or physical measurement source")
        if len(self.vertices) < 4:
            raise ValueError("hull requires at least four vertices")
        for vertex in self.vertices:
            if len(vertex) != 3 or not finite(*vertex):
                raise ValueError("hull vertices must be finite XYZ triples")
        if self.bottom >= 0 or self.top <= 0 or self.radius <= 0:
            raise ValueError("hull must extend around the base_link origin")

    @cached_property
    def radius(self):
        # With no accepted live-attitude envelope, the circumsphere is the
        # conservative projection for every roll and pitch.
        return max(math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
                   for v in self.vertices)

    @property
    def bottom(self):
        return -self.radius

    @property
    def top(self):
        return self.radius


@dataclass(frozen=True)
class PlannerConfig:
    geometry_confirmed: bool = False
    sector_count: int = 72
    lookahead: float = 1.0
    waypoint_step: float = 0.5
    obstacle_clearance: float = 0.08
    min_passage_width: float = 0.62
    sensor_horizon: float = 2.0
    max_speed: float = 0.8
    max_acceleration: float = 1.0
    braking_acceleration: float = 1.0
    max_vertical_speed: float = 0.4
    min_agl: float = 0.35
    max_agl: float = 2.5
    floor_clearance: float = 0.08
    ceiling_clearance: float = 0.08
    max_terrain_variance: float = 0.02
    goal_tolerance: float = 0.12
    progress_epsilon: float = 0.05
    progress_timeout: float = 2.0
    blocked_hold: float = 0.3
    obstacle_persistence: float = 0.6
    prediction_horizon: float = 0.8
    max_track_displacement: float = 0.6
    min_prediction_speed: float = 0.5
    max_steering_angle: float = math.radians(60.0)
    heading_weight: float = 1.0
    hysteresis_weight: float = 0.35
    clearance_weight: float = 0.12
    voxel_size: float = 0.08
    snapshot_motion_margin: float = 0.0
    snapshot_speed_margin: float = 0.0

    def __post_init__(self):
        if type(self.geometry_confirmed) is not bool:
            raise ValueError("geometry_confirmed must be boolean")
        if type(self.sector_count) is not int or self.sector_count < 16 or self.sector_count % 2:
            raise ValueError("sector_count must be an even integer of at least 16")
        positive = (
            "lookahead", "waypoint_step", "obstacle_clearance", "min_passage_width",
            "sensor_horizon", "max_speed", "max_acceleration", "braking_acceleration",
            "max_vertical_speed", "max_agl", "floor_clearance", "ceiling_clearance",
            "max_terrain_variance", "goal_tolerance", "progress_epsilon",
            "progress_timeout", "blocked_hold", "obstacle_persistence",
            "prediction_horizon", "max_track_displacement", "min_prediction_speed",
            "max_steering_angle",
            "heading_weight", "hysteresis_weight", "clearance_weight", "voxel_size",
        )
        if any(not finite(getattr(self, name)) or getattr(self, name) <= 0 for name in positive):
            raise ValueError("planner numeric configuration must be finite and positive")
        if any(not finite(value) or value < 0 for value in
               (self.snapshot_motion_margin, self.snapshot_speed_margin)):
            raise ValueError("snapshot margins must be finite and nonnegative")
        if not finite(self.min_agl) or self.min_agl < 0 or self.min_agl >= self.max_agl:
            raise ValueError("invalid AGL envelope")
        if self.waypoint_step > self.lookahead or self.lookahead > self.sensor_horizon:
            raise ValueError("waypoint_step <= lookahead <= sensor_horizon is required")
        if self.max_steering_angle > math.pi / 2:
            raise ValueError("max_steering_angle cannot exceed pi/2")


@dataclass(frozen=True)
class PlanResult:
    state: str
    reason: str
    target: Pose
    heading: float
    clearance: float
    progress: float
    candidate_agl: float
    speed: float
    publish_target: bool
    hold_allowed: bool = False


class Planner:
    """Stateful VFH-style sector planner with geometric candidate validation."""

    def __init__(self, config, hull=None):
        if not isinstance(config, PlannerConfig):
            raise TypeError("config must be PlannerConfig")
        self.config = config
        self.hull = hull
        self._previous_heading = None
        self._previous_speed = 0.0
        self._previous_command_velocity = None
        self._previous_stamp = None
        self._previous_points = ()
        self._current_points = ()
        self._validated_cloud = None
        self._best_goal_distance = None
        self._last_progress_stamp = None
        self._blocked_since = None

    def reset_context(self):
        """Clear perception, steering, and progress history after a discontinuity."""
        self._previous_heading = None
        self._previous_speed = 0.0
        self._previous_command_velocity = None
        self._previous_stamp = None
        self._previous_points = ()
        self._current_points = ()
        self._validated_cloud = None
        self._best_goal_distance = None
        self._last_progress_stamp = None
        self._blocked_since = None

    @staticmethod
    def _pose_valid(pose):
        return isinstance(pose, Pose) and finite(pose.x, pose.y, pose.z, pose.yaw)

    def _invalid_reason(self, snapshot):
        self._validated_cloud = None
        if not self.config.geometry_confirmed or not isinstance(self.hull, Hull):
            return "measured hull geometry is not confirmed"
        if not isinstance(snapshot, Snapshot) or not finite(snapshot.stamp) or snapshot.stamp <= 0:
            return "invalid planning snapshot stamp"
        if not self._pose_valid(snapshot.current) or not self._pose_valid(snapshot.goal):
            return "non-finite current pose or goal"
        if not isinstance(snapshot.velocity, Vec3) or not finite(
                snapshot.velocity.x, snapshot.velocity.y, snapshot.velocity.z):
            return "non-finite velocity"
        terrain = snapshot.terrain
        if (not isinstance(terrain, Terrain) or terrain.valid is not True
                or not finite(terrain.ground_z, terrain.agl, terrain.variance)
                or terrain.variance < 0 or terrain.variance > self.config.max_terrain_variance):
            return "terrain is invalid or high variance"
        if _np is not None and len(snapshot.obstacles) >= 256:
            try:
                array = _np.asarray(snapshot.obstacles, dtype=_np.float64)
            except (TypeError, ValueError, OverflowError):
                return "obstacle cloud contains malformed points"
            if array.shape != (len(snapshot.obstacles), 3) or not _np.isfinite(array).all():
                return "obstacle cloud contains malformed points"
            with _np.errstate(over="ignore", invalid="ignore"):
                grid = array / self.config.voxel_size
            if not _np.isfinite(grid).all():
                return "obstacle coordinates exceed voxel numeric range"
            self._validated_cloud = (snapshot.obstacles, array, grid)
            return ""
        for point in snapshot.obstacles:
            if len(point) != 3 or not finite(*point):
                return "obstacle cloud contains malformed points"
            if not all(math.isfinite(float(axis) / self.config.voxel_size) for axis in point):
                return "obstacle coordinates exceed voxel numeric range"
        return ""

    def _expanded_obstacles(self, snapshot):
        current = self._voxelize(snapshot.obstacles)
        self._current_points = current
        expanded = list(current)
        if self._previous_stamp is None:
            return tuple(expanded)
        dt = snapshot.stamp - self._previous_stamp
        if dt <= 0:
            return tuple(expanded)
        if dt <= self.config.obstacle_persistence:
            expanded.extend(self._previous_points)
        if not current or not self._previous_points:
            return tuple(expanded)
        matches = self._nearest_previous_batch(current, self._previous_points)
        static_axis_limit = self.config.min_prediction_speed * dt / math.sqrt(3.0) * (1 - 1e-12)
        for point, previous in zip(current, matches):
            if previous is None:
                continue
            # This inscribed cube is strictly below the speed threshold. Keep
            # the original norm calculation for every boundary/fast point.
            if (abs(point[0] - previous[0]) < static_axis_limit
                    and abs(point[1] - previous[1]) < static_axis_limit
                    and abs(point[2] - previous[2]) < static_axis_limit):
                continue
            velocity = tuple((point[i] - previous[i]) / dt for i in range(3))
            if math.sqrt(sum(component * component for component in velocity)) < self.config.min_prediction_speed:
                continue
            for fraction in (0.25, 0.5, 0.75, 1.0):
                horizon = fraction * self.config.prediction_horizon
                expanded.append(tuple(point[i] + velocity[i] * horizon for i in range(3)))
        return tuple(expanded)

    def _nearest_previous_batch(self, current, previous):
        """Bounded exact nearest matches, with original ordinal tie breaking."""
        if not current or not previous:
            return (None,) * len(current)
        exact = {}
        for point in previous:
            exact.setdefault(point, point)
        matches = [exact.get(point) for point in current]
        pending = [index for index, match in enumerate(matches) if match is None]
        if not pending:
            return tuple(matches)
        if _CKDTree is None:
            cells = self._tracking_index(previous)
            for index in pending:
                matches[index] = self._nearest_previous(current[index], cells)
            return tuple(matches)

        tree = _CKDTree(previous)
        width = self.config.max_track_displacement
        padding = max(width * 1e-12, 1e-15)
        # k=2 detects possible ties without materializing dense radius
        # neighborhoods for every point. These options also work on SciPy 1.3.
        distances, ordinals = tree.query([current[index] for index in pending], k=2,
                                         eps=0, distance_upper_bound=width + padding)
        finite_rows = _np.isfinite(distances[:, 0])
        unique = (finite_rows & (distances[:, 1] > distances[:, 0] + padding)
                  & (distances[:, 0] < width - padding))
        for row, ordinal in zip(_np.flatnonzero(unique).tolist(), ordinals[unique, 0].tolist()):
            matches[pending[row]] = previous[ordinal]
        for row in _np.flatnonzero(finite_rows & ~unique).tolist():
            index, pair, neighbors = pending[row], distances[row], ordinals[row]
            point = current[index]
            candidates = [int(neighbors[0])]
            if pair[1] <= pair[0] + padding:
                candidates.extend(tree.query_ball_point(point, float(pair[0]) + padding))
            # Recompute using the same arithmetic as the fallback: a near tie
            # remains a true distance comparison, and the radius is inclusive.
            best = min((sum((point[axis] - previous[ordinal][axis]) ** 2
                            for axis in range(3)), ordinal) for ordinal in candidates)
            if best[0] <= width ** 2:
                matches[index] = previous[best[1]]
        return tuple(matches)

    def _candidate_obstacles(self, snapshot, obstacles):
        # Every candidate Z remains between current and absolute goal Z.
        # Filter only after persistence/prediction: a low observed point can
        # still have a predicted future position that intersects the hull.
        low = (min(snapshot.current.z, snapshot.goal.z) + self.hull.bottom
               - self.config.floor_clearance - self.config.snapshot_motion_margin)
        high = (max(snapshot.current.z, snapshot.goal.z) + self.hull.top
                + self.config.ceiling_clearance + self.config.snapshot_motion_margin)
        return tuple(point for point in obstacles if low <= point[2] <= high)

    def _tracking_index(self, points):
        width = self.config.max_track_displacement
        cells = {}
        for ordinal, point in enumerate(points):
            key = tuple(math.floor(axis / width) for axis in point)
            cells.setdefault(key, []).append((ordinal, point))
        return cells

    def _nearest_previous(self, point, cells):
        width = self.config.max_track_displacement
        key = tuple(math.floor(axis / width) for axis in point)
        best, result = (width ** 2, math.inf), None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for ordinal, candidate in cells.get((key[0]+dx, key[1]+dy, key[2]+dz), ()):
                        distance2 = sum((point[i]-candidate[i]) ** 2 for i in range(3))
                        score = (distance2, ordinal)
                        if score < best:
                            best, result = score, candidate
        return result

    def _voxelize(self, points):
        if _np is not None and len(points) >= 256:
            if self._validated_cloud is not None and points is self._validated_cloud[0]:
                _, array, grid = self._validated_cloud
            else:
                array = _np.asarray(points, dtype=_np.float64)
                with _np.errstate(over="ignore", invalid="ignore"):
                    grid = array / self.config.voxel_size
            # Keep Python's arbitrary-width integer keys for extreme finite
            # coordinates; never let float-to-int64 overflow wrap a cell.
            if _np.isfinite(grid).all() and (_np.abs(grid) < 2 ** 62).all():
                keys = _np.floor(grid).astype(_np.int64)
                cells, inverse, counts = _np.unique(keys, axis=0, return_inverse=True,
                                                    return_counts=True)
                totals = _np.zeros((len(cells), 3), dtype=_np.float64)
                # add.at retains input addition order for repeated cells;
                # reduceat/pairwise sums would change boundary centroids.
                _np.add.at(totals, inverse.reshape(-1), array)
                return tuple(map(tuple, (totals / counts[:, None]).tolist()))
        return self._voxelize_python(points)

    def _voxelize_python(self, points):
        cells = {}
        size = self.config.voxel_size
        for point in points:
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            key = (math.floor(x / size), math.floor(y / size), math.floor(z / size))
            total = cells.setdefault(key, [0.0, 0.0, 0.0, 0])
            total[0] += x
            total[1] += y
            total[2] += z
            total[3] += 1
        return tuple((total[0] / total[3], total[1] / total[3], total[2] / total[3])
                     for _key, total in sorted(cells.items()))

    def _vertical_intersects(self, point_z, start_z, end_z, along):
        base_z = start_z + (end_z - start_z) * along
        low = base_z + self.hull.bottom - self.config.floor_clearance - self.config.snapshot_motion_margin
        high = base_z + self.hull.top + self.config.ceiling_clearance + self.config.snapshot_motion_margin
        return low <= point_z <= high

    def _segment_clearance(self, start, end, obstacles, inflation):
        dx, dy = end.x - start.x, end.y - start.y
        length2 = dx * dx + dy * dy
        minimum = math.inf
        dz = end.z - start.z
        low_offset = self.hull.bottom - self.config.floor_clearance - self.config.snapshot_motion_margin
        high_offset = self.hull.top + self.config.ceiling_clearance + self.config.snapshot_motion_margin
        for ox, oy, oz in obstacles:
            base_low, base_high = oz - high_offset, oz - low_offset
            if abs(dz) <= 1e-15:
                z_interval = ((0.0, 1.0) if base_low <= start.z <= base_high else None)
            else:
                z0, z1 = ((base_low - start.z) / dz, (base_high - start.z) / dz)
                z_interval = (max(0.0, min(z0, z1)), min(1.0, max(z0, z1)))
                if z_interval[0] > z_interval[1]:
                    z_interval = None
            if z_interval is None:
                continue
            sx, sy = start.x - ox, start.y - oy
            if length2 <= 1e-15:
                xy_interval = (0.0, 1.0) if sx * sx + sy * sy <= inflation ** 2 else None
                along = 0.0
            else:
                along = max(0.0, min(1.0, -(sx * dx + sy * dy) / length2))
                b = 2.0 * (sx * dx + sy * dy)
                c = sx * sx + sy * sy - inflation ** 2
                discriminant = b * b - 4.0 * length2 * c
                if discriminant < 0:
                    xy_interval = None
                else:
                    root = math.sqrt(max(0.0, discriminant))
                    xy_interval = (max(0.0, (-b - root) / (2.0 * length2)),
                                   min(1.0, (-b + root) / (2.0 * length2)))
                    if xy_interval[0] > xy_interval[1]:
                        xy_interval = None
            if (xy_interval is not None and z_interval is not None
                    and max(xy_interval[0], z_interval[0]) <= min(xy_interval[1], z_interval[1])):
                return -1e-9
            px, py = start.x + along * dx, start.y + along * dy
            distance = math.hypot(ox - px, oy - py)
            minimum = min(minimum, distance - inflation)
        return minimum

    def _point_collision(self, pose, obstacles, inflation):
        for ox, oy, oz in obstacles:
            if (math.hypot(ox - pose.x, oy - pose.y) <= inflation
                    and self._vertical_intersects(oz, pose.z, pose.z, 0.0)):
                return True
        return False

    def _candidate(self, snapshot, heading, distance, vertical_dt):
        goal = snapshot.goal
        current = snapshot.current
        horizontal_goal = math.hypot(goal.x - current.x, goal.y - current.y)
        ratio = 1.0 if horizontal_goal < 1e-9 else min(1.0, distance / horizontal_goal)
        desired_z = current.z + (goal.z - current.z) * ratio
        z_limit = self.config.max_vertical_speed * vertical_dt
        desired_z = current.z + max(-z_limit, min(z_limit, desired_z - current.z))
        target = Pose(current.x + distance * math.cos(heading),
                      current.y + distance * math.sin(heading), desired_z, heading)
        candidate_agl = snapshot.terrain.agl + target.z - current.z
        bottom_agl = candidate_agl + self.hull.bottom - self.config.floor_clearance - self.config.snapshot_motion_margin
        top_agl = candidate_agl + self.hull.top + self.config.ceiling_clearance + self.config.snapshot_motion_margin
        if bottom_agl < self.config.min_agl or top_agl > self.config.max_agl:
            return None, candidate_agl
        return target, candidate_agl

    @staticmethod
    def _limit_distance(current, target, limit):
        delta = (target.x - current.x, target.y - current.y, target.z - current.z)
        length = math.sqrt(sum(value * value for value in delta))
        ratio = min(1.0, limit / length) if length > 0 else 1.0
        return Pose(current.x + delta[0] * ratio, current.y + delta[1] * ratio,
                    current.z + delta[2] * ratio, target.yaw)

    def _speed(self, snapshot):
        desired = min(self.config.max_speed,
                      math.sqrt(max(0.0, 2 * self.config.braking_acceleration
                                    * self.config.lookahead)))
        if self._previous_stamp is None:
            return min(desired, self.config.max_acceleration * 0.1)
        dt = max(0.0, snapshot.stamp - self._previous_stamp)
        return min(desired, self._previous_speed + self.config.max_acceleration * dt)

    def _limit_vector_acceleration(self, snapshot, target, dt):
        """Find a feasible speed along this already checked candidate direction."""
        if dt <= 0:
            return None
        delta = (target.x - snapshot.current.x, target.y - snapshot.current.y,
                 target.z - snapshot.current.z)
        distance = math.sqrt(sum(value * value for value in delta))
        previous = self._previous_command_velocity
        if previous is None:
            previous = (snapshot.velocity.x, snapshot.velocity.y, snapshot.velocity.z)
        change_limit = self.config.max_acceleration * dt
        previous2 = sum(value * value for value in previous)
        if distance <= 1e-15:
            return target if previous2 <= change_limit ** 2 + 1e-12 else None
        direction = tuple(value / distance for value in delta)
        along = sum(previous[i] * direction[i] for i in range(3))
        discriminant = along ** 2 - previous2 + change_limit ** 2
        if discriminant < -1e-12:
            return None
        root = math.sqrt(max(0, discriminant))
        low, high = max(0, along - root), min(distance / dt, along + root)
        if high < 0 or low > high + 1e-12:
            return None
        return self._limit_distance(snapshot.current, target, max(0, high) * dt)

    def _finish(self, result, snapshot):
        dt = (0.1 if self._previous_stamp is None else
              snapshot.stamp - self._previous_stamp)
        if result.publish_target and dt > 0:
            self._previous_command_velocity = (
                (result.target.x - snapshot.current.x) / dt,
                (result.target.y - snapshot.current.y) / dt,
                (result.target.z - snapshot.current.z) / dt)
        # BLOCKED hands braking to the separate controller. Retain the last
        # advancing velocity until fresh odometry confirms a stopped vehicle.
        elif result.state == "BLOCKED":
            self._previous_command_velocity = (snapshot.velocity.x, snapshot.velocity.y,
                                               snapshot.velocity.z)
        self._previous_stamp = snapshot.stamp
        self._previous_points = self._current_points
        self._previous_speed = (math.sqrt(sum(value * value for value in
                                             self._previous_command_velocity))
                                if self._previous_command_velocity is not None else 0.0)
        if result.state in ("CLEAR", "AVOIDING"):
            self._previous_heading = result.heading
        return result

    def _blocked_result(self, reason, snapshot, clearance=0.0, progress=0.0):
        if self._blocked_since is None:
            self._blocked_since = snapshot.stamp
        hold = snapshot.stamp - self._blocked_since <= self.config.blocked_hold
        return PlanResult("BLOCKED", reason, snapshot.current, snapshot.current.yaw,
                          clearance, progress, snapshot.terrain.agl, 0.0, False, hold)

    def plan(self, snapshot):
        reason = self._invalid_reason(snapshot)
        if reason:
            pose = snapshot.current if isinstance(snapshot, Snapshot) else Pose(0, 0, 0, 0)
            return PlanResult("STALE_INPUT", reason, pose, pose.yaw, -math.inf,
                              0.0, math.nan, 0.0, False)

        current, goal = snapshot.current, snapshot.goal
        goal_distance = math.sqrt((goal.x - current.x) ** 2 + (goal.y - current.y) ** 2
                                  + (goal.z - current.z) ** 2)
        obstacles = self._candidate_obstacles(snapshot, self._expanded_obstacles(snapshot))
        current_speed = math.sqrt(snapshot.velocity.x ** 2 + snapshot.velocity.y ** 2
                                  + snapshot.velocity.z ** 2)
        braking = (current_speed + self.config.snapshot_speed_margin) ** 2 / (2 * self.config.braking_acceleration)
        inflation = self.config.snapshot_motion_margin + max(
            self.hull.radius + self.config.obstacle_clearance + braking,
            self.config.min_passage_width / 2.0)
        observed_margin = max(inflation, self.hull.radius
                              + max(self.config.floor_clearance, self.config.ceiling_clearance)
                              + braking + self.config.snapshot_motion_margin)
        observed_range = self.config.sensor_horizon - observed_margin
        if observed_range <= 0:
            return self._finish(self._blocked_result(
                "stopping envelope exceeds observed sensor horizon", snapshot), snapshot)
        if (snapshot.terrain.agl + self.hull.bottom - self.config.floor_clearance - self.config.snapshot_motion_margin
                < self.config.min_agl
                or snapshot.terrain.agl + self.hull.top + self.config.ceiling_clearance + self.config.snapshot_motion_margin
                > self.config.max_agl):
            return self._finish(self._blocked_result(
                "current pose violates terrain or hull clearance", snapshot), snapshot)
        if self._point_collision(current, obstacles, inflation):
            result = self._blocked_result(
                "vehicle starts inside inflated obstacle", snapshot)
            return self._finish(result, snapshot)
        if self._point_collision(goal, obstacles, inflation):
            result = self._blocked_result("goal lies inside inflated obstacle", snapshot)
            return self._finish(result, snapshot)
        goal_agl = snapshot.terrain.agl + goal.z - current.z
        if (goal_agl + self.hull.bottom - self.config.floor_clearance - self.config.snapshot_motion_margin < self.config.min_agl
                or goal_agl + self.hull.top + self.config.ceiling_clearance + self.config.snapshot_motion_margin > self.config.max_agl):
            result = self._blocked_result(
                "goal violates terrain or hull clearance", snapshot)
            return self._finish(result, snapshot)
        speed = self._speed(snapshot)
        command_dt = (0.1 if self._previous_stamp is None else
                      max(0.0, snapshot.stamp - self._previous_stamp))
        if goal_distance <= self.config.goal_tolerance:
            if self._segment_clearance(current, goal, obstacles, inflation) < 0:
                result = self._blocked_result(
                    "final goal segment intersects inflated obstacle", snapshot)
                return self._finish(result, snapshot)
            limit = min(self.config.waypoint_step, speed * command_dt, observed_range)
            if abs(goal.z - current.z) > 0:
                limit = min(limit, goal_distance * self.config.max_vertical_speed
                            * command_dt / abs(goal.z - current.z))
            target = self._limit_distance(current, goal, limit)
            target = self._limit_vector_acceleration(snapshot, target, command_dt)
            if target is None:
                return self._finish(self._blocked_result(
                    "terminal command exceeds vector acceleration limit", snapshot), snapshot)
            return self._finish(PlanResult(
                "GOAL_REACHED", "goal tolerance reached", target, goal.yaw, math.inf,
                1.0, snapshot.terrain.agl + target.z - current.z, speed, True), snapshot)

        horizontal_goal = math.hypot(goal.x - current.x, goal.y - current.y)
        if horizontal_goal < 1e-9:
            goal_heading = current.yaw
        else:
            goal_heading = math.atan2(goal.y - current.y, goal.x - current.x)
        command_step = min(self.config.waypoint_step, horizontal_goal,
                           speed * command_dt)
        safety_step = min(self.config.lookahead, horizontal_goal, observed_range)
        safety_dt = max(command_dt, safety_step / max(speed, 1e-6))
        sector_width = TAU / self.config.sector_count
        sector_limit = int(self.config.max_steering_angle / sector_width + 1e-9)
        candidates = []
        for offset in range(-sector_limit, sector_limit + 1):
            heading = wrap(goal_heading + offset * sector_width)
            target, candidate_agl = self._candidate(snapshot, heading, command_step,
                                                    command_dt)
            if target is None:
                continue
            target = self._limit_distance(current, target, min(speed * command_dt,
                                                               observed_range))
            target = self._limit_vector_acceleration(snapshot, target, command_dt)
            if target is None:
                continue
            candidate_agl = snapshot.terrain.agl + target.z - current.z
            safety_target, _ = self._candidate(snapshot, heading, safety_step, safety_dt)
            if safety_target is None:
                continue
            safety_target = self._limit_distance(current, safety_target, observed_range)
            clearance = self._segment_clearance(current, safety_target, obstacles, inflation)
            if (clearance < 0 or
                    self._segment_clearance(current, target, obstacles, inflation) < 0):
                continue
            hysteresis = (0.0 if self._previous_heading is None else
                          abs(angle_delta(heading, self._previous_heading)))
            cost = (self.config.heading_weight * abs(offset) * sector_width
                    + self.config.hysteresis_weight * hysteresis
                    - self.config.clearance_weight * min(clearance, self.config.sensor_horizon))
            # Positive offset is the documented deterministic tie break.
            candidates.append((round(cost, 12), -offset, heading, target,
                               clearance, candidate_agl, offset))

        progress = 0.0
        if self._best_goal_distance is None:
            self._best_goal_distance = goal_distance
            self._last_progress_stamp = snapshot.stamp
        elif goal_distance < self._best_goal_distance - self.config.progress_epsilon:
            progress = self._best_goal_distance - goal_distance
            self._best_goal_distance = goal_distance
            self._last_progress_stamp = snapshot.stamp

        if not candidates:
            result = self._blocked_result(
                "no swept-hull safe sector", snapshot, progress=progress)
            return self._finish(result, snapshot)

        candidates.sort(key=lambda value: (value[0], value[1]))
        _, _, heading, target, clearance, candidate_agl, offset = candidates[0]
        avoiding = offset != 0
        if (self._last_progress_stamp is not None
                and snapshot.stamp - self._last_progress_stamp > self.config.progress_timeout):
            result = self._blocked_result(
                "no progress timeout", snapshot, clearance, progress)
            return self._finish(result, snapshot)

        self._blocked_since = None
        state = "AVOIDING" if avoiding else "CLEAR"
        reason = "selected safe alternative sector" if avoiding else "direct sector is safe"
        return self._finish(PlanResult(state, reason, target, heading, clearance, progress,
                                       candidate_agl, speed, True), snapshot)
