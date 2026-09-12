#!/usr/bin/env python3
"""ROS adapter for the deterministic ducted local planner."""
import math
import threading
from dataclasses import replace
from time import monotonic

import rospy
import tf2_ros
from ducted_msgs.msg import FlightControlStatus, FlightSetpoint, TerrainHeight
from ducted_navigation.msg import NavigationStatus
from ducted_navigation.planner import Hull, Planner, PlannerConfig, Pose, Terrain, Vec3
from ducted_navigation.runtime import (
    NavigationRuntime, RuntimeConfig, lookup_exact, transform_points,
    transform_pose_full, rotate,
)
from ducted_navigation.srv import Navigate, NavigateResponse
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


def _stamp(message):
    try:
        return float(message.header.stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return math.nan


def _xyz(value):
    return (float(value.x), float(value.y), float(value.z))


def _quaternion(value):
    return (float(value.x), float(value.y), float(value.z), float(value.w))


def _yaw(value):
    x, y, z, w = _quaternion(value)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("orientation quaternion is invalid")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class NavigationNode:
    def __init__(self):
        self.odom_frame = rospy.get_param("~frames/odom", "odom")
        self.map_frame = rospy.get_param("~frames/map", "map")
        self.base_frame = rospy.get_param("~frames/base", "base_link")
        self.enable_output = rospy.get_param("~enable_output", False) is True
        self.geometry_confirmed = rospy.get_param("~geometry_confirmed", False) is True
        if rospy.get_param("~agl_reference", "base_link") != "base_link":
            raise rospy.ROSInitException("navigation and terrain AGL reference must be base_link")

        runtime_config = RuntimeConfig(
            source_timeout=float(rospy.get_param("~timeouts/source", 0.5)),
            arrival_timeout=float(rospy.get_param("~timeouts/arrival", 0.5)),
            readiness_timeout=float(rospy.get_param("~timeouts/readiness", 1.5)),
            future_tolerance=float(rospy.get_param("~timeouts/future_tolerance", 0.05)),
            max_skew=float(rospy.get_param("~timeouts/max_skew", 0.08)),
            tf_wait_timeout=float(rospy.get_param("~timeouts/tf_wait", 0.06)),
        )
        planner_values = {}
        for field in PlannerConfig.__dataclass_fields__:
            if field == "geometry_confirmed":
                planner_values[field] = self.geometry_confirmed
            else:
                default = PlannerConfig.__dataclass_fields__[field].default
                planner_values[field] = rospy.get_param("~planner/" + field, default)
        planner_config = PlannerConfig(**planner_values)
        runtime_config = replace(
            runtime_config,
            max_snapshot_translation=planner_config.snapshot_motion_margin,
            max_snapshot_speed_increase=planner_config.snapshot_speed_margin,
            max_terrain_variance=planner_config.max_terrain_variance)
        hull = self._load_hull() if self.geometry_confirmed else None
        self.runtime = NavigationRuntime(runtime_config)
        backend=rospy.get_param("~planner_backend", "ego")
        if backend != "ego":
            raise rospy.ROSInitException("local planner backend is ego; obsolete backends are disabled")
        from ducted_navigation.trajectory_guard import TrajectoryGuard
        from ducted_navigation.ego_planner_client import EgoPlannerClient
        self.planner=TrajectoryGuard(planner_config,hull)
        self._ego_planner=EgoPlannerClient(self)
        self._planner_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._watchdog_stop = threading.Event()
        self._plan_event = threading.Event()
        self._last_planned_stamp = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_pub = rospy.Publisher("target", FlightSetpoint, queue_size=1)
        self.status_pub = rospy.Publisher("status", NavigationStatus, queue_size=1, latch=True)
        self.preview_pub = rospy.Publisher("preview", Path, queue_size=1)
        self.command_service = rospy.Service("command", Navigate, self._command)
        self.subscribers = (
            rospy.Subscriber("odom", Odometry, self._odom_callback, queue_size=1),
            rospy.Subscriber("cloud", PointCloud2, self._cloud_callback, queue_size=1),
            rospy.Subscriber("terrain", TerrainHeight, self._terrain_callback, queue_size=1),
            rospy.Subscriber("controller_status", FlightControlStatus,
                             self._controller_callback, queue_size=1),
            rospy.Subscriber("base_ready", Bool, self._base_ready_callback, queue_size=1),
            rospy.Subscriber("external_ready", Bool, self._external_ready_callback, queue_size=1),
            rospy.Subscriber("terrain_ready", Bool, self._terrain_ready_callback, queue_size=1),
        )
        period = float(rospy.get_param("~timeouts/watchdog_period", 0.1))
        if not math.isfinite(period) or period <= 0:
            raise rospy.ROSInitException("watchdog period must be finite and positive")
        self.watchdog = threading.Thread(
            target=self._watchdog_loop, args=(period,), name="navigation-watchdog",
            daemon=True)
        self.watchdog.start()
        self.planning_worker = threading.Thread(
            target=self._planning_loop, name="navigation-planner", daemon=True)
        self.planning_worker.start()
        rospy.on_shutdown(self._shutdown)
        self._publish_status("IDLE", "no active navigation request", math.inf, 0.0, 0.0)

    def _load_hull(self):
        vertices = rospy.get_param("~body_vertices", [])
        source = rospy.get_param("~geometry_measurement_source", "")
        if not vertices or not source.strip():
            raise rospy.ROSInitException(
                "confirmed geometry requires measured body_vertices and measurement source")
        try:
            return Hull(tuple(tuple(float(axis) for axis in vertex) for vertex in vertices),
                        "base_link", measurement_source=source)
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException(str(error))

    def _now(self):
        now = rospy.Time.now()
        return now, float(now.to_sec()), monotonic()

    def _lookup(self, target, source, ros_stamp, numeric_stamp):
        exceptions = (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                      tf2_ros.ExtrapolationException)

        def attempt(target_frame, source_frame, _stamp, _timeout):
            return self.tf_buffer.lookup_transform(target_frame, source_frame, ros_stamp,
                                                   rospy.Duration(0.0))

        return lookup_exact(attempt, target, source, numeric_stamp,
                            self.runtime.config.tf_wait_timeout, monotonic,
                            lambda: threading.Event().wait(0.002), exceptions)

    @staticmethod
    def _transform_parts(transform):
        item = transform.transform
        return _xyz(item.translation), _quaternion(item.rotation)

    def _odom_callback(self, message):
        _, ros_now, wall = self._now()
        stamp = _stamp(message)
        try:
            p, q = message.pose.pose.position, message.pose.pose.orientation
            pose = Pose(float(p.x), float(p.y), float(p.z), _yaw(q))
            v = message.twist.twist.linear
            # nav_msgs/Odometry twist belongs to child_frame_id (body FLU).
            # Candidate dynamics use the gravity-aligned odom axes.
            velocity = Vec3(*rotate(_xyz(v), _quaternion(q)))
            angular = _xyz(message.twist.twist.angular)
            pose_covariance = tuple(message.pose.covariance)
            twist_covariance = tuple(message.twist.covariance)
            if (len(pose_covariance) != 36 or len(twist_covariance) != 36
                    or not all(math.isfinite(float(value)) for value in
                               angular + pose_covariance + twist_covariance)):
                raise ValueError("odometry twist or covariance is invalid")
            frame, child = message.header.frame_id, message.child_frame_id
        except (AttributeError, TypeError, ValueError):
            pose, velocity, frame, child = Pose(math.nan, 0, 0, 0), Vec3(0, 0, 0), "", ""
        self.runtime.accept_odom(stamp, wall, frame, child, pose, velocity, ros_now)
        self._plan_event.set()

    def _terrain_callback(self, message):
        _, ros_now, wall = self._now()
        try:
            terrain = Terrain(float(message.ground_z), float(message.agl),
                              float(message.variance), message.valid)
            frame = message.header.frame_id
        except (AttributeError, TypeError, ValueError):
            terrain, frame = Terrain(math.nan, 0, 0, False), ""
        self.runtime.accept_terrain(_stamp(message), wall, frame, terrain, ros_now)
        self._plan_event.set()

    def _controller_callback(self, message):
        _, ros_now, wall = self._now()
        self.runtime.accept_controller(_stamp(message), wall, message.ready, ros_now)
        self._plan_event.set()

    def _readiness(self, name, message):
        _, ros_now, wall = self._now()
        self.runtime.accept_readiness(name, message.data, ros_now, wall, ros_now)
        self._plan_event.set()

    def _base_ready_callback(self, message):
        self._readiness("base_ready", message)

    def _external_ready_callback(self, message):
        self._readiness("external_ready", message)

    def _terrain_ready_callback(self, message):
        self._readiness("terrain_ready", message)

    def _cloud_callback(self, message):
        _, _ros_at_arrival, wall_at_arrival = self._now()
        stamp = _stamp(message)
        try:
            points = tuple(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=False))
            if message.header.frame_id != self.odom_frame:
                transform = self._lookup(self.odom_frame, message.header.frame_id,
                                         message.header.stamp, stamp)
                points = transform_points(points, *self._transform_parts(transform))
            _, ros_now, _ = self._now()
            accepted = self.runtime.accept_cloud(stamp, wall_at_arrival, self.odom_frame,
                                                 points, ros_now)
        except (ValueError, TypeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            _, ros_now, _ = self._now()
            accepted = self.runtime.accept_cloud(stamp, wall_at_arrival, "", (), ros_now)
        if accepted:
            self._plan_event.set()
        else:
            self._reset_planner()
            self._publish_status("STALE_INPUT", "cloud rejected", math.inf, 0.0, 0.0)

    def _command(self, request):
        command = request.command.strip().lower() if isinstance(request.command, str) else ""
        if command == "cancel":
            result = self.runtime.cancel(request.request_id)
            if result.accepted:
                self._reset_planner()
                self._plan_event.set()
            return NavigateResponse(result.accepted, result.message)
        if command != "goal":
            return NavigateResponse(False, "command must be goal or cancel")
        stamp = _stamp(request.target)
        frame = request.target.header.frame_id
        _, ros_now, _ = self._now()
        goal_token, reservation = self.runtime.begin_goal(
            request.request_id, stamp, ros_now)
        if not reservation.accepted:
            return NavigateResponse(False, reservation.message)
        try:
            position = _xyz(request.target.pose.position)
            orientation = _quaternion(request.target.pose.orientation)
            if frame == self.map_frame:
                transform = self._lookup(self.odom_frame, frame,
                                         request.target.header.stamp, stamp)
                position, orientation = transform_pose_full(
                    position, orientation, *self._transform_parts(transform))
            elif frame != self.odom_frame:
                raise ValueError("goal frame must be map or odom")
            else:
                position, orientation = transform_pose_full(
                    position, orientation, (0, 0, 0), (0, 0, 0, 1))
            yaw = _yaw(type("Q", (), dict(zip(("x", "y", "z", "w"), orientation)))())
            goal = Pose(position[0], position[1], position[2], yaw)
        except (ValueError, TypeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.runtime.abort_goal(goal_token)
            return NavigateResponse(False, "goal or exact source-time transform is invalid")
        _, ros_now, _ = self._now()
        result = self.runtime.complete_goal(goal_token, goal, orientation, ros_now)
        if result.accepted:
            self._reset_planner()
            self._plan_event.set()
        return NavigateResponse(result.accepted, result.message)

    def _plan_once(self, ros_now, wall):
        token, snapshot, reason = self.runtime.snapshot(
            ros_now, wall, clock=lambda: self._now()[1:])
        if reason:
            state = "IDLE" if reason in ("no active navigation request", "controller missing or stale") else "STALE_INPUT"
            if reason != "input source skew exceeds limit":
                self._reset_planner()
            self._publish_status(state, reason, math.inf, 0.0, 0.0)
            return
        with self._planner_lock:
            if snapshot.stamp <= self._last_planned_stamp:
                return
            # A request or cloud may have changed while this callback waited
            # for the previous computation. Do not contaminate planner history.
            _, plan_ros_now, plan_wall = self._now()
            if not self.runtime.revalidate(token, plan_ros_now, plan_wall):
                return
            result = (self._ego_planner.plan(snapshot) if getattr(self,"_ego_planner",None)
                      else self.planner.plan(snapshot))
            self._last_planned_stamp = snapshot.stamp
        _, commit_ros_now, commit_wall = self._now()
        if not self.runtime.revalidate(token, commit_ros_now, commit_wall):
            return
        self._publish_status(result.state, result.reason, result.clearance,
                             result.progress, snapshot.stamp, token=token)
        self._publish_preview(result, snapshot.stamp, token=token)
        if (not self.enable_output or not (result.publish_target or result.hold_allowed)
                ):
            return
        message = FlightSetpoint()
        message.header.stamp = rospy.Time.from_sec(snapshot.stamp)
        message.header.frame_id = self.odom_frame
        message.request_id = token.request_id
        self._fill_pose(message.pose, result.target,
                        self._terminal_orientation(token.request_id, result.state))
        self.runtime.commit_publish(
            token, snapshot.stamp, lambda: self._now()[1:],
            lambda: self.target_pub.publish(message))

    def _terminal_orientation(self, request_id, state):
        active_id, orientation = self.runtime.goal_orientation
        if state == "GOAL_REACHED" and request_id == active_id:
            return orientation
        return None

    def _reset_planner(self):
        with self._planner_lock:
            self.planner.reset_context()

    def _try_reset_planner(self):
        if self._planner_lock.acquire(blocking=False):
            try:
                self.planner.reset_context()
            finally:
                self._planner_lock.release()

    @staticmethod
    def _fill_pose(output, pose, orientation=None):
        output.position.x, output.position.y, output.position.z = pose.x, pose.y, pose.z
        if orientation is None:
            orientation = (0, 0, math.sin(pose.yaw / 2), math.cos(pose.yaw / 2))
        (output.orientation.x, output.orientation.y,
         output.orientation.z, output.orientation.w) = orientation

    def _publish_preview(self, result, stamp, token=None):
        path = Path()
        path.header.stamp = rospy.Time.from_sec(stamp)
        path.header.frame_id = self.odom_frame
        point = PoseStamped()
        point.header = path.header
        self._fill_pose(point.pose, result.target)
        path.poses = [point]
        if token is None:
            self.preview_pub.publish(path)
        else:
            self.runtime.commit_observation(
                token, lambda: self._now()[1:], lambda: self.preview_pub.publish(path))

    def _publish_status(self, state, reason, clearance, progress, stamp, token=None):
        with self._status_lock:
            self._publish_status_locked(state, reason, clearance, progress, stamp, token)

    def _publish_status_locked(self, state, reason, clearance, progress, stamp, token):
        now, ros_now, _ = self._now()
        message = NavigationStatus()
        # Status describes this decision. Sensor ages below retain source time;
        # targets and preview paths still use the accepted cloud timestamp.
        message.header.stamp = now
        message.header.frame_id = self.odom_frame
        message.request_id = token.request_id if token is not None else self.runtime.request_id
        message.state, message.reason = state, reason
        ages = self.runtime.ages(ros_now)
        if token is not None:
            for sample in token.samples:
                ages[sample.name] = ros_now - sample.stamp
        message.odom_age = ages["odom"]
        message.cloud_age = ages["cloud"]
        message.terrain_age = ages["terrain"]
        message.controller_age = ages["controller"]
        message.clearance, message.progress = clearance, progress
        if token is None:
            self.status_pub.publish(message)
        else:
            self.runtime.commit_observation(
                token, lambda: self._now()[1:], lambda: self.status_pub.publish(message))

    def _watchdog_loop(self, period):
        while not self._watchdog_stop.wait(period):
            _, ros_now, wall = self._now()
            _token, _snapshot, reason = self.runtime.snapshot(
                ros_now, wall, clock=lambda: self._now()[1:])
            if reason:
                state = "IDLE" if reason == "no active navigation request" else "STALE_INPUT"
                self._publish_status(state, reason, math.inf, 0.0, 0.0)
                if reason != "input source skew exceeds limit":
                    self._try_reset_planner()

    def _planning_loop(self):
        while not self._watchdog_stop.is_set():
            if not self._plan_event.wait(0.1):
                continue
            self._plan_event.clear()
            if self._watchdog_stop.is_set():
                return
            _, ros_now, wall = self._now()
            self._plan_once(ros_now, wall)

    def _shutdown(self):
        self._watchdog_stop.set()
        self._plan_event.set()


if __name__ == "__main__":
    rospy.init_node("ducted_navigation")
    try:
        NavigationNode()
        rospy.spin()
    except (ValueError, rospy.ROSInitException) as error:
        rospy.logfatal("navigation initialization failed: %s", error)
        raise SystemExit(1)
