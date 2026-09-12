#!/usr/bin/env python3
import math
import sys
import threading
from collections import deque
from dataclasses import dataclass
from time import monotonic

import rospy
import tf2_ros
from ducted_msgs.msg import TerrainHeight
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from ducted_bringup.terrain_height import (
    TerrainEstimate,
    TerrainEstimator,
    lowest_body_z,
    normalize_quaternion,
    transform_point,
    transform_cloud_points,
    validate_body_vertices,
)


def _frame_matches(actual, expected):
    return bool(actual) and actual.lstrip("/") == expected.lstrip("/")


@dataclass(frozen=True)
class CloudSnapshot:
    message: object
    stamp: object
    stamp_ns: int
    cloud_generation: int
    odom: object
    odom_stamp: object
    odom_stamp_ns: int
    odom_generation: int
    odom_received_wall: float
    revocation_generation: int


class TerrainHeightNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.odom_condition = threading.Condition(self.lock)
        self.odom_history = deque(maxlen=32)
        self.stop_event = threading.Event()
        self.map_frame = rospy.get_param("~frames/map", "map")
        self.output_frame = rospy.get_param("~frames/output", "odom")
        self.sensor_frame = rospy.get_param("~frames/sensor", "livox_frame")
        self.base_frame = rospy.get_param("~frames/base", "base_link")
        frames = (
            self.map_frame,
            self.output_frame,
            self.sensor_frame,
            self.base_frame,
        )
        if any(not isinstance(frame, str) or not frame.strip() for frame in frames):
            raise rospy.ROSInitException(
                "terrain frame names must be nonempty strings"
            )

        self.odom_timeout = self._positive_param("~timeouts/odom", 0.5)
        self.cloud_timeout = self._positive_param("~timeouts/cloud", 0.5)
        self.base_timeout = self._positive_param("~timeouts/base_ready", 1.0)
        self.max_header_age = self._positive_param(
            "~timeouts/max_header_age", 0.25
        )
        self.future_tolerance = self._nonnegative_param(
            "~timeouts/future_tolerance", 0.05
        )
        self.odom_cloud_skew = self._nonnegative_param(
            "~timeouts/odom_cloud_skew", 0.05
        )
        self.tf_timeout = self._nonnegative_param("~timeouts/tf_lookup", 0.05)

        self.require_base_ready = bool(rospy.get_param("~require_base_ready", True))
        self.base_ready_topic = rospy.get_param(
            "~base_ready_topic", "/ducted/system/ready"
        )
        self.reference_confirmed = (
            rospy.get_param("~reference_confirmed", False) is True
        )
        self.agl_reference = rospy.get_param("~agl_reference", "base_link")
        if self.agl_reference not in ("base_link", "body_bottom"):
            raise rospy.ROSInitException(
                "agl_reference must be base_link or body_bottom"
            )
        raw_vertices = rospy.get_param("~body_vertices", [])
        try:
            self.body_vertices = (
                validate_body_vertices(raw_vertices)
                if raw_vertices or self.agl_reference == "body_bottom"
                else tuple()
            )
        except ValueError as error:
            raise rospy.ROSInitException(str(error))

        try:
            raw_min_points = rospy.get_param("~terrain/min_points", 20)
            if isinstance(raw_min_points, bool) or int(raw_min_points) != raw_min_points:
                raise ValueError("min_points must be an integer")
            self.estimator = TerrainEstimator(
                radius=float(rospy.get_param("~terrain/radius", 1.0)),
                ground_quantile=float(
                    rospy.get_param("~terrain/ground_quantile", 0.25)
                ),
                ground_band=float(rospy.get_param("~terrain/ground_band", 0.15)),
                min_points=int(raw_min_points),
                max_below=float(rospy.get_param("~terrain/max_below", 5.0)),
                max_above=float(rospy.get_param("~terrain/max_above", 0.1)),
                min_agl=float(rospy.get_param("~terrain/min_agl", 0.0)),
                max_agl=float(rospy.get_param("~terrain/max_agl", 4.0)),
                max_variance=float(
                    rospy.get_param("~terrain/max_variance", 0.03)
                ),
                max_slope=float(rospy.get_param("~terrain/max_slope", 1.0)),
                min_planar_variance=float(
                    rospy.get_param("~terrain/min_planar_variance", 0.0025)
                ),
                min_support_extent=float(
                    rospy.get_param("~terrain/min_support_extent", 0.1)
                ),
                max_step_height=float(
                    rospy.get_param("~terrain/max_step_height", 5.0)
                ),
            )
        except (TypeError, ValueError) as error:
            raise rospy.ROSInitException(
                "invalid terrain estimator configuration: {}".format(error)
            )

        self.odom = None
        self.odom_received_wall = None
        self.cloud_received_wall = None
        self.base_received_wall = None
        # Seen watermarks advance before payload validation. Accepted stamps
        # remain separate so malformed data cannot become an age/skew source.
        self.last_odom_stamp_ns = 0
        self.last_cloud_stamp_ns = 0
        self.accepted_odom_stamp = None
        self.accepted_odom_stamp_ns = 0
        self.accepted_cloud_stamp = None
        self.accepted_cloud_stamp_ns = 0
        self.odom_generation = 0
        self.cloud_generation = 0
        self.revocation_generation = 0
        self.base_ready = not self.require_base_ready
        self.ready = False
        self.last_watchdog_reason = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.height_pub = rospy.Publisher("height", TerrainHeight, queue_size=5)
        self.ready_pub = rospy.Publisher("ready", Bool, queue_size=1, latch=True)
        self.ready_pub.publish(False)
        rospy.Subscriber("odom", Odometry, self._odom_callback, queue_size=5)
        rospy.Subscriber("cloud", PointCloud2, self._cloud_callback, queue_size=1)
        rospy.Subscriber(
            self.base_ready_topic, Bool, self._base_ready_callback, queue_size=1
        )
        if not self.reference_confirmed:
            rospy.logwarn(
                "Terrain height blocked: confirm the configured AGL reference geometry"
            )

    @staticmethod
    def _positive_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException(
                "{} must be positive and finite".format(name)
            )
        return value

    @staticmethod
    def _nonnegative_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value < 0.0:
            raise rospy.ROSInitException(
                "{} must be nonnegative and finite".format(name)
            )
        return value

    def start_watchdog(self):
        rospy.on_shutdown(self.stop_event.set)

        def run():
            while not self.stop_event.wait(0.05):
                self._watchdog()

        threading.Thread(target=run, daemon=True).start()

    def _publish(self, estimate, stamp):
        if not estimate.valid:
            rospy.logwarn_throttle(2.0, "Terrain height invalid: %s", estimate.reason)
        message = TerrainHeight()
        message.header.stamp = stamp
        message.header.frame_id = self.output_frame
        message.ground_z = estimate.ground_z
        message.agl = estimate.agl
        message.variance = estimate.variance
        message.valid = estimate.valid
        message.reason = estimate.reason
        self.height_pub.publish(message)
        self.ready_pub.publish(estimate.valid)
        self.ready = estimate.valid
        if estimate.valid:
            self.last_watchdog_reason = None

    def _invalid(self, reason, stamp=None):
        self._publish(
            TerrainEstimate(math.nan, math.nan, math.inf, False, reason),
            stamp if stamp is not None else rospy.Time.now(),
        )

    @staticmethod
    def _pose_values(message):
        pose = message.pose.pose
        return (
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _stamp_ns(self, stamp, source):
        try:
            stamp_ns = stamp.to_nsec()
            age = (rospy.Time.now() - stamp).to_sec()
        except (AttributeError, TypeError, ValueError):
            raise ValueError("{} timestamp is malformed".format(source))
        if stamp_ns <= 0:
            raise ValueError("{} timestamp is zero".format(source))
        if not math.isfinite(age) or age > self.max_header_age:
            raise ValueError("{} timestamp is stale".format(source))
        if age < -self.future_tolerance:
            raise ValueError("{} timestamp is in the future".format(source))
        return stamp_ns

    def _clear_odom(self):
        self.odom_history.clear()
        self.revocation_generation += 1
        self.odom = None
        self.odom_received_wall = None
        self.accepted_odom_stamp = None
        self.accepted_odom_stamp_ns = 0

    def _odom_callback(self, message):
        with self.lock:
            self.odom_generation += 1
            try:
                stamp_ns = self._stamp_ns(message.header.stamp, "odometry")
                if stamp_ns <= self.last_odom_stamp_ns:
                    self._clear_odom()
                    self._invalid(
                        "odometry timestamp is duplicate or backwards",
                        message.header.stamp,
                    )
                    return
                self.last_odom_stamp_ns = stamp_ns
                if not _frame_matches(message.header.frame_id, self.map_frame):
                    raise ValueError("odometry frame is not map")
                if not _frame_matches(message.child_frame_id, self.sensor_frame):
                    raise ValueError(
                        "odometry child frame is not the configured sensor"
                    )
                position, orientation = self._pose_values(message)
                if not all(math.isfinite(float(value)) for value in position):
                    raise ValueError("odometry position is non-finite")
                normalize_quaternion(orientation)
            except (AttributeError, TypeError, ValueError) as error:
                self._clear_odom()
                stamp = getattr(getattr(message, "header", None), "stamp", None)
                self._invalid(str(error), stamp)
                return
            self.odom = message
            self.odom_received_wall = monotonic()
            self.accepted_odom_stamp = message.header.stamp
            self.accepted_odom_stamp_ns = stamp_ns
            self.odom_history.append((message, self.odom_received_wall, stamp_ns))
            self.odom_condition.notify_all()

    def _base_gate_open(self, now_wall):
        return (
            not self.require_base_ready
            or (
                self.base_ready
                and self.base_received_wall is not None
                and now_wall - self.base_received_wall <= self.base_timeout
            )
        )

    def _base_ready_callback(self, message):
        with self.lock:
            self.base_ready = bool(message.data)
            self.base_received_wall = monotonic()
            if self.require_base_ready and not self.base_ready:
                self.revocation_generation += 1
                self._invalid("base system is not ready")

    @staticmethod
    def _transform_values(message):
        transform = message.transform
        translation = (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        )
        orientation = normalize_quaternion(
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
        )
        if not all(math.isfinite(float(value)) for value in translation):
            raise ValueError("TF translation is non-finite")
        return translation, orientation

    def _cloud_callback(self, message):
        with self.lock:
            snapshot = self._prepare_cloud(message)
        if snapshot is None:
            return

        deadline = monotonic() + self.tf_timeout
        try:
            map_to_output = self._lookup_exact_transform(
                self.output_frame, self.map_frame, snapshot.stamp, deadline
            )
            base_to_output = self._lookup_exact_transform(
                self.output_frame, self.base_frame, snapshot.stamp, deadline
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            self._invalidate_snapshot(
                snapshot, "exact-time TF unavailable: {}".format(error)
            )
            return

        with self.lock:
            if not self._recheck_snapshot(snapshot):
                return
        try:
            map_translation, map_orientation = self._transform_values(map_to_output)
            base_position, base_orientation = self._transform_values(base_to_output)
            reference_z = (
                base_position[2]
                if self.agl_reference == "base_link"
                else lowest_body_z(
                    self.body_vertices, base_position, base_orientation
                )
            )
            raw_points = point_cloud2.read_points(
                snapshot.message, field_names=("x", "y", "z"), skip_nans=True
            )
            points = transform_cloud_points(raw_points, map_translation, map_orientation)
            estimate = self.estimator.estimate(
                points, base_position[0], base_position[1], reference_z
            )
        except (AttributeError, TypeError, ValueError) as error:
            self._invalidate_snapshot(
                snapshot,
                "invalid TF, cloud, or reference geometry: {}".format(error),
            )
            return
        with self.lock:
            if self._recheck_snapshot(snapshot):
                self._publish(estimate, snapshot.stamp)

    def _matching_odom(self, cloud_stamp_ns):
        candidates = [sample for sample in self.odom_history
                      if abs(sample[2] - cloud_stamp_ns) <= self.odom_cloud_skew * 1e9]
        return min(candidates, key=lambda sample: abs(sample[2] - cloud_stamp_ns)) if candidates else None

    def _prepare_cloud(self, message):
        self.cloud_generation += 1
        try:
            stamp_ns = self._stamp_ns(message.header.stamp, "point cloud")
        except (AttributeError, TypeError, ValueError) as error:
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            self._invalid(str(error), stamp)
            return
        if stamp_ns <= self.last_cloud_stamp_ns:
            self._invalid(
                "point cloud timestamp is duplicate or backwards",
                message.header.stamp,
            )
            return
        self.last_cloud_stamp_ns = stamp_ns
        if not _frame_matches(message.header.frame_id, self.map_frame):
            self._invalid("point cloud frame is not map", message.header.stamp)
            return

        now_wall = monotonic()
        self.cloud_received_wall = now_wall
        self.accepted_cloud_stamp = message.header.stamp
        self.accepted_cloud_stamp_ns = stamp_ns
        if not self.reference_confirmed:
            self._invalid("AGL reference geometry is not confirmed", message.header.stamp)
            return
        if not self._base_gate_open(now_wall):
            self._invalid(
                "base ready heartbeat is unavailable or stale", message.header.stamp
            )
            return
        generation = self.cloud_generation
        # ROS topics arrive independently. Give the matching odometry a bounded
        # opportunity to arrive without holding up the watchdog or callbacks.
        if self._matching_odom(stamp_ns) is None and self.accepted_odom_stamp_ns < stamp_ns:
            self.odom_condition.wait_for(
                lambda: (self._matching_odom(stamp_ns) is not None
                         or self.accepted_odom_stamp_ns >= stamp_ns
                         or self.cloud_generation != generation
                         or self.stop_event.is_set()),
                timeout=self.tf_timeout)
        if generation != self.cloud_generation or self.stop_event.is_set():
            return
        now_wall = monotonic()
        if not self._base_gate_open(now_wall):
            self._invalid("base ready heartbeat is unavailable or stale", message.header.stamp)
            return
        if self.odom is None or self.odom_received_wall is None:
            self._invalid("odometry unavailable", message.header.stamp)
            return
        if now_wall - self.odom_received_wall > self.odom_timeout:
            self._invalid("odometry arrival is stale", message.header.stamp)
            return
        try:
            odom_stamp_ns = self._stamp_ns(
                self.accepted_odom_stamp, "odometry"
            )
            if odom_stamp_ns != self.accepted_odom_stamp_ns:
                raise ValueError("odometry timestamp changed after acceptance")
            matched = self._matching_odom(stamp_ns)
            if matched is None:
                skew = math.inf
            else:
                matched_odom, matched_wall, matched_ns = matched
                if self._stamp_ns(matched_odom.header.stamp, "paired odometry") != matched_ns:
                    raise ValueError("paired odometry timestamp changed after acceptance")
                if now_wall - matched_wall > self.odom_timeout:
                    raise ValueError("paired odometry arrival is stale")
                skew = abs(stamp_ns - matched_ns) * 1e-9
        except (AttributeError, TypeError, ValueError) as error:
            self.odom_generation += 1
            self._clear_odom()
            self._invalid(str(error), message.header.stamp)
            return
        if not math.isfinite(skew) or skew > self.odom_cloud_skew:
            self._invalid(
                "odometry and point cloud timestamps are skewed", message.header.stamp
            )
            return
        return CloudSnapshot(
            message=message,
            stamp=message.header.stamp,
            stamp_ns=stamp_ns,
            cloud_generation=self.cloud_generation,
            odom=matched_odom,
            odom_stamp=matched_odom.header.stamp,
            odom_stamp_ns=matched_ns,
            odom_generation=self.odom_generation,
            odom_received_wall=matched_wall,
            revocation_generation=self.revocation_generation,
        )

    def _lookup_exact_transform(self, target, source, stamp, deadline):
        while True:
            try:
                return self.tf_buffer.lookup_transform(
                    target, source, stamp, rospy.Duration(0.0)
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                remaining = deadline - monotonic()
                if remaining <= 0.0 or self.stop_event.is_set():
                    raise
                self.stop_event.wait(min(0.005, remaining))

    def _snapshot_current(self, snapshot):
        return (
            snapshot.cloud_generation == self.cloud_generation
            and snapshot.revocation_generation == self.revocation_generation
            and any(sample[0] is snapshot.odom and sample[2] == snapshot.odom_stamp_ns
                    for sample in self.odom_history)
            and snapshot.stamp_ns == self.accepted_cloud_stamp_ns
        )

    def _recheck_snapshot(self, snapshot):
        if not self._snapshot_current(snapshot):
            return False
        now_wall = monotonic()
        if not self.reference_confirmed:
            self._invalid("AGL reference geometry is not confirmed", snapshot.stamp)
            return False
        if not self._base_gate_open(now_wall):
            self._invalid(
                "base ready heartbeat is unavailable or stale", snapshot.stamp
            )
            return False
        if (
            self.odom_received_wall is None
            or now_wall - self.odom_received_wall > self.odom_timeout
            or now_wall - snapshot.odom_received_wall > self.odom_timeout
        ):
            self._invalid("odometry arrival is stale", snapshot.stamp)
            return False
        if (
            self.cloud_received_wall is None
            or now_wall - self.cloud_received_wall > self.cloud_timeout
        ):
            self._invalid("point cloud arrival is stale", snapshot.stamp)
            return False
        try:
            odom_stamp_ns = self._stamp_ns(snapshot.odom_stamp, "odometry")
            cloud_stamp_ns = self._stamp_ns(snapshot.stamp, "point cloud")
        except (AttributeError, TypeError, ValueError) as error:
            self.odom_generation += 1
            self._clear_odom()
            self._invalid(str(error), snapshot.stamp)
            return False
        if (
            odom_stamp_ns != snapshot.odom_stamp_ns
            or cloud_stamp_ns != snapshot.stamp_ns
        ):
            self.odom_generation += 1
            self._clear_odom()
            self._invalid("source timestamp changed after snapshot", snapshot.stamp)
            return False
        return True

    def _invalidate_snapshot(self, snapshot, reason):
        with self.lock:
            if self._snapshot_current(snapshot):
                self._invalid(reason, snapshot.stamp)

    def _watchdog(self):
        with self.lock:
            now_wall = monotonic()
            reason = None
            if not self._base_gate_open(now_wall):
                reason = "base ready heartbeat is unavailable or stale"
            elif (
                self.odom_received_wall is None
                or now_wall - self.odom_received_wall > self.odom_timeout
            ):
                reason = "odometry unavailable or stale"
            elif (
                self.cloud_received_wall is None
                or now_wall - self.cloud_received_wall > self.cloud_timeout
            ):
                reason = "point cloud unavailable or stale"
            elif self.ready:
                try:
                    self._stamp_ns(self.accepted_odom_stamp, "odometry")
                    self._stamp_ns(self.accepted_cloud_stamp, "point cloud")
                except (AttributeError, TypeError, ValueError) as error:
                    reason = str(error)
            if reason is not None and (
                self.ready or reason != self.last_watchdog_reason
            ):
                self.last_watchdog_reason = reason
                self.revocation_generation += 1
                self._invalid(reason)


def wait_for_base_ready():
    if not rospy.get_param("~require_base_ready", True):
        return True
    topic = rospy.get_param("~base_ready_topic", "/ducted/system/ready")
    timeout = float(rospy.get_param("~base_ready_timeout", 15.0))
    if not math.isfinite(timeout) or timeout <= 0.0:
        rospy.logfatal("base_ready_timeout must be positive and finite")
        return False
    deadline = monotonic() + timeout
    rospy.loginfo("Waiting for ready base system on %s", topic)
    while not rospy.is_shutdown():
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            break
        try:
            if rospy.wait_for_message(
                topic, Bool, timeout=min(1.0, max(0.05, remaining))
            ).data:
                return True
        except rospy.ROSException:
            pass
    rospy.logfatal(
        "Base system is not ready on %s; start base_system.launch first", topic
    )
    return False


def main():
    rospy.init_node("terrain_height")
    if not wait_for_base_ready():
        return 2
    node = TerrainHeightNode()
    node.start_watchdog()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
