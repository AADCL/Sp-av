#!/usr/bin/env python3
import math
import threading
from time import monotonic

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from std_msgs.msg import Bool

from ducted_bringup.fastlio_odometry import (
    AdapterError,
    adapt_odometry_values,
    quaternion_from_rpy,
    quaternion_inverse,
    quaternion_multiply,
    rotate_vector,
)


class FastlioOdometryAdapter:
    def __init__(self):
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.last_sample_wall = None
        self.last_sample_stamp = None
        self.last_stamp_ns = 0
        self.base_received_wall = None
        self.input_contract_confirmed = rospy.get_param("~input_contract_confirmed", False) is True
        self.map_frame = rospy.get_param("~frames/map", "map")
        self.align_with_fcu = rospy.get_param("~align_with_fcu", True) is True
        self.output_frame = rospy.get_param("~frames/output", "odom") if self.align_with_fcu else self.map_frame
        self.fcu_state = None
        self.fcu_state_wall = None
        self.fcu_attitude = None
        self.fcu_attitude_wall = None
        self.q_output_map = None
        self.sensor_frame = rospy.get_param("~frames/sensor", "livox_frame")
        self.base_frame = rospy.get_param("~frames/base", "base_link")
        self.max_age = float(rospy.get_param("~timeouts/max_odom_age", 0.25))
        self.future_tolerance = float(rospy.get_param("~timeouts/future_tolerance", 0.05))
        self.base_max_age = float(rospy.get_param("~timeouts/base_ready", 1.0))
        if (not all(math.isfinite(v) for v in (self.max_age, self.future_tolerance, self.base_max_age))
                or self.max_age <= 0 or self.base_max_age <= 0 or self.future_tolerance < 0):
            raise rospy.ROSInitException("invalid odometry timeout configuration")
        self.require_base_ready = bool(rospy.get_param("~require_base_ready", True))
        self.require_localization_ready = bool(
            rospy.get_param("~require_localization_ready", True)
        )

        translation = rospy.get_param("~extrinsic/base_to_livox/translation")
        rpy = rospy.get_param("~extrinsic/base_to_livox/rpy")
        if len(translation) != 3 or len(rpy) != 3:
            raise rospy.ROSInitException("extrinsic translation and rpy must have 3 values")
        self.translation = tuple(float(value) for value in translation)
        self.orientation = quaternion_from_rpy(*[float(value) for value in rpy])
        if not all(math.isfinite(v) for v in self.translation):
            raise rospy.ROSInitException("non-finite extrinsic translation")
        # FAST-LIO owns map->sensor. Attach base below sensor using the inverse;
        # publishing base->sensor as well would give sensor two parents.
        self.inverse_orientation = quaternion_inverse(self.orientation)
        self.inverse_translation = rotate_vector(
            self.inverse_orientation, tuple(-v for v in self.translation)
        )
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.pose_diagonal = self._diagonal_param("~covariance/pose_diagonal")
        self.twist_diagonal = self._diagonal_param("~covariance/twist_diagonal")
        self.base_ready = not self.require_base_ready
        self.localization_ready = not self.require_localization_ready

        self.publisher = rospy.Publisher("output", Odometry, queue_size=10)
        self.ready_publisher = rospy.Publisher("ready", Bool, queue_size=1, latch=True)
        self.ready_publisher.publish(False)
        rospy.Subscriber("base_ready", Bool, self._base_ready_callback, queue_size=1)
        rospy.Subscriber(
            "localization_ready", Bool, self._localization_ready_callback, queue_size=1
        )
        rospy.Subscriber("input", Odometry, self._odometry_callback, queue_size=10)
        rospy.Subscriber("fcu_state", State, self._state_callback, queue_size=1)
        rospy.Subscriber("fcu_attitude", Imu, self._attitude_callback, queue_size=1)
        if not self.input_contract_confirmed:
            rospy.logwarn("External odometry blocked: verify mount reference and upstream twist contract first")

    def start_watchdog(self):
        rospy.on_shutdown(self.stop_event.set)
        def run():
            while not self.stop_event.wait(0.05):
                self._watchdog()
        threading.Thread(target=run, daemon=True).start()

    def _state_callback(self, message):
        with self.lock:
            self.fcu_state = message
            self.fcu_state_wall = monotonic()

    def _attitude_callback(self, message):
        with self.lock:
            self.fcu_attitude = message
            self.fcu_attitude_wall = monotonic()

    def _align_pose(self, adapted):
        if not self.align_with_fcu:
            return adapted.position, adapted.child_orientation
        if self.q_output_map is None:
            now = monotonic()
            if (self.fcu_state is None or self.fcu_attitude is None or
                    not self.fcu_state.connected or self.fcu_state.armed or
                    now - self.fcu_state_wall > 2.0 or now - self.fcu_attitude_wall > self.max_age):
                raise AdapterError("alignment needs fresh connected/disarmed FCU and attitude")
            age = (rospy.Time.now() - self.fcu_attitude.header.stamp).to_sec()
            if age > self.max_age or age < -self.future_tolerance:
                raise AdapterError("stale FCU attitude during alignment")
            q = self.fcu_attitude.orientation
            # Normalize/validate via inverse twice; do not adopt an invalid quaternion.
            q_fcu = quaternion_inverse(quaternion_inverse((q.x, q.y, q.z, q.w)))
            self.q_output_map = quaternion_multiply(q_fcu, quaternion_inverse(adapted.child_orientation))
            rospy.loginfo("Fixed %s->%s rotation established: %s", self.output_frame,
                          self.map_frame, self.q_output_map)
        return (rotate_vector(self.q_output_map, adapted.position),
                quaternion_multiply(self.q_output_map, adapted.child_orientation))

    def _gates_open(self):
        return (self.input_contract_confirmed
                and (not self.require_base_ready or
                     (self.base_ready and self.base_received_wall is not None
                      and monotonic() - self.base_received_wall <= self.base_max_age))
                and (not self.require_localization_ready or self.localization_ready))

    def _watchdog(self):
        with self.lock:
            stale = self.last_sample_wall is None or monotonic() - self.last_sample_wall > self.max_age
            if self.last_sample_stamp is not None:
                age = (rospy.Time.now() - self.last_sample_stamp).to_sec()
                stale = stale or age > self.max_age or age < -self.future_tolerance
            if stale or not self._gates_open():
                self.ready_publisher.publish(False)

    @staticmethod
    def _diagonal_param(name):
        values = [float(value) for value in rospy.get_param(name)]
        if len(values) != 6 or any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise rospy.ROSInitException("{} must contain 6 positive finite values".format(name))
        return values

    def _base_ready_callback(self, message):
        with self.lock:
            self.base_ready = bool(message.data)
            self.base_received_wall = monotonic()
            if self.require_base_ready and not self.base_ready:
                self.ready_publisher.publish(False)

    def _localization_ready_callback(self, message):
        with self.lock:
            self.localization_ready = bool(message.data)
            if self.require_localization_ready and not self.localization_ready:
                self.ready_publisher.publish(False)

    @staticmethod
    def _covariance(diagonal):
        covariance = [0.0] * 36
        for index, value in enumerate(diagonal):
            covariance[index * 6 + index] = value
        return covariance

    def _odometry_callback(self, message):
        with self.lock:
            self._process_odometry(message)

    def _process_odometry(self, message):
        if not self._gates_open():
            self.ready_publisher.publish(False)
            return
        stamp_ns = message.header.stamp.to_nsec()
        if stamp_ns <= self.last_stamp_ns:
            self.ready_publisher.publish(False)
            rospy.logerr_throttle(1.0, "external odometry rejected: non-increasing timestamp")
            return
        if message.header.frame_id.lstrip("/") != self.map_frame.lstrip("/"):
            rospy.logerr_throttle(1.0, "external odometry rejected: expected map frame %s", self.map_frame)
            self.ready_publisher.publish(False)
            return
        if message.child_frame_id.lstrip("/") != self.sensor_frame.lstrip("/"):
            rospy.logerr_throttle(
                1.0, "external odometry rejected: expected sensor frame %s", self.sensor_frame
            )
            self.ready_publisher.publish(False)
            return

        age = (rospy.Time.now() - message.header.stamp).to_sec()
        if age > self.max_age or age < -self.future_tolerance:
            rospy.logerr_throttle(1.0, "external odometry rejected: timestamp age %.3f s", age)
            self.ready_publisher.publish(False)
            return

        pose = message.pose.pose
        twist = message.twist.twist
        try:
            adapted = adapt_odometry_values(
                position=(pose.position.x, pose.position.y, pose.position.z),
                orientation=(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                linear_velocity_map=(twist.linear.x, twist.linear.y, twist.linear.z),
                angular_velocity_sensor=(twist.angular.x, twist.angular.y, twist.angular.z),
                base_to_sensor_translation=self.translation,
                base_to_sensor_orientation=self.orientation,
            )
            output_position, output_orientation = self._align_pose(adapted)
        except AdapterError as error:
            rospy.logerr_throttle(1.0, "external odometry rejected: %s", error)
            self.ready_publisher.publish(False)
            return

        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = output_position
        (
            output.pose.pose.orientation.x,
            output.pose.pose.orientation.y,
            output.pose.pose.orientation.z,
            output.pose.pose.orientation.w,
        ) = output_orientation
        (
            output.twist.twist.linear.x,
            output.twist.twist.linear.y,
            output.twist.twist.linear.z,
        ) = adapted.linear_velocity_base
        (
            output.twist.twist.angular.x,
            output.twist.twist.angular.y,
            output.twist.twist.angular.z,
        ) = adapted.angular_velocity_base
        output.pose.covariance = self._covariance(self.pose_diagonal)
        output.twist.covariance = self._covariance(self.twist_diagonal)
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.sensor_frame
        transform.child_frame_id = self.base_frame
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = self.inverse_translation
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = self.inverse_orientation
        if self.align_with_fcu:
            alignment = TransformStamped()
            alignment.header.stamp = message.header.stamp
            alignment.header.frame_id = self.output_frame
            alignment.child_frame_id = self.map_frame
            (alignment.transform.rotation.x, alignment.transform.rotation.y,
             alignment.transform.rotation.z, alignment.transform.rotation.w) = self.q_output_map
            self.broadcaster.sendTransform([alignment, transform])
        else:
            self.broadcaster.sendTransform(transform)
        self.publisher.publish(output)
        self.last_sample_wall = monotonic()
        self.last_sample_stamp = message.header.stamp
        self.last_stamp_ns = stamp_ns
        self.ready_publisher.publish(True)


if __name__ == "__main__":
    rospy.init_node("fastlio_odometry_adapter")
    adapter = FastlioOdometryAdapter()
    adapter.start_watchdog()
    rospy.spin()
