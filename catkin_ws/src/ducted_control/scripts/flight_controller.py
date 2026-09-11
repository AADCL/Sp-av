#!/usr/bin/env python3
"""ROS boundary for bounded PX4 native position control."""
import math
from queue import Empty, Full, Queue
import sys
import threading
from time import monotonic

import rospy
from ducted_msgs.msg import FlightControlStatus, FlightSetpoint, RCState
from ducted_msgs.srv import FlightCommand, FlightCommandResponse
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from ducted_control.flight import FlightConfig, FlightPolicy, Pose


class FlightController:
    def __init__(self):
        raw_config = dict(rospy.get_param('~flight'))
        self.loop_hz = float(raw_config.pop('loop_hz', 20.))
        if not math.isfinite(self.loop_hz) or self.loop_hz < 20. or self.loop_hz > 100.:
            raise ValueError('loop_hz must be finite and between 20 and 100 Hz')
        self.policy = FlightPolicy(FlightConfig(raw_config))
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.mode_queue = Queue(maxsize=1)
        self.mode_client = None
        self.mode_worker = None

        self.setpoint_pub = rospy.Publisher('setpoint', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('status', FlightControlStatus, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher('ready', Bool, queue_size=1, latch=True)
        self.ready_pub.publish(Bool(data=False))

        self.fcu_sub = rospy.Subscriber('fcu_state', State, self._fcu_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('local_pose', PoseStamped, self._pose_callback, queue_size=1)
        self.external_pose_sub = rospy.Subscriber(
            'external_pose', Odometry, self._external_pose_callback, queue_size=1)
        self.landed_sub = rospy.Subscriber(
            'extended_state', ExtendedState, self._landed_callback, queue_size=1)
        self.rc_sub = rospy.Subscriber('rc_state', RCState, self._rc_callback, queue_size=1)
        self.base_sub = rospy.Subscriber('base_ready', Bool, self._base_callback, queue_size=1)
        self.external_sub = rospy.Subscriber(
            'external_ready', Bool, self._external_callback, queue_size=1)
        self.target_sub = rospy.Subscriber(
            'target', FlightSetpoint, self._target_callback, queue_size=1)
        self.automatic_sub = rospy.Subscriber(
            '/ducted/automatic/flight_lease', Bool, self._automatic_callback, queue_size=1)
        self.command_srv = rospy.Service('command', FlightCommand, self._command_callback)

        if self.policy.config.enable_flight_output:
            self.mode_worker = threading.Thread(target=self._mode_loop, daemon=True)
            self.mode_worker.start()

    @staticmethod
    def _stamp(message):
        try:
            return float(message.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            return math.nan

    def _automatic_callback(self, message):
        with self.lock:
            self.policy.update_automatic_lease(bool(message.data), monotonic())

    @staticmethod
    def _pose_from_message(message):
        try:
            p, q = message.position, message.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(float(value)) for value in values):
                return None
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if abs(norm - 1.) > 1e-3:
                return None
            yaw = math.atan2(2. * (q.w*q.z + q.x*q.y),
                             1. - 2. * (q.y*q.y + q.z*q.z))
            return Pose(float(p.x), float(p.y), float(p.z), yaw)
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _write_pose(message, pose):
        message.position.x, message.position.y, message.position.z = pose.x, pose.y, pose.z
        message.orientation.x = 0.
        message.orientation.y = 0.
        message.orientation.z = math.sin(pose.yaw / 2.)
        message.orientation.w = math.cos(pose.yaw / 2.)

    def _fcu_callback(self, message):
        with self.lock:
            now, wall = rospy.get_time(), monotonic()
            self.policy.update_fcu(bool(message.connected), bool(message.armed),
                                   str(message.mode), int(message.system_status),
                                   self._stamp(message), now, wall)

    def _pose_callback(self, message):
        pose = self._pose_from_message(message.pose)
        if str(message.header.frame_id) != self.policy.config.frame_id:
            pose = None
        if pose is None:
            pose = Pose(math.nan, math.nan, math.nan, math.nan)
        with self.lock:
            now, wall = rospy.get_time(), monotonic()
            self.policy.update_pose(pose, self._stamp(message), now, wall)

    def _landed_callback(self, message):
        with self.lock:
            now, wall = rospy.get_time(), monotonic()
            self.policy.update_landed(int(message.landed_state), self._stamp(message), now, wall)

    def _external_pose_callback(self, message):
        pose = self._pose_from_message(message.pose.pose)
        if str(message.header.frame_id) != self.policy.config.frame_id:
            pose = None
        if pose is None:
            pose = Pose(math.nan, math.nan, math.nan, math.nan)
        with self.lock:
            now, wall = rospy.get_time(), monotonic()
            self.policy.update_external_pose(pose, self._stamp(message), now, wall)

    def _rc_callback(self, message):
        with self.lock:
            now, wall = rospy.get_time(), monotonic()
            self.policy.update_rc(bool(message.valid), str(message.mode),
                                  bool(message.kill_switch), bool(message.land_switch),
                                  self._stamp(message), now, wall)

    def _base_callback(self, message):
        with self.lock:
            self.policy.update_base_ready(bool(message.data), monotonic())

    def _external_callback(self, message):
        with self.lock:
            self.policy.update_external_ready(bool(message.data), monotonic())

    def _target_callback(self, message):
        pose = self._pose_from_message(message.pose)
        if pose is None:
            pose = Pose(math.nan, math.nan, math.nan, math.nan)
        with self.lock:
            now, wall = rospy.get_time(), monotonic()
            result = self.policy.accept_target(str(message.request_id), pose,
                                               str(message.header.frame_id),
                                               self._stamp(message), now, wall)
        if not result.accepted:
            rospy.logwarn_throttle(1., 'Flight target rejected: %s', result.message)

    def _command_callback(self, request):
        target = None
        command = str(request.command).strip().lower()
        if command == 'takeoff':
            stamp_value = self._stamp(request.target)
            frame = str(request.target.header.frame_id)
            target = self._pose_from_message(request.target.pose)
            if target is None or frame != self.policy.config.frame_id or stamp_value <= 0:
                return FlightCommandResponse(
                    accepted=False, message='takeoff requires a fresh finite absolute odom target')
        with self.lock:
            now, wall = rospy.get_time(), monotonic()
            if (command == 'takeoff' and not
                    -self.policy.config.future_tolerance <= now - stamp_value
                    <= self.policy.config.target_timeout):
                return FlightCommandResponse(
                    accepted=False, message='takeoff requires a fresh finite absolute odom target')
            result = self.policy.command(command, target, now, wall)
        return FlightCommandResponse(accepted=result.accepted, message=result.message)

    def _setpoint_message(self, pose):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.policy.config.frame_id
        self._write_pose(message.pose, pose)
        return message

    def _status_message(self, status):
        message = FlightControlStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.policy.config.frame_id
        message.state = status.state
        message.reason = status.reason
        message.ready = status.ready
        message.request_id = status.request_id
        self._write_pose(message.target, status.target)
        return message

    def _queue_mode(self, action):
        try:
            self.mode_queue.put_nowait(action)
        except Full:
            with self.lock:
                self.policy.mode_result(action.token, False, monotonic())

    def _dispatch_actions(self, actions):
        for action in actions:
            with self.lock:
                if not self.policy.action_current(action, rospy.get_time(), monotonic()):
                    continue
                if action.kind == 'setpoint':
                    # queue_size=1 makes this a nonblocking handoff to rospy's publisher thread.
                    self.setpoint_pub.publish(self._setpoint_message(action.pose))
                elif action.kind == 'mode':
                    self._queue_mode(action)

    def _watchdog(self):
        with self.lock:
            # Sampling before the lock can predate a callback that acquires it
            # first, producing negative receipt ages for healthy telemetry.
            now, wall = rospy.get_time(), monotonic()
            actions = self.policy.tick(now, wall)
        self._dispatch_actions(actions)
        with self.lock:
            status = self.policy.snapshot()
        self.status_pub.publish(self._status_message(status))
        self.ready_pub.publish(Bool(data=status.ready))

    def _mode_loop(self):
        while not self.stop_event.is_set():
            try:
                action = self.mode_queue.get(timeout=.1)
            except Empty:
                continue
            if action is None:
                return
            with self.lock:
                if not self.policy.action_current(action, rospy.get_time(), monotonic()):
                    continue
            accepted = False
            try:
                rospy.wait_for_service('set_mode', timeout=self.policy.config.mode_request_timeout)
                with self.lock:
                    if not self.policy.action_current(action, rospy.get_time(), monotonic()):
                        continue
                    if not self.policy.mode_dispatched(action.token, monotonic()):
                        continue
                if self.mode_client is None:
                    self.mode_client = rospy.ServiceProxy('set_mode', SetMode, persistent=True)
                accepted = bool(self.mode_client(0, action.mode).mode_sent)
            except Exception as error:
                self.mode_client = None
                rospy.logerr('Mode request %s failed: %s', action.mode, error)
            with self.lock:
                self.policy.mode_result(action.token, accepted, monotonic())

    def start(self):
        rospy.on_shutdown(self.shutdown)

        def run():
            period = 1. / self.loop_hz
            while not self.stop_event.wait(period):
                self._watchdog()
        threading.Thread(target=run, daemon=True).start()

    def shutdown(self):
        self.stop_event.set()
        try:
            self.mode_queue.put_nowait(None)
        except Full:
            pass


def main():
    rospy.init_node('flight_controller')
    node = FlightController()
    node.start()
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
