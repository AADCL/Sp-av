#!/usr/bin/env python3
"""Forward validated body odometry to MAVROS without owning or changing TF."""
import math
import threading
from time import monotonic

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class ExternalOdometryRelay:
    def __init__(self):
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.confirmed = rospy.get_param('~input_contract_confirmed', False) is True
        self.require_base = rospy.get_param('~require_base_ready', True) is True
        self.require_localization = rospy.get_param('~require_localization_ready', True) is True
        self.frame = rospy.get_param('~frames/output', 'odom')
        self.child = rospy.get_param('~frames/base', 'base_link')
        self.max_age = float(rospy.get_param('~timeouts/max_odom_age', .25))
        self.future = float(rospy.get_param('~timeouts/future_tolerance', .05))
        self.base_timeout = float(rospy.get_param('~timeouts/base_ready', 1.))
        if (not all(math.isfinite(x) for x in (self.max_age, self.future, self.base_timeout))
                or self.max_age <= 0 or self.future < 0 or self.base_timeout <= 0):
            raise rospy.ROSInitException('invalid relay timeouts')
        self.base_ready = False
        self.localization_ready = False
        self.base_wall = None
        self.last_stamp_ns = 0
        self.last_valid_stamp = None
        self.last_valid_wall = None
        self.publisher = rospy.Publisher('output', Odometry, queue_size=10)
        self.ready_publisher = rospy.Publisher('ready', Bool, queue_size=1, latch=True)
        self.ready_publisher.publish(False)
        rospy.Subscriber('base_ready', Bool, self._base_ready_callback, queue_size=1)
        rospy.Subscriber('localization_ready', Bool, self._localization_ready_callback, queue_size=1)
        rospy.Subscriber('input', Odometry, self._odometry_callback, queue_size=10)
        if not self.confirmed:
            rospy.logwarn('External odometry transmission disabled; localization TF remains available')

    def _gates_open(self):
        return (self.confirmed and (not self.require_base or
                (self.base_ready and self.base_wall is not None and
                 monotonic() - self.base_wall <= self.base_timeout)) and
                (not self.require_localization or self.localization_ready))

    def _base_ready_callback(self, msg):
        with self.lock:
            self.base_ready, self.base_wall = bool(msg.data), monotonic()
            if not self._gates_open():
                self.ready_publisher.publish(False)

    def _localization_ready_callback(self, msg):
        with self.lock:
            self.localization_ready = bool(msg.data)
            if not self._gates_open():
                self.ready_publisher.publish(False)

    def _fresh(self, stamp):
        age = (rospy.Time.now() - stamp).to_sec()
        return math.isfinite(age) and -self.future <= age <= self.max_age

    def _valid_payload(self, msg):
        if msg.header.frame_id != self.frame or msg.child_frame_id != self.child:
            return False
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]
        if not all(math.isfinite(x) for x in values):
            return False
        if abs(sum(x*x for x in (q.x, q.y, q.z, q.w)) - 1.) > .01:
            return False
        for covariance in (msg.pose.covariance, msg.twist.covariance):
            if (len(covariance) != 36 or not all(math.isfinite(x) for x in covariance)
                    or any(covariance[i*7] < 0 for i in range(6))):
                return False
        return True

    def _odometry_callback(self, msg):
        with self.lock:
            stamp = msg.header.stamp.to_nsec()
            if not self._gates_open() or not self._fresh(msg.header.stamp) or stamp <= self.last_stamp_ns:
                self.ready_publisher.publish(False)
                return
            self.last_stamp_ns = stamp
            if not self._valid_payload(msg) or not self._gates_open() or not self._fresh(msg.header.stamp):
                self.ready_publisher.publish(False)
                return
            self.publisher.publish(msg)
            self.last_valid_stamp, self.last_valid_wall = msg.header.stamp, monotonic()
            self.ready_publisher.publish(True)

    def _watchdog(self):
        with self.lock:
            if (not self._gates_open() or self.last_valid_wall is None or
                    monotonic() - self.last_valid_wall > self.max_age or
                    not self._fresh(self.last_valid_stamp)):
                self.ready_publisher.publish(False)

    def start_watchdog(self):
        rospy.on_shutdown(self.stop_event.set)
        def run():
            while not self.stop_event.wait(.05):
                self._watchdog()
        threading.Thread(target=run, daemon=True).start()


if __name__ == '__main__':
    rospy.init_node('external_odometry_relay')
    node = ExternalOdometryRelay()
    node.start_watchdog()
    rospy.spin()
