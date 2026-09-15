#!/usr/bin/env python3
"""Publish decoded RC state; never arm, change modes, or send setpoints."""
import math
import sys
import threading
from time import monotonic

import rospy
from ducted_msgs.msg import RCState
from mavros_msgs.msg import RCIn
from std_msgs.msg import Bool

from ducted_control.rc import RCDecoder


class RCMonitor:
    def __init__(self):
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.decoder = RCDecoder(rospy.get_param('~rc'))
        self.mapping_confirmed = rospy.get_param('~mapping_confirmed', False) is True
        self.base_timeout = float(rospy.get_param('~base_timeout', 1.))
        if not math.isfinite(self.base_timeout) or self.base_timeout <= 0:
            raise ValueError('invalid base heartbeat timeout')
        self.base_ready = False
        self.base_wall = None
        self.source_stamp = None
        self.state_pub = rospy.Publisher('state', RCState, queue_size=1)
        self.ready_pub = rospy.Publisher('ready', Bool, queue_size=1, latch=True)
        self.ready_pub.publish(False)
        self.rc_sub = rospy.Subscriber('input', RCIn, self._rc_callback, queue_size=1)
        self.base_sub = rospy.Subscriber('base_ready', Bool, self._base_callback, queue_size=1)

    def _base_fresh(self):
        return (self.base_ready and self.base_wall is not None
                and 0 <= monotonic() - self.base_wall <= self.base_timeout)

    def _base_callback(self, message):
        with self.lock:
            self.base_ready = bool(message.data)
            self.base_wall = monotonic()
            if not self.base_ready:
                self._publish()

    def _rc_callback(self, message):
        with self.lock:
            self.decoder.update(message.channels, message.header.stamp.to_sec(),
                                rospy.get_time(), monotonic(), message.rssi)
            self.source_stamp = message.header.stamp
            self._publish(new_measurement=True)

    def _publish(self, new_measurement=False):
        state = self.decoder.evaluate(rospy.get_time(), monotonic())
        message = RCState()
        message.header.stamp = self.source_stamp if state.valid else rospy.Time.now()
        message.header.frame_id = ''
        for field in ('valid', 'reason', 'roll', 'pitch', 'throttle', 'yaw', 'mode',
                      'kill_switch', 'arm_switch', 'land_switch', 'trigger_switch'):
            setattr(message, field, getattr(state, field))
        if not self._base_fresh():
            message.valid, message.reason = False, 'base system not ready or heartbeat stale'
        elif not self.mapping_confirmed:
            message.valid, message.reason = False, 'RC mapping awaits physical confirmation'
        # Each valid state must correspond to one new RC measurement. Replaying
        # it from the watchdog makes strict downstream timestamp checks reject it.
        # The watchdog still publishes invalidation immediately on loss/expiry.
        if message.valid and not new_measurement:
            return
        self.state_pub.publish(message)
        self.ready_pub.publish(message.valid and not message.kill_switch)

    def _watchdog(self):
        with self.lock:
            self._publish()

    def start_watchdog(self):
        rospy.on_shutdown(self.stop_event.set)
        def run():
            while not self.stop_event.wait(.05):
                self._watchdog()
        threading.Thread(target=run, daemon=True).start()

    def wait_for_base(self, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('invalid base startup timeout')
        deadline = monotonic() + timeout
        while not rospy.is_shutdown() and monotonic() < deadline:
            with self.lock:
                if self._base_fresh():
                    return True
            self.stop_event.wait(.05)
        return False


def main():
    rospy.init_node('rc_monitor')
    node = RCMonitor()
    node.start_watchdog()
    if not node.wait_for_base(float(rospy.get_param('~base_ready_timeout', 15.))):
        rospy.logfatal('Start base_system.launch and wait for its ready heartbeat first')
        node.stop_event.set()
        return 2
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
