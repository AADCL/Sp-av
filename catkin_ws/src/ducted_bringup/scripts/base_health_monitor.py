#!/usr/bin/env python3
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import State
from std_msgs.msg import Bool, Header

from ducted_bringup.health import BaseHealth


class BaseHealthMonitor:
    def __init__(self):
        config_namespace = rospy.get_param("~config_namespace", "/ducted").rstrip("/")

        def config(path, default=None):
            return rospy.get_param(f"{config_namespace}/{path}", default)

        self._startup_time = time.monotonic()
        self._startup_grace_sec = float(config("health/startup_grace_sec", 15.0))
        self._last_ready = None
        self._health = BaseHealth(
            require_mavros=rospy.get_param("~require_mavros", True),
            require_livox=rospy.get_param("~require_livox", True),
            mavros_timeout_sec=float(config("health/mavros_timeout_sec", 3.0)),
            lidar_timeout_sec=float(config("health/lidar_timeout_sec", 2.0)),
        )

        ready_topic = config("topics/system_ready", "/ducted/system/ready")
        diagnostics_topic = config("topics/diagnostics", "/diagnostics")
        mavros_topic = config("topics/mavros_state", "/mavros/state")
        lidar_topic = config("topics/lidar_points", "/livox/lidar")

        self._ready_publisher = rospy.Publisher(ready_topic, Bool, queue_size=1, latch=True)
        self._diagnostics_publisher = rospy.Publisher(
            diagnostics_topic, DiagnosticArray, queue_size=2
        )
        self._mavros_subscriber = rospy.Subscriber(
            mavros_topic, State, self._mavros_callback, queue_size=5
        )
        self._livox_subscriber = rospy.Subscriber(
            lidar_topic, rospy.AnyMsg, self._livox_callback, queue_size=2
        )

        publish_rate_hz = float(config("health/publish_rate_hz", 2.0))
        if publish_rate_hz <= 0.0:
            raise ValueError("health publish rate must be positive")
        self._publish()
        self._timer = rospy.Timer(rospy.Duration(1.0 / publish_rate_hz), self._publish)

    def _mavros_callback(self, message):
        self._health.update_mavros(message.connected)

    def _livox_callback(self, _message):
        self._health.update_livox()

    def _publish(self, _event=None):
        result = self._health.evaluate()
        self._ready_publisher.publish(Bool(data=result.ready))

        if result.ready:
            level = DiagnosticStatus.OK
            summary = "base system ready"
        elif time.monotonic() - self._startup_time <= self._startup_grace_sec:
            level = DiagnosticStatus.WARN
            summary = "; ".join(result.reasons)
        else:
            level = DiagnosticStatus.ERROR
            summary = "; ".join(result.reasons)

        status = DiagnosticStatus(
            level=level,
            name="ducted/base_system",
            message=summary,
            hardware_id="ducted-quadrotor",
            values=[
                KeyValue(key="ready", value=str(result.ready).lower()),
                KeyValue(key="require_mavros", value=str(self._health.require_mavros).lower()),
                KeyValue(key="require_livox", value=str(self._health.require_livox).lower()),
            ],
        )
        self._diagnostics_publisher.publish(
            DiagnosticArray(header=Header(stamp=rospy.Time.now()), status=[status])
        )

        if result.ready != self._last_ready:
            if result.ready:
                rospy.loginfo("Base system is ready")
            else:
                rospy.logwarn("Base system is not ready: %s", summary)
            self._last_ready = result.ready


def main():
    rospy.init_node("base_health_monitor")
    BaseHealthMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
