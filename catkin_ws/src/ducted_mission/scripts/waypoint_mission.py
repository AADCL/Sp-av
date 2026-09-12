#!/usr/bin/env python3
"""ROS boundary for explicitly activated waypoint missions."""

import math
import sys
import threading
from time import monotonic

import rospy
import tf2_geometry_msgs
import tf2_ros
import yaml
from ducted_mission.mission import (
    ControllerObservation,
    ExecutionTarget,
    GateSnapshot,
    MissionConfig,
    MissionCore,
    OdomObservation,
    PlannerObservation,
    StampedInput,
)
from ducted_mission.msg import MissionStatus
from ducted_mission.runtime import (
    GoalTransformer,
    PersistentDeadlineProcess,
    ServiceReply,
    ServiceWorker,
    WorkItem,
)
from ducted_mission.service_call import ros_service_bootstrap
from ducted_msgs.msg import FlightControlStatus, RCState, TerrainHeight
from ducted_navigation.msg import NavigationStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse


class WaypointMissionNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.require_automatic_lease=rospy.get_param("~require_automatic_lease",False) is True
        self.height_mode=rospy.get_param('~height_mode','terrain')
        if self.height_mode not in ('terrain','takeoff_relative'):
            raise ValueError('unknown mission height mode')
        if self.height_mode=='takeoff_relative' and (not self.require_automatic_lease
                or rospy.get_param('/ducted_navigation/height_mode','')!='takeoff_relative'):
            raise ValueError('relative mission requires matching navigation and automatic lease')
        self.automatic_lease=(False,-math.inf)
        rospy.Subscriber("/ducted/automatic/mission_lease",Bool,self._automatic_lease_callback,queue_size=1)
        self.odom_frame = str(rospy.get_param("~odom_frame", "odom"))
        mission_file = str(rospy.get_param("~mission_file"))
        with open(mission_file, "r") as stream:
            config = MissionConfig.from_dict(yaml.safe_load(stream))
        self.core = MissionCore(
            config, rospy.get_param("~enable_commands", False) is True)

        timeout = config.telemetry_timeout
        future = float(rospy.get_param("~future_tolerance", 0.05))
        self.rc_stream = StampedInput("RC", timeout, timeout, future)
        self.odom_stream = StampedInput("odometry", timeout, timeout, future)
        self.terrain_stream = StampedInput("terrain", timeout, timeout, future)
        self.planner_stream = StampedInput("planner", timeout, timeout, future)
        self.controller_stream = StampedInput("controller", timeout, timeout, future)
        self.base_ready = False
        self.base_arrival = None
        self.terrain_ready = False
        self.terrain_ready_arrival = None
        self.rc = None
        self.odom = None
        self.odom_ros_stamp = None
        self.terrain = None
        self.planner = None
        self.controller = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transformer = GoalTransformer(
            PoseStamped, self.tf_buffer.lookup_transform,
            tf2_geometry_msgs.do_transform_pose,
            lambda: rospy.Duration(0.0))
        self.service_timeout = float(rospy.get_param("~service_timeout", 2.0))
        if not math.isfinite(self.service_timeout) or self.service_timeout <= 0.0:
            raise ValueError("service_timeout must be positive and finite")
        self.service_warm_timeout = float(
            rospy.get_param("~service_warm_timeout", 5.0))
        if (not math.isfinite(self.service_warm_timeout)
                or self.service_warm_timeout <= 0.0):
            raise ValueError("service_warm_timeout must be positive and finite")
        self.navigation_service = rospy.resolve_name("navigation_command")
        self.control_service = rospy.resolve_name("control_command")
        self.deadline_process = PersistentDeadlineProcess(
            ros_service_bootstrap,
            (self.navigation_service, self.control_service,
             self.service_warm_timeout))
        self.helper_ready, self.helper_reason = (
            self.deadline_process.ensure_ready(self.service_warm_timeout))
        self.dispatch_rejections = {}
        self.worker = ServiceWorker(
            self._navigation_call, self._control_call, self._worker_result,
            capacity=int(rospy.get_param("~worker_queue_capacity", 4)),
            current_callback=self._work_current,
            ready_callback=self._work_ready)

        self.status_pub = rospy.Publisher(
            "status", MissionStatus, queue_size=1, latch=True)
        rospy.Subscriber("base_ready", Bool, self._base_callback, queue_size=1)
        rospy.Subscriber("rc_state", RCState, self._rc_callback, queue_size=1)
        rospy.Subscriber("odom", Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber("terrain", TerrainHeight, self._terrain_callback, queue_size=1)
        rospy.Subscriber(
            "terrain_ready", Bool, self._terrain_ready_callback, queue_size=1)
        rospy.Subscriber(
            "navigation_status", NavigationStatus,
            self._planner_callback, queue_size=1)
        rospy.Subscriber(
            "control_status", FlightControlStatus,
            self._controller_callback, queue_size=1)
        rospy.Service("start", Trigger, self._start)
        rospy.Service("pause", Trigger, self._pause)
        rospy.Service("resume", Trigger, self._resume)
        rospy.Service("cancel", Trigger, self._cancel)
        rospy.on_shutdown(self.shutdown)
        self.worker.start()
        self.watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        self._publish_status()

    def _automatic_lease_callback(self,message):
        with self.lock:self.automatic_lease=(bool(message.data),monotonic())

    @staticmethod
    def _stamp(message):
        try:
            return float(message.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            return math.nan

    @staticmethod
    def _finite(*values):
        try:
            return all(math.isfinite(float(value)) for value in values)
        except (TypeError, ValueError):
            return False

    def _clock(self):
        return float(rospy.Time.now().to_sec()), monotonic()

    def _accept(self, stream, message, payload_valid, wall, ros_now):
        return stream.accept(self._stamp(message), wall, payload_valid, ros_now)

    def _base_callback(self, message):
        with self.lock:
            self.base_ready = bool(message.data)
            self.base_arrival = monotonic()

    def _terrain_ready_callback(self, message):
        with self.lock:
            self.terrain_ready = bool(message.data)
            self.terrain_ready_arrival = monotonic()

    def _rc_callback(self, message):
        ros_now, wall = self._clock()
        payload_valid = (
            bool(message.valid)
            and str(message.mode) in ("manual", "hold", "command")
            and self._finite(message.roll, message.pitch, message.throttle,
                             message.yaw))
        with self.lock:
            if self._accept(self.rc_stream, message, payload_valid, wall, ros_now):
                self.rc = (bool(message.valid), str(message.mode),
                           bool(message.kill_switch))
            else:
                self.rc = None

    @staticmethod
    def _pose(message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        pose_covariance = tuple(message.pose.covariance)
        twist_covariance = tuple(message.twist.covariance)
        if len(pose_covariance) != 36 or len(twist_covariance) != 36:
            raise ValueError("odometry covariance must contain 36 values")
        values = (
            float(position.x), float(position.y), float(position.z),
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w),
            float(linear.x), float(linear.y), float(linear.z),
            float(angular.x), float(angular.y), float(angular.z))
        if not all(math.isfinite(value) for value in
                   values + pose_covariance + twist_covariance):
            raise ValueError("odometry contains non-finite values")
        norm = math.sqrt(sum(value * value for value in values[3:7]))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("odometry quaternion is invalid")
        yaw = math.atan2(
            2.0 * (values[6] * values[5] + values[3] * values[4]),
            1.0 - 2.0 * (values[4] * values[4] + values[5] * values[5]))
        speed = math.sqrt(sum(value * value for value in values[7:10]))
        return OdomObservation(values[0], values[1], values[2], yaw, speed)

    def _odom_callback(self, message):
        ros_now, wall = self._clock()
        try:
            observation = self._pose(message)
            valid = str(message.header.frame_id).lstrip("/") == self.odom_frame.lstrip("/")
        except (AttributeError, TypeError, ValueError):
            observation, valid = None, False
        with self.lock:
            if self._accept(self.odom_stream, message, valid, wall, ros_now):
                self.odom = observation
                self.odom_ros_stamp = message.header.stamp
            else:
                self.odom = None
                self.odom_ros_stamp = None

    def _terrain_callback(self, message):
        ros_now, wall = self._clock()
        valid = (bool(message.valid)
                 and str(message.header.frame_id).lstrip("/") == self.odom_frame.lstrip("/")
                 and self._finite(message.ground_z, message.agl, message.variance)
                 and float(message.variance) >= 0.0)
        with self.lock:
            if self._accept(self.terrain_stream, message, valid, wall, ros_now):
                self.terrain = True
            else:
                self.terrain = None

    def _planner_callback(self, message):
        ros_now, wall = self._clock()
        state = str(message.state)
        valid = state in (
            "CLEAR", "AVOIDING", "BLOCKED", "STALE_INPUT",
            "GOAL_REACHED", "IDLE")
        with self.lock:
            if self._accept(self.planner_stream, message, valid, wall, ros_now):
                self.planner = PlannerObservation(str(message.request_id), state)
            else:
                self.planner = None

    def _controller_callback(self, message):
        ros_now, wall = self._clock()
        state = str(message.state)
        valid = bool(state) and isinstance(message.request_id, str)
        with self.lock:
            if self._accept(self.controller_stream, message, valid, wall, ros_now):
                self.controller = ControllerObservation(
                    bool(message.ready), state, str(message.request_id))
            else:
                self.controller = None

    @staticmethod
    def _age(stream, wall):
        return math.inf if stream.arrival is None else wall - stream.arrival

    def _snapshot(self, ros_now, wall):
        rc_fresh = self.rc_stream.fresh(wall, ros_now)
        odom_fresh = self.odom_stream.fresh(wall, ros_now)
        terrain_fresh = self.terrain_stream.fresh(wall, ros_now)
        planner_fresh = self.planner_stream.fresh(wall, ros_now)
        controller_fresh = self.controller_stream.fresh(wall, ros_now)
        base_age = math.inf if self.base_arrival is None else wall - self.base_arrival
        terrain_ready_age = (math.inf if self.terrain_ready_arrival is None
                             else wall - self.terrain_ready_arrival)
        rc = self.rc if rc_fresh and self.rc is not None else (False, "", True)
        controller = (self.controller if controller_fresh and self.controller is not None
                      else ControllerObservation(False, "", ""))
        gates = GateSnapshot(
            height_mode=getattr(self,'height_mode','terrain'),
            now=wall,
            base_ready=(self.base_ready and (not getattr(self,'require_automatic_lease',False)
                        or (self.automatic_lease[0] and 0<=wall-self.automatic_lease[1]<=.5))),
            base_age=base_age,
            rc_valid=bool(rc[0]),
            rc_command=rc[1] == "command",
            kill=bool(rc[2]),
            rc_age=self._age(self.rc_stream, wall),
            odom_age=self._age(self.odom_stream, wall) if odom_fresh else math.inf,
            terrain_valid=terrain_fresh and self.terrain is not None,
            terrain_ready=(self.terrain_ready
                           and terrain_ready_age <= self.core.config.telemetry_timeout),
            terrain_age=self._age(self.terrain_stream, wall),
            planner_age=self._age(self.planner_stream, wall) if planner_fresh else math.inf,
            controller_ready=controller.ready,
            controller_state=controller.state,
            controller_age=(self._age(self.controller_stream, wall)
                            if controller_fresh else math.inf))
        odom = (self.odom if odom_fresh and self.odom is not None
                else OdomObservation(math.nan, math.nan, math.nan, math.nan, math.inf))
        planner = (self.planner if planner_fresh and self.planner is not None
                   else PlannerObservation("", "STALE_INPUT"))
        return gates, odom, planner, controller

    def _service_result(self, result):
        self._dispatch_actions(result.actions)
        self._publish_status()
        return TriggerResponse(success=result.accepted, message=result.message)

    def _start(self, _request):
        ready, message = self.deadline_process.ensure_ready(
            self.service_warm_timeout)
        if not ready:
            return TriggerResponse(success=False, message=message)
        with self.lock:
            ros_now, wall = self._clock()
            gates, _, _, _ = self._snapshot(ros_now, wall)
            result = self.core.start(wall, gates)
        return self._service_result(result)

    def _pause(self, _request):
        with self.lock:
            result = self.core.pause(monotonic())
        return self._service_result(result)

    def _resume(self, _request):
        ready, message = self.deadline_process.ensure_ready(
            self.service_warm_timeout)
        if not ready:
            return TriggerResponse(success=False, message=message)
        with self.lock:
            ros_now, wall = self._clock()
            gates, _, _, _ = self._snapshot(ros_now, wall)
            result = self.core.resume(wall, gates)
        return self._service_result(result)

    def _cancel(self, _request):
        with self.lock:
            result = self.core.cancel(monotonic())
        return self._service_result(result)

    def _tick(self, _event):
        with self.lock:
            ros_now, wall = self._clock()
            gates, odom, planner, controller = self._snapshot(ros_now, wall)
            actions = self.core.update(wall, gates, odom, planner, controller)
        self._dispatch_actions(actions)
        self._publish_status()

    def _watchdog_loop(self):
        while not self.stop_event.wait(0.05):
            self._tick(None)

    def shutdown(self):
        self.stop_event.set()
        self.deadline_process.close()
        self.worker.shutdown()

    def _dispatch_actions(self, actions):
        for action in actions:
            if action.kind == "stop":
                if not self.worker.submit(WorkItem(
                        "stop", action.generation, action.request_id, PoseStamped())):
                    self._worker_result("stop", action.generation, False,
                                        "service queue is full")
                continue
            with self.lock:
                if action.generation != self.core.generation:
                    continue
                if self.odom_ros_stamp is None:
                    odom_stamp = None
                else:
                    odom_stamp = self.odom_ros_stamp
            if odom_stamp is None:
                self._worker_result(
                    "goal", action.generation, False,
                    "accepted odometry disappeared before waypoint transform")
                continue
            try:
                target = self.transformer.transform(
                    action, odom_stamp, self.odom_frame)
                x, y, z, yaw = self.transformer.values(target)
            except Exception as error:
                self._worker_result(
                    "goal", action.generation, False,
                    "exact-time waypoint transform failed: {}".format(error))
                continue
            approved = False
            with self.lock:
                ros_now, wall = self._clock()
                gates, _, _, _ = self._snapshot(ros_now, wall)
                gate_reason = self.core._gate_reason(gates)
                if action.generation != self.core.generation:
                    continue
                if gate_reason:
                    followup = self.core.pause(wall, gate_reason).actions
                elif not self.core.set_execution_target(
                        action.generation, ExecutionTarget(x, y, z, yaw)):
                    followup = self.core.ack_goal(action.generation, False)
                else:
                    followup = ()
                    approved = True
            if followup:
                self._dispatch_actions(followup)
                continue
            if not approved:
                continue
            if not self.worker.submit(WorkItem(
                    "goal", action.generation, action.request_id, target)):
                self._worker_result("goal", action.generation, False,
                                    "service queue is full")

    @staticmethod
    def _target_values(target):
        try:
            stamp = float(target.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            stamp = 0.0
        pose = target.pose
        return (
            stamp, str(target.header.frame_id),
            float(pose.position.x), float(pose.position.y), float(pose.position.z),
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w))

    def _navigation_call(self, command, request_id, target):
        accepted, message = self.deadline_process.call(
            ("navigation", command, request_id, self._target_values(target)),
            self.service_timeout)
        return ServiceReply(accepted, message)

    def _control_call(self, command, target):
        accepted, message = self.deadline_process.call(
            ("control", command, "", self._target_values(target)),
            self.service_timeout)
        return ServiceReply(accepted, message)

    def _work_ready(self, _item):
        ready, message = self.deadline_process.ensure_ready(
            self.service_warm_timeout)
        with self.lock:
            self.helper_ready = ready
            self.helper_reason = message
        return ready

    def _work_current(self, item):
        with self.lock:
            ros_now, wall = self._clock()
            if item.kind == "stop":
                return (item.generation == self.core.stop_generation
                        and not self.core.stop_complete)
            if (item.generation != self.core.generation
                    or self.core.state not in self.core.ACTIVE_STATES
                    or self.core.execution_target is None):
                return False
            gates, _, _, _ = self._snapshot(ros_now, wall)
            reason = self.core._gate_reason(gates)
            if reason:
                self.dispatch_rejections[item.generation] = reason
                return False
            return True

    def _worker_result(self, kind, generation, accepted, message):
        with self.lock:
            was_current = generation == self.core.generation
            dispatch_reason = ""
            if kind == "goal":
                dispatch_reason = self.dispatch_rejections.pop(generation, "")
                if (dispatch_reason and generation == self.core.generation
                        and self.core.state in self.core.ACTIVE_STATES):
                    actions = self.core.pause(monotonic(), dispatch_reason).actions
                else:
                    actions = self.core.ack_goal(generation, accepted)
            else:
                was_current = generation == self.core.stop_generation
                self.core.ack_stop(generation, accepted)
                actions = ()
            if not accepted and was_current and message and not dispatch_reason:
                self.core.reason = message
                rospy.logerr("Mission service operation failed: %s", message)
        self._dispatch_actions(actions)
        self._publish_status()

    def _status_message(self):
        message = MissionStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.odom_frame
        message.mission_id = self.core.config.mission_id
        message.session_id = self.core.session_id
        message.epoch = self.core.epoch
        message.waypoint_index = self.core.waypoint_index
        message.waypoint_count = len(self.core.config.waypoints)
        message.retry = self.core.retry
        message.request_id = self.core.request_id
        message.state = self.core.state
        message.reason = self.core.reason
        return message

    def _publish_status(self):
        with self.lock:
            message = self._status_message()
        self.status_pub.publish(message)


def main():
    rospy.init_node("waypoint_mission")
    try:
        WaypointMissionNode()
        rospy.spin()
        return 0
    except (IOError, OSError, TypeError, ValueError, rospy.ROSInitException) as error:
        rospy.logfatal("waypoint mission initialization failed: %s", error)
        return 2


if __name__ == "__main__":
    sys.exit(main())
