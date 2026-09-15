#!/usr/bin/env python3
"""Exercise the production C++ node against a complete fake MAVROS surface."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from xmlrpc.client import ServerProxy

PORT = 11327
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
os.environ['ROS_HOSTNAME'] = '127.0.0.1'

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Header
from ducted_offboard.msg import OffboardStatus


ROOT = Path(__file__).resolve().parents[4]
LOG_ROOT = ROOT / 'logs' / 'offboard_control_isolated'
RESULT = LOG_ROOT / 'result.json'


def launch(command, name):
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    stream = (LOG_ROOT / (name + '.log')).open('w')
    process = subprocess.Popen(command, cwd=str(ROOT), stdout=stream,
                               stderr=subprocess.STDOUT, start_new_session=True)
    process._log_stream = stream
    return process


def stop(process):
    if process is None:
        return
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
    process._log_stream.close()


def wait_for(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            value = predicate()
        except (ConnectionError, OSError):
            value = None
        if value:
            return value
        time.sleep(.02)
    raise RuntimeError('timeout waiting for ' + description)


class FakeMavros:
    def __init__(self):
        self.lock = threading.RLock()
        self.connected = True
        self.armed = False
        self.mode = 'POSCTL'
        self.landed = ExtendedState.LANDED_STATE_ON_GROUND
        self.position = [0., 0., 0.]
        self.yaw = 0.
        self.speed = 0.
        self.target = None
        self.mode_accept = True
        self.mode_delay = .08
        self.arm_accept = True
        self.service_block = 0.
        self.publish_state = True
        self.publish_pose = True
        self.publish_velocity = True
        self.publish_extended = True
        self.pending_mode = None
        self.pending_mode_at = 0.
        self.mode_calls = []
        self.arm_calls = []
        self.setpoint_times = []
        self.state_pub = rospy.Publisher('/mavros/state', State, queue_size=3)
        self.pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped,
                                        queue_size=3)
        self.velocity_pub = rospy.Publisher('/mavros/local_position/velocity_local',
                                            TwistStamped, queue_size=3)
        self.extended_pub = rospy.Publisher('/mavros/extended_state', ExtendedState,
                                           queue_size=3)
        self.setpoint_sub = rospy.Subscriber('/mavros/setpoint_position/local',
                                             PoseStamped, self.setpoint, queue_size=20)
        self.mode_service = rospy.Service('/mavros/set_mode', SetMode, self.set_mode)
        self.arm_service = rospy.Service('/mavros/cmd/arming', CommandBool, self.arm)
        self.timer = rospy.Timer(rospy.Duration(.02), self.publish)

    def reset(self):
        with self.lock:
            self.armed = False
            self.mode = 'POSCTL'
            self.landed = ExtendedState.LANDED_STATE_ON_GROUND
            self.position = [0., 0., 0.]
            self.yaw = 0.
            self.speed = 0.
            self.target = None
            self.pending_mode = None
            self.mode_accept = True
            self.arm_accept = True
            self.service_block = 0.
            self.publish_state = True
            self.publish_pose = True
            self.publish_velocity = True
            self.publish_extended = True
            self.mode_calls.clear()
            self.arm_calls.clear()
            self.setpoint_times.clear()

    def setpoint(self, message):
        with self.lock:
            q = message.pose.orientation
            self.target = (message.pose.position.x, message.pose.position.y,
                           message.pose.position.z,
                           math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
            self.setpoint_times.append(time.monotonic())

    def set_mode(self, request):
        with self.lock:
            self.mode_calls.append((time.monotonic(), request.custom_mode))
            block = self.service_block
            accepted = self.mode_accept
        if block:
            time.sleep(block)
        if accepted:
            with self.lock:
                self.pending_mode = request.custom_mode
                self.pending_mode_at = time.monotonic() + self.mode_delay
        return SetModeResponse(mode_sent=accepted)

    def arm(self, request):
        with self.lock:
            self.arm_calls.append((time.monotonic(), request.value))
            accepted = self.arm_accept
            if accepted:
                self.armed = request.value
                if request.value:
                    self.landed = ExtendedState.LANDED_STATE_TAKEOFF
        return CommandBoolResponse(success=accepted, result=0)

    def advance_offboard(self):
        delta = [self.target[i] - self.position[i] for i in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        step = min(distance, .025)
        if distance > 1e-9:
            self.position = [self.position[i] + delta[i] * step / distance
                             for i in range(3)]
        yaw_delta = (self.target[3] - self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.yaw += max(-.04, min(.04, yaw_delta))
        if self.position[2] > .05:
            self.landed = ExtendedState.LANDED_STATE_IN_AIR

    def publish(self, _event):
        now_wall = time.monotonic()
        with self.lock:
            if self.pending_mode and now_wall >= self.pending_mode_at:
                self.mode = self.pending_mode
                self.pending_mode = None
            old = tuple(self.position)
            if self.armed and self.mode == 'OFFBOARD' and self.target:
                self.advance_offboard()
            elif self.armed and self.mode == 'AUTO.LAND':
                self.position[2] = max(0., self.position[2] - .012)
                if self.position[2] <= .015:
                    self.position[2] = 0.
                    self.landed = ExtendedState.LANDED_STATE_ON_GROUND
                else:
                    self.landed = ExtendedState.LANDED_STATE_LANDING
            self.speed = math.sqrt(sum((self.position[i] - old[i]) ** 2 for i in range(3))) / .02
            stamp = rospy.Time.now()
            self.last_pose_stamp = stamp
            state = State(header=Header(stamp=stamp), connected=self.connected,
                          armed=self.armed, mode=self.mode)
            pose = PoseStamped(header=Header(stamp=stamp, frame_id='odom'))
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
            pose.pose.orientation.z = math.sin(self.yaw / 2.)
            pose.pose.orientation.w = math.cos(self.yaw / 2.)
            velocity = TwistStamped(header=Header(stamp=stamp, frame_id='odom'))
            velocity.twist.linear.x = self.speed
            extended = ExtendedState(header=Header(stamp=stamp),
                                     landed_state=self.landed)
            publish_state = self.publish_state
            publish_pose = self.publish_pose
            publish_velocity = self.publish_velocity
            publish_extended = self.publish_extended
        if publish_state:
            self.state_pub.publish(state)
        if publish_pose:
            self.pose_pub.publish(pose)
        if publish_velocity:
            self.velocity_pub.publish(velocity)
        if publish_extended:
            self.extended_pub.publish(extended)


class StatusSink:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.history = []
        self.subscriber = rospy.Subscriber('/ctrl_cmd/status', OffboardStatus,
                                           self.receive, queue_size=20)

    def receive(self, message):
        with self.lock:
            self.latest = message
            self.history.append((time.monotonic(), message.phase, message.reason))

    def phase(self, name):
        with self.lock:
            return self.latest if self.latest and self.latest.phase == name else None

    def clear(self):
        with self.lock:
            self.latest = None


def node_command(**overrides):
    args = {
        'target_source': 'topic', 'topic_mode': 'sequence',
        'prestream_duration': '.25', 'retry_interval': '.15',
        'feedback_timeout': '.12', 'service_timeout': '.35',
        'takeoff_height': '.35', 'horizontal_speed': '1.0',
        'vertical_speed': '1.0', 'yaw_rate_deg': '180',
        'arrival_dwell': '.15', 'ground_disarm_delay': '.25',
        'takeoff_timeout': '5', 'waypoint_timeout': '5',
        'landing_timeout': '5', 'log_root': str(LOG_ROOT),
        'log_max_bytes': '1024',
    }
    args.update({key: str(value) for key, value in overrides.items()})
    return ['rosrun', 'ducted_offboard', 'offboard_controller_node'] + [
        '_%s:=%s' % item for item in args.items()]


def publish_sequence(publisher, points):
    message = NavPath(header=Header(stamp=rospy.Time.now(),
                                    frame_id='mission_start'))
    for x, y, z, yaw in points:
        pose = PoseStamped(header=message.header)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.z = math.sin(yaw / 2.)
        pose.pose.orientation.w = math.cos(yaw / 2.)
        message.poses.append(pose)
    publisher.publish(message)


def main():
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('test port %d is already occupied' % PORT)
    processes = []
    checks = []
    master = launch(['roscore', '-p', str(PORT)], 'master')
    processes.append(master)
    try:
        wait_for(lambda: ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1,
                 10, 'isolated ROS master')
        rospy.init_node('offboard_integration_test', disable_signals=True)
        fake = FakeMavros()
        sink = StatusSink()
        mission_pub = rospy.Publisher('/ctrl_cmd/waypoints', NavPath, queue_size=1,
                                      latch=True)

        rospy.set_param('/ctrl_cmd/state', 1)
        before_sessions = set(LOG_ROOT.glob('session_*'))
        node = launch(node_command(), 'normal_node')
        processes.append(node)
        wait_for(lambda: sink.phase('BOOT_WAIT_ZERO'), 4, 'legacy command rejection')
        assert len(fake.mode_calls) == 0
        wait_for(lambda: len(fake.setpoint_times) > 0, 2, 'first prestream setpoint')
        with fake.lock:
            initial_stream = len(fake.setpoint_times)
        time.sleep(.35)
        with fake.lock:
            assert len(fake.setpoint_times) - initial_stream >= 5
        checks.append('startup command is inert while hold setpoints stream')

        rospy.set_param('/ctrl_cmd/state', 0)
        wait_for(lambda: sink.phase('IDLE_GROUND'), 3, 'idle after zero')
        publish_sequence(mission_pub, [(0.35, 0., .35, 0.),
                                       (0.60, 0.20, .35, math.pi / 4.)])
        wait_for(lambda: sink.latest and sink.latest.mission_loaded, 3, 'mission accepted before takeoff')
        rospy.set_param('/ctrl_cmd/state', 1)
        wait_for(lambda: sink.phase('TAKEOFF'), 4, 'armed takeoff')
        hold = wait_for(lambda: sink.phase('HOLDING'), 5, 'takeoff hold')
        hold_position = tuple(fake.position)
        time.sleep(.6)
        assert sink.phase('HOLDING') and math.dist(hold_position, tuple(fake.position)) < .08
        assert hold.reason == 'takeoff complete; waiting for state=2'
        checks.append('takeoff holds with loaded mission until state=2')
        rospy.set_param('/ctrl_cmd/state', 2)
        wait_for(lambda: sink.phase('GUIDING'), 3, 'explicit guiding')
        checks.append('state=2 starts loaded mission after takeoff')

        rospy.set_param('/ctrl_cmd/state', 0)
        paused = wait_for(lambda: sink.phase('PAUSED'), 3, 'pause')
        paused_position = tuple(fake.position)
        time.sleep(.25)
        assert math.dist(paused_position, tuple(fake.position)) < .08
        rospy.set_param('/ctrl_cmd/state', 2)
        wait_for(lambda: sink.phase('GUIDING'), 3, 'resume')
        complete = wait_for(lambda: sink.phase('HOLDING'), 6, 'sequence completion')
        assert complete.current_waypoint == 2 and complete.waypoint_count == 2
        checks.append('pause, resume, dwell and sequential waypoint completion')

        rospy.set_param('/ctrl_cmd/state', 3)
        wait_for(lambda: sink.phase('LANDING'), 3, 'AUTO.LAND feedback')
        wait_for(lambda: sink.phase('IDLE_GROUND'), 5, 'ground disarm completion')
        assert fake.mode == 'AUTO.LAND' and not fake.armed
        assert any(value is False for _, value in fake.arm_calls)
        checks.append('AUTO.LAND confirmation, ground feedback and normal disarm')

        rospy.set_param('/ctrl_cmd/state', 0)
        publish_sequence(mission_pub, [(0., 0., .35, 0.)])
        rospy.set_param('/ctrl_cmd/state', 1)
        wait_for(lambda: sink.phase('TAKEOFF'), 4, 'second takeoff')
        rospy.set_param('/ctrl_cmd/state', 3)
        wait_for(lambda: sink.phase('LANDING'), 3, 'second landing')
        wait_for(lambda: sink.phase('IDLE_GROUND'), 5, 'second cycle complete')
        checks.append('second task cycle without node restart')
        stop(node)
        processes.remove(node)

        sessions = set(LOG_ROOT.glob('session_*')) - before_sessions
        assert sessions
        session = max(sessions, key=lambda path: path.stat().st_mtime)
        assert (session / 'events.log').stat().st_size > 0
        assert (session / 'config.yaml').stat().st_size > 0
        telemetry_files = [session / 'telemetry.csv'] + sorted(
            session.glob('telemetry.csv.*'))
        assert len(telemetry_files) > 1
        telemetry_rows = 0
        for telemetry_file in telemetry_files:
            lines = telemetry_file.read_text().splitlines()
            assert lines and lines[0].startswith('utc,ros_time,monotonic,')
            telemetry_rows += len(lines) - 1
        assert telemetry_rows > 3
        checks.append('session event, telemetry and configuration logs')

        fake.reset()
        fake.mode_accept = False
        sink.clear()
        rospy.set_param('/ctrl_cmd/state', 0)
        failed = launch(node_command(), 'mode_failure_node')
        processes.append(failed)
        wait_for(lambda: sink.phase('IDLE_GROUND'), 3, 'failure scenario idle')
        rospy.set_param('/ctrl_cmd/state', 1)
        wait_for(lambda: len(fake.mode_calls) == 4, 4, 'four OFFBOARD attempts')
        wait_for(lambda: failed.poll() is not None, 3, 'node failure exit')
        assert len(fake.mode_calls) == 4 and failed.returncode != 0
        stop(failed)
        processes.remove(failed)
        checks.append('initial OFFBOARD request plus three retries exits nonzero')

        fake.reset()
        fake.service_block = 1.0
        sink.clear()
        rospy.set_param('/ctrl_cmd/state', 0)
        blocked = launch(node_command(), 'blocked_service_node')
        processes.append(blocked)
        wait_for(lambda: sink.phase('IDLE_GROUND'), 3, 'blocked scenario idle')
        with fake.lock:
            start_count = len(fake.setpoint_times)
        rospy.set_param('/ctrl_cmd/state', 1)
        wait_for(lambda: sink.phase('ERROR'), 3, 'bounded service timeout')
        time.sleep(1.1)
        assert sink.phase('ERROR')
        with fake.lock:
            streamed = len(fake.setpoint_times) - start_count
        assert streamed >= 6
        checks.append('blocked service does not block setpoint/status loop')
        stop(blocked)
        processes.remove(blocked)

        fake.reset()
        fake.arm_accept = False
        sink.clear()
        rospy.set_param('/ctrl_cmd/state', 0)
        arm_failed = launch(node_command(), 'arm_failure_node')
        processes.append(arm_failed)
        wait_for(lambda: sink.phase('IDLE_GROUND'), 3, 'arm failure idle')
        rospy.set_param('/ctrl_cmd/state', 1)
        arm_error = wait_for(lambda: sink.phase('ERROR'), 4, 'four arm failures')
        with fake.lock:
            arm_attempts = [value for _, value in fake.arm_calls if value]
        assert len(arm_attempts) == 4 and 'ARM failed' in arm_error.reason
        checks.append('four rejected arming requests stop progression')
        stop(arm_failed)
        processes.remove(arm_failed)

        fake.reset()
        sink.clear()
        rospy.set_param('/ctrl_cmd/state', 0)
        stale_pose = launch(node_command(), 'pose_dropout_node')
        processes.append(stale_pose)
        wait_for(lambda: sink.phase('IDLE_GROUND'), 3, 'pose dropout idle')
        rospy.set_param('/ctrl_cmd/state', 1)
        wait_for(lambda: sink.phase('TAKEOFF'), 4, 'pose dropout takeoff')
        with fake.lock:
            fake.publish_pose = False
        pose_error = wait_for(lambda: sink.phase('ERROR'), 3, 'pose dropout error')
        with fake.lock:
            stopped_count = len(fake.setpoint_times)
        time.sleep(.25)
        with fake.lock:
            assert len(fake.setpoint_times) == stopped_count
        assert 'pose unavailable' in pose_error.reason
        checks.append('pose dropout stops stale target output')
        stop(stale_pose)
        processes.remove(stale_pose)

        fake.reset()
        sink.clear()
        rospy.set_param('/ctrl_cmd/state', 0)
        mode_exit = launch(node_command(), 'manual_mode_exit_node')
        processes.append(mode_exit)
        wait_for(lambda: sink.phase('IDLE_GROUND'), 3, 'mode exit idle')
        rospy.set_param('/ctrl_cmd/state', 1)
        wait_for(lambda: sink.phase('TAKEOFF'), 4, 'mode exit takeoff')
        with fake.lock:
            fake.mode = 'POSCTL'
            fake.pending_mode = None
        mode_error = wait_for(lambda: sink.phase('ERROR'), 3, 'manual mode exit')
        with fake.lock:
            stopped_count = len(fake.setpoint_times)
        time.sleep(.25)
        with fake.lock:
            assert len(fake.setpoint_times) == stopped_count
        assert 'left OFFBOARD' in mode_error.reason
        checks.append('manual mode exit is never automatically reclaimed')

        report = {'passed': True, 'checks': checks, 'check_count': len(checks),
                  'scope': 'fake MAVROS; no hardware commands or flight'}
        RESULT.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report))
        return 0
    finally:
        for process in reversed(processes):
            stop(process)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        RESULT.write_text(json.dumps({'passed': False, 'error': str(error)},
                                     indent=2) + '\n')
        raise
