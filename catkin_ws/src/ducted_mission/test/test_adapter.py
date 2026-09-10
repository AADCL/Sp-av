import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock, patch


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))


class Stamp:
    def __init__(self, value=0.0):
        self.value = value

    def to_sec(self):
        return self.value


def blank_pose():
    return NS(
        position=NS(x=0.0, y=0.0, z=0.0),
        orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


def pose_stamped():
    return NS(header=NS(stamp=Stamp(), frame_id=""), pose=blank_pose())


def mission_status():
    return NS(header=NS(), mission_id="", session_id="", epoch=0, waypoint_index=0,
              waypoint_count=0, retry=0, request_id="", state="", reason="")


class FakeThread:
    instances = []

    def __init__(self, target=None, daemon=None):
        self.target = target
        self.daemon = daemon
        self.started = False
        FakeThread.instances.append(self)

    def start(self):
        self.started = True

    def join(self, timeout=None):
        return None


class FakePersistentDeadlineProcess:
    def __init__(self, *_args, **_kwargs):
        self.process = None
        self.ready_calls = 0

    def ensure_ready(self, _timeout):
        self.ready_calls += 1
        return True, "ready"

    def call(self, _arguments, _timeout):
        return True, "ok"

    def close(self):
        return None


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.ros = MagicMock()
        self.ros.Time.now.return_value = Stamp(100.0)
        self.ros.Time.from_sec.side_effect = Stamp
        self.ros.Time.side_effect = Stamp
        self.ros.Duration.side_effect = lambda value: value
        self.ros.resolve_name.side_effect = lambda name: "/ducted/mission/" + name
        self.params = {
            "~mission_file": str(PACKAGE / "config" / "mission.yaml"),
            "~enable_commands": True,
            "~worker_queue_capacity": 4,
            "~service_timeout": 0.2,
        }
        self.ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        self.publishers = {}

        def publisher(name, *_args, **_kwargs):
            result = MagicMock()
            self.publishers[name] = result
            return result

        self.ros.Publisher.side_effect = publisher
        self.buffer = MagicMock()
        self.buffer.lookup_transform.return_value = NS()

        def transformed(message, _transform):
            output = pose_stamped()
            output.header = message.header
            output.pose = message.pose
            output.pose.position.z += 1.0
            return output

        modules = {
            "rospy": self.ros,
            "tf2_ros": NS(Buffer=lambda: self.buffer,
                           TransformListener=lambda _buffer: NS()),
            "tf2_geometry_msgs": NS(do_transform_pose=transformed),
            "ducted_mission.msg": NS(MissionStatus=mission_status),
            "ducted_msgs.msg": NS(FlightControlStatus=object, RCState=object,
                                    TerrainHeight=object),
            "ducted_navigation.msg": NS(NavigationStatus=object),
            "geometry_msgs.msg": NS(PoseStamped=pose_stamped),
            "nav_msgs.msg": NS(Odometry=object),
            "std_msgs.msg": NS(Bool=object),
            "std_srvs.srv": NS(
                Trigger=object,
                TriggerResponse=lambda success=False, message="": NS(
                    success=success, message=message)),
        }
        FakeThread.instances = []
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                "waypoint_mission_tested", PACKAGE / "scripts" / "waypoint_mission.py")
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.thread_patch = patch.object(self.module.threading, "Thread", FakeThread)
        self.thread_patch.start()
        self.addCleanup(self.thread_patch.stop)
        self.clock_patch = patch.object(self.module, "monotonic", return_value=10.0)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.module.PersistentDeadlineProcess = FakePersistentDeadlineProcess
        self.node = self.module.WaypointMissionNode()
        self.node.worker = MagicMock()
        self.node.worker.submit.return_value = True

    @staticmethod
    def header(value=100.0, frame="odom"):
        return NS(stamp=Stamp(value), frame_id=frame)

    def feed_ready(self, value=100.0):
        self.ros.Time.now.return_value = Stamp(value)
        self.node._base_callback(NS(data=True))
        self.node._terrain_ready_callback(NS(data=True))
        self.node._rc_callback(NS(
            header=self.header(value), valid=True, mode="command",
            kill_switch=False, roll=0.0, pitch=0.0, throttle=0.5, yaw=0.0))
        self.node._odom_callback(NS(
            header=self.header(value),
            pose=NS(pose=blank_pose(), covariance=[0.0] * 36),
            twist=NS(twist=NS(linear=NS(x=0.0, y=0.0, z=0.0),
                              angular=NS(x=0.0, y=0.0, z=0.0)),
                     covariance=[0.0] * 36)))
        self.node._terrain_callback(NS(
            header=self.header(value), valid=True,
            ground_z=0.0, agl=1.0, variance=0.001))
        self.node._planner_callback(NS(
            header=self.header(value), request_id="", state="IDLE"))
        self.node._controller_callback(NS(
            header=self.header(value), request_id="old", state="HOLD", ready=True))

    def test_startup_is_idle_and_performs_no_service_call(self):
        self.assertEqual(self.node.core.state, "IDLE")
        self.assertEqual(self.node.deadline_process.process, None)
        self.assertGreaterEqual(self.node.deadline_process.ready_calls, 1)
        self.ros.ServiceProxy.assert_not_called()
        self.assertTrue(any(thread.target == self.node._watchdog_loop
                            for thread in FakeThread.instances))

    def test_tick_samples_time_after_concurrent_telemetry_lock(self):
        self.feed_ready()
        self.assertTrue(self.node._start(None).success)
        lock = MagicMock()

        def enter():
            self.module.monotonic.return_value = 10.01
            self.ros.Time.now.return_value = Stamp(100.01)
            self.node.odom_stream.arrival = 10.01
        lock.__enter__.side_effect = enter
        self.node.lock = lock
        self.node._tick(None)
        self.assertEqual(self.node.core.state, 'RUNNING')

    def test_start_samples_time_after_concurrent_telemetry_lock(self):
        self.feed_ready()
        lock = MagicMock()

        def enter():
            self.module.monotonic.return_value = 10.01
            self.ros.Time.now.return_value = Stamp(100.01)
            self.node.odom_stream.arrival = 10.01
        lock.__enter__.side_effect = enter
        self.node.lock = lock
        self.assertTrue(self.node._start(None).success)

    def test_duplicate_odometry_revokes_cached_sample(self):
        self.feed_ready()
        self.assertIsNotNone(self.node.odom)
        duplicate = NS(
            header=self.header(100.0),
            pose=NS(pose=blank_pose(), covariance=[0.0] * 36),
            twist=NS(twist=NS(linear=NS(x=0.0, y=0.0, z=0.0),
                              angular=NS(x=0.0, y=0.0, z=0.0)),
                     covariance=[0.0] * 36))
        self.node._odom_callback(duplicate)
        self.assertIsNone(self.node.odom)
        self.assertFalse(self.node.odom_stream.valid)

    def test_nonfinite_angular_twist_and_covariance_revoke_odometry(self):
        self.feed_ready()
        invalid = NS(
            header=self.header(100.1),
            pose=NS(pose=blank_pose(), covariance=[0.0] * 35 + [math.nan]),
            twist=NS(twist=NS(linear=NS(x=0.0, y=0.0, z=0.0),
                              angular=NS(x=0.0, y=math.inf, z=0.0)),
                     covariance=[0.0] * 36))
        self.ros.Time.now.return_value = Stamp(100.1)
        self.node._odom_callback(invalid)
        self.assertIsNone(self.node.odom)
        self.assertFalse(self.node.odom_stream.valid)

    def test_explicit_start_transforms_full_pose_and_pause_invalidates_goal(self):
        self.feed_ready()
        response = self.node._start(None)
        self.assertTrue(response.success)
        goal = self.node.worker.submit.call_args_list[-1].args[0]
        self.assertEqual(goal.kind, "goal")
        self.assertAlmostEqual(goal.target.pose.position.z, 2.0)
        lookup = self.buffer.lookup_transform.call_args
        self.assertEqual(lookup.args[0:2], ("odom", "map"))
        self.assertIs(lookup.args[2], self.node.odom_ros_stamp)
        self.assertEqual(lookup.args[3], 0.0)
        pause = self.node._pause(None)
        self.assertTrue(pause.success)
        stop = self.node.worker.submit.call_args_list[-1].args[0]
        self.assertEqual(stop.kind, "stop")
        self.assertFalse(self.node._work_current(goal))
        self.assertTrue(self.node._work_current(stop))

    def test_gate_rejection_never_falls_through_to_goal_submission(self):
        self.feed_ready()
        with self.node.lock:
            result = self.node.core.start(10.0, self.node._snapshot(100.0, 10.0)[0])
        action = result.actions[0]
        self.node.worker.reset_mock()
        self.node.core._gate_reason = MagicMock(return_value="authority lost")
        self.node.core.pause = MagicMock(return_value=NS(actions=()))
        self.node._dispatch_actions((action,))
        self.node.worker.submit.assert_not_called()


if __name__ == "__main__":
    unittest.main()
