"""Exercise node commit/watchdog paths with small ROS substitutes."""
import importlib.util
import math
import pathlib
import sys
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch


PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "src"))

from ducted_navigation.planner import PlanResult, Pose, Terrain, Vec3  # noqa: E402
from ducted_navigation.runtime import NavigationRuntime, RuntimeConfig  # noqa: E402


class TFError(Exception):
    pass


class StopAfterOne:
    def __init__(self):
        self.calls = 0

    def wait(self, _period):
        self.calls += 1
        return self.calls > 1


def populated_runtime():
    state = NavigationRuntime(RuntimeConfig())
    state.set_goal("r", Pose(2, 0, 1, 0), 10)
    state.accept_odom(10, 20, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0))
    state.accept_cloud(10, 20, "odom", ())
    state.accept_terrain(10, 20, "odom", Terrain(0, 1, 0.001, True))
    state.accept_controller(10, 20, True)
    for name in state.READINESS_NAMES:
        state.accept_readiness(name, True, 10, 20)
    return state


class NodeRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy = NS(ROSInitException=RuntimeError)
        tf2 = NS(Buffer=object, TransformListener=object, LookupException=TFError,
                 ConnectivityException=TFError, ExtrapolationException=TFError)
        modules = {
            "rospy": rospy,
            "tf2_ros": tf2,
            "ducted_msgs.msg": NS(FlightControlStatus=object, FlightSetpoint=object,
                                   TerrainHeight=object),
            "ducted_navigation.msg": NS(NavigationStatus=object),
            "ducted_navigation.srv": NS(Navigate=object, NavigateResponse=object),
            "nav_msgs.msg": NS(Odometry=object, Path=object),
            "geometry_msgs.msg": NS(PoseStamped=object),
            "sensor_msgs": NS(point_cloud2=NS()),
            "sensor_msgs.point_cloud2": NS(),
            "sensor_msgs.msg": NS(PointCloud2=object),
            "std_msgs.msg": NS(Bool=object),
        }
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                "navigation_node_under_test", PKG / "scripts/ducted_navigation_node.py")
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)

    def node(self):
        node = self.module.NavigationNode.__new__(self.module.NavigationNode)
        node.runtime = populated_runtime()
        node._planner_lock = threading.Lock()
        node._status_lock = threading.Lock()
        node._plan_event = threading.Event()
        node._watchdog_stop = threading.Event()
        node._last_planned_stamp = 0.0
        node.enable_output = True
        node.odom_frame = "odom"
        node.target_pub = MagicMock()
        node.status_pub = MagicMock()
        node.preview_pub = MagicMock()
        node._publish_status = MagicMock()
        node._publish_preview = MagicMock()
        return node

    def test_slow_planning_refreshes_wall_clock_before_commit(self):
        node = self.node()
        clock = [[None, 10.1, 20.8]]
        node._now = lambda: clock[0]
        node.planner = MagicMock()
        node.planner.plan.return_value = PlanResult(
            "CLEAR", "safe", Pose(0.1, 0, 1, 0), 0, 1, 0, 1, 0.1, True)

        node._plan_once(10.1, 20.1)

        node.target_pub.publish.assert_not_called()
        node._publish_preview.assert_not_called()

    def test_callback_generation_change_during_planning_revokes_output(self):
        node = self.node()
        node._now = lambda: (None, 10.1, 20.1)

        def mutate(_snapshot):
            node.runtime.accept_terrain(10.1, 20.1, "odom", Terrain(0, 1, 0.001, False))
            return PlanResult("CLEAR", "safe", Pose(0.1, 0, 1, 0), 0, 1, 0,
                              1, 0.1, True)

        node.planner = MagicMock()
        node.planner.plan.side_effect = mutate
        node._plan_once(10.1, 20.1)
        node.target_pub.publish.assert_not_called()

    def test_monotonic_watchdog_revokes_with_frozen_ros_time(self):
        node = self.node()
        node._watchdog_stop = StopAfterOne()
        node._now = lambda: (None, 10.1, 20.8)
        node._try_reset_planner = MagicMock()

        node._watchdog_loop(0.1)

        state = node._publish_status.call_args.args[0]
        self.assertEqual(state, "STALE_INPUT")
        node._try_reset_planner.assert_called_once()

    def output_messages(self):
        self.module.rospy.Time = NS(from_sec=lambda stamp: stamp)
        self.module.FlightSetpoint = lambda: NS(
            header=NS(), pose=NS(position=NS(), orientation=NS()))
        self.module.NavigationStatus = lambda: NS(header=NS())
        self.module.Path = lambda: NS(header=NS())
        self.module.PoseStamped = lambda: NS(pose=NS(position=NS(), orientation=NS()))

    def test_status_delay_cannot_publish_expired_target(self):
        self.output_messages()
        node = self.node()
        clock = [None, 10.1, 20.1]
        node._now = lambda: tuple(clock)
        node.planner = MagicMock()
        node.planner.plan.return_value = PlanResult(
            "CLEAR", "safe", Pose(0.01, 0, 1, 0), 0, 1, 0, 1, 0.1, True)
        node._publish_status.side_effect = lambda *a, **kw: clock.__setitem__(2, 20.8)
        node._plan_once(10.1, 20.1)
        node.target_pub.publish.assert_not_called()

    def test_status_clock_does_not_go_backwards_when_idle_becomes_active(self):
        self.output_messages()
        node = self.node()
        node._publish_status = self.module.NavigationNode._publish_status.__get__(node)
        clock = [10.1, 10.1, 20.1]
        node._now = lambda: tuple(clock)
        node._publish_status('IDLE', 'waiting', 1., 0., 0.)
        idle = node.status_pub.publish.call_args.args[0]
        clock[:] = [10.11, 10.11, 20.11]
        token, snapshot, reason = node.runtime.snapshot(10.11, 20.11)
        self.assertFalse(reason)
        node._publish_status('CLEAR', 'safe', 1., 0., snapshot.stamp, token=token)
        active = node.status_pub.publish.call_args.args[0]
        self.assertGreater(active.header.stamp, idle.header.stamp)
        self.assertAlmostEqual(active.cloud_age, 10.11-snapshot.stamp)

    def test_old_completion_is_not_published_for_replacement_goal(self):
        self.output_messages()
        node = self.node()
        node._now = lambda: (None, 10.1, 20.1)
        node._publish_status = self.module.NavigationNode._publish_status.__get__(node)
        node._publish_preview = self.module.NavigationNode._publish_preview.__get__(node)
        node.planner = MagicMock()
        node.planner.plan.return_value = PlanResult(
            "GOAL_REACHED", "reached", Pose(0.01, 0, 1, 0), 0, 1, 1, 1, 0, True)
        original = node.runtime.revalidate
        replaced = [False]

        def replace_after_validation(*args):
            valid = original(*args)
            if not replaced[0]:
                replaced[0] = True
                node.runtime.set_goal("new-goal", Pose(4, 0, 1, 0), 10.1)
            return valid

        node.runtime.revalidate = replace_after_validation
        node._plan_once(10.1, 20.1)
        node.status_pub.publish.assert_not_called()
        node.preview_pub.publish.assert_not_called()
        node.target_pub.publish.assert_not_called()

    def test_odometry_rejects_nonfinite_angular_twist_and_covariances(self):
        for field in ("angular", "pose_covariance", "twist_covariance"):
            node = self.node()
            node._now = lambda: (None, 10.1, 20.1)
            message = NS(
                header=NS(stamp=NS(to_sec=lambda: 10.1), frame_id="odom"),
                child_frame_id="base_link",
                pose=NS(pose=NS(position=NS(x=0, y=0, z=1),
                                orientation=NS(x=0, y=0, z=0, w=1)), covariance=[0.] * 36),
                twist=NS(twist=NS(linear=NS(x=0, y=0, z=0),
                                  angular=NS(x=0, y=0, z=0)), covariance=[0.] * 36))
            if field == "angular":
                message.twist.twist.angular.z = math.nan
            elif field == "pose_covariance":
                message.pose.covariance[5] = math.inf
            else:
                message.twist.covariance[35] = math.nan
            node._odom_callback(message)
            self.assertEqual(node.runtime.snapshot(10.1, 20.1)[2], "odom missing or stale")
            self.assertFalse(node.runtime.accept_odom(
                10.05, 20.1, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0)))

    def test_body_linear_twist_is_rotated_into_odom_for_yaw_and_tilt(self):
        for quaternion, expected in (((0, 0, 2 ** -0.5, 2 ** -0.5), (0, 1, 0)),
                                     ((0, 2 ** -0.5, 0, 2 ** -0.5), (0, 0, -1))):
            node = self.node()
            node._now = lambda: (None, 10.02, 20.02)
            message = NS(
                header=NS(stamp=NS(to_sec=lambda: 10.02), frame_id="odom"),
                child_frame_id="base_link",
                pose=NS(pose=NS(position=NS(x=0, y=0, z=1),
                                orientation=NS(**dict(zip(("x", "y", "z", "w"), quaternion)))),
                        covariance=[0.] * 36),
                twist=NS(twist=NS(linear=NS(x=1, y=0, z=0), angular=NS(x=0, y=0, z=0)),
                         covariance=[0.] * 36))
            node._odom_callback(message)
            node.runtime.accept_cloud(10.02, 20.02, "odom", ())
            node.runtime.accept_terrain(10.02, 20.02, "odom", Terrain(0, 1, 0.001, True))
            _, snapshot, reason = node.runtime.snapshot(10.02, 20.02)
            self.assertEqual(reason, "")
            for actual, wanted in zip((snapshot.velocity.x, snapshot.velocity.y, snapshot.velocity.z), expected):
                self.assertAlmostEqual(actual, wanted)

    def test_late_matching_terrain_wakes_planning_worker(self):
        node = self.node()
        node._now = lambda: (None, 10.2, 20.2)
        node._terrain_callback(NS(header=NS(stamp=NS(to_sec=lambda: 10.2), frame_id="odom"),
                                  ground_z=0, agl=1, variance=0.001, valid=True))
        self.assertTrue(node._plan_event.is_set())

    def test_cloud_then_late_terrain_plans_once_without_resetting_progress(self):
        self.output_messages()
        self.module.point_cloud2.read_points = lambda *args, **kwargs: ()
        node = self.node()
        node._now = lambda: (None, 10.2, 20.2)
        node.planner = MagicMock()
        node.planner.plan.return_value = PlanResult(
            "CLEAR", "safe", Pose(0.01, 0, 1, 0), 0, 1, 0, 1, 0.1, True)
        node.runtime.accept_odom(10.2, 20.2, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0))
        waiting, published = threading.Event(), threading.Event()
        node._publish_status.side_effect = lambda *args, **kwargs: (
            waiting.set() if args[1] == "input source skew exceeds limit" else None)
        node.target_pub.publish.side_effect = lambda _message: published.set()
        worker = threading.Thread(target=node._planning_loop)
        worker.start()
        try:
            node._cloud_callback(NS(header=NS(stamp=NS(to_sec=lambda: 10.2), frame_id="odom")))
            self.assertTrue(waiting.wait(1))
            node.planner.plan.assert_not_called()
            node.planner.reset_context.assert_not_called()
            node._terrain_callback(NS(header=NS(stamp=NS(to_sec=lambda: 10.2), frame_id="odom"),
                                      ground_z=0, agl=1, variance=0.001, valid=True))
            self.assertTrue(published.wait(1))
            for _ in range(3):
                node._plan_once(10.2, 20.2)
            node.planner.plan.assert_called_once()
            node.target_pub.publish.assert_called_once()
        finally:
            node._shutdown()
            worker.join(1)
        self.assertFalse(worker.is_alive())


if __name__ == "__main__":
    unittest.main()
