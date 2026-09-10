"""Exercise terrain callbacks with scoped ROS and TF substitutes."""
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


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def to_nsec(self):
        return round(self.seconds * 1e9)

    def __sub__(self, other):
        return Stamp(self.seconds - other.seconds)


class ROSInitException(Exception):
    pass


class ROSException(Exception):
    pass


class TransformException(Exception):
    pass


def vector(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    return NS(x=x, y=y, z=z, w=w)


def transform(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    return NS(
        transform=NS(
            translation=vector(*translation),
            rotation=quaternion(*rotation),
        )
    )


def odometry(stamp=100.0):
    return NS(
        header=NS(stamp=Stamp(stamp), frame_id="map"),
        child_frame_id="livox_frame",
        pose=NS(
            pose=NS(
                position=vector(),
                orientation=quaternion(),
            )
        ),
    )


def cloud(points, stamp=100.0):
    return NS(header=NS(stamp=Stamp(stamp), frame_id="map"), points=points)


def terrain_message():
    return NS(
        header=NS(stamp=None, frame_id=""),
        ground_z=0.0,
        agl=0.0,
        variance=0.0,
        valid=False,
    )


def flat_points(z=0.0):
    return [
        (0.1 * ix, 0.1 * iy, z)
        for ix in range(-3, 4)
        for iy in range(-3, 4)
    ]


class TerrainRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.ros = MagicMock()
        self.ros.Time.now.return_value = Stamp(100.0)
        self.ros.Duration.side_effect = lambda seconds: seconds
        self.ros.ROSInitException = ROSInitException
        self.ros.ROSException = ROSException
        self.params = {
            "~reference_confirmed": True,
            "~terrain/min_points": 6,
            "~terrain/ground_band": 0.12,
            "~timeouts/odom": 0.5,
            "~timeouts/cloud": 0.5,
            "~timeouts/base_ready": 1.0,
            "~timeouts/max_header_age": 0.25,
            "~timeouts/future_tolerance": 0.05,
            "~timeouts/odom_cloud_skew": 0.05,
        }
        self.ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        self.publishers = {}

        def publisher(topic, *_args, **_kwargs):
            self.publishers[topic] = MagicMock()
            return self.publishers[topic]

        self.ros.Publisher.side_effect = publisher
        self.buffer = MagicMock()
        self.buffer.lookup_transform.side_effect = self._lookup_transform
        self.transforms = {
            ("odom", "map"): transform(),
            ("odom", "base_link"): transform((0.0, 0.0, 1.5)),
        }
        tf2 = NS(
            Buffer=MagicMock(return_value=self.buffer),
            TransformListener=MagicMock(),
            LookupException=TransformException,
            ConnectivityException=TransformException,
            ExtrapolationException=TransformException,
        )
        point_cloud2 = NS(read_points=lambda message, **_kwargs: iter(message.points))
        modules = {
            "rospy": self.ros,
            "tf2_ros": tf2,
            "ducted_msgs.msg": NS(TerrainHeight=terrain_message),
            "nav_msgs.msg": NS(Odometry=object),
            "sensor_msgs": NS(point_cloud2=point_cloud2),
            "sensor_msgs.point_cloud2": point_cloud2,
            "sensor_msgs.msg": NS(PointCloud2=object),
            "std_msgs.msg": NS(Bool=lambda data=False: NS(data=data)),
        }
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                "terrain_node_under_test", PKG / "scripts/terrain_height_node.py"
            )
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.clock = patch.object(
            self.module, "monotonic", return_value=10.0, create=True
        )
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = self.module.TerrainHeightNode()
        if hasattr(self.node, "_base_ready_callback"):
            self.node._base_ready_callback(NS(data=True))
        else:
            self.node.base_ready = True

    def _lookup_transform(self, target, source, stamp, _timeout):
        self.assertNotEqual(stamp.to_nsec(), 0, "TF lookup must use the measurement stamp")
        try:
            return self.transforms[(target, source)]
        except KeyError as error:
            raise TransformException(str(error))

    def _latest_height(self):
        return self.publishers["height"].publish.call_args[0][0]

    def _publish_scene(self, points=None, stamp=100.0):
        self.node._odom_callback(odometry(stamp))
        self.node._cloud_callback(cloud(points or flat_points(), stamp))
        return self._latest_height()

    def test_flat_scene_publishes_base_agl_in_odom(self):
        height = self._publish_scene()

        self.assertTrue(height.valid)
        self.assertEqual(height.header.frame_id, "odom")
        self.assertAlmostEqual(height.ground_z, 0.0)
        self.assertAlmostEqual(height.agl, 1.5)

    def test_tilted_map_scene_has_same_vertical_agl_after_exact_transform(self):
        angle = math.radians(30.0)
        q_odom_map = (0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0))
        self.transforms[("odom", "map")] = transform(rotation=q_odom_map)
        # Inverse-rotate odom-frame z=0 ground into the tilted map frame.
        tilted_map_points = [
            (math.cos(angle) * x, y, math.sin(angle) * x)
            for x, y, _z in flat_points()
        ]

        height = self._publish_scene(tilted_map_points)

        self.assertTrue(height.valid)
        self.assertAlmostEqual(height.ground_z, 0.0, places=10)
        self.assertAlmostEqual(height.agl, 1.5, places=10)
        calls = self.buffer.lookup_transform.call_args_list
        self.assertEqual((calls[0].args[0], calls[0].args[1]), ("odom", "map"))
        self.assertEqual((calls[1].args[0], calls[1].args[1]), ("odom", "base_link"))

    def test_body_bottom_uses_measured_body_tilt(self):
        self.params["~agl_reference"] = "body_bottom"
        self.params["~body_vertices"] = [[-0.5, 0.0, -0.2], [0.5, 0.0, -0.2]]
        half_angle = math.pi / 8.0
        self.transforms[("odom", "base_link")] = transform(
            (0.0, 0.0, 1.0), (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))
        )
        node = self.module.TerrainHeightNode()
        node._base_ready_callback(NS(data=True))
        node._odom_callback(odometry())
        node._cloud_callback(cloud(flat_points()))
        height = self.publishers["height"].publish.call_args[0][0]

        self.assertTrue(height.valid)
        self.assertAlmostEqual(height.agl, 1.0 - 0.7 / math.sqrt(2.0))

    def test_unavailable_exact_time_tf_invalidates_output(self):
        wall = [10.0]

        def unavailable(*_args):
            wall[0] += 0.06
            raise TransformException("no exact transform")

        self.now.side_effect = lambda: wall[0]
        self.buffer.lookup_transform.side_effect = unavailable

        height = self._publish_scene()

        self.assertFalse(height.valid)
        self.publishers["ready"].publish.assert_called_with(False)

    def test_short_tf_lag_retries_nonblocking_lookup_and_still_publishes(self):
        wall = [10.0]
        attempts = []

        def lag_once(target, source, stamp, timeout):
            attempts.append((target, source, timeout))
            if len(attempts) == 1:
                wall[0] += 0.01
                raise TransformException("TF has not arrived yet")
            return self._lookup_transform(target, source, stamp, timeout)

        self.now.side_effect = lambda: wall[0]
        self.buffer.lookup_transform.side_effect = lag_once

        height = self._publish_scene()

        self.assertTrue(height.valid)
        self.assertGreaterEqual(len(attempts), 3)
        self.assertTrue(all(call[2] == 0.0 for call in attempts))

    def test_watchdog_can_revoke_while_tf_lookup_is_blocked(self):
        self.assertTrue(self._publish_scene().valid)
        self.node._odom_callback(odometry(100.01))
        entered = threading.Event()
        release = threading.Event()

        def blocked_lookup(target, source, stamp, timeout):
            self.assertEqual(timeout, 0.0)
            if target == "odom" and source == "map":
                entered.set()
                release.wait(1.0)
            return self._lookup_transform(target, source, stamp, timeout)

        self.buffer.lookup_transform.side_effect = blocked_lookup
        callback = threading.Thread(
            target=self.node._cloud_callback,
            args=(cloud(flat_points(), 100.01),),
        )
        callback.start()
        self.assertTrue(entered.wait(0.2))
        self.now.return_value = 11.1

        watchdog = threading.Thread(target=self.node._watchdog)
        watchdog.start()
        watchdog.join(0.2)
        self.assertFalse(watchdog.is_alive(), "TF wait held the state lock")
        self.assertFalse(self._latest_height().valid)

        release.set()
        callback.join(0.5)
        self.assertFalse(callback.is_alive())
        self.assertFalse(self._latest_height().valid)

    def test_replaced_cloud_cannot_publish_after_lookup_finishes(self):
        self.node._odom_callback(odometry(100.0))
        entered = threading.Event()
        release = threading.Event()

        def controlled_lookup(target, source, stamp, timeout):
            self.assertEqual(timeout, 0.0)
            if stamp.to_nsec() == Stamp(100.0).to_nsec() and source == "map":
                entered.set()
                release.wait(1.0)
            if source == "base_link" and stamp.to_nsec() == Stamp(100.01).to_nsec():
                return transform((0.0, 0.0, 1.8))
            return self._lookup_transform(target, source, stamp, timeout)

        self.buffer.lookup_transform.side_effect = controlled_lookup
        old_callback = threading.Thread(
            target=self.node._cloud_callback, args=(cloud(flat_points(), 100.0),)
        )
        old_callback.start()
        self.assertTrue(entered.wait(0.2))

        self.node._odom_callback(odometry(100.01))
        self.node._cloud_callback(cloud(flat_points(), 100.01))
        self.assertTrue(self._latest_height().valid)
        self.assertAlmostEqual(self._latest_height().agl, 1.8)

        release.set()
        old_callback.join(0.5)
        self.assertFalse(old_callback.is_alive())
        self.assertTrue(self._latest_height().valid)
        self.assertAlmostEqual(self._latest_height().agl, 1.8)

    def test_nonfinite_exact_time_tf_invalidates_output(self):
        self.transforms[("odom", "map")] = transform((0.0, 0.0, math.nan))

        height = self._publish_scene()

        self.assertFalse(height.valid)
        self.publishers["ready"].publish.assert_called_with(False)

    def test_stale_future_and_skewed_headers_are_rejected(self):
        stale = odometry(99.0)
        self.node._odom_callback(stale)
        self.assertIsNone(self.node.odom)
        future = odometry(101.0)
        self.node._odom_callback(future)
        self.assertIsNone(self.node.odom)
        self.node._odom_callback(odometry(100.0))
        self.node._cloud_callback(cloud(flat_points(), 100.2))
        self.assertFalse(self._latest_height().valid)

    def test_stale_and_future_cloud_headers_are_rejected(self):
        self.node._odom_callback(odometry())
        self.node._cloud_callback(cloud(flat_points(), 99.0))
        self.assertFalse(self._latest_height().valid)
        self.node._cloud_callback(cloud(flat_points(), 101.0))
        self.assertFalse(self._latest_height().valid)

    def test_duplicate_measurements_do_not_refresh_monotonic_arrival(self):
        self.node._odom_callback(odometry())
        self.now.return_value = 10.2
        self.node._odom_callback(odometry())
        self.assertIsNone(self.node.odom)
        self.assertIsNone(self.node.odom_received_wall)
        self.node._cloud_callback(cloud(flat_points()))
        cloud_received = self.node.cloud_received_wall
        self.now.return_value = 10.3
        self.node._cloud_callback(cloud(flat_points()))
        self.assertEqual(self.node.cloud_received_wall, cloud_received)
        self.assertFalse(self._latest_height().valid)

    def test_malformed_newer_odometry_advances_seen_watermark(self):
        self.node._odom_callback(odometry(100.0))
        malformed = odometry(100.02)
        malformed.pose.pose.position.z = math.nan
        self.node._odom_callback(malformed)

        self.node._odom_callback(odometry(100.01))

        self.assertIsNone(self.node.odom)
        self.assertEqual(self.node.last_odom_stamp_ns, Stamp(100.02).to_nsec())

    def test_invalid_newer_cloud_advances_seen_watermark(self):
        self.node._odom_callback(odometry(100.02))
        malformed = cloud(flat_points(), 100.02)
        malformed.header.frame_id = "wrong_map"
        self.node._cloud_callback(malformed)

        self.node._cloud_callback(cloud(flat_points(), 100.01))

        self.assertFalse(self._latest_height().valid)
        self.assertEqual(self.node.last_cloud_stamp_ns, Stamp(100.02).to_nsec())

    def test_cached_odometry_header_is_rechecked_for_cloud_snapshot(self):
        self.node._odom_callback(odometry(100.0))
        self.ros.Time.now.return_value = Stamp(100.29)

        self.node._cloud_callback(cloud(flat_points(), 100.04))

        self.assertIsNone(self.node.odom)
        self.assertFalse(self._latest_height().valid)
        self.buffer.lookup_transform.assert_not_called()

    def test_malformed_odometry_clears_cached_sample(self):
        self.node._odom_callback(odometry())
        malformed = odometry(100.01)
        malformed.pose.pose.position.z = math.nan

        self.node._odom_callback(malformed)

        self.assertIsNone(self.node.odom)
        self.assertFalse(self._latest_height().valid)

    def test_base_heartbeat_loss_revokes_validity_with_frozen_ros_clock(self):
        self.assertTrue(self._publish_scene().valid)
        self.now.return_value = 11.1

        self.node._watchdog()

        self.assertFalse(self._latest_height().valid)
        self.publishers["ready"].publish.assert_called_with(False)

    def test_watchdog_rechecks_source_age_before_arrival_timeout(self):
        self.assertTrue(self._publish_scene().valid)
        self.now.return_value = 10.3
        self.ros.Time.now.return_value = Stamp(100.3)
        self.node._watchdog()
        self.assertFalse(self._latest_height().valid)

    def test_ros_clock_rollback_revokes_cached_height(self):
        self.assertTrue(self._publish_scene().valid)
        self.ros.Time.now.return_value = Stamp(99.0)
        self.node._watchdog()
        self.assertFalse(self._latest_height().valid)

    def test_reference_gate_defaults_false_and_blocks_height(self):
        self.params.pop("~reference_confirmed")
        node = self.module.TerrainHeightNode()
        node._base_ready_callback(NS(data=True))
        node._odom_callback(odometry())
        node._cloud_callback(cloud(flat_points()))

        height = self.publishers["height"].publish.call_args[0][0]
        self.assertFalse(height.valid)

    def test_invalid_body_geometry_and_timeout_configuration_fail_init(self):
        self.params["~agl_reference"] = "body_bottom"
        self.params["~body_vertices"] = []
        with self.assertRaises(ROSInitException):
            self.module.TerrainHeightNode()
        self.params["~agl_reference"] = "base_link"
        self.params["~timeouts/odom"] = math.nan
        with self.assertRaises(ROSInitException):
            self.module.TerrainHeightNode()
        self.params["~timeouts/odom"] = 0.5
        self.params["~terrain/max_slope"] = math.nan
        with self.assertRaises(ROSInitException):
            self.module.TerrainHeightNode()

    def test_startup_wait_uses_monotonic_deadline_when_ros_time_is_frozen(self):
        self.params["~base_ready_timeout"] = 0.1
        self.ros.is_shutdown.return_value = False
        self.ros.wait_for_message.side_effect = ROSException("no heartbeat")
        self.now.side_effect = [20.0, 20.05, 20.11]

        self.assertFalse(self.module.wait_for_base_ready())


if __name__ == "__main__":
    unittest.main()
