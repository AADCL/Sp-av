"""The optional PX4 sender must not own TF or reinterpret the converted pose."""
import importlib.util
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_odometry_runtime import Stamp, odometry


class RelayTest(unittest.TestCase):
    def setUp(self):
        self.ros = MagicMock()
        self.ros.Time.now.return_value = Stamp(100.)
        self.ros.Publisher.side_effect = lambda *a, **kw: MagicMock()
        self.params = {'~input_contract_confirmed': True}
        self.ros.get_param.side_effect = lambda k, d=None: self.params.get(k, d)
        spec = importlib.util.spec_from_file_location('relay_under_test',
            Path(__file__).resolve().parents[1] / 'scripts/external_odometry_relay.py')
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'rospy': self.ros,
                'nav_msgs.msg': NS(Odometry=object), 'std_msgs.msg': NS(Bool=object)}):
            spec.loader.exec_module(self.module)
        timer = patch.object(self.module, 'monotonic', return_value=10.)
        self.clock = timer.start()
        self.addCleanup(timer.stop)
        self.node = self.module.ExternalOdometryRelay()
        self.node._base_ready_callback(NS(data=True))
        self.node._localization_ready_callback(NS(data=True))

    def message(self, stamp=100.):
        m = odometry()
        m.header.frame_id = 'odom'
        m.header.stamp = Stamp(stamp)
        m.child_frame_id = 'base_link'
        m.pose.covariance = [.01] * 36
        m.twist.covariance = [.02] * 36
        return m

    def test_enabled_forwarding_preserves_entire_message(self):
        msg = self.message()
        self.node._odometry_callback(msg)
        self.node.publisher.publish.assert_called_once_with(msg)
        self.assertIs(self.node.publisher.publish.call_args[0][0], msg)
        self.node.ready_publisher.publish.assert_called_with(True)

    def test_default_confirmation_gate_blocks_output(self):
        self.params.clear()
        node = self.module.ExternalOdometryRelay()
        node._base_ready_callback(NS(data=True))
        node._localization_ready_callback(NS(data=True))
        node._odometry_callback(self.message())
        node.publisher.publish.assert_not_called()

    def test_raw_lidar_odometry_cannot_be_forwarded(self):
        self.node._odometry_callback(odometry())
        self.node.publisher.publish.assert_not_called()

    def test_duplicate_clears_ready_and_does_not_repeat(self):
        self.node._odometry_callback(self.message())
        self.node._odometry_callback(self.message())
        self.assertEqual(self.node.publisher.publish.call_count, 1)
        self.node.ready_publisher.publish.assert_called_with(False)

    def test_invalid_payload_consumes_stamp(self):
        bad = self.message()
        bad.pose.pose.position.x = float('nan')
        self.node._odometry_callback(bad)
        self.node._odometry_callback(self.message())
        self.node.publisher.publish.assert_not_called()

    def test_future_stamp_does_not_poison_recovery(self):
        self.node._odometry_callback(self.message(1000.))
        self.node._odometry_callback(self.message())
        self.assertEqual(self.node.publisher.publish.call_count, 1)

    def test_bad_quaternion_or_covariance_blocks_output(self):
        for index, field in enumerate(('quaternion', 'covariance')):
            bad = self.message(100. + index * .01)
            if field == 'quaternion': bad.pose.pose.orientation.w = 0.
            else: bad.twist.covariance[0] = float('inf')
            self.node._odometry_callback(bad)
        self.node.publisher.publish.assert_not_called()

    def test_upstream_revocation_blocks_immediately(self):
        self.node._localization_ready_callback(NS(data=False))
        self.node._odometry_callback(self.message())
        self.node.publisher.publish.assert_not_called()

    def test_base_timeout_blocks_even_with_frozen_ros_clock(self):
        self.clock.return_value = 12.
        self.node._odometry_callback(self.message())
        self.node.publisher.publish.assert_not_called()

    def test_watchdog_revokes_on_dropout(self):
        self.node._odometry_callback(self.message())
        self.clock.return_value = 10.4
        self.node._watchdog()
        self.node.ready_publisher.publish.assert_called_with(False)

    def test_watchdog_checks_source_age_and_clock_rollback(self):
        for stamp in (100.3, 99.):
            self.ros.Time.now.return_value = Stamp(100.)
            node = self.module.ExternalOdometryRelay()
            node._base_ready_callback(NS(data=True))
            node._localization_ready_callback(NS(data=True))
            node._odometry_callback(self.message())
            self.ros.Time.now.return_value = Stamp(stamp)
            node._watchdog()
            node.ready_publisher.publish.assert_called_with(False)


if __name__ == '__main__':
    unittest.main()
