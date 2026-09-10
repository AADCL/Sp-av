"""Exercise the ROS callbacks without a flight controller or ROS installation."""
import importlib.util
import pathlib
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'src'))


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def to_nsec(self):
        return round(self.seconds * 1e9)

    def __sub__(self, other):
        return Stamp(self.seconds - other.seconds)


def vector():
    return NS(x=0., y=0., z=0.)


def odometry():
    return NS(header=NS(stamp=Stamp(100.), frame_id='map'), child_frame_id='livox_frame',
              pose=NS(pose=NS(position=vector(), orientation=NS(x=0., y=0., z=0., w=1.))),
              twist=NS(twist=NS(linear=vector(), angular=vector())))


def transform():
    return NS(header=NS(stamp=None, frame_id=''), child_frame_id='',
              transform=NS(translation=vector(), rotation=NS(x=0., y=0., z=0., w=1.)))


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.ros = MagicMock()
        self.ros.Time.now.return_value = Stamp(100.)
        self.params = {
            '~extrinsic/base_to_livox/translation': [0.13, 0., 0.],
            '~extrinsic/base_to_livox/rpy': [0., 0., 0.],
            '~covariance/pose_diagonal': [0.01] * 6,
            '~covariance/twist_diagonal': [0.02] * 6,
            '~input_contract_confirmed': True,
            '~align_with_fcu': False,
        }
        self.ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        self.ros.Publisher.side_effect = lambda *a, **k: MagicMock()
        self.tf = MagicMock()
        modules = {'rospy': self.ros, 'nav_msgs.msg': NS(Odometry=odometry),
                   'std_msgs.msg': NS(Bool=lambda data: NS(data=data)),
                   'tf2_ros': self.tf, 'geometry_msgs.msg': NS(TransformStamped=transform),
                   'sensor_msgs.msg': NS(Imu=object), 'mavros_msgs.msg': NS(State=object)}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('adapter_under_test',
                                                        PKG / 'scripts/fastlio_odometry_adapter.py')
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.clock = patch.object(self.module, 'monotonic', return_value=10., create=True)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = self.module.FastlioOdometryAdapter()
        self.node._base_ready_callback(NS(data=True))
        self.node._localization_ready_callback(NS(data=True))

    def test_unconfirmed_input_never_sends_to_px4(self):
        self.params['~input_contract_confirmed'] = False
        node = self.module.FastlioOdometryAdapter()
        node._base_ready_callback(NS(data=True))
        node._localization_ready_callback(NS(data=True))
        node._odometry_callback(odometry())
        node.publisher.publish.assert_not_called()

    def test_dropout_clears_ready_even_if_ros_clock_stops(self):
        self.node._odometry_callback(odometry())
        self.node.ready_publisher.publish.assert_called_with(True)
        self.now.return_value = 10.4
        self.node._watchdog()
        self.node.ready_publisher.publish.assert_called_with(False)

    def test_duplicate_and_backward_stamps_never_republish(self):
        self.node._odometry_callback(odometry())
        self.node._odometry_callback(odometry())
        older = odometry()
        older.header.stamp = Stamp(99.99)
        self.node._odometry_callback(older)
        self.assertEqual(self.node.publisher.publish.call_count, 1)

    def test_dead_base_monitor_cannot_leave_gate_open(self):
        self.now.return_value = 12.
        self.node._odometry_callback(odometry())
        self.node.publisher.publish.assert_not_called()

    def test_tf_uses_inverse_extrinsic_without_reparenting_sensor(self):
        msg = odometry()
        msg.pose.pose.position.x = 1.13
        self.node._odometry_callback(msg)
        sent = self.node.publisher.publish.call_args[0][0]
        self.assertAlmostEqual(sent.pose.pose.position.x, 1.)
        broadcaster = self.tf.TransformBroadcaster.return_value
        self.assertEqual(broadcaster.sendTransform.call_count, 1)
        tf = broadcaster.sendTransform.call_args[0][0]
        self.assertEqual((tf.header.frame_id, tf.child_frame_id), ('livox_frame', 'base_link'))
        self.assertAlmostEqual(tf.transform.translation.x, -0.13)

    def test_wrong_frame_and_stale_samples_are_rejected(self):
        wrong = odometry()
        wrong.header.frame_id = 'another_map'
        self.node._odometry_callback(wrong)
        old = odometry()
        old.header.stamp = Stamp(99.)
        self.node._odometry_callback(old)
        self.node.publisher.publish.assert_not_called()

    def test_rotated_tf_composes_back_to_identity(self):
        from ducted_bringup.fastlio_odometry import rotate_vector, quaternion_multiply
        self.params['~extrinsic/base_to_livox/rpy'] = [0.03, 0.4567, 0.]
        node = self.module.FastlioOdometryAdapter()
        node._base_ready_callback(NS(data=True))
        node._localization_ready_callback(NS(data=True))
        node._odometry_callback(odometry())
        tf = self.tf.TransformBroadcaster.return_value.sendTransform.call_args[0][0]
        tr = tf.transform.translation
        qr = tf.transform.rotation
        rotated = rotate_vector(node.orientation, (tr.x, tr.y, tr.z))
        for a, b in zip(rotated, node.translation):
            self.assertAlmostEqual(a + b, 0., places=12)
        product = quaternion_multiply(node.orientation, (qr.x, qr.y, qr.z, qr.w))
        for a, b in zip(product, (0., 0., 0., 1.)):
            self.assertAlmostEqual(a, b, places=12)

    def test_rotation_about_base_removes_sensor_lever_velocity(self):
        msg = odometry()
        msg.twist.twist.angular.z = 2.
        msg.twist.twist.linear.y = 0.26
        self.node._odometry_callback(msg)
        sent = self.node.publisher.publish.call_args[0][0]
        self.assertAlmostEqual(sent.twist.twist.linear.y, 0., places=12)

    def test_base_false_immediately_blocks_next_sample(self):
        self.node._odometry_callback(odometry())
        self.node._base_ready_callback(NS(data=False))
        msg = odometry()
        msg.header.stamp = Stamp(100.01)
        self.node._odometry_callback(msg)
        self.assertEqual(self.node.publisher.publish.call_count, 1)
        self.node.ready_publisher.publish.assert_called_with(False)

    def test_alignment_waits_for_fresh_disarmed_fcu_attitude(self):
        self.params['~align_with_fcu'] = True
        node = self.module.FastlioOdometryAdapter()
        node._base_ready_callback(NS(data=True))
        node._localization_ready_callback(NS(data=True))
        node._odometry_callback(odometry())
        node.publisher.publish.assert_not_called()

    def test_tilted_map_is_aligned_once_to_fcu_enu(self):
        from ducted_bringup.fastlio_odometry import quaternion_from_rpy
        self.params['~align_with_fcu'] = True
        self.params['~extrinsic/base_to_livox/translation'] = [0., 0., 0.]
        self.params['~extrinsic/base_to_livox/rpy'] = [0., 0.4567, 0.]
        node = self.module.FastlioOdometryAdapter()
        node._base_ready_callback(NS(data=True))
        node._localization_ready_callback(NS(data=True))
        node._state_callback(NS(connected=True, armed=False))
        node._attitude_callback(NS(header=NS(stamp=Stamp(100.)),
                                   orientation=NS(x=0., y=0., z=0., w=1.)))
        node._odometry_callback(odometry())
        sent = node.publisher.publish.call_args[0][0]
        self.assertEqual(sent.header.frame_id, 'odom')
        self.assertAlmostEqual(sent.pose.pose.orientation.y, 0., places=12)
        self.assertAlmostEqual(sent.pose.pose.orientation.w, 1., places=12)
        # A later FCU attitude update must not drag the fixed map around.
        q = quaternion_from_rpy(0., 0., 0.5)
        node._attitude_callback(NS(header=NS(stamp=Stamp(100.)),
                                   orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))
        msg = odometry(); msg.header.stamp = Stamp(100.01)
        node._odometry_callback(msg)
        sent = node.publisher.publish.call_args[0][0]
        self.assertAlmostEqual(sent.pose.pose.orientation.z, 0., places=12)


if __name__ == '__main__':
    unittest.main()
