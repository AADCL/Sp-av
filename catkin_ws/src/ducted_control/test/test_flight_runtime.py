import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import threading
from unittest.mock import MagicMock, patch
import unittest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'src'))


def raw_config(enabled=False):
    return {
        'enable_flight_output': enabled, 'frame_id': 'odom',
        'telemetry_timeout': .5, 'base_ready_timeout': 1.5,
        'external_ready_timeout': .5, 'target_timeout': .4,
        'external_pose_timeout': .5, 'pose_pair_timeout': .3,
        'pose_history_duration': 1., 'max_external_pose_skew': .05,
        'max_external_position_error': .5, 'max_external_yaw_error': .35,
        'future_tolerance': .05, 'prestream_duration': 1.,
        'prestream_max_gap': .2, 'activation_timeout': 3.,
        'mode_feedback_timeout': 1., 'mode_request_timeout': .5,
        'planner_hold_grace': 2., 'takeoff_timeout': 5.,
        'takeoff_tolerance': .08, 'takeoff_settle_time': .3,
        'land_timeout': 8., 'min_takeoff_rise': .2,
        'min_x': -2., 'max_x': 2., 'min_y': -2., 'max_y': 2.,
        'min_z': -.5, 'max_z': 3., 'max_xy_speed': .6,
        'max_z_speed': .4, 'max_yaw_rate': .5, 'max_lead': .3,
        'loop_hz': 20.,
    }


def stamp(value):
    return NS(to_sec=lambda: value)


def pose_message(t=100., x=0., y=0., z=0., yaw=0., frame='odom'):
    return NS(header=NS(stamp=stamp(t), frame_id=frame), pose=NS(
        position=NS(x=x, y=y, z=z),
        orientation=NS(x=0., y=0., z=math.sin(yaw/2.), w=math.cos(yaw/2.))))


def blank_pose():
    return NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


def odom_message(t=100., x=0., y=0., z=0., yaw=0., frame='odom'):
    message = pose_message(t, x, y, z, yaw, frame)
    return NS(header=message.header, child_frame_id='base_link', pose=NS(pose=message.pose))


class FlightRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.ros = MagicMock()
        self.ros.get_time.return_value = 100.
        self.ros.Time.now.return_value = stamp(100.)
        self.params = {'~flight': raw_config(False)}
        self.ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        self.publishers = {}

        def publisher(name, *args, **kwargs):
            pub = MagicMock()
            self.publishers[name] = pub
            return pub
        self.ros.Publisher.side_effect = publisher
        self.ros.Service.side_effect = lambda *a, **k: MagicMock()

        geometry = NS(PoseStamped=lambda: NS(header=NS(), pose=blank_pose()))
        messages = NS(
            FlightSetpoint=object,
            FlightControlStatus=lambda: NS(header=NS(), state='', reason='', ready=False,
                                            request_id='', target=blank_pose()),
            RCState=object, TerrainHeight=object)
        services = NS(
            FlightCommand=object,
            FlightCommandResponse=lambda accepted=False, message='': NS(
                accepted=accepted, message=message))
        modules = {
            'rospy': self.ros,
            'geometry_msgs.msg': geometry,
            'ducted_msgs.msg': messages,
            'ducted_msgs.srv': services,
            'mavros_msgs.msg': NS(State=object, ExtendedState=NS(LANDED_STATE_ON_GROUND=1)),
            'mavros_msgs.srv': NS(SetMode=object),
            'nav_msgs.msg': NS(Odometry=object),
            'std_msgs.msg': NS(Bool=lambda data=False: NS(data=data)),
        }
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                'flight_controller_tested', PKG / 'scripts/flight_controller.py')
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.clock = patch.object(self.module, 'monotonic', return_value=10.)
        self.wall = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = self.module.FlightController()

    def feed_valid(self, t=100., node=None, fcu_mode='POSCTL'):
        node = self.node if node is None else node
        self.ros.get_time.return_value = t
        node._fcu_callback(NS(header=NS(stamp=stamp(t)), connected=True,
                              armed=True, mode=fcu_mode, system_status=4))
        node._pose_callback(pose_message(t))
        node._external_pose_callback(odom_message(t))
        node._landed_callback(NS(header=NS(stamp=stamp(t)), landed_state=1))
        node._rc_callback(NS(header=NS(stamp=stamp(t)), valid=True, mode='command',
                             kill_switch=False, land_switch=False))
        node._base_callback(NS(data=True))
        node._external_callback(NS(data=True))

    def test_default_disabled_has_status_only_and_no_backend_actions(self):
        self.feed_valid()
        response = self.node._command_callback(NS(command='engage', target=pose_message()))
        self.assertFalse(response.accepted)
        self.node._watchdog()
        self.publishers['setpoint'].publish.assert_not_called()
        self.ros.ServiceProxy.assert_not_called()
        self.assertFalse(self.publishers['ready'].publish.call_args[0][0].data)
        self.assertEqual(self.publishers['status'].publish.call_args[0][0].state, 'DISABLED')

    def test_landing_terrain_requires_correct_reference_geometry_and_fresh_attitude(self):
        self.assertTrue(hasattr(self.node, '_terrain_callback'))
        self.node.terrain_reference = 'base_link'
        self.node.body_vertices = [(-.3,-.35,-.1),(.3,.35,.1)]
        self.feed_valid()
        msg = NS(header=NS(stamp=stamp(100.),frame_id='odom'),
                 valid=True,ground_z=-.1,agl=.1,variance=.001)
        self.node._terrain_callback(msg)
        self.assertTrue(self.node.policy._terrain['valid'])
        self.assertAlmostEqual(self.node.policy._terrain['contact_agl'],.1)
        msg.header.stamp = stamp(100.01); msg.header.frame_id='map'
        self.node._terrain_callback(msg)
        self.assertFalse(self.node.policy._terrain['valid'])

    def test_newer_invalid_pose_clears_cached_pose_availability(self):
        self.feed_valid()
        self.assertTrue(self.node.policy.snapshot().pose_valid)
        invalid = pose_message(100.1)
        invalid.pose.position.x = math.nan
        self.ros.get_time.return_value = 100.1
        self.node._pose_callback(invalid)
        self.assertFalse(self.node.policy.snapshot().pose_valid)
        older = pose_message(100.05)
        self.node._pose_callback(older)
        self.assertFalse(self.node.policy.snapshot().pose_valid)

    def test_wrong_frame_local_pose_invalidates_cached_pose(self):
        self.feed_valid()
        wrong = pose_message(100.1, frame='map')
        self.ros.get_time.return_value = 100.1
        self.node._pose_callback(wrong)
        self.assertFalse(self.node.policy.snapshot().pose_valid)

    def test_invalid_external_odometry_pose_revokes_consistency_gate(self):
        self.feed_valid()
        invalid = odom_message(100.1)
        invalid.pose.pose.position.z = math.nan
        self.ros.get_time.return_value = 100.1
        self.node._external_pose_callback(invalid)
        self.assertIn('external pose', self.node.policy._gate_reason(100.1, 10.))

    def test_dispatch_revalidates_captured_action_before_publish(self):
        self.node.policy.action_current = MagicMock(return_value=False)
        action = NS(kind='setpoint', pose=NS(), token=0)
        self.node._dispatch_actions((action,))
        self.node.policy.action_current.assert_called_once()
        self.publishers['setpoint'].publish.assert_not_called()

    def test_callback_waiting_for_lock_uses_current_clock(self):
        self.feed_valid(fcu_mode='OFFBOARD')
        self.node.policy.state = 'HOLD'
        self.assertTrue(self.node.policy.snapshot().ready)
        lock = MagicMock()

        def enter():
            # Another callback wins the lock after this callback starts.
            self.wall.return_value = 10.01
            self.ros.get_time.return_value = 100.01
            self.node.policy.update_external_ready(True, 10.01)
        lock.__enter__.side_effect = enter
        self.node.lock = lock
        self.node._pose_callback(pose_message(100.005))
        self.assertTrue(self.node.policy.snapshot().ready)

    def test_watchdog_waiting_for_lock_does_not_revoke_fresh_ownership(self):
        self.feed_valid(fcu_mode='OFFBOARD')
        self.node.policy.config.enable_flight_output = True
        self.node.policy.state = 'HOLD'
        lock = MagicMock()

        def enter():
            self.wall.return_value = 10.01
            self.ros.get_time.return_value = 100.01
            self.node.policy.update_external_ready(True, 10.01)
        lock.__enter__.side_effect = enter
        self.node.lock = lock
        self.node._watchdog()
        self.assertEqual(self.node.policy.state, 'HOLD')
        self.assertTrue(self.publishers['ready'].publish.call_args[0][0].data)

    def test_mode_worker_rechecks_token_after_bounded_service_wait(self):
        action = NS(kind='mode', mode='OFFBOARD', token=7)
        self.node.policy = MagicMock()
        self.node.policy.config.mode_request_timeout = .5
        self.node.policy.action_current.side_effect = [True, False]
        self.node.mode_queue = MagicMock()
        self.node.mode_queue.get.side_effect = [action, None]
        self.node._mode_loop()
        self.assertEqual(self.node.policy.action_current.call_count, 2)
        self.ros.wait_for_service.assert_called_once()
        self.ros.ServiceProxy.assert_not_called()

    def test_invalid_quaternion_target_is_rejected_before_policy_acceptance(self):
        self.feed_valid()
        target = NS(header=NS(stamp=stamp(100.), frame_id='odom'), request_id='goal',
                    pose=blank_pose())
        target.pose.orientation.w = 0.
        self.node._target_callback(target)
        self.assertEqual(self.node.policy.snapshot().request_id, '')

    def test_takeoff_service_rejects_wrong_frame_and_zero_stamp(self):
        self.feed_valid()
        for frame, value in [('map', 100.), ('odom', 0.)]:
            target = pose_message(value, z=1., frame=frame)
            response = self.node._command_callback(NS(command='takeoff', target=target))
            self.assertFalse(response.accepted)

    def test_enabled_path_prestreams_and_uses_isolated_mode_backend_once(self):
        self.params['~flight'] = raw_config(True)
        called = threading.Event()
        client = MagicMock()
        client.side_effect = lambda *args: (called.set(), NS(mode_sent=True))[1]
        self.ros.ServiceProxy.return_value = client
        node = self.module.FlightController()
        self.addCleanup(node.shutdown)
        self.feed_valid(node=node)
        response = node._command_callback(NS(command='engage', target=pose_message()))
        self.assertTrue(response.accepted)
        for index in range(1, 25):
            self.wall.return_value = 10. + index * .05
            self.feed_valid(100. + index * .05, node=node)
            node._watchdog()
        self.assertTrue(called.wait(.5))
        self.wall.return_value = 11.25
        self.feed_valid(101.25, node=node, fcu_mode='OFFBOARD')
        node._watchdog()
        client.assert_called_once_with(0, 'OFFBOARD')
        self.assertTrue(self.publishers['ready'].publish.call_args[0][0].data)
        self.assertGreater(self.publishers['setpoint'].publish.call_count, 20)


if __name__ == '__main__':
    unittest.main()
