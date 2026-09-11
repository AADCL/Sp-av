import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from ducted_control.flight import FlightConfig, FlightPolicy, Pose


def config(**overrides):
    values = {
        'enable_flight_output': True,
        'frame_id': 'odom',
        'telemetry_timeout': .5,
        'base_ready_timeout': .5,
        'external_ready_timeout': .5,
        'external_pose_timeout': .5,
        'pose_pair_timeout': .3,
        'pose_history_duration': 1.,
        'max_external_pose_skew': .05,
        'max_external_position_error': .5,
        'max_external_yaw_error': .35,
        'target_timeout': .4,
        'future_tolerance': .05,
        'prestream_duration': 1.,
        'prestream_max_gap': .2,
        'activation_timeout': 3.,
        'mode_feedback_timeout': 1.,
        'mode_request_timeout': .5,
        'planner_hold_grace': 2.,
        'takeoff_timeout': 5.,
        'takeoff_tolerance': .08,
        'takeoff_settle_time': .3,
        'land_timeout': 8.,
        'min_takeoff_rise': .2,
        'min_x': -2., 'max_x': 2., 'min_y': -2., 'max_y': 2.,
        'min_z': -.5, 'max_z': 3.,
        'max_xy_speed': .6, 'max_z_speed': .4,
        'max_yaw_rate': .5, 'max_lead': .3,
    }
    values.update(overrides)
    return FlightConfig(values)


class Harness:
    def __init__(self, **overrides):
        self.p = FlightPolicy(config(**overrides))
        self.ros = 100.
        self.wall = 10.
        self.pose = Pose(0., 0., 0., 0.)

    def advance(self, seconds):
        self.ros += seconds
        self.wall += seconds

    def telemetry(self, rc_mode='command', armed=True, fcu_mode='POSCTL',
                  kill=False, land=False, landed=1, pose=None, external_pose=None):
        self.pose = self.pose if pose is None else pose
        external_pose = self.pose if external_pose is None else external_pose
        self.p.update_fcu(True, armed, fcu_mode, 4, self.ros, self.ros, self.wall)
        self.p.update_pose(self.pose, self.ros, self.ros, self.wall)
        self.p.update_external_pose(external_pose, self.ros, self.ros, self.wall)
        self.p.update_landed(landed, self.ros, self.ros, self.wall)
        self.p.update_rc(True, rc_mode, kill, land, self.ros, self.ros, self.wall)
        self.p.update_base_ready(True, self.wall)
        self.p.update_external_ready(True, self.wall)

    def refresh(self, **kwargs):
        self.advance(.05)
        self.telemetry(**kwargs)

    def engage(self):
        result = self.p.command('engage', None, self.ros, self.wall)
        self.assert_accepted(result)
        return result

    @staticmethod
    def assert_accepted(result):
        if not result.accepted:
            raise AssertionError(result.message)

    def finish_engage(self):
        self.telemetry()
        self.engage()
        requests = []
        for _ in range(24):
            actions = self.p.tick(self.ros, self.wall)
            requests += [a for a in actions if a.kind == 'mode']
            self.refresh()
        if len(requests) != 1:
            raise AssertionError('expected one OFFBOARD request, got %d' % len(requests))
        request = requests[0]
        self.p.mode_result(request.token, True, self.wall)
        self.refresh(fcu_mode='OFFBOARD')
        self.p.tick(self.ros, self.wall)
        return request


class FlightPolicyTest(unittest.TestCase):
    def test_automatic_lease_loss_revokes_output_without_reentry(self):
        h=Harness(require_automatic_lease=True)
        original=h.telemetry
        def leased(**kwargs):
            h.p.update_automatic_lease(True,h.wall)
            original(**kwargs)
        h.telemetry=leased
        h.finish_engage()
        h.telemetry=original
        h.advance(.6);h.telemetry(fcu_mode='OFFBOARD')
        self.assertEqual((),h.p.tick(h.ros,h.wall))
        self.assertEqual('DISABLED',h.p.state)
        h.p.update_automatic_lease(True,h.wall)
        self.assertEqual((),h.p.tick(h.ros,h.wall))

    def test_disarmed_reset_does_not_require_old_automatic_lease(self):
        h=Harness(require_automatic_lease=True)
        h.telemetry(armed=False);h.p.state='INHIBITED'
        self.assertTrue(h.p.command('reset',None,h.ros,h.wall).accepted)

    def test_abort_during_prestream_prevents_offboard_request(self):
        h=Harness();h.telemetry();h.engage()
        self.assertTrue(h.p.command('abort',None,h.ros,h.wall).accepted)
        self.assertEqual('DISABLED',h.p.state)
        self.assertEqual((),h.p.tick(h.ros,h.wall))

    def test_abort_climb_holds_actual_pose(self):
        h=Harness();h.finish_engage()
        self.assertTrue(h.p.command('takeoff',Pose(0,0,1),h.ros,h.wall).accepted)
        h.refresh(fcu_mode='OFFBOARD',pose=Pose(0,0,.3),landed=2)
        self.assertTrue(h.p.command('abort',None,h.ros,h.wall).accepted)
        self.assertEqual('HOLD',h.p.state)
        self.assertEqual(h.pose,h.p.target)

    def test_explicit_hold_revokes_old_stream_and_pre_hold_messages(self):
        h = Harness(); h.finish_engage()
        target = Pose(.1, 0., 0., 0.)
        self.assertTrue(h.p.accept_target('old-goal', target, 'odom', h.ros,
                                        h.ros, h.wall).accepted)
        h.refresh(fcu_mode='OFFBOARD')
        pending_stamp = h.ros
        h.refresh(fcu_mode='OFFBOARD')
        self.assertTrue(h.p.command('hold', None, h.ros, h.wall).accepted)
        h.refresh(fcu_mode='OFFBOARD')
        self.assertFalse(h.p.accept_target('delayed-goal', target, 'odom', pending_stamp,
                                         h.ros, h.wall).accepted)
        self.assertEqual(h.p.state, 'HOLD')
        self.assertFalse(h.p.accept_target('old-goal', target, 'odom', h.ros,
                                         h.ros, h.wall).accepted)
        self.assertEqual(h.p.state, 'HOLD')
        h.refresh(fcu_mode='OFFBOARD')
        self.assertTrue(h.p.accept_target('new-goal', target, 'odom', h.ros,
                                        h.ros, h.wall).accepted)
        self.assertEqual(h.p.state, 'TRACK')

    def test_disabled_default_and_startup_never_produce_actions(self):
        p = FlightPolicy(config(enable_flight_output=False))
        self.assertFalse(p.command('engage', None, 100., 10.).accepted)
        self.assertEqual(p.tick(100., 10.), ())
        self.assertEqual(p.snapshot().state, 'DISABLED')

    def test_disabled_output_still_reports_latched_kill_without_actions(self):
        h = Harness(enable_flight_output=False); h.telemetry(kill=True)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.snapshot().state, 'INHIBITED')

    def test_first_high_and_reconnection_do_not_engage_or_land(self):
        h = Harness(); h.telemetry(rc_mode='command', land=True)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        h.advance(.6); h.telemetry(rc_mode='command', land=True)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.snapshot().state, 'DISABLED')

    def test_engage_requires_every_fresh_gate_and_armed_feedback(self):
        h = Harness(); h.telemetry(armed=False)
        self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)
        h.telemetry(armed=True); h.advance(.51)
        self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)
        for mode in ('manual', 'invalid'):
            h.telemetry(rc_mode=mode)
            self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)

    def test_invalid_duplicate_future_and_stale_updates_clear_availability(self):
        h = Harness(); h.telemetry()
        h.p.update_pose(Pose(0, 0, 0, 0), h.ros, h.ros, h.wall)
        self.assertFalse(h.p.snapshot().pose_valid)
        h.advance(.1)
        h.p.update_fcu(True, True, 'POSCTL', 4, h.ros + .1, h.ros, h.wall)
        self.assertFalse(h.p.snapshot().fcu_valid)
        h.p.update_rc(False, 'command', False, False, h.ros, h.ros, h.wall)
        self.assertFalse(h.p.snapshot().rc_valid)
        h.telemetry(); h.advance(.51)
        h.p.tick(h.ros, h.wall)
        self.assertFalse(h.p.snapshot().base_valid)
        self.assertFalse(h.p.snapshot().external_valid)

    def test_base_and_external_heartbeats_have_independent_freshness(self):
        h = Harness(base_ready_timeout=1.5, external_ready_timeout=.5)
        h.telemetry(); h.advance(.6); h.p.tick(h.ros, h.wall)
        self.assertTrue(h.p.snapshot().base_valid)
        self.assertFalse(h.p.snapshot().external_valid)

    def test_external_and_fcu_pose_must_have_consistent_origin_yaw_and_time(self):
        for external in (Pose(.51, 0, 0, 0), Pose(0, 0, 0, .36)):
            h = Harness(); h.telemetry(external_pose=external)
            result = h.p.command('engage', None, h.ros, h.wall)
            self.assertFalse(result.accepted)
            self.assertIn('consistency', result.message)
        h = Harness(); h.telemetry(); h.advance(.31)
        h.p.update_fcu(True, True, 'POSCTL', 4, h.ros, h.ros, h.wall)
        h.p.update_pose(h.pose, h.ros, h.ros, h.wall)
        h.p.update_landed(1, h.ros, h.ros, h.wall)
        h.p.update_rc(True, 'command', False, False, h.ros, h.ros, h.wall)
        h.p.update_base_ready(True, h.wall); h.p.update_external_ready(True, h.wall)
        result = h.p.command('engage', None, h.ros, h.wall)
        self.assertFalse(result.accepted)
        self.assertIn('pair', result.message)

    def test_delayed_external_pose_pairs_against_bounded_fcu_history(self):
        h = Harness()
        h.p.update_fcu(True, True, 'POSCTL', 4, h.ros, h.ros, h.wall)
        h.p.update_pose(Pose(), h.ros, h.ros, h.wall)
        h.p.update_landed(1, h.ros, h.ros, h.wall)
        h.p.update_rc(True, 'command', False, False, h.ros, h.ros, h.wall)
        h.p.update_base_ready(True, h.wall); h.p.update_external_ready(True, h.wall)
        source_stamp = h.ros
        h.advance(.12)
        h.p.update_external_pose(Pose(), source_stamp, h.ros, h.wall)
        h.p.update_fcu(True, True, 'POSCTL', 4, h.ros, h.ros, h.wall)
        h.p.update_landed(1, h.ros, h.ros, h.wall)
        h.p.update_rc(True, 'command', False, False, h.ros, h.ros, h.wall)
        h.p.update_base_ready(True, h.wall); h.p.update_external_ready(True, h.wall)
        self.assertTrue(h.p.command('engage', None, h.ros, h.wall).accepted)
        for _ in range(120):
            h.advance(.005)
            h.p.update_pose(Pose(), h.ros, h.ros, h.wall)
        self.assertLessEqual(len(h.p._pose_history), 100)

    def test_prestream_is_continuous_and_requests_offboard_once(self):
        h = Harness(); h.telemetry(); h.engage()
        first = h.p.tick(h.ros, h.wall)
        self.assertEqual([a.kind for a in first], ['setpoint'])
        h.advance(.21); h.telemetry()
        self.assertEqual([a.kind for a in h.p.tick(h.ros, h.wall)], ['setpoint'])
        requests = []
        for _ in range(19):
            h.refresh(); requests += [a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode']
        self.assertEqual(len(requests), 0)
        for _ in range(2):
            h.refresh(); requests += [a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode']
        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0].mode, 'OFFBOARD')
        self.assertTrue(h.p.mode_pending(requests[0].token))

    def test_mode_acceptance_is_not_completion_and_reject_timeout_late_are_bounded(self):
        h = Harness(); h.telemetry(); h.engage()
        request = None
        for _ in range(22):
            actions = h.p.tick(h.ros, h.wall)
            request = next((a for a in actions if a.kind == 'mode'), request)
            h.refresh()
        self.assertIsNotNone(request)
        h.p.mode_result(request.token, True, h.wall)
        self.assertFalse(h.p.snapshot().ready)
        h.advance(1.01); h.telemetry()
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.snapshot().state, 'DISABLED')
        self.assertFalse(h.p.mode_pending(request.token))
        self.assertFalse(h.p.mode_result(request.token, True, h.wall))

        h = Harness(); h.telemetry(); h.engage()
        requests = []
        for _ in range(22):
            actions = h.p.tick(h.ros, h.wall); h.refresh()
            requests += [a for a in actions if a.kind == 'mode']
        request = requests[0]
        self.assertTrue(h.p.mode_result(request.token, False, h.wall))
        self.assertEqual(h.p.snapshot().state, 'DISABLED')
        self.assertEqual(h.p.tick(h.ros, h.wall), ())

    def test_sent_mode_timeout_latches_uncertainty_until_late_backend_return(self):
        h = Harness(); h.telemetry(); h.engage()
        request = None
        for _ in range(22):
            request = next((a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode'), request)
            h.refresh()
        self.assertTrue(h.p.mode_dispatched(request.token, h.wall))
        h.advance(.51); h.telemetry()
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.snapshot().state, 'INHIBITED')
        self.assertIn('uncertain', h.p.snapshot().reason)
        h.advance(.05); h.telemetry(armed=False)
        self.assertFalse(h.p.command('reset', None, h.ros, h.wall).accepted)
        self.assertFalse(h.p.mode_result(request.token, True, h.wall))
        self.assertEqual(h.p.snapshot().state, 'INHIBITED')
        self.assertTrue(h.p.command('reset', None, h.ros, h.wall).accepted)

    def test_gate_loss_while_mode_call_is_sent_blocks_reengage_until_return(self):
        h = Harness(); h.telemetry(); h.engage()
        request = None
        for _ in range(22):
            request = next((a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode'), request)
            h.refresh()
        self.assertTrue(h.p.mode_dispatched(request.token, h.wall))
        h.advance(.05)
        h.p.update_rc(False, 'command', False, False, h.ros, h.ros, h.wall)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.snapshot().state, 'INHIBITED')
        h.advance(.05)
        h.telemetry(armed=False)
        self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)
        self.assertFalse(h.p.mode_result(request.token, False, h.wall))
        self.assertEqual(h.p.snapshot().state, 'INHIBITED')
        self.assertTrue(h.p.command('reset', None, h.ros, h.wall).accepted)

    def test_measured_offboard_feedback_completes_engage_and_overlap_is_rejected(self):
        h = Harness(); request = h.finish_engage()
        self.assertEqual(h.p.snapshot().state, 'HOLD')
        self.assertTrue(h.p.snapshot().ready)
        self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)
        self.assertFalse(h.p.mode_result(request.token, True, h.wall))

    def test_manual_kill_rc_loss_and_pilot_takeover_revoke_without_reentry(self):
        for loss in ('manual', 'kill', 'rc', 'pilot'):
            h = Harness(); h.finish_engage()
            if loss == 'manual': h.refresh(rc_mode='manual', fcu_mode='OFFBOARD')
            elif loss == 'kill': h.refresh(kill=True, fcu_mode='OFFBOARD')
            elif loss == 'rc': h.advance(.51)
            else: h.refresh(fcu_mode='POSCTL')
            actions = h.p.tick(h.ros, h.wall)
            self.assertFalse(any(a.kind == 'setpoint' for a in actions), loss)
            self.assertFalse(h.p.snapshot().ready, loss)
            for _ in range(30):
                h.refresh(fcu_mode='POSCTL'); h.p.tick(h.ros, h.wall)
            self.assertFalse(any(a.kind == 'mode' and a.mode == 'OFFBOARD'
                                 for a in h.p.tick(h.ros, h.wall)), loss)
            if loss == 'kill': self.assertEqual(h.p.snapshot().state, 'INHIBITED')

    def test_captured_actions_are_invalid_after_callback_revokes_their_gates(self):
        h = Harness(); h.finish_engage()
        action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
        self.assertTrue(h.p.action_current(action, h.ros, h.wall))
        h.advance(.05)
        h.p.update_rc(True, 'command', True, False, h.ros, h.ros, h.wall)
        self.assertFalse(h.p.action_current(action, h.ros, h.wall))

    def test_invalid_newer_target_cancels_already_captured_setpoint_action(self):
        h = Harness(); h.finish_engage()
        self.assertTrue(h.p.accept_target('g', Pose(.2, 0, .2, 0), 'odom',
                                          h.ros, h.ros, h.wall).accepted)
        action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
        h.advance(.05); h.telemetry(fcu_mode='OFFBOARD')
        self.assertFalse(h.p.accept_target('g', Pose(math.nan, 0, .2, 0), 'odom',
                                           h.ros, h.ros, h.wall).accepted)
        self.assertFalse(h.p.action_current(action, h.ros, h.wall))

    def test_explicit_release_requests_posctl_once_and_rejects_overlap(self):
        h = Harness(); h.finish_engage()
        self.assertTrue(h.p.command('release', None, h.ros, h.wall).accepted)
        self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)
        actions = h.p.tick(h.ros, h.wall)
        request = next(a for a in actions if a.kind == 'mode')
        self.assertEqual(request.mode, 'POSCTL')
        self.assertTrue(h.p.mode_pending(request.token))
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        h.p.mode_result(request.token, True, h.wall)
        h.refresh(fcu_mode='POSCTL')
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.snapshot().state, 'DISABLED')

    def test_inhibit_reset_requires_disarmed_kill_clear_and_fresh_telemetry(self):
        h = Harness(); h.telemetry(kill=True)
        h.p.tick(h.ros, h.wall)
        self.assertEqual(h.p.snapshot().state, 'INHIBITED')
        h.telemetry(kill=False, armed=True)
        self.assertFalse(h.p.command('reset', None, h.ros, h.wall).accepted)
        h.advance(.05)
        h.telemetry(kill=False, armed=False)
        self.assertTrue(h.p.command('reset', None, h.ros, h.wall).accepted)
        self.assertEqual(h.p.snapshot().state, 'DISABLED')

    def test_pose_outside_operational_envelope_rejects_engage(self):
        h = Harness(); h.telemetry(pose=Pose(2.01, 0, 0, 0))
        self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)
        self.assertIn('envelope', h.p._gate_reason(h.ros, h.wall))

    def test_target_validation_and_same_request_id_advancing_stamp(self):
        h = Harness(); h.finish_engage()
        bad = [('', Pose(.1, 0, 0, 0), 'odom', h.ros),
               ('a', Pose(math.nan, 0, 0, 0), 'odom', h.ros),
               ('a', Pose(.1, 0, 0, 0), 'map', h.ros),
               ('a', Pose(3, 0, 0, 0), 'odom', h.ros),
               ('a', Pose(.1, 0, 0, 0), 'odom', 0.),
               ('a', Pose(.1, 0, 0, 0), 'odom', h.ros + .1)]
        for request_id, pose, frame, stamp in bad:
            self.assertFalse(h.p.accept_target(request_id, pose, frame, stamp,
                                                h.ros, h.wall).accepted)
        h.advance(.11); h.telemetry(fcu_mode='OFFBOARD')
        self.assertTrue(h.p.accept_target('goal-1', Pose(1, 0, .2, 1.), 'odom',
                                          h.ros, h.ros, h.wall).accepted)
        self.assertFalse(h.p.accept_target('goal-1', Pose(1, 0, .2, 1.), 'odom',
                                           h.ros, h.ros, h.wall).accepted)
        h.refresh(fcu_mode='OFFBOARD')
        self.assertTrue(h.p.accept_target('goal-1', Pose(1.1, 0, .2, 1.1), 'odom',
                                          h.ros, h.ros, h.wall).accepted)
        self.assertEqual(h.p.snapshot().request_id, 'goal-1')

    def test_position_lead_speed_and_yaw_progress_are_bounded(self):
        h = Harness(); h.finish_engage()
        h.p.accept_target('g', Pose(2, 2, 2, math.pi), 'odom', h.ros, h.ros, h.wall)
        h.refresh(fcu_mode='OFFBOARD')
        action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
        self.assertLessEqual(math.hypot(action.pose.x, action.pose.y), .6 * .05 + 1e-9)
        self.assertLessEqual(action.pose.z, .4 * .05 + 1e-9)
        self.assertLessEqual(abs(action.pose.yaw), .5 * .05 + 1e-9)
        for _ in range(30):
            h.pose = Pose(.05, .05, .05, 0.)
            h.refresh(fcu_mode='OFFBOARD', pose=h.pose)
            action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
            self.assertLessEqual(math.hypot(action.pose.x-h.pose.x, action.pose.y-h.pose.y), .300001)
            self.assertLessEqual(abs(action.pose.z-h.pose.z), .300001)

    def test_planner_timeout_holds_actual_then_requests_posctl_once(self):
        h = Harness(); h.finish_engage()
        h.p.accept_target('g', Pose(.2, 0, .2, 0), 'odom', h.ros, h.ros, h.wall)
        h.advance(.41); h.telemetry(fcu_mode='OFFBOARD', pose=Pose(.1, 0, .1, 0))
        action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
        self.assertEqual(action.pose, Pose(.1, 0, .1, 0))
        self.assertEqual(h.p.snapshot().state, 'HOLD_GRACE')
        h.advance(2.01); h.telemetry(fcu_mode='OFFBOARD', pose=Pose(.1, 0, .1, 0))
        actions = h.p.tick(h.ros, h.wall)
        self.assertEqual([a.mode for a in actions if a.kind == 'mode'], ['POSCTL'])
        self.assertFalse(any(a.kind == 'setpoint' for a in actions))
        self.assertEqual(h.p.tick(h.ros, h.wall), ())

    def test_explicit_hold_captures_actual_and_has_no_grace_timeout(self):
        h = Harness(); h.finish_engage()
        h.pose = Pose(.2, -.1, .4, .2); h.refresh(rc_mode='hold', fcu_mode='OFFBOARD', pose=h.pose)
        action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
        self.assertEqual(action.pose, h.pose)
        captured = h.pose
        h.advance(2.1)
        moved = Pose(.25, -.1, .4, .2)
        h.telemetry(rc_mode='hold', fcu_mode='OFFBOARD', pose=moved)
        action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
        self.assertEqual(action.pose, captured)

    def test_absolute_takeoff_preserves_xy_yaw_and_uses_actual_completion(self):
        h = Harness(); h.pose = Pose(.4, -.2, 1., .3); h.finish_engage()
        result = h.p.command('takeoff', Pose(1., 1., 1.5, 2.), h.ros, h.wall)
        self.assertTrue(result.accepted)
        h.refresh(fcu_mode='OFFBOARD', landed=1, pose=h.pose)
        action = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'setpoint')
        self.assertEqual((action.pose.x, action.pose.y, action.pose.yaw), (.4, -.2, .3))
        self.assertLessEqual(action.pose.z - 1., .4 * .05 + 1e-9)
        # Follow the bounded climb; teleporting directly by .5 m exceeds the
        # final lead limit and must revoke control instead of proving arrival.
        for _ in range(25):
            h.refresh(fcu_mode='OFFBOARD', landed=2, pose=action.pose)
            action = h.p.tick(h.ros, h.wall)[0]
        for _ in range(7):
            h.refresh(fcu_mode='OFFBOARD', landed=2, pose=Pose(.4, -.2, 1.5, .3))
            h.p.tick(h.ros, h.wall)
        self.assertEqual(h.p.snapshot().state, 'HOLD')

    def test_active_commands_require_current_offboard_feedback(self):
        h = Harness(); h.finish_engage(); h.refresh(fcu_mode='POSCTL')
        for command in ('hold', 'release', 'takeoff', 'land'):
            target = Pose(0, 0, 1, 0) if command == 'takeoff' else None
            self.assertFalse(h.p.command(command, target, h.ros, h.wall).accepted)

    def test_takeoff_rejects_nonabsolute_not_above_or_not_on_ground_and_times_out(self):
        h = Harness(); h.finish_engage()
        for target in (Pose(0, 0, .1, 0), Pose(0, 0, 4., 0), Pose(0, 0, math.nan, 0)):
            self.assertFalse(h.p.command('takeoff', target, h.ros, h.wall).accepted)
        h.refresh(fcu_mode='OFFBOARD', landed=2)
        self.assertFalse(h.p.command('takeoff', Pose(0, 0, 1., 0), h.ros, h.wall).accepted)
        h.refresh(fcu_mode='OFFBOARD', landed=1)
        self.assertTrue(h.p.command('takeoff', Pose(0, 0, 1., 0), h.ros, h.wall).accepted)
        h.advance(5.01); h.telemetry(fcu_mode='OFFBOARD', landed=2)
        actions = h.p.tick(h.ros, h.wall)
        self.assertFalse(any(a.kind == 'setpoint' for a in actions))
        self.assertEqual([a.mode for a in actions if a.kind == 'mode'], ['POSCTL'])
        self.assertEqual(h.p.snapshot().state, 'RELEASING')

    def test_land_requests_auto_land_once_and_requires_feedback_and_disarm(self):
        h = Harness(); h.finish_engage()
        self.assertTrue(h.p.command('land', None, h.ros, h.wall).accepted)
        actions = h.p.tick(h.ros, h.wall)
        self.assertTrue(any(a.kind == 'setpoint' for a in actions))
        request = next(a for a in actions if a.kind == 'mode')
        self.assertEqual(request.mode, 'AUTO.LAND')
        waiting = h.p.tick(h.ros, h.wall)
        self.assertTrue(any(a.kind == 'setpoint' for a in waiting))
        self.assertFalse(any(a.kind == 'mode' for a in waiting))
        h.p.mode_result(request.token, True, h.wall)
        h.refresh(fcu_mode='AUTO.LAND', landed=1, armed=True)
        h.p.tick(h.ros, h.wall)
        self.assertEqual(h.p.snapshot().state, 'LANDING')
        h.refresh(fcu_mode='AUTO.LAND', landed=1, armed=False)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.snapshot().state, 'DISABLED')

    def test_land_rejection_and_timeout_are_visible_without_retry(self):
        h = Harness(); h.finish_engage(); h.p.command('land', None, h.ros, h.wall)
        request = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode')
        h.p.mode_result(request.token, False, h.wall)
        self.assertEqual(h.p.snapshot().state, 'DISABLED')
        self.assertIn('rejected', h.p.snapshot().reason)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())

        h = Harness(); h.finish_engage(); h.p.command('land', None, h.ros, h.wall)
        request = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode')
        h.p.mode_result(request.token, True, h.wall)
        h.advance(8.01); h.telemetry(fcu_mode='AUTO.LAND', landed=2)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertIn('timeout', h.p.snapshot().reason)

    def test_ch6_rising_is_baselined_and_can_land_only_while_active(self):
        h = Harness(); h.finish_engage()
        h.refresh(fcu_mode='OFFBOARD', land=False); h.p.tick(h.ros, h.wall)
        h.refresh(fcu_mode='OFFBOARD', land=True)
        actions = h.p.tick(h.ros, h.wall)
        self.assertEqual([a.mode for a in actions if a.kind == 'mode'], ['AUTO.LAND'])

    def test_configuration_rejects_nonfinite_ranges_and_unsafe_rates(self):
        for key, value in [('telemetry_timeout', 0), ('max_xy_speed', math.nan),
                           ('max_lead', 0), ('min_x', 3), ('frame_id', '')]:
            with self.assertRaises(ValueError): config(**{key: value})

    def test_rc_hold_interrupts_takeoff_and_captures_actual_once(self):
        h = Harness(); h.finish_engage()
        self.assertTrue(h.p.command('takeoff', Pose(0, 0, 1, 0), h.ros, h.wall).accepted)
        h.refresh(fcu_mode='OFFBOARD', rc_mode='hold', pose=Pose(0, 0, .1, 0))
        actions = h.p.tick(h.ros, h.wall)
        self.assertEqual(h.p.state, 'HOLD')
        self.assertEqual(h.p.target, h.pose)
        self.assertEqual(actions[0].pose, h.pose)
        captured = h.pose
        h.refresh(fcu_mode='OFFBOARD', rc_mode='hold', pose=Pose(0, 0, .12, 0))
        self.assertEqual(h.p.tick(h.ros, h.wall)[0].pose, captured)
        self.assertEqual(h.p.target, captured)

    def test_takeoff_requires_rc_command_instead_of_already_hold(self):
        h = Harness(); h.finish_engage()
        h.refresh(fcu_mode='OFFBOARD', rc_mode='hold')
        h.p.tick(h.ros, h.wall)
        result = h.p.command('takeoff', Pose(0, 0, 1, 0), h.ros, h.wall)
        self.assertFalse(result.accepted)
        self.assertIn('RC command', result.message)
        self.assertEqual(h.p.state, 'HOLD')

    def test_rc_hold_cancels_captured_takeoff_action_before_watchdog(self):
        h = Harness(); h.finish_engage()
        h.p.command('takeoff', Pose(0, 0, 1, 0), h.ros, h.wall)
        h.refresh(fcu_mode='OFFBOARD')
        action = h.p.tick(h.ros, h.wall)[0]
        h.refresh(fcu_mode='OFFBOARD', rc_mode='hold')
        self.assertFalse(h.p.action_current(action, h.ros, h.wall))

    def test_final_lead_violation_revokes_output_instead_of_breaking_slew(self):
        h = Harness(); h.finish_engage()
        for _ in range(6):
            h.refresh(fcu_mode='OFFBOARD')
            h.p.accept_target('g', Pose(2, 0, 0, 0), 'odom', h.ros, h.ros, h.wall)
            h.p.tick(h.ros, h.wall)
        h.refresh(fcu_mode='OFFBOARD', pose=Pose(-.3, 0, 0, 0))
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.state, 'DISABLED')
        self.assertIn('lead', h.p.reason)
        self.assertFalse(h.p.snapshot().ready)
        self.assertEqual(h.p.request_id, '')

    def test_dispatch_rechecks_final_lead_after_new_pose_callback(self):
        h = Harness(); h.finish_engage()
        action = h.p.tick(h.ros, h.wall)[0]
        h.refresh(fcu_mode='OFFBOARD', pose=Pose(-.31, 0, 0, 0))
        self.assertFalse(h.p.action_current(action, h.ros, h.wall))
        self.assertEqual(h.p.state, 'DISABLED')

    def test_overdue_dispatched_mode_response_latches_before_watchdog(self):
        for command, mode in [('engage', 'OFFBOARD'), ('land', 'AUTO.LAND'),
                              ('release', 'POSCTL')]:
            for accepted in (True, False):
                with self.subTest(command=command, accepted=accepted):
                    h = Harness()
                    if command == 'engage':
                        h.telemetry(); h.engage()
                        request = None
                        while request is None:
                            request = next((a for a in h.p.tick(h.ros, h.wall)
                                            if a.kind == 'mode'), None)
                            if request is None: h.refresh()
                    else:
                        h.finish_engage()
                        h.p.command(command, None, h.ros, h.wall)
                        request = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode')
                    self.assertTrue(h.p.mode_dispatched(request.token, h.wall))
                    h.advance(h.p.config.mode_request_timeout + .01)
                    h.telemetry(fcu_mode=mode)
                    self.assertFalse(h.p.mode_result(request.token, accepted, h.wall))
                    self.assertEqual(h.p.state, 'INHIBITED')
                    self.assertIsNone(h.p._mode_in_flight_token)
                    self.assertEqual(h.p.tick(h.ros, h.wall), ())
                    self.assertFalse(h.p.command('engage', None, h.ros, h.wall).accepted)
                    h.refresh(armed=False)
                    self.assertTrue(h.p.command('reset', None, h.ros, h.wall).accepted)

    def test_overdue_queued_mode_cannot_dispatch_before_watchdog(self):
        h = Harness(); h.finish_engage()
        h.p.command('land', None, h.ros, h.wall)
        request = next(a for a in h.p.tick(h.ros, h.wall) if a.kind == 'mode')
        h.advance(h.p.config.mode_request_timeout + .01)
        h.telemetry(fcu_mode='OFFBOARD')
        self.assertFalse(h.p.action_current(request, h.ros, h.wall))
        self.assertFalse(h.p.mode_dispatched(request.token, h.wall))


if __name__ == '__main__':
    unittest.main()
