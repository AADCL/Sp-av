"""PX4 datum landing must never infer contact from an airborne plateau."""
import unittest
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_flight import Harness, Pose, config


class RelativeLandingTests(unittest.TestCase):
    def harness(self):
        h = Harness(height_mode='takeoff_relative', takeoff_reference_z=.3,
                    slow_land_speed=.2, slow_land_timeout=30.)
        h.pose = Pose(0, 0, .3, 0)
        h.finish_engage()
        h.refresh(fcu_mode='OFFBOARD', pose=Pose(0, 0, 1.3, 0), landed=2)
        h.p._last_output = h.pose
        self.assertTrue(h.p.command('slow_land', None, h.ros, h.wall).accepted)
        return h

    def test_no_terrain_required_and_target_slew_is_point_two(self):
        h = self.harness()
        previous = 1.3
        for _ in range(40):
            h.refresh(fcu_mode='OFFBOARD', pose=Pose(0, 0, previous, 0), landed=2)
            actions = h.p.tick(h.ros, h.wall)
            z = next(a.pose.z for a in actions if a.kind == 'setpoint')
            self.assertLessEqual(previous-z, .2*.05+1e-8)
            self.assertGreaterEqual(z, .15-1e-8)
            previous = z
        self.assertLess(previous, 1.15)

    def test_five_seconds_stationary_in_air_never_requests_land(self):
        h = self.harness()
        for _ in range(120):
            h.refresh(fcu_mode='OFFBOARD', landed=2)
            self.assertFalse(any(a.kind == 'mode' for a in h.p.tick(h.ros, h.wall)))
        self.assertNotIn(h.p.state, ('LANDING', 'LAND_MODE_WAIT'))

    def test_near_datum_ground_feedback_and_five_seconds_stable_confirm(self):
        h = self.harness()
        h.p._last_output = Pose(0, 0, .3, 0)
        requests = []
        for i in range(110):
            h.refresh(fcu_mode='OFFBOARD', pose=Pose(0, 0, .3, 0), landed=1)
            actions = h.p.tick(h.ros, h.wall)
            requests += [a for a in actions if a.kind == 'mode']
            if i < 99:
                self.assertFalse(requests)
            if requests:
                break
        self.assertEqual([r.mode for r in requests], ['AUTO.LAND'])

    def test_ground_feedback_at_flight_height_does_not_confirm(self):
        h = self.harness()
        for _ in range(120):
            h.refresh(fcu_mode='OFFBOARD', landed=1)
            self.assertFalse(any(a.kind == 'mode' for a in h.p.tick(h.ros, h.wall)))

    def test_below_descent_limit_holds_without_land(self):
        h = self.harness()
        h.p._last_output = Pose(0, 0, .05, 0)
        h.refresh(fcu_mode='OFFBOARD', pose=Pose(0, 0, .05, 0), landed=2)
        h.p.tick(h.ros, h.wall)
        self.assertEqual(h.p.state, 'HOLD')

    def test_missing_pose_does_not_count_towards_contact(self):
        h = self.harness()
        h.advance(.7)
        self.assertEqual(h.p.tick(h.ros, h.wall), ())
        self.assertEqual(h.p.state, 'DISABLED')

    def test_unsafe_reference_and_unknown_mode_rejected(self):
        for values in ({'height_mode':'ignore'},
                       {'height_mode':'takeoff_relative','takeoff_reference_z':float('nan')},
                       {'height_mode':'takeoff_relative','takeoff_reference_z':-20.}):
            with self.assertRaises(ValueError):
                config(**values)


if __name__ == '__main__':
    unittest.main()
