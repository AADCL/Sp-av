import unittest
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_runtime import runtime, Pose, Vec3


class RelativeHeightTests(unittest.TestCase):
    def ready(self):
        s = runtime(height_mode='takeoff_relative', reference_z=.3)
        s.set_goal('r', Pose(1, 0, 1.3, 0), 10.)
        s.accept_odom(10., 20., 'odom', 'base_link', Pose(0, 0, 1.3, 0), Vec3(0, 0, 0))
        s.accept_cloud(10., 20., 'odom', ((3., 0., 1.3),))
        s.accept_controller(10., 20., True)
        s.accept_readiness('base_ready', True, 10., 20.)
        s.accept_readiness('external_ready', True, 10., 20.)
        return s

    def test_reference_constraint_does_not_require_or_claim_measured_terrain(self):
        s = self.ready()
        token, snapshot, reason = s.snapshot(10.01, 20.01)
        self.assertEqual(reason, '')
        self.assertAlmostEqual(snapshot.terrain.agl, 1.)
        self.assertEqual(snapshot.terrain.source, 'TAKEOFF_DATUM')
        self.assertTrue(s.revalidate(token, 10.01, 20.01))

    def test_cloud_dropout_still_revokes_reference_mode(self):
        s = self.ready()
        self.assertIn('stale', s.snapshot(10.6, 20.6)[2])

    def test_new_obstacle_cloud_revokes_pending_output(self):
        s = self.ready()
        token, _, _ = s.snapshot(10.01, 20.01)
        s.accept_cloud(10.02, 20.02, 'odom', ((.1, 0., 1.3),))
        self.assertFalse(s.revalidate(token, 10.02, 20.02))

    def test_default_mode_still_requires_terrain(self):
        s = runtime()
        s.set_goal('r', Pose(1, 0, 1, 0), 10.)
        self.assertIsNone(s.snapshot(10.01, 20.01)[1])


if __name__ == '__main__':
    unittest.main()
