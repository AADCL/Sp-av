#!/usr/bin/env python3
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
try:
    from ducted_localization.core import Session, pose_matrix, initial_correction, valid_transform
except ImportError:
    Session = None


class LocalizationTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(Session, 'global/manual localization policy is not implemented')

    def test_manual_pose_is_airframe_center_with_tilted_mount(self):
        # Existing mount: base->body. A manual map->base pose must not be treated as map->body.
        base_body = pose_matrix([.13, 0, 0], [0, np.sin(.23), 0, np.cos(.23)])
        local_body = pose_matrix([2, -1, .3], [0, 0, np.sin(.4), np.cos(.4)])
        map_base = pose_matrix([5, 3, .1], [0, 0, 0, 1])
        correction = initial_correction(map_base, local_body, base_body)
        np.testing.assert_allclose(correction @ local_body @ np.linalg.inv(base_body), map_base, atol=1e-10)

    def test_bad_quaternion_and_reflection_are_rejected(self):
        with self.assertRaises(ValueError): pose_matrix([0, 0, 0], [0, 0, 0, 0])
        with self.assertRaises(ValueError): pose_matrix([np.nan, 0, 0], [0, 0, 0, 1])
        bad = np.eye(4); bad[0, 0] = -1
        self.assertFalse(valid_transform(bad))

    def test_independent_confirmations_and_quality(self):
        s = Session(); generation = s.begin('AUTO', 10.)
        self.assertFalse(s.accept(generation, 1, np.eye(4), .8, .05, 11.))
        self.assertFalse(s.accept(generation, 1, np.eye(4), .8, .05, 11.1))
        self.assertTrue(s.accept(generation, 2, np.eye(4), .8, .05, 12.))
        self.assertEqual(s.state, 'TRACKING')
        self.assertTrue(s.fresh(12.5))

    def test_manual_supersedes_inflight_auto_result(self):
        s = Session(); old = s.begin('AUTO', 10.)
        new = s.begin('MANUAL', 11., np.eye(4))
        self.assertFalse(s.accept(old, 1, np.eye(4), .9, .01, 11.2))
        self.assertEqual(s.confirmations, 0)
        self.assertFalse(s.accept(new, 2, np.eye(4), .9, .01, 11.3))
        self.assertTrue(s.accept(new, 3, np.eye(4), .9, .01, 11.4))

    def test_inconsistent_confirmation_restarts_count(self):
        s = Session(); g = s.begin('AUTO', 0.)
        s.accept(g, 1, np.eye(4), .9, .01, 1.)
        wrong = np.eye(4); wrong[0, 3] = 4.
        self.assertFalse(s.accept(g, 2, wrong, .9, .01, 2.))
        self.assertFalse(s.accept(g, 3, np.eye(4), .9, .01, 3.))
        self.assertTrue(s.accept(g, 4, np.eye(4), .9, .01, 4.))

    def test_tracking_loss_and_staleness_do_not_auto_restart(self):
        s = Session(); g = s.begin('AUTO', 0.)
        s.accept(g, 1, np.eye(4), .8, .01, 1.)
        s.accept(g, 2, np.eye(4), .8, .01, 2.)
        self.assertFalse(s.fresh(6.))
        for i in range(3): s.accept(g, i + 3, np.eye(4), .01, 2., 2.1 + i * .1)
        self.assertEqual(s.state, 'LOST')
        self.assertFalse(s.accept(g, 6, np.eye(4), .9, .01, 3.))

    def test_timeout_and_cancel_reject_late_result(self):
        s = Session(timeout=2.); g = s.begin('AUTO', 0.)
        s.accept(g, 1, np.eye(4), .9, .01, 1.)
        self.assertFalse(s.accept(g, 2, np.eye(4), .9, .01, 2.1))
        self.assertEqual(s.state, 'FAILED')
        g = s.begin('AUTO', 3.)
        s.invalidate('sensor clock reset')
        self.assertFalse(s.accept(g, 3, np.eye(4), .9, .01, 3.1))


if __name__ == '__main__': unittest.main()
