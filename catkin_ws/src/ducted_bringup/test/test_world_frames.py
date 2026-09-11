import math
from pathlib import Path
import sys
import unittest
import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'src'))


class FrameContractTest(unittest.TestCase):
    def test_mapping_has_local_frames_only(self):
        frames = yaml.safe_load((PKG / 'config/mapping_mid360.yaml').read_text())['frames']
        self.assertEqual(frames, {'map': 'camera_init', 'body': 'body'})

    def test_correction_composes_with_local_pose(self):
        from ducted_bringup.world_frames import correction
        from ducted_bringup.fastlio_odometry import rotate_vector, quaternion_multiply
        local_p, local_q = (2., -1., .4), (0., 0., 0., 1.)
        global_p, global_q = (8., 3., 1.), (0., 0., math.sqrt(.5), math.sqrt(.5))
        p, q = correction(global_p, global_q, local_p, local_q)
        actual = tuple(a+b for a,b in zip(p, rotate_vector(q,local_p)))
        for a,b in zip(actual,global_p):self.assertAlmostEqual(a,b)
        for a,b in zip(quaternion_multiply(q,local_q),global_q):self.assertAlmostEqual(a,b)

    def test_nonfinite_correction_rejected(self):
        from ducted_bringup.world_frames import correction
        with self.assertRaises(ValueError):
            correction((float('nan'),0,0),(0,0,0,1),(0,0,0),(0,0,0,1))


if __name__ == '__main__':unittest.main()
