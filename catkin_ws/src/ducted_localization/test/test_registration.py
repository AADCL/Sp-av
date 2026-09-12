#!/usr/bin/env python3
"""Actual Open3D registration test; run with the localization environment."""
import pathlib
import sys
import tempfile
import unittest
import numpy as np
import open3d as o3d
import yaml
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/'src'))
from ducted_localization.registration import RegistrationEngine
from registration_fixture import fixture


class RegistrationTest(unittest.TestCase):
    def test_unknown_pose_and_manual_seed(self):
        config = yaml.safe_load((pathlib.Path(__file__).resolve().parents[1]/'config/global_relocalization.yaml').read_text())
        map_points, local_points, expected = fixture()
        with tempfile.TemporaryDirectory() as directory:
            path = str(pathlib.Path(directory)/'map.pcd')
            o3d.io.write_point_cloud(path, o3d.geometry.PointCloud(o3d.utility.Vector3dVector(map_points)))
            engine = RegistrationEngine(path, config)
            for seed in (None, expected.copy()):
                if seed is not None: seed[0, 3] += .10
                result, fitness, rmse = engine.match(local_points, seed, 19)
                self.assertGreater(fitness, .9)
                self.assertLess(rmse, .10)
                self.assertLess(np.linalg.norm(result[:3, 3]-expected[:3, 3]), .05)
                self.assertLess(np.linalg.norm(result[:3, :3]-expected[:3, :3]), .03)
                print('registration', 'AUTO' if seed is None else 'MANUAL', 'fitness', fitness, 'rmse', rmse)


if __name__ == '__main__': unittest.main()
