"""AG-TEST FPFH/RANSAC + point-to-plane ICP, adapted to Open3D's Python API.

Original algorithm: AADCL/AG-TEST, ground_air_localization/global_relocalizer_node.cpp.
Apache-2.0. Run in a separate process so manual input and deadlines remain responsive.
"""
import os
import traceback
import numpy as np


class RegistrationEngine:
    def __init__(self, map_file, config):
        import open3d as o3d
        self.o3d, self.cfg = o3d, config
        if not os.path.isabs(map_file) or not os.path.isfile(map_file):
            raise ValueError('map_file must name an existing absolute PCD path')
        cloud = o3d.io.read_point_cloud(map_file)
        self.coarse = self.prepare(np.asarray(cloud.points), config['coarse_voxel'])
        self.fine = self.prepare(np.asarray(cloud.points), config['fine_voxel'])
        if len(self.coarse.points) < 30 or len(self.fine.points) < config['min_scan_points']:
            raise ValueError('reference map contains too few usable points')
        self.feature = self.features(self.coarse)

    def prepare(self, xyz, voxel):
        points = np.asarray(xyz, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3: raise ValueError('invalid point array')
        points = points[np.isfinite(points).all(axis=1)]
        cloud = self.o3d.geometry.PointCloud(self.o3d.utility.Vector3dVector(points))
        cloud = cloud.voxel_down_sample(voxel)
        cloud.estimate_normals(self.o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2.5, max_nn=40))
        cloud.orient_normals_to_align_with_direction([0., 0., 1.])
        return cloud

    def features(self, cloud):
        return self.o3d.pipelines.registration.compute_fpfh_feature(cloud,
            self.o3d.geometry.KDTreeSearchParamHybrid(radius=self.cfg['coarse_voxel']*5, max_nn=100))

    def match(self, xyz, seed=None, random_seed=0):
        reg = self.o3d.pipelines.registration
        fine = self.prepare(xyz, self.cfg['fine_voxel'])
        if len(fine.points) < self.cfg['min_scan_points']: raise ValueError('too few scan points')
        if seed is None:
            # Each confirmation repeats the global solve on a NEW submap; never seed it with the previous match.
            coarse = self.prepare(xyz, self.cfg['coarse_voxel'])
            if len(coarse.points) < 30: raise ValueError('too few coarse scan points')
            self.o3d.utility.random.seed(int(random_seed) % 2147483647)
            distance = self.cfg['coarse_voxel']*1.5
            result = reg.registration_ransac_based_on_feature_matching(
                coarse, self.coarse, self.features(coarse), self.feature, True, distance,
                reg.TransformationEstimationPointToPoint(False), 4,
                [reg.CorrespondenceCheckerBasedOnEdgeLength(.90),
                 reg.CorrespondenceCheckerBasedOnDistance(distance)],
                reg.RANSACConvergenceCriteria(self.cfg['ransac_iterations'], .999))
            seed = result.transformation
        result = reg.registration_icp(fine, self.fine, self.cfg['fine_voxel']*2,
            seed, reg.TransformationEstimationPointToPlane(), reg.ICPConvergenceCriteria(1e-6, 1e-6, 60))
        checked = reg.evaluate_registration(fine, self.fine, self.cfg['fine_voxel']*2, result.transformation)
        return checked.transformation, checked.fitness, checked.inlier_rmse


def worker_main(map_file, config, requests, responses):
    os.environ['OMP_NUM_THREADS'] = str(config.get('worker_threads', 2))
    os.environ['OPENBLAS_NUM_THREADS'] = str(config.get('worker_threads', 2))
    try:
        engine = RegistrationEngine(map_file, config)
        display = np.asarray(engine.coarse.points)
        responses.put({'kind': 'loaded', 'points': display[:config.get('max_map_display_points', 200000)]})
        while True:
            job = requests.get()
            if job is None: return
            try:
                matrix, fitness, rmse = engine.match(job['points'], job['seed'], job['id'])
                responses.put({'kind': 'result', 'generation': job['generation'], 'id': job['id'],
                               'matrix': matrix, 'fitness': fitness, 'rmse': rmse})
            except Exception as error:
                responses.put({'kind': 'result', 'generation': job['generation'], 'id': job['id'],
                               'error': str(error)})
    except Exception:
        responses.put({'kind': 'fatal', 'error': traceback.format_exc(limit=4)})
