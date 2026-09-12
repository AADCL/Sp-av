"""Asymmetric room surfaces, independent of the registration implementation."""
import numpy as np


def fixture():
    rng = np.random.RandomState(413)
    planes = []
    for normal, origin, size in [
        (2, (0, 0, 0), (8, 6, 0)), (0, (0, 0, 0), (0, 6, 3)),
        (1, (0, 0, 0), (8, 0, 3)), (0, (8, 2, 0), (0, 4, 2.7)),
        (1, (2, 6, 0), (6, 0, 3)), (2, (1, 1, .9), (2, 1, 0)),
        (0, (3, 1, 0), (0, 1, .9)), (1, (1, 2, 0), (2, 0, .9)),
        (0, (5.3, 3.2, 0), (0, .8, 2.2)), (1, (5.3, 3.2, 0), (.7, 0, 2.2)),
        (2, (5.3, 3.2, 2.2), (.7, .8, 0))]:
        p = rng.uniform(size=(1800, 3))*np.asarray(size) + np.asarray(origin)
        p[:, normal] += rng.normal(0, .003, len(p))
        planes.append(p)
    map_points = np.concatenate(planes)
    angle = 1.1
    transform = np.array([[np.cos(angle), -np.sin(angle), 0, 4.],
                          [np.sin(angle), np.cos(angle), 0, -2.], [0, 0, 1, .7], [0, 0, 0, 1.]])
    local_points = (map_points-transform[:3, 3]) @ transform[:3, :3]
    return map_points, local_points, transform
