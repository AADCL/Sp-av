"""Rigid correction: T_map_odom = T_map_base * inverse(T_odom_base)."""
import math
from ducted_bringup.fastlio_odometry import quaternion_inverse, quaternion_multiply, rotate_vector


def correction(global_position, global_orientation, local_position, local_orientation):
    if not all(math.isfinite(v) for v in tuple(global_position)+tuple(local_position)):
        raise ValueError('nonfinite position')
    q = quaternion_multiply(quaternion_inverse(quaternion_inverse(global_orientation)),
                            quaternion_inverse(local_orientation))
    rotated = rotate_vector(q, local_position)
    return tuple(a-b for a,b in zip(global_position,rotated)), q
