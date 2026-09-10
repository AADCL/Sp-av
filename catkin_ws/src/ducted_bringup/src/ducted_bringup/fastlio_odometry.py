#!/usr/bin/env python3
import math
from dataclasses import dataclass


class AdapterError(ValueError):
    pass


@dataclass(frozen=True)
class AdaptedOdometry:
    position: tuple
    child_orientation: tuple
    linear_velocity_base: tuple
    angular_velocity_base: tuple


def _finite(values, name):
    values = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in values):
        raise AdapterError("{} contains a non-finite value".format(name))
    return values


def _normalize_quaternion(quaternion, name="quaternion"):
    quaternion = _finite(quaternion, name)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-9:
        raise AdapterError("{} has zero norm".format(name))
    return tuple(value / norm for value in quaternion)


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return _normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(quaternion):
    x, y, z, w = _normalize_quaternion(quaternion)
    return (-x, -y, -z, w)


def rotate_vector(quaternion, vector):
    x, y, z, w = _normalize_quaternion(quaternion)
    vx, vy, vz = _finite(vector, "vector")
    # Unit-quaternion rotation without constructing temporary quaternions.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _cross(left, right):
    ax, ay, az = left
    bx, by, bz = right
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)


def adapt_odometry_values(
    position,
    orientation,
    linear_velocity_map,
    angular_velocity_sensor,
    base_to_sensor_translation,
    base_to_sensor_orientation,
):
    """Convert a map->sensor FAST-LIO state into REP-147 map->base odometry.

    FAST-LIO publishes linear velocity in map and angular velocity in its body
    (sensor) frame.  MAVROS odometry/out expects both twist vectors in the child
    frame; MAVROS then performs the ENU/FLU to NED/FRD conversion.
    """
    position = _finite(position, "position")
    orientation = _normalize_quaternion(orientation, "sensor orientation")
    linear_velocity_map = _finite(linear_velocity_map, "linear velocity")
    angular_velocity_sensor = _finite(angular_velocity_sensor, "angular velocity")
    translation = _finite(base_to_sensor_translation, "extrinsic translation")
    q_base_sensor = _normalize_quaternion(
        base_to_sensor_orientation, "extrinsic orientation"
    )

    q_map_base = _normalize_quaternion(
        quaternion_multiply(orientation, quaternion_inverse(q_base_sensor)),
        "base orientation",
    )
    sensor_offset_map = rotate_vector(q_map_base, translation)
    base_position = tuple(position[i] - sensor_offset_map[i] for i in range(3))

    angular_base = rotate_vector(q_base_sensor, angular_velocity_sensor)
    sensor_linear_base = rotate_vector(quaternion_inverse(q_map_base), linear_velocity_map)
    lever_velocity = _cross(angular_base, translation)
    base_linear = tuple(sensor_linear_base[i] - lever_velocity[i] for i in range(3))

    return AdaptedOdometry(
        position=base_position,
        child_orientation=q_map_base,
        linear_velocity_base=base_linear,
        angular_velocity_base=angular_base,
    )
