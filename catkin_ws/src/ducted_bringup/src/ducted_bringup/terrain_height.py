#!/usr/bin/env python3
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class TerrainEstimate:
    ground_z: float
    agl: float
    variance: float
    valid: bool
    reason: str = ""


class TerrainEstimator:
    """Estimate local ground height around a reference position."""

    def __init__(
        self,
        radius: float,
        ground_quantile: float,
        ground_band: float,
        min_points: int,
        max_below: float,
        max_above: float,
        min_agl: float,
        max_agl: float,
        max_variance: float,
        max_slope: float = 1.0,
        min_planar_variance: float = 0.0025,
        min_support_extent: float = 0.1,
        max_step_height: float = 5.0,
    ):
        numeric = (
            radius,
            ground_quantile,
            ground_band,
            max_below,
            max_above,
            min_agl,
            max_agl,
            max_variance,
            max_slope,
            min_planar_variance,
            min_support_extent,
            max_step_height,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("terrain configuration must be finite")
        if radius <= 0.0 or ground_band <= 0.0 or min_points < 1:
            raise ValueError("radius, ground_band and min_points must be positive")
        if isinstance(min_points, bool) or int(min_points) != min_points:
            raise ValueError("min_points must be an integer")
        if not 0.0 <= ground_quantile <= 1.0:
            raise ValueError("ground_quantile must be in [0, 1]")
        if max_below < 0.0 or max_above < 0.0:
            raise ValueError("max_below and max_above must be nonnegative")
        if min_agl > max_agl:
            raise ValueError("min_agl must not exceed max_agl")
        if max_variance < 0.0:
            raise ValueError("max_variance must be nonnegative")
        if max_slope <= 0.0:
            raise ValueError("max_slope must be positive")
        if min_planar_variance <= 0.0 or min_support_extent <= 0.0:
            raise ValueError(
                "min_planar_variance and min_support_extent must be positive"
            )
        if max_step_height <= ground_band:
            raise ValueError("max_step_height must exceed ground_band")
        self.radius = radius
        self.ground_quantile = ground_quantile
        self.ground_band = ground_band
        self.min_points = min_points
        self.max_below = max_below
        self.max_above = max_above
        self.min_agl = min_agl
        self.max_agl = max_agl
        self.max_variance = max_variance
        self.max_slope = max_slope
        self.min_planar_variance = min_planar_variance
        self.min_support_extent = min_support_extent
        self.max_step_height = max_step_height

    @staticmethod
    def _invalid(reason: str) -> TerrainEstimate:
        return TerrainEstimate(math.nan, math.nan, math.inf, False, reason)

    def estimate(
        self,
        points: Iterable[Sequence[float]],
        sensor_x: float,
        sensor_y: float,
        sensor_z: float,
    ) -> TerrainEstimate:
        if not all(math.isfinite(float(value)) for value in (sensor_x, sensor_y, sensor_z)):
            return self._invalid("reference position is non-finite")
        radius_sq = self.radius * self.radius
        candidates = []
        for point in points:
            if len(point) < 3:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if (x - sensor_x) ** 2 + (y - sensor_y) ** 2 > radius_sq:
                continue
            relative_z = z - sensor_z
            if -self.max_below <= relative_z <= self.max_above:
                candidates.append((x, y, z))

        if len(candidates) < self.min_points:
            return self._invalid("insufficient candidate points")

        candidates.sort(key=lambda point: point[2])
        quantile_index = int((len(candidates) - 1) * self.ground_quantile)
        quantile_z = candidates[quantile_index][2]
        inliers = [
            point
            for point in candidates
            if abs(point[2] - quantile_z) <= self.ground_band
        ]
        plane = (
            self._fit_plane(inliers, sensor_x, sensor_y)
            if len(inliers) >= self.min_points
            else None
        )
        if plane is None:
            # A steep clean plane may leave only a narrow vertical seed strip.
            # The all-candidate fallback is accepted only after residual checks.
            plane = self._fit_plane(candidates, sensor_x, sensor_y)
            if plane is None:
                return self._invalid("terrain points are poorly conditioned")

        for _iteration in range(2):
            inliers = [
                point
                for point in candidates
                if abs(self._plane_residual(plane, point, sensor_x, sensor_y))
                <= self.ground_band
            ]
            if len(inliers) < self.min_points:
                return self._invalid("insufficient ground-band points")
            fitted = self._fit_plane(inliers, sensor_x, sensor_y)
            if fitted is None:
                return self._invalid("terrain points are poorly conditioned")
            plane = fitted

        alternate = [
            (
                self._plane_residual(plane, point, sensor_x, sensor_y),
                point,
            )
            for point in candidates
            if self.ground_band
            < abs(self._plane_residual(plane, point, sensor_x, sensor_y))
            <= self.max_step_height
        ]
        if self._has_alternate_surface(alternate, sensor_x, sensor_y):
            return self._invalid("ambiguous multi-level terrain")

        slope = math.hypot(plane[0], plane[1])
        if slope > self.max_slope:
            return self._invalid("ground slope exceeds configured limit")
        if not self._supports_reference(inliers, sensor_x, sensor_y):
            return self._invalid("insufficient terrain support at reference")

        ground_z = plane[2]
        residuals = [
            self._plane_residual(plane, point, sensor_x, sensor_y)
            for point in inliers
        ]
        variance = sum(residual * residual for residual in residuals) / len(inliers)
        agl = sensor_z - ground_z
        if not self.min_agl <= agl <= self.max_agl:
            return self._invalid("AGL outside configured limits")
        if variance > self.max_variance:
            return self._invalid("ground variance exceeds configured limit")
        return TerrainEstimate(ground_z, agl, variance, True)

    @staticmethod
    def _plane_residual(plane, point, reference_x, reference_y):
        slope_x, slope_y, reference_z = plane
        predicted = (
            reference_z
            + slope_x * (point[0] - reference_x)
            + slope_y * (point[1] - reference_y)
        )
        return point[2] - predicted

    def _fit_plane(self, points, reference_x, reference_y):
        count = len(points)
        if count < self.min_points:
            return None
        mean_x = sum(point[0] for point in points) / count
        mean_y = sum(point[1] for point in points) / count
        mean_z = sum(point[2] for point in points) / count
        xx = sum((point[0] - mean_x) ** 2 for point in points) / count
        yy = sum((point[1] - mean_y) ** 2 for point in points) / count
        xy = sum(
            (point[0] - mean_x) * (point[1] - mean_y) for point in points
        ) / count
        xz = sum(
            (point[0] - mean_x) * (point[2] - mean_z) for point in points
        ) / count
        yz = sum(
            (point[1] - mean_y) * (point[2] - mean_z) for point in points
        ) / count
        discriminant = max(0.0, (xx - yy) ** 2 + 4.0 * xy * xy)
        minimum_eigenvalue = 0.5 * (xx + yy - math.sqrt(discriminant))
        determinant = xx * yy - xy * xy
        if minimum_eigenvalue < self.min_planar_variance or determinant <= 0.0:
            return None
        slope_x = (xz * yy - yz * xy) / determinant
        slope_y = (yz * xx - xz * xy) / determinant
        reference_z = (
            mean_z
            + slope_x * (reference_x - mean_x)
            + slope_y * (reference_y - mean_y)
        )
        values = (slope_x, slope_y, reference_z)
        return values if all(math.isfinite(value) for value in values) else None

    def _has_alternate_surface(self, residual_points, reference_x, reference_y):
        if len(residual_points) < self.min_points:
            return False
        residual_points.sort(key=lambda item: item[0])
        clusters = []
        current = []
        previous = None
        for residual, point in residual_points:
            if previous is not None and residual - previous > self.ground_band:
                clusters.append(current)
                current = []
            current.append(point)
            previous = residual
        clusters.append(current)

        all_points = [point for _residual, point in residual_points]
        for seed in clusters:
            if len(seed) < self.min_points:
                continue
            plane = self._fit_plane(seed, reference_x, reference_y)
            if plane is None:
                continue
            for _iteration in range(2):
                support = [
                    point
                    for point in all_points
                    if abs(
                        self._plane_residual(
                            plane, point, reference_x, reference_y
                        )
                    )
                    <= self.ground_band
                ]
                fitted = self._fit_plane(support, reference_x, reference_y)
                if fitted is None:
                    break
                plane = fitted
            else:
                residual_variance = sum(
                    self._plane_residual(
                        plane, point, reference_x, reference_y
                    ) ** 2
                    for point in support
                ) / len(support)
                if (
                    residual_variance <= self.max_variance
                    and math.hypot(plane[0], plane[1]) <= self.max_slope
                ):
                    return True
        return False

    @staticmethod
    def _convex_hull(points):
        xy = sorted(set((point[0], point[1]) for point in points))
        if len(xy) < 3:
            return []

        def cross(origin, left, right):
            return (
                (left[0] - origin[0]) * (right[1] - origin[1])
                - (left[1] - origin[1]) * (right[0] - origin[0])
            )

        lower = []
        for point in xy:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
                lower.pop()
            lower.append(point)
        upper = []
        for point in reversed(xy):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
                upper.pop()
            upper.append(point)
        return lower[:-1] + upper[:-1]

    def _supports_reference(self, points, reference_x, reference_y):
        hull = self._convex_hull(points)
        if len(hull) < 3:
            return False
        extent = self.min_support_extent
        for index, start in enumerate(hull):
            end = hull[(index + 1) % len(hull)]
            edge_x = end[0] - start[0]
            edge_y = end[1] - start[1]
            length = math.hypot(edge_x, edge_y)
            if length <= 0.0:
                return False
            signed_distance = (
                edge_x * (reference_y - start[1])
                - edge_y * (reference_x - start[0])
            ) / length
            if signed_distance < extent:
                return False
        return True


def _finite_tuple(values, expected_length, name):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError("{} must contain numeric values".format(name)) from error
    if len(result) != expected_length or not all(math.isfinite(value) for value in result):
        raise ValueError("{} must contain {} finite values".format(name, expected_length))
    return result


def normalize_quaternion(quaternion):
    quaternion = _finite_tuple(quaternion, 4, "quaternion")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-9:
        raise ValueError("quaternion has zero norm")
    return tuple(value / norm for value in quaternion)


def rotate_vector(quaternion, vector):
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = _finite_tuple(vector, 3, "vector")
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def transform_point(point, translation, quaternion):
    translation = _finite_tuple(translation, 3, "translation")
    rotated = rotate_vector(quaternion, point)
    return tuple(rotated[index] + translation[index] for index in range(3))


def validate_body_vertices(vertices):
    try:
        result = tuple(
            _finite_tuple(vertex, 3, "body vertex") for vertex in vertices
        )
    except TypeError as error:
        raise ValueError("body_vertices must be a sequence") from error
    if not result:
        raise ValueError("body_bottom requires at least one body vertex")
    return result


def lowest_body_z(vertices, base_position, base_orientation):
    """Return the lowest vertex Z after applying the measured base pose."""
    vertices = validate_body_vertices(vertices)
    base_position = _finite_tuple(base_position, 3, "base position")
    orientation = normalize_quaternion(base_orientation)
    return min(
        base_position[2] + rotate_vector(orientation, vertex)[2]
        for vertex in vertices
    )
