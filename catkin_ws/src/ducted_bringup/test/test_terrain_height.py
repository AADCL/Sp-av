#!/usr/bin/env python3
import math
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "src"))

try:
    from ducted_bringup.terrain_height import TerrainEstimator
except ImportError:
    TerrainEstimator = None

try:
    from ducted_bringup.terrain_height import lowest_body_z
except ImportError:
    lowest_body_z = None


class TerrainEstimatorAvailabilityTest(unittest.TestCase):
    def test_estimator_exists(self):
        self.assertIsNotNone(TerrainEstimator)

    def test_body_reference_helper_exists(self):
        self.assertIsNotNone(lowest_body_z)


@unittest.skipIf(TerrainEstimator is None, "terrain estimator is not implemented")
class TerrainEstimatorTest(unittest.TestCase):
    def setUp(self):
        self.estimator = TerrainEstimator(
            radius=1.0,
            ground_quantile=0.25,
            ground_band=0.12,
            min_points=6,
            max_below=5.0,
            max_above=0.1,
            min_agl=0.0,
            max_agl=4.0,
            max_variance=0.02,
        )

    @staticmethod
    def grid(z_function, center_x=0.0, center_y=0.0, spacing=0.2):
        return [
            (center_x + spacing * ix, center_y + spacing * iy,
             z_function(center_x + spacing * ix, center_y + spacing * iy))
            for ix in range(-2, 3)
            for iy in range(-2, 3)
        ]

    def test_flat_ground_returns_reference_agl_in_output_frame(self):
        points = self.grid(lambda x, y: 2.0 + 0.005 * ((round(10 * x) + round(10 * y)) % 3 - 1))

        result = self.estimator.estimate(points, sensor_x=0.0, sensor_y=0.0, sensor_z=3.5)

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.ground_z, 2.0, places=2)
        self.assertAlmostEqual(result.agl, 1.5, places=2)
        self.assertLess(result.variance, 0.001)

    def test_gentle_slope_is_estimated_at_the_local_reference(self):
        points = self.grid(lambda x, y: 0.1 * x + 0.05 * y)

        result = self.estimator.estimate(
            points, sensor_x=0.0, sensor_y=0.0, sensor_z=1.5
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.ground_z, 0.0, places=2)
        self.assertAlmostEqual(result.agl, 1.5, places=2)

    def test_moderate_slope_reports_plane_height_at_offset_reference(self):
        reference_x, reference_y = 0.2, -0.1
        points = self.grid(
            lambda x, y: 0.3 + 0.45 * x - 0.2 * y,
            center_x=reference_x,
            center_y=reference_y,
        )

        result = self.estimator.estimate(
            points,
            sensor_x=reference_x,
            sensor_y=reference_y,
            sensor_z=1.5,
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.ground_z, 0.41, places=10)
        self.assertAlmostEqual(result.agl, 1.09, places=10)

    def test_excessive_slope_is_rejected_explicitly(self):
        points = self.grid(lambda x, y: 1.2 * x)

        result = self.estimator.estimate(
            points, sensor_x=0.0, sensor_y=0.0, sensor_z=1.5
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "ground slope exceeds configured limit")

    def test_step_edge_is_rejected_as_ambiguous_multi_level_terrain(self):
        for step_height in (0.3, 1.0):
            with self.subTest(step_height=step_height):
                points = [
                    (0.1 * ix, 0.1 * iy, 0.0 if ix < 0 else step_height)
                    for ix in range(-5, 6)
                    for iy in range(-4, 5)
                ]

                result = self.estimator.estimate(
                    points, sensor_x=0.0, sensor_y=0.0, sensor_z=1.5
                )

                self.assertFalse(result.valid)
                self.assertEqual(result.reason, "ambiguous multi-level terrain")

    def test_supported_step_layer_cannot_be_masked_by_obstacle_returns(self):
        step = [
            (0.1 * x, 0.1 * y, 0.0 if x + y < 1 else 0.3)
            for x in range(-6, 7)
            for y in range(-6, 7)
        ]
        extra = [(0.1 * x, 0.2, 1.2) for x in range(0, 6)]

        result = self.estimator.estimate(
            step + extra, sensor_x=0.1, sensor_y=0.1, sensor_z=1.5
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "ambiguous multi-level terrain")

    def test_one_sided_diagonal_support_does_not_surround_reference(self):
        points = [
            (0.1 * x, 0.1 * y, 0.0)
            for x in range(-5, 6)
            for y in range(-5, 6)
            if x + y >= 2
        ]

        result = self.estimator.estimate(
            points, sensor_x=0.0, sensor_y=0.0, sensor_z=1.5
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "insufficient terrain support at reference")

    def test_reference_well_on_each_step_side_resolves_local_surface(self):
        estimator = TerrainEstimator(
            radius=0.45,
            ground_quantile=0.25,
            ground_band=0.12,
            min_points=6,
            max_below=5.0,
            max_above=0.1,
            min_agl=0.0,
            max_agl=4.0,
            max_variance=0.02,
        )
        points = [
            (0.1 * ix, 0.1 * iy, 0.0 if ix < 0 else 0.3)
            for ix in range(-12, 13)
            for iy in range(-4, 5)
        ]

        lower = estimator.estimate(points, sensor_x=-0.7, sensor_y=0.0, sensor_z=1.5)
        upper = estimator.estimate(points, sensor_x=0.7, sensor_y=0.0, sensor_z=1.5)

        self.assertTrue(lower.valid)
        self.assertAlmostEqual(lower.ground_z, 0.0)
        self.assertAlmostEqual(lower.agl, 1.5)
        self.assertTrue(upper.valid)
        self.assertAlmostEqual(upper.ground_z, 0.3)
        self.assertAlmostEqual(upper.agl, 1.2)

    def test_obstacles_distant_points_and_low_outlier_do_not_move_ground(self):
        ground = self.grid(lambda x, y: 0.005 * ((round(10 * x) + round(10 * y)) % 3 - 1))
        low_obstacle = [(0.1 * i, 0.2, 0.3) for i in range(-3, 4)]
        high_obstacle = [(0.1 * i, -0.2, 1.2) for i in range(-3, 4)]
        points = ground + low_obstacle + high_obstacle + [
            (2.0, 0.0, -2.0),
            (0.0, 0.0, -4.5),
        ]

        result = self.estimator.estimate(points, sensor_x=0.0, sensor_y=0.0, sensor_z=1.5)

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.ground_z, 0.0, places=2)
        self.assertAlmostEqual(result.agl, 1.5, places=2)

    def test_too_few_ground_points_is_invalid(self):
        result = self.estimator.estimate(
            [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)],
            sensor_x=0.0,
            sensor_y=0.0,
            sensor_z=1.0,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "insufficient candidate points")
        self.assertTrue(math.isnan(result.agl))

    def test_implausible_clearance_is_invalid(self):
        points = self.grid(lambda _x, _y: 0.0)

        result = self.estimator.estimate(points, sensor_x=0.0, sensor_y=0.0, sensor_z=5.0)

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "AGL outside configured limits")

    def test_nonfinite_reference_position_is_invalid(self):
        result = self.estimator.estimate(
            [(0.05 * i, 0.0, 0.0) for i in range(-4, 5)],
            sensor_x=math.nan,
            sensor_y=0.0,
            sensor_z=1.0,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "reference position is non-finite")

    def test_all_numeric_configuration_is_validated(self):
        defaults = dict(
            radius=1.0,
            ground_quantile=0.25,
            ground_band=0.12,
            min_points=6,
            max_below=5.0,
            max_above=0.1,
            min_agl=0.0,
            max_agl=4.0,
            max_variance=0.02,
        )
        bad_values = (
            ("radius", math.nan),
            ("ground_quantile", math.inf),
            ("ground_band", 0.0),
            ("min_points", 0),
            ("max_below", -0.1),
            ("max_above", math.nan),
            ("min_agl", 5.0),
            ("max_agl", -1.0),
            ("max_variance", -0.1),
        )
        for name, value in bad_values:
            with self.subTest(name=name, value=value):
                values = dict(defaults)
                values[name] = value
                with self.assertRaises(ValueError):
                    TerrainEstimator(**values)


@unittest.skipIf(lowest_body_z is None, "body geometry helper is not implemented")
class BodyReferenceTest(unittest.TestCase):
    def test_body_tilt_changes_lowest_world_z(self):
        half_angle = math.pi / 8.0
        pitch_45 = (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))
        vertices = [(-0.5, 0.0, -0.2), (0.5, 0.0, -0.2)]

        level = lowest_body_z(vertices, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
        tilted = lowest_body_z(vertices, (0.0, 0.0, 1.0), pitch_45)

        self.assertAlmostEqual(level, 0.8)
        self.assertAlmostEqual(tilted, 1.0 - 0.7 / math.sqrt(2.0))

    def test_empty_or_nonfinite_body_vertices_are_rejected(self):
        with self.assertRaises(ValueError):
            lowest_body_z([], (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
        with self.assertRaises(ValueError):
            lowest_body_z(
                [(0.0, math.nan, 0.0)],
                (0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0, 1.0),
            )


class TerrainLaunchContractTest(unittest.TestCase):
    def test_launch_is_optional_gated_and_uses_stable_topics(self):
        root = ET.parse(PKG / "launch" / "terrain_height.launch").getroot()
        node = root.find(".//node[@type='terrain_height_node.py']")
        self.assertIsNotNone(node)
        self.assertEqual(node.attrib.get("required"), "true")
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
        remaps = {r.attrib["from"]: r.attrib["to"] for r in node.findall("remap")}
        params = {p.attrib["name"]: p.attrib.get("value") for p in node.findall("param")}
        self.assertEqual(args["base_ready_topic"], "/ducted/system/ready")
        self.assertEqual(args["odom_topic"], "/ducted/localization/odom")
        self.assertEqual(args["cloud_topic"], "/ducted/relocalization/registered_scan")
        self.assertEqual(args["reference_confirmed"], "false")
        self.assertEqual(params["require_base_ready"], "true")
        self.assertEqual(params["reference_confirmed"], "$(arg reference_confirmed)")
        self.assertEqual(remaps["height"], "/ducted/terrain/height")
        self.assertEqual(remaps["ready"], "/ducted/terrain/ready")

    def test_config_declares_gravity_frame_body_reference_and_staleness(self):
        config = yaml.safe_load((PKG / "config" / "terrain_height.yaml").read_text())
        self.assertEqual(config["frames"]["map"], "map")
        self.assertEqual(config["frames"]["output"], "odom")
        self.assertEqual(config["frames"]["base"], "base_link")
        self.assertEqual(config["agl_reference"], "base_link")
        self.assertFalse(config["reference_confirmed"])
        self.assertGreater(config["terrain"]["max_slope"], 0.0)
        self.assertGreater(config["terrain"]["min_planar_variance"], 0.0)
        self.assertGreater(config["terrain"]["min_support_extent"], 0.0)
        self.assertGreater(config["terrain"]["max_step_height"], 0.0)
        self.assertGreater(config["timeouts"]["odom"], 0.0)
        self.assertGreater(config["timeouts"]["cloud"], 0.0)
        self.assertGreater(config["timeouts"]["base_ready"], 0.0)


if __name__ == "__main__":
    unittest.main()
