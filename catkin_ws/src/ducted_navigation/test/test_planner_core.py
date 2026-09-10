"""Deterministic scene tests for the ROS-independent local planner."""
import math
import random
import pathlib
import sys
import unittest
import yaml
from unittest.mock import patch



PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "src"))

import ducted_navigation.planner as planner_module

from ducted_navigation.planner import (  # noqa: E402
    Hull,
    Planner,
    PlannerConfig,
    Pose,
    Snapshot,
    Terrain,
    Vec3,
)


TEST_HULL = Hull(
    vertices=(
        (-0.22, -0.18, -0.12), (-0.22, 0.18, -0.12),
        (0.22, -0.18, -0.12), (0.22, 0.18, -0.12),
        (-0.22, -0.18, 0.16), (-0.22, 0.18, 0.16),
        (0.22, -0.18, 0.16), (0.22, 0.18, 0.16),
    ),
    reference="base_link",
    declared_test_geometry=True,
)


def config(**changes):
    values = dict(
        geometry_confirmed=True,
        sector_count=72,
        lookahead=1.0,
        waypoint_step=0.5,
        obstacle_clearance=0.08,
        min_passage_width=0.62,
        sensor_horizon=2.0,
        max_speed=0.8,
        max_acceleration=1.0,
        braking_acceleration=1.0,
        min_agl=0.35,
        max_agl=2.5,
        floor_clearance=0.08,
        ceiling_clearance=0.08,
        goal_tolerance=0.12,
        progress_timeout=1.0,
        blocked_hold=0.3,
        obstacle_persistence=0.6,
        prediction_horizon=0.8,
    )
    values.update(changes)
    return PlannerConfig(**values)


def snapshot(points=(), stamp=1.0, current=Pose(0, 0, 1, 0),
             goal=Pose(2, 0, 1, 0), velocity=Vec3(0, 0, 0),
             terrain=Terrain(ground_z=0, agl=1, variance=0.001, valid=True)):
    return Snapshot(stamp, current, velocity, goal, tuple(points), terrain)


def wall_x(x, y0=-1.0, y1=1.0, z0=0.4, z1=1.6, spacing=0.08):
    points = []
    y = y0
    while y <= y1 + 1e-9:
        z = z0
        while z <= z1 + 1e-9:
            points.append((x, y, z))
            z += spacing
        y += spacing
    return points


class PlannerCoreTest(unittest.TestCase):
    def test_user_airframe_blocks_obstacles_inside_each_measured_extent(self):
        raw = yaml.safe_load((PKG.parent / 'ducted_bringup/config/airframe_geometry.yaml').read_text())
        hull = Hull(tuple(tuple(p) for p in raw['body_vertices']), 'base_link',
                    measurement_source=raw['geometry_measurement_source'])
        for obstacle in ((.29, 0, 1), (-.29, 0, 1), (0, .34, 1),
                         (0, -.34, 1), (0, 0, 1.09), (0, 0, .91)):
            result = Planner(config(), hull).plan(snapshot([obstacle]))
            self.assertEqual(result.state, 'BLOCKED', obstacle)
            self.assertFalse(result.publish_target)
        self.assertEqual(Planner(config(), hull).plan(snapshot()).state, 'CLEAR')

    @staticmethod
    def reference_voxels(points, size):
        cells = {}
        for point in points:
            key = tuple(int(math.floor(float(axis) / size)) for axis in point)
            total = cells.setdefault(key, [0., 0., 0., 0])
            for axis in range(3):
                total[axis] += float(point[axis])
            total[3] += 1
        return tuple(tuple(total[axis] / total[3] for axis in range(3))
                     for _, total in sorted(cells.items()))

    def test_vector_voxels_preserve_lexical_order_and_sequential_centroids(self):
        planner = Planner(config(), TEST_HULL)
        rng = random.Random(45)
        points = tuple(tuple(rng.randrange(-4, 5) * .08 + rng.uniform(0, .07)
                             for _ in range(3)) for _ in range(3000))
        points += ((-.08, 0., .08), (-.08 - 1e-14, 0., .08 + 1e-14)) * 500
        self.assertEqual(planner._voxelize(points), self.reference_voxels(points, .08))

    def test_large_finite_voxel_coordinates_keep_python_integer_fallback(self):
        planner = Planner(config(), TEST_HULL)
        points = ((1e20, -1e20, 1.), (-1e20, 1e20, 0.), (0., 0., 0.)) * 100
        self.assertEqual(planner._voxelize(points), self.reference_voxels(points, .08))

    def test_large_cloud_nonfinite_values_and_overflow_are_rejected(self):
        for bad in (math.nan, math.inf, 1e308):
            points = ((0., 0., 0.),) * 300 + ((bad, 0., 0.),)
            result = Planner(config(), TEST_HULL).plan(snapshot(points))
            self.assertEqual(result.state, "STALE_INPUT")
            self.assertFalse(result.publish_target)

    def test_batch_tracking_preserves_exact_nearest_ties_and_inclusive_boundary(self):
        planner = Planner(config(), TEST_HULL)
        generator = random.Random(173)
        previous = tuple(tuple(generator.uniform(-2, 2) for _ in range(3)) for _ in range(200))
        current = tuple(tuple(generator.uniform(-2, 2) for _ in range(3)) for _ in range(120))
        previous += ((-.6, 0., 0.), (.6, 0., 0.), (5., 5., 5.), (5., 5., 5.))
        current += ((0., 0., 0.), (5., 5., 5.), (20., 20., 20.))
        expected = []
        for point in current:
            ordinal = min(range(len(previous)), key=lambda i: (
                sum((point[axis] - previous[i][axis]) ** 2 for axis in range(3)), i))
            candidate = previous[ordinal]
            expected.append(candidate if sum((point[i] - candidate[i]) ** 2 for i in range(3)) <= .6 ** 2 else None)
        self.assertEqual(planner._nearest_previous_batch(current, previous), tuple(expected))
        self.assertEqual(planner._nearest_previous_batch(((0., 0., 0.),),
                         ((.6, 0., 0.), (-.6, 0., 0.))), ((.6, 0., 0.),))
        self.assertEqual(planner._nearest_previous_batch((), previous), ())
        self.assertEqual(planner._nearest_previous_batch(((0., 0., 0.),), ()), (None,))

    def test_batch_tracking_backends_preserve_near_ties(self):
        planner = Planner(config(), TEST_HULL)
        previous = ((.3 + 1e-14, 0., 0.), (-.3, 0., 0.), (.6 + 1e-12, 2., 0.))
        current = ((0., 0., 0.), (0., 2., 0.))
        for backend in (planner_module._CKDTree, None):
            with patch.object(planner_module, "_CKDTree", backend):
                self.assertEqual(planner._nearest_previous_batch(current, previous),
                                 ((-.3, 0., 0.), None))

    def test_batch_tracking_keeps_moving_scene_predictions_and_decisions(self):
        accelerated, reference = Planner(config(), TEST_HULL), Planner(config(), TEST_HULL)

        def brute(current, previous):
            result = []
            for point in current:
                candidate = min(previous, key=lambda old: sum((point[i] - old[i]) ** 2 for i in range(3)))
                result.append(candidate if sum((point[i] - candidate[i]) ** 2 for i in range(3)) <= .6 ** 2 else None)
            return tuple(result)

        reference._nearest_previous_batch = brute
        for index in range(4):
            points = [(0.6, -.45 + index * .1, 1.)] + wall_x(1.5, -.8, .8, 0, 0)
            observation = snapshot(points, stamp=1 + index * .1)
            self.assertEqual(accelerated._expanded_obstacles(observation), reference._expanded_obstacles(observation))
            self.assertEqual(accelerated.plan(observation), reference.plan(observation))

    def test_global_vertical_filter_keeps_floor_points_that_predict_into_path(self):
        planner = Planner(config(), TEST_HULL)
        planner.plan(snapshot(((.6, 0., 0.),), stamp=1.0))
        observation = snapshot(((.6, 0., .2),), stamp=1.1)
        expanded = planner._expanded_obstacles(observation)
        retained = planner._candidate_obstacles(observation, expanded)
        self.assertNotIn((.6, 0., .2), retained)
        self.assertTrue(any(.8 < point[2] < 1.3 for point in retained))
        self.assertNotEqual(planner.plan(observation).state, "CLEAR")

    def test_global_vertical_filter_preserves_full_plan_results(self):
        filtered, reference = Planner(config(), TEST_HULL), Planner(config(), TEST_HULL)
        reference._candidate_obstacles = lambda _snapshot, points: points
        floor = [(i * .1, j * .1, 0.) for i in range(-8, 9) for j in range(-8, 9)]
        for index in range(3):
            points = floor + [(x, y, 3.) for x, y, _ in floor] + [(0.7, -.3 + index * .1, 1.)]
            observation = snapshot(points, stamp=1 + .1 * index, goal=Pose(2, 0, 1.5, 0))
            self.assertEqual(filtered.plan(observation), reference.plan(observation))

    def test_expansion_matches_reference_at_prediction_speed_threshold(self):
        dt = .125
        for factor in (1 - 1e-12, 1., 1 + 1e-12):
            for direction in ((1., 0., 0.), (3 ** -.5,) * 3):
                planner = Planner(config(), TEST_HULL)
                planner._previous_stamp = 1.
                planner._previous_points = ((0., 0., 0.),)
                point = tuple(value * .5 * dt * factor for value in direction)
                velocity = tuple(value / dt for value in point)
                expected = [point, (0., 0., 0.)]
                if math.sqrt(sum(value * value for value in velocity)) >= .5:
                    for fraction in (.25, .5, .75, 1.):
                        expected.append(tuple(point[i] + velocity[i] * (fraction * .8) for i in range(3)))
                actual = planner._expanded_obstacles(snapshot((point,), stamp=1 + dt))
                self.assertEqual(actual, tuple(expected))

    def test_snapshot_motion_margin_protects_vertical_and_agl_clearance(self):
        planner = Planner(config(snapshot_motion_margin=.05), TEST_HULL)
        current = Pose(0, 0, 1, 0)
        ceiling = ((0., 0., 1 + TEST_HULL.top + .08 + .025),)
        self.assertLess(planner._segment_clearance(current, current, ceiling, .5), 0)
        agl = .35 - TEST_HULL.bottom + .08 + .025
        result = planner.plan(snapshot(terrain=Terrain(1-agl, agl, .001, True)))
        self.assertEqual(result.reason, 'current pose violates terrain or hull clearance')

    def test_snapshot_speed_margin_reserves_extra_stopping_distance(self):
        planner = Planner(config(snapshot_speed_margin=.5), TEST_HULL)
        result = planner.plan(snapshot(velocity=Vec3(1.5, 0, 0)))
        self.assertEqual(result.reason, 'stopping envelope exceeds observed sensor horizon')

    def test_spatial_tracking_matches_brute_force_with_ties_and_boundary(self):
        rng = random.Random(29)
        previous = tuple((rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-1, 2))
                         for _ in range(300)) + ((-.6, 0., 0.), (.6, 0., 0.))
        queries = [(rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-1, 2))
                   for _ in range(100)] + [(0., 0., 0.), (50., 0., 0.)]
        planner = Planner(config(), TEST_HULL)
        index = planner._tracking_index(previous)
        for point in queries:
            brute = min(enumerate(previous), key=lambda item: (
                sum((point[i] - item[1][i]) ** 2 for i in range(3)), item[0]))[1]
            expected = brute if sum((point[i] - brute[i]) ** 2 for i in range(3)) <= .6 ** 2 else None
            self.assertEqual(planner._nearest_previous(point, index), expected)
        index = planner._tracking_index(((-.6, 0., 0.), (.6, 0., 0.)))
        self.assertEqual(planner._nearest_previous((0., 0., 0.), index), (-.6, 0., 0.))

    def test_empty_space_progresses_directly_and_preserves_absolute_z(self):
        result = Planner(config(), TEST_HULL).plan(snapshot())
        self.assertEqual(result.state, "CLEAR")
        self.assertGreater(result.target.x, 0)
        self.assertAlmostEqual(result.target.y, 0)
        self.assertAlmostEqual(result.target.z, 1)

    def test_frontal_obstacle_selects_deterministic_side(self):
        points = wall_x(0.55, -0.18, 0.18)
        first = Planner(config(), TEST_HULL).plan(snapshot(points))
        second = Planner(config(), TEST_HULL).plan(snapshot(points))
        self.assertEqual(first.state, "AVOIDING")
        self.assertEqual(first.target, second.target)
        self.assertGreater(abs(first.heading), 0.2)

    def test_mirrored_scene_mirrors_steering(self):
        upper = wall_x(0.55, -0.05, 0.36)
        lower = [(x, -y, z) for x, y, z in upper]
        a = Planner(config(), TEST_HULL).plan(snapshot(upper))
        b = Planner(config(), TEST_HULL).plan(snapshot(lower))
        self.assertAlmostEqual(a.target.x, b.target.x, places=6)
        self.assertAlmostEqual(a.target.y, -b.target.y, places=6)

    def test_wide_corridor_is_stable(self):
        points = wall_x(0.7, -1.2, -0.48) + wall_x(0.7, 0.48, 1.2)
        planner = Planner(config(), TEST_HULL)
        headings = [planner.plan(snapshot(points, 1 + i * 0.1)).heading for i in range(5)]
        self.assertTrue(all(abs(h) < 1e-6 for h in headings))

    def test_narrow_doorway_blocks_but_just_wide_doorway_passes(self):
        narrow = wall_x(0.55, -1, -0.28) + wall_x(0.55, 0.28, 1)
        wide = wall_x(0.55, -1, -0.46) + wall_x(0.55, 0.46, 1)
        self.assertEqual(Planner(config(), TEST_HULL).plan(snapshot(narrow)).state, "BLOCKED")
        self.assertIn(Planner(config(), TEST_HULL).plan(snapshot(wide)).state,
                      ("CLEAR", "AVOIDING"))

    def test_fully_blocked_ring_blocks(self):
        points = [(0.42 * math.cos(a), 0.42 * math.sin(a), 1.0)
                  for a in [i * math.pi / 36 for i in range(72)]]
        self.assertEqual(Planner(config(), TEST_HULL).plan(snapshot(points)).state, "BLOCKED")

    def test_blocked_hold_is_bounded_while_fresh_snapshots_advance(self):
        planner = Planner(config(blocked_hold=0.25), TEST_HULL)
        points = [(0.42 * math.cos(a), 0.42 * math.sin(a), 1.0)
                  for a in [i * math.pi / 36 for i in range(72)]]
        first = planner.plan(snapshot(points, stamp=1.0))
        later = planner.plan(snapshot(points, stamp=1.3))
        self.assertTrue(first.hold_allowed)
        self.assertFalse(later.hold_allowed)
        self.assertFalse(later.publish_target)

    def test_no_progress_recovery_is_bounded_then_blocks(self):
        planner = Planner(config(progress_timeout=0.25), TEST_HULL)
        points = wall_x(0.5, -0.05, 0.05)
        states = [planner.plan(snapshot(points, 1 + 0.1 * i)).state for i in range(6)]
        self.assertIn("AVOIDING", states[:3])
        self.assertEqual(states[-1], "BLOCKED")

    def test_no_progress_timeout_also_applies_to_nominally_clear_route(self):
        planner = Planner(config(progress_timeout=0.2), TEST_HULL)
        self.assertEqual(planner.plan(snapshot(stamp=1.0)).state, "CLEAR")
        self.assertEqual(planner.plan(snapshot(stamp=1.3)).state, "BLOCKED")

    def test_moving_obstacle_crossing_route_is_persisted_and_predicted(self):
        planner = Planner(config(), TEST_HULL)
        planner.plan(snapshot([(0.6, -0.45, 1)], stamp=1.0))
        result = planner.plan(snapshot([(0.6, -0.18, 1)], stamp=1.2))
        self.assertNotEqual(result.state, "CLEAR")

    def test_sparse_and_dense_sampling_have_same_decision(self):
        sparse = wall_x(0.55, -0.2, 0.2, spacing=0.2)
        dense = wall_x(0.55, -0.2, 0.2, spacing=0.03)
        a = Planner(config(), TEST_HULL).plan(snapshot(sparse))
        b = Planner(config(), TEST_HULL).plan(snapshot(dense))
        self.assertEqual(a.state, b.state)
        self.assertAlmostEqual(a.heading, b.heading)

    def test_candidate_agl_uses_base_delta_without_replacing_absolute_z(self):
        goal = Pose(1, 0, 1.3, 0)
        terrain = Terrain(ground_z=0.1, agl=0.9, variance=0.001, valid=True)
        result = Planner(config(), TEST_HULL).plan(
            snapshot(goal=goal, current=Pose(0, 0, 1.0, 0), terrain=terrain))
        expected_agl = terrain.agl + result.target.z - 1.0
        self.assertAlmostEqual(result.candidate_agl, expected_agl)
        self.assertGreater(result.target.z, 1.0)

    def test_invalid_or_high_variance_terrain_is_stale(self):
        invalid = Terrain(0, 1, 0, False)
        noisy = Terrain(0, 1, 1.0, True)
        self.assertEqual(Planner(config(), TEST_HULL).plan(snapshot(terrain=invalid)).state,
                         "STALE_INPUT")
        self.assertEqual(Planner(config(max_terrain_variance=0.02), TEST_HULL).plan(
            snapshot(terrain=noisy)).state, "STALE_INPUT")

    def test_low_overhead_and_floor_returns_block_vertical_intersection(self):
        overhead = [(0.4, 0, 1.13)]
        floor = [(0.4, 0, 0.87)]
        self.assertEqual(Planner(config(), TEST_HULL).plan(snapshot(overhead)).state, "BLOCKED")
        self.assertEqual(Planner(config(), TEST_HULL).plan(snapshot(floor)).state, "BLOCKED")

    def test_nonintersecting_high_return_does_not_block(self):
        result = Planner(config(), TEST_HULL).plan(snapshot([(0.4, 0, 2.0)]))
        self.assertEqual(result.state, "CLEAR")

    def test_start_inside_inflation_and_goal_inside_obstacle_block(self):
        self.assertEqual(Planner(config(), TEST_HULL).plan(
            snapshot([(0.05, 0, 1)])).state, "BLOCKED")
        self.assertEqual(Planner(config(), TEST_HULL).plan(
            snapshot([(0.45, 0, 1)], goal=Pose(0.45, 0, 1, 0))).state, "BLOCKED")

    def test_angle_wrap_chooses_short_direction(self):
        current = Pose(0, 0, 1, math.pi - 0.01)
        goal = Pose(-2, -0.02, 1, -math.pi + 0.01)
        result = Planner(config(), TEST_HULL).plan(snapshot(current=current, goal=goal))
        self.assertLess(abs(abs(result.heading) - math.pi), 0.03)

    def test_hysteresis_prevents_alternating_steering(self):
        planner = Planner(config(), TEST_HULL)
        ys = []
        for index in range(6):
            offset = 0.01 if index % 2 else -0.01
            points = wall_x(0.55, -0.2 + offset, 0.2 + offset)
            ys.append(planner.plan(snapshot(points, 1 + 0.1 * index)).target.y)
        nonzero = [math.copysign(1, y) for y in ys if abs(y) > 1e-6]
        self.assertLessEqual(len(set(nonzero)), 1)

    def test_missing_geometry_inhibits_output(self):
        result = Planner(config(geometry_confirmed=False), TEST_HULL).plan(snapshot())
        self.assertEqual(result.state, "STALE_INPUT")
        self.assertFalse(result.publish_target)

    def test_goal_reached_has_terminal_state(self):
        result = Planner(config(), TEST_HULL).plan(
            snapshot(current=Pose(1, 2, 1.1, 0), goal=Pose(1.05, 2, 1.1, 0.2)))
        self.assertEqual(result.state, "GOAL_REACHED")
        self.assertGreater(result.target.x, 1.0)
        self.assertLessEqual(result.target.x, 1.01 + 1e-12)
        self.assertEqual((result.target.y, result.target.z, result.target.yaw), (2, 1.1, 0.2))
        self.assertTrue(result.publish_target)

    def test_near_goal_still_checks_goal_collision_and_agl(self):
        current = Pose(0, 0, 1, 0)
        goal = Pose(0.05, 0, 1, 0)
        result = Planner(config(), TEST_HULL).plan(
            snapshot([(0.05, 0, 1)], current=current, goal=goal))
        self.assertEqual(result.state, "BLOCKED")
        high = Planner(config(max_agl=1.1), TEST_HULL).plan(
            snapshot(current=current, goal=Pose(0, 0, 1.05, 0)))
        self.assertEqual(high.state, "BLOCKED")

    def test_speed_and_acceleration_bounds_apply(self):
        planner = Planner(config(max_speed=0.4, max_acceleration=0.5), TEST_HULL)
        first = planner.plan(snapshot(stamp=1.0))
        second = planner.plan(snapshot(stamp=1.2))
        self.assertLessEqual(first.speed, 0.4)
        self.assertLessEqual(second.speed - first.speed, 0.1 + 1e-9)
        self.assertLessEqual(math.hypot(first.target.x, first.target.y), first.speed * 0.1 + 1e-9)

    def test_pure_vertical_goal_progresses_with_vertical_speed_bound(self):
        planner = Planner(config(max_vertical_speed=0.3), TEST_HULL)
        result = planner.plan(snapshot(goal=Pose(0, 0, 1.8, 0)))
        self.assertGreater(result.target.z, 1.0)
        self.assertLessEqual(result.target.z - 1.0, 0.03 + 1e-9)

    def test_diagonal_sweep_requires_xy_and_z_collision_at_same_time(self):
        planner = Planner(config(max_vertical_speed=10), TEST_HULL)
        # At the closest XY point, this return is above the hull. The rising hull
        # intersects it later while still horizontally inflated.
        result = planner.plan(snapshot([(0.42, 0, 1.5)], goal=Pose(1, 0, 2, 0)))
        self.assertNotEqual(result.state, "CLEAR")

    def test_hull_requires_explicit_test_or_physical_declaration(self):
        with self.assertRaises(ValueError):
            Hull(vertices=TEST_HULL.vertices, reference="base_link")
        with self.assertRaises(ValueError):
            Hull(vertices=TEST_HULL.vertices, reference="body_bottom",
                 declared_test_geometry=True)

    def test_vertical_and_diagonal_commands_obey_total_speed_and_acceleration(self):
        for goal in (Pose(0, 0, 1.8, 0), Pose(2, 0, 1.8, 0)):
            planner = Planner(config(max_speed=0.1, max_acceleration=0.01), TEST_HULL)
            for index in range(3):
                result = planner.plan(snapshot(stamp=1 + index * 0.1, goal=goal))
                distance = math.sqrt(result.target.x ** 2 + result.target.y ** 2
                                     + (result.target.z - 1) ** 2)
                self.assertEqual(result.state, "CLEAR")
                self.assertLessEqual(distance, 0.1 * result.speed + 1e-12)
                self.assertLessEqual(result.speed, 0.001 * (index + 1) + 1e-12)

    def test_stopping_envelope_outside_observed_horizon_blocks_empty_cloud(self):
        planner = Planner(config(), TEST_HULL)
        result = planner.plan(snapshot(velocity=Vec3(3, 0, 0)))
        self.assertEqual(result.state, "BLOCKED")
        self.assertIn("horizon", result.reason)
        self.assertFalse(result.publish_target)

    def test_full_3d_candidate_and_hull_stay_inside_observed_horizon(self):
        planner = Planner(config(sensor_horizon=1.05), TEST_HULL)
        result = planner.plan(snapshot(goal=Pose(1, 0, 2, 0)))
        self.assertIn(result.state, ("CLEAR", "AVOIDING"))
        distance = math.sqrt(result.target.x ** 2 + result.target.y ** 2
                             + (result.target.z - 1) ** 2)
        self.assertLessEqual(distance + TEST_HULL.radius + 0.08, 1.05)

    def test_terminal_goal_stream_keeps_3d_motion_bounds(self):
        planner = Planner(config(max_speed=0.1, max_acceleration=0.01), TEST_HULL)
        result = planner.plan(snapshot(goal=Pose(0, 0, 1.05, 0)))
        self.assertEqual(result.state, "GOAL_REACHED")
        self.assertLessEqual(result.target.z - 1, 0.0001 + 1e-12)
        self.assertGreater(result.target.z, 1)

    def test_current_agl_outside_hull_envelope_blocks_before_correction(self):
        planner = Planner(config(max_vertical_speed=10), TEST_HULL)
        result = planner.plan(snapshot(goal=Pose(0, 0, 1.8, 0),
                                       terrain=Terrain(0.5, 0.5, 0.001, True)))
        self.assertEqual(result.state, "BLOCKED")
        self.assertFalse(result.publish_target)

    def test_heading_reversal_blocks_instead_of_exceeding_vector_acceleration(self):
        planner = Planner(config(max_acceleration=0.5, progress_timeout=10), TEST_HULL)
        for index in range(10):
            result = planner.plan(snapshot(stamp=1 + index * 0.1))
            self.assertEqual(result.state, "CLEAR")
        reversed_goal = Pose(-2, 0, 1, 0)
        result = planner.plan(snapshot(stamp=2.0, goal=reversed_goal))
        self.assertEqual(result.state, "BLOCKED")
        self.assertFalse(result.publish_target)

    def test_feasible_turn_bounds_change_in_full_command_velocity(self):
        planner = Planner(config(max_acceleration=0.5, progress_timeout=10), TEST_HULL)
        previous_velocity = (0, 0, 0)
        for index in range(10):
            goal = Pose(2, 0 if index < 5 else 0.4, 1.4, 0)
            result = planner.plan(snapshot(stamp=1 + index * 0.1, goal=goal))
            self.assertIn(result.state, ("CLEAR", "AVOIDING"))
            velocity = (result.target.x / 0.1, result.target.y / 0.1,
                        (result.target.z - 1) / 0.1)
            change = math.sqrt(sum((velocity[i] - previous_velocity[i]) ** 2
                                   for i in range(3)))
            self.assertLessEqual(change, 0.05 + 1e-10)
            previous_velocity = velocity


if __name__ == "__main__":
    unittest.main()
