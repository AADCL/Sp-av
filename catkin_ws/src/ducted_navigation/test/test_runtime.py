"""Input lifetime, transform, and publish-token tests for navigation runtime."""
import math
import pathlib
import sys
import threading
import unittest


PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "src"))

from ducted_navigation.planner import Pose, Terrain, Vec3  # noqa: E402
from ducted_navigation.runtime import (  # noqa: E402
    NavigationRuntime,
    RuntimeConfig,
    lookup_exact,
    transform_points,
    transform_pose,
    transform_pose_full,
)


class TransformUnavailable(Exception):
    pass


def runtime(**changes):
    values = dict(source_timeout=0.5, arrival_timeout=0.5, future_tolerance=0.05,
                  max_skew=0.08, tf_wait_timeout=0.06)
    values.update(changes)
    return NavigationRuntime(RuntimeConfig(**values))


def populate(state, stamp=10.0, wall=20.0):
    state.accept_odom(stamp, wall, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0))
    state.accept_cloud(stamp, wall, "odom", ())
    state.accept_terrain(stamp, wall, "odom", Terrain(0, 1, 0.001, True))
    state.accept_controller(stamp, wall, True)
    state.accept_readiness("base_ready", True, stamp, wall)
    state.accept_readiness("external_ready", True, stamp, wall)
    state.accept_readiness("terrain_ready", True, stamp, wall)


class NavigationRuntimeTest(unittest.TestCase):
    def test_fresh_empty_cloud_is_valid_but_missing_cloud_is_stale(self):
        state = runtime()
        state.set_goal("r1", Pose(2, 0, 1, 0), 10.0)
        state.accept_odom(10, 20, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0))
        state.accept_terrain(10, 20, "odom", Terrain(0, 1, 0.001, True))
        state.accept_controller(10, 20, True)
        state.accept_readiness("base_ready", True, 10, 20)
        state.accept_readiness("external_ready", True, 10, 20)
        state.accept_readiness("terrain_ready", True, 10, 20)
        self.assertEqual(state.snapshot(10.1, 20.1)[2], "cloud missing or stale")
        state.accept_cloud(10, 20, "odom", ())
        token, snap, reason = state.snapshot(10.1, 20.1)
        self.assertEqual(reason, "")
        self.assertEqual(snap.obstacles, ())
        self.assertTrue(state.revalidate(token, 10.1, 20.1))

    def test_stale_dropout_and_arrival_age_revoke_snapshot(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        self.assertIn("stale", state.snapshot(10.7, 20.1)[2])
        self.assertIn("stale", state.snapshot(10.1, 20.7)[2])

    def test_wrong_frame_and_malformed_newer_stamp_clear_accepted_state(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        self.assertFalse(state.accept_cloud(10.1, 20.1, "map", ()))
        self.assertIn("cloud", state.snapshot(10.1, 20.1)[2])
        self.assertFalse(state.accept_cloud(10.05, 20.2, "odom", ()))
        self.assertTrue(state.accept_cloud(10.2, 20.2, "odom", ()))

    def test_nonincreasing_and_future_stamps_are_rejected(self):
        state = runtime()
        self.assertTrue(state.accept_cloud(10, 20, "odom", ()))
        self.assertFalse(state.accept_cloud(10, 20.1, "odom", ()))
        self.assertFalse(state.accept_cloud(11, 20.2, "odom", (), ros_now=10))
        self.assertTrue(state.accept_cloud(10.1, 20.3, "odom", (), ros_now=10.1))

    def test_duplicate_callback_revokes_previously_usable_snapshot(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        self.assertEqual(state.snapshot(10.1, 20.1)[2], "")
        self.assertFalse(state.accept_odom(
            10, 20.2, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0)))
        self.assertEqual(state.snapshot(10.1, 20.2)[2], "odom missing or stale")

    def test_source_skew_blocks_snapshot(self):
        state = runtime(max_skew=0.05)
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        state.accept_cloud(10.2, 20.1, "odom", ())
        self.assertEqual(state.snapshot(10.2, 20.1)[2], "input source skew exceeds limit")

    def test_request_supersession_cancel_and_token_revalidation(self):
        state = runtime()
        state.set_goal("first", Pose(1, 0, 1, 0), 10)
        populate(state)
        first, _, _ = state.snapshot(10.1, 20.1)
        self.assertTrue(state.set_goal("second", Pose(2, 0, 1, 0), 10.1).accepted)
        self.assertFalse(state.revalidate(first, 10.1, 20.1))
        second, snap, _ = state.snapshot(10.1, 20.1)
        self.assertEqual(snap.goal, Pose(2, 0, 1, 0))
        self.assertFalse(state.cancel("first").accepted)
        self.assertTrue(state.cancel("second").accepted)
        self.assertFalse(state.revalidate(second, 10.1, 20.1))

    def test_goal_transform_completion_cannot_resurrect_superseded_request(self):
        state = runtime()
        old, result = state.begin_goal("old", 10.0, 10.0)
        self.assertTrue(result.accepted)
        new, result = state.begin_goal("new", 10.1, 10.1)
        self.assertTrue(result.accepted)
        stale = state.complete_goal(old, Pose(1, 0, 1, 0), (0, 0, 0, 1), 10.1)
        self.assertFalse(stale.accepted)
        self.assertTrue(state.complete_goal(
            new, Pose(2, 0, 1, 0), (0, 0, 0, 1), 10.1).accepted)
        self.assertEqual(state.request_id, "new")

    def test_telemetry_heartbeats_do_not_supersede_goal_tf_work(self):
        state = runtime()
        token, result = state.begin_goal("goal", 10.0, 10.0)
        self.assertTrue(result.accepted)
        state.accept_controller(10.01, 20.01, True)
        for name in state.READINESS_NAMES:
            state.accept_readiness(name, True, 10.01, 20.01)
        self.assertTrue(state.complete_goal(
            token, Pose(1, 0, 1, 0), (0, 0, 0, 1), 10.02).accepted)

    def test_cancel_revokes_pending_goal_transform(self):
        state = runtime()
        token, _ = state.begin_goal("pending", 10.0, 10.0)
        self.assertTrue(state.cancel("pending").accepted)
        self.assertFalse(state.complete_goal(
            token, Pose(1, 0, 1, 0), (0, 0, 0, 1), 10.0).accepted)
        self.assertTrue(state.cancel("pending").accepted)

    def test_cancel_is_idempotent_only_when_no_other_request_is_active(self):
        state = runtime()
        self.assertTrue(state.cancel("cleanup").accepted)
        state.set_goal("active", Pose(1, 0, 1, 0), 10)
        self.assertFalse(state.cancel("wrong").accepted)
        self.assertTrue(state.cancel("active").accepted)
        self.assertTrue(state.cancel("active").accepted)

    def test_goal_stamp_freshness_and_watermark_are_enforced(self):
        state = runtime()
        self.assertFalse(state.begin_goal("future", 11.0, 10.0)[1].accepted)
        token, accepted = state.begin_goal("valid", 10.0, 10.0)
        self.assertTrue(accepted.accepted)
        self.assertFalse(state.complete_goal(
            token, Pose(1, 0, 1, 0), (0, 0, 0, 1), 10.6).accepted)
        self.assertFalse(state.begin_goal("replay", 10.0, 10.0)[1].accepted)

    def test_output_stamp_must_be_strictly_increasing(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.1, 20.1)
        self.assertTrue(state.claim_publish(token, 10.0, 10.1, 20.1))
        self.assertFalse(state.claim_publish(token, 10.0, 10.1, 20.1))
        populate(state, 10.2, 20.2)
        token2, _, _ = state.snapshot(10.2, 20.2)
        self.assertTrue(state.claim_publish(token2, 10.2, 10.2, 20.2))

    def test_commit_publish_enqueues_once_before_later_cancel(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.1, 20.1)
        sent = []
        self.assertTrue(state.commit_publish(
            token, 10.0, lambda: (10.1, 20.1), lambda: sent.append("r")))
        self.assertTrue(state.cancel("r").accepted)
        self.assertEqual(sent, ["r"])
        self.assertFalse(state.commit_publish(
            token, 10.1, lambda: (10.1, 20.1), lambda: sent.append("late")))

    def test_concurrent_20hz_heartbeats_do_not_starve_snapshot_commit(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.01, 20.01)
        start = threading.Event()
        finished = threading.Event()

        def heartbeats():
            start.wait(1)
            for index in range(1, 5):
                stamp = 10 + index * 0.02
                state.accept_controller(stamp, 20 + index * 0.02, True)
                for name in state.READINESS_NAMES:
                    state.accept_readiness(name, True, stamp, 20 + index * 0.02)
            finished.set()

        worker = threading.Thread(target=heartbeats)
        worker.start()
        start.set()
        self.assertTrue(finished.wait(1))
        sent = []
        self.assertTrue(state.commit_publish(
            token, 10.0, lambda: (10.09, 20.09), lambda: sent.append("r")))
        worker.join(1)
        self.assertEqual(sent, ["r"])

    def test_future_heartbeat_timestamp_jump_revokes_commit(self):
        state = runtime(max_skew=0.1)
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.01, 20.01)
        state.accept_controller(10.3, 20.02, True, ros_now=10.02)
        self.assertFalse(state.commit_publish(token, 10.0, lambda: (10.3, 20.02), lambda: None))

    def test_publish_clock_is_sampled_after_waiting_for_runtime_lock(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.1, 20.1)
        clock = [10.1, 20.1]
        waiting = threading.Event()
        result, sent = [], []

        def worker():
            waiting.set()
            result.append(state.commit_publish(
                token, 10.0, lambda: tuple(clock), lambda: sent.append("target")))

        with state._lock:
            thread = threading.Thread(target=worker)
            thread.start()
            self.assertTrue(waiting.wait(1))
            clock[1] = 20.8
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [False])
        self.assertEqual(sent, [])

    def test_heavy_work_token_is_revoked_by_invalid_terrain(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.1, 20.1)
        state.accept_terrain(10.1, 20.1, "odom", Terrain(0, 1, 0.001, False))
        self.assertFalse(state.claim_publish(token, 10.1, 10.1, 20.1))

    def test_delayed_cloud_selects_historical_pose_and_terrain_without_widening_skew(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        for i in range(1, 11):
            stamp = 10 + i * 0.02
            state.accept_odom(stamp, 20 + i * 0.02, "odom", "base_link",
                              Pose(i * 0.002, 0, 1, 0), Vec3(0, 0, 0))
            state.accept_terrain(stamp, 20 + i * 0.02, "odom", Terrain(0, 1, 0.001, True))
            state.accept_controller(stamp, 20 + i * 0.02, True)
        token, snap, reason = state.snapshot(10.2, 20.2)
        self.assertEqual(reason, "")
        self.assertEqual(token.stamps, (10.0, 10.0, 10.0))
        self.assertEqual(snap.current, Pose(0, 0, 1, 0))
        self.assertTrue(state.commit_publish(token, snap.stamp,
                                             lambda: (10.2, 20.2), lambda: None))

    def test_valid_50hz_updates_do_not_starve_immutable_snapshot(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, snap, _ = state.snapshot(10.01, 20.01)
        for i in range(1, 11):
            stamp = 10 + i * 0.02
            state.accept_odom(stamp, 20 + i * 0.02, "odom", "base_link",
                              Pose(0.001 * i, 0, 1, 0), Vec3(0, 0, 0))
            state.accept_terrain(stamp, 20 + i * 0.02, "odom", Terrain(0, 1, 0.001, True))
            state.accept_controller(stamp, 20 + i * 0.02, True)
        self.assertTrue(state.revalidate(token, 10.2, 20.2))
        self.assertEqual(snap.current, Pose(0, 0, 1, 0))

    def test_selected_historical_sample_expiry_still_blocks_commit(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.01, 20.01)
        state.accept_odom(10.3, 20.6, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0))
        state.accept_terrain(10.3, 20.6, "odom", Terrain(0, 1, 0.001, True))
        self.assertFalse(state.revalidate(token, 10.3, 20.6))

    def test_actual_motion_since_selected_pose_revokes_commit(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.01, 20.01)
        state.accept_odom(10.02, 20.02, "odom", "base_link", Pose(0.2, 0, 1, 0), Vec3(0, 0, 0))
        self.assertFalse(state.revalidate(token, 10.02, 20.02))

    def test_invalid_then_valid_sensor_does_not_resurrect_old_history(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        token, _, _ = state.snapshot(10.01, 20.01)
        state.accept_terrain(10.02, 20.02, "odom", Terrain(0, 1, 0.5, True))
        state.accept_terrain(10.2, 20.2, "odom", Terrain(0, 1, 0.001, True))
        self.assertFalse(state.revalidate(token, 10.2, 20.2))
        self.assertEqual(state.snapshot(10.2, 20.2)[2], "input source skew exceeds limit")

    def test_combined_source_span_is_bounded_not_just_each_cloud_difference(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state, 9.94, 20)
        state.accept_cloud(10, 20.1, "odom", ())
        state.accept_terrain(10.06, 20.1, "odom", Terrain(0, 1, 0.001, True))
        # The older matching terrain still provides a valid pair; invalidating
        # it forces the otherwise individually-close +/-.06 pair to be rejected.
        state.accept_terrain(10.06, 20.1, "odom", Terrain(0, 1, 0.001, True))
        state.accept_terrain(10.061, 20.1, "odom", Terrain(0, 1, 0.001, True))
        self.assertEqual(state.snapshot(10.07, 20.1)[2], "input source skew exceeds limit")

    def test_selected_pose_expires_even_when_cloud_and_live_inputs_are_fresh(self):
        state = runtime()
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        state.accept_cloud(10.07, 20.07, "odom", ())
        state.accept_terrain(10.07, 20.07, "odom", Terrain(0, 1, 0.001, True))
        token, _, reason = state.snapshot(10.07, 20.07)
        self.assertEqual(reason, "")
        state.accept_odom(10.52, 20.52, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0))
        state.accept_terrain(10.52, 20.52, "odom", Terrain(0, 1, 0.001, True))
        state.accept_controller(10.52, 20.52, True)
        self.assertFalse(state.revalidate(token, 10.52, 20.52))

    def test_new_ground_or_excess_speed_revokes_reserved_margin(self):
        for source in ("ground", "velocity"):
            state = runtime()
            state.set_goal("r", Pose(1, 0, 1, 0), 10)
            populate(state)
            token, _, _ = state.snapshot(10.01, 20.01)
            if source == "ground":
                state.accept_terrain(10.02, 20.02, "odom", Terrain(0.1, 0.9, 0.001, True))
            else:
                state.accept_odom(10.02, 20.02, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0.2, 0, 0))
            self.assertFalse(state.revalidate(token, 10.02, 20.02))

    def test_history_is_bounded_and_never_restamps_evicted_samples(self):
        state = runtime(history_limit=4)
        state.set_goal("r", Pose(1, 0, 1, 0), 10)
        populate(state)
        for i in range(1, 21):
            stamp = 10 + i * 0.01
            state.accept_odom(stamp, 20 + i * 0.01, "odom", "base_link", Pose(0, 0, 1, 0), Vec3(0, 0, 0))
            state.accept_terrain(stamp, 20 + i * 0.01, "odom", Terrain(0, 1, 0.001, True))
        self.assertTrue(all(len(history) <= 4 for history in state._history.values()))
        self.assertEqual(state.snapshot(10.2, 20.2)[2], "input source skew exceeds limit")

    def test_tilted_map_pose_transform_uses_full_xyz_and_orientation(self):
        angle = math.pi / 6
        rotation = (0, math.sin(angle / 2), 0, math.cos(angle / 2))
        pose = transform_pose(Pose(1, 0, 0, 0), (0, 0, 2), rotation)
        self.assertAlmostEqual(pose.x, math.cos(angle))
        self.assertAlmostEqual(pose.z, 2 - math.sin(angle))
        self.assertAlmostEqual(pose.yaw, 0)

    def test_cloud_transform_uses_same_tilted_transform(self):
        angle = math.pi / 4
        rotation = (0, math.sin(angle / 2), 0, math.cos(angle / 2))
        point = transform_points(((1, 0, 0),), (0, 0, 0), rotation)[0]
        self.assertAlmostEqual(point[0], math.cos(angle))
        self.assertAlmostEqual(point[2], -math.sin(angle))

    def test_full_pose_transform_preserves_tilted_orientation(self):
        angle = math.pi / 5
        rotation = (0, math.sin(angle / 2), 0, math.cos(angle / 2))
        position, orientation = transform_pose_full(
            (0, 0, 0), (0, 0, 0, 2), (1, 2, 3), rotation)
        self.assertEqual(position, (1.0, 2.0, 3.0))
        self.assertAlmostEqual(sum(value * value for value in orientation), 1.0)
        self.assertAlmostEqual(orientation[1], math.sin(angle / 2))

    def test_exact_lookup_retries_nonblocking_at_source_stamp(self):
        calls = []
        wall = [1.0]

        def lookup(target, source, stamp, timeout):
            calls.append((target, source, stamp, timeout))
            wall[0] += 0.02
            if len(calls) < 3:
                raise TransformUnavailable()
            return "transform"

        result = lookup_exact(lookup, "odom", "map", 10.0, 0.06,
                              clock=lambda: wall[0], pause=lambda: None,
                              exceptions=(TransformUnavailable,))
        self.assertEqual(result, "transform")
        self.assertTrue(all(call == ("odom", "map", 10.0, 0.0) for call in calls))

    def test_delayed_tf_expires_without_latest_transform_fallback(self):
        wall = [1.0]
        stamps = []

        def unavailable(_target, _source, stamp, timeout):
            stamps.append((stamp, timeout))
            wall[0] += 0.04
            raise TransformUnavailable()

        with self.assertRaises(TransformUnavailable):
            lookup_exact(unavailable, "odom", "map", 10.0, 0.06,
                         clock=lambda: wall[0], pause=lambda: None,
                         exceptions=(TransformUnavailable,))
        self.assertTrue(all(stamp == 10.0 and timeout == 0.0 for stamp, timeout in stamps))


if __name__ == "__main__":
    unittest.main()
