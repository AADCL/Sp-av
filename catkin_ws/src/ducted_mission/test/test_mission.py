import math
import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))

from ducted_mission.mission import (  # noqa: E402
    ControllerObservation,
    ExecutionTarget,
    GateSnapshot,
    MissionConfig,
    MissionCore,
    OdomObservation,
    PlannerObservation,
    StampedInput,
)


def config_dict(count=2):
    return {
        "mission_id": "survey",
        "allowed_frames": ["map", "odom"],
        "workspace": {"x": [-10, 10], "y": [-10, 10], "z": [0.2, 4.0]},
        "defaults": {
            "xy_tolerance": 0.2,
            "z_tolerance": 0.15,
            "yaw_tolerance": 0.1,
            "speed_tolerance": 0.1,
            "dwell": 0.5,
            "timeout": 10.0,
        },
        "mission_timeout": 30.0,
        "blocked_timeout": 2.0,
        "waypoints": [
            {
                "frame_id": "map",
                "position": [float(i + 1), 0.0, 1.0],
                "yaw": 0.0,
            }
            for i in range(count)
        ],
    }


def gates(now=1.0, **changes):
    values = dict(
        base_ready=True,
        base_age=0.1,
        rc_valid=True,
        rc_command=True,
        kill=False,
        rc_age=0.1,
        odom_age=0.1,
        terrain_valid=True,
        terrain_ready=True,
        terrain_age=0.1,
        planner_age=0.1,
        controller_ready=True,
        controller_state="HOLD",
        controller_age=0.1,
    )
    values.update(changes)
    return GateSnapshot(now=now, **values)


def at_waypoint(core, now, planner_state="GOAL_REACHED", controller_state="TRACK"):
    waypoint = core.config.waypoints[core.waypoint_index]
    if core.execution_target is None:
        core.set_execution_target(
            core.generation,
            ExecutionTarget(waypoint.x, waypoint.y, waypoint.z, waypoint.yaw))
    return core.update(
        now,
        gates(now, controller_state=controller_state),
        OdomObservation(waypoint.x, waypoint.y, waypoint.z, waypoint.yaw, 0.0),
        PlannerObservation(core.request_id, planner_state),
        ControllerObservation(True, controller_state, core.request_id),
    )


class ConfigTests(unittest.TestCase):
    def test_validates_waypoint_and_global_bounds(self):
        parsed = MissionConfig.from_dict(config_dict(3))
        self.assertEqual(len(parsed.waypoints), 3)
        for mutate in (
            lambda d: d["waypoints"][0].update(yaw=math.nan),
            lambda d: d["waypoints"][0].update(frame_id="earth"),
            lambda d: d.update(allowed_frames=["earth"]),
            lambda d: d["waypoints"][0].update(position=[11, 0, 1]),
            lambda d: d["defaults"].update(dwell=-1),
            lambda d: d["defaults"].update(timeout=0),
            lambda d: d["defaults"].update(xy_tolerance=0),
            lambda d: d.update(mission_timeout=math.inf),
        ):
            raw = config_dict()
            mutate(raw)
            with self.assertRaises(ValueError):
                MissionConfig.from_dict(raw)


class StampedInputTests(unittest.TestCase):
    def test_watermark_advances_before_payload_validation(self):
        stream = StampedInput("odom", source_timeout=0.5, arrival_timeout=0.5)
        self.assertFalse(stream.accept(10.0, 20.0, False, now_source=10.1))
        self.assertFalse(stream.accept(10.0, 20.1, True, now_source=10.1))
        self.assertFalse(stream.valid)
        self.assertTrue(stream.accept(10.2, 20.2, True, now_source=10.2))
        self.assertTrue(stream.fresh(20.4, 10.4))
        self.assertFalse(stream.fresh(20.8, 10.4))

    def test_stale_future_backward_and_malformed_stamps_invalidate(self):
        stream = StampedInput("planner", 0.5, 0.5, future_tolerance=0.05)
        self.assertFalse(stream.accept(9.0, 20.0, True, now_source=10.0))
        self.assertFalse(stream.accept(10.2, 20.1, True, now_source=10.0))
        self.assertFalse(stream.accept(float("nan"), 20.2, True, now_source=10.0))
        self.assertTrue(stream.accept(10.0, 20.3, True, now_source=10.0))
        self.assertFalse(stream.accept(9.9, 20.4, True, now_source=10.0))


class MissionCoreTests(unittest.TestCase):
    def setUp(self):
        self.core = MissionCore(
            MissionConfig.from_dict(config_dict()), True, session_id="testsession")

    def test_load_is_idle_and_activation_is_explicit_and_enabled(self):
        disabled = MissionCore(self.core.config, False, session_id="testsession")
        self.assertEqual(disabled.state, "IDLE")
        self.assertFalse(disabled.start(1.0, gates()).accepted)
        self.assertEqual(disabled.state, "IDLE")
        result = self.core.start(1.0, gates())
        self.assertTrue(result.accepted)
        self.assertEqual(result.actions[0].kind, "goal")
        self.assertEqual(
            self.core.request_id,
            "survey-testsession-e000001-wp-000-try-000")

    def test_start_rejects_each_unready_gate(self):
        changes = (
            {"base_age": 2.0}, {"rc_command": False}, {"kill": True},
            {"odom_age": 1.0}, {"terrain_valid": False},
            {"terrain_ready": False}, {"planner_age": 1.0},
            {"controller_ready": False}, {"controller_state": "OFF"},
        )
        for change in changes:
            core = MissionCore(self.core.config, True, session_id="testsession")
            self.assertFalse(core.start(1.0, gates(**change)).accepted, change)

    def test_matching_actual_state_and_dwell_advances_in_order(self):
        self.core.start(1.0, gates())
        self.assertEqual(at_waypoint(self.core, 2.0), [])
        actions = at_waypoint(self.core, 2.5)
        self.assertEqual([a.kind for a in actions], ["goal"])
        self.assertEqual(self.core.waypoint_index, 1)
        self.assertEqual(
            self.core.request_id,
            "survey-testsession-e000001-wp-001-try-000")
        at_waypoint(self.core, 3.0)
        actions = at_waypoint(self.core, 3.5)
        self.assertEqual(self.core.state, "COMPLETING")
        self.assertEqual([a.kind for a in actions], ["stop"])
        self.assertNotIn("land", [a.kind for a in actions])
        self.assertTrue(self.core.ack_stop(actions[0].generation, True))
        self.assertEqual(self.core.state, "SUCCEEDED")

    def test_three_waypoints_keep_order_and_use_transformed_execution_target(self):
        core = MissionCore(MissionConfig.from_dict(config_dict(3)), True,
                           session_id="testsession")
        action = core.start(1.0, gates()).actions[0]
        visited = []
        for expected_index in range(3):
            self.assertEqual(core.waypoint_index, expected_index)
            target = ExecutionTarget(-1.0 + expected_index, -2.0, 1.5, 0.25)
            self.assertTrue(core.set_execution_target(action.generation, target))
            visited.append(core.request_id)
            planner = PlannerObservation(core.request_id, "GOAL_REACHED")
            controller = ControllerObservation(True, "TRACK", core.request_id)
            odom = OdomObservation(target.x, target.y, target.z, target.yaw, 0.0)
            self.assertEqual(core.update(2.0 + expected_index,
                                         gates(2.0, controller_state="TRACK"),
                                         odom, planner, controller), [])
            actions = core.update(2.5 + expected_index,
                                  gates(2.5, controller_state="TRACK"),
                                  odom, planner, controller)
            action = actions[0]
        self.assertEqual(visited, [
            "survey-testsession-e000001-wp-000-try-000",
            "survey-testsession-e000001-wp-001-try-000",
            "survey-testsession-e000001-wp-002-try-000",
        ])
        self.assertEqual(action.kind, "stop")
        self.assertEqual(core.state, "COMPLETING")
        core.ack_stop(action.generation, True)
        self.assertEqual(core.state, "SUCCEEDED")

    def test_transformed_target_must_remain_inside_workspace(self):
        action = self.core.start(1.0, gates()).actions[0]
        self.assertFalse(self.core.set_execution_target(
            action.generation, ExecutionTarget(100.0, 0.0, 1.0, 0.0)))

    def test_actual_feedback_cannot_complete_before_transform_is_installed(self):
        self.core.start(1.0, gates())
        waypoint = self.core.config.waypoints[0]
        self.core.update(
            2.0, gates(2.0, controller_state="TRACK"),
            OdomObservation(waypoint.x, waypoint.y, waypoint.z, waypoint.yaw, 0.0),
            PlannerObservation(self.core.request_id, "GOAL_REACHED"),
            ControllerObservation(True, "TRACK", self.core.request_id))
        self.assertIsNone(self.core.dwell_started)

    def test_false_success_wrong_ids_hold_speed_pose_and_yaw_do_not_dwell(self):
        self.core.start(1.0, gates())
        waypoint = self.core.config.waypoints[0]
        cases = (
            (PlannerObservation("old", "GOAL_REACHED"), ControllerObservation(True, "TRACK", self.core.request_id), OdomObservation(1, 0, 1, 0, 0)),
            (PlannerObservation(self.core.request_id, "GOAL_REACHED"), ControllerObservation(True, "TRACK", "old"), OdomObservation(1, 0, 1, 0, 0)),
            (PlannerObservation(self.core.request_id, "GOAL_REACHED"), ControllerObservation(True, "HOLD", self.core.request_id), OdomObservation(1, 0, 1, 0, 0)),
            (PlannerObservation(self.core.request_id, "CLEAR"), ControllerObservation(True, "TRACK", self.core.request_id), OdomObservation(1, 0, 1, 0, 0)),
            (PlannerObservation(self.core.request_id, "GOAL_REACHED"), ControllerObservation(True, "TRACK", self.core.request_id), OdomObservation(waypoint.x + 1, 0, 1, 0, 0)),
            (PlannerObservation(self.core.request_id, "GOAL_REACHED"), ControllerObservation(True, "TRACK", self.core.request_id), OdomObservation(1, 0, 1, 0.5, 0)),
            (PlannerObservation(self.core.request_id, "GOAL_REACHED"), ControllerObservation(True, "TRACK", self.core.request_id), OdomObservation(1, 0, 1, 0, 0.5)),
        )
        for planner, controller, odom in cases:
            self.core.update(2.0, gates(2.0, controller_state=controller.state), odom, planner, controller)
            self.assertIsNone(self.core.dwell_started)

    def test_drift_resets_uninterrupted_dwell(self):
        self.core.start(1.0, gates())
        at_waypoint(self.core, 2.0)
        waypoint = self.core.config.waypoints[0]
        self.core.update(2.4, gates(2.4, controller_state="TRACK"),
                         OdomObservation(waypoint.x + 1, 0, 1, 0, 0),
                         PlannerObservation(self.core.request_id, "GOAL_REACHED"),
                         ControllerObservation(True, "TRACK", self.core.request_id))
        self.assertIsNone(self.core.dwell_started)
        at_waypoint(self.core, 2.5)
        self.assertEqual(at_waypoint(self.core, 2.9), [])
        self.assertEqual(self.core.waypoint_index, 0)

    def test_pause_resume_requires_stop_ack_and_uses_new_retry_id(self):
        self.core.start(1.0, gates())
        pause = self.core.pause(2.0, "operator")
        self.assertTrue(pause.accepted)
        self.assertEqual(pause.actions[0].kind, "stop")
        self.assertFalse(self.core.resume(2.1, gates(2.1)).accepted)
        self.assertFalse(self.core.ack_stop(pause.actions[0].generation - 1, True))
        self.assertTrue(self.core.ack_stop(pause.actions[0].generation, True))
        resume = self.core.resume(2.2, gates(2.2))
        self.assertTrue(resume.accepted)
        self.assertEqual(
            self.core.request_id,
            "survey-testsession-e000001-wp-000-try-001")
        self.assertGreater(resume.actions[0].generation, pause.actions[0].generation)

    def test_late_goal_response_cannot_resurrect_cancelled_mission(self):
        start = self.core.start(1.0, gates())
        generation = start.actions[0].generation
        cancel = self.core.cancel(1.1)
        self.assertEqual(self.core.state, "CANCELED")
        self.assertEqual(cancel.actions[0].kind, "stop")
        self.assertEqual(self.core.ack_goal(generation, True), [])
        self.assertEqual(self.core.state, "CANCELED")

    def test_restart_waits_for_terminal_stop_and_failed_final_hold_fails(self):
        self.core.start(1.0, gates())
        at_waypoint(self.core, 2.0)
        at_waypoint(self.core, 2.5)
        self.assertEqual(self.core.state, "RUNNING")
        at_waypoint(self.core, 3.0)
        final_stop = at_waypoint(self.core, 3.5)[0]
        self.assertEqual(self.core.state, "COMPLETING")
        self.assertFalse(self.core.start(4.0, gates(4.0)).accepted)
        self.assertFalse(self.core.ack_stop(final_stop.generation, False))
        self.assertEqual(self.core.state, "FAILED")
        self.assertTrue(self.core.ack_stop(final_stop.generation, True))
        self.assertTrue(self.core.start(4.1, gates(4.1)).accepted)

    def test_blocked_timeout_waypoint_timeout_and_gate_loss_stop(self):
        self.core.start(1.0, gates())
        rid = self.core.request_id
        controller = ControllerObservation(True, "TRACK", rid)
        odom = OdomObservation(0, 0, 1, 0, 0)
        self.assertEqual(self.core.update(2.0, gates(2.0, controller_state="TRACK"), odom,
                                          PlannerObservation(rid, "BLOCKED"), controller), [])
        actions = self.core.update(4.1, gates(4.1, controller_state="TRACK"), odom,
                                   PlannerObservation(rid, "BLOCKED"), controller)
        self.assertEqual(self.core.state, "FAILED")
        self.assertEqual(actions[0].kind, "stop")

        core = MissionCore(self.core.config, True, session_id="testsession")
        core.start(1.0, gates())
        actions = core.update(11.1, gates(11.1), odom,
                              PlannerObservation(core.request_id, "CLEAR"),
                              ControllerObservation(True, "HOLD", "old"))
        self.assertEqual(core.state, "FAILED")
        self.assertEqual(actions[0].kind, "stop")

        core = MissionCore(self.core.config, True, session_id="testsession")
        core.start(1.0, gates())
        actions = core.update(1.5, gates(1.5, rc_command=False), odom,
                              PlannerObservation(core.request_id, "CLEAR"),
                              ControllerObservation(True, "HOLD", "old"))
        self.assertEqual(core.state, "PAUSED")
        self.assertEqual(actions[0].kind, "stop")


if __name__ == "__main__":
    unittest.main()
