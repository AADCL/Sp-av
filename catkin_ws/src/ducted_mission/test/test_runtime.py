import math
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS


PACKAGE = Path(__file__).resolve().parents[1]
# catkin invokes nose with -P (no path adjustment). Spawned children must
# import this module to unpickle the top-level synthetic service bootstrap.
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(PACKAGE / "src"))

from ducted_mission.mission import MissionAction, Waypoint  # noqa: E402
from ducted_mission.runtime import (  # noqa: E402
    DeadlineProcess,
    GoalTransformer,
    PersistentDeadlineProcess,
    ServiceWorker,
    WorkItem,
)


def delayed_result(delay):
    time.sleep(delay)
    return True, "late"


def persistent_bootstrap(delay=0.0):
    time.sleep(delay)

    def handle(command):
        if command == "goal":
            time.sleep(1.0)
        return True, "{}:{}".format(command, os.getpid())

    return handle


def pose_message():
    return NS(
        header=NS(frame_id="", stamp=None),
        pose=NS(
            position=NS(x=0.0, y=0.0, z=0.0),
            orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def waypoint(frame="map"):
    return Waypoint(frame, 1.0, 2.0, 3.0, 0.5,
                    0.2, 0.1, 0.1, 0.1, 0.5, 10.0)


class GoalTransformerTests(unittest.TestCase):
    def test_exact_time_full_pose_transform_preserves_transformed_z(self):
        calls = []

        def lookup(target, source, stamp, duration):
            calls.append((target, source, stamp, duration))
            return "tf"

        def transform(message, transform):
            self.assertEqual(transform, "tf")
            output = pose_message()
            output.header.frame_id = "odom"
            output.header.stamp = message.header.stamp
            output.pose.position.x = message.pose.position.x + 10.0
            output.pose.position.y = message.pose.position.y - 4.0
            output.pose.position.z = message.pose.position.z + 2.5
            output.pose.orientation = message.pose.orientation
            return output

        transformer = GoalTransformer(pose_message, lookup, transform,
                                      duration_zero=lambda: 0.0)
        action = MissionAction("goal", 4, "request", waypoint())
        output = transformer.transform(action, stamp=12.25, odom_frame="odom")
        self.assertEqual(calls, [("odom", "map", 12.25, 0.0)])
        self.assertEqual(output.header.frame_id, "odom")
        self.assertAlmostEqual(output.pose.position.z, 5.5)
        self.assertAlmostEqual(output.pose.orientation.z, math.sin(0.25))
        self.assertAlmostEqual(output.pose.orientation.w, math.cos(0.25))

    def test_odom_waypoint_does_not_request_tf(self):
        transformer = GoalTransformer(
            pose_message,
            lambda *_: self.fail("lookup should not run"),
            lambda *_: self.fail("transform should not run"),
            duration_zero=lambda: 0.0,
        )
        output = transformer.transform(
            MissionAction("goal", 1, "r", waypoint("odom")), 3.0, "odom")
        self.assertEqual(output.header.stamp, 3.0)
        self.assertEqual(output.pose.position.z, 3.0)


class ServiceWorkerTests(unittest.TestCase):
    def test_constructor_and_startup_do_not_call_services(self):
        calls = []
        worker = ServiceWorker(lambda *_: calls.append("nav"),
                               lambda *_: calls.append("flight"),
                               lambda *_: None, capacity=2)
        worker.start()
        time.sleep(0.02)
        worker.shutdown()
        self.assertEqual(calls, [])

    def test_delayed_goal_then_cancel_and_hold_are_serialized(self):
        entered = threading.Event()
        release = threading.Event()
        completed = threading.Event()
        calls = []

        def navigation(command, request_id, target):
            calls.append(command + "-begin")
            if command == "goal":
                entered.set()
                release.wait(1.0)
            calls.append(command + "-end")
            return NS(accepted=True, message="ok")

        def flight(command, target):
            calls.append(command)
            return NS(accepted=True, message="ok")

        results = []
        worker = ServiceWorker(navigation, flight,
                               lambda *args: (results.append(args), completed.set()),
                               capacity=2)
        worker.start()
        self.assertTrue(worker.submit(WorkItem("goal", 1, "r1", object())))
        self.assertTrue(entered.wait(1.0))
        self.assertTrue(worker.submit(WorkItem("stop", 2, "r1", object())))
        release.set()
        self.assertTrue(completed.wait(1.0))
        deadline = time.time() + 1.0
        while len(results) < 2 and time.time() < deadline:
            time.sleep(0.005)
        worker.shutdown()
        self.assertEqual(calls, ["goal-begin", "goal-end", "cancel-begin",
                                 "cancel-end", "hold"])
        self.assertNotIn("land", calls)

    def test_network_calls_run_without_the_owner_lock(self):
        owner_lock = threading.Lock()
        called = threading.Event()

        def navigation(*_):
            self.assertTrue(owner_lock.acquire(timeout=0.2))
            owner_lock.release()
            called.set()
            return NS(accepted=True, message="ok")

        worker = ServiceWorker(navigation, lambda *_: NS(accepted=True, message="ok"),
                               lambda *_: None, capacity=1)
        worker.start()
        self.assertTrue(worker.submit(WorkItem("goal", 1, "r", object())))
        self.assertTrue(called.wait(1.0))
        worker.shutdown()

    def test_queue_is_bounded(self):
        worker = ServiceWorker(lambda *_: None, lambda *_: None,
                               lambda *_: None, capacity=1)
        self.assertTrue(worker.submit(WorkItem("goal", 1, "one", object())))
        self.assertFalse(worker.submit(WorkItem("goal", 2, "two", object())))

    def test_cancel_exception_still_executes_hold(self):
        calls = []
        finished = threading.Event()

        def navigation(command, _request_id, _target):
            calls.append(command)
            raise RuntimeError("cancel transport failed")

        def flight(command, _target):
            calls.append(command)
            return NS(accepted=True, message="holding")

        results = []
        worker = ServiceWorker(
            navigation, flight,
            lambda *result: (results.append(result), finished.set()), capacity=1)
        worker.start()
        worker.submit(WorkItem("stop", 1, "r", object()))
        self.assertTrue(finished.wait(1.0))
        worker.shutdown()
        self.assertEqual(calls, ["cancel", "hold"])
        self.assertFalse(results[0][2])

    def test_stop_priority_and_dispatch_token_drop_queued_stale_goal(self):
        entered = threading.Event()
        release = threading.Event()
        done = threading.Event()
        current_generation = [1]
        calls = []

        def navigation(command, request_id, _target):
            calls.append((command, request_id))
            if request_id == "active" and command == "goal":
                entered.set()
                release.wait(1.0)
            return NS(accepted=True, message="ok")

        results = []
        worker = ServiceWorker(
            navigation,
            lambda command, _target: (calls.append((command, ""))
                                      or NS(accepted=True, message="ok")),
            lambda *result: (results.append(result), done.set()),
            capacity=3,
            current_callback=lambda item: item.generation == current_generation[0])
        worker.start()
        worker.submit(WorkItem("goal", 1, "active", object()))
        self.assertTrue(entered.wait(1.0))
        worker.submit(WorkItem("goal", 2, "stale", object()))
        current_generation[0] = 3
        worker.submit(WorkItem("stop", 3, "active", object()))
        release.set()
        deadline = time.time() + 1.0
        while len(results) < 3 and time.time() < deadline:
            time.sleep(0.005)
        worker.shutdown()
        self.assertEqual(calls, [("goal", "active"), ("cancel", "active"),
                                 ("hold", "")])


class DeadlineProcessTests(unittest.TestCase):
    def test_hung_call_is_terminated_at_deadline(self):
        runner = DeadlineProcess()
        accepted, message = runner.call(delayed_result, (1.0,), timeout=0.05)
        self.assertFalse(accepted)
        self.assertIn("outcome uncertain", message)
        self.assertFalse(runner.process_alive)

    def test_goal_deadline_still_runs_cancel_and_hold_cleanup(self):
        runner = DeadlineProcess()
        calls = []
        finished = threading.Event()
        worker_ref = []

        def navigation(command, request_id, target):
            calls.append(command)
            if command == "goal":
                accepted, message = runner.call(
                    delayed_result, (1.0,), timeout=0.05)
                return NS(accepted=accepted, message=message)
            return NS(accepted=True, message="canceled")

        def result(kind, generation, accepted, _message):
            if kind == "goal" and not accepted:
                worker_ref[0].submit(WorkItem("stop", generation + 1, "r", object()))
            elif kind == "stop":
                finished.set()

        worker = ServiceWorker(
            navigation,
            lambda command, _target: (calls.append(command)
                                      or NS(accepted=True, message="holding")),
            result, capacity=2)
        worker_ref.append(worker)
        worker.start()
        worker.submit(WorkItem("goal", 1, "r", object()))
        self.assertTrue(finished.wait(2.0))
        worker.shutdown()
        self.assertEqual(calls, ["goal", "cancel", "hold"])
        self.assertFalse(runner.process_alive)

    def test_cancel_terminates_active_helper(self):
        runner = DeadlineProcess()
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                runner.call(delayed_result, (2.0,), timeout=5.0)))
        thread.start()
        deadline = time.time() + 2.0
        while not runner.process_alive and time.time() < deadline:
            time.sleep(0.005)
        self.assertTrue(runner.process_alive)
        runner.cancel()
        thread.join(2.0)
        self.assertFalse(thread.is_alive())
        self.assertFalse(runner.process_alive)
        self.assertFalse(result[0][0])


class PersistentDeadlineProcessTests(unittest.TestCase):
    def test_process_is_prewarmed_and_reused(self):
        runner = PersistentDeadlineProcess(persistent_bootstrap, (0.0,))
        self.addCleanup(runner.close)
        ready, reason = runner.ensure_ready(1.0)
        self.assertTrue(ready, reason)
        first = runner.call(("first",), timeout=0.5)
        second = runner.call(("second",), timeout=0.5)
        runner.close()
        self.assertTrue(first[0] and second[0])
        self.assertEqual(first[1].split(":")[1], second[1].split(":")[1])

    def test_delayed_bootstrap_precedes_goal_freshness_validation(self):
        runner = PersistentDeadlineProcess(persistent_bootstrap, (0.1,))
        started = time.monotonic()
        events = []
        called = []
        finished = threading.Event()

        def ready(_item):
            events.append("ready-begin")
            accepted = runner.ensure_ready(1.0)[0]
            events.append("ready-end")
            return accepted

        def current(_item):
            events.append("current")
            return time.monotonic() - started <= 0.05

        worker = ServiceWorker(
            lambda *_: called.append("navigation"), lambda *_: None,
            lambda *_: finished.set(), capacity=1,
            ready_callback=ready, current_callback=current)
        worker.start()
        worker.submit(WorkItem("goal", 1, "r", object()))
        self.assertTrue(finished.wait(2.0))
        worker.shutdown()
        runner.close()
        self.assertEqual(events, ["ready-begin", "ready-end", "current"])
        self.assertEqual(called, [])

    def test_goal_timeout_restarts_helper_for_cancel_and_hold(self):
        runner = PersistentDeadlineProcess(persistent_bootstrap, (0.0,))
        operations = []
        results = []
        finished = threading.Event()
        worker_ref = []

        def call(command):
            operations.append(command)
            return NS(*(), **dict(zip(("accepted", "message"),
                                      runner.call((command,), timeout=0.05))))

        def result(kind, generation, accepted, message):
            results.append((kind, accepted, message))
            if kind == "goal":
                worker_ref[0].submit(WorkItem("stop", generation + 1, "r", object()))
            else:
                finished.set()

        worker = ServiceWorker(
            lambda command, _request_id, _target: call(command),
            lambda command, _target: call(command), result, capacity=2,
            ready_callback=lambda _item: runner.ensure_ready(1.0)[0])
        worker_ref.append(worker)
        worker.start()
        worker.submit(WorkItem("goal", 1, "r", object()))
        self.assertTrue(finished.wait(3.0))
        worker.shutdown()
        runner.close()
        self.assertEqual(operations, ["goal", "cancel", "hold"])
        self.assertFalse(results[0][1])
        self.assertTrue(results[-1][1])


if __name__ == "__main__":
    unittest.main()
