"""Bounded service worker and injectable full-pose transformation helpers."""

from dataclasses import dataclass
import math
import multiprocessing
from queue import Empty, Full, PriorityQueue
import threading
from time import monotonic


@dataclass(frozen=True)
class WorkItem:
    kind: str
    generation: int
    request_id: str
    target: object


@dataclass(frozen=True)
class ServiceReply:
    accepted: bool
    message: str


def _deadline_entry(connection, function, arguments):
    try:
        result = function(*arguments)
        connection.send(result)
    except BaseException as error:
        connection.send((False, str(error)))
    finally:
        connection.close()


class DeadlineProcess:
    """Run one operation in a process that can be terminated at a deadline."""

    def __init__(self, context=None):
        self.context = context or multiprocessing.get_context("spawn")
        self._process = None
        self._lock = threading.Lock()
        self._closed = False
        self._cancel_event = threading.Event()

    @property
    def process_alive(self):
        with self._lock:
            return self._process is not None and self._process.is_alive()

    def call(self, function, arguments, timeout):
        if not math.isfinite(float(timeout)) or timeout <= 0.0:
            raise ValueError("service timeout must be positive and finite")
        receiver, sender = self.context.Pipe(duplex=False)
        process = self.context.Process(
            target=_deadline_entry, args=(sender, function, arguments))
        with self._lock:
            if self._closed:
                receiver.close()
                sender.close()
                return False, "service helper is closed; outcome uncertain"
            self._cancel_event.clear()
            self._process = process
            process.start()
        sender.close()
        try:
            deadline = monotonic() + timeout
            response_ready = False
            while not self._cancel_event.is_set():
                remaining = deadline - monotonic()
                if remaining <= 0.0:
                    break
                if receiver.poll(min(0.05, remaining)):
                    response_ready = True
                    break
            if response_ready:
                try:
                    result = receiver.recv()
                except EOFError:
                    return False, "service helper stopped; outcome uncertain"
                process.join(timeout=0.2)
                if (not isinstance(result, tuple) or len(result) != 2):
                    return False, "service helper returned a malformed response"
                return bool(result[0]), str(result[1])
            if self._cancel_event.is_set():
                return False, "service helper stopped; outcome uncertain"
            process.terminate()
            process.join(timeout=0.5)
            if process.is_alive() and hasattr(process, "kill"):
                process.kill()
                process.join(timeout=0.5)
            return False, "service response deadline exceeded; outcome uncertain"
        finally:
            receiver.close()
            if process.is_alive():
                process.terminate()
                process.join(timeout=0.5)
            with self._lock:
                if self._process is process:
                    self._process = None

    def cancel(self, close=False):
        with self._lock:
            if close:
                self._closed = True
            self._cancel_event.set()
            process = self._process
            if process is not None and process.is_alive():
                process.terminate()
        if process is not None:
            process.join(timeout=0.5)
            if process.is_alive() and hasattr(process, "kill"):
                process.kill()
                process.join(timeout=0.5)

    def close(self):
        self.cancel(close=True)


def _persistent_entry(connection, bootstrap, bootstrap_arguments):
    try:
        handler = bootstrap(*bootstrap_arguments)
        connection.send(("ready", True, ""))
    except BaseException as error:
        connection.send(("ready", False, str(error)))
        connection.close()
        return
    while True:
        try:
            request = connection.recv()
        except EOFError:
            break
        if request is None:
            break
        sequence, arguments = request
        try:
            result = handler(*arguments)
            if not isinstance(result, tuple) or len(result) != 2:
                result = (False, "service helper returned a malformed response")
            accepted, message = bool(result[0]), str(result[1])
        except BaseException as error:
            accepted, message = False, str(error)
        try:
            connection.send(("result", sequence, accepted, message))
        except (BrokenPipeError, EOFError, OSError):
            break
    connection.close()


class PersistentDeadlineProcess:
    """Prewarm one helper and terminate it when a response misses its deadline."""

    def __init__(self, bootstrap, bootstrap_arguments=(), context=None):
        self.bootstrap = bootstrap
        self.bootstrap_arguments = tuple(bootstrap_arguments)
        self.context = context or multiprocessing.get_context("spawn")
        self.operation_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.process = None
        self.connection = None
        self.ready = False
        self.closed = False
        self.sequence = 0

    @property
    def process_alive(self):
        with self.state_lock:
            return self.process is not None and self.process.is_alive()

    def _wait_locked(self, timeout):
        deadline = monotonic() + timeout
        while not self.cancel_event.is_set():
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                return None
            if self.connection.poll(min(0.05, remaining)):
                try:
                    return self.connection.recv()
                except EOFError:
                    return None
        return None

    def _stop_locked(self):
        process, connection = self.process, self.connection
        self.process = None
        self.connection = None
        self.ready = False
        if process is not None and process.is_alive():
            process.terminate()
        if process is not None:
            process.join(timeout=0.5)
            if process.is_alive() and hasattr(process, "kill"):
                process.kill()
                process.join(timeout=0.5)
        if connection is not None:
            connection.close()

    def ensure_ready(self, timeout):
        if not math.isfinite(float(timeout)) or timeout <= 0.0:
            raise ValueError("warm timeout must be positive and finite")
        with self.operation_lock:
            with self.state_lock:
                if self.closed:
                    return False, "service helper is closed"
                if self.ready and self.process is not None and self.process.is_alive():
                    return True, "service helper is ready"
                self._stop_locked()
                parent, child = self.context.Pipe(duplex=True)
                process = self.context.Process(
                    target=_persistent_entry,
                    args=(child, self.bootstrap, self.bootstrap_arguments))
                self.cancel_event.clear()
                self.process = process
                self.connection = parent
                process.start()
                child.close()
            response = self._wait_locked(timeout)
            if (response is None or len(response) != 3
                    or response[0] != "ready" or not response[1]):
                message = ("service helper warmup deadline exceeded"
                           if response is None else str(response[2]))
                with self.state_lock:
                    self._stop_locked()
                return False, message
            with self.state_lock:
                self.ready = True
            return True, "service helper is ready"

    def call(self, arguments, timeout):
        if not math.isfinite(float(timeout)) or timeout <= 0.0:
            raise ValueError("service timeout must be positive and finite")
        with self.operation_lock:
            with self.state_lock:
                if (self.closed or not self.ready or self.process is None
                        or not self.process.is_alive()):
                    return False, "service helper is not ready; outcome uncertain"
                self.sequence += 1
                sequence = self.sequence
                try:
                    self.connection.send((sequence, tuple(arguments)))
                except (BrokenPipeError, EOFError, OSError):
                    self._stop_locked()
                    return False, "service helper transport failed; outcome uncertain"
            response = self._wait_locked(timeout)
            if (response is None or len(response) != 4
                    or response[0] != "result" or response[1] != sequence):
                with self.state_lock:
                    self._stop_locked()
                return False, "service response deadline exceeded; outcome uncertain"
            return bool(response[2]), str(response[3])

    def close(self):
        with self.state_lock:
            self.closed = True
            self.cancel_event.set()
        with self.operation_lock:
            with self.state_lock:
                self._stop_locked()


class GoalTransformer:
    def __init__(self, pose_factory, lookup_transform, transform_pose,
                 duration_zero):
        self.pose_factory = pose_factory
        self.lookup_transform = lookup_transform
        self.transform_pose = transform_pose
        self.duration_zero = duration_zero

    def transform(self, action, stamp, odom_frame):
        waypoint = action.waypoint
        message = self.pose_factory()
        message.header.frame_id = waypoint.frame_id
        message.header.stamp = stamp
        message.pose.position.x = waypoint.x
        message.pose.position.y = waypoint.y
        message.pose.position.z = waypoint.z
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        message.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        if waypoint.frame_id == odom_frame:
            return message
        transform = self.lookup_transform(
            odom_frame, waypoint.frame_id, stamp, self.duration_zero())
        output = self.transform_pose(message, transform)
        output.header.frame_id = odom_frame
        output.header.stamp = stamp
        return output

    @staticmethod
    def values(message):
        position = message.pose.position
        orientation = message.pose.orientation
        values = (
            float(position.x), float(position.y), float(position.z),
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("transformed waypoint is non-finite")
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("transformed waypoint quaternion is invalid")
        yaw = math.atan2(
            2.0 * (values[6] * values[5] + values[3] * values[4]),
            1.0 - 2.0 * (values[4] * values[4] + values[5] * values[5]),
        )
        return values[0], values[1], values[2], yaw


class ServiceWorker:
    """Execute navigation and hold calls in one bounded FIFO thread."""

    def __init__(self, navigation_call, flight_call, result_callback, capacity=4,
                 current_callback=None, ready_callback=None):
        if isinstance(capacity, bool) or int(capacity) != capacity or capacity < 1:
            raise ValueError("capacity must be a positive integer")
        self.navigation_call = navigation_call
        self.flight_call = flight_call
        self.result_callback = result_callback
        self.current_callback = current_callback or (lambda _item: True)
        self.ready_callback = ready_callback or (lambda _item: True)
        self.queue = PriorityQueue(maxsize=int(capacity))
        self.sequence = 0
        self.sequence_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def submit(self, item):
        if not isinstance(item, WorkItem):
            raise TypeError("item must be WorkItem")
        try:
            with self.sequence_lock:
                self.sequence += 1
                sequence = self.sequence
            priority = 0 if item.kind == "stop" else 1
            self.queue.put_nowait((priority, sequence, item))
            return True
        except Full:
            return False

    @staticmethod
    def _response(response):
        try:
            return bool(response.accepted), str(response.message)
        except AttributeError:
            return False, "service returned a malformed response"

    def _run(self):
        while not self.stop_event.is_set():
            try:
                _, _, item = self.queue.get(timeout=0.1)
            except Empty:
                continue
            if item is None:
                self.queue.task_done()
                return
            try:
                ready = self.ready_callback(item)
            except Exception as error:
                ready = False
                ready_message = "service helper readiness failed: {}".format(error)
            else:
                ready_message = "service helper is unavailable"
            if not ready:
                self.result_callback(
                    item.kind, item.generation, False, ready_message)
                self.queue.task_done()
                continue
            try:
                current = self.current_callback(item)
            except Exception as error:
                current = False
                stale_message = "dispatch validation failed: {}".format(error)
            else:
                stale_message = "work item became stale before service dispatch"
            if not current:
                self.result_callback(
                    item.kind, item.generation, False,
                    stale_message)
                self.queue.task_done()
                continue
            accepted = False
            message = ""
            if item.kind == "goal":
                try:
                    accepted, message = self._response(
                        self.navigation_call("goal", item.request_id, item.target))
                except Exception as error:
                    message = str(error)
            elif item.kind == "stop":
                try:
                    cancel_ok, cancel_message = self._response(
                        self.navigation_call("cancel", item.request_id, item.target))
                except Exception as error:
                    cancel_ok, cancel_message = False, str(error)
                hold_message = "hold helper is unavailable or stop became stale"
                try:
                    hold_ready = (self.ready_callback(item)
                                  and self.current_callback(item))
                except Exception as error:
                    hold_ready = False
                    hold_message = str(error)
                if hold_ready:
                    try:
                        hold_ok, hold_message = self._response(
                            self.flight_call("hold", item.target))
                    except Exception as error:
                        hold_ok, hold_message = False, str(error)
                else:
                    hold_ok = False
                accepted = cancel_ok and hold_ok
                message = "cancel: {}; hold: {}".format(
                    cancel_message, hold_message)
            else:
                message = "unknown work item"
            try:
                self.result_callback(
                    item.kind, item.generation, accepted, message)
            finally:
                self.queue.task_done()

    def shutdown(self):
        self.stop_event.set()
        if self.thread is None:
            return
        try:
            with self.sequence_lock:
                self.sequence += 1
                sequence = self.sequence
            self.queue.put_nowait((-1, sequence, None))
        except Full:
            pass
        self.thread.join(timeout=1.0)
