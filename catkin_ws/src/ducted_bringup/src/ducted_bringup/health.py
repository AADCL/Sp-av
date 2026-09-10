import time
from collections import namedtuple


HealthResult = namedtuple("HealthResult", ("ready", "reasons"))


class BaseHealth:
    def __init__(
        self,
        require_mavros,
        require_livox,
        lidar_timeout_sec,
        mavros_timeout_sec=3.0,
    ):
        if mavros_timeout_sec <= 0.0 or lidar_timeout_sec <= 0.0:
            raise ValueError("health timeouts must be positive")

        self.require_mavros = bool(require_mavros)
        self.require_livox = bool(require_livox)
        self.mavros_timeout_sec = float(mavros_timeout_sec)
        self.lidar_timeout_sec = float(lidar_timeout_sec)
        self._mavros_connected = False
        self._last_mavros_time = None
        self._last_livox_time = None

    def update_mavros(self, connected, now=None):
        self._mavros_connected = bool(connected)
        self._last_mavros_time = time.monotonic() if now is None else float(now)

    def update_livox(self, now=None):
        self._last_livox_time = time.monotonic() if now is None else float(now)

    def evaluate(self, now=None):
        current_time = time.monotonic() if now is None else float(now)
        reasons = []

        if self.require_mavros:
            if self._last_mavros_time is None:
                reasons.append("mavros state missing")
            elif current_time - self._last_mavros_time > self.mavros_timeout_sec:
                reasons.append("mavros state stale")
            elif not self._mavros_connected:
                reasons.append("mavros disconnected")

        if self.require_livox:
            if self._last_livox_time is None:
                reasons.append("livox points missing")
            elif current_time - self._last_livox_time > self.lidar_timeout_sec:
                reasons.append("livox points stale")

        return HealthResult(ready=not reasons, reasons=tuple(reasons))
