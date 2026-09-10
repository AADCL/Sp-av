"""Validated RC decoding; this module never issues flight commands.

Role layout follows the legacy swing_control RC_Input, with calibrated scaling,
bounds checks, timestamp checks, and level states replacing latched action flags.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RCState:
    valid: bool = False
    reason: str = 'RC unavailable'
    stamp: float = 0.
    roll: float = 0.
    pitch: float = 0.
    throttle: float = 0.
    yaw: float = 0.
    mode: str = 'manual'
    kill_switch: bool = True
    arm_switch: bool = False
    land_switch: bool = False
    trigger_switch: bool = False


class RCDecoder:
    ROLES = ('roll', 'pitch', 'throttle', 'yaw', 'mode', 'land', 'arm', 'kill', 'trigger')

    def __init__(self, config):
        self.channels = dict(config['channels'])
        if set(self.channels) != set(self.ROLES):
            raise ValueError('all RC roles must be configured exactly once')
        if any(type(i) is not int or not 1 <= i <= 18 for i in self.channels.values()):
            raise ValueError('RC channels must be integer indices from 1 to 18')
        if len(set(self.channels.values())) != len(self.channels):
            raise ValueError('RC roles must not share channels')
        self.calibration = {}
        for role, index in self.channels.items():
            try:
                entry = config['calibration'][str(index)]
                low, high, trim, reverse = (float(entry[k]) for k in ('min', 'max', 'trim', 'reverse'))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError('missing or invalid RC calibration') from error
            if (not all(math.isfinite(v) for v in (low, high, trim, reverse))
                    or not 500 <= low < high <= 2500 or not low <= trim <= high
                    or reverse not in (-1, 1)):
                raise ValueError('invalid RC calibration')
            if role in ('roll', 'pitch', 'yaw') and not low < trim < high:
                raise ValueError('centered stick trim must be inside endpoints')
            self.calibration[role] = (low, high, trim, reverse)
        self.timeout = float(config.get('timeout', .5))
        self.future_tolerance = float(config.get('future_tolerance', .05))
        self.deadzone = float(config.get('deadzone', .05))
        self.hysteresis = float(config.get('hysteresis', .03))
        self.margin = float(config.get('pwm_margin', 75))
        if (not all(math.isfinite(v) for v in (self.timeout, self.future_tolerance,
                                              self.deadzone, self.hysteresis, self.margin))
                or self.timeout <= 0 or self.future_tolerance < 0
                or not 0 <= self.deadzone < 1 or not 0 <= self.hysteresis < .2
                or not 0 <= self.margin <= 200):
            raise ValueError('invalid RC validation configuration')
        self.latest = RCState()
        self.last_stamp = 0.
        self.seen_stamp = 0.
        self.last_wall = None

    def _invalid(self, reason):
        self.latest = RCState(reason=reason)
        return self.latest

    def _mode(self, value, previous):
        if previous == 'manual' and value <= .25 + self.hysteresis:
            return 'manual'
        if previous == 'command' and value >= .75 - self.hysteresis:
            return 'command'
        if previous == 'hold' and .25 - self.hysteresis <= value <= .75 + self.hysteresis:
            return 'hold'
        return 'manual' if value <= .25 else 'command' if value >= .75 else 'hold'

    def update(self, channels, stamp, ros_now, wall_now, rssi=255):
        if not all(math.isfinite(v) for v in (stamp, ros_now, wall_now)):
            return self._invalid('nonfinite RC time')
        if stamp <= 0 or stamp <= self.seen_stamp:
            return self._invalid('non-increasing RC timestamp')
        if not -self.future_tolerance <= ros_now - stamp <= self.timeout:
            return self._invalid('RC timestamp stale or future')
        # Even a malformed payload consumes its measurement time. An older
        # switch state arriving afterward must not become valid again.
        self.seen_stamp = stamp
        if type(rssi) is not int or not 0 < rssi <= 255:
            return self._invalid('RC RSSI lost or invalid')
        if len(channels) < max(self.channels.values()):
            return self._invalid('insufficient RC channels')
        previous = self.evaluate(ros_now, wall_now)
        normalized = {}
        for role, index in self.channels.items():
            try:
                pwm = float(channels[index - 1])
            except (ValueError, TypeError):
                return self._invalid('invalid RC PWM')
            low, high, trim, reverse = self.calibration[role]
            if not math.isfinite(pwm) or not low - self.margin <= pwm <= high + self.margin:
                return self._invalid('RC PWM outside calibration bounds')
            pwm = min(high, max(low, pwm))
            if role in ('roll', 'pitch', 'yaw'):
                value = (pwm - trim) / ((high - trim) if pwm >= trim else (trim - low)) * reverse
                value = math.copysign(max(0., abs(value) - self.deadzone) / (1 - self.deadzone), value)
            else:
                value = (pwm - low) / (high - low)
                if reverse == -1:
                    value = 1 - value
            normalized[role] = value
        self.latest = RCState(True, '', stamp, normalized['roll'], normalized['pitch'],
                              normalized['throttle'], normalized['yaw'],
                              self._mode(normalized['mode'], previous.mode if previous.valid else None),
                              normalized['kill'] > .75, normalized['arm'] > .75,
                              normalized['land'] > .75, normalized['trigger'] > .75)
        self.last_stamp, self.last_wall = stamp, wall_now
        return self.latest

    def evaluate(self, ros_now, wall_now):
        if not self.latest.valid:
            return self.latest
        if (not all(math.isfinite(v) for v in (ros_now, wall_now))
                or not 0 <= wall_now - self.last_wall <= self.timeout
                or not -self.future_tolerance <= ros_now - self.last_stamp <= self.timeout):
            return self._invalid('RC data unavailable or stale')
        return self.latest
