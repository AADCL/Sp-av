"""Pure flight ownership, transition, and position-limit policy.

The module has no ROS or MAVROS imports.  Callers supply source time and
monotonic receipt time, then execute the returned actions outside their lock.
"""
from dataclasses import dataclass
from collections import deque
import math


ON_GROUND = 1
IN_AIR = 2


@dataclass(frozen=True)
class Pose:
    x: float = 0.
    y: float = 0.
    z: float = 0.
    yaw: float = 0.


@dataclass(frozen=True)
class Result:
    accepted: bool
    message: str


@dataclass(frozen=True)
class Action:
    kind: str
    pose: Pose = None
    mode: str = ''
    token: int = 0
    generation: int = 0


@dataclass(frozen=True)
class Status:
    state: str
    reason: str
    ready: bool
    request_id: str
    target: Pose
    fcu_valid: bool
    pose_valid: bool
    rc_valid: bool
    base_valid: bool
    external_valid: bool


class FlightConfig:
    POSITIVE = (
        'telemetry_timeout', 'base_ready_timeout', 'external_ready_timeout',
        'external_pose_timeout', 'pose_pair_timeout', 'pose_history_duration',
        'max_external_pose_skew',
        'max_external_position_error', 'max_external_yaw_error', 'target_timeout',
        'prestream_duration', 'prestream_max_gap', 'activation_timeout',
        'mode_feedback_timeout', 'mode_request_timeout', 'planner_hold_grace',
        'takeoff_timeout', 'takeoff_tolerance', 'takeoff_settle_time',
        'land_timeout', 'min_takeoff_rise', 'max_xy_speed', 'max_z_speed',
        'max_yaw_rate', 'max_lead')

    def __init__(self, values):
        self.enable_flight_output = values.get('enable_flight_output', False) is True
        self.require_automatic_lease = values.get('require_automatic_lease', False) is True
        self.frame_id = values.get('frame_id', 'odom')
        if self.frame_id != 'odom':
            raise ValueError('flight frame_id must be odom')
        for name in self.POSITIVE:
            value = float(values[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)
            setattr(self, name, value)
        self.future_tolerance = float(values['future_tolerance'])
        if not math.isfinite(self.future_tolerance) or self.future_tolerance < 0:
            raise ValueError('future_tolerance must be finite and nonnegative')
        for axis in ('x', 'y', 'z'):
            low, high = float(values['min_' + axis]), float(values['max_' + axis])
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError('invalid %s envelope' % axis)
            setattr(self, 'min_' + axis, low)
            setattr(self, 'max_' + axis, high)
        if self.prestream_max_gap >= self.prestream_duration:
            raise ValueError('prestream gap must be shorter than duration')
        if self.mode_request_timeout > self.mode_feedback_timeout:
            raise ValueError('mode request timeout exceeds feedback timeout')


class FlightPolicy:
    OWNED = ('HOLD', 'TRACK', 'HOLD_GRACE', 'TAKEOFF')
    STREAMING = ('PRESTREAM', 'MODE_WAIT') + OWNED

    def __init__(self, config):
        self.config = config
        self.state = 'DISABLED'
        self.reason = 'flight output disabled' if not config.enable_flight_output else 'operator engage required'
        self.request_id = ''
        self.target = Pose()
        self._fcu = self._source_record()
        self._pose = self._source_record()
        self._external_pose = self._source_record()
        self._pose_history = deque(maxlen=100)
        self._external_pose_history = deque(maxlen=100)
        self._pose_pair = {'valid': False, 'wall': None}
        self._landed = self._source_record()
        self._rc = self._source_record()
        self._base = {'valid': False, 'wall': None}
        self._external = {'valid': False, 'wall': None}
        self._automatic_lease = {'valid': False, 'wall': None}
        self._last_ros = 0.
        self._last_wall = 0.
        self._last_output = None
        self._last_stream_wall = None
        self._prestream_start = None
        self._transition_start = None
        self._pending_token = None
        self._pending_kind = ''
        self._mode_sent = False
        self._mode_sent_wall = None
        self._mode_accepted = False
        self._mode_result_wall = None
        self._next_token = 1
        self._generation = 0
        self._mode_in_flight_token = None
        self._target_record = self._source_record()
        self._target_seen_stamp = 0.
        self._held_request_id = ''
        self._grace_deadline = None
        self._settled_since = None
        self._land_baselined = False
        self._last_land_level = False
        self._land_rising = False
        self._rc_hold_captured = False

    @staticmethod
    def _source_record():
        return {'valid': False, 'stamp': 0., 'wall': None, 'seen': 0.}

    @staticmethod
    def _finite(*values):
        return all(math.isfinite(float(value)) for value in values)

    @classmethod
    def _finite_pose(cls, pose):
        return isinstance(pose, Pose) and cls._finite(pose.x, pose.y, pose.z, pose.yaw)

    def _set_now(self, ros_now, wall_now):
        if not self._finite(ros_now, wall_now):
            return False
        self._last_ros, self._last_wall = float(ros_now), float(wall_now)
        return True

    def _source_update(self, record, valid, stamp, ros_now, wall_now, **fields):
        self._set_now(ros_now, wall_now)
        valid_time = self._finite(stamp, ros_now, wall_now)
        stamp = float(stamp) if valid_time else 0.
        if (not valid_time or stamp <= 0 or stamp <= record['seen']
                or float(ros_now) - stamp < -self.config.future_tolerance):
            record.update(valid=False, stamp=0., wall=None)
            return False
        record['seen'] = stamp
        if not valid or float(ros_now) - stamp > self.config.telemetry_timeout:
            record.update(valid=False, stamp=stamp, wall=float(wall_now))
            return False
        record.update(valid=True, stamp=stamp, wall=float(wall_now), **fields)
        return True

    def update_fcu(self, connected, armed, mode, system_status, stamp, ros_now, wall_now):
        valid = (type(connected) is bool and type(armed) is bool and isinstance(mode, str)
                 and mode != '' and type(system_status) is int)
        return self._source_update(self._fcu, valid, stamp, ros_now, wall_now,
                                   connected=connected, armed=armed, mode=mode,
                                   system_status=system_status)

    def update_pose(self, pose, stamp, ros_now, wall_now):
        accepted = self._source_update(self._pose, self._finite_pose(pose), stamp, ros_now,
                                       wall_now, pose=pose)
        self._record_pose(accepted, self._pose, self._pose_history,
                          self._external_pose_history, True)
        return accepted

    def update_external_pose(self, pose, stamp, ros_now, wall_now):
        accepted = self._source_update(self._external_pose, self._finite_pose(pose), stamp,
                                       ros_now, wall_now, pose=pose)
        self._record_pose(accepted, self._external_pose, self._external_pose_history,
                          self._pose_history, False)
        return accepted

    def _record_pose(self, accepted, record, history, other_history, is_fcu):
        if not accepted:
            self._pose_pair['valid'] = False
            return
        sample = (record['stamp'], record['wall'], record['pose'])
        history.append(sample)
        newest = sample[0]
        while history and newest - history[0][0] > self.config.pose_history_duration:
            history.popleft()
        if not other_history:
            return
        other = min(other_history, key=lambda item: abs(item[0] - sample[0]))
        if abs(other[0] - sample[0]) > self.config.max_external_pose_skew:
            return
        fcu, external = (sample, other) if is_fcu else (other, sample)
        self._pose_pair = {'valid': True, 'wall': max(fcu[1], external[1]),
                           'fcu_stamp': fcu[0], 'external_stamp': external[0],
                           'fcu_pose': fcu[2], 'external_pose': external[2]}

    def update_landed(self, landed_state, stamp, ros_now, wall_now):
        valid = type(landed_state) is int and landed_state >= 0
        return self._source_update(self._landed, valid, stamp, ros_now, wall_now,
                                   landed_state=landed_state)

    def update_rc(self, valid, mode, kill, land, stamp, ros_now, wall_now):
        previous_fresh = self._fresh(self._rc, ros_now, wall_now, self.config.telemetry_timeout)
        accepted = self._source_update(
            self._rc,
            valid is True and mode in ('manual', 'hold', 'command')
            and type(kill) is bool and type(land) is bool,
            stamp, ros_now, wall_now, mode=mode, kill=kill, land=land)
        if not accepted:
            self._land_baselined = False
            self._land_rising = False
        elif not previous_fresh or not self._land_baselined:
            self._last_land_level = land
            self._land_baselined = True
            self._land_rising = False
        else:
            self._land_rising = land and not self._last_land_level
            self._last_land_level = land
        return accepted

    def update_base_ready(self, ready, wall_now):
        if not self._finite(wall_now) or type(ready) is not bool:
            self._base.update(valid=False, wall=None)
            return False
        self._last_wall = float(wall_now)
        self._base.update(valid=ready, wall=float(wall_now))
        return ready

    def update_automatic_lease(self, ready, wall_now):
        self._automatic_lease.update(valid=ready is True,wall=wall_now)

    def update_external_ready(self, ready, wall_now):
        if not self._finite(wall_now) or type(ready) is not bool:
            self._external.update(valid=False, wall=None)
            return False
        self._last_wall = float(wall_now)
        self._external.update(valid=ready, wall=float(wall_now))
        return ready

    def _fresh(self, record, ros_now, wall_now, timeout):
        if not record['valid'] or record['wall'] is None:
            return False
        return (0 <= wall_now - record['wall'] <= timeout
                and -self.config.future_tolerance <= ros_now - record['stamp'] <= timeout)

    @staticmethod
    def _ready_fresh(record, wall_now, timeout):
        return (record['valid'] and record['wall'] is not None
                and 0 <= wall_now - record['wall'] <= timeout)

    def _validity(self, ros_now, wall_now):
        return {
            'fcu': self._fresh(self._fcu, ros_now, wall_now, self.config.telemetry_timeout),
            'pose': self._fresh(self._pose, ros_now, wall_now, self.config.telemetry_timeout),
            'external_pose': self._fresh(
                self._external_pose, ros_now, wall_now, self.config.external_pose_timeout),
            'landed': self._fresh(self._landed, ros_now, wall_now, self.config.telemetry_timeout),
            'rc': self._fresh(self._rc, ros_now, wall_now, self.config.telemetry_timeout),
            'base': self._ready_fresh(self._base, wall_now, self.config.base_ready_timeout),
            'external': self._ready_fresh(
                self._external, wall_now, self.config.external_ready_timeout),
        }

    def _pose_pair_fresh(self, ros_now, wall_now):
        pair = self._pose_pair
        if not pair['valid'] or pair['wall'] is None:
            return False
        return (0 <= wall_now - pair['wall'] <= self.config.pose_pair_timeout
                and -self.config.future_tolerance <= ros_now - pair['fcu_stamp']
                    <= self.config.telemetry_timeout
                and -self.config.future_tolerance <= ros_now - pair['external_stamp']
                    <= self.config.external_pose_timeout)

    def _gate_reason(self, ros_now, wall_now, require_armed=True, require_automatic=True):
        valid = self._validity(ros_now, wall_now)
        if require_automatic and self.config.require_automatic_lease and not self._ready_fresh(self._automatic_lease, wall_now, .5):
            return 'automatic flight lease unavailable or stale'
        for key in ('fcu', 'pose', 'external_pose', 'landed', 'rc', 'base', 'external'):
            if not valid[key]:
                return '%s unavailable or stale' % key.replace('_', ' ')
        if not self._fcu['connected'] or self._fcu['system_status'] not in (3, 4):
            return 'FCU not connected and operational'
        if self._rc['kill']:
            return 'kill switch active'
        if self._rc['mode'] not in ('hold', 'command'):
            return 'RC mode does not grant command authority'
        if not self._pose_pair_fresh(ros_now, wall_now):
            return 'external/FCU matched pose pair unavailable or stale'
        actual, external = self._pose_pair['fcu_pose'], self._pose_pair['external_pose']
        position_error = math.sqrt((actual.x-external.x)**2 + (actual.y-external.y)**2
                                   + (actual.z-external.z)**2)
        yaw_error = abs(self._angle_delta(actual.yaw, external.yaw))
        if (position_error > self.config.max_external_position_error
                or yaw_error > self.config.max_external_yaw_error):
            return 'external/FCU pose consistency exceeds position or yaw limit'
        if not self._in_envelope(actual):
            return 'FCU pose is outside configured envelope'
        if require_armed and not self._fcu['armed']:
            return 'FCU is not armed'
        return ''

    def _in_envelope(self, pose):
        c = self.config
        return (c.min_x <= pose.x <= c.max_x and c.min_y <= pose.y <= c.max_y
                and c.min_z <= pose.z <= c.max_z)

    def _clear_ownership(self, reason, state='DISABLED'):
        if self._mode_in_flight_token is not None and state != 'INHIBITED':
            state = 'INHIBITED'
            reason += '; mode outcome uncertain while backend call is unresolved'
        self._generation += 1
        self.state, self.reason = state, reason
        self.request_id = ''
        self._pending_token = None
        self._pending_kind = ''
        self._mode_sent = False
        self._mode_sent_wall = None
        self._mode_accepted = False
        self._last_stream_wall = None
        self._prestream_start = None
        self._target_record['valid'] = False
        self._grace_deadline = None

    def _new_mode_action(self, kind, mode):
        token = self._next_token
        self._next_token += 1
        self._pending_token, self._pending_kind = token, kind
        self._mode_sent = True
        self._mode_sent_wall = self._last_wall
        self._mode_accepted = False
        self._mode_result_wall = None
        return Action('mode', mode=mode, token=token, generation=self._generation)

    def mode_pending(self, token):
        return (token == self._pending_token
                and self.state in ('MODE_WAIT', 'LAND_MODE_WAIT', 'RELEASING'))

    def mode_dispatched(self, token, wall_now):
        if (not self._finite(wall_now) or not self.mode_pending(token)
                or not self._mode_request_current(wall_now)):
            return False
        self._mode_in_flight_token = token
        return True

    def _mode_request_current(self, wall_now):
        return (self._mode_sent_wall is not None
                and 0 <= wall_now - self._mode_sent_wall <= self.config.mode_request_timeout)

    def command(self, command, target, ros_now, wall_now):
        if not self._set_now(ros_now, wall_now):
            return Result(False, 'invalid command time')
        if not self.config.enable_flight_output:
            return Result(False, 'flight output disabled')
        command = command.strip().lower() if isinstance(command, str) else ''
        if command == 'abort':
            if self.state in ('DISABLED', 'INHIBITED'):
                return Result(True, 'automatic output already stopped')
            if self.state == 'PRESTREAM':
                self._clear_ownership('automatic sequence aborted before OFFBOARD')
                return Result(True, self.reason)
            if self.state == 'MODE_WAIT':
                self._clear_ownership('automatic sequence aborted during mode transition', 'INHIBITED')
                return Result(True, self.reason)
            if self.state in ('LAND_MODE_WAIT', 'LANDING', 'RELEASING'):
                # Do not countermand a landing or a release already owned by PX4.
                return Result(True, 'PX4 landing/release retained; automatic sequence stopped')
            gate = self._gate_reason(ros_now, wall_now)
            if gate:
                self._clear_ownership('automatic sequence aborted: ' + gate, 'INHIBITED')
                return Result(True, self.reason)
            return self.command('hold', None, ros_now, wall_now)
        if command == 'reset':
            if self.state != 'INHIBITED':
                return Result(False, 'reset only applies to latched inhibit')
            if self._mode_in_flight_token is not None:
                return Result(False, 'reset blocked by unresolved mode service call')
            gate = self._gate_reason(ros_now, wall_now, require_armed=False, require_automatic=False)
            if gate or self._fcu.get('armed', True) or self._rc.get('kill', True):
                return Result(False, gate or 'reset requires disarmed FCU and kill clear')
            self._clear_ownership('operator engage required')
            return Result(True, 'inhibit reset')
        if command == 'engage':
            if self._mode_in_flight_token is not None:
                return Result(False, 'unresolved mode service call blocks engage')
            if self.state != 'DISABLED':
                return Result(False, 'another transition or ownership is active')
            gate = self._gate_reason(ros_now, wall_now)
            if gate:
                return Result(False, gate)
            self.state, self.reason = 'PRESTREAM', 'prestreaming captured pose'
            self._generation += 1
            self.target = self._pose['pose']
            self._last_output = self.target
            self._transition_start = wall_now
            self._prestream_start = None
            self._last_stream_wall = None
            self._rc_hold_captured = self._rc.get('mode') == 'hold'
            return Result(True, 'engage prestream started')
        if command in ('hold', 'release', 'takeoff', 'land') and self.state not in self.OWNED:
            return Result(False, 'active OFFBOARD ownership required')
        if command in ('hold', 'release', 'takeoff', 'land') and self._fcu.get('mode') != 'OFFBOARD':
            return Result(False, 'fresh OFFBOARD feedback required')
        if command == 'hold':
            self._generation += 1
            # Revoke the previous stream, including messages already in transit.
            # A new command requires a distinct ID and a post-hold source stamp.
            self._held_request_id = self.request_id or self._held_request_id
            self._target_seen_stamp = max(self._target_seen_stamp, float(ros_now))
            self.target = self._pose['pose']
            self._last_output = self.target
            self.state, self.reason = 'HOLD', 'explicit hold'
            self._target_record['valid'] = False
            self._rc_hold_captured = True
            return Result(True, 'holding measured pose')
        if command == 'release':
            self._generation += 1
            self.state, self.reason = 'RELEASING', 'operator release requested'
            self.request_id = ''
            self._mode_sent = False
            self._mode_sent_wall = None
            return Result(True, 'release requested')
        if command == 'takeoff':
            gate = self._gate_reason(ros_now, wall_now)
            if gate:
                return Result(False, gate)
            if self._rc.get('mode') != 'command':
                return Result(False, 'takeoff requires RC command mode')
            if not self._finite_pose(target) or not self._in_envelope(target):
                return Result(False, 'takeoff target is invalid or outside envelope')
            if not self._validity(ros_now, wall_now)['landed'] or self._landed['landed_state'] != ON_GROUND:
                return Result(False, 'takeoff requires fresh ON_GROUND feedback')
            actual = self._pose['pose']
            if target.z < actual.z + self.config.min_takeoff_rise:
                return Result(False, 'takeoff target must be above current pose')
            self.target = Pose(actual.x, actual.y, target.z, actual.yaw)
            self._generation += 1
            self.state, self.reason = 'TAKEOFF', 'climbing to absolute odom target'
            self._transition_start = wall_now
            self._settled_since = None
            return Result(True, 'takeoff accepted')
        if command == 'land':
            self._generation += 1
            self.target = self._pose['pose']
            self._last_output = self.target
            self.state, self.reason = 'LAND_MODE_WAIT', 'AUTO.LAND request pending'
            self._transition_start = wall_now
            self._mode_sent = False
            self._pending_token = None
            return Result(True, 'landing transition accepted')
        return Result(False, 'unknown command')

    def accept_target(self, request_id, pose, frame_id, stamp, ros_now, wall_now):
        if not self._set_now(ros_now, wall_now):
            return Result(False, 'invalid target receipt time')
        valid_time = (self._finite(stamp) and stamp > 0 and stamp > self._target_seen_stamp
                      and ros_now - stamp >= -self.config.future_tolerance)
        if valid_time:
            self._target_seen_stamp = float(stamp)
        valid = (self.config.enable_flight_output and self.state in ('HOLD', 'TRACK')
                 and self._gate_reason(ros_now, wall_now) == ''
                 and self._fcu.get('mode') == 'OFFBOARD' and self._rc.get('mode') == 'command'
                 and isinstance(request_id, str) and request_id != ''
                 and request_id != self._held_request_id
                 and frame_id == self.config.frame_id and self._finite_pose(pose)
                 and self._in_envelope(pose) and valid_time
                 and -self.config.future_tolerance <= ros_now - stamp <= self.config.target_timeout)
        if not valid:
            if self.state == 'TRACK':
                self._generation += 1
            self._target_record['valid'] = False
            return Result(False, 'target rejected by authority, frame, time, finite, or envelope check')
        self._target_record.update(valid=True, stamp=float(stamp), wall=float(wall_now),
                                   seen=float(stamp), pose=pose, request_id=request_id)
        self.target = pose
        self._generation += 1
        self.request_id = request_id
        self.state, self.reason = 'TRACK', 'tracking accepted planner stream'
        return Result(True, 'target accepted')

    def mode_result(self, token, accepted, wall_now):
        if type(accepted) is not bool or not self._finite(wall_now):
            return False
        dispatched = token == self._mode_in_flight_token
        if dispatched:
            self._mode_in_flight_token = None
            if not self.mode_pending(token):
                if self.state == 'INHIBITED' and 'uncertain' in self.reason:
                    self.reason = 'late backend response returned; explicit reset required'
                return False
        if not self.mode_pending(token):
            return False
        if not self._mode_request_current(wall_now):
            self._clear_ownership(
                '%s response arrived after deadline; explicit reset required' % self._pending_kind
                if dispatched else '%s service response timeout' % self._pending_kind,
                'INHIBITED' if dispatched else 'DISABLED')
            return False
        self._mode_result_wall = float(wall_now)
        if not accepted:
            self._clear_ownership('%s mode request rejected' % self._pending_kind.lower())
            return True
        self._mode_accepted = True
        self.reason = '%s mode request accepted; awaiting feedback' % self._pending_kind
        return True

    @staticmethod
    def _angle_delta(target, current):
        return (target - current + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _move_toward(value, target, maximum):
        return value + max(-maximum, min(maximum, target - value))

    def _bounded_target(self, desired, actual, dt):
        dx, dy, dz = desired.x - actual.x, desired.y - actual.y, desired.z - actual.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > self.config.max_lead:
            scale = self.config.max_lead / distance
            desired = Pose(actual.x + dx * scale, actual.y + dy * scale,
                           actual.z + dz * scale, desired.yaw)
        previous = self._last_output if self._last_output is not None else actual
        dx, dy = desired.x - previous.x, desired.y - previous.y
        xy = math.hypot(dx, dy)
        xy_step = self.config.max_xy_speed * max(0., dt)
        scale = min(1., xy_step / xy) if xy > 0 else 1.
        yaw_step = self.config.max_yaw_rate * max(0., dt)
        yaw = previous.yaw + max(-yaw_step, min(yaw_step,
                                                self._angle_delta(desired.yaw, previous.yaw)))
        if yaw > math.pi or yaw <= -math.pi:
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        output = Pose(previous.x + dx * scale, previous.y + dy * scale,
                      self._move_toward(previous.z, desired.z,
                                        self.config.max_z_speed * max(0., dt)),
                      yaw)
        return output

    def _within_lead(self, output, actual):
        return ((output.x-actual.x)**2 + (output.y-actual.y)**2 + (output.z-actual.z)**2
                <= self.config.max_lead**2 + 1e-12)

    def _stream_actions(self, wall_now):
        dt = 0. if self._last_stream_wall is None else max(0., wall_now - self._last_stream_wall)
        actual = self._pose['pose']
        desired = self.target
        output = self._bounded_target(desired, actual, dt)
        # Clamping desired lead before the slew limiter is insufficient when
        # actual pose moves away from the previous setpoint. Never jump the
        # output to repair that conflict: stop and require explicit re-engage.
        if not self._within_lead(output, actual):
            self._clear_ownership('final setpoint lead exceeds limit after slew limiting')
            return ()
        self._last_output = output
        self._last_stream_wall = wall_now
        return (Action('setpoint', pose=output, generation=self._generation),)

    def _release_action(self, reason):
        self._generation += 1
        self.state, self.reason = 'RELEASING', reason
        self.request_id = ''
        self._mode_sent = True
        return self._new_mode_action('POSCTL', 'POSCTL')

    def action_current(self, action, ros_now, wall_now):
        if (not self.config.enable_flight_output or not self._finite(ros_now, wall_now)
                or getattr(action, 'generation', None) != self._generation):
            return False
        valid = self._validity(ros_now, wall_now)
        if action.kind == 'setpoint':
            if self.state not in self.STREAMING + ('LAND_MODE_WAIT',):
                return False
            if self._gate_reason(ros_now, wall_now) != '':
                return False
            if self.state in self.OWNED + ('LAND_MODE_WAIT',) and self._fcu.get('mode') != 'OFFBOARD':
                return False
            if self.state in ('TRACK', 'TAKEOFF') and self._rc.get('mode') != 'command':
                return False
            if not self._within_lead(action.pose, self._pose['pose']):
                self._clear_ownership('final setpoint lead exceeds limit at dispatch')
                return False
            return True
        if action.kind != 'mode' or not self.mode_pending(action.token):
            return False
        if not self._mode_request_current(wall_now):
            return False
        if action.mode in ('OFFBOARD', 'AUTO.LAND'):
            return (self._gate_reason(ros_now, wall_now) == ''
                    and (action.mode != 'AUTO.LAND' or self._fcu.get('mode') == 'OFFBOARD'))
        if action.mode == 'POSCTL':
            return (all(valid[key] for key in (
                        'fcu', 'pose', 'external_pose', 'landed', 'rc', 'base', 'external'))
                    and self._fcu.get('connected') and self._fcu.get('armed')
                    and self._fcu.get('system_status') in (3, 4)
                    and not self._rc.get('kill') and self._fcu.get('mode') == 'OFFBOARD')
        return False

    def tick(self, ros_now, wall_now):
        if not self._set_now(ros_now, wall_now):
            return ()
        valid = self._validity(ros_now, wall_now)
        termination = (valid['rc'] and self._rc.get('kill', False)) or (
            valid['fcu'] and self._fcu.get('system_status') == 8)
        if termination:
            if self.state != 'INHIBITED':
                self._clear_ownership('kill or flight termination latched', 'INHIBITED')
            return ()
        if not self.config.enable_flight_output:
            return ()
        if self.state == 'INHIBITED':
            return ()
        if self.state == 'DISABLED':
            return ()
        if self.state == 'RELEASING':
            if valid['fcu'] and self._fcu.get('mode') != 'OFFBOARD':
                self._clear_ownership('control released; operator engage required')
                return ()
            if not self._mode_sent:
                return (self._new_mode_action('POSCTL', 'POSCTL'),)
            if (not self._mode_accepted
                    and wall_now - self._mode_sent_wall > self.config.mode_request_timeout):
                if self._mode_in_flight_token == self._pending_token:
                    self._clear_ownership('POSCTL mode outcome uncertain; backend call unresolved',
                                          'INHIBITED')
                else:
                    self._clear_ownership('POSCTL service response timeout')
            elif (self._mode_accepted
                  and wall_now - self._mode_result_wall > self.config.mode_feedback_timeout):
                self._clear_ownership('POSCTL feedback timeout')
            return ()

        if (self.state in self.OWNED and valid['rc'] and self._rc.get('mode') == 'manual'
                and all(valid[key] for key in (
                    'fcu', 'pose', 'external_pose', 'landed', 'base', 'external'))
                and self._fcu.get('connected') and self._fcu.get('armed')
                and self._fcu.get('system_status') in (3, 4)):
            return (self._release_action('RC manual release'),)

        require_armed = self.state not in ('LAND_MODE_WAIT', 'LANDING')
        gate = self._gate_reason(ros_now, wall_now, require_armed=require_armed)
        if gate:
            self._clear_ownership(gate)
            return ()
        if self.state in self.OWNED and self._fcu['mode'] != 'OFFBOARD':
            self._clear_ownership('pilot or FCU left OFFBOARD')
            return ()
        if self._land_rising:
            self._land_rising = False
            if self.state in self.OWNED:
                accepted = self.command('land', None, ros_now, wall_now)
                if accepted.accepted:
                    return self.tick(ros_now, wall_now)

        if self.state == 'PRESTREAM':
            if wall_now - self._transition_start > self.config.activation_timeout:
                self._clear_ownership('OFFBOARD activation timeout')
                return ()
            if (self._last_stream_wall is None
                    or wall_now - self._last_stream_wall > self.config.prestream_max_gap):
                self._prestream_start = wall_now
            actions = self._stream_actions(wall_now)
            if not actions:
                return ()
            if wall_now - self._prestream_start >= self.config.prestream_duration:
                mode = self._new_mode_action('OFFBOARD', 'OFFBOARD')
                self.state, self.reason = 'MODE_WAIT', 'OFFBOARD request pending'
                return actions + (mode,)
            return actions

        if self.state == 'MODE_WAIT':
            actions = self._stream_actions(wall_now)
            if not actions:
                return ()
            if (not self._mode_accepted
                    and wall_now - self._mode_sent_wall > self.config.mode_request_timeout):
                if self._mode_in_flight_token == self._pending_token:
                    self._clear_ownership('OFFBOARD mode outcome uncertain; backend call unresolved',
                                          'INHIBITED')
                else:
                    self._clear_ownership('OFFBOARD service response timeout')
                return ()
            if self._mode_accepted and self._fcu['mode'] == 'OFFBOARD':
                self.state, self.reason = 'HOLD', 'OFFBOARD ownership confirmed'
                self._pending_token = None
            elif (wall_now - self._transition_start > self.config.activation_timeout
                  or (self._mode_accepted and wall_now - self._mode_result_wall > self.config.mode_feedback_timeout)):
                self._clear_ownership('OFFBOARD feedback timeout')
                return ()
            return actions

        if self.state == 'LAND_MODE_WAIT':
            if wall_now - self._transition_start > self.config.land_timeout:
                self._clear_ownership('AUTO.LAND completion timeout')
                return ()
            if not self._mode_sent:
                actions = self._stream_actions(wall_now)
                if not actions:
                    return ()
                return actions + (self._new_mode_action('AUTO.LAND', 'AUTO.LAND'),)
            if not self._mode_accepted:
                if wall_now - self._mode_sent_wall > self.config.mode_request_timeout:
                    if self._mode_in_flight_token == self._pending_token:
                        self._clear_ownership(
                            'AUTO.LAND mode outcome uncertain; backend call unresolved', 'INHIBITED')
                    else:
                        self._clear_ownership('AUTO.LAND service response timeout')
                    return ()
                return self._stream_actions(wall_now)
            if self._fcu['mode'] == 'AUTO.LAND':
                self.state, self.reason = 'LANDING', 'AUTO.LAND feedback confirmed'
                self._pending_token = None
            elif wall_now - self._mode_result_wall > self.config.mode_feedback_timeout:
                self._clear_ownership('AUTO.LAND feedback timeout')
                return ()
            else:
                return self._stream_actions(wall_now)
            return ()

        if self.state == 'LANDING':
            if wall_now - self._transition_start > self.config.land_timeout:
                self._clear_ownership('AUTO.LAND completion timeout')
            elif self._landed['landed_state'] == ON_GROUND and not self._fcu['armed']:
                self._clear_ownership('landed and disarmed')
            elif self._fcu['mode'] != 'AUTO.LAND':
                self._clear_ownership('FCU left AUTO.LAND')
            return ()

        if self.state in ('TRACK', 'TAKEOFF'):
            if self._rc['mode'] == 'hold':
                self._generation += 1
                self.target = self._pose['pose']
                self._last_output = self.target
                self.state, self.reason = 'HOLD', 'RC hold captured measured pose'
                self._target_record['valid'] = False
                self._rc_hold_captured = True

        if self.state == 'TRACK':
            target_fresh = (self._target_record['valid']
                            and self._fresh(self._target_record, ros_now, wall_now,
                                            self.config.target_timeout))
            if not target_fresh:
                self.target = self._pose['pose']
                self._last_output = self.target
                self.state, self.reason = 'HOLD_GRACE', 'planner stream expired; holding actual pose'
                self._grace_deadline = wall_now + self.config.planner_hold_grace

        if self.state == 'HOLD_GRACE' and wall_now > self._grace_deadline:
            return (self._release_action('planner grace expired; releasing to POSCTL'),)

        if self.state == 'TAKEOFF':
            if wall_now - self._transition_start > self.config.takeoff_timeout:
                action = self._release_action('takeoff timeout; releasing to POSCTL')
                return (action,)
            actual = self._pose['pose']
            distance = math.sqrt((actual.x-self.target.x)**2 + (actual.y-self.target.y)**2
                                 + (actual.z-self.target.z)**2)
            if distance <= self.config.takeoff_tolerance:
                self._settled_since = wall_now if self._settled_since is None else self._settled_since
                if wall_now - self._settled_since >= self.config.takeoff_settle_time:
                    self.state, self.reason = 'HOLD', 'takeoff target reached; holding'
            else:
                self._settled_since = None

        if self._rc['mode'] == 'command':
            self._rc_hold_captured = False
        if self.state == 'HOLD' and self._rc['mode'] == 'hold' and not self._rc_hold_captured:
            self.target = self._pose['pose']
            self._last_output = self.target
            self.reason = 'RC hold captured measured pose'
            self._rc_hold_captured = True
        return self._stream_actions(wall_now)

    def snapshot(self):
        valid = self._validity(self._last_ros, self._last_wall)
        ready = (self.state in self.OWNED and not self._gate_reason(
            self._last_ros, self._last_wall) and self._fcu.get('mode') == 'OFFBOARD')
        return Status(self.state, self.reason, bool(ready), self.request_id, self.target,
                      valid['fcu'], valid['pose'], valid['rc'], valid['base'], valid['external'])
