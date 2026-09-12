"""Bounded takeoff initialization; never a replacement for measured terrain."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class HeightReading:
    valid: bool
    agl: float = math.nan
    source: str = 'UNAVAILABLE'
    measured: bool = False
    reason: str = ''


class TakeoffReference:
    MAX_PREPARATION_AGE = 300.
    MAX_BLIND_TIME = 8.
    MAX_BLIND_AGL = 1.1
    MAX_XY_DRIFT = .15

    def __init__(self, data, run_id):
        if (data.get('confirmed') is not True or data.get('source') != 'operator_ground_contact'
                or data.get('frame_id') != 'odom' or not run_id or data.get('run_id') != run_id):
            raise ValueError('explicit ground-contact confirmation in the current ROS session required')
        try:
            self.anchor = {k:float(data['anchor'][k]) for k in ('x','y','z','yaw')}
            self.ground_z = float(data['ground_z'])
            self.contact_agl = float(data['contact_agl'])
            self.prepared_stamp = float(data['prepared_stamp'])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('invalid contact reference') from error
        if (not all(math.isfinite(v) for v in (*self.anchor.values(), self.ground_z,
                                              self.contact_agl, self.prepared_stamp))
                or not .02 <= self.contact_agl <= .3 or self.prepared_stamp <= 0
                or abs(self.anchor['z']-self.ground_z-self.contact_agl) > .001):
            raise ValueError('contact reference geometry or time is inconsistent')
        self.started = None
        self.last_ros = self.prepared_stamp
        self.last_wall = None
        self.acquired = False
        self.failure = ''
        self.samples = deque(maxlen=32)

    def begin(self, ros_now, wall_now):
        if self.started is not None or self.failure:
            raise ValueError('contact reference already consumed; prepare again')
        if not all(math.isfinite(v) for v in (ros_now, wall_now)):
            raise ValueError('invalid reference activation time')
        if not 0 <= ros_now-self.prepared_stamp <= self.MAX_PREPARATION_AGE:
            raise ValueError('contact preparation expired; prepare again')
        self.started = wall_now
        self.last_ros, self.last_wall = ros_now, wall_now
        self.samples.clear()
        self.acquired = False

    def evaluate(self, x, y, z, yaw, tilt, speed, on_ground, ros_now, wall_now, measured=None):
        reason = self.failure
        numbers = (x,y,z,yaw,tilt,speed,ros_now,wall_now)
        if not all(math.isfinite(v) for v in numbers):
            reason = 'invalid contact-reference observation'
        elif ros_now < self.last_ros or (self.last_wall is not None and wall_now < self.last_wall):
            reason = 'contact-reference clock rollback'
        self.last_ros, self.last_wall = ros_now, wall_now
        agl = z-self.ground_z
        if not reason and self.started is None:
            distance = math.sqrt(sum((v-self.anchor[k])**2 for k,v in (('x',x),('y',y),('z',z))))
            angle = abs((yaw-self.anchor['yaw']+math.pi)%(2*math.pi)-math.pi)
            if not 0 <= ros_now-self.prepared_stamp <= self.MAX_PREPARATION_AGE:
                reason = 'contact preparation expired; prepare again'
            elif not on_ground or distance > .08 or angle > .1 or tilt > .1 or speed > .1:
                reason = 'aircraft moved or is not level on the confirmed ground; prepare again'
        if not reason and self.started is not None and not self.acquired:
            if (wall_now-self.started > self.MAX_BLIND_TIME or agl > self.MAX_BLIND_AGL
                    or agl < self.contact_agl-.03 or math.hypot(x-self.anchor['x'],y-self.anchor['y']) > self.MAX_XY_DRIFT
                    or tilt > .2):
                reason = 'takeoff ground-reference height/time/motion limit exceeded; measured height required'
        if not reason and measured is not None:
            if (not all(math.isfinite(measured[k]) for k in ('ground_z','agl','stamp'))
                    or abs(measured['ground_z']-self.ground_z) > .08):
                reason = 'measured floor conflicts with confirmed takeoff ground'
            elif self.started is not None and not self.acquired:
                stamp = measured['stamp']
                if self.samples and stamp-self.samples[-1] > .25:
                    self.samples.clear()
                if not self.samples or stamp > self.samples[-1]:
                    self.samples.append(stamp)
                self.acquired = len(self.samples) >= 3 and self.samples[-1]-self.samples[0] >= .15-1e-9
        elif measured is None:
            self.samples.clear()
            if self.acquired:
                reason = 'measured terrain lost after takeoff handover'
        if reason:
            self.failure = reason
            return HeightReading(False, reason=reason)
        if self.acquired:
            return HeightReading(True, measured['agl'], 'LIDAR', True)
        return HeightReading(True, agl, 'CONTACT_REFERENCE', False)
