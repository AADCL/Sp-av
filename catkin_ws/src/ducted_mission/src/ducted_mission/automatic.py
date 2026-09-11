"""Explicitly started flight sequence. No ROS calls, arming, or background retries."""
from dataclasses import dataclass
import math


def matched_cloud_pose(stamp, history, max_skew=.05):
    if not history or not math.isfinite(stamp):return None
    sample=min(history,key=lambda item:abs(item[0]-stamp))
    return sample[1] if abs(sample[0]-stamp)<=max_skew else None


@dataclass(frozen=True)
class AutoConfig:
    enabled: bool = False
    takeoff_rise: float = 1.1
    finish: str = 'hold'
    transition_timeout: float = 8.
    takeoff_timeout: float = 25.
    mission_timeout: float = 180.
    landing_timeout: float = 70.
    tolerance: float = .1
    speed_tolerance: float = .1
    maximum_z: float = 3.

    def __post_init__(self):
        if type(self.enabled) is not bool or self.finish not in ('hold','land'):
            raise ValueError('enabled must be boolean; finish must be hold or land')
        for name in ('takeoff_rise','transition_timeout','takeoff_timeout','mission_timeout',
                     'landing_timeout','tolerance','speed_tolerance','maximum_z'):
            v=getattr(self,name)
            if isinstance(v,bool) or not math.isfinite(float(v)) or v<=0:
                raise ValueError(name+' must be finite and positive')


@dataclass(frozen=True)
class Observation:
    healthy: bool = False
    reason: str = 'telemetry unavailable'
    armed: bool = False
    on_ground: bool = False
    in_air: bool = False
    offboard: bool = False
    controller: str = ''
    controller_ready: bool = False
    mission: str = ''
    mission_session: str = ''
    x: float = 0.
    y: float = 0.
    z: float = 0.
    speed: float = 0.
    corridor_clear: bool = False
    target_agl_safe: bool = True


@dataclass(frozen=True)
class AutoAction:
    generation: int
    command: str
    z: float = 0.


class AutomaticFlight:
    ACTIVE=('ENGAGING','WAIT_OFFBOARD','TAKEOFF_REQUEST','WAIT_TAKEOFF',
            'MISSION_REQUEST','WAIT_MISSION','RUNNING','LAND_REQUEST','LANDING')

    def __init__(self, config):
        self.config=config
        self.state='IDLE'
        self.reason='explicit start required'
        self.generation=0
        self.pending=None
        self.entered=0.
        self.target_z=0.
        self.initial_session=''
        self.active_session=''
        self.seen_takeoff=False
        self.fault_pending=False

    def _action(self,command,state,now):
        self.state=state;self.entered=now
        self.pending=AutoAction(self.generation,command,self.target_z)
        return self.pending

    def start(self,o,now):
        reason=''
        if not self.config.enabled:reason='automatic commands disabled'
        elif self.state not in ('IDLE','SUCCEEDED','ABORTED'):reason='sequence active or fault latched'
        elif not o.healthy:reason=o.reason
        elif not o.armed or not o.on_ground:reason='operator-armed aircraft with fresh ON_GROUND required'
        elif o.controller!='DISABLED':reason='controller must be DISABLED before automatic start'
        elif o.mission not in ('IDLE','SUCCEEDED','CANCELED'):reason='waypoint mission is already active'
        elif not o.corridor_clear:reason='takeoff corridor is not clear'
        elif not o.target_agl_safe:reason='takeoff height cannot enter the configured navigation AGL envelope'
        elif not all(math.isfinite(v) for v in (o.x,o.y,o.z,o.speed,now)):reason='invalid starting observation'
        elif o.z+self.config.takeoff_rise>self.config.maximum_z:reason='takeoff target exceeds absolute height limit'
        if reason:
            self.reason=reason;return False,None
        self.generation+=1;self.fault_pending=False
        self.target_z=o.z+self.config.takeoff_rise
        self.initial_session=o.mission_session;self.active_session='';self.seen_takeoff=False
        self.reason='prestream and OFFBOARD requested'
        return True,self._action('engage','ENGAGING',now)

    def dispatch_current(self,action):
        return self.pending==action and action.generation==self.generation

    def cancel(self,now,reason='operator canceled'):
        if self.state in ('IDLE','ABORTED','FAULT','SUCCEEDED'):
            return ()
        self.generation+=1;self.reason=reason
        return (self._action('stop','STOPPING',now),)

    def ack(self,generation,command,accepted,message,now):
        if not self.pending or generation!=self.generation or command!=self.pending.command:
            return
        self.pending=None;self.entered=now
        if command=='fault_stop':
            self.state='FAULT'
            if not accepted:self.reason+='; stop outcome: '+message
            return
        if not accepted:
            # A timed-out ROS call may have executed. Never retry a flight
            # transition or permit another automatic start in this process.
            self.state='FAULT';self.reason=message or command+' rejected'
            self.fault_pending=command!='stop'
            return
        self.state={'engage':'WAIT_OFFBOARD','takeoff':'WAIT_TAKEOFF',
                    'mission_start':'WAIT_MISSION','land':'LANDING','stop':'ABORTED'}[command]
        if command=='stop':self.reason+='; sequence stopped; explicit restart required'

    def tick(self,o,now):
        if self.state=='FAULT' and self.fault_pending:
            self.fault_pending=False;self.generation+=1
            # Remain fault-latched after the stop attempt, including success.
            self.pending=AutoAction(self.generation,'fault_stop',self.target_z)
            return (self.pending,)
        if self.state not in self.ACTIVE:return ()
        landing=self.state in ('LAND_REQUEST','LANDING')
        if not o.healthy:
            return self.cancel(now,o.reason)
        if not landing and not o.armed:
            return self.cancel(now,'arming lost')
        timeout=(self.config.landing_timeout if landing else
                 self.config.takeoff_timeout if self.state in ('TAKEOFF_REQUEST','WAIT_TAKEOFF') else
                 self.config.mission_timeout if self.state=='RUNNING' else self.config.transition_timeout)
        if now-self.entered>timeout or now<self.entered:
            return self.cancel(now,'automatic phase timeout or clock rollback')
        if self.state in ('WAIT_OFFBOARD','TAKEOFF_REQUEST','WAIT_TAKEOFF') and not o.corridor_clear:
            return self.cancel(now,'takeoff corridor blocked or unavailable')
        if self.pending:return ()
        if self.state=='WAIT_OFFBOARD' and o.offboard and o.controller_ready and o.controller=='HOLD':
            return (self._action('takeoff','TAKEOFF_REQUEST',now),)
        if self.state=='WAIT_TAKEOFF':
            self.seen_takeoff |= o.controller=='TAKEOFF'
            if (self.seen_takeoff and o.in_air and o.controller=='HOLD' and o.controller_ready
                    and abs(o.z-self.target_z)<=self.config.tolerance and o.speed<=self.config.speed_tolerance):
                return (self._action('mission_start','MISSION_REQUEST',now),)
        if self.state=='WAIT_MISSION':
            if o.mission_session!=self.initial_session and o.mission in ('DISPATCHING','RUNNING','DWELLING','SUCCEEDED'):
                self.active_session=o.mission_session;self.state='RUNNING';self.entered=now
        if self.state=='RUNNING':
            if o.mission_session!=self.active_session:
                return self.cancel(now,'waypoint mission ownership changed')
            if o.mission in ('PAUSED','FAILED','CANCELED'):
                return self.cancel(now,'waypoint mission '+o.mission.lower())
            if o.mission=='SUCCEEDED' and o.controller=='HOLD' and o.controller_ready:
                if self.config.finish=='land':return (self._action('land','LAND_REQUEST',now),)
                self.state='SUCCEEDED';self.reason='waypoints complete; holding final position'
        if self.state=='LANDING' and o.on_ground and not o.armed and o.controller=='DISABLED':
            self.state='SUCCEEDED';self.reason='landed and disarmed feedback confirmed'
        return ()
