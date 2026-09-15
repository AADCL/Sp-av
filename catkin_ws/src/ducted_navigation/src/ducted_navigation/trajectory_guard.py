"""Validate EGO-Planner's certified local reference for a position controller.

The controller consumes poses; this layer checks the actual straight command
segment after slew limiting instead of claiming acceleration feed-forward.
"""
import math
from .planner import Planner, PlanResult, Pose, finite


class TrajectoryGuard(Planner):
    def reject(self, snapshot, reason):
        # No unchecked hold pose is emitted on RPC/map failure. The flight
        # controller owns its bounded target-loss hold and mode fallback.
        result = PlanResult("BLOCKED", reason, snapshot.current,
                            snapshot.current.yaw, -math.inf, 0.,
                            snapshot.terrain.agl, 0., False, False)
        return self._finish(result, snapshot)

    def envelope(self, snapshot):
        reason = self._invalid_reason(snapshot)
        if reason:
            raise ValueError(reason)
        speed = math.sqrt(sum(v*v for v in (snapshot.velocity.x,
                                           snapshot.velocity.y, snapshot.velocity.z)))
        braking = (speed+self.config.snapshot_speed_margin)**2/(2*self.config.braking_acceleration)
        inflation = self.config.snapshot_motion_margin + max(
            self.hull.radius+self.config.obstacle_clearance+braking,
            self.config.min_passage_width/2)
        ground = snapshot.current.z-snapshot.terrain.agl
        if snapshot.terrain.source == 'TAKEOFF_DATUM':
            # The datum is a centre-position reference, not an observed floor.
            # Real floor/ceiling points still undergo the full swept-sphere test.
            minimum = ground+self.config.min_agl
            maximum = ground+self.config.max_agl
        else:
            minimum = ground+self.config.min_agl-self.hull.bottom+self.config.floor_clearance+self.config.snapshot_motion_margin
            maximum = ground+self.config.max_agl-self.hull.top-self.config.ceiling_clearance-self.config.snapshot_motion_margin
        if not minimum <= snapshot.current.z <= maximum:
            raise ValueError("current pose violates height envelope")
        return inflation, minimum, maximum

    def _slew_reference_velocity(self, snapshot, target, dt):
        previous=self._previous_command_velocity
        if previous is None:
            previous=(snapshot.velocity.x,snapshot.velocity.y,snapshot.velocity.z)
        desired=tuple((getattr(target,a)-getattr(snapshot.current,a))/dt for a in ('x','y','z'))
        delta=tuple(desired[i]-previous[i] for i in range(3))
        change=math.sqrt(sum(v*v for v in delta))
        ratio=min(1.,self.config.max_acceleration*dt/change) if change else 1.
        velocity=tuple(previous[i]+ratio*delta[i] for i in range(3))
        if (math.sqrt(sum(v*v for v in velocity))>self.config.max_speed+1e-9 or
                abs(velocity[2])>self.config.max_vertical_speed+1e-9):
            return None
        return Pose(*(getattr(snapshot.current,a)+velocity[i]*dt for i,a in enumerate(('x','y','z'))),target.yaw)

    def plan_reference(self, snapshot, reference, route_length):
        try:
            inflation, minimum, maximum = self.envelope(snapshot)
        except ValueError as error:
            return self.reject(snapshot, str(error))
        if (not self._pose_valid(reference) or not finite(route_length)
                or route_length < 0 or not minimum <= reference.z <= maximum):
            return self.reject(snapshot, "invalid planner reference or height envelope")
        obstacles = self._expanded_obstacles(snapshot)
        current = snapshot.current
        available = self.config.sensor_horizon-inflation
        if (available <= 0 or self._point_collision(current, obstacles, inflation)
                or self._segment_clearance(current, reference, obstacles, inflation)<0):
            return self.reject(snapshot, "reference swept volume intersects live obstacle")
        dt = .1 if self._previous_stamp is None else snapshot.stamp-self._previous_stamp
        if dt<=0 or dt>.5:
            return self.reject(snapshot, "invalid trajectory command time interval")
        speed = self._speed(snapshot)
        distance = math.sqrt((reference.x-current.x)**2+(reference.y-current.y)**2+(reference.z-current.z)**2)
        # Reserve the distance needed to stop before the certified reference.
        # A constant cruise step near the endpoint can demand an instantaneous
        # stop and make the vector-acceleration guard revoke the stream.
        acceleration=self.config.max_acceleration
        stopping_speed=max(0.,math.sqrt((acceleration*dt)**2+2*acceleration*distance)-acceleration*dt)
        speed=min(speed,stopping_speed)
        limit = min(speed*dt, self.config.waypoint_step, available)
        if abs(reference.z-current.z)>1e-12:
            limit=min(limit,distance*self.config.max_vertical_speed*dt/abs(reference.z-current.z))
        target=self._limit_distance(current,reference,limit)
        desired_target=target
        target=self._limit_vector_acceleration(snapshot,desired_target,dt)
        if target is None:
            # A replan can rotate its tangent faster than the command may turn.
            # Slew the velocity vector, then collision-check the ACTUAL chord.
            # Never substitute a chord pointing straight at the final goal.
            target=self._slew_reference_velocity(snapshot,desired_target,dt)
        if target is None or not minimum <= target.z <= maximum:
            return self.reject(snapshot,"reference cannot satisfy command acceleration or height")
        if self._segment_clearance(current,target,obstacles,inflation)<0:
            return self.reject(snapshot,"limited command swept segment blocked")
        actual_goal_distance=math.sqrt((snapshot.goal.x-current.x)**2+(snapshot.goal.y-current.y)**2+(snapshot.goal.z-current.z)**2)
        terminal=actual_goal_distance<=self.config.goal_tolerance
        if self._best_goal_distance is None or route_length<self._best_goal_distance-self.config.progress_epsilon:
            self._best_goal_distance=route_length
            self._last_progress_stamp=snapshot.stamp
        elif not terminal and snapshot.stamp-self._last_progress_stamp>self.config.progress_timeout:
            return self.reject(snapshot,"local goal progress timeout")
        self._blocked_since=None
        return self._finish(PlanResult(
            "GOAL_REACHED" if terminal else "AVOIDING",
            "goal tolerance reached" if terminal else "following guarded EGO-Planner reference",
            target,reference.yaw, self._segment_clearance(current,target,obstacles,inflation),
            1. if terminal else 0.,snapshot.terrain.agl+target.z-current.z,speed,True),snapshot)
