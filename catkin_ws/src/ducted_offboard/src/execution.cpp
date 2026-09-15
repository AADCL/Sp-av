#include "ducted_offboard/execution.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ducted_offboard {
namespace {
bool finitePose(const Pose4d& p) {
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(p.yaw);
}
}
bool validSensors(const SensorState& s) {
  return s.connected&&s.state_fresh&&s.pose_fresh&&s.velocity_fresh&&
      finitePose(s.pose)&&std::isfinite(s.speed)&&s.speed>=0;
}
bool withinArrival(const Pose4d& p,const SensorState& s,const ControllerConfig& c) {
  return validSensors(s)&&std::hypot(p.x-s.pose.x,p.y-s.pose.y)<=c.horizontal_tolerance&&
      std::abs(p.z-s.pose.z)<=c.vertical_tolerance&&
      std::abs(wrapAngle(p.yaw-s.pose.yaw))<=c.yaw_tolerance&&s.speed<=c.speed_tolerance;
}
void validateDirectConfig(const ControllerConfig& c) {
  if(c.execution_mode!="direct") throw std::invalid_argument("EGO 已冻结 (EGO is frozen); execution_mode must be direct");
  for(double value:{c.task_timeout,c.feedback_timeout,c.service_timeout,c.takeoff_height,
      c.horizontal_speed,c.vertical_speed,c.yaw_rate,c.takeoff_timeout,c.waypoint_timeout,c.landing_timeout})
    if(!std::isfinite(value)||value<=0)throw std::invalid_argument("control limits and timeouts must be finite and positive");
  for(double value:{c.prestream_duration,c.retry_interval,c.horizontal_tolerance,c.vertical_tolerance,
      c.yaw_tolerance,c.speed_tolerance,c.arrival_dwell,c.ground_disarm_delay})
    if(!std::isfinite(value)||value<0)throw std::invalid_argument("control thresholds must be finite and nonnegative");
  if(c.max_attempts<1)throw std::invalid_argument("max_attempts must be positive");
}
FlightExecutor::FlightExecutor(const ControllerConfig& c):config_(c) {validateDirectConfig(c);}
bool FlightExecutor::landing() const {
  return phase_==Phase::LAND_REQUEST||phase_==Phase::LANDING||phase_==Phase::WAIT_DISARM;
}
bool FlightExecutor::fresh(const ExecutionIntent& i,double now) const {
  return i.issued_at>=0&&std::isfinite(i.issued_at)&&now>=i.issued_at&&now-i.issued_at<=config_.task_timeout;
}
void FlightExecutor::cancelAction() {
  action_pending_=waiting_feedback_=false;action_id_=0;++next_action_id_;
}
void FlightExecutor::fail(const std::string& reason,bool exit) {
  cancelAction();phase_=Phase::ERROR;reason_=reason;should_exit_=exit;takeoff_pending_=false;
}
void FlightExecutor::actionPhase(Phase p,double now,const std::string& reason) {
  cancelAction();phase_=p;phase_started_=now;attempts_=0;next_action_time_=now;reason_=reason;
}
void FlightExecutor::hold(const SensorState& s,Phase p,const std::string& reason) {
  cancelAction();takeoff_pending_=false;phase_=p;reason_=reason;
  if(finitePose(s.pose))setpoint_=target_=s.pose;
  arrival_started_=-1;
}
void FlightExecutor::acceptActionResult(std::uint64_t id,bool transport,bool accepted,double now) {
  if(!action_pending_||id!=action_id_)return;
  if(now<action_started_||now-action_started_>config_.service_timeout) {
    fail("service result arrived after its deadline");return;
  }
  action_pending_=false;action_id_=0;
  waiting_feedback_=transport&&accepted;
  feedback_deadline_=now+config_.feedback_timeout;
}
ActionRequest FlightExecutor::requestAction(ActionKind kind,double now,bool exit) {
  if(action_pending_) {
    if(now-action_started_>config_.service_timeout)fail(actionName(kind)+" service call did not return");
    return {};
  }
  if(waiting_feedback_&&now<feedback_deadline_)return {};
  waiting_feedback_=false;
  if(now<next_action_time_)return {};
  if(attempts_>=config_.max_attempts) {
    fail(actionName(kind)+" failed after "+std::to_string(config_.max_attempts)+" attempts",exit);return {};
  }
  action_id_=next_action_id_++;action_pending_=true;action_started_=now;
  next_action_time_=now+config_.retry_interval;++attempts_;
  return {kind,action_id_,accepted_.generation,accepted_.request_id};
}
bool FlightExecutor::actionAllowed(const ActionRequest& a,double now,const SensorState& s,
                                   const ExecutionIntent& latest) const {
  if(!action_pending_||a.id!=action_id_||a.request_id!=accepted_.request_id||
     a.generation!=accepted_.generation||now-action_started_>config_.service_timeout||
     !s.connected||!s.state_fresh)return false;
  if(!landing()&&(!fresh(latest,now)||latest.request_id!=accepted_.request_id||
      latest.generation!=accepted_.generation||authority_lost_))return false;
  if(a.kind==ActionKind::SET_OFFBOARD)return phase_==Phase::OFFBOARD_REQUEST&&validSensors(s)&&!s.armed;
  if(a.kind==ActionKind::ARM)return phase_==Phase::ARMING&&validSensors(s)&&s.extended_fresh&&
      s.on_ground&&!s.armed&&s.mode=="OFFBOARD";
  if(a.kind==ActionKind::SET_LAND)return phase_==Phase::LAND_REQUEST&&s.armed;
  if(a.kind==ActionKind::DISARM)return phase_==Phase::WAIT_DISARM&&s.extended_fresh&&s.on_ground&&s.armed;
  return false;
}

ExecutionFeedback FlightExecutor::tick(double now,const SensorState& s,const ExecutionIntent& incoming) {
  if(!std::isfinite(now))throw std::invalid_argument("executor clock must be finite");
  if(last_tick_>=0&&now<last_tick_)fail("monotonic clock moved backwards");
  const bool healthy=validSensors(s);
  if(phase_==Phase::IDLE_GROUND) {
    if(healthy&&s.extended_fresh&&s.on_ground&&!s.armed) {
      if(prestream_started_<0)prestream_started_=now;
      setpoint_=target_=s.pose;
    } else prestream_started_=-1;
  }
  if(fresh(incoming,now)&&incoming.request_id>blocked_through_&&
      incoming.request_id>=accepted_.request_id&&incoming.generation>=accepted_.generation) {
    if(incoming.request_id==accepted_.request_id) {
      // Only renew the lease. An identifier cannot mutate an accepted command.
      if(incoming.kind==accepted_.kind&&incoming.generation==accepted_.generation&&
         incoming.target.x==accepted_.target.x&&incoming.target.y==accepted_.target.y&&
         incoming.target.z==accepted_.target.z&&incoming.target.yaw==accepted_.target.yaw)
        accepted_.issued_at=incoming.issued_at;
    } else if(!landing()&&finitePose(incoming.target)) {
      const bool allowed=incoming.kind==IntentKind::LAND ? s.state_fresh&&s.armed :
          phase_!=Phase::ERROR&&(incoming.kind!=IntentKind::TRACK||
          (origin_valid_&&incoming.generation==accepted_.generation&&s.armed&&s.mode=="OFFBOARD"&&
           (phase_==Phase::HOLDING||phase_==Phase::PAUSED||phase_==Phase::GUIDING)))&&
          (incoming.kind!=IntentKind::TAKEOFF||(phase_==Phase::IDLE_GROUND&&!s.armed));
      if(allowed) {
        accepted_=incoming;authority_lost_=false;cancelAction();
        if(incoming.kind==IntentKind::TAKEOFF) {
          takeoff_pending_=true;reason_="takeoff requested; waiting for prestream readiness";
        } else if(incoming.kind==IntentKind::TRACK) {
          takeoff_pending_=false;phase_=Phase::GUIDING;target_=incoming.target;reason_="tracking task target";
        } else if(incoming.kind==IntentKind::LAND) {
          takeoff_pending_=false;if(finitePose(s.pose))setpoint_=target_=s.pose;
          ground_started_=-1;actionPhase(Phase::LAND_REQUEST,now,"requesting AUTO.LAND");
        } else {
          hold(s,s.armed?Phase::PAUSED:Phase::IDLE_GROUND,s.armed?"holding by task request":"idle");
          if(!s.armed)origin_valid_=false;
        }
      }
    }
  }
  if(!landing()&&phase_!=Phase::ERROR&&accepted_.request_id>0&&
      !fresh(accepted_,now)&&!authority_lost_) {
    authority_lost_=true;blocked_through_=accepted_.request_id;
    hold(s,s.armed?Phase::PAUSED:Phase::IDLE_GROUND,"task heartbeat expired; explicit command required");
    if(!s.armed)origin_valid_=false;
  }
  // An already dispatched arm operation may be confirmed after lease revocation.
  // Capture a hover independently of the stalled task rather than continue takeoff.
  if(authority_lost_&&phase_==Phase::IDLE_GROUND&&s.armed&&healthy&&s.mode=="OFFBOARD")
    hold(s,Phase::PAUSED,"late arm feedback after task revocation; holding current pose");
  if((phase_==Phase::TAKEOFF||phase_==Phase::GUIDING||phase_==Phase::HOLDING||phase_==Phase::PAUSED)&&!s.armed)
    fail("unexpected disarm outside landing; automatic output stopped");
  if(phase_!=Phase::IDLE_GROUND&&phase_!=Phase::ERROR&&!landing()&&!healthy)
    fail(!s.pose_fresh?"pose unavailable or stale":!s.velocity_fresh?"velocity unavailable or stale":"FCU state unavailable or stale");
  if((phase_==Phase::TAKEOFF||phase_==Phase::GUIDING||phase_==Phase::PAUSED||phase_==Phase::HOLDING)&&s.mode!="OFFBOARD")
    fail("operator or failsafe left OFFBOARD; automatic output stopped");
  if(phase_==Phase::IDLE_GROUND&&takeoff_pending_&&fresh(accepted_,now)&&healthy&&
      s.extended_fresh&&s.on_ground&&!s.armed&&prestream_started_>=0&&
      now-prestream_started_>=config_.prestream_duration) {
    origin_=s.pose;origin_valid_=true;setpoint_=target_=s.pose;takeoff_pending_=false;
    actionPhase(Phase::OFFBOARD_REQUEST,now,"requesting OFFBOARD");
  }
  ActionRequest action;
  if(phase_==Phase::OFFBOARD_REQUEST) {
    if(s.mode=="OFFBOARD")actionPhase(Phase::ARMING,now,"OFFBOARD confirmed; requesting arm");
    else action=requestAction(ActionKind::SET_OFFBOARD,now,true);
  }
  if(phase_==Phase::ARMING) {
    if(s.mode!="OFFBOARD")fail("OFFBOARD lost before arming");
    else if(s.armed) {
      cancelAction();phase_=Phase::TAKEOFF;phase_started_=now;arrival_started_=-1;
      target_=origin_;target_.z+=config_.takeoff_height;reason_="taking off";
    } else if(action.kind==ActionKind::NONE)action=requestAction(ActionKind::ARM,now);
  }
  if(phase_==Phase::TAKEOFF||phase_==Phase::GUIDING) {
    // A delayed executor iteration must not make a large jump to catch up.
    const double dt=last_tick_<0?0:std::max(0.,std::min(.1,now-last_tick_));
    setpoint_=advanceSetpoint(setpoint_,target_,dt,config_.horizontal_speed,config_.vertical_speed,config_.yaw_rate);
  }
  if(phase_==Phase::TAKEOFF) {
    if(now-phase_started_>config_.takeoff_timeout)hold(s,Phase::PAUSED,"takeoff timeout; holding current pose");
    else if(withinArrival(target_,s,config_)) {
      if(arrival_started_<0)arrival_started_=now;
      else if(now-arrival_started_>=config_.arrival_dwell)hold(s,Phase::HOLDING,"takeoff complete; waiting for state=2");
    } else arrival_started_=-1;
  } else if(phase_==Phase::LAND_REQUEST) {
    if(!s.state_fresh)fail("FCU state unavailable while requesting AUTO.LAND");
    else if(s.mode=="AUTO.LAND") {
      cancelAction();phase_=Phase::LANDING;phase_started_=now;ground_started_=-1;reason_="AUTO.LAND confirmed";
    } else action=requestAction(ActionKind::SET_LAND,now);
  } else if(phase_==Phase::LANDING||phase_==Phase::WAIT_DISARM) {
    if(!s.state_fresh)fail("FCU state unavailable during landing");
    else if(!s.armed) {
      cancelAction();phase_=Phase::IDLE_GROUND;origin_valid_=false;prestream_started_=-1;
      reason_="landed and disarmed; set state=0 before the next task";
    } else if(phase_==Phase::WAIT_DISARM) {
      if(!s.extended_fresh||!s.on_ground)fail("ground confirmation lost before disarm");
      else action=requestAction(ActionKind::DISARM,now);
    } else if(now-phase_started_>config_.landing_timeout)fail("landing timeout; vehicle remains armed");
    else if(s.extended_fresh&&s.on_ground) {
      if(ground_started_<0)ground_started_=now;
      else if(now-ground_started_>=config_.ground_disarm_delay) {
        actionPhase(Phase::WAIT_DISARM,now,"landed but still armed; requesting normal disarm");
        action=requestAction(ActionKind::DISARM,now);
      }
    } else ground_started_=-1;
  }
  last_tick_=now;
  ExecutionFeedback f;f.stamp=now;f.sensors=s;f.origin=origin_;f.generation=accepted_.generation;
  f.request_id=accepted_.request_id;f.authority_lost=authority_lost_;
  auto& o=f.output;o.phase=phase_;o.reason=reason_;o.setpoint=setpoint_;o.final_target=target_;
  o.action=action;o.reference_valid=origin_valid_;o.takeoff_reference_z=origin_valid_?origin_.z:0;
  o.should_exit=should_exit_;o.exit_code=should_exit_?2:0;o.planner_state="FROZEN";
  o.executor_phase=phaseName(phase_);o.task_generation=accepted_.generation;
  o.task_heartbeat_age=incoming.issued_at<0?-1:std::max(0.,now-incoming.issued_at);
  o.publish_setpoint=healthy&&phase_!=Phase::ERROR&&phase_!=Phase::LANDING&&phase_!=Phase::WAIT_DISARM&&
      (phase_!=Phase::IDLE_GROUND||(!s.armed&&s.extended_fresh&&s.on_ground));
  return f;
}
}  // namespace ducted_offboard
