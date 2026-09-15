#include "ducted_offboard/task_machine.hpp"
#include <cmath>
#include <stdexcept>

namespace ducted_offboard {
TaskMachine::TaskMachine(const ControllerConfig& c):config_(c) {validateDirectConfig(c);}
void TaskMachine::issue(IntentKind kind,double now,const Pose4d& target) {
  state_.intent.kind=kind;state_.intent.issued_at=now;state_.intent.target=target;
  ++state_.intent.request_id;arrival_started_=-1;
}
bool TaskMachine::load(const std::vector<Waypoint>& points,bool sequence) {
  if(points.empty()||points.size()>10000)return false;
  for(const auto& p:points)resolveWaypoint(p,Pose4d{});
  const auto p=state_.phase;
  if(p==Phase::LAND_REQUEST||p==Phase::LANDING||p==Phase::WAIT_DISARM||p==Phase::ERROR)return false;
  if(sequence&&p!=Phase::BOOT_WAIT_ZERO&&p!=Phase::IDLE_GROUND&&p!=Phase::HOLDING&&p!=Phase::PAUSED)return false;
  mission_=points;sequence_=sequence;state_.waypoint_count=points.size();state_.waypoint_index=0;
  state_.reason=sequence?"mission sequence loaded":"single target loaded";
  if(p==Phase::GUIDING&&!sequence)guide(last_tick_);
  return true;
}
bool TaskMachine::matched(const ExecutionFeedback& f) const {
  return f.request_id==state_.intent.request_id&&f.generation==state_.intent.generation;
}
void TaskMachine::guide(double now) {
  if(mission_.empty()||!origin_valid_) {
    state_.reason="no task loaded; load target then send a new state=2 command";return;
  }
  if(state_.waypoint_index>=mission_.size())state_.waypoint_index=0;
  state_.target=resolveWaypoint(mission_[state_.waypoint_index],origin_);
  issue(IntentKind::TRACK,now,state_.target);target_started_=now;
  state_.phase=Phase::GUIDING;state_.reason="guiding";
}
TaskSnapshot TaskMachine::tick(double now,int command,const ExecutionFeedback& f) {
  if(!std::isfinite(now)||(last_tick_>=0&&now<last_tick_))throw std::invalid_argument("invalid task clock");
  last_tick_=now;
  const bool feedback_fresh=f.stamp>=0&&now>=f.stamp&&now-f.stamp<=config_.task_timeout;
  if(feedback_fresh) {
    if(f.output.reference_valid&&f.generation==state_.intent.generation) {
      origin_=f.origin;origin_valid_=true;
    }
    if(f.output.phase==Phase::ERROR) {
      state_.phase=Phase::ERROR;state_.reason=f.output.reason;
    } else if(f.authority_lost&&state_.phase!=Phase::BOOT_WAIT_ZERO&&
              state_.phase!=Phase::PAUSED&&state_.phase!=Phase::IDLE_GROUND) {
      state_.phase=f.sensors.armed?Phase::PAUSED:Phase::IDLE_GROUND;
      state_.reason=f.output.reason;issue(f.sensors.armed?IntentKind::HOLD:IntentKind::WAIT,now);
    } else if(matched(f)) {
      if(state_.phase==Phase::TAKEOFF&&f.output.phase==Phase::HOLDING) {
        state_.phase=Phase::HOLDING;state_.reason=f.output.reason;state_.target=f.output.setpoint;
      } else if((state_.phase==Phase::TAKEOFF||state_.phase==Phase::GUIDING)&&f.output.phase==Phase::PAUSED) {
        state_.phase=Phase::PAUSED;state_.reason=f.output.reason;issue(IntentKind::HOLD,now);
      } else if(state_.phase==Phase::LAND_REQUEST&&f.output.phase==Phase::IDLE_GROUND&&!f.sensors.armed) {
        state_.phase=Phase::IDLE_GROUND;state_.reason=f.output.reason;origin_valid_=false;
        state_.waypoint_index=0;if(!sequence_)mission_.clear();state_.waypoint_count=mission_.size();
        issue(IntentKind::WAIT,now);
      } else if(state_.phase==Phase::GUIDING&&f.output.phase==Phase::GUIDING) {
        if(now-target_started_>config_.waypoint_timeout) {
          state_.phase=Phase::PAUSED;state_.reason="waypoint timeout; holding current pose";issue(IntentKind::HOLD,now);
        } else if(withinArrival(state_.target,f.sensors,config_)) {
          if(arrival_started_<0)arrival_started_=now;
          else if(now-arrival_started_>=config_.arrival_dwell) {
            if(state_.waypoint_index+1<mission_.size()) {++state_.waypoint_index;guide(now);state_.reason="advancing to next waypoint";}
            else {state_.phase=Phase::HOLDING;state_.reason="mission complete; waiting for state=3";issue(IntentKind::HOLD,now);}
          }
        } else arrival_started_=-1;
      }
    }
  } else if(state_.phase==Phase::GUIDING||state_.phase==Phase::TAKEOFF) {
    state_.phase=Phase::PAUSED;state_.reason="executor feedback stale; explicit command required";issue(IntentKind::HOLD,now);
  }
  if(command>=0&&command<=3&&command!=state_.command) {
    state_.command=command;
    if(command==0) {
      zero_seen_=true;
      if(state_.phase!=Phase::ERROR&&state_.phase!=Phase::LAND_REQUEST) {
        const bool air=feedback_fresh&&f.sensors.armed;
        state_.phase=air?Phase::PAUSED:Phase::IDLE_GROUND;
        state_.reason=air?"paused by state=0":"idle";
        issue(air?IntentKind::HOLD:IntentKind::WAIT,now);
      }
    } else if(!zero_seen_)state_.reason="state=0 must be observed before accepting a command";
    else if(command==1) {
      if(state_.phase==Phase::IDLE_GROUND&&feedback_fresh&&!f.sensors.armed) {
        ++state_.intent.generation;origin_valid_=false;state_.waypoint_index=0;
        state_.phase=Phase::TAKEOFF;state_.reason="takeoff requested; waiting for prestream readiness";
        issue(IntentKind::TAKEOFF,now);
      } else state_.reason="takeoff rejected outside idle ground state";
    } else if(command==2) {
      if((state_.phase==Phase::HOLDING||state_.phase==Phase::PAUSED)&&feedback_fresh&&
         f.sensors.armed&&f.sensors.mode=="OFFBOARD"&&validSensors(f.sensors))guide(now);
      else state_.reason="guiding command rejected outside an airborne hold";
    } else if(command==3&&state_.phase!=Phase::LAND_REQUEST) {
      if(feedback_fresh&&f.sensors.state_fresh&&f.sensors.armed) {
        state_.phase=Phase::LAND_REQUEST;state_.reason="requesting AUTO.LAND";issue(IntentKind::LAND,now);
      } else state_.reason="landing requires fresh armed FCU feedback";
    }
  }
  state_.intent.issued_at=now;
  return state_;
}
ControllerOutput combinedStatus(const TaskSnapshot& t,const ExecutionFeedback& f,double now) {
  auto o=f.output;o.task_phase=phaseName(t.phase);o.executor_phase=phaseName(f.output.phase);
  o.task_generation=t.intent.generation;o.task_heartbeat_age=t.intent.issued_at<0?-1:std::max(0.,now-t.intent.issued_at);
  o.waypoint_index=t.waypoint_index;o.waypoint_count=t.waypoint_count;
  if(f.output.phase==Phase::IDLE_GROUND&&t.phase==Phase::BOOT_WAIT_ZERO)o.phase=Phase::BOOT_WAIT_ZERO;
  if(f.output.phase!=Phase::ERROR&&!f.authority_lost&&f.output.phase!=Phase::TAKEOFF&&
      f.output.phase!=Phase::OFFBOARD_REQUEST&&f.output.phase!=Phase::ARMING&&
      f.output.phase!=Phase::LAND_REQUEST&&f.output.phase!=Phase::LANDING&&f.output.phase!=Phase::WAIT_DISARM) {
    if(t.phase==Phase::HOLDING||t.phase==Phase::PAUSED)o.phase=t.phase;
    if(t.phase==o.phase)o.reason=t.reason;
  }
  if(t.phase==Phase::GUIDING&&f.output.phase==Phase::GUIDING&&f.request_id==t.intent.request_id)o.final_target=t.target;
  o.actual_state=(o.phase==Phase::TAKEOFF||o.phase==Phase::OFFBOARD_REQUEST||o.phase==Phase::ARMING)?1:
      o.phase==Phase::GUIDING?2:(o.phase==Phase::LAND_REQUEST||o.phase==Phase::LANDING||o.phase==Phase::WAIT_DISARM)?3:0;
  o.planner_state="FROZEN";o.planner_request_id.clear();o.planner_ready=false;o.planner_target_age=-1;
  return o;
}
}  // namespace ducted_offboard
