#include "ducted_offboard/core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ducted_offboard {
namespace {

constexpr double kPi = 3.14159265358979323846;

double moveToward(double current, double goal, double maximum_step) {
  const double delta = goal - current;
  if (std::abs(delta) <= maximum_step) {
    return goal;
  }
  return current + std::copysign(maximum_step, delta);
}

bool finitePose(const Pose4d& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.z) && std::isfinite(pose.yaw);
}

}  // namespace

Controller::Controller(const ControllerConfig& config) : config_(config) {
  if ((config_.execution_mode != "direct" && config_.execution_mode != "ego") ||
      (config_.execution_mode == "ego" && config_.session_id.empty()) ||
      !std::isfinite(config_.planner_timeout) || config_.planner_timeout <= 0 ||
      !std::isfinite(config_.planner_first_target_timeout) ||
      config_.planner_first_target_timeout <= 0 ||
      !std::isfinite(config_.planner_future_tolerance) ||
      config_.planner_future_tolerance < 0) {
    throw std::invalid_argument("invalid planner configuration");
  }
  if (config_.prestream_duration < 0.0 || config_.retry_interval < 0.0 ||
      config_.feedback_timeout <= 0.0 || config_.service_timeout <= 0.0 ||
      config_.max_attempts <= 0 || config_.takeoff_height <= 0.0 ||
      config_.horizontal_speed <= 0.0 || config_.vertical_speed <= 0.0 ||
      config_.yaw_rate <= 0.0 || config_.horizontal_tolerance < 0.0 ||
      config_.vertical_tolerance < 0.0 || config_.yaw_tolerance < 0.0 ||
      config_.speed_tolerance < 0.0 || config_.arrival_dwell < 0.0 ||
      config_.takeoff_timeout <= 0.0 || config_.waypoint_timeout <= 0.0 ||
      config_.landing_timeout <= 0.0 || config_.ground_disarm_delay < 0.0) {
    throw std::invalid_argument("invalid controller configuration");
  }
}

void Controller::observeCommand(int command, double) {
  if (command < 0 || command > 3 || command == last_command_) {
    return;
  }
  last_command_ = command;
  pending_command_ = command;
  command_pending_ = true;
}

void Controller::setMission(const std::vector<Waypoint>& mission, bool sequence) {
  raw_mission_ = mission;
  sequence_ = sequence;
  waypoint_index_ = 0;
  if (origin_valid_) {
    resolveMission();
  }
}

bool Controller::replaceSequence(const std::vector<Waypoint>& mission) {
  if (phase_ != Phase::BOOT_WAIT_ZERO && phase_ != Phase::IDLE_GROUND &&
      phase_ != Phase::PAUSED && phase_ != Phase::HOLDING) {
    return false;
  }
  setMission(mission, true);
  mission_replaced_while_paused_ = phase_ == Phase::PAUSED || phase_ == Phase::HOLDING;
  reason_ = mission.empty() ? "empty mission loaded" : "mission sequence loaded";
  return true;
}

bool Controller::replaceSingle(const Waypoint& waypoint) {
  if (phase_ == Phase::LAND_REQUEST || phase_ == Phase::LANDING ||
      phase_ == Phase::WAIT_DISARM || phase_ == Phase::ERROR) {
    return false;
  }
  setMission({waypoint}, false);
  reason_ = "single target loaded";
  // Loading a target while holding never grants permission to execute it.
  if (origin_valid_ && phase_ == Phase::GUIDING) {
    startGuiding(last_tick_ < 0.0 ? 0.0 : last_tick_);
  }
  return true;
}

void Controller::acceptActionResult(std::uint64_t id, bool transport_ok,
                                    bool accepted, double now) {
  if (!action_pending_ || id != pending_action_id_) {
    return;
  }
  action_pending_ = false;
  pending_action_id_ = 0;
  if (transport_ok && accepted) {
    waiting_feedback_ = true;
    feedback_deadline_ = now + config_.feedback_timeout;
  } else {
    waiting_feedback_ = false;
  }
}

bool Controller::inputsReady(const SensorState& sensors) const {
  return sensors.connected && sensors.state_fresh && sensors.pose_fresh &&
         sensors.velocity_fresh && finitePose(sensors.pose) &&
         std::isfinite(sensors.speed);
}

bool Controller::reached(const Pose4d& target,
                         const SensorState& sensors) const {
  return std::hypot(target.x - sensors.pose.x, target.y - sensors.pose.y) <=
             config_.horizontal_tolerance &&
         std::abs(target.z - sensors.pose.z) <= config_.vertical_tolerance &&
         std::abs(wrapAngle(target.yaw - sensors.pose.yaw)) <=
             config_.yaw_tolerance &&
         sensors.speed <= config_.speed_tolerance;
}

void Controller::cancelPendingAction() {
  action_pending_ = false;
  waiting_feedback_ = false;
  pending_action_id_ = 0;
  ++next_action_id_;
}

void Controller::fail(const std::string& reason, bool should_exit) {
  revokePlanner();
  cancelPendingAction();
  phase_ = Phase::ERROR;
  reason_ = reason;
  should_exit_ = should_exit;
  exit_code_ = should_exit ? 2 : 0;
}

void Controller::startActionPhase(Phase phase, double now) {
  cancelPendingAction();
  phase_ = phase;
  phase_started_ = now;
  action_attempts_ = 0;
  next_action_time_ = now;
}

void Controller::startOffboard(double now, const SensorState& sensors) {
  takeoff_requested_ = false;
  mission_origin_ = sensors.pose;
  origin_valid_ = true;
  setpoint_ = sensors.pose;
  final_target_ = sensors.pose;
  resolveMission();
  startActionPhase(Phase::OFFBOARD_REQUEST, now);
  reason_ = "requesting OFFBOARD";
}

void Controller::startTakeoff(double now) {
  phase_ = Phase::TAKEOFF;
  phase_started_ = now;
  target_started_ = now;
  arrival_started_ = -1.0;
  final_target_ = mission_origin_;
  final_target_.z += config_.takeoff_height;
  reason_ = "taking off";
  cancelPendingAction();
}

void Controller::finishTakeoff(double now, const SensorState& sensors) {
  revokePlanner();
  phase_ = Phase::HOLDING;
  phase_started_ = now;
  setpoint_ = sensors.pose;
  final_target_ = setpoint_;
  reason_ = "takeoff complete; waiting for state=2";
}

void Controller::startGuiding(double now) {
  if (mission_.empty()) {
    phase_ = Phase::HOLDING;
    final_target_ = setpoint_;
    reason_ = "no task loaded; load target then send a new state=2 command";
    return;
  }
  if (waypoint_index_ >= mission_.size()) {
    waypoint_index_ = 0;
  }
  phase_ = Phase::GUIDING;
  phase_started_ = now;
  target_started_ = now;
  arrival_started_ = -1.0;
  final_target_ = mission_[waypoint_index_];
  mission_replaced_while_paused_ = false;
  reason_ = "guiding";
  if (config_.execution_mode == "ego") beginPlanner(now);
}

void Controller::pauseAtActual(const SensorState& sensors,
                               const std::string& reason) {
  revokePlanner();
  cancelPendingAction();
  phase_ = Phase::PAUSED;
  setpoint_ = sensors.pose;
  final_target_ = sensors.pose;
  arrival_started_ = -1.0;
  reason_ = reason;
}

void Controller::startLanding(double now, const SensorState& sensors) {
  revokePlanner();
  setpoint_ = sensors.pose;
  final_target_ = sensors.pose;
  ground_started_ = -1.0;
  startActionPhase(Phase::LAND_REQUEST, now);
  reason_ = "requesting AUTO.LAND";
}

void Controller::finishLanding() {
  revokePlanner();
  cancelPendingAction();
  phase_ = Phase::IDLE_GROUND;
  reason_ = "landed and disarmed; set state=0 before the next task";
  origin_valid_ = false;
  takeoff_requested_ = false;
  waypoint_index_ = 0;
  arrival_started_ = -1.0;
  ground_started_ = -1.0;
  if (!sequence_ || raw_mission_.empty()) {
    raw_mission_.clear();
    mission_.clear();
  } else {
    mission_.clear();
  }
}

void Controller::resolveMission() {
  mission_.clear();
  mission_.reserve(raw_mission_.size());
  for (const auto& waypoint : raw_mission_) {
    mission_.push_back(resolveWaypoint(waypoint, mission_origin_));
  }
  waypoint_index_ = 0;
}

void Controller::processCommand(double now, const SensorState& sensors) {
  if (!command_pending_) {
    return;
  }
  const int command = pending_command_;
  command_pending_ = false;
  if (command == 0) {
    zero_seen_ = true;
    takeoff_requested_ = false;
    if (phase_ == Phase::BOOT_WAIT_ZERO) {
      phase_ = Phase::IDLE_GROUND;
      reason_ = "idle";
    } else if (phase_ == Phase::OFFBOARD_REQUEST || phase_ == Phase::ARMING) {
      cancelPendingAction();
      if (sensors.armed) {
        pauseAtActual(sensors, "takeover interrupted; holding");
      } else {
        phase_ = Phase::IDLE_GROUND;
        reason_ = "takeover cancelled";
      }
    } else if (phase_ == Phase::TAKEOFF || phase_ == Phase::GUIDING ||
               (config_.execution_mode == "ego" && phase_ == Phase::HOLDING)) {
      pauseAtActual(sensors, "paused by state=0");
    }
    return;
  }
  if (!zero_seen_) {
    reason_ = "state=0 must be observed before accepting a command";
    return;
  }
  if (command == 1) {
    if (phase_ == Phase::IDLE_GROUND && !sensors.armed) {
      takeoff_requested_ = true;
      reason_ = "takeoff requested; waiting for prestream readiness";
    } else {
      reason_ = "takeoff rejected outside idle ground state";
    }
    return;
  }
  if (command == 2) {
    if ((phase_ == Phase::PAUSED || phase_ == Phase::HOLDING) && sensors.armed &&
        sensors.mode == "OFFBOARD") {
      if (mission_replaced_while_paused_) {
        waypoint_index_ = 0;
      }
      startGuiding(now);
    } else {
      reason_ = "guiding command rejected outside an airborne hold";
    }
    return;
  }
  if (command == 3) {
    if (sensors.armed) {
      startLanding(now, sensors);
    } else {
      reason_ = "already disarmed";
    }
  }
}

ActionRequest Controller::maybeRequest(ActionKind kind, double now,
                                       bool fatal_on_exhaustion) {
  if (action_pending_) {
    if (now - action_started_ > config_.service_timeout) {
      fail(actionName(kind) + " service call did not return", false);
    }
    return {};
  }
  if (waiting_feedback_) {
    if (now < feedback_deadline_) {
      return {};
    }
    waiting_feedback_ = false;
  }
  if (now < next_action_time_) {
    return {};
  }
  if (action_attempts_ >= config_.max_attempts) {
    fail(actionName(kind) + " failed after " +
             std::to_string(config_.max_attempts) + " attempts",
         fatal_on_exhaustion);
    return {};
  }
  ActionRequest request;
  request.kind = kind;
  request.id = next_action_id_++;
  pending_action_id_ = request.id;
  action_pending_ = true;
  action_started_ = now;
  next_action_time_ = now + config_.retry_interval;
  ++action_attempts_;
  return request;
}

void Controller::updateSetpoint(double now) {
  const double dt = last_tick_ < 0.0 ? 0.0 : std::max(0.0, now - last_tick_);
  if (phase_ == Phase::TAKEOFF ||
      (phase_ == Phase::GUIDING && config_.execution_mode == "direct")) {
    setpoint_ = advanceSetpoint(setpoint_, final_target_, dt,
                                config_.horizontal_speed,
                                config_.vertical_speed, config_.yaw_rate);
  }
}

void Controller::updateArrival(double now, const SensorState& sensors,
                               bool takeoff) {
  if (!reached(final_target_, sensors) ||
      (!takeoff && config_.execution_mode == "ego" && planner_state_ != "GOAL_REACHED")) {
    arrival_started_ = -1.0;
    return;
  }
  if (arrival_started_ < 0.0) {
    arrival_started_ = now;
    return;
  }
  if (now - arrival_started_ < config_.arrival_dwell) {
    return;
  }
  arrival_started_ = -1.0;
  if (takeoff) {
    finishTakeoff(now, sensors);
    return;
  }
  if (waypoint_index_ + 1 < mission_.size()) {
    ++waypoint_index_;
    final_target_ = mission_[waypoint_index_];
    target_started_ = now;
    reason_ = "advancing to next waypoint";
    if (config_.execution_mode == "ego") {
      setpoint_ = sensors.pose;
      beginPlanner(now);
    }
  } else {
    if (config_.execution_mode == "ego") {
      revokePlanner();
      setpoint_ = sensors.pose;
    }
    phase_ = Phase::HOLDING;
    phase_started_ = now;
    reason_ = "mission complete; waiting for state=3";
  }
}

ControllerOutput Controller::tick(double now, const SensorState& sensors) {
  if (!std::isfinite(now)) {
    throw std::invalid_argument("tick time must be finite");
  }
  if (last_tick_ >= 0.0 && now < last_tick_) {
    fail("monotonic clock moved backwards", false);
  }

  if (inputsReady(sensors) && sensors.extended_fresh && !sensors.armed &&
      sensors.on_ground) {
    if (prestream_started_ < 0.0) {
      prestream_started_ = now;
    }
    if (phase_ == Phase::BOOT_WAIT_ZERO || phase_ == Phase::IDLE_GROUND) {
      setpoint_ = sensors.pose;
      final_target_ = sensors.pose;
    }
  } else if (phase_ == Phase::BOOT_WAIT_ZERO || phase_ == Phase::IDLE_GROUND) {
    prestream_started_ = -1.0;
  }

  processCommand(now, sensors);

  if (phase_ != Phase::BOOT_WAIT_ZERO && phase_ != Phase::IDLE_GROUND &&
      phase_ != Phase::LAND_REQUEST && phase_ != Phase::LANDING &&
      phase_ != Phase::WAIT_DISARM && phase_ != Phase::ERROR &&
      !inputsReady(sensors)) {
    fail(!sensors.pose_fresh ? "pose unavailable or stale" :
         !sensors.velocity_fresh ? "velocity unavailable or stale" :
         "FCU state unavailable or stale", false);
  }

  if ((phase_ == Phase::TAKEOFF || phase_ == Phase::GUIDING ||
       phase_ == Phase::PAUSED || phase_ == Phase::HOLDING) &&
      sensors.mode != "OFFBOARD") {
    fail("operator or failsafe left OFFBOARD; automatic output stopped", false);
  }

  if (phase_ == Phase::IDLE_GROUND && takeoff_requested_ && inputsReady(sensors) &&
      sensors.extended_fresh && !sensors.armed && sensors.on_ground &&
      prestream_started_ >= 0.0 &&
      now - prestream_started_ >= config_.prestream_duration) {
    startOffboard(now, sensors);
  }

  ActionRequest action;
  if (phase_ == Phase::OFFBOARD_REQUEST) {
    if (sensors.mode == "OFFBOARD") {
      startActionPhase(Phase::ARMING, now);
      reason_ = "OFFBOARD confirmed; requesting arm";
    } else {
      action = maybeRequest(ActionKind::SET_OFFBOARD, now, true);
    }
  }
  if (phase_ == Phase::ARMING) {
    if (sensors.mode != "OFFBOARD") {
      fail("OFFBOARD lost before arming", false);
    } else if (sensors.armed) {
      startTakeoff(now);
    } else if (action.kind == ActionKind::NONE) {
      action = maybeRequest(ActionKind::ARM, now, false);
    }
  }

  updateSetpoint(now);
  if (phase_ == Phase::GUIDING && config_.execution_mode == "ego") {
    updatePlanner(now, sensors);
  }

  if (phase_ == Phase::TAKEOFF) {
    if (now - phase_started_ > config_.takeoff_timeout) {
      pauseAtActual(sensors, "takeoff timeout; holding current pose");
    } else {
      updateArrival(now, sensors, true);
    }
  } else if (phase_ == Phase::GUIDING) {
    if (now - target_started_ > config_.waypoint_timeout) {
      pauseAtActual(sensors, "waypoint timeout; holding current pose");
    } else {
      updateArrival(now, sensors, false);
    }
  } else if (phase_ == Phase::LAND_REQUEST) {
    if (!sensors.state_fresh) {
      fail("FCU state unavailable while requesting AUTO.LAND", false);
    } else if (sensors.mode == "AUTO.LAND") {
      cancelPendingAction();
      phase_ = Phase::LANDING;
      phase_started_ = now;
      ground_started_ = -1.0;
      reason_ = "AUTO.LAND confirmed";
    } else {
      action = maybeRequest(ActionKind::SET_LAND, now, false);
    }
  } else if (phase_ == Phase::LANDING) {
    if (!sensors.state_fresh) {
      fail("FCU state unavailable during landing", false);
    } else if (!sensors.armed) {
      finishLanding();
    } else if (now - phase_started_ > config_.landing_timeout) {
      fail("landing timeout; vehicle remains armed", false);
    } else if (sensors.extended_fresh && sensors.on_ground) {
      if (ground_started_ < 0.0) {
        ground_started_ = now;
      } else if (now - ground_started_ >= config_.ground_disarm_delay) {
        startActionPhase(Phase::WAIT_DISARM, now);
        reason_ = "landed but still armed; requesting normal disarm";
        action = maybeRequest(ActionKind::DISARM, now, false);
      }
    } else {
      ground_started_ = -1.0;
    }
  } else if (phase_ == Phase::WAIT_DISARM) {
    if (!sensors.state_fresh) {
      fail("FCU state unavailable while waiting for disarm", false);
    } else if (!sensors.armed) {
      finishLanding();
    } else if (!(sensors.extended_fresh && sensors.on_ground)) {
      fail("ground confirmation lost before disarm", false);
    } else {
      action = maybeRequest(ActionKind::DISARM, now, false);
    }
  }

  last_tick_ = now;
  // A goal may start in processCommand/updateArrival. Capture the hold before
  // publishing the new planner context; never expose the final mission goal.
  if (config_.execution_mode == "ego" && phase_ == Phase::GUIDING && planner_capture_hold_) {
    setpoint_ = sensors.pose;
    planner_capture_hold_ = false;
  }
  return makeOutput(sensors, action);
}

ControllerOutput Controller::makeOutput(const SensorState& sensors,
                                        const ActionRequest& action) const {
  ControllerOutput output;
  output.phase = phase_;
  output.reason = reason_;
  output.action = action;
  output.should_exit = should_exit_;
  output.exit_code = exit_code_;
  output.setpoint = setpoint_;
  output.final_target = final_target_;
  output.waypoint_index = waypoint_index_;
  output.waypoint_count = origin_valid_ ? mission_.size() : raw_mission_.size();
  output.planner_request_id = planner_id_;
  output.planner_state = planner_state_;
  output.reference_valid = origin_valid_;
  output.takeoff_reference_z = origin_valid_ ? mission_origin_.z : 0;
  output.planner_ready = config_.execution_mode == "ego" && phase_ == Phase::GUIDING &&
      !planner_id_.empty() && inputsReady(sensors) && sensors.frame_aligned &&
      sensors.armed && sensors.mode == "OFFBOARD";
  output.planner_target_age = planner_target_receipt_ < 0 ? -1 :
      std::max(0.0, last_tick_ - planner_target_receipt_);
  if (phase_ == Phase::OFFBOARD_REQUEST || phase_ == Phase::ARMING ||
      phase_ == Phase::TAKEOFF) {
    output.actual_state = 1;
  } else if (phase_ == Phase::GUIDING) {
    output.actual_state = 2;
  } else if (phase_ == Phase::LAND_REQUEST || phase_ == Phase::LANDING ||
             phase_ == Phase::WAIT_DISARM) {
    output.actual_state = 3;
  } else {
    output.actual_state = 0;
  }
  const bool publish_phase = phase_ == Phase::BOOT_WAIT_ZERO ||
      phase_ == Phase::IDLE_GROUND || phase_ == Phase::OFFBOARD_REQUEST ||
      phase_ == Phase::ARMING || phase_ == Phase::TAKEOFF ||
      phase_ == Phase::GUIDING || phase_ == Phase::PAUSED ||
      phase_ == Phase::HOLDING || phase_ == Phase::LAND_REQUEST;
  const bool ground_idle = phase_ == Phase::BOOT_WAIT_ZERO ||
                           phase_ == Phase::IDLE_GROUND;
  output.publish_setpoint = publish_phase && inputsReady(sensors) &&
      (!ground_idle || (sensors.extended_fresh && !sensors.armed &&
                        sensors.on_ground));
  return output;
}

}  // namespace ducted_offboard
