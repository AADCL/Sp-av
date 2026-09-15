#include "ducted_offboard/core.hpp"
#include <algorithm>
#include <cmath>

namespace ducted_offboard {

void Controller::revokePlanner() {
  planner_id_.clear();
  planner_state_ = "INACTIVE";
  planner_error_.clear();
  planner_target_stamp_ = planner_status_stamp_ = 0;
  planner_target_receipt_ = planner_status_receipt_ = -1;
  planner_target_deadline_ = planner_status_deadline_ = -1;
  planner_capture_hold_ = false;
}

void Controller::beginPlanner(double now) {
  revokePlanner();
  planner_id_ = config_.session_id + "-" + std::to_string(++planner_generation_);
  planner_state_ = "WAITING";
  planner_started_ = now;
  planner_capture_hold_ = true;
  reason_ = "waiting for EGO tracking target";
}

bool Controller::plannerSampleFresh(double stamp, double receipt, double deadline,
                                     double now, double ros_now) const {
  return receipt >= 0 && now >= receipt && now - receipt <= config_.planner_timeout &&
      now <= deadline && (ros_now < 0 ||
      (ros_now - stamp >= -config_.planner_future_tolerance &&
       ros_now - stamp <= config_.planner_timeout));
}

void Controller::acceptPlannerTarget(const std::string& id, const Pose4d& pose,
                                      double stamp, double receipt, double ros_now) {
  if (config_.execution_mode != "ego" || phase_ != Phase::GUIDING ||
      id.empty() || id != planner_id_) return;
  const double age = ros_now - stamp;
  if (!std::isfinite(stamp) || stamp <= 0 || stamp <= planner_target_stamp_ ||
      !std::isfinite(receipt) || !std::isfinite(age) ||
      age < -config_.planner_future_tolerance || age > config_.planner_timeout ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.z) || !std::isfinite(pose.yaw)) {
    planner_error_ = "invalid or stale EGO tracking target";
    return;
  }
  planner_target_ = pose;
  planner_target_stamp_ = stamp;
  planner_target_receipt_ = receipt;
  planner_target_deadline_ = receipt + config_.planner_timeout - std::max(0.0, age);
}

void Controller::acceptPlannerStatus(const std::string& id, const std::string& state,
                                      const std::string& reason, double stamp,
                                      double receipt, double ros_now) {
  if (config_.execution_mode != "ego" || phase_ != Phase::GUIDING ||
      id.empty() || id != planner_id_) return;
  const double age = ros_now - stamp;
  if (!std::isfinite(stamp) || stamp <= 0 || stamp <= planner_status_stamp_ ||
      !std::isfinite(receipt) || !std::isfinite(age) ||
      age < -config_.planner_future_tolerance || age > config_.planner_timeout) {
    planner_error_ = "invalid or stale EGO navigation status";
    return;
  }
  planner_status_stamp_ = stamp;
  planner_status_receipt_ = receipt;
  planner_status_deadline_ = receipt + config_.planner_timeout - std::max(0.0, age);
  planner_state_ = state;
  if (state != "IDLE" && state != "CLEAR" && state != "AVOIDING" && state != "GOAL_REACHED")
    planner_error_ = "EGO " + state + ": " + reason;
}

void Controller::acceptPlannerResponse(const std::string& id, bool accepted,
                                        const std::string& reason, double) {
  if (config_.execution_mode == "ego" && phase_ == Phase::GUIDING &&
      !id.empty() && id == planner_id_ && !accepted)
    planner_error_ = "navigation request failed: " + reason;
}

void Controller::updatePlanner(double now, const SensorState& sensors) {
  if (planner_capture_hold_) {
    setpoint_ = sensors.pose;
    planner_capture_hold_ = false;
  }
  if (!sensors.frame_aligned) {
    pauseAtActual(sensors, "external odometry and PX4 pose are not aligned or stale");
    return;
  }
  if (!planner_error_.empty()) {
    const std::string error = planner_error_;
    pauseAtActual(sensors, error);
    return;
  }
  if (planner_target_receipt_ < 0) {
    if (now - planner_started_ > config_.planner_first_target_timeout)
      pauseAtActual(sensors, "EGO first tracking target timeout");
    return;
  }
  if (!plannerSampleFresh(planner_target_stamp_, planner_target_receipt_,
                          planner_target_deadline_, now, sensors.ros_time) ||
      !plannerSampleFresh(planner_status_stamp_, planner_status_receipt_,
                          planner_status_deadline_, now, sensors.ros_time)) {
    pauseAtActual(sensors, "EGO tracking target or navigation status stale");
    return;
  }
  // Position was already slew-limited and its final swept segment checked by
  // TrajectoryGuard. A second position filter would change that checked segment.
  const double dt = last_tick_ < 0 ? 0 : std::max(0.0, now - last_tick_);
  const double delta = wrapAngle(planner_target_.yaw - setpoint_.yaw);
  const double yaw = wrapAngle(setpoint_.yaw + std::copysign(
      std::min(std::abs(delta), config_.yaw_rate * dt), delta));
  setpoint_ = planner_target_;
  setpoint_.yaw = yaw;
  reason_ = "following EGO tracking targets";
}

}  // namespace ducted_offboard
