#include "ducted_offboard/types.hpp"

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

double wrapAngle(double angle) {
  if(!std::isfinite(angle))throw std::invalid_argument("angle must be finite");
  angle=std::remainder(angle,2.0*kPi);
  if(angle<=-kPi)angle+=2.0*kPi;
  return angle;
}

Pose4d resolveWaypoint(const Waypoint& waypoint, const Pose4d& origin) {
  if (!finitePose(waypoint.pose) || !finitePose(origin)) {
    throw std::invalid_argument("waypoint contains a non-finite value");
  }
  if (waypoint.frame_id == "odom") {
    return waypoint.pose;
  }
  if (waypoint.frame_id != "mission_start") {
    throw std::invalid_argument("waypoint frame must be mission_start or odom");
  }
  const double c = std::cos(origin.yaw);
  const double s = std::sin(origin.yaw);
  Pose4d output;
  output.x = origin.x + c * waypoint.pose.x - s * waypoint.pose.y;
  output.y = origin.y + s * waypoint.pose.x + c * waypoint.pose.y;
  output.z = origin.z + waypoint.pose.z;
  output.yaw = wrapAngle(origin.yaw + waypoint.pose.yaw);
  return output;
}

Pose4d advanceSetpoint(const Pose4d& current, const Pose4d& goal, double dt,
                       double horizontal_speed, double vertical_speed,
                       double yaw_rate) {
  if (!(std::isfinite(dt) && dt >= 0.0 && horizontal_speed > 0.0 &&
        vertical_speed > 0.0 && yaw_rate > 0.0)) {
    throw std::invalid_argument("invalid setpoint rate limit");
  }
  Pose4d output = current;
  const double dx = goal.x - current.x;
  const double dy = goal.y - current.y;
  const double distance = std::hypot(dx, dy);
  const double horizontal_step = horizontal_speed * dt;
  if (distance <= horizontal_step || distance == 0.0) {
    output.x = goal.x;
    output.y = goal.y;
  } else {
    const double scale = horizontal_step / distance;
    output.x += dx * scale;
    output.y += dy * scale;
  }
  output.z = moveToward(current.z, goal.z, vertical_speed * dt);
  const double yaw_delta = wrapAngle(goal.yaw - current.yaw);
  const double yaw_step = yaw_rate * dt;
  output.yaw = wrapAngle(current.yaw +
                         std::copysign(std::min(std::abs(yaw_delta), yaw_step),
                                       yaw_delta));
  return output;
}

std::string phaseName(Phase phase) {
  switch (phase) {
    case Phase::BOOT_WAIT_ZERO: return "BOOT_WAIT_ZERO";
    case Phase::IDLE_GROUND: return "IDLE_GROUND";
    case Phase::OFFBOARD_REQUEST: return "OFFBOARD_REQUEST";
    case Phase::ARMING: return "ARMING";
    case Phase::TAKEOFF: return "TAKEOFF";
    case Phase::GUIDING: return "GUIDING";
    case Phase::PAUSED: return "PAUSED";
    case Phase::HOLDING: return "HOLDING";
    case Phase::LAND_REQUEST: return "LAND_REQUEST";
    case Phase::LANDING: return "LANDING";
    case Phase::WAIT_DISARM: return "WAIT_DISARM";
    case Phase::ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

std::string actionName(ActionKind action) {
  switch (action) {
    case ActionKind::NONE: return "NONE";
    case ActionKind::SET_OFFBOARD: return "SET_OFFBOARD";
    case ActionKind::ARM: return "ARM";
    case ActionKind::SET_LAND: return "SET_LAND";
    case ActionKind::DISARM: return "DISARM";
  }
  return "UNKNOWN";
}


}  // namespace ducted_offboard
