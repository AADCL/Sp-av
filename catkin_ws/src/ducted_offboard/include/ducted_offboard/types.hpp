#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ducted_offboard {

struct Pose4d {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct Waypoint {
  Pose4d pose;
  std::string frame_id{"mission_start"};
};

enum class Phase {
  BOOT_WAIT_ZERO,
  IDLE_GROUND,
  OFFBOARD_REQUEST,
  ARMING,
  TAKEOFF,
  GUIDING,
  PAUSED,
  HOLDING,
  LAND_REQUEST,
  LANDING,
  WAIT_DISARM,
  ERROR,
};

enum class ActionKind { NONE, SET_OFFBOARD, ARM, SET_LAND, DISARM };

struct ActionRequest {
  ActionKind kind{ActionKind::NONE};
  std::uint64_t id{0};
  std::uint64_t generation{0};
  std::uint64_t request_id{0};
};

struct SensorState {
  bool connected{false};
  bool state_fresh{false};
  bool pose_fresh{false};
  bool velocity_fresh{false};
  bool extended_fresh{false};
  std::string mode;
  bool armed{false};
  bool on_ground{false};
  Pose4d pose;
  double speed{0.0};
  bool frame_aligned{false};
  double ros_time{-1.0};
};

struct ControllerConfig {
  std::string execution_mode{"direct"};
  std::string session_id;
  double task_timeout{0.5};
  double planner_timeout{0.5};
  double planner_first_target_timeout{2.0};
  double planner_future_tolerance{0.05};
  double prestream_duration{2.0};
  double retry_interval{5.0};
  double feedback_timeout{2.0};
  double service_timeout{3.0};
  int max_attempts{4};
  double takeoff_height{1.0};
  double horizontal_speed{0.3};
  double vertical_speed{0.2};
  double yaw_rate{20.0 * 3.14159265358979323846 / 180.0};
  double horizontal_tolerance{0.15};
  double vertical_tolerance{0.10};
  double yaw_tolerance{10.0 * 3.14159265358979323846 / 180.0};
  double speed_tolerance{0.15};
  double arrival_dwell{1.0};
  double takeoff_timeout{30.0};
  double waypoint_timeout{120.0};
  double landing_timeout{120.0};
  double ground_disarm_delay{5.0};
};

struct ControllerOutput {
  Phase phase{Phase::BOOT_WAIT_ZERO};
  int actual_state{0};
  std::string reason;
  bool publish_setpoint{false};
  Pose4d setpoint;
  Pose4d final_target;
  ActionRequest action;
  bool should_exit{false};
  int exit_code{0};
  std::size_t waypoint_index{0};
  std::size_t waypoint_count{0};
  std::string planner_request_id;
  std::string planner_state;
  bool planner_ready{false};
  bool reference_valid{false};
  double takeoff_reference_z{0.0};
  double planner_target_age{-1.0};
  std::string task_phase;
  std::string executor_phase;
  std::uint64_t task_generation{0};
  double task_heartbeat_age{-1.0};
};

double wrapAngle(double angle);
Pose4d resolveWaypoint(const Waypoint& waypoint, const Pose4d& mission_origin);
Pose4d advanceSetpoint(const Pose4d& current, const Pose4d& goal, double dt,
                       double horizontal_speed, double vertical_speed,
                       double yaw_rate);
std::string phaseName(Phase phase);
std::string actionName(ActionKind action);


}  // namespace ducted_offboard
