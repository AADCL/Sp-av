#pragma once
#include "ducted_offboard/types.hpp"

namespace ducted_offboard {

class Controller {
 public:
  explicit Controller(const ControllerConfig& config);

  void observeCommand(int command, double now);
  void setMission(const std::vector<Waypoint>& mission, bool sequence);
  bool replaceSequence(const std::vector<Waypoint>& mission);
  bool replaceSingle(const Waypoint& waypoint);
  void acceptActionResult(std::uint64_t id, bool transport_ok, bool accepted,
                          double now);
  ControllerOutput tick(double now, const SensorState& sensors);
  void acceptPlannerTarget(const std::string& id, const Pose4d& pose,
                           double stamp, double receipt, double ros_now);
  void acceptPlannerStatus(const std::string& id, const std::string& state,
                           const std::string& reason, double stamp,
                           double receipt, double ros_now);
  void acceptPlannerResponse(const std::string& id, bool accepted,
                             const std::string& reason, double now);

  Phase phase() const { return phase_; }
  const std::string& reason() const { return reason_; }

 private:
  bool inputsReady(const SensorState& sensors) const;
  bool reached(const Pose4d& target, const SensorState& sensors) const;
  void processCommand(double now, const SensorState& sensors);
  void startOffboard(double now, const SensorState& sensors);
  void startActionPhase(Phase phase, double now);
  ActionRequest maybeRequest(ActionKind kind, double now, bool fatal_on_exhaustion);
  void cancelPendingAction();
  void fail(const std::string& reason, bool should_exit);
  void startTakeoff(double now);
  void finishTakeoff(double now, const SensorState& sensors);
  void startGuiding(double now);
  void pauseAtActual(const SensorState& sensors, const std::string& reason);
  void startLanding(double now, const SensorState& sensors);
  void finishLanding();
  void resolveMission();
  void updateSetpoint(double now);
  void updateArrival(double now, const SensorState& sensors, bool takeoff);
  void beginPlanner(double now);
  void revokePlanner();
  void updatePlanner(double now, const SensorState& sensors);
  bool plannerSampleFresh(double stamp, double receipt, double deadline,
                          double now, double ros_now) const;
  ControllerOutput makeOutput(const SensorState& sensors,
                              const ActionRequest& action) const;

  ControllerConfig config_;
  Phase phase_{Phase::BOOT_WAIT_ZERO};
  std::string reason_{"waiting for /ctrl_cmd/state=0"};
  bool zero_seen_{false};
  bool command_pending_{false};
  bool takeoff_requested_{false};
  int last_command_{-1};
  int pending_command_{-1};
  double prestream_started_{-1.0};
  double last_tick_{-1.0};
  double phase_started_{0.0};
  double target_started_{0.0};
  double arrival_started_{-1.0};
  double ground_started_{-1.0};
  Pose4d mission_origin_;
  bool origin_valid_{false};
  Pose4d setpoint_;
  Pose4d final_target_;
  std::vector<Waypoint> raw_mission_;
  std::vector<Pose4d> mission_;
  bool sequence_{true};
  std::size_t waypoint_index_{0};
  bool mission_replaced_while_paused_{false};
  std::uint64_t planner_generation_{0};
  std::string planner_id_;
  std::string planner_state_{"INACTIVE"};
  std::string planner_error_;
  Pose4d planner_target_;
  double planner_target_stamp_{0.0};
  double planner_target_receipt_{-1.0};
  double planner_target_deadline_{-1.0};
  double planner_status_stamp_{0.0};
  double planner_status_receipt_{-1.0};
  double planner_status_deadline_{-1.0};
  double planner_started_{0.0};
  bool planner_capture_hold_{false};

  int action_attempts_{0};
  std::uint64_t next_action_id_{1};
  std::uint64_t pending_action_id_{0};
  bool action_pending_{false};
  bool waiting_feedback_{false};
  double action_started_{0.0};
  double feedback_deadline_{0.0};
  double next_action_time_{0.0};
  bool should_exit_{false};
  int exit_code_{0};
};

}  // namespace ducted_offboard
