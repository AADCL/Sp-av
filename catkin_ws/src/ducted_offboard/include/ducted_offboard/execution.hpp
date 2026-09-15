#pragma once
#include "ducted_offboard/types.hpp"

namespace ducted_offboard {

enum class IntentKind { WAIT, TAKEOFF, HOLD, TRACK, LAND };

// Value snapshots only. Task ownership never crosses into the execution loop.
struct ExecutionIntent {
  IntentKind kind{IntentKind::WAIT};
  std::uint64_t generation{0};
  std::uint64_t request_id{0};
  double issued_at{-1};
  Pose4d target;
};

struct ExecutionFeedback {
  ControllerOutput output;
  SensorState sensors;
  Pose4d origin;
  std::uint64_t generation{0};
  std::uint64_t request_id{0};
  double stamp{-1};
  bool authority_lost{false};
};

bool validSensors(const SensorState& sensors);
bool withinArrival(const Pose4d& target, const SensorState& sensors,
                   const ControllerConfig& config);
void validateDirectConfig(const ControllerConfig& config);

class FlightExecutor {
 public:
  explicit FlightExecutor(const ControllerConfig& config);
  ExecutionFeedback tick(double now, const SensorState& sensors,
                         const ExecutionIntent& intent);
  void acceptActionResult(std::uint64_t id, bool transport, bool accepted, double now);
  bool actionAllowed(const ActionRequest& action, double now,
                     const SensorState& sensors, const ExecutionIntent& latest) const;

 private:
  void cancelAction();
  void fail(const std::string& reason, bool exit = false);
  void actionPhase(Phase phase, double now, const std::string& reason);
  void hold(const SensorState& sensors, Phase phase, const std::string& reason);
  ActionRequest requestAction(ActionKind kind, double now, bool exit = false);
  bool landing() const;
  bool fresh(const ExecutionIntent& intent, double now) const;
  ControllerConfig config_;
  ExecutionIntent accepted_;
  Phase phase_{Phase::IDLE_GROUND};
  std::string reason_{"waiting for task command"};
  Pose4d setpoint_, target_, origin_;
  bool origin_valid_{false}, takeoff_pending_{false}, authority_lost_{false};
  bool should_exit_{false};
  std::uint64_t blocked_through_{0}, next_action_id_{1}, action_id_{0};
  bool action_pending_{false}, waiting_feedback_{false};
  double last_tick_{-1}, prestream_started_{-1}, phase_started_{0};
  double arrival_started_{-1}, ground_started_{-1}, action_started_{0};
  double feedback_deadline_{0}, next_action_time_{0};
  int attempts_{0};
};

}  // namespace ducted_offboard
