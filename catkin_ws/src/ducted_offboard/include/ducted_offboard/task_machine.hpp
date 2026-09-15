#pragma once
#include "ducted_offboard/execution.hpp"

namespace ducted_offboard {

struct TaskSnapshot {
  ExecutionIntent intent;
  Phase phase{Phase::BOOT_WAIT_ZERO};
  std::string reason{"waiting for /ctrl_cmd/state=0"};
  int command{-1};
  std::size_t waypoint_index{0}, waypoint_count{0};
  Pose4d target;
};

class TaskMachine {
 public:
  explicit TaskMachine(const ControllerConfig& config);
  bool load(const std::vector<Waypoint>& mission, bool sequence);
  TaskSnapshot tick(double now, int command, const ExecutionFeedback& feedback);

 private:
  void issue(IntentKind kind, double now, const Pose4d& target = Pose4d{});
  void guide(double now);
  bool matched(const ExecutionFeedback& feedback) const;
  ControllerConfig config_;
  TaskSnapshot state_;
  std::vector<Waypoint> mission_;
  Pose4d origin_;
  bool sequence_{false}, zero_seen_{false}, origin_valid_{false};
  double last_tick_{-1}, target_started_{0}, arrival_started_{-1};
};

ControllerOutput combinedStatus(const TaskSnapshot& task,
                                const ExecutionFeedback& execution, double now);

}  // namespace ducted_offboard
