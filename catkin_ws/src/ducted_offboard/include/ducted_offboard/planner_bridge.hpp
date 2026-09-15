#pragma once
#include <memory>
#include <functional>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include "ducted_offboard/core.hpp"

namespace ducted_offboard {
// Owns planner IO and one asynchronous navigation RPC. All controller mutations
// are drained by poll() on the control thread under the controller mutex.
class PlannerBridge {
 public:
  using Log = std::function<void(const std::string&, const std::string&)>;
  PlannerBridge(ros::NodeHandle node, ros::NodeHandle private_node, Log log);
  ~PlannerBridge();
  void acceptFcuPose(const geometry_msgs::PoseStamped& pose);
  bool aligned(double wall, double ros_now) const;
  void poll(Controller& controller, double wall, double ros_now);
  void update(const ControllerOutput& output, double wall, const ros::Time& stamp);
  std::string configuration() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
