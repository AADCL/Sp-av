#include "ducted_offboard/mission_loader.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace ducted_offboard {
namespace {

void validateFrame(const std::string& frame) {
  if (frame != "mission_start" && frame != "odom") {
    throw std::invalid_argument("mission frame must be mission_start or odom");
  }
}

double finiteValue(double value, const std::string& name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
  return value;
}

double yamlValue(const YAML::Node& node, const std::string& name,
                 std::size_t index) {
  if (!node[name] || !node[name].IsScalar()) {
    throw std::invalid_argument("waypoint " + std::to_string(index) +
                                " is missing " + name);
  }
  try {
    return finiteValue(node[name].as<double>(), name);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("invalid " + name + ": " + error.what());
  }
}

double xmlValue(const XmlRpc::XmlRpcValue& node, const std::string& name,
                int index) {
  if (!node.hasMember(name)) {
    throw std::invalid_argument("waypoint " + std::to_string(index) +
                                " is missing " + name);
  }
  const auto& value = node[name];
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return finiteValue(static_cast<double>(value), name);
  }
  throw std::invalid_argument(name + " must be numeric");
}

}  // namespace

std::vector<Waypoint> parseMissionYaml(const YAML::Node& root,
                                       const std::string& default_frame) {
  if (!root || !root.IsMap()) {
    throw std::invalid_argument("mission YAML root must be a map");
  }
  std::string frame = default_frame;
  if (root["frame_id"]) {
    try {
      frame = root["frame_id"].as<std::string>();
    } catch (const YAML::Exception& error) {
      throw std::invalid_argument("invalid frame_id: " + std::string(error.what()));
    }
  }
  validateFrame(frame);
  const auto points = root["waypoints"];
  if (!points || !points.IsSequence() || points.size() == 0) {
    throw std::invalid_argument("waypoints must be a non-empty sequence");
  }
  std::vector<Waypoint> mission;
  mission.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto point = points[index];
    if (!point.IsMap()) {
      throw std::invalid_argument("waypoint " + std::to_string(index) +
                                  " must be a map");
    }
    Waypoint waypoint;
    waypoint.frame_id = frame;
    waypoint.pose.x = yamlValue(point, "x", index);
    waypoint.pose.y = yamlValue(point, "y", index);
    waypoint.pose.z = yamlValue(point, "z", index);
    waypoint.pose.yaw = wrapAngle(
        yamlValue(point, "yaw_deg", index) * 3.14159265358979323846 / 180.0);
    mission.push_back(waypoint);
  }
  return mission;
}

std::vector<Waypoint> loadMissionFile(const std::string& path,
                                      const std::string& default_frame) {
  if (path.empty()) {
    throw std::invalid_argument("mission_file is empty");
  }
  try {
    return parseMissionYaml(YAML::LoadFile(path), default_frame);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("cannot load mission file " + path + ": " +
                                error.what());
  }
}

std::vector<Waypoint> parseMissionParam(const XmlRpc::XmlRpcValue& root,
                                        const std::string& frame_id) {
  validateFrame(frame_id);
  if (root.getType() != XmlRpc::XmlRpcValue::TypeArray || root.size() == 0) {
    throw std::invalid_argument("waypoints parameter must be a non-empty array");
  }
  std::vector<Waypoint> mission;
  mission.reserve(root.size());
  for (int index = 0; index < root.size(); ++index) {
    const auto& point = root[index];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::invalid_argument("waypoint " + std::to_string(index) +
                                  " must be a dictionary");
    }
    Waypoint waypoint;
    waypoint.frame_id = frame_id;
    waypoint.pose.x = xmlValue(point, "x", index);
    waypoint.pose.y = xmlValue(point, "y", index);
    waypoint.pose.z = xmlValue(point, "z", index);
    waypoint.pose.yaw = wrapAngle(
        xmlValue(point, "yaw_deg", index) * 3.14159265358979323846 / 180.0);
    mission.push_back(waypoint);
  }
  return mission;
}

double yawFromQuaternion(double x, double y, double z, double w) {
  if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
        std::isfinite(w))) {
    throw std::invalid_argument("quaternion must be finite");
  }
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm < 1e-9 || std::abs(norm - 1.0) > 1e-3) {
    throw std::invalid_argument("quaternion must be normalized");
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

}  // namespace ducted_offboard
