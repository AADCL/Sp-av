#pragma once

#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>

#include "ducted_offboard/types.hpp"

namespace ducted_offboard {

std::vector<Waypoint> parseMissionYaml(const YAML::Node& root,
                                       const std::string& default_frame);
std::vector<Waypoint> loadMissionFile(const std::string& path,
                                      const std::string& default_frame);
std::vector<Waypoint> parseMissionParam(const XmlRpc::XmlRpcValue& root,
                                        const std::string& frame_id);
double yawFromQuaternion(double x, double y, double z, double w);

}  // namespace ducted_offboard
