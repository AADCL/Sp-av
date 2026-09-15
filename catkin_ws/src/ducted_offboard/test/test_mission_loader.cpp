#include <gtest/gtest.h>

#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

#include "ducted_offboard/mission_loader.hpp"

namespace d = ducted_offboard;

TEST(MissionYaml, LoadsRequiredFieldsAndRootFrame) {
  const auto root = YAML::Load(
      "frame_id: mission_start\n"
      "waypoints:\n"
      "  - {x: 1.0, y: -2.0, z: 1.2, yaw_deg: 90}\n"
      "  - {x: 2.0, y: 0.0, z: 1.2, yaw_deg: -30}\n");
  const auto mission = d::parseMissionYaml(root, "odom");
  ASSERT_EQ(mission.size(), 2u);
  EXPECT_EQ(mission[0].frame_id, "mission_start");
  EXPECT_NEAR(mission[0].pose.y, -2.0, 1e-9);
  EXPECT_NEAR(mission[0].pose.yaw, M_PI_2, 1e-9);
  EXPECT_NEAR(mission[1].pose.yaw, -M_PI / 6.0, 1e-9);
}

TEST(MissionParam, AcceptsXmlRpcIntegersAndDoubles) {
  XmlRpc::XmlRpcValue root;
  root.setSize(1);
  root[0]["x"] = 3;
  root[0]["y"] = 0.5;
  root[0]["z"] = 1;
  root[0]["yaw_deg"] = 180;
  const auto mission = d::parseMissionParam(root, "odom");
  ASSERT_EQ(mission.size(), 1u);
  EXPECT_EQ(mission[0].frame_id, "odom");
  EXPECT_DOUBLE_EQ(mission[0].pose.x, 3.0);
  EXPECT_DOUBLE_EQ(mission[0].pose.y, 0.5);
  EXPECT_NEAR(std::abs(mission[0].pose.yaw), M_PI, 1e-9);
}

TEST(MissionValidation, RejectsMissingFieldsAndUnknownFrames) {
  EXPECT_THROW(d::parseMissionYaml(
      YAML::Load("waypoints: [{x: 1, y: 0, z: 1}]"), "mission_start"),
      std::invalid_argument);
  EXPECT_THROW(d::parseMissionYaml(
      YAML::Load("frame_id: map\nwaypoints: [{x: 1, y: 0, z: 1, yaw_deg: 0}]"),
      "mission_start"), std::invalid_argument);
}

TEST(QuaternionValidation, ReturnsYawOnlyForFiniteNormalizedQuaternion) {
  EXPECT_NEAR(d::yawFromQuaternion(0.0, 0.0, std::sin(M_PI / 4.0),
                                   std::cos(M_PI / 4.0)), M_PI_2, 1e-9);
  EXPECT_THROW(d::yawFromQuaternion(0.0, 0.0, 0.0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(d::yawFromQuaternion(0.0, 0.0, 0.0, 2.0),
               std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
