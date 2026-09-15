#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ducted_offboard/core.hpp"

namespace d = ducted_offboard;

namespace {

d::SensorState readySensors() {
  d::SensorState state;
  state.connected = true;
  state.state_fresh = true;
  state.pose_fresh = true;
  state.velocity_fresh = true;
  state.extended_fresh = true;
  state.mode = "POSCTL";
  state.armed = false;
  state.on_ground = true;
  state.pose = {1.0, 2.0, 0.4, M_PI_2};
  state.speed = 0.0;
  return state;
}

void acceptAction(d::Controller& controller, const d::ControllerOutput& output,
                  double now) {
  ASSERT_NE(output.action.kind, d::ActionKind::NONE);
  controller.acceptActionResult(output.action.id, true, true, now);
}

void enterTakeoff(d::Controller& controller, d::SensorState& sensors) {
  controller.observeCommand(0, 0.0);
  controller.tick(0.0, sensors);
  controller.observeCommand(1, 0.1);
  auto output = controller.tick(0.1, sensors);
  acceptAction(controller, output, 0.1);
  sensors.mode = "OFFBOARD";
  output = controller.tick(0.2, sensors);
  acceptAction(controller, output, 0.2);
  sensors.armed = true;
  sensors.on_ground = false;
  output = controller.tick(0.3, sensors);
  ASSERT_EQ(output.phase, d::Phase::TAKEOFF);
}

}  // namespace

TEST(Coordinates, ResolvesMissionStartUsingTakeoffYaw) {
  const d::Pose4d origin{10.0, 20.0, 2.0, M_PI_2};
  const d::Waypoint relative{{3.0, 1.0, 1.0, M_PI_2}, "mission_start"};
  const auto resolved = d::resolveWaypoint(relative, origin);
  EXPECT_NEAR(resolved.x, 9.0, 1e-9);
  EXPECT_NEAR(resolved.y, 23.0, 1e-9);
  EXPECT_NEAR(resolved.z, 3.0, 1e-9);
  EXPECT_NEAR(resolved.yaw, M_PI, 1e-9);
  EXPECT_EQ(d::resolveWaypoint({{4.0, 5.0, 6.0, -0.2}, "odom"}, origin).x,
            4.0);
}

TEST(Setpoint, AppliesIndependentHorizontalVerticalAndYawLimits) {
  const d::Pose4d current{0.0, 0.0, 0.0, 0.0};
  const d::Pose4d goal{3.0, 4.0, 2.0, M_PI};
  const auto next = d::advanceSetpoint(current, goal, 1.0, 0.3, 0.2,
                                       20.0 * M_PI / 180.0);
  EXPECT_NEAR(next.x, 0.18, 1e-9);
  EXPECT_NEAR(next.y, 0.24, 1e-9);
  EXPECT_NEAR(next.z, 0.2, 1e-9);
  EXPECT_NEAR(next.yaw, 20.0 * M_PI / 180.0, 1e-9);
}

TEST(Configuration, RejectsNegativeArrivalThresholds) {
  d::ControllerConfig config;
  config.horizontal_tolerance = -0.01;
  EXPECT_THROW(d::Controller controller(config), std::invalid_argument);
}

TEST(Command, RequiresObservedZeroAndTriggersOnlyOnValueChange) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  d::Controller controller(config);
  auto sensors = readySensors();

  controller.observeCommand(1, 0.0);
  auto output = controller.tick(0.0, sensors);
  EXPECT_EQ(output.phase, d::Phase::BOOT_WAIT_ZERO);
  EXPECT_EQ(output.action.kind, d::ActionKind::NONE);

  controller.observeCommand(0, 0.1);
  controller.tick(0.1, sensors);
  controller.observeCommand(1, 0.2);
  output = controller.tick(0.2, sensors);
  EXPECT_EQ(output.phase, d::Phase::OFFBOARD_REQUEST);
  EXPECT_EQ(output.action.kind, d::ActionKind::SET_OFFBOARD);

  controller.acceptActionResult(output.action.id, false, false, 0.2);
  controller.observeCommand(1, 0.3);
  output = controller.tick(0.3, sensors);
  EXPECT_EQ(output.action.kind, d::ActionKind::NONE);
}

TEST(Command, RequiresFreshGroundFeedbackBeforeTakeoff) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  d::Controller controller(config);
  auto sensors = readySensors();
  sensors.extended_fresh = false;
  controller.observeCommand(0, 0.0);
  controller.tick(0.0, sensors);
  controller.observeCommand(1, 0.1);
  auto output = controller.tick(0.1, sensors);
  EXPECT_EQ(output.phase, d::Phase::IDLE_GROUND);
  EXPECT_EQ(output.action.kind, d::ActionKind::NONE);

  sensors.extended_fresh = true;
  output = controller.tick(0.2, sensors);
  EXPECT_EQ(output.phase, d::Phase::OFFBOARD_REQUEST);
  EXPECT_EQ(output.action.kind, d::ActionKind::SET_OFFBOARD);
}

TEST(Setpoint, GroundPrestreamTracksCurrentPoseAndRejectsArmedGroundState) {
  d::Controller controller(d::ControllerConfig{});
  auto sensors = readySensors();
  sensors.pose = {4.0, -2.0, 0.3, -0.4};
  auto output = controller.tick(0.0, sensors);
  ASSERT_TRUE(output.publish_setpoint);
  EXPECT_DOUBLE_EQ(output.setpoint.x, sensors.pose.x);
  EXPECT_DOUBLE_EQ(output.setpoint.y, sensors.pose.y);
  EXPECT_DOUBLE_EQ(output.setpoint.z, sensors.pose.z);
  EXPECT_DOUBLE_EQ(output.setpoint.yaw, sensors.pose.yaw);

  sensors.armed = true;
  output = controller.tick(0.1, sensors);
  EXPECT_FALSE(output.publish_setpoint);
}

TEST(Offboard, ExitsAfterInitialAttemptAndThreeRetries) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  config.retry_interval = 0.1;
  d::Controller controller(config);
  auto sensors = readySensors();
  controller.observeCommand(0, 0.0);
  controller.tick(0.0, sensors);
  controller.observeCommand(1, 0.01);

  for (int attempt = 0; attempt < 4; ++attempt) {
    const double now = 0.01 + attempt * 0.11;
    const auto output = controller.tick(now, sensors);
    ASSERT_EQ(output.action.kind, d::ActionKind::SET_OFFBOARD);
    controller.acceptActionResult(output.action.id, true, false, now);
  }
  const auto failed = controller.tick(0.46, sensors);
  EXPECT_EQ(failed.phase, d::Phase::ERROR);
  EXPECT_TRUE(failed.should_exit);
  EXPECT_EQ(failed.exit_code, 2);
  EXPECT_NE(failed.reason.find("4 attempts"), std::string::npos);
}

TEST(Takeoff, UsesDefaultOneMetreAndRequiresFeedbackBeforeAdvancing) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  d::Controller controller(config);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);

  auto output = controller.tick(1.3, sensors);
  EXPECT_TRUE(output.publish_setpoint);
  EXPECT_NEAR(output.final_target.x, 1.0, 1e-9);
  EXPECT_NEAR(output.final_target.y, 2.0, 1e-9);
  EXPECT_NEAR(output.final_target.z, 1.4, 1e-9);
  EXPECT_NEAR(output.final_target.yaw, M_PI_2, 1e-9);

  sensors.pose.z = 1.4;
  controller.tick(2.0, sensors);
  output = controller.tick(3.1, sensors);
  EXPECT_EQ(output.phase, d::Phase::HOLDING);
}

TEST(Guiding, AdvancesSequenceOnlyAfterContinuousArrivalDwell) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  config.arrival_dwell = 1.0;
  d::Controller controller(config);
  controller.setMission({{{1.0, 0.0, 1.0, 0.0}, "mission_start"},
                         {{2.0, 0.0, 1.0, 0.0}, "mission_start"}}, true);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);
  sensors.pose.z = 1.4;
  controller.tick(1.0, sensors);
  auto output = controller.tick(2.1, sensors);
  ASSERT_EQ(output.phase, d::Phase::HOLDING);
  controller.observeCommand(2, 2.11);
  output = controller.tick(2.11, sensors);
  ASSERT_EQ(output.phase, d::Phase::GUIDING);
  EXPECT_EQ(output.waypoint_index, 0u);

  sensors.pose = output.final_target;
  controller.tick(2.2, sensors);
  output = controller.tick(2.8, sensors);
  EXPECT_EQ(output.waypoint_index, 0u);
  output = controller.tick(3.3, sensors);
  EXPECT_EQ(output.waypoint_index, 1u);

  sensors.pose = output.final_target;
  controller.tick(3.4, sensors);
  output = controller.tick(4.5, sensors);
  EXPECT_EQ(output.phase, d::Phase::HOLDING);
  EXPECT_EQ(output.waypoint_count, 2u);
}

TEST(Guiding, PausesAtActualPoseAndResumesOriginalWaypoint) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  d::Controller controller(config);
  controller.setMission({{{1.0, 0.0, 1.0, 0.0}, "mission_start"}}, true);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);
  sensors.pose.z = 1.4;
  controller.tick(1.0, sensors);
  controller.tick(2.1, sensors);
  controller.observeCommand(2, 2.11);
  ASSERT_EQ(controller.tick(2.11, sensors).phase, d::Phase::GUIDING);
  sensors.pose = {1.1, 2.2, 1.4, 1.4};

  controller.observeCommand(0, 2.2);
  auto paused = controller.tick(2.2, sensors);
  ASSERT_EQ(paused.phase, d::Phase::PAUSED);
  EXPECT_NEAR(paused.final_target.x, sensors.pose.x, 1e-9);
  EXPECT_EQ(paused.waypoint_index, 0u);

  controller.observeCommand(2, 2.3);
  const auto resumed = controller.tick(2.3, sensors);
  EXPECT_EQ(resumed.phase, d::Phase::GUIDING);
  EXPECT_EQ(resumed.waypoint_index, 0u);
}

TEST(TargetUpdates, RejectsSequenceWhileGuidingAndAllTargetsWhileLanding) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  d::Controller controller(config);
  controller.setMission({{{1.0, 0.0, 1.0, 0.0}, "mission_start"}}, true);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);
  sensors.pose.z = 1.4;
  controller.tick(1.0, sensors);
  ASSERT_EQ(controller.tick(2.1, sensors).phase, d::Phase::HOLDING);
  controller.observeCommand(2, 2.11);
  ASSERT_EQ(controller.tick(2.11, sensors).phase, d::Phase::GUIDING);
  EXPECT_FALSE(controller.replaceSequence(
      {{{2.0, 0.0, 1.0, 0.0}, "mission_start"}}));

  controller.observeCommand(3, 2.2);
  ASSERT_EQ(controller.tick(2.2, sensors).phase, d::Phase::LAND_REQUEST);
  EXPECT_FALSE(controller.replaceSingle(
      {{3.0, 0.0, 1.0, 0.0}, "mission_start"}));
}

TEST(Landing, StopsSetpointsAfterLandFeedbackAndDisarmsOnGround) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  config.retry_interval = 0.1;
  config.ground_disarm_delay = 5.0;
  d::Controller controller(config);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);

  controller.observeCommand(3, 1.0);
  auto output = controller.tick(1.0, sensors);
  ASSERT_EQ(output.action.kind, d::ActionKind::SET_LAND);
  EXPECT_TRUE(output.publish_setpoint);
  acceptAction(controller, output, 1.0);

  sensors.mode = "AUTO.LAND";
  output = controller.tick(1.1, sensors);
  EXPECT_EQ(output.phase, d::Phase::LANDING);
  EXPECT_FALSE(output.publish_setpoint);

  sensors.on_ground = true;
  controller.tick(2.0, sensors);
  output = controller.tick(7.1, sensors);
  ASSERT_EQ(output.action.kind, d::ActionKind::DISARM);
  acceptAction(controller, output, 7.1);
  sensors.armed = false;
  output = controller.tick(7.2, sensors);
  EXPECT_EQ(output.phase, d::Phase::IDLE_GROUND);
  EXPECT_EQ(output.actual_state, 0);
}

TEST(Landing, DoesNotUseStaleDisarmedFeedbackAsCompletion) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  d::Controller controller(config);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);
  controller.observeCommand(3, 1.0);
  auto output = controller.tick(1.0, sensors);
  acceptAction(controller, output, 1.0);
  sensors.mode = "AUTO.LAND";
  ASSERT_EQ(controller.tick(1.1, sensors).phase, d::Phase::LANDING);

  sensors.armed = false;
  sensors.state_fresh = false;
  output = controller.tick(4.2, sensors);
  EXPECT_EQ(output.phase, d::Phase::ERROR);
  EXPECT_NE(output.reason.find("FCU state"), std::string::npos);
}

TEST(Safety, StopsPublishingWhenPoseBecomesStaleInFlight) {
  d::ControllerConfig config;
  config.prestream_duration = 0.0;
  d::Controller controller(config);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);
  sensors.pose_fresh = false;
  const auto output = controller.tick(1.0, sensors);
  EXPECT_EQ(output.phase, d::Phase::ERROR);
  EXPECT_FALSE(output.publish_setpoint);
  EXPECT_NE(output.reason.find("pose"), std::string::npos);
}

TEST(Stepwise, LoadedMissionWaitsForGuidingAfterTakeoff) {
  for (const auto& mode : {"direct", "ego"}) {
    for (bool sequence : {false, true}) {
      d::ControllerConfig config;
      config.execution_mode = mode;
      config.session_id = "stepwise-loaded";
      config.prestream_duration = 0;
      d::Controller controller(config);
      controller.setMission({{{3, 0, 1, 0}, "mission_start"}}, sequence);
      auto sensors = readySensors();
      sensors.frame_aligned = true;
      enterTakeoff(controller, sensors);
      sensors.pose.z = 1.4;
      controller.tick(1, sensors);
      auto output = controller.tick(2.1, sensors);
      ASSERT_EQ(output.phase, d::Phase::HOLDING) << mode;
      EXPECT_TRUE(output.publish_setpoint);
      EXPECT_FALSE(output.planner_ready);
      EXPECT_TRUE(output.planner_request_id.empty());
      EXPECT_EQ(output.waypoint_count, 1u);
      EXPECT_DOUBLE_EQ(output.final_target.x, 1);
      EXPECT_DOUBLE_EQ(output.final_target.y, 2);
      EXPECT_NEAR(output.final_target.z, 1.4, 1e-9);
      controller.observeCommand(1, 3);
      EXPECT_EQ(controller.tick(3, sensors).phase, d::Phase::HOLDING);
      controller.observeCommand(2, 3.1);
      output = controller.tick(3.1, sensors);
      EXPECT_EQ(output.phase, d::Phase::GUIDING);
      EXPECT_NEAR(output.final_target.y, 5, 1e-9);
      EXPECT_EQ(output.planner_ready, config.execution_mode == "ego");
    }
  }
}

TEST(Stepwise, NewTargetsInAirborneHoldDoNotStartGuiding) {
  for (const auto& mode : {"direct", "ego"}) {
    d::ControllerConfig config;
    config.execution_mode = mode;
    config.session_id = "stepwise-targets";
    config.prestream_duration = 0;
    d::Controller controller(config);
    auto sensors = readySensors();
    sensors.frame_aligned = true;
    enterTakeoff(controller, sensors);
    sensors.pose.z = 1.4;
    controller.tick(1, sensors);
    ASSERT_EQ(controller.tick(2.1, sensors).phase, d::Phase::HOLDING);
    ASSERT_TRUE(controller.replaceSingle({{3, 0, 1, 0}, "mission_start"}));
    auto output = controller.tick(2.2, sensors);
    EXPECT_EQ(output.phase, d::Phase::HOLDING) << mode;
    EXPECT_FALSE(output.planner_ready);
    EXPECT_DOUBLE_EQ(output.final_target.y, 2);
    ASSERT_TRUE(controller.replaceSequence({{{2, 0, 1, 0}, "mission_start"}}));
    EXPECT_EQ(controller.tick(2.3, sensors).phase, d::Phase::HOLDING);
    controller.observeCommand(2, 2.4);
    output = controller.tick(2.4, sensors);
    EXPECT_EQ(output.phase, d::Phase::GUIDING);
    const auto old_id = output.planner_request_id;
    ASSERT_TRUE(controller.replaceSingle({{4, 0, 1, 0}, "mission_start"}));
    output = controller.tick(2.5, sensors);
    EXPECT_EQ(output.phase, d::Phase::GUIDING);
    EXPECT_NEAR(output.final_target.y, 6, 1e-9);
    if (config.execution_mode == "ego") EXPECT_NE(output.planner_request_id, old_id);
  }
}

TEST(Stepwise, GuidingCommandDuringTakeoffDoesNotQueueAutomaticExecution) {
  d::ControllerConfig config;
  config.prestream_duration = 0;
  d::Controller controller(config);
  controller.setMission({{{3, 0, 1, 0}, "mission_start"}}, true);
  auto sensors = readySensors();
  enterTakeoff(controller, sensors);
  controller.observeCommand(2, .4);
  EXPECT_EQ(controller.tick(.4, sensors).phase, d::Phase::TAKEOFF);
  sensors.pose.z = 1.4;
  controller.tick(1, sensors);
  EXPECT_EQ(controller.tick(2.1, sensors).phase, d::Phase::HOLDING);
  controller.observeCommand(2, 2.2);
  EXPECT_EQ(controller.tick(2.2, sensors).phase, d::Phase::HOLDING);
  controller.observeCommand(0, 2.3);
  controller.tick(2.3, sensors);
  controller.observeCommand(2, 2.4);
  EXPECT_EQ(controller.tick(2.4, sensors).phase, d::Phase::GUIDING);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
