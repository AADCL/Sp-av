#include <gtest/gtest.h>
#include "ducted_offboard/core.hpp"

namespace d = ducted_offboard;
namespace {
d::ControllerConfig config() {
  d::ControllerConfig c;
  c.execution_mode = "ego";
  c.session_id = "test-session";
  c.prestream_duration = 0;
  c.arrival_dwell = .1;
  return c;
}
d::SensorState sensors() {
  d::SensorState s;
  s.connected = s.state_fresh = s.pose_fresh = s.velocity_fresh = true;
  s.extended_fresh = s.on_ground = s.frame_aligned = true;
  s.mode = "OFFBOARD";
  s.pose = {0, 0, .4, 0};
  return s;
}
d::ControllerOutput airborne(d::Controller& c, d::SensorState& s) {
  c.observeCommand(0, 0); c.tick(0, s);
  c.observeCommand(1, .1); c.tick(.1, s);
  s.armed = true; s.on_ground = false; c.tick(.2, s);
  s.pose.z = 1.4;
  c.tick(1, s);
  auto output = c.tick(1.11, s);
  EXPECT_EQ(output.phase, d::Phase::HOLDING);
  EXPECT_FALSE(output.planner_ready);
  if (output.waypoint_count) {
    c.observeCommand(2, 1.12);
    output = c.tick(1.12, s);
  }
  return output;
}
void sample(d::Controller& c, const std::string& id, const d::Pose4d& p,
            double t, const std::string& state = "AVOIDING") {
  c.acceptPlannerTarget(id, p, t, t, t);
  c.acceptPlannerStatus(id, state, "", t, t, t);
}
}

TEST(Ego, WaitsWithoutMovingDirectlyToFinalGoalAndPreservesMission) {
  d::Controller c(config()); auto s = sensors();
  c.setMission({{{3,0,1,0},"mission_start"}}, true);
  auto o = airborne(c,s);
  ASSERT_TRUE(o.planner_ready);
  EXPECT_EQ(o.final_target.x, 3);
  EXPECT_EQ(o.setpoint.x, 0);
  EXPECT_NEAR(o.takeoff_reference_z, .4, 1e-9);
  auto id = o.planner_request_id;
  sample(c,id,{.08,.03,1.4,.8},1.2);
  o = c.tick(1.2,s);
  EXPECT_DOUBLE_EQ(o.setpoint.x,.08);
  EXPECT_DOUBLE_EQ(o.setpoint.y,.03);
  EXPECT_LE(o.setpoint.yaw,config().yaw_rate*.1);
  EXPECT_DOUBLE_EQ(o.final_target.x,3);
  EXPECT_EQ(o.waypoint_index,0u);
}

TEST(Ego, PausesOnLostStreamAndOnlyExplicitResumeCreatesNewId) {
  d::Controller c(config()); auto s = sensors();
  c.setMission({{{3,0,1,0},"mission_start"}}, true);
  auto o=airborne(c,s); auto old=o.planner_request_id;
  sample(c,old,{.04,0,1.4,0},1.2); c.tick(1.2,s);
  o=c.tick(1.71,s);
  EXPECT_EQ(o.phase,d::Phase::PAUSED); EXPECT_FALSE(o.planner_ready);
  sample(c,old,{3,0,1.4,0},1.72);
  EXPECT_EQ(c.tick(1.72,s).phase,d::Phase::PAUSED);
  c.observeCommand(0,1.79); c.tick(1.79,s);
  c.observeCommand(2,1.8); o=c.tick(1.8,s);
  EXPECT_NE(o.planner_request_id,old); EXPECT_TRUE(o.planner_ready);
  sample(c,old,{9,0,1.4,0},1.81);
  EXPECT_DOUBLE_EQ(c.tick(1.81,s).setpoint.x,s.pose.x);
}

TEST(Ego, RejectsStaleWrongFrameEquivalentAndDuplicateInputs) {
  d::Controller c(config()); auto s=sensors();
  c.setMission({{{3,0,1,0},"mission_start"}},false);
  auto o=airborne(c,s); auto id=o.planner_request_id;
  sample(c,id,{.04,0,1.4,0},1.2); c.tick(1.2,s);
  c.acceptPlannerTarget(id,{1,0,1.4,0},1.2,1.3,1.3);
  EXPECT_EQ(c.tick(1.3,s).phase,d::Phase::PAUSED);
}

TEST(Ego, ArrivalRequiresFinalGoalAndFreshMatchingNavigationCompletion) {
  d::Controller c(config()); auto s=sensors();
  c.setMission({{{1,0,1,0},"mission_start"},{{2,0,1,0},"mission_start"}},true);
  auto o=airborne(c,s); auto id=o.planner_request_id;
  sample(c,id,{0,0,1.4,0},1.2,"GOAL_REACHED"); c.tick(1.2,s);
  sample(c,id,{0,0,1.4,0},1.4,"GOAL_REACHED");
  EXPECT_EQ(c.tick(1.4,s).waypoint_index,0u);
  s.pose.x=1;
  sample(c,id,s.pose,1.5); c.tick(1.5,s);
  sample(c,id,s.pose,1.7);
  EXPECT_EQ(c.tick(1.7,s).waypoint_index,0u);
  sample(c,id,s.pose,1.8,"GOAL_REACHED"); c.tick(1.8,s);
  sample(c,id,s.pose,1.91,"GOAL_REACHED"); o=c.tick(1.91,s);
  EXPECT_EQ(o.waypoint_index,1u); EXPECT_NE(o.planner_request_id,id);
  EXPECT_EQ(o.setpoint.x,s.pose.x);
}

TEST(Ego, PauseInHoldBlocksLaterTargetAndLandingRevokesPlanner) {
  d::Controller c(config()); auto s=sensors();
  auto o=airborne(c,s); EXPECT_EQ(o.phase,d::Phase::HOLDING);
  c.observeCommand(0,1.2); c.tick(1.2,s);
  c.replaceSingle({{2,0,1,0},"mission_start"});
  EXPECT_EQ(c.tick(1.3,s).phase,d::Phase::PAUSED);
  c.observeCommand(2,1.4); o=c.tick(1.4,s); auto id=o.planner_request_id;
  c.observeCommand(3,1.5); o=c.tick(1.5,s);
  EXPECT_FALSE(o.planner_ready);
  sample(c,id,{2,0,1.4,0},1.6);
  EXPECT_EQ(c.tick(1.6,s).phase,d::Phase::LAND_REQUEST);
}

TEST(Ego, FirstTargetTimeoutFailureAndFrameMismatchPause) {
  for(int kind=0;kind<3;++kind) {
    d::Controller c(config()); auto s=sensors();
    c.setMission({{{1,0,1,0},"mission_start"}},true);
    auto o=airborne(c,s);
    if(kind==1)c.acceptPlannerResponse(o.planner_request_id,false,"service timeout",1.2);
    if(kind==2)s.frame_aligned=false;
    o=c.tick(kind==0?3.2:1.2,s);
    EXPECT_EQ(o.phase,d::Phase::PAUSED);
    EXPECT_EQ(o.setpoint.x,s.pose.x);
  }
}

TEST(Ego, NewFlightHasNewReferenceAndSessionTaskId) {
  d::Controller c(config()); auto s=sensors();
  c.setMission({{{1,0,1,0},"mission_start"}},true);
  auto o=airborne(c,s); const auto old=o.planner_request_id;
  c.observeCommand(3,1.2);c.tick(1.2,s);
  s.mode="AUTO.LAND";c.tick(1.3,s);
  s.armed=false;s.on_ground=true;s.pose.z=2;
  o=c.tick(1.4,s);EXPECT_FALSE(o.reference_valid);
  s.mode="OFFBOARD";
  c.observeCommand(0,1.5);c.tick(1.5,s);
  c.observeCommand(1,1.6);c.tick(1.6,s);
  s.armed=true;s.on_ground=false;c.tick(1.7,s);
  s.pose.z=3;c.tick(2,s);o=c.tick(2.11,s);
  EXPECT_NEAR(o.takeoff_reference_z,2,1e-9);
  EXPECT_EQ(o.phase,d::Phase::HOLDING);
  EXPECT_FALSE(o.planner_ready);
  EXPECT_TRUE(o.planner_request_id.empty());
  c.observeCommand(2,2.12);o=c.tick(2.12,s);
  EXPECT_TRUE(o.planner_ready);
  EXPECT_NE(o.planner_request_id,old);
}

TEST(Ego, DirectModeIgnoresPlannerTraffic) {
  d::Controller c(d::ControllerConfig{});
  c.acceptPlannerTarget("old",{100,0,0,0},1,1,1);
  auto s=sensors();auto o=c.tick(1,s);
  EXPECT_EQ(o.setpoint.x,0); EXPECT_FALSE(o.planner_ready);
}

TEST(Ego, SourceClockRollbackAndRepeatedTargetsCannotExtendMissionTimeout) {
  for(bool rollback:{false,true}) {
    auto cfg=config();cfg.waypoint_timeout=.3;
    d::Controller c(cfg);auto s=sensors();
    c.setMission({{{1,0,1,0},"mission_start"}},true);
    auto o=airborne(c,s);auto id=o.planner_request_id;
    sample(c,id,{.01,0,1.4,0},1.2);c.tick(1.2,s);
    if(rollback) {
      s.ros_time=1.0;
      o=c.tick(1.25,s);
    }else{
      sample(c,id,{.02,0,1.4,0},1.3);c.tick(1.3,s);
      sample(c,id,{.03,0,1.4,0},1.43);o=c.tick(1.43,s);
    }
    EXPECT_EQ(o.phase,d::Phase::PAUSED);
    EXPECT_FALSE(o.planner_ready);
  }
}

TEST(Ego, LocalizationLossStopsOutputAndRevokesContext) {
  d::Controller c(config());auto s=sensors();
  c.setMission({{{1,0,1,0},"mission_start"}},true);
  auto o=airborne(c,s);auto id=o.planner_request_id;
  sample(c,id,{.03,0,1.4,0},1.2);c.tick(1.2,s);
  s.pose_fresh=false;
  o=c.tick(1.3,s);
  EXPECT_EQ(o.phase,d::Phase::ERROR);
  EXPECT_FALSE(o.publish_setpoint);EXPECT_FALSE(o.planner_ready);
  s.pose_fresh=true;sample(c,id,{5,0,1.4,0},1.4);
  o=c.tick(1.4,s);
  EXPECT_EQ(o.phase,d::Phase::ERROR);EXPECT_FALSE(o.publish_setpoint);
}

int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
