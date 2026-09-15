#include <gtest/gtest.h>
#include <stdexcept>
#include <cmath>
#include <limits>
#include "ducted_offboard/task_machine.hpp"

namespace d = ducted_offboard;
namespace {
d::ControllerConfig config() {
  d::ControllerConfig c; c.prestream_duration=0; c.arrival_dwell=.1;
  c.retry_interval=.1; return c;
}
struct Rig {
  d::ControllerConfig cfg;
  d::TaskMachine task;
  d::FlightExecutor executor;
  d::SensorState sensors;
  d::TaskSnapshot request;
  d::ExecutionFeedback feedback;
  int command=0;
  explicit Rig(d::ControllerConfig c=config()):cfg(c),task(c),executor(c) {
    sensors.connected=sensors.state_fresh=sensors.pose_fresh=true;
    sensors.velocity_fresh=sensors.extended_fresh=sensors.on_ground=true;
    sensors.mode="POSCTL"; sensors.pose={1,2,.4,1.5707963267948966};
  }
  void tick(double now,bool task_running=true) {
    if(task_running) request=task.tick(now,command,feedback);
    feedback=executor.tick(now,sensors,request.intent);
  }
  void takeoff() {
    tick(0); command=1; tick(.1);
    ASSERT_EQ(feedback.output.action.kind,d::ActionKind::SET_OFFBOARD);
    executor.acceptActionResult(feedback.output.action.id,true,true,.11);
    sensors.mode="OFFBOARD"; tick(.2);
    ASSERT_EQ(feedback.output.action.kind,d::ActionKind::ARM);
    executor.acceptActionResult(feedback.output.action.id,true,true,.21);
    sensors.armed=true;sensors.on_ground=false;tick(.3);
    ASSERT_EQ(feedback.output.phase,d::Phase::TAKEOFF);
    sensors.pose.z=1.4;tick(.4);tick(.6);tick(.7);tick(1);tick(1.2);tick(1.3);
    ASSERT_EQ(feedback.output.phase,d::Phase::HOLDING);
    ASSERT_EQ(request.phase,d::Phase::HOLDING);
  }
  void start(double now=1.4) { command=2;tick(now); }
};
}

TEST(SplitTask, StartupCommandAndStepwiseTakeoff) {
  Rig r;r.task.load({{{3,0,1,0},"mission_start"}},true);
  r.command=1;r.tick(-.1);
  EXPECT_EQ(r.request.phase,d::Phase::BOOT_WAIT_ZERO);
  EXPECT_EQ(r.feedback.output.action.kind,d::ActionKind::NONE);
  r.command=0;r.takeoff();
  EXPECT_NEAR(r.feedback.output.setpoint.y,2,1e-9);
  EXPECT_EQ(r.request.waypoint_count,1u);
  r.start();
  EXPECT_EQ(r.feedback.output.phase,d::Phase::GUIDING);
  EXPECT_NEAR(r.feedback.output.final_target.y,5,1e-9);
  EXPECT_NEAR(r.feedback.output.final_target.z,1.4,1e-9);
}

TEST(SplitTask, HoldAndPauseLoadWithoutAutomaticExecution) {
  Rig r;r.takeoff();
  ASSERT_TRUE(r.task.load({{{3,0,1,0},"mission_start"}},false));
  r.tick(1.4);EXPECT_EQ(r.feedback.output.phase,d::Phase::HOLDING);
  r.start(1.5);r.command=0;r.tick(1.6);
  EXPECT_EQ(r.request.phase,d::Phase::PAUSED);
  ASSERT_TRUE(r.task.load({{{2,0,1,0},"mission_start"}},true));
  r.tick(1.7);EXPECT_EQ(r.feedback.output.phase,d::Phase::PAUSED);
  r.command=2;r.tick(1.8);EXPECT_EQ(r.feedback.output.phase,d::Phase::GUIDING);
  EXPECT_NEAR(r.feedback.output.final_target.y,4,1e-9);
}

TEST(SplitTask, SequenceDwellAndSingleReplacement) {
  Rig r;r.task.load({{{1,0,1,0},"mission_start"},{{2,0,1,0},"mission_start"}},true);
  r.takeoff();r.start();r.tick(1.5);
  EXPECT_FALSE(r.task.load({{{4,0,1,0},"mission_start"}},true));
  r.sensors.pose={1,3,1.4,1.5707963267948966};
  r.tick(1.6);r.tick(1.65);EXPECT_EQ(r.request.waypoint_index,0u);
  r.sensors.speed=.5;r.tick(1.7);r.tick(1.8);
  r.sensors.speed=0;r.tick(1.9);r.tick(2);r.tick(2.11);
  EXPECT_EQ(r.request.waypoint_index,1u);
  ASSERT_TRUE(r.task.load({{{4,0,1,0},"mission_start"}},false));
  r.tick(2.2);EXPECT_EQ(r.feedback.output.phase,d::Phase::GUIDING);
  EXPECT_NEAR(r.feedback.output.final_target.y,6,1e-9);
}

TEST(SplitExecutor, TaskLeaseExpiryHoldsAndDoesNotResumeOnHeartbeat) {
  Rig r;r.task.load({{{3,0,1,0},"mission_start"}},true);
  r.takeoff();r.start();r.tick(1.5);
  const auto old=r.request.intent;
  r.sensors.pose.y=2.05;r.tick(2.01,false);
  EXPECT_TRUE(r.feedback.authority_lost);
  EXPECT_EQ(r.feedback.output.phase,d::Phase::PAUSED);
  EXPECT_TRUE(r.feedback.output.publish_setpoint);
  EXPECT_DOUBLE_EQ(r.feedback.output.setpoint.y,2.05);
  auto replay=old;replay.issued_at=2.02;
  r.feedback=r.executor.tick(2.02,r.sensors,replay);
  EXPECT_EQ(r.feedback.output.phase,d::Phase::PAUSED);
  r.tick(2.1);r.tick(2.2);
  EXPECT_EQ(r.request.phase,d::Phase::PAUSED);
  r.command=0;r.tick(2.3);r.command=2;r.tick(2.4);
  EXPECT_EQ(r.feedback.output.phase,d::Phase::GUIDING);
  EXPECT_GT(r.request.intent.request_id,old.request_id);
}

TEST(SplitExecutor, GroundLeaseExpiryRevokesArmingAndLateResponse) {
  Rig r;r.tick(0);r.command=1;r.tick(.1);
  const auto action=r.feedback.output.action;
  r.tick(.7,false);
  EXPECT_EQ(r.feedback.output.phase,d::Phase::IDLE_GROUND);
  EXPECT_FALSE(r.executor.actionAllowed(action,.71,r.sensors,r.request.intent));
  r.executor.acceptActionResult(action.id,true,true,.71);
  r.tick(.72,false);EXPECT_EQ(r.feedback.output.action.kind,d::ActionKind::NONE);
}

TEST(SplitExecutor, PositionLossAndManualModeExitStopPublishing) {
  for(bool mode_loss:{false,true}) {
    Rig r;r.takeoff();
    if(mode_loss)r.sensors.mode="POSCTL";else r.sensors.pose_fresh=false;
    r.tick(1.4);
    EXPECT_EQ(r.feedback.output.phase,d::Phase::ERROR);
    EXPECT_FALSE(r.feedback.output.publish_setpoint);
    r.sensors.mode="OFFBOARD";r.sensors.pose_fresh=true;r.command=2;r.tick(1.5);
    EXPECT_EQ(r.feedback.output.phase,d::Phase::ERROR);
  }
}

TEST(SplitExecutor, LandContinuesWithoutTaskAndRequiresGroundToDisarm) {
  Rig r;r.takeoff();r.command=3;r.tick(1.4);
  const auto land=r.feedback.output.action;
  ASSERT_EQ(land.kind,d::ActionKind::SET_LAND);
  r.executor.acceptActionResult(land.id,true,true,1.41);
  r.sensors.mode="AUTO.LAND";r.tick(1.5,false);
  EXPECT_FALSE(r.feedback.output.publish_setpoint);
  r.tick(2.2,false);EXPECT_EQ(r.feedback.output.phase,d::Phase::LANDING);
  r.sensors.on_ground=true;r.tick(3,false);r.tick(8.1,false);
  EXPECT_EQ(r.feedback.output.action.kind,d::ActionKind::DISARM);
  r.sensors.extended_fresh=false;r.tick(8.2,false);
  EXPECT_EQ(r.feedback.output.phase,d::Phase::ERROR);
}

TEST(SplitTask, SecondFlightUsesNewOriginAndWaitsAgain) {
  Rig r;r.task.load({{{3,0,1,0},"mission_start"}},true);
  r.takeoff();r.command=3;r.tick(1.4);r.sensors.mode="AUTO.LAND";r.tick(1.5);
  r.sensors.armed=false;r.sensors.on_ground=true;r.sensors.pose={10,20,2,0};
  r.tick(1.6);r.tick(1.7);r.command=0;r.tick(1.8);r.command=1;r.tick(1.9);
  r.sensors.mode="OFFBOARD";r.tick(2);r.sensors.armed=true;r.sensors.on_ground=false;r.tick(2.1);
  r.sensors.pose.z=3;r.tick(2.2);r.tick(2.4);r.tick(2.5);
  EXPECT_EQ(r.request.phase,d::Phase::HOLDING);
  EXPECT_EQ(r.request.waypoint_count,1u);
  EXPECT_NEAR(r.feedback.origin.z,2,1e-9);
  r.command=2;r.tick(2.6);
  EXPECT_NEAR(r.feedback.output.final_target.x,13,1e-9);
  EXPECT_NEAR(r.feedback.output.final_target.z,3,1e-9);
}

TEST(SplitConfig, EgoAndNonfiniteLimitsAreRejected) {
  auto c=config();c.execution_mode="ego";
  EXPECT_THROW(d::FlightExecutor e(c),std::invalid_argument);
  c=config();c.task_timeout=0;
  EXPECT_THROW(d::TaskMachine t(c),std::invalid_argument);
}


TEST(SplitExecutor, SupersededServiceCannotDispatchOrAdvance) {
  Rig r;r.tick(0);r.command=1;r.tick(.1);auto old=r.feedback.output.action;
  EXPECT_TRUE(r.executor.actionAllowed(old,.11,r.sensors,r.request.intent));
  r.command=0;r.tick(.12);
  EXPECT_FALSE(r.executor.actionAllowed(old,.13,r.sensors,r.request.intent));
  r.executor.acceptActionResult(old.id,true,true,.14);
  r.sensors.mode="OFFBOARD";r.tick(.2);
  EXPECT_EQ(r.feedback.output.action.kind,d::ActionKind::NONE);
  EXPECT_EQ(r.feedback.output.phase,d::Phase::IDLE_GROUND);
}
TEST(SplitExecutor, LateArmAfterLeaseRevokedHoldsWithoutClimb) {
  Rig r;r.tick(0);r.command=1;r.tick(.1);r.sensors.mode="OFFBOARD";r.tick(.2);
  auto arm=r.feedback.output.action;r.tick(.8,false);
  r.sensors.armed=true;r.sensors.on_ground=false;r.sensors.pose.z=.5;
  r.executor.acceptActionResult(arm.id,true,true,.81);r.tick(.82,false);
  EXPECT_EQ(r.feedback.output.phase,d::Phase::PAUSED);
  EXPECT_DOUBLE_EQ(r.feedback.output.setpoint.z,.5);
  EXPECT_TRUE(r.feedback.output.publish_setpoint);
}
TEST(SplitTask, ArrivalRequiresContinuousFreshFeedback) {
  Rig r;r.task.load({{{0,0,1,0},"mission_start"}},true);r.takeoff();r.start();
  r.tick(1.45);r.sensors.speed=.5;r.tick(1.5);r.tick(1.55);
  EXPECT_EQ(r.request.phase,d::Phase::GUIDING);
  r.sensors.speed=0;r.tick(1.6);r.tick(1.65);r.tick(1.7);
  EXPECT_EQ(r.request.phase,d::Phase::GUIDING);
  r.tick(1.81);EXPECT_EQ(r.request.phase,d::Phase::HOLDING);
}
TEST(SplitExecutor, NonmatchingLeaseCannotAlterTarget) {
  Rig r;r.task.load({{{3,0,1,0},"mission_start"}},true);r.takeoff();r.start();
  auto changed=r.request.intent;changed.target.x=100;changed.issued_at=1.6;
  auto f=r.executor.tick(1.6,r.sensors,changed);
  EXPECT_NE(f.output.final_target.x,100);
  f=r.executor.tick(2.,r.sensors,changed);
  EXPECT_TRUE(f.authority_lost);
}
TEST(SplitConfig, LargeYawNormalizesWithoutUnboundedLoop) {
  EXPECT_TRUE(std::isfinite(d::wrapAngle(1e100)));
  EXPECT_THROW(d::wrapAngle(std::numeric_limits<double>::infinity()),std::invalid_argument);
}

int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
