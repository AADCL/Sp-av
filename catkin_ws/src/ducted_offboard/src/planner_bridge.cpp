#include "ducted_offboard/planner_bridge.hpp"
#include "ducted_offboard/mission_loader.hpp"
#include <ducted_msgs/FlightSetpoint.h>
#include <ducted_msgs/PlannerContext.h>
#include <ducted_navigation/Navigate.h>
#include <ducted_navigation/NavigationStatus.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <sstream>
#include <thread>

namespace ducted_offboard {
namespace {
double wallNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
Pose4d fromPose(const geometry_msgs::Pose& p) {
  if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z))
    throw std::invalid_argument("nonfinite planner pose");
  return {p.position.x,p.position.y,p.position.z,
      yawFromQuaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)};
}
geometry_msgs::Pose toPose(const Pose4d& p) {
  geometry_msgs::Pose result;
  result.position.x=p.x;result.position.y=p.y;result.position.z=p.z;
  result.orientation.z=std::sin(p.yaw/2);result.orientation.w=std::cos(p.yaw/2);
  return result;
}
}

struct PlannerBridge::Impl {
  struct PoseSample { double stamp, receipt; Pose4d pose; };
  struct Event {
    int kind; // 0: target, 1: status, 2: service result
    std::string id, state, reason;
    Pose4d pose;
    double stamp{0}, receipt{0};
    bool accepted{false};
  };
  ros::Publisher context_pub;
  ros::Subscriber target_sub,status_sub,odom_sub;
  ros::ServiceClient client;
  Log log;
  mutable std::mutex mutex;
  std::condition_variable condition;
  std::thread worker;
  bool stopping{false}, queued{false}, working{false}, timed_out{false};
  bool permitted{false};
  ducted_navigation::Navigate request;
  std::string desired_id, dispatched_id, acknowledged_id, working_id;
  double deadline{0}, next_context{0};
  std::deque<Event> events;
  std::deque<PoseSample> fcu_history,external_history;
  double timeout{1}, input_timeout{.5}, future_tolerance{.05};
  double pair_skew{.05}, position_tolerance{.05}, yaw_tolerance{.1};
  std::string context_topic,target_topic,status_topic,odom_topic,service;

  Impl(ros::NodeHandle nh,ros::NodeHandle pn,Log callback):log(std::move(callback)) {
    pn.param("planner_service_timeout",timeout,1.0);
    pn.param("planner_timeout",input_timeout,.5);
    pn.param("future_tolerance",future_tolerance,.05);
    pn.param("frame_pair_max_skew",pair_skew,.05);
    pn.param("frame_position_tolerance",position_tolerance,.05);
    pn.param("frame_yaw_tolerance",yaw_tolerance,.1);
    for(double value:{timeout,input_timeout,pair_skew,position_tolerance,yaw_tolerance})
      if(!std::isfinite(value)||value<=0)throw std::invalid_argument("invalid planner bridge timeout or tolerance");
    pn.param<std::string>("planner_context_topic",context_topic,"/ctrl_cmd/planner_context");
    pn.param<std::string>("planner_target_topic",target_topic,"/ducted/control/target");
    pn.param<std::string>("planner_status_topic",status_topic,"/ducted/navigation/status");
    pn.param<std::string>("planner_command_service",service,"/ducted/navigation/command");
    pn.param<std::string>("external_odom_topic",odom_topic,"/mavros/odometry/out");
    context_pub=nh.advertise<ducted_msgs::PlannerContext>(context_topic,1,false);
    target_sub=nh.subscribe<ducted_msgs::FlightSetpoint>(target_topic,1,[this](const ducted_msgs::FlightSetpoint::ConstPtr& m){
      Event e; e.kind=0;e.id=m->request_id;e.stamp=m->header.stamp.toSec();e.receipt=wallNow();
      try {
        if(m->header.frame_id!="odom")throw std::invalid_argument("planner target frame is not odom");
        e.pose=fromPose(m->pose);
      } catch(const std::exception& error){e.kind=2;e.reason=error.what();}
      enqueue(e);
    });
    status_sub=nh.subscribe<ducted_navigation::NavigationStatus>(status_topic,10,[this](const ducted_navigation::NavigationStatus::ConstPtr& m){
      Event e;e.kind=1;e.id=m->request_id;e.state=m->state;e.reason=m->reason;
      e.stamp=m->header.stamp.toSec();e.receipt=wallNow();
      if(m->header.frame_id!="odom"){e.kind=2;e.reason="navigation status frame is not odom";}
      enqueue(e);
    });
    odom_sub=nh.subscribe<nav_msgs::Odometry>(odom_topic,20,[this](const nav_msgs::Odometry::ConstPtr& m){
      acceptPose(external_history,m->header,m->pose.pose,m->child_frame_id=="base_link");
    });
    client=nh.serviceClient<ducted_navigation::Navigate>(service,false);
    worker=std::thread([this]{run();});
  }
  ~Impl(){
    {std::lock_guard<std::mutex> lock(mutex);stopping=true;queued=false;permitted=false;}
    client.shutdown();condition.notify_all();if(worker.joinable())worker.join();
  }
  void enqueue(const Event& e){
    std::lock_guard<std::mutex> lock(mutex);
    if(events.size()>=128){
      events.clear();Event overflow;overflow.kind=2;overflow.id=desired_id;
      overflow.reason="planner event queue overflow";overflow.receipt=wallNow();events.push_back(overflow);
    }
    events.push_back(e);
  }
  void acceptPose(std::deque<PoseSample>& history,const std_msgs::Header& header,
                  const geometry_msgs::Pose& pose,bool child_ok=true){
    std::lock_guard<std::mutex> lock(mutex);
    const double stamp=header.stamp.toSec(), now=ros::Time::now().toSec();
    try {
      if(header.frame_id!="odom"||!child_ok||stamp<=0||now-stamp< -future_tolerance||
         now-stamp>input_timeout||(!history.empty()&&stamp<=history.back().stamp))
        throw std::invalid_argument("invalid pose sample");
      history.push_back({stamp,wallNow(),fromPose(pose)});
      while(history.size()>128||(!history.empty()&&stamp-history.front().stamp>1.0))history.pop_front();
    }catch(const std::exception&){history.clear();}
  }
  void run(){
    while(true){
      ducted_navigation::Navigate call;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock,[this]{return stopping||queued;});
        if(stopping)return;
        queued=false;
        if(!permitted||request.request.request_id!=desired_id)continue;
        call=request;call.request.target.header.stamp=ros::Time::now();
        working=true;working_id=call.request.request_id;timed_out=false;deadline=wallNow()+timeout;
      }
      const bool ok=client.call(call);
      Event e;e.kind=2;e.id=call.request.request_id;e.receipt=wallNow();
      {
        std::lock_guard<std::mutex> lock(mutex);
        e.accepted=ok&&call.response.accepted&&!timed_out&&e.receipt<=deadline;
        e.reason=e.receipt>deadline?"navigation service timeout":
            !ok?"navigation service unavailable":call.response.message;
        working=false;
        events.push_back(e);
      }
    }
  }
};

PlannerBridge::PlannerBridge(ros::NodeHandle n,ros::NodeHandle p,Log log):impl_(new Impl(n,p,std::move(log))){}
PlannerBridge::~PlannerBridge()=default;
void PlannerBridge::acceptFcuPose(const geometry_msgs::PoseStamped& p){impl_->acceptPose(impl_->fcu_history,p.header,p.pose);}
bool PlannerBridge::aligned(double wall,double ros_now)const{
  auto& i=*impl_;std::lock_guard<std::mutex> lock(i.mutex);
  wall=wallNow();ros_now=ros::Time::now().toSec();
  if(i.fcu_history.empty()||i.external_history.empty())return false;
  const auto fresh=[&](const Impl::PoseSample& s){return wall>=s.receipt&&wall-s.receipt<=i.input_timeout&&
      ros_now-s.stamp>=-i.future_tolerance&&ros_now-s.stamp<=i.input_timeout;};
  if(!fresh(i.external_history.back())||!fresh(i.fcu_history.back()))return false;
  // Streams arrive independently. Use the newest source-time pair for which
  // both samples have arrived, without rejecting a healthy older pair during
  // the few milliseconds before its successor's counterpart is delivered.
  for(auto external=i.external_history.rbegin();external!=i.external_history.rend();++external){
    if(!fresh(*external))continue;
    const auto nearest=std::min_element(i.fcu_history.begin(),i.fcu_history.end(),[&](const Impl::PoseSample& a,const Impl::PoseSample& b){
      return std::abs(a.stamp-external->stamp)<std::abs(b.stamp-external->stamp);
    });
    if(!fresh(*nearest)||std::abs(nearest->stamp-external->stamp)>i.pair_skew)continue;
    const auto& a=external->pose;const auto& b=nearest->pose;
    // A newer matched divergence must revoke immediately, never search past it
    // for an older successful match.
    return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z))<=i.position_tolerance&&
        std::abs(wrapAngle(a.yaw-b.yaw))<=i.yaw_tolerance;
  }
  return false;
}
void PlannerBridge::poll(Controller& controller,double wall,double ros_now){
  auto& i=*impl_;std::deque<Impl::Event> events;
  {
    std::lock_guard<std::mutex> lock(i.mutex);events.swap(i.events);
    wall=wallNow();ros_now=ros::Time::now().toSec();
    if(i.working&&!i.timed_out&&wall>i.deadline){
      i.timed_out=true;Impl::Event e;e.kind=2;e.id=i.working_id;e.receipt=wall;
      e.reason="navigation service timeout";events.push_back(e);
    }
  }
  for(const auto& e:events){
    if(e.kind==0)controller.acceptPlannerTarget(e.id,e.pose,e.stamp,e.receipt,ros_now);
    else if(e.kind==1){
      controller.acceptPlannerStatus(e.id,e.state,e.reason,e.stamp,e.receipt,ros_now);
      if(e.state=="IDLE"&&e.reason=="planner context ready"&&ros_now-e.stamp>=-i.future_tolerance&&
          ros_now-e.stamp<=i.input_timeout&&wall-e.receipt<=i.input_timeout){
        std::lock_guard<std::mutex> lock(i.mutex);
        if(e.id==i.desired_id)i.acknowledged_id=e.id;
      }
    }else{
      controller.acceptPlannerResponse(e.id,e.accepted,e.reason,wall);
      i.log(e.accepted?"INFO":"WARN","navigation response "+e.id+": "+e.reason);
    }
  }
}
void PlannerBridge::update(const ControllerOutput& o,double wall,const ros::Time& stamp){
  auto& i=*impl_;bool publish=false,dispatch=false;
  {
    std::lock_guard<std::mutex> lock(i.mutex);
    publish=wall>=i.next_context||i.desired_id!=o.planner_request_id||i.permitted!=o.planner_ready;
    if(i.desired_id!=o.planner_request_id){i.acknowledged_id.clear();i.dispatched_id.clear();i.queued=false;}
    i.desired_id=o.planner_request_id;i.permitted=o.planner_ready;
    if(!i.permitted)i.queued=false;
    if(publish)i.next_context=wall+.1;
    if(i.permitted&&!i.desired_id.empty()&&i.acknowledged_id==i.desired_id&&i.dispatched_id!=i.desired_id){
      if(!i.working&&!i.queued){
        i.request.request.command="goal";i.request.request.request_id=i.desired_id;
        i.request.request.target.header.frame_id="odom";
        i.request.request.target.pose=toPose(o.final_target);
        i.dispatched_id=i.desired_id;i.queued=true;dispatch=true;
      }
    }
  }
  if(publish){
    ducted_msgs::PlannerContext context;
    context.header.stamp=stamp;context.header.frame_id="odom";
    context.request_id=o.planner_request_id;context.ready=o.planner_ready;
    context.reference_valid=o.reference_valid;context.takeoff_reference_z=o.takeoff_reference_z;
    i.context_pub.publish(context);
  }
  if(dispatch){i.log("INFO","navigation goal submitted "+o.planner_request_id);i.condition.notify_one();}
}
std::string PlannerBridge::configuration()const{
  const auto& i=*impl_;std::ostringstream s;
  s<<"planner_service_timeout: "<<i.timeout<<"\nframe_pair_max_skew: "<<i.pair_skew
   <<"\nframe_position_tolerance: "<<i.position_tolerance<<"\nframe_yaw_tolerance: "<<i.yaw_tolerance
   <<"\nplanner_context_topic: "<<i.context_topic<<"\nplanner_target_topic: "<<i.target_topic
   <<"\nplanner_status_topic: "<<i.status_topic<<"\nplanner_command_service: "<<i.service
   <<"\nexternal_odom_topic: "<<i.odom_topic<<'\n';return s.str();
}
}
