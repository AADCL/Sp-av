#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <unistd.h>

#include "ducted_offboard/OffboardStatus.h"
#include "ducted_offboard/task_machine.hpp"
#include "ducted_offboard/mission_loader.hpp"

namespace ducted_offboard {
namespace {

constexpr double kPi = 3.14159265358979323846;

double steadyNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string utcNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&value, &utc);
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

geometry_msgs::Pose toRosPose(const Pose4d& pose) {
  geometry_msgs::Pose output;
  output.position.x = pose.x;
  output.position.y = pose.y;
  output.position.z = pose.z;
  output.orientation.z = std::sin(pose.yaw * 0.5);
  output.orientation.w = std::cos(pose.yaw * 0.5);
  return output;
}

bool finite(double value) { return std::isfinite(value); }

class LogSink {
 public:
  LogSink(const std::string& root, std::uintmax_t max_bytes,
                int keep_files, const std::string& configuration)
      : max_bytes_(max_bytes), keep_files_(std::max(1, keep_files)) {
    if (root.empty() || max_bytes_ == 0) {
      throw std::invalid_argument("invalid logging configuration");
    }
    boost::filesystem::path root_path(root);
    boost::filesystem::create_directories(root_path);
    if (!boost::filesystem::is_directory(root_path)) {
      throw std::runtime_error("log_root is not a directory: " + root);
    }
    std::string name = sessionName();
    session_path_ = root_path / name;
    int suffix = 1;
    while (boost::filesystem::exists(session_path_)) {
      session_path_ = root_path / (name + "_" + std::to_string(suffix++));
    }
    boost::filesystem::create_directories(session_path_);
    event_path_ = session_path_ / "events.log";
    telemetry_path_ = session_path_ / "telemetry.csv";
    event_stream_.open(event_path_.string(), std::ios::out | std::ios::app);
    telemetry_stream_.open(telemetry_path_.string(),
                           std::ios::out | std::ios::app);
    std::ofstream config_stream((session_path_ / "config.yaml").string(),
                                std::ios::out | std::ios::trunc);
    if (!event_stream_ || !telemetry_stream_ || !config_stream) {
      throw std::runtime_error("cannot create offboard session logs in " +
                               session_path_.string());
    }
    config_stream << configuration;
    config_stream.close();
    writeTelemetryHeader();
    event("INFO", "session started at " + session_path_.string());
  }

  const std::string path() const { return session_path_.string(); }

  void event(const std::string& level, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    rotateIfNeeded();
    event_stream_ << utcNow() << " [" << level << "] " << message << '\n';
    event_stream_.flush();
    if (level == "ERROR") {
      ROS_ERROR_STREAM(message);
    } else if (level == "WARN") {
      ROS_WARN_STREAM(message);
    } else {
      ROS_INFO_STREAM(message);
    }
  }

  void telemetry(double monotonic, const ros::Time& stamp,
                 const ControllerOutput& output, const SensorState& sensors,
                 int command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rotateIfNeeded(telemetry_stream_, telemetry_path_)) {
      writeTelemetryHeader();
    }
    const double horizontal = std::hypot(output.final_target.x - sensors.pose.x,
                                         output.final_target.y - sensors.pose.y);
    const double vertical = output.final_target.z - sensors.pose.z;
    const double yaw = wrapAngle(output.final_target.yaw - sensors.pose.yaw);
    telemetry_stream_ << utcNow() << ',' << std::fixed << std::setprecision(6)
                      << stamp.toSec() << ',' << monotonic << ','
                      << phaseName(output.phase) << ',' << command << ','
                      << sensors.mode << ',' << (sensors.armed ? 1 : 0) << ','
                      << output.waypoint_index << ',' << output.waypoint_count
                      << ',' << sensors.pose.x << ',' << sensors.pose.y << ','
                      << sensors.pose.z << ',' << sensors.pose.yaw << ','
                      << output.final_target.x << ',' << output.final_target.y
                      << ',' << output.final_target.z << ','
                      << output.final_target.yaw << ',' << horizontal << ','
                      << vertical << ',' << yaw << ',' << sensors.speed << ','
                      << (output.publish_setpoint ? 1 : 0) << ',';
    for (const char character : output.reason) {
      telemetry_stream_ << (character == ',' ? ';' : character);
    }
    telemetry_stream_ << ',' << output.planner_request_id << ',' << output.planner_state
                      << ',' << output.planner_target_age << ',' << output.setpoint.x
                      << ',' << output.setpoint.y << ',' << output.setpoint.z << ',' << output.task_phase
                      << ',' << output.executor_phase << ',' << output.task_generation << ',' << output.task_heartbeat_age << '\n';
    telemetry_stream_.flush();
  }

  void warningThrottled(const std::string& key, const std::string& message,
                        double interval = 2.0) {
    bool emit = false;
    const double now = steadyNow();
    {
      std::lock_guard<std::mutex> lock(warning_mutex_);
      const auto found = warning_times_.find(key);
      if (found == warning_times_.end() || now - found->second >= interval) {
        warning_times_[key] = now;
        emit = true;
      }
    }
    if (emit) {
      event("WARN", message);
    }
  }

 private:
  static std::string sessionName() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t value = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&value, &utc);
    std::ostringstream stream;
    stream << "session_" << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << '_'
           << static_cast<long>(getpid());
    return stream.str();
  }

  bool rotateIfNeeded(std::ofstream& stream,
                      const boost::filesystem::path& path) {
    if (stream.tellp() < 0 ||
        static_cast<std::uintmax_t>(stream.tellp()) < max_bytes_) {
      return false;
    }
    stream.close();
    for (int index = keep_files_ - 1; index >= 1; --index) {
      const boost::filesystem::path source =
          path.string() + "." + std::to_string(index);
      const boost::filesystem::path target =
          path.string() + "." + std::to_string(index + 1);
      if (boost::filesystem::exists(target)) {
        boost::filesystem::remove(target);
      }
      if (boost::filesystem::exists(source)) {
        boost::filesystem::rename(source, target);
      }
    }
    const boost::filesystem::path first = path.string() + ".1";
    if (boost::filesystem::exists(first)) {
      boost::filesystem::remove(first);
    }
    if (boost::filesystem::exists(path)) {
      boost::filesystem::rename(path, first);
    }
    stream.open(path.string(), std::ios::out | std::ios::trunc);
    return true;
  }

  void rotateIfNeeded() {
    rotateIfNeeded(event_stream_, event_path_);
  }

  void writeTelemetryHeader() {
    telemetry_stream_
        << "utc,ros_time,monotonic,phase,command,mode,armed,waypoint_index,"
           "waypoint_count,actual_x,actual_y,actual_z,actual_yaw,target_x,"
           "target_y,target_z,target_yaw,horizontal_error,vertical_error,"
           "yaw_error,speed,setpoint_active,reason,planner_request_id,planner_state,"
           "planner_target_age,setpoint_x,setpoint_y,setpoint_z,task_phase,executor_phase,task_generation,task_heartbeat_age\n";
  }

  std::uintmax_t max_bytes_;
  int keep_files_;
  boost::filesystem::path session_path_;
  boost::filesystem::path event_path_;
  boost::filesystem::path telemetry_path_;
  std::ofstream event_stream_;
  std::ofstream telemetry_stream_;
  std::mutex mutex_;
  std::mutex warning_mutex_;
  std::unordered_map<std::string, double> warning_times_;
};

// Disk/console writes never run on either control loop. Queue memory is bounded.
class SessionLogger {
 public:
  SessionLogger(const std::string& root, std::uintmax_t bytes, int keep,
                const std::string& config): sink_(root,bytes,keep,config) {
    thread_=std::thread(&SessionLogger::run,this);
  }
  ~SessionLogger() {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_=true;}
    condition_.notify_one(); if(thread_.joinable())thread_.join();
  }
  std::string path() const {return sink_.path();}
  void event(const std::string& level,const std::string& message) {
    Item i;i.level=level;i.message=message;i.utc=utcNow();enqueue(std::move(i));
  }
  void telemetry(double now,const ros::Time& stamp,const ControllerOutput& output,
                 const SensorState& sensors,int command) {
    Item i;i.csv=true;i.now=now;i.stamp=stamp;i.output=output;i.sensors=sensors;
    i.command=command;enqueue(std::move(i));
  }
  void warningThrottled(const std::string& key,const std::string& message,double interval=2.) {
    const double now=steadyNow();bool emit=false;
    {std::lock_guard<std::mutex> lock(warning_mutex_);
      auto found=warnings_.find(key);
      if(found==warnings_.end()||now-found->second>=interval){warnings_[key]=now;emit=true;}}
    if(emit)event("WARN",message);
  }
 private:
  struct Item {bool csv=false;std::string level,message,utc;double now=0;ros::Time stamp;
    ControllerOutput output;SensorState sensors;int command=-1;};
  void enqueue(Item item) {
    {std::lock_guard<std::mutex> lock(mutex_);
      if(queue_.size()>=1024) {
        ++dropped_;
        if(item.csv)return;
        auto csv=std::find_if(queue_.begin(),queue_.end(),[](const Item& q){return q.csv;});
        if(csv!=queue_.end())queue_.erase(csv);else queue_.pop_front();
      }
      queue_.push_back(std::move(item));}
    condition_.notify_one();
  }
  void run() {
    for(;;) {
      Item i;std::size_t dropped=0;
      {std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock,[this]{return stopping_||!queue_.empty();});
        if(queue_.empty()&&stopping_)return;
        i=std::move(queue_.front());queue_.pop_front();dropped=dropped_;dropped_=0;}
      try {
        if(dropped)sink_.event("WARN","asynchronous log queue dropped "+std::to_string(dropped)+" records");
        if(i.csv)sink_.telemetry(i.now,i.stamp,i.output,i.sensors,i.command);
        else sink_.event(i.level,"[observed "+i.utc+"] "+i.message);
      }catch(const std::exception& e){ROS_ERROR_THROTTLE(5.,"Log writer error: %s",e.what());}
    }
  }
  LogSink sink_;
  std::mutex mutex_,warning_mutex_;
  std::condition_variable condition_;
  std::deque<Item> queue_;
  std::unordered_map<std::string,double> warnings_;
  bool stopping_=false;std::size_t dropped_=0;
  std::thread thread_;
};

struct ServiceResult {
  std::uint64_t id{0};
  ActionKind kind{ActionKind::NONE};
  bool transport_ok{false};
  bool accepted{false};
};

class ServiceWorker {
 public:
  using Callback = std::function<void(const ServiceResult&)>;
  using Validator = std::function<bool(const ActionRequest&)>;

  ServiceWorker(ros::NodeHandle& node, const std::string& set_mode_service,
                const std::string& arming_service, Callback callback, Validator validator)
      : mode_client_(node.serviceClient<mavros_msgs::SetMode>(set_mode_service,
                                                              false)),
        arm_client_(node.serviceClient<mavros_msgs::CommandBool>(arming_service,
                                                                 false)),
        callback_(std::move(callback)), validator_(std::move(validator)) {
    thread_=std::thread(&ServiceWorker::run,this);
  }

  ~ServiceWorker() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      queued_ = false;
    }
    mode_client_.shutdown();
    arm_client_.shutdown();
    condition_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  bool dispatch(const ActionRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || queued_ || working_) {
      return false;
    }
    request_ = request;
    queued_ = true;
    condition_.notify_one();
    return true;
  }

 private:
  ServiceResult call(const ActionRequest& request) {
    ServiceResult result;
    result.id = request.id;
    result.kind = request.kind;
    if (request.kind == ActionKind::SET_OFFBOARD ||
        request.kind == ActionKind::SET_LAND) {
      mavros_msgs::SetMode service;
      service.request.base_mode = 0;
      service.request.custom_mode = request.kind == ActionKind::SET_OFFBOARD
                                        ? "OFFBOARD"
                                        : "AUTO.LAND";
      result.transport_ok = mode_client_.call(service);
      result.accepted = result.transport_ok && service.response.mode_sent;
    } else if (request.kind == ActionKind::ARM ||
               request.kind == ActionKind::DISARM) {
      mavros_msgs::CommandBool service;
      service.request.value = request.kind == ActionKind::ARM;
      result.transport_ok = arm_client_.call(service);
      result.accepted = result.transport_ok && service.response.success;
    }
    return result;
  }

  void run() {
    while (true) {
      ActionRequest request;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stopping_ || queued_; });
        if (stopping_) {
          return;
        }
        request = request_;
        queued_ = false;
        working_ = true;
      }
      ServiceResult result;result.id=request.id;result.kind=request.kind;
      // Dispatch is the linearization point. Superseded queued requests never call MAVROS.
      if(validator_(request))result=call(request);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        working_ = false;
      }
      callback_(result);
    }
  }

  ros::ServiceClient mode_client_;
  ros::ServiceClient arm_client_;
  Callback callback_;
  Validator validator_;
  std::mutex mutex_;
  std::condition_variable condition_;
  ActionRequest request_;
  bool queued_{false};
  bool working_{false};
  bool stopping_{false};
  std::thread thread_;
};

struct TimedSample {
  ros::Time source_stamp;
  double receipt_time{-1.0};

  bool accept(const ros::Time& stamp, double now) {
    if (stamp.isZero() || (!source_stamp.isZero() && stamp <= source_stamp)) {
      return false;
    }
    source_stamp = stamp;
    receipt_time = now;
    return true;
  }

  bool fresh(double now, const ros::Time& ros_now, double timeout,
             double future_tolerance) const {
    if (receipt_time < 0.0 || now - receipt_time > timeout ||
        source_stamp.isZero()) {
      return false;
    }
    const double age = (ros_now - source_stamp).toSec();
    return age >= -future_tolerance && age <= timeout;
  }
};

struct CachedSensors {
  mavros_msgs::State state;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped velocity;
  mavros_msgs::ExtendedState extended;
  TimedSample state_time;
  TimedSample pose_time;
  TimedSample velocity_time;
  TimedSample extended_time;
};

class OffboardNode {
 public:
  OffboardNode() : node_(), private_node_("~") {
    loadConfiguration();
    logger_.reset(new SessionLogger(log_root_, log_max_bytes_, log_keep_files_,
                                    configurationText()));
    task_.reset(new TaskMachine(controller_config_));
    executor_.reset(new FlightExecutor(controller_config_));
    task_snapshot_=std::make_shared<const TaskSnapshot>();
    feedback_snapshot_=std::make_shared<const ExecutionFeedback>();

    setpoint_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(setpoint_topic_, 20);
    status_publisher_ =
        node_.advertise<ducted_offboard::OffboardStatus>(status_topic_, 10, true);
    state_subscriber_ = node_.subscribe(state_topic_, 10,
                                        &OffboardNode::stateCallback, this);
    pose_subscriber_ = node_.subscribe(pose_topic_, 20,
                                       &OffboardNode::poseCallback, this);
    velocity_subscriber_ = node_.subscribe(
        velocity_topic_, 20, &OffboardNode::velocityCallback, this);
    extended_subscriber_ = node_.subscribe(
        extended_state_topic_, 10, &OffboardNode::extendedCallback, this);

    configureMissionSource();
    service_worker_.reset(new ServiceWorker(
        node_, set_mode_service_, arming_service_,
        [this](const ServiceResult& result) { serviceResultCallback(result); },
        [this](const ActionRequest& request) {
          std::lock_guard<std::mutex> lock(executor_mutex_);
          auto task=std::atomic_load(&task_snapshot_);
          double now=steadyNow();ros::Time stamp=ros::Time::now();
          const auto sensors=sensorSnapshot(now,stamp);
          const bool allowed=executor_->actionAllowed(request,now,sensors,task->intent);
          if(!allowed)logger_->event("WARN","[Executor] canceled stale service id="+std::to_string(request.id));
          return allowed;
        }));
    logger_->event("INFO", "ducted_offboard initialized; command parameter is " +
                               command_param_);
  }

  int run() {
    ros::AsyncSpinner spinner(3);spinner.start();
    task_thread_=std::thread(&OffboardNode::taskLoop,this);
    int result=0;
    try {result=executionLoop();}
    catch(const std::exception& e){logger_->event("ERROR",std::string("[Executor] stopped: ")+e.what());result=2;}
    stopping_=true;
    // Wake parameter/XMLRPC waits during process shutdown; no service calls here.
    ros::shutdown();
    if(task_thread_.joinable())task_thread_.join();
    spinner.stop();return result;
  }

 private:
  struct MissionInput {std::vector<Waypoint> points;bool sequence;ros::Time stamp;};

  void taskLoop() {
    auto next=std::chrono::steady_clock::now();
    std::string last_phase,last_reason;std::uint64_t last_request=0;
    try {
      while(ros::ok()&&!stopping_) {
#ifdef DUCTED_OFFBOARD_TEST_HOOKS
        // Test binary only, never compiled into the installed production node.
        double stall=0.;private_node_.getParam("test_task_stall_seconds",stall);
        if(stall>0) {
          private_node_.setParam("test_task_stall_seconds",0.);
          auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(static_cast<int>(std::min(10.,stall)*1000));
          while(!stopping_&&std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
#endif
        int command=-1;node_.getParamCached(command_param_,command);
        if(command!=last_observed_command_) {
          logger_->event("INFO","[Task] command observed: "+std::to_string(command));last_observed_command_=command;
        }
        auto feedback=std::atomic_load(&feedback_snapshot_);
        // Refresh state BEFORE applying queued tasks: a stalled task cannot load into stale GUIDING.
        auto state=task_->tick(steadyNow(),command,*feedback);
        std::shared_ptr<const MissionInput> pending=std::atomic_exchange(&pending_mission_,std::shared_ptr<const MissionInput>{});
        if(pending) {
          if(messageFresh(pending->stamp)&&task_->load(pending->points,pending->sequence))
            logger_->event("INFO","[Task] "+std::string(pending->sequence?"waypoint sequence accepted":"single target accepted"));
          else logger_->warningThrottled("mission_rejected","[Task] mission rejected: stale input or incompatible task phase");
          state=task_->tick(steadyNow(),command,*feedback);
        }
        std::atomic_store(&task_snapshot_,std::make_shared<const TaskSnapshot>(state));
        const auto phase=phaseName(state.phase);
        if(phase!=last_phase||state.reason!=last_reason) {
          logger_->event(state.phase==Phase::ERROR?"ERROR":"INFO","[Task] "+phase+": "+state.reason);
          last_phase=phase;last_reason=state.reason;
        }
        if(state.intent.request_id!=last_request) {
          logger_->event("INFO","[Task] execution request generation="+std::to_string(state.intent.generation)+
              " id="+std::to_string(state.intent.request_id)+" waypoint="+std::to_string(state.waypoint_index+1));
          last_request=state.intent.request_id;
        }
        next+=std::chrono::milliseconds(50);
        if(next<std::chrono::steady_clock::now())next=std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next);
      }
    } catch(const std::exception& e) {
      logger_->event("ERROR",std::string("[Task] loop stopped; executor heartbeat protection active: ")+e.what());
    }
  }

  int executionLoop() {
    auto next=std::chrono::steady_clock::now();
    double next_status=0,next_telemetry=0;std::uint64_t acknowledged=0;
    while(ros::ok()) {
      auto task=std::atomic_load(&task_snapshot_);
      double now=steadyNow();ros::Time stamp=ros::Time::now();
      SensorState sensors;ExecutionFeedback feedback;
      {
        std::lock_guard<std::mutex> lock(executor_mutex_);
        sensors=sensorSnapshot(now,stamp);
        drainServiceResults(now);
        feedback=executor_->tick(now,sensors,task->intent);
        if(feedback.output.publish_setpoint)publishSetpoint(feedback.output.setpoint,stamp);
      }
      std::atomic_store(&feedback_snapshot_,std::make_shared<const ExecutionFeedback>(feedback));
      const auto output=combinedStatus(*task,feedback,now);
      logStateChanges(feedback.output);
      if(feedback.request_id!=acknowledged) {
        logger_->event("INFO","[Executor] accepted request generation="+std::to_string(feedback.generation)+" id="+std::to_string(feedback.request_id));
        acknowledged=feedback.request_id;
      }
      if(output.publish_setpoint)logFirstSetpoint(output);
      if(output.action.kind!=ActionKind::NONE) {
        logger_->event("INFO","[Executor] service request "+actionName(output.action.kind)+" id="+std::to_string(output.action.id));
        if(!service_worker_->dispatch(output.action)) {
          logger_->event("WARN","[Executor] service worker still busy; request not dispatched");
          std::lock_guard<std::mutex> lock(executor_mutex_);
          executor_->acceptActionResult(output.action.id,false,false,steadyNow());
        }
      }
      if(now>=next_status){publishStatus(output,sensors,task->command,stamp);next_status=now+1./status_rate_;}
      if(now>=next_telemetry){logger_->telemetry(now,stamp,output,sensors,task->command);next_telemetry=now+1./telemetry_rate_;}
      if(output.should_exit)return output.exit_code;
      next+=std::chrono::milliseconds(50);
      if(next<std::chrono::steady_clock::now())next=std::chrono::steady_clock::now();
      std::this_thread::sleep_until(next);
    }
    return 0;
  }

  template <typename T>
  void parameter(const std::string& name, T& value, const T& default_value) {
    private_node_.param(name, value, default_value);
  }

  void loadConfiguration() {
    parameter("execution_mode", controller_config_.execution_mode, std::string("direct"));
    controller_config_.session_id = boost::uuids::to_string(boost::uuids::random_generator()());
    parameter("task_timeout", controller_config_.task_timeout, .5);
    parameter("target_source", target_source_, std::string("topic"));
    parameter("topic_mode", topic_mode_, std::string("single"));
    parameter("mission_frame", mission_frame_, std::string("mission_start"));
    parameter("mission_file", mission_file_, std::string());
    parameter("control_rate", control_rate_, 20.0);
    parameter("status_rate", status_rate_, 10.0);
    parameter("telemetry_rate", telemetry_rate_, 5.0);
    parameter("state_timeout", state_timeout_, 3.0);
    parameter("pose_timeout", pose_timeout_, 0.5);
    parameter("velocity_timeout", velocity_timeout_, 0.5);
    parameter("extended_state_timeout", extended_timeout_, 3.0);
    parameter("future_tolerance", future_tolerance_, 0.05);
    parameter("target_message_max_age", target_message_max_age_, 2.0);
    parameter("log_root", log_root_,
              std::string("/home/nrc/catkin_ws/logs/offboard_control"));
    int log_max_bytes = 5242880;
    parameter("log_max_bytes", log_max_bytes, 5242880);
    if (log_max_bytes <= 0) {
      throw std::invalid_argument("log_max_bytes must be positive");
    }
    log_max_bytes_ = static_cast<std::uintmax_t>(log_max_bytes);
    parameter("log_keep_files", log_keep_files_, 3);

    parameter("prestream_duration", controller_config_.prestream_duration, 2.0);
    parameter("retry_interval", controller_config_.retry_interval, 5.0);
    parameter("feedback_timeout", controller_config_.feedback_timeout, 2.0);
    parameter("service_timeout", controller_config_.service_timeout, 3.0);
    parameter("max_attempts", controller_config_.max_attempts, 4);
    parameter("takeoff_height", controller_config_.takeoff_height, 1.0);
    parameter("horizontal_speed", controller_config_.horizontal_speed, 0.3);
    parameter("vertical_speed", controller_config_.vertical_speed, 0.2);
    double yaw_rate_deg = 20.0;
    parameter("yaw_rate_deg", yaw_rate_deg, 20.0);
    controller_config_.yaw_rate = yaw_rate_deg * kPi / 180.0;
    parameter("horizontal_tolerance", controller_config_.horizontal_tolerance,
              0.15);
    parameter("vertical_tolerance", controller_config_.vertical_tolerance,
              0.10);
    double yaw_tolerance_deg = 10.0;
    parameter("yaw_tolerance_deg", yaw_tolerance_deg, 10.0);
    controller_config_.yaw_tolerance = yaw_tolerance_deg * kPi / 180.0;
    parameter("speed_tolerance", controller_config_.speed_tolerance, 0.15);
    parameter("arrival_dwell", controller_config_.arrival_dwell, 1.0);
    parameter("takeoff_timeout", controller_config_.takeoff_timeout, 30.0);
    parameter("waypoint_timeout", controller_config_.waypoint_timeout, 120.0);
    parameter("landing_timeout", controller_config_.landing_timeout, 120.0);
    parameter("ground_disarm_delay", controller_config_.ground_disarm_delay,
              5.0);

    parameter("command_param", command_param_, std::string("/ctrl_cmd/state"));
    parameter("status_topic", status_topic_, std::string("/ctrl_cmd/status"));
    parameter("target_topic", target_topic_, std::string("/ctrl_cmd/target"));
    parameter("waypoints_topic", waypoints_topic_,
              std::string("/ctrl_cmd/waypoints"));
    parameter("setpoint_topic", setpoint_topic_,
              std::string("/mavros/setpoint_position/local"));
    parameter("state_topic", state_topic_, std::string("/mavros/state"));
    parameter("pose_topic", pose_topic_,
              std::string("/mavros/local_position/pose"));
    parameter("velocity_topic", velocity_topic_,
              std::string("/mavros/local_position/velocity_local"));
    parameter("extended_state_topic", extended_state_topic_,
              std::string("/mavros/extended_state"));
    parameter("set_mode_service", set_mode_service_,
              std::string("/mavros/set_mode"));
    parameter("arming_service", arming_service_,
              std::string("/mavros/cmd/arming"));

    validateDirectConfig(controller_config_);
    for(double value:{control_rate_,status_rate_,telemetry_rate_,state_timeout_,pose_timeout_,velocity_timeout_,extended_timeout_,future_tolerance_,target_message_max_age_})
      if(!std::isfinite(value))throw std::invalid_argument("node parameters must be finite");
    if (control_rate_ != 20.0 || status_rate_ <= 0.0 || telemetry_rate_ <= 0.0 ||
        state_timeout_ <= 0.0 || pose_timeout_ <= 0.0 ||
        velocity_timeout_ <= 0.0 || extended_timeout_ <= 0.0 ||
        future_tolerance_ < 0.0 || target_message_max_age_ <= 0.0 ||
        log_keep_files_ <= 0 ||
        (mission_frame_ != "mission_start" && mission_frame_ != "odom")) {
      throw std::invalid_argument("invalid node configuration");
    }
  }

  std::string configurationText() const {
    std::ostringstream stream;
    stream << "execution_mode: " << controller_config_.execution_mode << '\n'
           << "session_id: " << controller_config_.session_id << '\n'
           << "task_rate: 20\ntask_timeout: " << controller_config_.task_timeout << '\n'
           << "target_source: " << target_source_ << '\n'
           << "topic_mode: " << topic_mode_ << '\n'
           << "mission_frame: " << mission_frame_ << '\n'
           << "mission_file: " << mission_file_ << '\n'
           << "control_rate: " << control_rate_ << '\n'
           << "status_rate: " << status_rate_ << '\n'
           << "telemetry_rate: " << telemetry_rate_ << '\n'
           << "prestream_duration: " << controller_config_.prestream_duration
           << '\n'
           << "retry_interval: " << controller_config_.retry_interval << '\n'
           << "feedback_timeout: " << controller_config_.feedback_timeout
           << '\n'
           << "service_timeout: " << controller_config_.service_timeout << '\n'
           << "max_attempts: " << controller_config_.max_attempts << '\n'
           << "takeoff_height: " << controller_config_.takeoff_height << '\n'
           << "horizontal_speed: " << controller_config_.horizontal_speed
           << '\n'
           << "vertical_speed: " << controller_config_.vertical_speed << '\n'
           << "yaw_rate_rad: " << controller_config_.yaw_rate << '\n'
           << "horizontal_tolerance: "
           << controller_config_.horizontal_tolerance << '\n'
           << "vertical_tolerance: " << controller_config_.vertical_tolerance
           << '\n'
           << "yaw_tolerance_rad: " << controller_config_.yaw_tolerance << '\n'
           << "speed_tolerance: " << controller_config_.speed_tolerance << '\n'
           << "arrival_dwell: " << controller_config_.arrival_dwell << '\n'
           << "takeoff_timeout: " << controller_config_.takeoff_timeout << '\n'
           << "waypoint_timeout: " << controller_config_.waypoint_timeout << '\n'
           << "landing_timeout: " << controller_config_.landing_timeout << '\n'
           << "ground_disarm_delay: " << controller_config_.ground_disarm_delay
           << '\n'
           << "state_timeout: " << state_timeout_ << '\n'
           << "pose_timeout: " << pose_timeout_ << '\n'
           << "velocity_timeout: " << velocity_timeout_ << '\n'
           << "extended_state_timeout: " << extended_timeout_ << '\n'
           << "future_tolerance: " << future_tolerance_ << '\n'
           << "target_message_max_age: " << target_message_max_age_ << '\n'
           << "log_root: " << log_root_ << '\n'
           << "log_max_bytes: " << log_max_bytes_ << '\n'
           << "log_keep_files: " << log_keep_files_ << '\n'
           << "command_param: " << command_param_ << '\n'
           << "status_topic: " << status_topic_ << '\n'
           << "target_topic: " << target_topic_ << '\n'
           << "waypoints_topic: " << waypoints_topic_ << '\n'
           << "setpoint_topic: " << setpoint_topic_ << '\n'
           << "state_topic: " << state_topic_ << '\n'
           << "pose_topic: " << pose_topic_ << '\n'
           << "velocity_topic: " << velocity_topic_ << '\n'
           << "extended_state_topic: " << extended_state_topic_ << '\n'
           << "set_mode_service: " << set_mode_service_ << '\n'
           << "arming_service: " << arming_service_ << '\n';
    return stream.str();
  }

  void configureMissionSource() {
    if (target_source_ == "file") {
      task_->load(loadMissionFile(mission_file_, mission_frame_), true);
      logger_->event("INFO", "mission loaded from file: " + mission_file_);
    } else if (target_source_ == "param") {
      XmlRpc::XmlRpcValue value;
      if (!private_node_.getParam("waypoints", value)) {
        throw std::invalid_argument("~waypoints is required for target_source=param");
      }
      task_->load(parseMissionParam(value, mission_frame_), true);
      logger_->event("INFO", "mission loaded from private parameter ~waypoints");
    } else if (target_source_ == "topic") {
      if (topic_mode_ == "single") {
        target_subscriber_ = node_.subscribe(target_topic_, 5,
                                              &OffboardNode::targetCallback, this);
      } else if (topic_mode_ == "sequence") {
        waypoints_subscriber_ = node_.subscribe(
            waypoints_topic_, 2, &OffboardNode::waypointsCallback, this);
      } else {
        throw std::invalid_argument("topic_mode must be single or sequence");
      }
    } else {
      throw std::invalid_argument("target_source must be file, param or topic");
    }
  }

  bool messageFresh(const ros::Time& stamp) const {
    if (stamp.isZero()) {
      return false;
    }
    const double age = (ros::Time::now() - stamp).toSec();
    return age >= -future_tolerance_ && age <= target_message_max_age_;
  }

  Waypoint waypointFromPose(const geometry_msgs::PoseStamped& message,
                            const std::string& fallback_frame,
                            const ros::Time& fallback_stamp) const {
    const ros::Time stamp = message.header.stamp.isZero()
                                ? fallback_stamp
                                : message.header.stamp;
    if (!messageFresh(stamp)) {
      throw std::invalid_argument("target timestamp is missing, stale or future");
    }
    Waypoint waypoint;
    waypoint.frame_id = message.header.frame_id.empty()
                            ? fallback_frame
                            : message.header.frame_id;
    if (waypoint.frame_id != "mission_start" && waypoint.frame_id != "odom") {
      throw std::invalid_argument("target frame must be mission_start or odom");
    }
    waypoint.pose.x = message.pose.position.x;
    waypoint.pose.y = message.pose.position.y;
    waypoint.pose.z = message.pose.position.z;
    const auto& q = message.pose.orientation;
    waypoint.pose.yaw = yawFromQuaternion(q.x, q.y, q.z, q.w);
    if (!finite(waypoint.pose.x) || !finite(waypoint.pose.y) ||
        !finite(waypoint.pose.z)) {
      throw std::invalid_argument("target position must be finite");
    }
    return waypoint;
  }

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    try {
      const Waypoint waypoint = waypointFromPose(*message, mission_frame_,
                                                 message->header.stamp);
      auto pending=std::make_shared<const MissionInput>(MissionInput{{waypoint},false,message->header.stamp});
      std::atomic_store(&pending_mission_,pending);
    } catch (const std::exception& error) {
      logger_->warningThrottled(
          "single_invalid", std::string("single target rejected: ") + error.what());
    }
  }

  void waypointsCallback(const nav_msgs::Path::ConstPtr& message) {
    try {
      if (message->poses.empty() || message->poses.size()>10000) {
        throw std::invalid_argument("waypoint sequence is empty");
      }
      if (!messageFresh(message->header.stamp)) {
        throw std::invalid_argument("sequence timestamp is missing, stale or future");
      }
      const std::string frame = message->header.frame_id.empty()
                                    ? mission_frame_
                                    : message->header.frame_id;
      std::vector<Waypoint> waypoints;
      waypoints.reserve(message->poses.size());
      for (const auto& pose : message->poses) {
        waypoints.push_back(
            waypointFromPose(pose, frame, message->header.stamp));
      }
      auto pending=std::make_shared<const MissionInput>(MissionInput{std::move(waypoints),true,message->header.stamp});
      std::atomic_store(&pending_mission_,pending);
    } catch (const std::exception& error) {
      logger_->warningThrottled(
          "sequence_invalid",
          std::string("waypoint sequence rejected: ") + error.what());
    }
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& message) {
    const double now = steadyNow();
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (sensors_.state_time.accept(message->header.stamp, now)) {
      sensors_.state = *message;
    }
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    try {
      const auto& p = message->pose.position;
      const auto& q = message->pose.orientation;
      if (message->header.frame_id!="odom" || !finite(p.x) || !finite(p.y) || !finite(p.z)) {
        throw std::invalid_argument("non-finite local position");
      }
      yawFromQuaternion(q.x, q.y, q.z, q.w);
      const double now = steadyNow();
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (sensors_.pose_time.accept(message->header.stamp, now)) {
        sensors_.pose = *message;
      }
    } catch (const std::exception& error) {
      {std::lock_guard<std::mutex> lock(sensor_mutex_);sensors_.pose_time.receipt_time=-1;}
      logger_->warningThrottled("pose_invalid",std::string("Invalid local pose: ")+error.what());
    }
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& message) {
    const auto& linear = message->twist.linear;
    if (!finite(linear.x) || !finite(linear.y) || !finite(linear.z)) {
      {std::lock_guard<std::mutex> lock(sensor_mutex_);sensors_.velocity_time.receipt_time=-1;}
      logger_->warningThrottled("velocity_invalid","Invalid local velocity");
      return;
    }
    const double now = steadyNow();
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (sensors_.velocity_time.accept(message->header.stamp, now)) {
      sensors_.velocity = *message;
    }
  }

  void extendedCallback(const mavros_msgs::ExtendedState::ConstPtr& message) {
    const double now = steadyNow();
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (sensors_.extended_time.accept(message->header.stamp, now)) {
      sensors_.extended = *message;
    }
  }

  SensorState sensorSnapshot(double& now, ros::Time& ros_now) const {
    CachedSensors cache;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      cache = sensors_;
      now = steadyNow();
      ros_now = ros::Time::now();
    }
    SensorState output;
    output.ros_time = ros_now.toSec();
    output.state_fresh = cache.state_time.fresh(
        now, ros_now, state_timeout_, future_tolerance_);
    output.pose_fresh = cache.pose_time.fresh(
        now, ros_now, pose_timeout_, future_tolerance_);
    output.velocity_fresh = cache.velocity_time.fresh(
        now, ros_now, velocity_timeout_, future_tolerance_);
    output.extended_fresh = cache.extended_time.fresh(
        now, ros_now, extended_timeout_, future_tolerance_);
    output.connected = output.state_fresh && cache.state.connected;
    output.mode = cache.state.mode;
    output.armed = cache.state.armed;
    output.on_ground = output.extended_fresh &&
        cache.extended.landed_state ==
            mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    const auto& p = cache.pose.pose.position;
    const auto& q = cache.pose.pose.orientation;
    output.pose = {p.x, p.y, p.z, 0.0};
    try {
      output.pose.yaw = yawFromQuaternion(q.x, q.y, q.z, q.w);
    } catch (const std::exception&) {
      output.pose_fresh = false;
    }
    const auto& velocity = cache.velocity.twist.linear;
    output.speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                             velocity.z * velocity.z);
    return output;
  }

  std::uint8_t landedState() const {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return sensors_.extended.landed_state;
  }

  void serviceResultCallback(const ServiceResult& result) {
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      service_results_.push_back(result);
    }
    logger_->event(result.accepted ? "INFO" : "WARN",
                   "[Executor] service result " + actionName(result.kind) + " id=" +
                       std::to_string(result.id) + " transport=" +
                       (result.transport_ok ? "true" : "false") +
                       " accepted=" + (result.accepted ? "true" : "false"));
  }

  void drainServiceResults(double now) {
    std::deque<ServiceResult> results;
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      results.swap(service_results_);
    }
    for (const auto& result : results) {
      executor_->acceptActionResult(result.id, result.transport_ok,
                                      result.accepted, now);
    }
  }

  void publishSetpoint(const Pose4d& pose, const ros::Time& stamp) {
    geometry_msgs::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    message.pose = toRosPose(pose);
    setpoint_publisher_.publish(message);
  }

  void publishStatus(const ControllerOutput& output, const SensorState& sensors,
                     int command, const ros::Time& stamp) {
    ducted_offboard::OffboardStatus message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    message.command = command;
    message.state = static_cast<std::uint8_t>(output.actual_state);
    message.phase = phaseName(output.phase);
    message.reason = output.reason;
    message.current_waypoint = output.waypoint_count == 0
                                   ? 0
                                   : static_cast<std::uint32_t>(
                                         output.waypoint_index + 1);
    message.waypoint_count =
        static_cast<std::uint32_t>(output.waypoint_count);
    message.fcu_mode = sensors.mode;
    message.connected = sensors.connected;
    message.armed = sensors.armed;
    message.landed_state = landedState();
    message.input_ready = sensors.connected && sensors.state_fresh &&
                          sensors.pose_fresh && sensors.velocity_fresh &&
                          sensors.extended_fresh;
    message.setpoint_active = output.publish_setpoint;
    message.mission_loaded = output.waypoint_count > 0;
    message.target = toRosPose(output.final_target);
    message.log_directory = logger_->path();
    message.execution_mode = controller_config_.execution_mode;
    message.planner_request_id = output.planner_request_id;
    message.planner_state = output.planner_state;
    message.planner_ready = output.planner_ready;
    message.planner_target_age = output.planner_target_age;
    message.reference_valid = output.reference_valid;
    message.takeoff_reference_z = output.takeoff_reference_z;
    message.task_phase=output.task_phase;message.executor_phase=output.executor_phase;
    message.task_generation=output.task_generation;message.task_heartbeat_age=output.task_heartbeat_age;
    status_publisher_.publish(message);
  }

  void logStateChanges(const ControllerOutput& output) {
    const std::string phase = phaseName(output.phase);
    if (phase != last_phase_ || output.reason != last_reason_) {
      logger_->event(output.phase == Phase::ERROR ? "ERROR" : "INFO",
                     "[Executor] state " + phase + ": " + output.reason);
      last_phase_ = phase;
      last_reason_ = output.reason;
    }
  }

  void logFirstSetpoint(const ControllerOutput& output) {
    std::ostringstream key;
    key << phaseName(output.phase);
    if (!output.planner_request_id.empty())
      key << ":" << output.planner_request_id << (output.planner_target_age < 0 ? ":waiting" : ":tracking");
    if (output.phase != Phase::BOOT_WAIT_ZERO &&
        output.phase != Phase::IDLE_GROUND) {
      key << ':' << output.waypoint_index << ':' << std::fixed
          << std::setprecision(4) << output.final_target.x << ':'
          << output.final_target.y << ':' << output.final_target.z << ':'
          << output.final_target.yaw;
    }
    if (key.str() != last_target_key_) {
      std::ostringstream detail;
      detail << "[Executor] first setpoint for " << key.str() << " sent=(" << output.setpoint.x
             << "," << output.setpoint.y << "," << output.setpoint.z << ")";
      logger_->event("INFO", detail.str());
      last_target_key_ = key.str();
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher setpoint_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber extended_subscriber_;
  ros::Subscriber target_subscriber_;
  ros::Subscriber waypoints_subscriber_;

  mutable std::mutex sensor_mutex_;
  CachedSensors sensors_;
  std::mutex executor_mutex_;
  std::mutex result_mutex_;
  std::deque<ServiceResult> service_results_;
  std::unique_ptr<TaskMachine> task_;
  std::unique_ptr<FlightExecutor> executor_;
  std::shared_ptr<const TaskSnapshot> task_snapshot_;
  std::shared_ptr<const ExecutionFeedback> feedback_snapshot_;
  std::shared_ptr<const MissionInput> pending_mission_;
  std::atomic<bool> stopping_{false};
  std::thread task_thread_;
  std::unique_ptr<SessionLogger> logger_;
  std::unique_ptr<ServiceWorker> service_worker_;
  ControllerConfig controller_config_;

  std::string target_source_;
  std::string topic_mode_;
  std::string mission_frame_;
  std::string mission_file_;
  double control_rate_{20.0};
  double status_rate_{10.0};
  double telemetry_rate_{5.0};
  double state_timeout_{3.0};
  double pose_timeout_{0.5};
  double velocity_timeout_{0.5};
  double extended_timeout_{3.0};
  double future_tolerance_{0.05};
  double target_message_max_age_{2.0};
  std::string log_root_;
  std::uintmax_t log_max_bytes_{5242880};
  int log_keep_files_{3};
  std::string command_param_;
  std::string status_topic_;
  std::string target_topic_;
  std::string waypoints_topic_;
  std::string setpoint_topic_;
  std::string state_topic_;
  std::string pose_topic_;
  std::string velocity_topic_;
  std::string extended_state_topic_;
  std::string set_mode_service_;
  std::string arming_service_;

  int last_observed_command_{-999};
  std::string last_phase_;
  std::string last_reason_;
  std::string last_target_key_;
};

}  // namespace
}  // namespace ducted_offboard

int main(int argc, char** argv) {
  ros::init(argc, argv, "ducted_offboard_controller");
  try {
    ducted_offboard::OffboardNode node;
    return node.run();
  } catch (const std::exception& error) {
    std::cerr << "ducted_offboard startup failed: " << error.what() << std::endl;
    ROS_FATAL_STREAM("ducted_offboard startup failed: " << error.what());
    return 2;
  }
}
