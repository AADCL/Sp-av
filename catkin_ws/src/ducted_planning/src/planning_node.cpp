#include <ducted_planning/PlanPath.h>
#include <ducted_planning/environment.hpp>
#include <path_searching/kinodynamic_astar.h>
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <unordered_set>
#include <array>
#include <fstream>
#include <chrono>

namespace {
using V=Eigen::Vector3d;
using Grid=ducted_planning::Grid;
V position(const geometry_msgs::Point& p) { return {p.x,p.y,p.z}; }
bool quaternion(const geometry_msgs::Quaternion& q) {
  double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;return std::isfinite(n)&&std::abs(n-1)<.001;
}
struct KeyHash {size_t operator()(const std::array<int,3>& k)const {size_t h=0;for(int v:k)h^=std::hash<int>{}(v)+0x9e3779b9+(h<<6)+(h>>2);return h;}};
class PlanningNode {
  ros::NodeHandle nh_{"~"};ros::ServiceServer service_;
  ros::Publisher global_pub_,local_pub_;
  std::shared_ptr<Grid> static_,working_;
  fast_planner::EDTEnvironment::Ptr env_;
  fast_planner::KinodynamicAstar kino_;
  fast_planner::BsplineOptimizer optimizer_;
  std::unordered_set<std::array<int,3>,KeyHash> observed_free_;
  Eigen::Matrix4d previous_transform_=Eigen::Matrix4d::Zero();
  double map_resolution_=.2,max_speed_=.5,max_acceleration_=.5,vertical_speed_=.3;
  std::string map_error_;
  std::array<int,3> key(const V& p)const{return {{int(std::floor(p.x()/map_resolution_)),int(std::floor(p.y()/map_resolution_)),int(std::floor(p.z()/map_resolution_))}};}
public:
  PlanningNode() {
    std::vector<double> origin,size;
    nh_.param("origin",origin,std::vector<double>{-6,-6,-1.5});
    nh_.param("size",size,std::vector<double>{12,12,6});
    if(origin.size()!=3||size.size()!=3)throw std::invalid_argument("origin and size require XYZ triples");
    static_.reset(new Grid(V(origin[0],origin[1],origin[2]),V(size[0],size[1],size[2]),.2));
    working_.reset(new Grid(*static_));
    nh_.param("max_speed",max_speed_,.5);nh_.param("max_acceleration",max_acceleration_,.5);nh_.param("max_vertical_speed",vertical_speed_,.3);
    if(!std::isfinite(max_speed_+max_acceleration_+vertical_speed_)||max_speed_<=0||max_acceleration_<=0||vertical_speed_<=0)
      throw std::invalid_argument("invalid planner dynamic limits");
    env_.reset(new fast_planner::EDTEnvironment);env_->sdf_map_.reset(new SDFMap);env_->sdf_map_->grid=working_;
    kino_.setParam(nh_);kino_.setEnvironment(env_);kino_.init();optimizer_.setParam(nh_);optimizer_.setEnvironment(env_);
    loadMap();
    global_pub_=nh_.advertise<nav_msgs::Path>("global_path",1,true);local_pub_=nh_.advertise<nav_msgs::Path>("local_path",1);
    service_=nh_.advertiseService("plan",&PlanningNode::plan,this);
  }
  void loadMap() {
    std::string file;nh_.param<std::string>("occupancy_file",file,"");
    try {
      if(file.empty())throw std::runtime_error("occupancy_file is required; use a newly saved observed_occupancy.pcd");
      std::ifstream metadata(file.substr(0,file.find_last_of('/'))+"/mapping_metadata.yaml");
      std::string text((std::istreambuf_iterator<char>(metadata)),std::istreambuf_iterator<char>());
      if(text.find("frame_id: map")==std::string::npos||text.find("occupancy_resolution: 0.2")==std::string::npos||text.find("format_version: 1")==std::string::npos)
        throw std::runtime_error("missing or unsupported observed occupancy metadata");
      pcl::PointCloud<pcl::PointXYZI> cloud;
      if(pcl::io::loadPCDFile(file,cloud)!=0||cloud.empty()||cloud.size()>2000000)throw std::runtime_error("invalid observed occupancy file");
      std::unordered_set<std::array<int,3>,KeyHash> blocked;
      for(const auto& p:cloud) {
        V v(p.x,p.y,p.z);if(!v.allFinite()||v.cwiseAbs().maxCoeff()>10000||!std::isfinite(p.intensity))throw std::runtime_error("nonfinite occupancy cell");
        if(p.intensity<=-.6190392084)observed_free_.insert(key(v));else blocked.insert(key(v));
      }
      for(const auto& cell:blocked)observed_free_.erase(cell);
      if(observed_free_.empty())throw std::runtime_error("map contains no observed free cells");
      ROS_INFO_STREAM("Loaded "<<observed_free_.size()<<" observed free map cells");
    }catch(const std::exception& e){map_error_=e.what();ROS_ERROR_STREAM(map_error_);}
  }
  void transformStatic(const Eigen::Matrix4d& map_to_odom) {
    if((map_to_odom-previous_transform_).cwiseAbs().maxCoeff()<1e-9)return;
    Eigen::Matrix4d inverse=map_to_odom.inverse();
    // Conservatively require every source map cell overlapping each transformed
    // odom cell's bounding box to be observed free. Rotation cannot open gaps.
    for(size_t i=0;i<static_->free.size();++i) {
      V center=static_->center(i),low=V::Constant(INFINITY),high=V::Constant(-INFINITY);
      for(int bits=0;bits<8;++bits) {
        V corner=center;for(int a=0;a<3;++a)corner[a]+=((bits>>a)&1?.5:-.5)*static_->resolution;
        V p=(inverse*Eigen::Vector4d(corner.x(),corner.y(),corner.z(),1)).head<3>();low=low.cwiseMin(p);high=high.cwiseMax(p);
      }
      auto first=key(low+V::Constant(1e-7)),last=key(high-V::Constant(1e-7));bool free=true;
      for(int x=first[0];x<=last[0]&&free;++x)for(int y=first[1];y<=last[1]&&free;++y)for(int z=first[2];z<=last[2];++z)
        if(!observed_free_.count({{x,y,z}})){free=false;break;}
      static_->free[i]=free;
    }
    previous_transform_=map_to_odom;
  }
  nav_msgs::Path path(const vector<V>& points,const std_msgs::Header& header,const geometry_msgs::Quaternion& orientation) {
    nav_msgs::Path out;out.header=header;
    for(const auto& v:points){geometry_msgs::PoseStamped p;p.header=header;p.pose.position.x=v.x();p.pose.position.y=v.y();p.pose.position.z=v.z();p.pose.orientation=orientation;out.poses.push_back(p);}return out;
  }
  bool plan(ducted_planning::PlanPath::Request& req,ducted_planning::PlanPath::Response& res) {
    auto began=std::chrono::steady_clock::now();
    try {
      if(!map_error_.empty())throw std::runtime_error(map_error_);
      double age=(ros::Time::now()-req.header.stamp).toSec();
      V start=position(req.start.position),goal=position(req.goal.position),velocity(req.velocity.x,req.velocity.y,req.velocity.z);
      if(req.header.frame_id!="odom"||req.obstacles.header.frame_id!="odom"||req.obstacles.header.stamp!=req.header.stamp||
         !std::isfinite(age)||age<-.05||age>.4||!start.allFinite()||!goal.allFinite()||!velocity.allFinite()||
         !quaternion(req.start.orientation)||!quaternion(req.goal.orientation)||!quaternion(req.map_to_odom.rotation)||
         !std::isfinite(req.clearance+req.minimum_z+req.maximum_z)||req.clearance<.1||req.clearance>3||req.minimum_z>=req.maximum_z)
        throw std::runtime_error("invalid or stale planning snapshot");
      auto& tr=req.map_to_odom;Eigen::Quaterniond q(tr.rotation.w,tr.rotation.x,tr.rotation.y,tr.rotation.z);
      Eigen::Matrix4d transform=Eigen::Matrix4d::Identity();transform.block<3,3>(0,0)=q.toRotationMatrix();transform.block<3,1>(0,3)=V(tr.translation.x,tr.translation.y,tr.translation.z);
      if(!transform.allFinite()||transform.block<3,1>(0,3).cwiseAbs().maxCoeff()>10000)throw std::runtime_error("invalid map transform");
      transformStatic(transform);working_->free=static_->free;
      pcl::PointCloud<pcl::PointXYZ> obstacles;pcl::fromROSMsg(req.obstacles,obstacles);
      if(obstacles.empty()||obstacles.size()>200000)throw std::runtime_error("empty or oversized live cloud");
      for(const auto& p:obstacles){V v(p.x,p.y,p.z);if(!v.allFinite())throw std::runtime_error("nonfinite live obstacle");working_->setFree(working_->index(v),false);}
      working_->computeDistances();env_->sdf_map_->clearance=req.clearance;
      env_->sdf_map_->minimum_z=req.minimum_z;env_->sdf_map_->maximum_z=req.maximum_z;
      std::string error;auto global=working_->route(start,goal,req.clearance,.1,error);
      if(global.empty())throw std::runtime_error(error);
      res.global_path=path(global,req.header,req.goal.orientation);global_pub_.publish(res.global_path);
      V local_goal=global[1];double dist=(local_goal-start).norm();if(dist>1.0)local_goal=start+(local_goal-start)/dist;
      if(local_goal.z()<req.minimum_z||local_goal.z()>req.maximum_z||start.z()<req.minimum_z||start.z()>req.maximum_z)
        throw std::runtime_error("local route violates current measured AGL envelope");
      vector<V> local;
      if((local_goal-start).norm()<.12) local={start,local_goal};
      else {
        kino_.reset();int status=kino_.search(start,velocity,V::Zero(),local_goal,V::Zero(),false);
        if(status==fast_planner::KinodynamicAstar::NO_PATH)throw std::runtime_error("Fast-Planner kinodynamic search failed");
        vector<V> samples,derivatives;double interval=.2;kino_.getSamples(interval,samples,derivatives);
        if(!std::isfinite(interval)||interval<=0||samples.size()<4)throw std::runtime_error("invalid kinodynamic samples");
        Eigen::MatrixXd controls;fast_planner::NonUniformBspline::parameterizeToBspline(interval,samples,derivatives,controls);
        if(!controls.allFinite())throw std::runtime_error("nonfinite B-spline controls");
        auto optimized=optimizer_.BsplineOptimizeTraj(controls,interval,fast_planner::BsplineOptimizer::NORMAL_PHASE,0,0);
        fast_planner::NonUniformBspline spline(optimized,3,interval);
        // Uniform knot scaling preserves the collision-checked curve. Bounds
        // on derivative control-point norms bound the entire continuous spline.
        auto initial_velocity=spline.getDerivative();auto initial_acceleration=initial_velocity.getDerivative();
        auto vc=initial_velocity.getControlPoint(),ac=initial_acceleration.getControlPoint();
        if(!vc.allFinite()||!ac.allFinite())throw std::runtime_error("invalid spline derivative bounds");
        double scale=1;
        for(int row=0;row<vc.rows();++row){scale=std::max(scale,vc.row(row).norm()/max_speed_);scale=std::max(scale,std::abs(vc(row,2))/vertical_speed_);}
        for(int row=0;row<ac.rows();++row)scale=std::max(scale,std::sqrt(ac.row(row).norm()/max_acceleration_));
        auto knots=spline.getKnot();double first_knot=knots[0];
        knots=((knots.array()-first_knot)*(scale*1.01)+first_knot).matrix();spline.setKnot(knots);
        double duration=spline.getTimeSum();
        if(!std::isfinite(duration)||duration<=0||duration>30)throw std::runtime_error("invalid B-spline duration");
        auto vel=spline.getDerivative(),acc=vel.getDerivative();
        for(double t=0;t<duration+.02;t+=.02) {
          double at=std::min(t,duration);V p=spline.evaluateDeBoorT(at),v=vel.evaluateDeBoorT(at),a=acc.evaluateDeBoorT(at);
          if(!p.allFinite()||!v.allFinite()||!a.allFinite()||v.norm()>max_speed_+1e-5||std::abs(v.z())>vertical_speed_+1e-5||a.norm()>max_acceleration_+1e-5||
             p.z()<req.minimum_z||p.z()>req.maximum_z||working_->distance(p)<req.clearance)
            throw std::runtime_error("optimized trajectory violates collision, AGL or dynamic envelope");
          local.push_back(p);
        }
      }
      V previous=start;
      for(const auto& p:local){if(!working_->segment(previous,p,req.clearance))throw std::runtime_error("B-spline swept segment blocked");previous=p;}
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>.35||(ros::Time::now()-req.header.stamp).toSec()>.45)
        throw std::runtime_error("planning result exceeded snapshot deadline");
      V tracking=start;
      // The downstream PX4 interface tracks bounded position commands. Certify
      // the straight reference chord as well as the full optimized curve.
      for(const auto& p:local) {
        if((p-start).norm()>.3)break;
        if(!working_->segment(start,p,req.clearance))break;
        tracking=p;
      }
      if((tracking-start).norm()<1e-6 && (goal-start).norm()>.12)throw std::runtime_error("no advancing certified tracking reference");
      res.tracking_target.x=tracking.x();res.tracking_target.y=tracking.y();res.tracking_target.z=tracking.z();
      res.local_path=path(local,req.header,req.goal.orientation);local_pub_.publish(res.local_path);
      res.success=true;res.reason="observed-space 3D A* + Fast-Planner kinodynamic B-spline";
    }catch(const std::exception& e){res.success=false;res.reason=e.what();}
    return true;
  }
};
}
int main(int argc,char** argv){ros::init(argc,argv,"global_local_planner");try{PlanningNode node;ros::spin();}catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 2;}return 0;}
