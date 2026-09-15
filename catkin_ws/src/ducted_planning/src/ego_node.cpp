#include <iostream>
#include <ducted_planning/PlanLocal.h>
#include <bspline_opt/bspline_optimizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace {
using V=Eigen::Vector3d;
using Spline=ego_planner::UniformBspline;
V position(const geometry_msgs::Point& p){return {p.x,p.y,p.z};}
bool unit(const geometry_msgs::Quaternion& q){
  double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(n)&&std::abs(n-1)<.001;
}
bool cloud_layout(const sensor_msgs::PointCloud2& c){
  uint64_t count=uint64_t(c.width)*c.height;
  if(!count||count>200000||c.is_bigendian||c.point_step<12||c.point_step>128||
     uint64_t(c.row_step)<uint64_t(c.width)*c.point_step||
     uint64_t(c.row_step)*c.height!=c.data.size())return false;
  for(const std::string name:{"x","y","z"}){
    bool found=false;
    for(const auto& field:c.fields)if(field.name==name){
      if(found||field.datatype!=sensor_msgs::PointField::FLOAT32||field.count!=1||uint64_t(field.offset)+4>c.point_step)return false;
      found=true;
    }
    if(!found)return false;
  }
  return true;
}
class EgoNode {
  ros::NodeHandle nh_{"~"}; ros::ServiceServer service_; ros::Publisher path_pub_,blocked_pub_;
  bool blocked_active_=false;
  GridMap::Ptr environment_{new GridMap};
  ego_planner::BsplineOptimizer optimizer_;
  double max_speed_,max_acceleration_,vertical_speed_,horizon_,snapshot_timeout_;
  std::vector<V> previous_curve_;
  V previous_goal_{V::Zero()};
  ros::Time previous_stamp_;
  std::string occupancyReason(const std::string& label, const V& query,
      const ducted_planning::PlanLocal::Request& req, const std::vector<V>& points) {
    pcl::PointCloud<pcl::PointXYZ> blockers;
    V nearest=V::Zero();double nearest_distance=std::numeric_limits<double>::infinity();
    for(const auto& p:points){
      const double distance=(p-query).norm();
      if(distance<nearest_distance){nearest=p;nearest_distance=distance;}
      if(environment_->sourceMayOccupy(p,query))
        blockers.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
    }
    sensor_msgs::PointCloud2 diagnostic;pcl::toROSMsg(blockers,diagnostic);diagnostic.header=req.header;
    blocked_pub_.publish(diagnostic);blocked_active_=true;
    std::ostringstream detail;detail<<std::fixed<<std::setprecision(3)
      <<label<<": blockers="<<blockers.size()
      <<" center=("<<query.x()<<","<<query.y()<<","<<query.z()<<")"
      <<" nearest=("<<nearest.x()<<","<<nearest.y()<<","<<nearest.z()<<")"
      <<" distance="<<nearest_distance<<" clearance="<<req.clearance
      <<" speed="<<V(req.velocity.x,req.velocity.y,req.velocity.z).norm()
      <<" voxel="<<environment_->getResolution();
    ROS_WARN_STREAM_THROTTLE(2.,detail.str());
    return detail.str();
  }
public:
  EgoNode(){
    nh_.param("max_speed",max_speed_,.5);nh_.param("max_acceleration",max_acceleration_,.5);
    nh_.param("max_vertical_speed",vertical_speed_,.3);nh_.param("planning_horizon",horizon_,2.8);
    nh_.param("snapshot_timeout",snapshot_timeout_,.5);
    for(double x:{max_speed_,max_acceleration_,vertical_speed_,horizon_})
      if(!std::isfinite(x)||x<=0)throw std::invalid_argument("invalid EGO dynamic limits");
    if(!std::isfinite(snapshot_timeout_)||snapshot_timeout_<=0||snapshot_timeout_>.5)
      throw std::invalid_argument("EGO snapshot timeout must be in (0, 0.5]");
    if(horizon_>4)throw std::invalid_argument("EGO local horizon must be <= 4 m");
    for(const std::string name:{"lambda_smooth","lambda_collision","lambda_feasibility","lambda_fitness","dist0"}){
      double value;nh_.param("optimization/"+name,value,-1.);
      if(!std::isfinite(value)||value<=0)throw std::invalid_argument("invalid EGO optimization/"+name);
    }
    int order;nh_.param("optimization/order",order,3);
    if(order!=3)throw std::invalid_argument("EGO integration requires cubic B-splines");
    nh_.setParam("optimization/max_vel",max_speed_);nh_.setParam("optimization/max_acc",max_acceleration_);
    optimizer_.setParam(nh_);optimizer_.setEnvironment(environment_);
    optimizer_.a_star_.reset(new AStar);
    optimizer_.a_star_->initGridMap(environment_,Eigen::Vector3i(90,90,50));
    path_pub_=nh_.advertise<nav_msgs::Path>("local_path",1);
    blocked_pub_=nh_.advertise<sensor_msgs::PointCloud2>("blocked_obstacles",1,true);
    service_=nh_.advertiseService("plan",&EgoNode::plan,this);
  }
  bool plan(ducted_planning::PlanLocal::Request& req,ducted_planning::PlanLocal::Response& res){
    auto began=std::chrono::steady_clock::now();
    try{
      V start=position(req.start.position),goal=position(req.goal.position);
      V velocity(req.velocity.x,req.velocity.y,req.velocity.z);
      double age=(ros::Time::now()-req.header.stamp).toSec();
      if(req.header.frame_id!="odom"||req.obstacles.header.frame_id!="odom"||
         req.obstacles.header.stamp!=req.header.stamp||req.header.stamp.isZero()||
         !std::isfinite(age)||age<-.05||age>snapshot_timeout_||!start.allFinite()||!goal.allFinite()||
         start.cwiseAbs().maxCoeff()>1e5||goal.cwiseAbs().maxCoeff()>1e5||
         !velocity.allFinite()||velocity.norm()>max_speed_+.15||
         !unit(req.start.orientation)||!unit(req.goal.orientation)||
         !std::isfinite(req.clearance+req.minimum_z+req.maximum_z+req.sensor_horizon)||
         req.clearance<.1||req.clearance>2||req.minimum_z>=req.maximum_z||
         req.sensor_horizon<1||req.sensor_horizon>5)
      {
        std::ostringstream detail;
        detail << "invalid or stale EGO snapshot: age=" << age
               << " speed=" << velocity.norm() << " clearance=" << req.clearance
               << " z_limits=" << req.minimum_z << "," << req.maximum_z;
        throw std::runtime_error(detail.str());
      }
      if(start.z()<req.minimum_z||start.z()>req.maximum_z||goal.z()<req.minimum_z||goal.z()>req.maximum_z)
        throw std::runtime_error("absolute target violates active height envelope");
      pcl::PointCloud<pcl::PointXYZ> cloud;
      if(!cloud_layout(req.obstacles))throw std::runtime_error("invalid live cloud layout or size");
      pcl::fromROSMsg(req.obstacles,cloud);
      if(cloud.empty()||cloud.size()>200000)throw std::runtime_error("empty or oversized live cloud");
      std::vector<V> points;points.reserve(cloud.size());
      for(const auto& p:cloud){V v(p.x,p.y,p.z);if(!v.allFinite()||v.cwiseAbs().maxCoeff()>1e5)
        throw std::runtime_error("invalid live obstacle");points.push_back(v);}
      environment_->reset(start,req.sensor_horizon,req.minimum_z,req.maximum_z,points,req.clearance);
      if(environment_->getInflateOccupancy(start))
        throw std::runtime_error(occupancyReason("current body intersects local occupancy",start,req,points));
      double distance=(goal-start).norm();
      double reach=std::min(horizon_,req.sensor_horizon-req.clearance-.2);
      if(reach<=.1)throw std::runtime_error("no usable local sensor horizon");
      V target=distance>reach ? V(start+(goal-start)*(reach/distance)) : goal;
      if(environment_->getInflateOccupancy(target))
        throw std::runtime_error(occupancyReason("local endpoint is occupied",target,req,points));
      if(blocked_active_){
        sensor_msgs::PointCloud2 empty;pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>(),empty);
        empty.header=req.header;blocked_pub_.publish(empty);blocked_active_=false;
      }
      std::vector<V> local;
      // Near the endpoint a tiny B-spline fit is ill-conditioned. Certify the
      // terminal chord explicitly; the downstream guard still enforces braking,
      // acceleration and the final swept volume. This is not a failure fallback.
      if(distance<.20 && velocity.norm()<.20){
        if(!environment_->segment(start,target))throw std::runtime_error("terminal segment blocked");
        local={start,target};
      }else{
        // Quintic boundary seed only: no global route, no ESDF, no kinodynamic
        // search. Upstream EGO discovers collision gradients and rebounds it.
        double length=(target-start).norm();
        double total=std::max(1.,2*length/max_speed_);
        int count=std::max(12,int(std::ceil(length/.12)));
        double dt=total/count;
        V delta=target-start, vT=velocity*total;
        std::vector<V> samples;
        for(int i=0;i<=count;++i){double u=double(i)/count;
          samples.push_back(start+vT*u+(10*delta-6*vT)*std::pow(u,3)
            +(-15*delta+8*vT)*std::pow(u,4)+(6*delta-3*vT)*std::pow(u,5));}
        // Continue the last locally certified bend when its endpoint and
        // source age still match. It is only an optimization seed: fresh
        // occupancy, rebound, derivative and full-curve checks remain required.
        if(previous_curve_.size()>2 && (previous_goal_-target).norm()<1e-4 &&
           req.header.stamp>previous_stamp_ && (req.header.stamp-previous_stamp_).toSec()<.5){
          size_t nearest=0;double best=1e9;
          for(size_t i=0;i<previous_curve_.size();++i){
            double d=(previous_curve_[i]-start).squaredNorm();
            if(d<best){best=d;nearest=i;}
          }
          if(best<.04 && nearest+2<previous_curve_.size()){
            std::vector<V> seed{start};
            seed.insert(seed.end(),previous_curve_.begin()+nearest+1,previous_curve_.end());
            std::vector<double> arc{0.};
            for(size_t i=1;i<seed.size();++i)arc.push_back(arc.back()+(seed[i]-seed[i-1]).norm());
            if(arc.back()>.05){
              samples.clear();size_t segment=1;
              for(int i=0;i<=count;++i){
                double d=arc.back()*i/count;
                while(segment+1<arc.size() && arc[segment]<d)++segment;
                double width=arc[segment]-arc[segment-1];
                double blend=width>1e-9?(d-arc[segment-1])/width:0.;
                samples.push_back(seed[segment-1]+blend*(seed[segment]-seed[segment-1]));
              }
            }
          }
        }
        Eigen::MatrixXd controls;
        Spline::parameterizeToBspline(dt,samples,{velocity,V::Zero(),V::Zero(),V::Zero()},controls);
        // Enforce exact cubic endpoint position and start velocity; the
        // least-squares parameterization alone can displace the initial pose.
        controls.col(0)=start-velocity*dt;controls.col(1)=start;controls.col(2)=start+velocity*dt;
        controls.rightCols(3).colwise()=target;
        if(!controls.allFinite())throw std::runtime_error("invalid EGO seed");
        optimizer_.initControlPoints(controls,true);
        if(environment_->expired()||!optimizer_.BsplineOptimizeTrajRebound(controls,dt))
          throw std::runtime_error("EGO rebound failed or exceeded deadline");
        if(!controls.allFinite())throw std::runtime_error("nonfinite EGO control points");
        Spline spline(controls,3,dt);
        auto vel=spline.getDerivative(), acc=vel.getDerivative();
        auto vc=vel.getControlPoint(),ac=acc.getControlPoint();
        double scale=1.;
        for(int i=0;i<vc.cols();++i){scale=std::max(scale,vc.col(i).norm()/max_speed_);
          scale=std::max(scale,std::abs(vc(2,i))/vertical_speed_);}
        for(int i=0;i<ac.cols();++i)scale=std::max(scale,std::sqrt(ac.col(i).norm()/max_acceleration_));
        if(!std::isfinite(scale))throw std::runtime_error("invalid spline derivative bound");
        auto knots=spline.getKnot();knots*=scale*1.01;spline.setKnot(knots);
        vel=spline.getDerivative();acc=vel.getDerivative();vc=vel.getControlPoint();ac=acc.getControlPoint();
        for(int i=0;i<vc.cols();++i)res.speed_bound=std::max(res.speed_bound,vc.col(i).norm());
        for(int i=0;i<ac.cols();++i)res.acceleration_bound=std::max(res.acceleration_bound,ac.col(i).norm());
        res.duration=spline.getTimeSum();
        if(!std::isfinite(res.duration)||res.duration<=0||res.duration>60)
          throw std::runtime_error("invalid EGO trajectory duration");
        // Audit ALL of the optimized curve. Upstream checks only its near 2/3.
        int steps=int(std::ceil(res.duration/.02));
        V previous=start;
        for(int i=0;i<=steps;++i){
          V p=spline.evaluateDeBoorT(res.duration*i/steps);
          if(!p.allFinite()||!environment_->segment(previous,p))
            throw std::runtime_error("EGO curve failed full collision/height/horizon validation");
          local.push_back(p);previous=p;
        }
        if((local.front()-start).norm()>1e-6||(local.back()-target).norm()>1e-6)
          throw std::runtime_error("EGO boundary pose mismatch");
      }
      V tracking=start;
      for(const auto& p:local){
        if((p-start).norm()>.3||!environment_->segment(start,p))break;
        tracking=p;
      }
      if((tracking-start).norm()<1e-6&&distance>.04)throw std::runtime_error("no certified advancing command chord");
      if(environment_->expired()||std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>.27||
         (ros::Time::now()-req.header.stamp).toSec()>snapshot_timeout_)
        throw std::runtime_error("EGO result exceeded snapshot deadline");
      res.local_path.header=req.header;
      for(const auto& v:local){geometry_msgs::PoseStamped p;p.header=req.header;p.pose.orientation=req.goal.orientation;
        p.pose.position.x=v.x();p.pose.position.y=v.y();p.pose.position.z=v.z();res.local_path.poses.push_back(p);}
      res.tracking_target.x=tracking.x();res.tracking_target.y=tracking.y();res.tracking_target.z=tracking.z();
      previous_curve_=local;previous_goal_=target;previous_stamp_=req.header.stamp;
      res.success=true;res.reason="EGO-Planner rebound B-spline, live local occupancy";
      path_pub_.publish(res.local_path);
    }catch(const std::exception& e){previous_curve_.clear();res.success=false;res.reason=e.what();res.local_path.poses.clear();}
    return true;
  }
};
}
int main(int argc,char** argv){
  std::cerr << "EGO 已冻结 (EGO is frozen); planner production entry is disabled." << std::endl;
  return 2;
ros::init(argc,argv,"ego_local_planner");
  try{EgoNode node;ros::spin();}catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 2;}return 0;}
