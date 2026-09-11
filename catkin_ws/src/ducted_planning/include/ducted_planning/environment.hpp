#pragma once
// Compatibility surface for Fast-Planner's search/optimizer. No ROS map
// callbacks: the service owns one immutable occupancy snapshot per computation.
#include <ducted_planning/grid.hpp>
#include <iostream>
using std::string; using std::min; using std::max; using std::vector; using std::shared_ptr; using std::unique_ptr;
using std::pair; using std::cout; using std::endl;
class SDFMap {
public:
  std::shared_ptr<ducted_planning::Grid> grid;
  double clearance=.6, minimum_z=-1, maximum_z=3;
  int getInflateOccupancy(const Eigen::Vector3d& p) const {
    return p.z()<minimum_z || p.z()>maximum_z || grid->distance(p)<clearance;
  }
  void getRegion(Eigen::Vector3d& origin,Eigen::Vector3d& size) const { origin=grid->origin;size=grid->size; }
};
namespace fast_planner {
class EDTEnvironment {
public:
  using Ptr=std::shared_ptr<EDTEnvironment>;
  std::shared_ptr<SDFMap> sdf_map_;
  void evaluateEDTWithGrad(const Eigen::Vector3d& p,double,double& distance,Eigen::Vector3d& gradient) {
    distance=sdf_map_->grid->distance(p);
    double h=sdf_map_->grid->resolution*.25;
    for(int a=0;a<3;++a) {auto plus=p,minus=p;plus[a]+=h;minus[a]-=h;gradient[a]=(sdf_map_->grid->distance(plus)-sdf_map_->grid->distance(minus))/(2*h);}
    if(sdf_map_->clearance+p.z()-sdf_map_->minimum_z<distance) {distance=sdf_map_->clearance+p.z()-sdf_map_->minimum_z;gradient=Eigen::Vector3d(0,0,1);}
    if(sdf_map_->clearance+sdf_map_->maximum_z-p.z()<distance) {distance=sdf_map_->clearance+sdf_map_->maximum_z-p.z();gradient=Eigen::Vector3d(0,0,-1);}
  }
};
}
