#pragma once
// Snapshot adapter for upstream EGO's occupancy-only environment API.
// No ESDF and no saved/global map. Points are already retained/predicted by
// the navigation guard; every request replaces the bounded local occupancy.
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

class GridMap {
  Eigen::Vector3d origin_, center_;
  Eigen::Vector3i dims_;
  std::vector<unsigned char> occupied_;
  std::vector<Eigen::Vector3d> source_points_;
  double resolution_=.1, radius_=0., min_z_=0., max_z_=0.;
  std::chrono::steady_clock::time_point deadline_;
  int address(const Eigen::Vector3i& k) const {
    if ((k.array()<0).any() || (k.array()>=dims_.array()).any()) return -1;
    return (k.x()*dims_.y()+k.y())*dims_.z()+k.z();
  }
  double clearance_=0.;
  double samplingReserve() const {return resolution_*.125;}
  double cellGapSquared(const Eigen::Vector3i& offset) const {
    // Minimum distance between the two CLOSED voxel boxes. Occupying the
    // entire source voxel covers every input point even when deduplicated.
    const Eigen::Vector3d gap =
        (offset.cwiseAbs().cast<double>().array()-1.).max(0.)*resolution_;
    return gap.squaredNorm();
  }
public:
  using Ptr=std::shared_ptr<GridMap>;
  double getResolution() const {return resolution_;}
  bool expired() const {return std::chrono::steady_clock::now()>deadline_;}
  void reset(const Eigen::Vector3d& center,double radius,double min_z,double max_z,
             const std::vector<Eigen::Vector3d>& points,double clearance) {
    deadline_=std::chrono::steady_clock::now()+std::chrono::milliseconds(240);
    center_=center; radius_=radius-clearance; min_z_=min_z; max_z_=max_z;
    origin_=center-Eigen::Vector3d::Constant(radius);
    dims_=Eigen::Vector3i::Constant(int(std::ceil(2*radius/resolution_)));
    if (dims_.x()<1 || dims_.x()>100) throw std::invalid_argument("invalid local map radius");
    occupied_.assign(size_t(dims_.x())*dims_.y()*dims_.z(),0);
    source_points_.clear();
    source_points_.reserve(points.size());
    clearance_=clearance;
    // Exact box-to-box distance replaces the isotropic two-half-diagonal
    // bound. Keep the body clearance and reserve half a segment sample step.
    // This still marks EVERY query voxel that could contain a colliding pose.
    double inflated=clearance+samplingReserve();
    int n=int(std::ceil(inflated/resolution_))+1;
    std::vector<Eigen::Vector3i> offsets;
    for(int x=-n;x<=n;++x) for(int y=-n;y<=n;++y) for(int z=-n;z<=n;++z)
      if(cellGapSquared(Eigen::Vector3i(x,y,z))<=inflated*inflated+1e-12)
        offsets.emplace_back(x,y,z);
    std::vector<unsigned char> inserted(occupied_.size(),0);
    for(const auto& p:points) {
      Eigen::Vector3i k=((p-origin_)/resolution_).array().floor().cast<int>();
      int a=address(k); if(a<0)continue;
      // Keep every observed/predicted point for the exact narrow phase, even
      // when its coarse source cell has already been inserted.
      source_points_.push_back(p);
      if(inserted[a])continue;
      inserted[a]=1;
      for(const auto& d:offsets){int b=address(k+d);if(b>=0)occupied_[b]=1;}
      if(expired())throw std::runtime_error("local occupancy construction deadline exceeded");
    }
  }
  bool sourceMayOccupy(const Eigen::Vector3d& source,const Eigen::Vector3d& query) const {
    const Eigen::Vector3i a=((source-origin_)/resolution_).array().floor().cast<int>();
    const Eigen::Vector3i b=((query-origin_)/resolution_).array().floor().cast<int>();
    const double inflated=clearance_+samplingReserve();
    return address(a)>=0 && address(b)>=0 && (source-query).squaredNorm()<=inflated*inflated+1e-12;
  }
  int getInflateOccupancy(const Eigen::Vector3d& p) const {
    if(!p.allFinite() || p.z()<min_z_ || p.z()>max_z_ || (p-center_).norm()>radius_)return 1;
    Eigen::Vector3i k=((p-origin_)/resolution_).array().floor().cast<int>();
    int a=address(k);
    if(a<0)return 1;
    if(!occupied_[a])return 0;
    // The coarse stencil is a conservative broad phase, not a physical
    // collision verdict. Compare actual coordinates before rejecting a pose.
    // Retain half a chord sample step to protect between-sample collisions.
    const double inflated=clearance_+samplingReserve();
    for(const auto& point:source_points_)
      if((point-p).squaredNorm()<=inflated*inflated+1e-12)return 1;
    return 0;
  }
  bool segment(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const {
    // The stencil reserves a sample half-step against a missed voxel corner;
    // the full chord, including both endpoints, is checked.
    int n=std::max(1,int(std::ceil((b-a).norm()/(resolution_*.25))));
    for(int i=0;i<=n;++i)if(getInflateOccupancy(a+(b-a)*(double(i)/n)))return false;
    return true;
  }
};
