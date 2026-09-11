#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <string>
#include <limits>

namespace ducted_planning {
class Grid {
public:
  Grid(const Eigen::Vector3d& origin,const Eigen::Vector3d& size,double resolution);
  int address(const Eigen::Vector3i& k) const;
  Eigen::Vector3i index(const Eigen::Vector3d& p) const;
  Eigen::Vector3i index(int address) const;
  Eigen::Vector3d center(int address) const;
  bool inside(const Eigen::Vector3d& p) const;
  void setFree(const Eigen::Vector3i& k,bool free=true);
  void computeDistances();
  double distance(const Eigen::Vector3d& p) const;
  bool segment(const Eigen::Vector3d& from,const Eigen::Vector3d& to,double clearance) const;
  std::vector<Eigen::Vector3d> route(const Eigen::Vector3d& from,const Eigen::Vector3d& to,
                                  double clearance,double timeout,std::string& error) const;
  Eigen::Vector3d origin,size;
  Eigen::Vector3i dims;
  double resolution;
  std::vector<unsigned char> free;
private:
  std::vector<double> distances_;
};
}
