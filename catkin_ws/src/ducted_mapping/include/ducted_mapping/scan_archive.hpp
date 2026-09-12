#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ducted_mapping/static_map_filter.hpp>
#include <string>
#include <vector>

namespace ducted_mapping {
struct ArchiveConfig {
  ArchiveConfig() { filter.map_voxel_size=.05; }
  StaticMapFilterConfig filter;
  std::string spool_root="/home/nrc/catkin_ws/logs";
  size_t max_bytes=2147483648ULL, max_scans=36000;
  double min_range=.5, max_range=50, scan_voxel=.05, radius=.15;
  int min_neighbors=2;
  bool radius_filter=true;
  bool export_observed_occupancy=false;
  int replay_workers=2;
};
struct ReplayResult { bool success=false; size_t static_points=0, scans=0; std::string error; double elapsed_seconds=0; };
class ScanArchive {
public:
  explicit ScanArchive(const ArchiveConfig& config);
  void capture(const pcl::PointCloud<pcl::PointXYZI>& cloud, size_t anchor,
               const Eigen::Matrix4d& anchor_to_lidar, double stamp);
  ReplayResult replay(const std::vector<Eigen::Matrix4d>& optimized_anchors,
                      const std::string& output_directory, double map_resolution=0,
                      const std::function<void(size_t, size_t)>& progress={}) const;
  const std::string& error() const { return error_; }
  const std::string& directory() const { return directory_; }
private:
  struct Scan { size_t anchor; Eigen::Matrix4d relative; double stamp; std::string file; };
  ArchiveConfig config_; std::string directory_, error_;
  size_t bytes_=0; double last_stamp_=0; std::vector<Scan> scans_;
};
}
