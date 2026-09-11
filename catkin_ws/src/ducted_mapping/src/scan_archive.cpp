#include <ducted_mapping/scan_archive.hpp>
#include <ducted_mapping/observed_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <unistd.h>

namespace ducted_mapping {
using Cloud=pcl::PointCloud<pcl::PointXYZI>;
ScanArchive::ScanArchive(const ArchiveConfig& c):config_(c) {
  StaticMapFilter validate(c.filter);
  if (!std::isfinite(c.min_range+c.max_range+c.scan_voxel+c.radius) || c.min_range<0 ||
      c.max_range<=c.min_range || c.scan_voxel<.001 || c.radius<=0 || c.min_neighbors<1 ||
      c.max_bytes<1024 || c.max_scans<1) throw std::invalid_argument("invalid scan archive settings");
  std::string pattern=c.spool_root+"/mapping-scans-XXXXXX";
  std::vector<char> path(pattern.begin(),pattern.end()); path.push_back(0);
  if (!mkdtemp(path.data())) throw std::runtime_error("cannot create mapping scan archive in "+c.spool_root);
  directory_=path.data();
}
void ScanArchive::capture(const Cloud& cloud,size_t anchor,const Eigen::Matrix4d& relative,double stamp) {
  if (!error_.empty()) return;
  try {
    if (!std::isfinite(stamp) || stamp<=last_stamp_ || !relative.allFinite())
      throw std::runtime_error("non-increasing scan stamp or invalid pose");
    if(scans_.size()>=config_.max_scans) throw std::runtime_error("scan archive count limit");
    Cloud::Ptr ranged(new Cloud), reduced(new Cloud);
    for(const auto& p:cloud) {
      double r=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
      if(std::isfinite(r) && r>=config_.min_range && r<=config_.max_range) ranged->push_back(p);
    }
    pcl::VoxelGrid<pcl::PointXYZI> voxel; voxel.setInputCloud(ranged);
    voxel.setLeafSize(config_.scan_voxel,config_.scan_voxel,config_.scan_voxel); voxel.filter(*reduced);
    size_t estimated=reduced->size()*sizeof(pcl::PointXYZI)+4096;
    if(bytes_+estimated>config_.max_bytes) throw std::runtime_error("scan archive byte limit");
    std::string file=directory_+"/"+std::to_string(scans_.size())+".pcd";
    if(pcl::io::savePCDFileBinary(file,*reduced)!=0) throw std::runtime_error("scan archive write failed");
    std::ofstream index(directory_+"/index.txt",std::ios::app);
    index<<std::setprecision(17)<<scans_.size()<<' '<<stamp<<' '<<anchor;
    for(int row=0;row<4;++row) for(int col=0;col<4;++col) index<<' '<<relative(row,col);
    index<<'\n'; index.close(); if(!index) throw std::runtime_error("archive index write failed");
    scans_.push_back({anchor,relative,stamp,file}); bytes_+=estimated; last_stamp_=stamp;
  } catch(const std::exception& e) { error_=e.what(); }
}
ReplayResult ScanArchive::replay(const std::vector<Eigen::Matrix4d>& anchors,const std::string& output,double map_resolution,
                               const std::function<void(size_t,size_t)>& progress) const {
  ReplayResult result;
  try {
    if(!error_.empty()) throw std::runtime_error(error_);
    if(scans_.empty()) throw std::runtime_error("no mapping scans recorded");
    auto filter_config=config_.filter;
    if(!std::isfinite(map_resolution)||map_resolution<0||map_resolution>1)throw std::runtime_error("invalid map save resolution");
    StaticMapFilter filter(filter_config); ObservedGrid grid;
    if(progress)progress(0,scans_.size());
    for(const auto& scan:scans_) {
      if(scan.anchor>=anchors.size() || !anchors[scan.anchor].allFinite()) throw std::runtime_error("missing optimized scan anchor");
      Cloud::Ptr raw(new Cloud), cleaned(new Cloud);
      if(pcl::io::loadPCDFile(scan.file,*raw)!=0) throw std::runtime_error("cannot reload mapping scan");
      Eigen::Matrix4d pose=anchors[scan.anchor]*scan.relative;
      Point3d sensor{pose(0,3),pose(1,3),pose(2,3)};
      auto transform=[&](const Cloud& cloud) {
        std::vector<Point3d> points; points.reserve(cloud.size());
        for(const auto& p:cloud) { Eigen::Vector4d q=pose*Eigen::Vector4d(p.x,p.y,p.z,1); points.push_back({q.x(),q.y(),q.z()}); }
        return points;
      };
      grid.scan(transform(*raw),sensor,config_.filter.max_clearing_range);
      if(config_.radius_filter && !raw->empty()) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius; radius.setInputCloud(raw);
        radius.setRadiusSearch(config_.radius); radius.setMinNeighborsInRadius(config_.min_neighbors); radius.filter(*cleaned);
      } else *cleaned=*raw;
      filter.updateScan(transform(*cleaned),sensor,scan.stamp);
      if(filter.capacityExceeded()) throw std::runtime_error("static map filter capacity exceeded");
      ++result.scans;
      if(progress)progress(result.scans,scans_.size());
    }
    Cloud static_cloud, occupancy;
    for(const auto& p:filter.points()) { pcl::PointXYZI q; q.x=p.x;q.y=p.y;q.z=p.z;q.intensity=0;static_cloud.push_back(q); }
    if(map_resolution>0 && !static_cloud.empty()) {
      // Temporal membership is decided at the original fine resolution first.
      // Coarser requested output cells must not merge different temporal states.
      Cloud::Ptr confirmed(new Cloud(static_cloud));pcl::VoxelGrid<pcl::PointXYZI> downsample;
      downsample.setInputCloud(confirmed);downsample.setLeafSize(map_resolution,map_resolution,map_resolution);
      downsample.filter(static_cloud);
    }
    if(static_cloud.empty()) throw std::runtime_error("no confirmed static points; observe surfaces for at least 2 seconds");
    for(const auto& cell:grid.cells()) { auto p=grid.center(cell.first); pcl::PointXYZI q;q.x=p.x;q.y=p.y;q.z=p.z;q.intensity=cell.second;occupancy.push_back(q); }
    for(const std::string name:{"GlobalMap.pcd","SurfMap.pcd","filterGlobalMap.pcd"})
      if(pcl::io::savePCDFileBinary(output+"/"+name,static_cloud)!=0) throw std::runtime_error("static map write failed");
    if(pcl::io::savePCDFileBinary(output+"/observed_occupancy.pcd",occupancy)!=0) throw std::runtime_error("occupancy write failed");
    std::ofstream meta(output+"/mapping_metadata.yaml");
    meta<<"format_version: 1\nframe_id: map\nfilter: ag_test_bayesian_temporal\nfilter_source_commit: 9a309a93fd900abccb8775ebdb727c65946f5c0d\n"
        <<"occupancy_resolution: "<<grid.resolution()<<"\nfree_log_odds_max: -0.6190392084\nunknown_policy: blocked\n"
        <<"static_map_resolution: "<<(map_resolution>0?map_resolution:filter_config.map_voxel_size)<<"\nmin_hit_scans: "<<filter_config.min_hit_scans<<"\nmin_observation_span: "<<filter_config.min_observation_span<<"\n"
        <<"loop_corrected_replay: true\nscan_count: "<<result.scans<<"\nstatic_point_count: "<<static_cloud.size()<<"\n";
    meta.close(); if(!meta) throw std::runtime_error("mapping metadata write failed");
    result.static_points=static_cloud.size(); result.success=true;
  } catch(const std::exception& e) { result.error=e.what(); }
  return result;
}
}
