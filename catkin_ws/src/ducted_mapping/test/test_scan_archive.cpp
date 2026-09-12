#include <ducted_mapping/scan_archive.hpp>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <iterator>
using namespace ducted_mapping;
int main() {
  ArchiveConfig c; c.radius_filter=false; c.filter.min_hit_scans=2; c.filter.min_observation_span=0;
  c.spool_root="/tmp"; ScanArchive archive(c);
  pcl::PointCloud<pcl::PointXYZI> cloud; pcl::PointXYZI p; p.x=1.01; p.y=.01; p.z=.01; p.intensity=5; cloud.push_back(p);
  auto identity=Eigen::Matrix4d::Identity().eval();
  archive.capture(cloud,0,identity,1); archive.capture(cloud,0,identity,2);
  std::vector<Eigen::Matrix4d> poses{identity}; poses[0](0,3)=3;
  char path[]="/tmp/ducted-export-test-XXXXXX"; if (!mkdtemp(path)) return 1;
  size_t updates=0, last_done=0;
  auto result=archive.replay(poses,path,0,[&](size_t done,size_t total) {
    if(total!=2 || done<last_done || done>total) throw std::runtime_error("invalid replay progress");
    last_done=done; ++updates;
  });
  if(result.scans!=2 || updates<2 || last_done!=2)return 12;
  if (!result.success || result.static_points!=1) return 2;
  pcl::PointCloud<pcl::PointXYZI> saved;
  if (pcl::io::loadPCDFile(std::string(path)+"/GlobalMap.pcd",saved)!=0 || saved.size()!=1) return 3;
  if (std::abs(saved[0].x-4.01)>.001) return 4; // corrected keyframe applied at export
  if (archive.replay({},path).success) return 5; // missing optimized anchor never falls back
  ArchiveConfig limited=c; limited.max_bytes=2048; ScanArchive quota(limited);
  quota.capture(cloud,0,identity,1);
  if(quota.error().empty() || quota.replay(poses,path).success) return 6;
  ScanArchive clock(c); clock.capture(cloud,0,identity,2); clock.capture(cloud,0,identity,1);
  if(clock.error().empty() || clock.replay(poses,path).success) return 7;
  ScanArchive coarse(c);coarse.capture(cloud,0,identity,1);
  p.x=1.21;cloud.push_back(p);coarse.capture(cloud,0,identity,2);
  char coarser[]="/tmp/ducted-coarse-test-XXXXXX";if(!mkdtemp(coarser))return 8;
  auto coarse_result=coarse.replay(poses,coarser,.5);
  if(!coarse_result.success||coarse_result.static_points!=1){std::cerr<<coarse_result.error<<" points="<<coarse_result.static_points<<std::endl;return 9;}
  pcl::io::loadPCDFile(std::string(coarser)+"/GlobalMap.pcd",saved);
  if(std::abs(saved[0].x-4.01)>.001)return 10; // transient must not contaminate coarse static voxel
  if(coarse.replay(poses,coarser,NAN).success)return 11;
  if(access((std::string(path)+"/observed_occupancy.pcd").c_str(),F_OK)==0) {
    std::cerr<<"default export should omit the unused observed grid"<<std::endl;return 13;
  }
  // Two stable details in one 10 cm cell must survive the default dense export.
  ScanArchive dense(c);cloud.clear();p.x=1.02;cloud.push_back(p);p.x=1.08;cloud.push_back(p);
  dense.capture(cloud,0,identity,1);dense.capture(cloud,0,identity,2);
  char dense_path[]="/tmp/ducted-dense-test-XXXXXX";if(!mkdtemp(dense_path))return 14;
  auto dense_result=dense.replay(poses,dense_path,0);
  if(!dense_result.success || dense_result.static_points!=2) {
    std::cerr<<"5 cm export lost stable surface detail"<<std::endl;return 15;
  }
  // Worker count and optional observed grid must not change the static cloud.
  ArchiveConfig with_grid=c;with_grid.export_observed_occupancy=true;with_grid.replay_workers=1;
  ScanArchive serial(with_grid);serial.capture(cloud,0,identity,1);serial.capture(cloud,0,identity,2);
  char serial_path[]="/tmp/ducted-serial-test-XXXXXX";if(!mkdtemp(serial_path))return 16;
  if(!serial.replay(poses,serial_path).success)return 17;
  auto bytes=[](const std::string& file) {std::ifstream f(file,std::ios::binary);return std::string(std::istreambuf_iterator<char>(f),{});};
  if(bytes(std::string(serial_path)+"/GlobalMap.pcd")!=bytes(std::string(dense_path)+"/GlobalMap.pcd"))return 18;
  if(access((std::string(serial_path)+"/observed_occupancy.pcd").c_str(),F_OK)!=0)return 19;
  if(bytes(std::string(dense_path)+"/mapping_metadata.yaml").find("occupancy_exported: false")==std::string::npos)return 20;
  // Asynchronous preparation errors must still fail the whole export.
  ScanArchive missing(c);missing.capture(cloud,0,identity,1);missing.capture(cloud,0,identity,2);
  unlink((missing.directory()+"/1.pcd").c_str());
  if(missing.replay(poses,serial_path).success)return 21;
  return 0;
}
