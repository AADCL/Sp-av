#include <gtest/gtest.h>
#include "../src/preprocess.h"
#include <ros/serialization.h>
#include <fstream>
#include <iterator>
#include <iostream>

static livox_ros_driver2::CustomMsg::Ptr fixture() {
  livox_ros_driver2::CustomMsg::Ptr m(new livox_ros_driver2::CustomMsg);
  const float xyz[][3]={{0,0,0},{.1,0,0},{.1,.1,0},{.1,.1,.1},
                       {.4,0,0},{.4,.1,0},{.4,.1,.1}};
  for (const auto& p:xyz) {
    livox_ros_driver2::CustomPoint point;
    point.x=p[0];point.y=p[1];point.z=p[2];point.line=0;point.tag=0x10;
    point.offset_time=m->points.size()*100000;
    m->points.push_back(point);
  }
  m->point_num=m->points.size();return m;
}

static PointCloudXYZI::Ptr filtered(const livox_ros_driver2::CustomMsg::Ptr& m,int stride=1) {
  Preprocess p;p.set(false,MID360,.3,stride);p.N_SCANS=4;
  PointCloudXYZI::Ptr out(new PointCloudXYZI);p.process(m,out);return out;
}

TEST(LivoxBlind, RejectsNearPointsWhenAnyCoordinateChanges) {
  auto out=filtered(fixture());
  ASSERT_EQ(3u,out->size());
  for(const auto& p:*out) EXPECT_GT(p.x*p.x+p.y*p.y+p.z*p.z,.3*.3);
}

TEST(LivoxBlind, SubsamplingCannotBypassBlindRadius) {
  auto out=filtered(fixture(),2);
  ASSERT_EQ(2u,out->size());
  for(const auto& p:*out) EXPECT_GT(p.x*p.x+p.y*p.y+p.z*p.z,.3*.3);
}

TEST(LivoxBlind, BoundaryUsesThreeDimensionalDistance) {
  auto m=fixture();m->points.resize(1);
  for(const auto& xyz:std::vector<std::vector<float>>{{.299f,0,0},{0,.299f,0},{0,0,.299f},
                                                  {.301f,0,0},{0,.301f,0},{0,0,.301f}}) {
    livox_ros_driver2::CustomPoint p;p.x=xyz[0];p.y=xyz[1];p.z=xyz[2];p.line=0;p.tag=0x10;
    m->points.push_back(p);
  }
  m->point_num=m->points.size();
  auto out=filtered(m);ASSERT_EQ(3u,out->size());
  for(const auto& p:*out) EXPECT_GT(p.x*p.x+p.y*p.y+p.z*p.z,.3*.3);
}

// Offline replay of the actual serialized Livox messages; no ROS master,
// publishers, or vehicle service clients are created by this executable.
static int replay(const std::string& directory) {
  unsigned total_near=0;
  std::cout<<"[";
  for(int i=0;i<5;++i) {
    std::ifstream file(directory+"/raw_"+std::to_string(i)+".bin",std::ios::binary);
    if(!file) return 2;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
    ros::serialization::IStream stream(bytes.data(),bytes.size());
    livox_ros_driver2::CustomMsg::Ptr msg(new livox_ros_driver2::CustomMsg);
    ros::serialization::deserialize(stream,*msg);
    auto out=filtered(msg,2);unsigned near=0;double minimum=1e9;
    for(const auto& p:*out) {
      double r=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
      if(r<=.3)++near;minimum=std::min(minimum,r);
    }
    total_near+=near;
    if(i)std::cout<<",";
    std::cout<<"{\"frame\":"<<i<<",\"input\":"<<msg->point_num
             <<",\"output\":"<<out->size()<<",\"near\":"<<near<<",\"minimum_range\":"<<minimum<<"}";
  }
  std::cout<<"]\n";return total_near?1:0;
}

int main(int argc,char** argv) {
  if(argc==3 && std::string(argv[1])=="--replay")return replay(argv[2]);
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
