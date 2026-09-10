#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

class PointCloudProcessor
{
public:
    PointCloudProcessor() : tf_listener_()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 获取高度阈值参数，默认1.0米
        private_nh.param<float>("height_threshold", height_threshold_, 1.0f);

        // 初始化订阅者和发布者
        cloud_sub_ = nh.subscribe("Laser_map", 1, &PointCloudProcessor::cloudCallback, this);
        filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("Laser_map_filter", 1);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // 转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // 坐标系转换
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (!transformCloud(msg->header.frame_id, "world", msg->header.stamp, cloud, transformed_cloud)) {
            ROS_WARN("Point cloud transform failed");
            return;
        }

        // 高度过滤
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        applyHeightFilter(transformed_cloud, filtered_cloud);

        // 发布过滤后的点云
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header.stamp = ros::Time::now();
        output_msg.header.frame_id = "world";
        filtered_pub_.publish(output_msg);
    }

    bool transformCloud(const std::string& source_frame,
                       const std::string& target_frame,
                       const ros::Time& stamp,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& output)
    {
        if (source_frame == target_frame) {
            *output = *input;
            return true;
        }

        try {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform(target_frame, source_frame, stamp, transform);
            
            // 手动构建变换矩阵
            Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
            
            // 提取旋转
            tf::Quaternion q = transform.getRotation();
            Eigen::Quaternionf eigen_quat(q.w(), q.x(), q.y(), q.z());
            transform_matrix.block<3,3>(0,0) = eigen_quat.toRotationMatrix();
            
            // 提取平移
            tf::Vector3 translation = transform.getOrigin();
            transform_matrix(0,3) = translation.x();
            transform_matrix(1,3) = translation.y();
            transform_matrix(2,3) = translation.z();
            
            pcl::transformPointCloud(*input, *output, transform_matrix);
            return true;
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("TF exception: %s", ex.what());
            return false;
        }
    }

    void applyHeightFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& output)
    {
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-std::numeric_limits<float>::max(), height_threshold_);
        pass.filter(*output);
    }

    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_pub_;
    tf::TransformListener tf_listener_;
    float height_threshold_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_processor");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}