#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::string world_frame_id;
tf::TransformListener*  tfListener;
std::string base_link_frame_id, odom_frame_id, lidar_frame_id, slam_topic, odometry_new_topic;
tf::StampedTransform trans_result;
bool init= false;
tf::TransformBroadcaster *tf_br;
ros::Publisher odometry_new_pub;
ros::Publisher cloud_world_pub;
double filter_size;

void callback(const nav_msgs::OdometryConstPtr slam_msg)
{
    init = true;

    // TF
    tf::Quaternion slam_quaternion;
    tf::quaternionMsgToTF(slam_msg->pose.pose.orientation, slam_quaternion);
    tf::Point slam_translate;
    tf::pointMsgToTF(slam_msg->pose.pose.position, slam_translate);
    tf::Transform slam_T;
    slam_T.setRotation(slam_quaternion);
    slam_T.setOrigin(slam_translate);

    // get odom to livox
    tf::StampedTransform odom2lidar_transform;
    try 
    {
        tfListener->waitForTransform(base_link_frame_id, lidar_frame_id, slam_msg->header.stamp, ros::Duration(0.5));
        tfListener->lookupTransform(base_link_frame_id, lidar_frame_id, slam_msg->header.stamp, odom2lidar_transform);
    } 
    catch(tf::TransformException &exception) {
        ROS_WARN_STREAM(exception.what());         
    }
    tf::Transform odom_T;
    odom_T.setRotation(odom2lidar_transform.getRotation());
    odom_T.setOrigin(odom2lidar_transform.getOrigin());
    tf::Transform fix_T;
    fix_T = slam_T * odom_T.inverse();

    trans_result = tf::StampedTransform(fix_T, slam_msg->header.stamp, odom_frame_id, base_link_frame_id);
    trans_result.stamp_ = slam_msg->header.stamp;
    tf_br->sendTransform(trans_result);

    nav_msgs::Odometry odometry_new;
    tf::StampedTransform world2base_link_transform;
    try 
    {
        tfListener->waitForTransform(world_frame_id, odom_frame_id, slam_msg->header.stamp, ros::Duration(0.5));
        tfListener->lookupTransform(world_frame_id, odom_frame_id, slam_msg->header.stamp, world2base_link_transform);
    } 
    catch(tf::TransformException &exception) {
        ROS_ERROR_STREAM(exception.what());         
    }
    tf::Transform world_T;
    world_T.setRotation(world2base_link_transform.getRotation());
    world_T.setOrigin(world2base_link_transform.getOrigin());
    tf::Transform world2base_T;
    world2base_T = world_T * fix_T;

    tf::Quaternion rotation = world2base_T.getRotation();
    tf::Vector3 translation = world2base_T.getOrigin();
    odometry_new.pose.pose.position.x = translation.x();
    odometry_new.pose.pose.position.y = translation.y();
    odometry_new.pose.pose.position.z = translation.z();
    odometry_new.pose.pose.orientation.x = rotation.x();
    odometry_new.pose.pose.orientation.y = rotation.y();
    odometry_new.pose.pose.orientation.z = rotation.z();
    odometry_new.pose.pose.orientation.w = rotation.w();

    // Transform pose covariance
    // Original covariance is in the frame of slam_msg->header.frame_id (odom) but related to child_frame_id (base_link)
    // We need to transform it to the world_frame_id
    tf::Matrix3x3 rot_matrix_tf = world2base_T.getBasis();
    Eigen::Matrix3d eigen_R;
    for (int i = 0; i < 3; ++i) {
        tf::Vector3 row = rot_matrix_tf.getRow(i);
        eigen_R(i, 0) = row.x();
        eigen_R(i, 1) = row.y();
        eigen_R(i, 2) = row.z();
    }

    Eigen::Matrix<double, 6, 6> cov_orig;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            cov_orig(i, j) = slam_msg->pose.covariance[i * 6 + j];
        }
    }

    Eigen::Matrix<double, 6, 6> cov_new;

    // Transform position covariance (top-left 3x3 block)
    cov_new.block<3,3>(0,0) = eigen_R * cov_orig.block<3,3>(0,0) * eigen_R.transpose();
    // Transform orientation covariance (bottom-right 3x3 block)
    cov_new.block<3,3>(3,3) = eigen_R * cov_orig.block<3,3>(3,3) * eigen_R.transpose();
    // Transform position-orientation cross-covariance (top-right 3x3 block)
    cov_new.block<3,3>(0,3) = eigen_R * cov_orig.block<3,3>(0,3) * eigen_R.transpose();
    // Transform orientation-position cross-covariance (bottom-left 3x3 block)
    cov_new.block<3,3>(3,0) = eigen_R * cov_orig.block<3,3>(3,0) * eigen_R.transpose();
    // Alternatively, if original covariance is symmetric, cov_new.block<3,3>(3,0) = cov_new.block<3,3>(0,3).transpose();
    // However, transforming independently is safer if symmetry isn't guaranteed post-multiplication.

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            odometry_new.pose.covariance[i * 6 + j] = cov_new(i, j);
        }
    }

    // Transform linear and angular velocities
    // The original velocities are in the frame slam_msg->child_frame_id (usually base_link)
    // We need to transform them to the world_frame_id
    tf::Vector3 linear_vel_orig_tf;
    // Convert geometry_msgs::Vector3 to tf::Vector3
    tf::vector3MsgToTF(slam_msg->twist.twist.linear, linear_vel_orig_tf);
    tf::Vector3 angular_vel_orig_tf;
    tf::vector3MsgToTF(slam_msg->twist.twist.angular, angular_vel_orig_tf);

    // world2base_T transforms points from base_link to world.
    // Its rotational part, world2base_T.getBasis(), rotates vectors from base_link to world.
    tf::Matrix3x3 rotation_base_to_world = world2base_T.getBasis();
    
    tf::Vector3 linear_vel_transformed = rotation_base_to_world * linear_vel_orig_tf;
    tf::Vector3 angular_vel_transformed = rotation_base_to_world * angular_vel_orig_tf;

    // Populate the new odometry message with transformed velocities
    odometry_new.twist.twist.linear.x = linear_vel_transformed.x();
    odometry_new.twist.twist.linear.y = linear_vel_transformed.y();
    odometry_new.twist.twist.linear.z = linear_vel_transformed.z();

    odometry_new.twist.twist.angular.x = angular_vel_transformed.x();
    odometry_new.twist.twist.angular.y = angular_vel_transformed.y();
    odometry_new.twist.twist.angular.z = angular_vel_transformed.z();
    
    // Copy the covariance matrix for twist
    // Note: For a more rigorous approach, the covariance matrix itself should be transformed.
    // This typically involves using Jacobian matrices of the transformation: C_new = J * C_orig * J^T.
    // For simplicity, we are directly copying it here. This assumes either that the
    // off-diagonal terms related to orientation-velocity coupling are small, or that
    // the user is aware of this simplification.
    odometry_new.twist.covariance = slam_msg->twist.covariance;
    
    odometry_new.child_frame_id = base_link_frame_id;
    odometry_new.header.stamp = slam_msg->header.stamp;
    odometry_new.header.frame_id = world_frame_id;
    odometry_new_pub.publish(odometry_new);
}

void callback_cloud(sensor_msgs::PointCloud2ConstPtr cloud_msg)
{
    if (!init) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    tf::StampedTransform world2lidar_transform;
    try 
    {
        // tfListener->waitForTransform(world_frame_id, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.5));
        tfListener->lookupTransform(world_frame_id, cloud_msg->header.frame_id, ros::Time(0), world2lidar_transform);
    } 
    catch(tf::TransformException &exception) {
        ROS_ERROR_STREAM("Error in transform: " << exception.what());
        return;         
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZ point = cloud->points[i];
        if(point.x*point.x + point.y* point.y + point.z*point.z < filter_size*filter_size)
        {
            continue;
        }
        tf::Vector3 pt(point.x, point.y, point.z);
        tf::Vector3 pt_world = world2lidar_transform * pt;

        pcl::PointXYZ pt_world_pcl;
        pt_world_pcl.x = pt_world.x();
        pt_world_pcl.y = pt_world.y();
        pt_world_pcl.z = pt_world.z();
        cloud_world->points.push_back(pt_world_pcl);
    }

    sensor_msgs::PointCloud2 cloud_world_msg;
    pcl::toROSMsg(*cloud_world, cloud_world_msg);
    cloud_world_msg.header.stamp = cloud_msg->header.stamp;
    cloud_world_msg.header.frame_id = world_frame_id;
    cloud_world_pub.publish(cloud_world_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fix_odom");
    ros::NodeHandle nh, private_nh("~");

    tfListener = new tf::TransformListener;
    tf_br = new tf::TransformBroadcaster;
    
    std::string origin_lidar_topic,cloud_world_topic;

    private_nh.param<std::string>("base_link_frame_id", base_link_frame_id, "base_link");
    private_nh.param<std::string>("lidar_frame_id",lidar_frame_id, "rslidar");
    private_nh.param<std::string>("slam_odom", slam_topic, "Odometry");
    private_nh.param<std::string>("odometry_new", odometry_new_topic, "lidar/odometry");
    private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    private_nh.param<std::string>("world_frame_id", world_frame_id, "world");
    private_nh.param<std::string>("origin_lidar_topic", origin_lidar_topic, "/livox/points");
    private_nh.param<std::string>("lidar_in_world_topic", cloud_world_topic, "/livox/cloud_in_world");
    private_nh.param<double>("drone_size", filter_size, 0.3);
    ROS_INFO_STREAM("base_link frame id: " << base_link_frame_id);
    ROS_INFO_STREAM("odom frame id: " << odom_frame_id);

    ros::Subscriber slam_odom_sub = nh.subscribe(slam_topic,10,&callback,ros::TransportHints().tcpNoDelay());
    odometry_new_pub = nh.advertise<nav_msgs::Odometry>(odometry_new_topic, 10);
    ros::Subscriber origin_lidar_sub = nh.subscribe(origin_lidar_topic, 10, &callback_cloud, ros::TransportHints().tcpNoDelay());
    cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_world_topic, 10);
    
    ros::spin();
    // ros::Rate rate(50);
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     if(init)
    //     {
    //         trans_result.stamp_ = ros::Time::now();
    //         tf_br->sendTransform(trans_result);
    //     }
    //     rate.sleep();
    // }
    return 0;
}