#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>

class ImuTransformer {
private:
    ros::NodeHandle nh_,private_nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher transformed_imu_pub_;

    // 平移矩阵
    Eigen::Vector3d translation_;
    // 旋转矩阵
    Eigen::Matrix3d rotation_matrix_;

public:
    ImuTransformer() {
        private_nh_ = ros::NodeHandle("~");
        std::vector<double> extrinT(3, 0.0);
        std::vector<double> extrinR(9, 0.0);

        private_nh_.param<std::vector<double>>("imu_extrinsic_T", extrinT, std::vector<double>());
        private_nh_.param<std::vector<double>>("imu_extrinsic_R", extrinR, std::vector<double>());

        translation_(0) = extrinT[0];
        translation_(1) = extrinT[1];
        translation_(2) = extrinT[2];
        for(int index=0;index<9;index++)
        {
            rotation_matrix_(index) = extrinR[index];
        }
        std::cout << translation_<< std::endl;
        std::cout << rotation_matrix_ << std::endl;

        // 订阅IMU话题
        imu_sub_ = nh_.subscribe("/imu/data", 10, &ImuTransformer::imuCallback, this);

        // 发布转换后的IMU话题
        transformed_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/transformed_imu", 10);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 转换线性加速度
        Eigen::Vector3d linear_acc(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
        
        // 应用旋转矩阵
        Eigen::Vector3d rotated_acc = rotation_matrix_ * linear_acc;
        
        // 应用平移
        Eigen::Vector3d transformed_acc = rotated_acc + translation_;

        // 转换角速度
        Eigen::Vector3d angular_vel(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
        
        // 应用旋转矩阵到角速度
        Eigen::Vector3d rotated_angular_vel = rotation_matrix_ * angular_vel;

        // 创建新的IMU消息
        sensor_msgs::Imu transformed_imu = *msg;
        
        // 更新线性加速度
        transformed_imu.linear_acceleration.x = transformed_acc(0);
        transformed_imu.linear_acceleration.y = transformed_acc(1);
        transformed_imu.linear_acceleration.z = transformed_acc(2);
        
        // 更新角速度
        transformed_imu.angular_velocity.x = rotated_angular_vel(0);
        transformed_imu.angular_velocity.y = rotated_angular_vel(1);
        transformed_imu.angular_velocity.z = rotated_angular_vel(2);

        // 发布转换后的IMU消息
        transformed_imu_pub_.publish(transformed_imu);
    }
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "imu_transformer_node");

    // 创建转换器实例
    ImuTransformer transformer;

    // spin
    ros::spin();

    return 0;
}