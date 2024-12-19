#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_calib_result_node");
    ros::NodeHandle nh;

    // 创建一个 TransformBroadcaster
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // 设置平移
    transform.setOrigin(tf::Vector3(-77.78174593 / 1000.0, -77.78174593 / 1000.0, 85.0 / 1000.0));

    // 角度转换为弧度
    double angle_z = -45.0 * M_PI / 180.0;
    double angle_x = -22.0 * M_PI / 180.0;

    // 绕 Z 轴旋转 -45 度的旋转矩阵
    Eigen::Matrix3d R_z;
    R_z << cos(angle_z), -sin(angle_z), 0,
           sin(angle_z), cos(angle_z), 0,
           0, 0, 1;

    // 绕当前 X 轴旋转 -22 度的旋转矩阵
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
           0, cos(angle_x), -sin(angle_x),
           0, sin(angle_x), cos(angle_x);

    // 总的旋转矩阵
    Eigen::Matrix3d R = R_z * R_x;

    // 将旋转矩阵转换为四元数
    Eigen::Quaterniond quaternion(R);

    // 设置旋转（四元数）
    tf::Quaternion q(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
    transform.setRotation(q);

    // 输出平移和四元数表示的变换
    ROS_INFO("Translation: [%f, %f, %f]", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    ROS_INFO("Quaternion: [%f, %f, %f, %f]", q.x(), q.y(), q.z(), q.w());

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        // 发布静态变换
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "wrist_3_link", "camera_link"));
        rate.sleep();
    }

    return 0;
}