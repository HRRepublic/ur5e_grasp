#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <thread>
#include <atomic>
#include <iostream>

// 文件路径
const std::string FILE_PATH = "/home/shi/catkin_ws/calibration_data.txt";

// 全局变量
std::atomic<bool> save_pose(false);
std::atomic<bool> exit_program(false);
geometry_msgs::PoseStamped aruco_pose;
std::mutex pose_mutex;

// 保存位姿到文件
void savePoseToFile(const std::string& label, const geometry_msgs::Pose& pose, std::ofstream& file)
{
    // 转换为 RX, RY, RZ 形式（角度制）
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 将弧度转换为角度
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    // 保存到文件
    file << label << "," << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
         << roll << "," << pitch << "," << yaw << std::endl;
}

// 回调函数
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex);
    aruco_pose = *msg;
}

// 键盘输入线程
void keyboardThread()
{
    char input;
    while (ros::ok() && !exit_program)
    {
        std::cin >> input;
        if (input == 's')
        {
            save_pose = true;
        }
        else if (input == 'q')
        {
            exit_program = true;
        }
    }
}

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "calibration_data_recorder_node");
    ros::NodeHandle nh;

    // 初始化 MoveIt! 接口
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // 订阅 /aruco_tracker/pose 主题
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_tracker/pose", 10, poseCallback);

    // 启动键盘输入线程
    std::thread keyboard_thread(keyboardThread);

    // 清空文件内容
    std::ofstream file;
    file.open(FILE_PATH, std::ios::out | std::ios::trunc);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", FILE_PATH.c_str());
        return 1;
    }
    file.close();

    // 使用 AsyncSpinner
    ros::AsyncSpinner spinner(1); // 使用一个线程
    spinner.start();

    // 主循环
    while (ros::ok() && !exit_program)
    {
        if (save_pose)
        {
            std::lock_guard<std::mutex> lock(pose_mutex);

            // 打开文件
            std::ofstream file;
            file.open(FILE_PATH, std::ios::app);
            if (!file.is_open())
            {
                ROS_ERROR("Failed to open file: %s", FILE_PATH.c_str());
                continue;
            }

            // 保存标定板位姿
            savePoseToFile("hand", aruco_pose.pose, file);

            // 获取机械臂末端位姿
            geometry_msgs::PoseStamped end_effector_pose = move_group.getCurrentPose();
            savePoseToFile("eye", end_effector_pose.pose, file);

            // 关闭文件
            file.close();

            save_pose = false;
        }

        // 等待一段时间以避免高 CPU 占用
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待键盘输入线程结束
    keyboard_thread.join();

    return 0;
}