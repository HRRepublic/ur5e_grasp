#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <thread>
#include <atomic>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

// 文件路径
const std::string FILE_PATH = "/home/shi/catkin_ws/calibration_data.txt";
const std::string IMAGE_PATH = "/home/shi/catkin_ws/images/";

// 全局变量
std::atomic<bool> save_pose(false);
std::atomic<bool> exit_program(false);
geometry_msgs::PoseStamped end_effector_pose;
std::mutex pose_mutex;
cv::Mat latest_image;
std::mutex image_mutex;

// 保存位姿到文件并输出到终端
void savePoseToFileAndPrint(const std::string& label, const geometry_msgs::Pose& pose, std::ofstream& file)
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

    // 将位置转换为毫米
    double x_mm = pose.position.x * 1000.0;
    double y_mm = pose.position.y * 1000.0;
    double z_mm = pose.position.z * 1000.0;

    // 保存到文件
    file << label << "," << x_mm << "," << y_mm << "," << z_mm << ","
         << roll << "," << pitch << "," << yaw << std::endl;

    // 输出到终端
    ROS_INFO("%s Position: [x: %f mm, y: %f mm, z: %f mm], Orientation (RPY): [roll: %f, pitch: %f, yaw: %f]",
             label.c_str(), x_mm, y_mm, z_mm, roll, pitch, yaw);
}

// 回调函数：保存最新的图像
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(image_mutex);
    try
    {
        latest_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
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

    // 订阅图像话题
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/stereo_inertial_publisher/color/image", 10, imageCallback);

    // 启动键盘输入线程
    std::thread keyboard_thread(keyboardThread);

    std::ofstream file;
    // 打开文件，清空文件内容
    //file.open(FILE_PATH, std::ios::out | std::ios::trunc);
    // 打开文件（以追加模式）
    file.open(FILE_PATH, std::ios::app);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", FILE_PATH.c_str());
        return 1;
    }
    file.close();

    // 使用 AsyncSpinner
    ros::AsyncSpinner spinner(1); // 使用一个线程
    spinner.start();

    int image_count = 19;

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

            // 保存机械臂末端位姿并输出到终端
            geometry_msgs::PoseStamped end_effector_pose = move_group.getCurrentPose();
            savePoseToFileAndPrint("hand", end_effector_pose.pose, file);

            // 保存图像
            if (!latest_image.empty())
            {
                std::string image_filename = IMAGE_PATH + "image_" + std::to_string(image_count++) + ".png";
                cv::imwrite(image_filename, latest_image);
                ROS_INFO("Saved image to %s", image_filename.c_str());
            }

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