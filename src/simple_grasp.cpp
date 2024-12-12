#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>

class GraspingDemo
{
    private:
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner;
        moveit::planning_interface::MoveGroupInterface armgroup;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const robot_state::JointModelGroup* joint_model_group;
        ros::Subscriber target_pose_sub;
        tf::Transform transform_camera_to_ee;
        geometry_msgs::Pose pregrasp_pose;
        geometry_msgs::PoseStamped currPose;
    
    public:
        GraspingDemo(ros::NodeHandle n_) : spinner(1), armgroup("manipulator"), nh(n_)
        {
            // 初始化手眼标定结果
            transform_camera_to_ee.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            transform_camera_to_ee.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

            // 初始化准备抓取位姿
            pregrasp_pose.position.x = -0.16277;
            pregrasp_pose.position.y = 0.578169;
            pregrasp_pose.position.z = 0.482127;
            pregrasp_pose.orientation.x = 0.910841;
            pregrasp_pose.orientation.y = -0.410504;
            pregrasp_pose.orientation.z = -0.0428172;
            pregrasp_pose.orientation.w = 0.00461386;

            spinner.start();

            // 设置规划器
            armgroup.setPlannerId("RRTstarkConfigDefault");

            // 获取机器人模型信息
            joint_model_group = armgroup.getCurrentState()->getJointModelGroup("manipulator");

            ROS_INFO("Planning frame: %s", armgroup.getPlanningFrame().c_str());
            ROS_INFO("End effector link: %s", armgroup.getEndEffectorLink().c_str());

            currPose = armgroup.getCurrentPose();
            ROS_INFO("Current pose: x=%f, y=%f, z=%f", currPose.pose.position.x, currPose.pose.position.y, currPose.pose.position.z);
            
            ROS_INFO_STREAM("Getting into the Grasping Position....");

            //控制机械臂运动到准备抓取的位姿
            armgroup.setPoseTarget(pregrasp_pose);
            bool success = (armgroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                ROS_INFO("Plan successful, executing...");
                armgroup.move(); // 执行运动
                ROS_INFO("Sucssefully moved to pregrasp pose!");
            }
            else
            {
                ROS_WARN("Plan failed!");
            }

            // 订阅目标位姿主题
            target_pose_sub = nh.subscribe("/target_position", 1, &GraspingDemo::targetPositionCallback, this);
        }

        ~GraspingDemo() {
            spinner.stop();
        }

        void attainPosition(float x, float y, float z);
        void targetPositionCallback(const geometry_msgs::Point& msg);

};

void GraspingDemo::attainPosition(float x, float y, float z)
{
    ROS_INFO("The attain position function called");

    ros::Duration(1.0).sleep();
    // 获取当前位姿
    currPose = armgroup.getCurrentPose();

    // 设置目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation = currPose.pose.orientation;
    armgroup.setPoseTarget(target_pose);
    armgroup.move(); // 执行运动

}

void GraspingDemo::targetPositionCallback(const geometry_msgs::Point& msg)
{
    // 获取相机坐标系中的目标位置，并获取目标位姿到相机坐标系的变换
    geometry_msgs::Point target_position_camera = msg;

    ROS_INFO("Target position in camera frame: x=%f, y=%f, z=%f", target_position_camera.x, target_position_camera.y, target_position_camera.z);

    geometry_msgs::Pose target_pose_camera;
    target_pose_camera.position = target_position_camera;
    target_pose_camera.orientation.w = 1.0;
    tf::Transform transform_target_to_camera;
    transform_target_to_camera.setOrigin(tf::Vector3(target_pose_camera.position.x, target_pose_camera.position.y, target_pose_camera.position.z));
    transform_target_to_camera.setRotation(tf::Quaternion(target_pose_camera.orientation.x, target_pose_camera.orientation.y, target_pose_camera.orientation.z, target_pose_camera.orientation.w));

    ros::Duration(1.0).sleep();
    // 获取机械臂末端当前位姿
    currPose = armgroup.getCurrentPose();
    ROS_INFO("Current pose: x=%f, y=%f, z=%f", currPose.pose.position.x, currPose.pose.position.y, currPose.pose.position.z);
    
    // 获取机械臂末端坐标系到机械臂基座坐标系的变换
    tf::Transform transform_ee_to_base;
    transform_ee_to_base.setOrigin(tf::Vector3(currPose.pose.position.x, currPose.pose.position.y, currPose.pose.position.z));
    transform_ee_to_base.setRotation(tf::Quaternion(currPose.pose.orientation.x, currPose.pose.orientation.y, currPose.pose.orientation.z, currPose.pose.orientation.w));

    tf::Transform transform_target_pose_base = transform_ee_to_base * transform_camera_to_ee * transform_target_to_camera;

    geometry_msgs::Pose target_pose_base_msg;
    tf::poseTFToMsg(transform_target_pose_base, target_pose_base_msg);

    ROS_INFO("Target position in base frame: x=%f, y=%f, z=%f", target_pose_base_msg.position.x, target_pose_base_msg.position.y, target_pose_base_msg.position.z);

    // 机械臂末端移动到相应位置
    attainPosition(target_pose_base_msg.position.x, target_pose_base_msg.position.y, target_pose_base_msg.position.z);

}

int main(int argc, char **argv)
{
  // 初始化ROS节点，节点名为simple_grasp_node
  ros::init(argc, argv, "simple_grasp_node");
 
  // 节点句柄
  ros::NodeHandle nh;
 
  // 创建一个对象，将参数传递进去
  GraspingDemo simpleGrasp(nh);
  ROS_INFO_STREAM("Waiting for five seconds..");
 
  ros::WallDuration(5.0).sleep();

  ROS_INFO_STREAM("Ready to grasp!");
 
  // 如果识别节点有目标位置发来，则进行抓取
  while (ros::ok())
  {
    // Process image callback
    ros::spinOnce();
  }
  return 0;
}