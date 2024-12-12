#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_simple_grasp_node");
    ros::NodeHandle nh;

    ros::Publisher target_point_pub = nh.advertise<geometry_msgs::Point>("/target_position", 10);

    ros::Rate loop_rate(1);
    
    ros::Duration(1.0).sleep();

    geometry_msgs::Point target_point;
    target_point.x = 0;
    target_point.y = 0;
    target_point.z = -0.1;

    target_point_pub.publish(target_point);

    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}