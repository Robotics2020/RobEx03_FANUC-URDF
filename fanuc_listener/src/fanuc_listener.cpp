#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fanuc_listener");
    ros::NodeHandle nodeHandle;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(1.0);
    while (nodeHandle.ok()) {
        geometry_msgs::TransformSpamped transformStamped;
        ROS_INFO("Hello, there! I successfully created a node!")
    }
}