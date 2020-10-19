#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <environment/obstacle.h>
#include <environment/vertex_se3.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Define obstacles position which will be added in rviz
    Eigen::Affine3d pose;
    pose.translation() << 1.0, 0.0, 0.0;
    pose.linear() = Eigen::Matrix3d::Identity();

    XBot::Obstacle::Obstacle obstacle(pose, "obstacle_1", nh);

    return true;
}
