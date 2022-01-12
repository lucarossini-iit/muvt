#include <controller/robot_controller.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh("");

    XBot::HyperGraph::Controller::RobotController controller;

    ros::Rate rate(30);
    while(ros::ok())
    {
        controller.run();
    }
}
