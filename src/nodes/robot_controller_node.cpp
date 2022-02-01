#include <controller/robot_controller.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh("");

    XBot::HyperGraph::Controller::RobotController controller;

    ros::Rate rate(40);
    while(ros::ok())
    {
        controller.run();
    }
}
