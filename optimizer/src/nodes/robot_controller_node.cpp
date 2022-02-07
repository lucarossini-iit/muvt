#include <controller/robot_controller.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh(""), nhpr("~");

    double ctrl_interpolation_time, opt_interpolation_time;
    if (!nhpr.hasParam("ctrl_interpolation_time") && !nh.getParam("ctrl_interpolation_time", ctrl_interpolation_time))
        ROS_ERROR("missing mandatory private parameter 'ctrl_interpolation_time!");

    if (!nh.hasParam("opt_interpolation_time") && !nh.getParam("opt_interpolation_time", opt_interpolation_time))
        ROS_ERROR("missing mandatory private parameter 'opt_interpolation_time!");

    std::cout << "ctrl interpolation time: " << ctrl_interpolation_time << "   opt_interpolation_time: " << opt_interpolation_time << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    ctrl_interpolation_time = 0.01;
    opt_interpolation_time = 0.05;
    int r;
    if (ctrl_interpolation_time <= opt_interpolation_time)
        r = int(1/ctrl_interpolation_time);
    else
        r = int(1/opt_interpolation_time);

    XBot::HyperGraph::Controller::RobotController controller;

    ros::Rate rate(r);
    while(ros::ok())
    {
        controller.run();
        rate.sleep();
        ros::spinOnce();
    }
}
