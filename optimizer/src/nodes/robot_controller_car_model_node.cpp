#include <controller/robot_controller_car_model.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh(""), nhpr("~");

    double ctrl_interpolation_time, opt_interpolation_time;
    opt_interpolation_time = nhpr.param("opt_interpolation_time", 0.0);
    ctrl_interpolation_time = nhpr.param("ctrl_interpolation_time", 0.0);
//    opt_interpolation_time = 0.1;
//    ctrl_interpolation_time = 0.01;

    if (opt_interpolation_time == 0)
    {
        ROS_ERROR("missing mandatory private parameter 'opt_interpolation_time!");
        throw std::runtime_error("opt_interpolation_time!");
    }

    if (ctrl_interpolation_time == 0)
    {
        ROS_ERROR("missing mandatory private parameter 'ctrl_interpolation_time!");
        throw std::runtime_error("ctrl_interpolation_time!");
    }

    int r;
    if (ctrl_interpolation_time <= opt_interpolation_time)
        r = int(1/ctrl_interpolation_time);
    else
        r = int(1/opt_interpolation_time);

    XBot::HyperGraph::Controller::RobotControllerCarModel controller;

    ros::Rate rate(r);
    while(ros::ok())
    {
        controller.run();
        rate.sleep();
        ros::spinOnce();
    }
}
