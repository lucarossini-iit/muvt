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

//    trajectory_msgs::JointTrajectory trajectory;
//    bool isCallbackDone = false;

//    auto solution_callback = [&trajectory, &isCallbackDone](trajectory_msgs::JointTrajectoryConstPtr msg)
//    {
//        trajectory = *msg;
//        isCallbackDone = true;
//    };
//    ros::Subscriber trj_sub = nh.subscribe<trajectory_msgs::JointTrajectory>("optimizer/solution", 10, solution_callback);

//    // create ModelInterface
//    auto cfg = XBot::ConfigOptionsFromParamServer();
//    auto model = XBot::ModelInterface::getModel(cfg);
//    XBot::RobotInterface::Ptr robot;
//    try
//    {
//        auto robot = RobotInterface::getRobot(cfg);
//        robot->setControlMode(ControlMode::Position());
//    }
//    catch(std::runtime_error& e)
//    {

//    }



//    // set home state
//    XBot::JointNameMap q;
//    model->getRobotState("home", q);
//    model->setJointPosition(q);
//    model->update();

//    XBot::Cartesian::Utils::RobotStatePublisher rs_pub(model);

//    int index = 0;
//    int incr = 1;

//    ros::Rate r(50);
//    while (ros::ok())
//    {
//        if (isCallbackDone)
//        {
//            Eigen::VectorXd q = Eigen::VectorXd::Map(trajectory.points[index].positions.data(), trajectory.points[index].positions.size());
//            model->setJointPosition(q);
//            model->update();

//            rs_pub.publishTransforms(ros::Time::now(), "");

//            index += incr;
//            if(index == trajectory.points.size() - 1 || index == 0)
//                incr *= -1;

//            ros::spinOnce();
//            r.sleep();
//        }
//        else
//        {
//            ros::spinOnce();
//            r.sleep();
//        }
//    }
}
