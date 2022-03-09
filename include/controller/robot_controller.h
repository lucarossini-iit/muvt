#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <teb_test/optimizer/optimizer.h>

#include <simulator/simulator.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

namespace XBot { namespace HyperGraph { namespace Controller {

class RobotController {
public:
    typedef std::shared_ptr<RobotController> Ptr;

    RobotController(std::string ns = "");

    void run();

private:
    void init_load_model();
    bool velocity_check(Eigen::VectorXd q_init, Eigen::VectorXd q_fin, int num_steps);

    void trajectory_callback(trajectory_msgs::JointTrajectoryConstPtr msg);

    bool init_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool replay_service(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response & res);

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _trj_sub;
    ros::Publisher _trj_index_pub;
    ros::ServiceServer _init_srv, _replay_srv;
    std::shared_ptr<ros::Rate> _r;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    double _opt_interpolation_time, _ctrl_interpolation_time;

    trajectory_msgs::JointTrajectory _trajectory;

    std::vector<std::string> _keys;

    bool _replay;

    int _index, _incr;

    Eigen::VectorXd _q_init;


};

} } }

#endif // ROBOT_CONTROLLER_H
