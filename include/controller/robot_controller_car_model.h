#ifndef ROBOT_CONTROLLER_CAR_MODEL_H
#define ROBOT_CONTROLLER_CAR_MODEL_H

#include <ros/ros.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <optimizer/optimizer.h>
#include <simulator/simulator.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>

namespace XBot { namespace HyperGraph { namespace Controller {

class RobotControllerCarModel {
public:
    typedef std::shared_ptr<RobotControllerCarModel> Ptr;

    RobotControllerCarModel(std::string ns = "");

    void run();

private:
    void init_load_model();
    void init_load_config();
    bool velocity_check(Eigen::VectorXd q_init, Eigen::VectorXd q_fin);

//    void on_timer_cb(const ros::TimerEvent&);

    void trajectory_callback(trajectory_msgs::JointTrajectoryConstPtr msg);

    bool replay_service(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response & res);

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _trj_sub;
    ros::Publisher _trj_index_pub, _postural_pub;
    ros::ServiceServer _replay_srv;
    trajectory_msgs::JointTrajectory _trajectory;

    XBot::ModelInterface::Ptr _model, _ci_model;
    XBot::RobotInterface::Ptr  _robot;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;

    Eigen::VectorXd _q_init;

    double _opt_interpolation_time, _ctrl_interpolation_time, _time;
    bool _replay;
    int _index, _incr;

};
} } }

#endif // ROBOT_CONTROLLER_CAR_MODEL_H
