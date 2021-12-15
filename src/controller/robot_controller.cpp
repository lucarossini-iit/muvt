#include <controller/robot_controller.h>

using namespace XBot::HyperGraph::Controller;

RobotController::RobotController(std::string ns):
_nh(ns)
{
    init_load_model();

    _trj_sub = _nh.subscribe<trajectory_msgs::JointTrajectoryConstPtr>("optimizer/solution", 10, &RobotController::trajectory_callback, this);

    _init_srv = _nh.advertiseService("init_service", &RobotController::init_service, this);
}

void RobotController::init_load_model()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();

    try
    {
        _robot = RobotInterface::getRobot(cfg);
        _robot->setControlMode(ControlMode::Position());
    }
    catch(std::runtime_error& e)
    {

    }

    _model = XBot::ModelInterface::getModel(cfg);
    _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);

    if(!_robot)
    {
        Eigen::VectorXd qhome;
        _model->getRobotState("home", qhome);
        _model->setJointPosition(qhome);
        _model->update();
    }
    else
    {
        _model->syncFrom(*_robot);
    }

    XBot::JointNameMap q_map;
    _model->getJointPosition(q_map);
    for (auto joint_pair : q_map)
        _keys.push_back(joint_pair.first);
}

void RobotController::trajectory_callback(trajectory_msgs::JointTrajectoryConstPtr msg)
{
    _trajectory = *msg;
}

bool RobotController::init_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    double T = 1;
    double dt = 0.01;
    XBot::JointNameMap q_init, q_fin, q_ref;
    _model->getJointPosition(q_init);
    q_ref = q_init;
    ros::Rate rate(1/dt);

    Eigen::VectorXd q = Eigen::VectorXd::Map(_trajectory.points[0].positions.data(), _trajectory.points[0].positions.size());
    _model->eigenToMap(q, q_fin);

    for (int i = 0; i < T/dt; i++)
    {
        for (auto key : _keys)
        {
            q_ref[key] = q_init[key] + ((q_fin[key] - q_init[key]) * i * dt / T);
        }
        if(_robot)
        {
            _robot->setPositionReference(q_ref);
            _robot->move();
        }
        _model->setJointPosition(q_ref);
        _model->update();
//        _rspub->publishTransforms(ros::Time::now(), _nh.getNamespace());
        rate.sleep();
    }
}

void RobotController::run()
{
    _model->syncFrom(*_robot);
    ros::spinOnce();
}
