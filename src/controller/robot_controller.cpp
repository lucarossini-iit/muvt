#include <controller/robot_controller.h>

using namespace XBot::HyperGraph::Controller;

RobotController::RobotController(std::string ns):
_nh(ns),
_nhpr("~"),
_replay(false),
_index(0),
_incr(1)
{
    init_load_model();

    _trj_sub = _nh.subscribe<trajectory_msgs::JointTrajectoryConstPtr>("optimizer/solution", 10, &RobotController::trajectory_callback, this);

    _init_srv = _nh.advertiseService("init_service", &RobotController::init_service, this);
    _replay_srv = _nh.advertiseService("replay_service", &RobotController::replay_service, this);

    int rate = _nhpr.param("rate", 30);
    _r = std::make_shared<ros::Rate>(rate);
}

void RobotController::init_load_model()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    ConfigOptions cfg_reduced;

    std::string urdf, srdf, jidmap;

    if(_nh.hasParam("robot_description_reduced") && _nh.getParam("robot_description_reduced", urdf))
    {
        cfg_reduced.set_urdf(urdf);
    }
    else
    {
        throw std::runtime_error("robot_description parameter not set");
    }

    if(_nh.hasParam("robot_description_semantic_reduced") && _nh.getParam("robot_description_semantic_reduced", srdf))
    {
        cfg_reduced.set_srdf(srdf);
    }
    else
    {
        throw std::runtime_error("robot_description_semantic parameter not set");
    }

    if(_nh.hasParam("robot_description_joint_id_map") && _nh.getParam("robot_description_joint_id_map", jidmap))
    {
        cfg_reduced.set_jidmap(jidmap);
    }
    else
    {
        //success = false;
        if(!cfg_reduced.generate_jidmap())
            throw std::runtime_error("robot_description_joint_id_map parameter not set, failed to auto-generate jid_map");
    }



    std::string model_type;
    bool is_model_floating_base;

    cfg_reduced.set_parameter("model_type", _nhpr.param<std::string>("model_type", "RBDL"));

    cfg_reduced.set_parameter("is_model_floating_base", _nhpr.param<bool>("is_model_floating_base", false));

    cfg_reduced.set_parameter<std::string>("framework", "ROS");


    try
    {
        _robot = RobotInterface::getRobot(cfg);
        _robot->setControlMode(ControlMode::Position());
        auto fixed_joint_map = _nhpr.param<std::map<std::string, double>>("fixed_joints", std::map<std::string, double>());
        std::map<std::string, XBot::ControlMode> idle_joint_map;
        for(auto pair : fixed_joint_map)
            idle_joint_map.insert(std::make_pair(pair.first, XBot::ControlMode::Idle()));
        _robot->setControlMode(idle_joint_map);
    }
    catch(std::runtime_error& e)
    {

    }

    _model = XBot::ModelInterface::getModel(cfg_reduced);
    _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);

    if(!_robot)
    {
        std::cout << "Skipping creating RobotInterface. Check if XBotCore is running" << std::endl;
        Eigen::VectorXd qhome;
        _model->getRobotState("home", qhome);
        _model->setJointPosition(qhome);
        _model->update();
    }
    else
    {
        std::cout << "RobotInterface succesfully generated!" << std::endl;
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
        std::cout << i << std::endl;
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
        if(!_robot)
            _rspub->publishTransforms(ros::Time::now(), _nh.getNamespace().substr(1));
        rate.sleep();
    }

    return true;
}

bool RobotController::replay_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    _replay = req.data;
    if (_replay)
        res.message = "starting replaying trajectory";
    else
    {
        _index = 0;
        _incr = 1;
        res.message = "stopping replaying trajectory";
    }

    std::cout << res.message << std::endl;
    return true;
}

void RobotController::run()
{
    if(_robot)
    {
        _robot->sense();
        _model->syncFrom(*_robot);
//        _rspub->publishTransforms(ros::Time::now(), "sim");
    }
    else
        _rspub->publishTransforms(ros::Time::now(), _nh.getNamespace().substr(1));

    if (_replay)
    {
        XBot::JointNameMap joint_map;
        Eigen::VectorXd q = Eigen::VectorXd::Map(_trajectory.points[_index].positions.data(), _trajectory.points[_index].positions.size());
        _model->eigenToMap(q, joint_map);
        if (_robot)
        {
            _robot->setPositionReference(joint_map);
            _robot->move();
        }
        else
        {
            _model->setJointPosition(q);
            _model->update();
        }

        _index += _incr;
        if(_index == _trajectory.points.size() - 1 || _index == 0)
           _incr *= -1;

        _r->sleep();
    }

    ros::spinOnce();
}
