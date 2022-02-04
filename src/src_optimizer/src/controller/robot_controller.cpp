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

    if (!_nhpr.hasParam("opt_interpolation_time") && !_nhpr.getParam("opt_interpolation_time", _opt_interpolation_time))
        ROS_ERROR("missing mandatory private parameter 'opt_interpolation_time!");

    if (!_nhpr.hasParam("ctrl_interpolation_time") && !_nhpr.getParam("ctrl_interpolation_time", _ctrl_interpolation_time))
        ROS_ERROR("missing mandatory private parameter 'ctrl_interpolation_time!");

    _opt_interpolation_time = 0.05;
    _ctrl_interpolation_time = 0.01;
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
        throw std::runtime_error("robot_description_reduced parameter not set");
    }

    if(_nh.hasParam("robot_description_semantic_reduced") && _nh.getParam("robot_description_semantic_reduced", srdf))
    {
        cfg_reduced.set_srdf(srdf);
    }
    else
    {
        throw std::runtime_error("robot_description_semantic_reduced parameter not set");
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
        // if the optimizer interpolation time is lower than the controller interpolation time
        // the robot is moved using the _opt_interpolation_time
        if (_opt_interpolation_time <= _ctrl_interpolation_time)
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
        }
        // else we interpolate between two adjacent states coming from the optimizer using the
        // controller interpolation time
        else
        {
            // find the number of interpolated states to be added
            int num_steps = int(_opt_interpolation_time / _ctrl_interpolation_time);

            // initial state
            if (_index % num_steps == 0)
                _model->getJointPosition(_q_init);

            // final state
            XBot::JointNameMap joint_map_fin;
            Eigen::VectorXd q_fin(_model->getJointNum());
            int trajectory_index;
            if (_incr == 1)
                trajectory_index = int(_index/num_steps);
            else
                trajectory_index = int(_index/num_steps) - 1;

            q_fin = Eigen::VectorXd::Map(_trajectory.points[trajectory_index].positions.data(), _trajectory.points[trajectory_index].positions.size());
            _model->eigenToMap(q_fin, joint_map_fin);          

            Eigen::VectorXd q(_model->getJointNum());
            XBot::JointNameMap joint_map;

            // reference state
            int index_reference;
            if (_incr == 1)
                index_reference = _index - num_steps*int(_index/num_steps)+1;
            else
                index_reference = _incr * (_index - num_steps*(trajectory_index+1) - num_steps);

            q = _q_init + (q_fin - _q_init) * (index_reference) / num_steps;
            _model->eigenToMap(q, joint_map);
            if (_robot)
            {
                _robot->setPositionReference(joint_map);
                _robot->move();
            }
            else
            {
                _model->setJointPosition(joint_map);
                _model->update();
            }

            _index += _incr;
            if(_index == _trajectory.points.size()*num_steps || (_index == num_steps && _incr == -1))
                _incr *= -1;

        }
    }
}