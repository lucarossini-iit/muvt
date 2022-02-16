#include <controller/robot_controller_car_model.h>

using namespace XBot::HyperGraph::Controller;

extern int num_steps, trajectory_index, old_trajectory_index, index_reference;

RobotControllerCarModel::RobotControllerCarModel(std::string ns):
_nh(ns),
_nhpr("~"),
_replay(false),
_index(0),
_incr(1)
{
    init_load_model();
    init_load_config();

    _replay_srv = _nh.advertiseService("replay_service", &RobotControllerCarModel::replay_service, this);

    _trj_sub = _nh.subscribe<trajectory_msgs::JointTrajectoryConstPtr>("optimizer/solution", 10, &RobotControllerCarModel::trajectory_callback, this);

    _trj_index_pub = _nh.advertise<std_msgs::Int16>("optimizer/trajectory_index", 10, true);
}

void RobotControllerCarModel::trajectory_callback(trajectory_msgs::JointTrajectoryConstPtr msg)
{
    _trajectory = *msg;
}

bool RobotControllerCarModel::replay_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    _replay = req.data;
    if (_replay)
        res.message = "starting replaying trajectory";
    else
    {
        _index = 0;
        res.message = "stopping replaying trajectory";
    }

    res.success = true;
    std::cout << res.message << std::endl;
    return res.success;
}

void RobotControllerCarModel::init_load_model()
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

    cfg_reduced.set_parameter("is_model_floating_base", _nhpr.param<bool>("is_model_floating_base", true));

    cfg_reduced.set_parameter<std::string>("framework", "ROS");

    try
    {
        _robot = RobotInterface::getRobot(cfg);
        _robot->setControlMode(ControlMode::Position());
//        auto fixed_joint_map = _nhpr.param<std::map<std::string, double>>("fixed_joints", std::map<std::string, double>());
//        std::map<std::string, XBot::ControlMode> idle_joint_map;
//        for(auto pair : fixed_joint_map)
//            idle_joint_map.insert(std::make_pair(pair.first, XBot::ControlMode::Idle()));
//        _robot->setControlMode(idle_joint_map);
    }
    catch(std::runtime_error& e)
    {

    }

    _model = XBot::ModelInterface::getModel(cfg_reduced);
    _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);

    _ci_model = XBot::ModelInterface::getModel(cfg);

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
}

void RobotControllerCarModel::init_load_config()
{

    if (!_nhpr.hasParam("opt_interpolation_time") && !_nhpr.getParam("opt_interpolation_time", _opt_interpolation_time))
    {
        ROS_ERROR("missing mandatory private parameter 'opt_interpolation_time!");
//        throw std::runtime_error("opt_interpolation_time!");
    }

    if (!_nhpr.hasParam("ctrl_interpolation_time") && !_nhpr.getParam("ctrl_interpolation_time", _ctrl_interpolation_time))
    {
        ROS_ERROR("missing mandatory private parameter 'ctrl_interpolation_time!");
//        throw std::runtime_error("ctrl_interpolation_time!");
    }

    std::string cartesian_stack;
    if(!_nh.getParam("cartesian_stack", cartesian_stack))
    {
        ROS_ERROR("cartesian_stackcartesian_stack!");
        throw std::runtime_error("cartesian_stack!");
    }

    auto ik_yaml_goal = YAML::Load(cartesian_stack);

    double ci_period = _ctrl_interpolation_time;
    auto ci_ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<XBot::Cartesian::Parameters>(ci_period), _ci_model);
    auto ik_prob = XBot::Cartesian::ProblemDescription(ik_yaml_goal, ci_ctx);

    _ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                        ik_prob, ci_ctx);

    _opt_interpolation_time = 0.1;
    _ctrl_interpolation_time = 0.01;

    // interpolator data
    num_steps = int(_opt_interpolation_time / _ctrl_interpolation_time);
    trajectory_index = 1;
    old_trajectory_index = trajectory_index;
}

bool RobotControllerCarModel::velocity_check(Eigen::VectorXd q_init, Eigen::VectorXd q_fin)
{
    Eigen::VectorXd qdot = (q_fin - q_init)/(_ctrl_interpolation_time * num_steps);
    Eigen::VectorXd qdot_max;
    _model->getVelocityLimits(qdot_max);
    qdot_max /= 10;

    for (int i = 0; i < qdot.size(); i++)
        if(qdot(i) > qdot_max(i) || qdot(i) < -qdot_max(i))
        {
            return false;
        }

    return true;
}

void RobotControllerCarModel::run()
{
    if(_robot)
    {
        _robot->sense();
        _model->syncFrom(*_robot);
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
            index_reference++;
            // update
            if ((index_reference - 1) % num_steps == 0)
            {
                // reset num_steps to the nominal value
                num_steps = int(_opt_interpolation_time / _ctrl_interpolation_time);

                // reset index_reference
                index_reference = 1;

                // update trajectory_index
                if (trajectory_index == _trajectory.points.size() - 1 || trajectory_index == 0)
                    _incr *= -1;
                trajectory_index += _incr;

                std_msgs::Int16 msg;
                msg.data = trajectory_index;
                _trj_index_pub.publish(msg);

                if(_incr == 1)
                    _q_init = Eigen::VectorXd::Map(_trajectory.points[trajectory_index-1].positions.data(), _trajectory.points[trajectory_index-1].positions.size());
                else
                    _q_init = Eigen::VectorXd::Map(_trajectory.points[trajectory_index+1].positions.data(), _trajectory.points[trajectory_index+1].positions.size());
            }

            // final state
            XBot::JointNameMap joint_map_fin;
            Eigen::VectorXd q_fin(_model->getJointNum());
            q_fin = Eigen::VectorXd::Map(_trajectory.points[trajectory_index].positions.data(), _trajectory.points[trajectory_index].positions.size());
            _model->eigenToMap(q_fin, joint_map_fin);

            // velocity check increases the number of steps to move from two adjacent optimizer states
            while (!velocity_check(_q_init, q_fin))
                num_steps += 1;

            // compute reference
            Eigen::VectorXd q(_model->getJointNum());
            XBot::JointNameMap joint_map;
            q = _q_init + (q_fin - _q_init) * (index_reference) / num_steps;
            _model->eigenToMap(q, joint_map);

            // once computed the q_ref, find the new reference for the wheels
//            XBot::JointNameMap ci_joint_map;
//            _ci_model->getJointPosition(ci_joint_map);
//            for (auto pair : joint_map)
//                ci_joint_map[pair.first] = pair.second;
//            _ci_model->setJointPosition(ci_joint_map);
//            _ci_model->update();
//            auto time = ros::Time::now();
//            _ci->reset(time.toSec());
//            _ci->update(time.toSec(), _ctrl_interpolation_time);
//            Eigen::VectorXd ci_qdot, ci_q;
//            _ci_model->mapToEigen(ci_joint_map, ci_q);
//            _ci_model->getJointVelocity(ci_qdot);
//            ci_q += ci_qdot * _ctrl_interpolation_time;
//            _model->eigenToMap(ci_q, ci_joint_map);
//            for (auto& pair : joint_map)
//                pair.second = ci_joint_map[pair.first];

            // move
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
        }
    }
    else
    {
        std_msgs::Int16 msg;
        msg.data = trajectory_index;
        _trj_index_pub.publish(msg);
    }
}
