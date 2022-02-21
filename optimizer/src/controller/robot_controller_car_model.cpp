#include <controller/robot_controller_car_model.h>

using namespace XBot::HyperGraph::Controller;

extern int num_steps, trajectory_index, old_trajectory_index, index_reference;
Eigen::VectorXd q_fb_old;
bool start;
double velodyne_joint_pos;

RobotControllerCarModel::RobotControllerCarModel(std::string ns):
_nh(ns),
_nhpr("~"),
_replay(false),
_index(0),
_incr(1),
_time(0.0)
{
    init_load_model();
    init_load_config();

    _init_srv = _nh.advertiseService("init_service",  &RobotControllerCarModel::init_service, this);
    _replay_srv = _nh.advertiseService("replay_service", &RobotControllerCarModel::replay_service, this);

    _trj_sub = _nh.subscribe<trajectory_msgs::JointTrajectoryConstPtr>("optimizer/solution", 10, &RobotControllerCarModel::trajectory_callback, this);
    _xbot_sub = _nh.subscribe<xbot_msgs::JointStateConstPtr>("/xbotcore/joint_states", 10, &RobotControllerCarModel::robot_callback, this);

    _trj_index_pub = _nh.advertise<std_msgs::Int16>("optimizer/trajectory_index", 10, true);
    _postural_pub = _nh.advertise<sensor_msgs::JointState>("/cartesian/Postural/reference", 10, true);

    start = false;

    velodyne_joint_pos = 0;
}

void RobotControllerCarModel::trajectory_callback(trajectory_msgs::JointTrajectoryConstPtr msg)
{
    _trajectory = *msg;
}

void RobotControllerCarModel::robot_callback(const xbot_msgs::JointStateConstPtr msg)
{
    XBot::JointNameMap joint_map;
    for (int i = 0; i < msg->name.size(); i++)
    {
        joint_map[msg->name[i]] = msg->motor_position[i];
    }

    velodyne_joint_pos = joint_map["velodyne_joint"];
}

bool RobotControllerCarModel::init_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    double T = 1;
    double dt = 0.01;
    Eigen::VectorXd q_init, q_fin, q_ref, qdot, ci_q;
    _ci_model->getJointPosition(q_init);
    q_ref = q_init;
    ros::Rate rate(1/dt);

    q_fin = Eigen::VectorXd::Map(_trajectory.points[0].positions.data(), _trajectory.points[0].positions.size());

    for (int i = 0; i < T/dt; i++)
    {
        q_ref = q_init + ((q_fin - q_init) * i * dt / T);
        _ci->setReferencePosture(q_ref);
        _ci->update(i*dt/T, dt);
        _ci_model->getJointVelocity(qdot);
        _ci_model->getJointPosition(ci_q);
        ci_q += qdot * dt;
        if(_robot)
        {
            _robot->setPositionReference(ci_q);
            _robot->move();
        }
        _ci_model->setJointPosition(ci_q);
        _ci_model->update();
        if(!_robot)
            _rspub->publishTransforms(ros::Time::now(), _nh.getNamespace().substr(1));
        rate.sleep();
    }

    trajectory_index = 1;

    return true;

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
        std::map<std::string, ControlMode> control_map;
        _robot->getControlMode(control_map);
        control_map["j_wheel_1"] = ControlMode::Velocity();
        control_map["j_wheel_2"] = ControlMode::Velocity();
        control_map["j_wheel_3"] = ControlMode::Velocity();
        control_map["j_wheel_4"] = ControlMode::Velocity();
        control_map["velodyne_joint"] = ControlMode::Idle();
        _robot->setControlMode(control_map);
    }
    catch(std::runtime_error& e)
    {

    }

    _model = XBot::ModelInterface::getModel(cfg_reduced);
    _ci_model = XBot::ModelInterface::getModel(cfg);
    _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_ci_model);

    if(!_robot)
    {
        std::cout << "Skipping creating RobotInterface. Check if XBotCore is running" << std::endl;
        Eigen::VectorXd qhome;
        _model->getRobotState("home", qhome);
        _model->setJointPosition(qhome);
        _model->update();
        qhome.resize(_ci_model->getJointNum());
        _ci_model->getRobotState("home", qhome);
        _ci_model->setJointPosition(qhome);
        _ci_model->update();

    }
    else
    {
        std::cout << "RobotInterface succesfully generated!" << std::endl;
        _ci_model->syncFrom(*_robot);
        _ci_model->getJointPosition(q_fb_old);
    }
}

void RobotControllerCarModel::init_load_config()
{
    _opt_interpolation_time = _nhpr.param("opt_interpolation_time", 0.0);
    _ctrl_interpolation_time = _nhpr.param("ctrl_interpolation_time", 0.0);

//    _opt_interpolation_time = 0.1;
//    _ctrl_interpolation_time = 0.01;

    if (_opt_interpolation_time == 0)
    {
        ROS_ERROR("missing mandatory private parameter 'opt_interpolation_time!");
        throw std::runtime_error("opt_interpolation_time!");
    }

    if (_ctrl_interpolation_time == 0)
    {
        ROS_ERROR("missing mandatory private parameter 'ctrl_interpolation_time!");
        throw std::runtime_error("ctrl_interpolation_time!");
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
    double vel_max_fb_tr = 0.25;
    double vel_max_fb_or = 0.2;
    qdot_max(0) = vel_max_fb_tr; qdot_max(1) = vel_max_fb_tr; qdot_max(2) = vel_max_fb_tr; qdot_max(3) = vel_max_fb_or; qdot_max(4) = vel_max_fb_or; qdot_max(5) = vel_max_fb_or;
    qdot_max /= 1;

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
        if(!start)
        {
            _ci_model->syncFrom(*_robot);
        }
        else
        {
            _ci_model->setJointPosition(q_fb_old);
            _ci_model->update();
        }
    }

    _rspub->publishTransforms(ros::Time::now(), _nh.getNamespace().substr(1));

    if (_replay)
    {
        if (!start && _robot)
        {
            Eigen::VectorXd q_robot;
            _robot->getJointPosition(q_robot);
            q_fb_old.tail(q_robot.size()) = q_robot;
            _ci_model->setJointPosition(q_fb_old);
            _ci_model->update();
            start = true;
        }
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
            // Retrieve joint position
            XBot::JointNameMap ci_joint_map, old_ci_joint_map;
            Eigen::VectorXd q_new, q_old;
            _ci_model->getJointPosition(ci_joint_map);
            _ci_model->getJointPosition(old_ci_joint_map);

            // replace optimized joint positions in the full q
            for (auto pair : joint_map)
                ci_joint_map[pair.first] = pair.second;

            // set references
            _ci_model->mapToEigen(ci_joint_map, q_new);
            _ci_model->mapToEigen(old_ci_joint_map, q_old);
            auto task = _ci->getTask("Postural");
            auto postural = std::dynamic_pointer_cast<XBot::Cartesian::PosturalTask>(task);
            postural->setReferencePosture(q_new);

            // compute
            if (!_ci->update(_time, _ctrl_interpolation_time))
                std::cout << "unable to solve" << std::endl;

            // integrate
            Eigen::VectorXd ci_qdot, ci_q;
            _ci_model->getJointVelocity(ci_qdot);
            q_old += ci_qdot * _ctrl_interpolation_time;
            _ci_model->eigenToMap(q_old, ci_joint_map);

            // set velodyne_joint position
            ci_joint_map["velodyne_joint"] = velodyne_joint_pos;

            XBot::JointNameMap joint_vel_map;
            _ci_model->eigenToMap(ci_qdot, joint_vel_map);

            // move
            if (_robot)
            {
                _robot->setPositionReference(ci_joint_map);
                _robot->setVelocityReference(joint_vel_map);
                _robot->move();
            }
            else
            {
                _ci_model->setJointPosition(ci_joint_map);
                _ci_model->update();
            }
            _time += _ctrl_interpolation_time;
            q_fb_old = q_old;
        }
    }
    else
    {
        std_msgs::Int16 msg;
        msg.data = trajectory_index;
        _trj_index_pub.publish(msg);
    }
}
