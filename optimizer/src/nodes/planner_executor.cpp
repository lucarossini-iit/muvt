#include "planner_executor.h"

using namespace XBot::HyperGraph::Planner;

PlannerExecutor::PlannerExecutor():
_nh(""),
_nhpr("~"),
_planner(),
_execute(false)
{
    init_load_model();
    init_load_config();
    init_load_cartesian_interface();

    // advertise topics
    _zmp_pub = _nh.advertise<visualization_msgs::MarkerArray>("zmp", 1, true);
    _cp_pub = _nh.advertise<visualization_msgs::MarkerArray>("cp", 1, true);
    _com_pub = _nh.advertise<visualization_msgs::MarkerArray>("com", 1, true);
    _footstep_pub = _nh.advertise<visualization_msgs::MarkerArray>("footstep", 1, true);

    // advertise services
    _exec_srv = _nh.advertiseService("execute_trajectory", &PlannerExecutor::execute_service, this);

    plan();
    publish_markers();
}

void PlannerExecutor::init_load_model()
{
    // ModelInterface
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();

    std::string world_frame_link;
    if(_nhpr.hasParam("world_frame_link"))
    {
        _nhpr.getParam("world_frame_link", world_frame_link);
        Eigen::Affine3d T;
        if(_model->getPose(world_frame_link,T))
        {
            ROS_INFO("Setting planner world frame in %s", world_frame_link.c_str());

            _model->setFloatingBasePose(T.inverse());
            _model->update();
        }
        else
            ROS_ERROR("world_frame_link %s does not exists, keeping original world!", world_frame_link.c_str());
    }

    // RobotInterface
    try
    {
        _robot = RobotInterface::getRobot(cfg);
        _robot->setControlMode(ControlMode::Position());
    }
    catch(std::runtime_error& e)
    {
        std::cout << "\033[1;31m[planner_executor] \033[0m" << "\033[31mRobotInterface not constructed!\033[0m" << std::endl;
    }

    // RobotStatePublisher
    _rspub = std::make_shared<Cartesian::Utils::RobotStatePublisher>(_model);
}

void PlannerExecutor::init_load_config()
{
    if(!_nhpr.hasParam("dcm_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'dcm_config' missing");
    }

    std::string optimizer_config_string;
    _nhpr.getParam("dcm_config", optimizer_config_string);

    auto config = YAML::Load(optimizer_config_string);

    YAML_PARSE_OPTION(config["dcm_planner"], z_com, double, 0);
    if (z_com == 0)
        throw std::runtime_error("missing mandatory argument 'z_com'!");
    _planner.setZCoM(z_com);

    YAML_PARSE_OPTION(config["dcm_planner"], step_time, double, 0);
    if (step_time == 0)
        throw std::runtime_error("missing mandatory argument 'T_step'!");
    _planner.setStepTime(step_time);

    YAML_PARSE_OPTION(config["dcm_planner"], step_size, double, 0.2);
    _planner.setStepSize(step_size);

    YAML_PARSE_OPTION(config["dcm_planner"], n_steps, double, 0);
    _planner.setNumSteps(n_steps);

    YAML_PARSE_OPTION(config["dcm_planner"], dt, double, 0.01);
    _planner.setdT(dt);

//    Eigen::Vector3d com;
//    _model->getCOM(com);
//    _planner.setZCoM(com(2));

    // generate step sequence
    _planner.generateSteps();
    std::cout << "\033[1;32m[planner_executor] \033[0m" << "\033[32mconfigs loaded! \033[0m" << std::endl;
}

void PlannerExecutor::init_load_cartesian_interface()
{
    std::string cartesian_stack;
    if(!_nh.getParam("cartesio_stack", cartesian_stack))
    {
        ROS_ERROR("cartesio_stack!");
        throw std::runtime_error("cartesio_stack!");
    }
    auto ik_yaml_goal = YAML::Load(cartesian_stack);

    double ci_period = _planner.getdT();
    auto ci_ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<XBot::Cartesian::Parameters>(ci_period), _model);
    auto ik_prob = XBot::Cartesian::ProblemDescription(ik_yaml_goal, ci_ctx);

    _ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                        ik_prob, ci_ctx);
}

bool PlannerExecutor::execute_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    _execute = !_execute;

    // reset
    Eigen::Affine3d T_left, T_right, T_com;
    T_left.translation().x() = 0.0;
    T_left.translation().y() = 0.1;
    T_left.translation().z() = 0.0;
    T_left.linear().setIdentity();

    T_right.translation().x() = _footstep_seq[0].state.pose.translation().x();
    T_right.translation().y() = _footstep_seq[0].state.pose.translation().y();
    T_right.translation().z() = _footstep_seq[0].state.pose.translation().z();
    T_right.linear().setIdentity();

    auto task = _ci->getTask("l_sole");
    auto task_l_foot = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
    task_l_foot->setPoseReference(T_left);

    task = _ci->getTask("r_sole");
    auto task_r_foot = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
    task_r_foot->setPoseReference(T_right);

    _ci->setComPositionReference(_com_trj[0]);
    _ci->update(0, _planner.getdT());
    Eigen::VectorXd q, dq;
    _model->getJointPosition(q);
    _model->getJointVelocity(dq);
    q += dq * _planner.getdT();
    _model->setJointPosition(q);
    _model->update();
    _rspub->publishTransforms(ros::Time::now(), "");

    std::cout << "\033[1m[planner_executor] \033[0m" << "starting execution!" << std::endl;
    return true;
}

void PlannerExecutor::publish_markers()
{
    visualization_msgs::MarkerArray ma_zmp, ma_cp, ma_footstep, ma_com;
    for (int i = 0; i < _footstep_seq.size(); i++)
    {
        // publish zmp
        visualization_msgs::Marker m_zmp;
        m_zmp.header.frame_id = "world";
        m_zmp.header.stamp = ros::Time::now();
        m_zmp.id = i;
        m_zmp.action = visualization_msgs::Marker::ADD;
        m_zmp.type = visualization_msgs::Marker::SPHERE;
        m_zmp.scale.x = 0.01; m_zmp.scale.y = 0.01; m_zmp.scale.z = 0.01;
        m_zmp.color.r = 1; m_zmp.color.g = 0; m_zmp.color.b = 0; m_zmp.color.a = 1;
        m_zmp.pose.position.x = _footstep_seq[i].state.zmp(0);
        m_zmp.pose.position.y = _footstep_seq[i].state.zmp(1);
        m_zmp.pose.position.z = _footstep_seq[i].state.zmp(2);
        ma_zmp.markers.push_back(m_zmp);

        // publish footsteps
        visualization_msgs::Marker m_footstep;
        m_footstep.header.frame_id = "world";
        m_footstep.header.stamp = ros::Time::now();
        m_footstep.id = i;
        m_footstep.action = visualization_msgs::Marker::ADD;
        m_footstep.type = visualization_msgs::Marker::CUBE;
        m_footstep.scale.x = 0.2; m_footstep.scale.y = 0.1; m_footstep.scale.z = 0.02;
        m_footstep.color.r = 1; m_footstep.color.g = 1; m_footstep.color.b = 0; m_footstep.color.a = 0.5;
        m_footstep.pose.position.x = _footstep_seq[i].state.pose.translation().x();
        m_footstep.pose.position.y = _footstep_seq[i].state.pose.translation().y();
        m_footstep.pose.position.z = _footstep_seq[i].state.pose.translation().z();
        ma_footstep.markers.push_back(m_footstep);
    }

    // publish cp
    for (int i = 0; i < _cp_trj.size(); i++)
    {
        visualization_msgs::Marker m_cp;
        m_cp.header.frame_id = "world";
        m_cp.header.stamp = ros::Time::now();
        m_cp.id = i;
        m_cp.action = visualization_msgs::Marker::ADD;
        m_cp.type = visualization_msgs::Marker::SPHERE;
        m_cp.scale.x = 0.01; m_cp.scale.y = 0.01; m_cp.scale.z = 0.01;
        m_cp.color.r = 0; m_cp.color.g = 0; m_cp.color.b = 1; m_cp.color.a = 1;
        m_cp.pose.position.x = _cp_trj[i](0);
        m_cp.pose.position.y = _cp_trj[i](1);
        m_cp.pose.position.z = _cp_trj[i](2);
        ma_cp.markers.push_back(m_cp);
    }

    // publish com
    for (int i = 0; i < _com_trj.size(); i++)
    {
        visualization_msgs::Marker m_com;
        m_com.header.frame_id = "world";
        m_com.header.stamp = ros::Time::now();
        m_com.id = i;
        m_com.action = visualization_msgs::Marker::ADD;
        m_com.type = visualization_msgs::Marker::SPHERE;
        m_com.scale.x = 0.01; m_com.scale.y = 0.01; m_com.scale.z = 0.01;
        m_com.color.r = 1; m_com.color.g = 0; m_com.color.b = 1; m_com.color.a = 1;
        m_com.pose.position.x = _com_trj[i](0);
        m_com.pose.position.y = _com_trj[i](1);
        m_com.pose.position.z = _com_trj[i](2);
        ma_com.markers.push_back(m_com);
    }

    _zmp_pub.publish(ma_zmp);
    _cp_pub.publish(ma_cp);
    _com_pub.publish(ma_com);
    _footstep_pub.publish(ma_footstep);
}

void PlannerExecutor::plan()
{
    // clear vectors
    if(!_footstep_seq.empty())
        _footstep_seq.clear();
    if(!_com_trj.empty())
        _com_trj.clear();
    if(!_cp_trj.empty())
        _cp_trj.clear();

    // solve and publish the solution
    _planner.solve();
    _planner.getSolution(_footstep_seq, _cp_trj, _com_trj);

    // IK
    // first trial: let's fix the left foot in the homing position and the right foot on the first position of the footstep sequence
    Eigen::Affine3d T_left, T_right, T_com;
    T_left.translation().x() = 0.0;
    T_left.translation().y() = 0.1;
    T_left.translation().z() = 0.0;
    T_left.linear().setIdentity();

    T_right.translation().x() = _footstep_seq[0].state.pose.translation().x();
    T_right.translation().y() = _footstep_seq[0].state.pose.translation().y();
    T_right.translation().z() = _footstep_seq[0].state.pose.translation().z();
    T_right.linear().setIdentity();

    auto task = _ci->getTask("l_sole");
    auto task_l_foot = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
    task_l_foot->setPoseReference(T_left);

    task = _ci->getTask("r_sole");
    auto task_r_foot = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
    task_r_foot->setPoseReference(T_right);

    _ci->setComPositionReference(_com_trj[0]);

}

void PlannerExecutor::run()
{
    if (_execute)
    {
        ros::Rate r(100);
        int index = 0;
        for(int i = 1; i < _footstep_seq.size(); i++)
        {
            double time = 0;
            Eigen::Affine3d x_fin = _footstep_seq[i].state.pose;
            Eigen::Affine3d x_init;
            _model->getPose(_footstep_seq[i].getDistalLink(), x_init);
            while(time <= _planner.getStepTime())
            {
                // skip first step: only the com must move!
                if (i > 1)
                {
                    auto task = _ci->getTask(_footstep_seq[i].getDistalLink());
                    auto c_task = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
                    Eigen::Affine3d x_ref = swing_trajectory(time, x_init, x_fin);
                    c_task->setPoseReference(x_ref);
                }

                _ci->setComPositionReference(_com_trj[index]);
                _ci->update(time, _planner.getdT());
                Eigen::VectorXd q, dq;
                _model->getJointPosition(q);
                _model->getJointVelocity(dq);
                q += dq * _planner.getdT();
                _model->setJointPosition(q);
                _model->update();
                _rspub->publishTransforms(ros::Time::now(), "ci");

                if (_robot)
                {
                    XBot::JointNameMap jmap;
                    _model->eigenToMap(q, jmap);
                    _robot->setPositionReference(jmap);
                    _robot->move();
                }

                time += _planner.getdT();
                index++;
                r.sleep();
            }
            if (i == _footstep_seq.size() - 1)
                _execute = false;
        }
    }
    else
    {
        _ci->update(0.0, _planner.getdT());
        Eigen::VectorXd q(_model->getJointNum()), dq(_model->getJointNum());
        _model->getJointPosition(q);
        _model->getJointVelocity(dq);
        q += dq * _planner.getdT();
        _model->setJointPosition(q);
        _model->update();
        _rspub->publishTransforms(ros::Time::now(), "ci");
    }
}

Eigen::Affine3d PlannerExecutor::swing_trajectory(double time, Eigen::Affine3d x_init, Eigen::Affine3d x_fin)
{
    double h = 0.05;
    double step_time = _planner.getStepTime();

    // compute z parabolic trajectory
    double z_fin = x_fin.translation().z();
    double z_init = x_init.translation().z();

    double a = -4 / pow(step_time, 2) * h;
    double b = -a * step_time;
    double z = a * pow(time, 2) + b * time;

    // linear trajectory for x and y
    b = x_init.translation().x();
    a = (x_fin.translation().x() - x_init.translation().x()) / step_time;
    double x = a * time + b;

    b = x_init.translation().y();
    a = (x_fin.translation().y() - x_init.translation().y()) / step_time;
    double y = a * time + b;

    Eigen::Affine3d x_ref;
    x_ref.translation() << x, y, z;
    x_ref.linear().setIdentity();

    return x_ref;
}
