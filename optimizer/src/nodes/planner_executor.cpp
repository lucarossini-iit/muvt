#include "planner_executor.h"

using namespace XBot::HyperGraph::Planner;

PlannerExecutor::PlannerExecutor():
_nh(""),
_nhpr("~"),
_planner(),
_g2o_optimizer(),
_execute(false)
{
    // MatLogger2
    MatLogger2::Options logger_opt;
    logger_opt.default_buffer_size = 1e5;
    _logger = MatLogger2::MakeLogger("/tmp/dcm_planner_logger", logger_opt);
    _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);

    init_load_model();
    init_load_config();
    init_interactive_marker();
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

    YAML_PARSE_OPTION(config["dcm_planner"], z_com, double, 0.0);
    if (z_com == 0)
        throw std::runtime_error("missing mandatory argument 'z_com'!");
    _planner.setZCoM(z_com);


    YAML_PARSE_OPTION(config["dcm_planner"], step_time, double, 0.0);
    if (step_time == 0)
        throw std::runtime_error("missing mandatory argument 'step_time'!");
    _planner.setStepTime(step_time);
    _logger->add("step_time", step_time);

    YAML_PARSE_OPTION(config["dcm_planner"], step_size, double, 0.2);
    _planner.setStepSize(step_size);
    _logger->add("step_size", step_size);

    YAML_PARSE_OPTION(config["dcm_planner"], n_steps, double, 0.0);
    _planner.setNumSteps(n_steps);
    _logger->add("n_steps", n_steps);

    YAML_PARSE_OPTION(config["dcm_planner"], dt, double, 0.01);
    _planner.setdT(dt);
    _logger->add("dt", dt);

    Eigen::Vector3d com;
    _model->getCOM(com);
    _planner.setZCoM(com(2));
    _logger->add("z_com", _planner.getZCoM());

    // generate step sequence
    _planner.generateSteps();
    std::cout << "\033[1;32m[planner_executor] \033[0m" << "\033[32mconfigs loaded! \033[0m" << std::endl;
}

void PlannerExecutor::init_interactive_marker()
{
    _server = std::make_shared<interactive_markers::InteractiveMarkerServer>("planner_executor");

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "obstacle";
    int_marker.description = "obstacle";

    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0; m.color.a = 0.3;

    visualization_msgs::InteractiveMarkerControl obs_control;
    obs_control.always_visible = true;
    obs_control.markers.push_back(m);
    int_marker.controls.push_back(obs_control);
    int_marker.pose.position.x = 0.5;
    int_marker.pose.position.y = -1.2;
    int_marker.pose.position.z = 0.0;

    visualization_msgs::InteractiveMarkerControl move_control_x;
    move_control_x.name = "move_x";
    move_control_x.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control_x);

    visualization_msgs::InteractiveMarkerControl move_control_y;
    move_control_y.name = "move_y";
    Eigen::Matrix3d R;
    R << cos(M_PI/2), -sin(M_PI/2), 0, sin(M_PI/2), cos(M_PI/2), 0, 0, 0, 1;
    Eigen::Quaternion<double> quat(R);
    tf::quaternionEigenToMsg(quat, move_control_y.orientation);
    move_control_y.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control_y);

    visualization_msgs::InteractiveMarkerControl move_control_z;
    move_control_z.name = "move_z";
    R << cos(-M_PI/2), 0, sin(-M_PI/2), 0, 1, 0, -sin(-M_PI/2), 0, cos(-M_PI/2);
    Eigen::Quaternion<double> quat_z(R);
    tf::quaternionEigenToMsg(quat_z, move_control_z.orientation);
    move_control_z.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control_z);

    _server->insert(int_marker);
    _server->setCallback(int_marker.name, boost::bind(&PlannerExecutor::interactive_markers_feedback, this, _1));
    _server->applyChanges();
}

void PlannerExecutor::interactive_markers_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    auto c = feedback->marker_name.back();
    Eigen::Vector3d obs(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

    auto edges = _g2o_optimizer.getEdges();
    for(auto edge : edges)
    {
        if(EdgeCollision* e = dynamic_cast<EdgeCollision*>(edge))
        {
            e->setObstacles(obs);
        }
    }
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
        m_zmp.action = visualization_msgs::Marker::MODIFY;
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
        m_footstep.action = visualization_msgs::Marker::MODIFY;
        m_footstep.type = visualization_msgs::Marker::CUBE;
        m_footstep.scale.x = 0.2; m_footstep.scale.y = 0.1; m_footstep.scale.z = 0.02;
        m_footstep.color.r = 1; m_footstep.color.g = 1; m_footstep.color.b = 0; m_footstep.color.a = 0.5;
        m_footstep.pose.position.x = _footstep_seq[i].state.pose.translation().x();
        m_footstep.pose.position.y = _footstep_seq[i].state.pose.translation().y();
        m_footstep.pose.position.z = _footstep_seq[i].state.pose.translation().z();
        Eigen::Quaternion<double> q(_footstep_seq[i].state.pose.linear());
        m_footstep.pose.orientation.x = q.coeffs().x();
        m_footstep.pose.orientation.y = q.coeffs().y();
        m_footstep.pose.orientation.z = q.coeffs().z();
        m_footstep.pose.orientation.w = q.coeffs().w();
        ma_footstep.markers.push_back(m_footstep);
    }

    // publish cp
    for (int i = 0; i < _cp_trj.size(); i += 10)
    {
        visualization_msgs::Marker m_cp;
        m_cp.header.frame_id = "world";
        m_cp.header.stamp = ros::Time::now();
        m_cp.id = i;
        m_cp.action = visualization_msgs::Marker::MODIFY;
        m_cp.type = visualization_msgs::Marker::SPHERE;
        m_cp.scale.x = 0.01; m_cp.scale.y = 0.01; m_cp.scale.z = 0.01;
        m_cp.color.r = 0; m_cp.color.g = 0; m_cp.color.b = 1; m_cp.color.a = 1;
        m_cp.pose.position.x = _cp_trj[i](0);
        m_cp.pose.position.y = _cp_trj[i](1);
        m_cp.pose.position.z = _cp_trj[i](2);
        ma_cp.markers.push_back(m_cp);
    }

    // publish com
    for (int i = 0; i < _com_trj.size(); i += 10)
    {
        visualization_msgs::Marker m_com;
        m_com.header.frame_id = "world";
        m_com.header.stamp = ros::Time::now();
        m_com.id = i;
        m_com.action = visualization_msgs::Marker::MODIFY;
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

    // create g2o vertices
    std::vector<OptimizableGraph::Vertex*> g2o_vertices;
    for (int i = 0; i < _footstep_seq.size(); i++)
    {
        VertexContact* vertex = new VertexContact();
        vertex->setId(i);
        vertex->setEstimate(_footstep_seq[i]);
        if (0 == i || 1 == i ||  (_footstep_seq.size() - 1) == i || (_footstep_seq.size() - 2) == i)
            vertex->setFixed(true);
        auto v = dynamic_cast<OptimizableGraph::Vertex*>(vertex);
        g2o_vertices.push_back(v);
    }
    _g2o_optimizer.setVertices(g2o_vertices);


    std::vector<OptimizableGraph::Edge*> g2o_edges;
    for (int i = 0; i < g2o_vertices.size(); i++)
    {
        EdgeCollision* edge = new EdgeCollision();
        Eigen::MatrixXd info(1, 1);
        info.setIdentity();  info *= 100;
        edge->setInformation(info);
        edge->setObstacles(Eigen::Vector3d(0.5, -1.2, 0.0));
        edge->vertices()[0] = g2o_vertices[i];
        auto e = dynamic_cast<OptimizableGraph::Edge*>(edge);
        g2o_edges.push_back(e);
    }

    for (int i = 0; i < g2o_vertices.size() - 1; i++)
    {
        EdgeRelativePose* edge_succ = new EdgeRelativePose();
        Eigen::MatrixXd info_succ(3, 3);
        info_succ.setIdentity();  info_succ *= 100;
        edge_succ->setInformation(info_succ);
        edge_succ->setStepTime(_planner.getStepTime());
        edge_succ->setStepSize(_planner.getStepSize());
        edge_succ->vertices()[0] = g2o_vertices[i];
        edge_succ->vertices()[1] = g2o_vertices[i+1];
        edge_succ->checkVertices();
        auto e = dynamic_cast<OptimizableGraph::Edge*>(edge_succ);
        g2o_edges.push_back(e);
    }

    for (int i = 2; i < g2o_vertices.size(); i++)
    {
        EdgeSteering* edge = new EdgeSteering();
        Eigen::MatrixXd info(3, 3);
        info.setIdentity();
        edge->setInformation(info);
        edge->setPreviousContact(g2o_vertices[i-2]);
        edge->vertices()[0] = g2o_vertices[i];
        auto e = dynamic_cast<OptimizableGraph::Edge*>(edge);
        g2o_edges.push_back(e);
    }
    _g2o_optimizer.setEdges(g2o_edges);
    _g2o_optimizer.update();

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

bool PlannerExecutor::check_cp_inside_support_polygon(Eigen::Vector3d cp, Eigen::Affine3d foot_pose)
{
    // TODO: remove hardcoded foot size
    // foot limits in foot frame
    double x_max = 0.1;
    double x_min = -0.1;
    double y_max = 0.05;
    double y_min = -0.05;

    // foot limits in world frame
    double foot_x_max = foot_pose.translation().x() + (foot_pose.linear() * Eigen::Vector3d(x_max, 0, 0))(0);
    double foot_x_min = foot_pose.translation().x() + (foot_pose.linear() * Eigen::Vector3d(x_min, 0, 0))(0);
    double foot_y_max = foot_pose.translation().y() + (foot_pose.linear() * Eigen::Vector3d(0, y_max, 0))(1);
    double foot_y_min = foot_pose.translation().y() + (foot_pose.linear() * Eigen::Vector3d(0, y_min, 0))(1);

    if(cp(0) > foot_x_min && cp(0) < foot_x_max && cp(1) > foot_y_min && cp(1) < foot_y_max)
        return true;
    else
        return false;
}

void PlannerExecutor::run()
{
    // g2o local refinement
    _g2o_optimizer.solve();
    _g2o_optimizer.getFootsteps(_footstep_seq);
    _planner.setFootsteps(_footstep_seq);
    _planner.solve();
    _planner.getSolution(_footstep_seq, _cp_trj, _com_trj);

    if (_execute)
    {
        // only single stance phases with the robot oscillating between the footholds
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
                if (i > 2)
                {
                    auto task = _ci->getTask(_footstep_seq[i].getDistalLink());
                    auto c_task = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
                    Eigen::Affine3d x_ref = swing_trajectory(time, x_init, x_fin, 0, _planner.getStepTime());
                    c_task->setPoseReference(x_ref);
                }

                // first step is half og the nominal one, thus we divide also also the step time
                else if (i == 2 && time > _planner.getStepTime() / 3)
                {
                    auto task = _ci->getTask(_footstep_seq[i].getDistalLink());
                    auto c_task = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
                    Eigen::Affine3d x_ref = swing_trajectory(time, x_init, x_fin, _planner.getStepTime()/3, _planner.getStepTime());
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

                // add in logger
                _logger->add("com_ref", _com_trj[index]);
                Eigen::Vector3d com_logger;
                _model->getCOM(com_logger);
                _logger->add("com", com_logger);

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

        // double stance walk: the swing trajectory starts when the CP lays on the support polygon drawn by the single stance foot
//        for (int i = 1; i < _footstep_seq.size(); i++)
//        {
//            double time = 0;
//            Eigen::Affine3d x_fin = _footstep_seq[i+1].state.pose;
//            Eigen::Affine3d x_init;
//            _model->getPose(_footstep_seq[i+1].getDistalLink(), x_init);
//            double t_init = 0.0;
//            bool start_step = false;

//            while(time <= _planner.getStepTime())
//            {
//                // skip first step: only the com must move!
//                if (check_cp_inside_support_polygon(_cp_trj[index], _footstep_seq[i].state.pose) && start_step == false)
//                {
//                    std::cout << "cp in support polygon, start stepping..." << std::endl;
//                    start_step = true;
//                    t_init = time;
//                }
//                if(start_step && time < t_init + 0.15)
//                {
//                    auto task = _ci->getTask(_footstep_seq[i+1].getDistalLink());
//                    auto c_task = std::dynamic_pointer_cast<Cartesian::CartesianTask>(task);
//                    Eigen::Affine3d x_ref = swing_trajectory(time, x_init, x_fin, t_init, t_init + 0.15);
//                    c_task->setPoseReference(x_ref);
//                }

//                _ci->setComPositionReference(_com_trj[index]);
//                _ci->update(time, _planner.getdT());
//                Eigen::VectorXd q, dq;
//                _model->getJointPosition(q);
//                _model->getJointVelocity(dq);
//                q += dq * _planner.getdT();
//                _model->setJointPosition(q);
//                _model->update();
//                _rspub->publishTransforms(ros::Time::now(), "ci");

//                // add in logger
//                _logger->add("com_ref", _com_trj[index]);
//                Eigen::Vector3d com_logger;
//                _model->getCOM(com_logger);
//                _logger->add("com", com_logger);

//                if (_robot)
//                {
//                    XBot::JointNameMap jmap;
//                    _model->eigenToMap(q, jmap);
//                    _robot->setPositionReference(jmap);
//                    _robot->move();
//                }

//                time += _planner.getdT();
//                index++;
//                r.sleep();
//            }
//            if (i == _footstep_seq.size() - 1)
//                _execute = false;
//        }
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
    publish_markers();
}

double linear_interpolation(double init, double fin, double tf, double ti, double t)
{
    double a = (fin - init) / (tf - ti);
    double b = init - a * ti;
    double res = a * t + b;
    return res;
}

Eigen::Affine3d PlannerExecutor::swing_trajectory(double time, Eigen::Affine3d x_init, Eigen::Affine3d x_fin, double t_init, double step_time)
{
    double h = 0.05;

    // compute z parabolic trajectory
    double z_fin = x_fin.translation().z();
    double z_init = x_init.translation().z();
    double a = -4 / pow(step_time - t_init, 2) * h;
    double b = -a * ((step_time*step_time - t_init*t_init) / (step_time - t_init));
    double c = -a * pow(t_init, 2) - b * t_init;
    double z = a * pow(time, 2) + b * time + c;

    // linear trajectory for x and y
    double x = linear_interpolation(x_init.translation().x(), x_fin.translation().x(), step_time, t_init, time);
    double y = linear_interpolation(x_init.translation().y(), x_fin.translation().y(), step_time, t_init, time);

    // quaternion linear interpolation
    Eigen::Quaternion<double> q_init(x_init.linear()), q_fin(x_fin.linear()), q_slerp;
    double normalized_time = time / step_time;
    q_slerp = q_init.slerp(normalized_time, q_fin);

    Eigen::Affine3d x_ref;
    x_ref.translation() << x, y, z;
    x_ref.linear() = q_slerp.toRotationMatrix();

    return x_ref;
}
