#include "planner_executor.h"

using namespace Muvt::HyperGraph::Planner;

PlannerExecutor::PlannerExecutor():
_nh(""),
_nhpr("~"),
_planner(),
_g2o_optimizer()
{
    init_load_config();
    init_interactive_marker();

    // advertise topics
    _zmp_pub = _nh.advertise<visualization_msgs::MarkerArray>("zmp", 1, true);
    _cp_pub = _nh.advertise<visualization_msgs::MarkerArray>("cp", 1, true);
    _com_pub = _nh.advertise<visualization_msgs::MarkerArray>("com", 1, true);
    _footstep_pub = _nh.advertise<visualization_msgs::MarkerArray>("footstep", 1, true);

    plan();
    publish_markers();
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

    YAML_PARSE_OPTION(config["dcm_planner"], step_size, double, 0.2);
    _planner.setStepSize(step_size);

    YAML_PARSE_OPTION(config["dcm_planner"], n_steps, double, 0.0);
    _planner.setNumSteps(n_steps);

    YAML_PARSE_OPTION(config["dcm_planner"], dt, double, 0.01);
    _planner.setdT(dt);

    std::vector<std::string> contact_names;
    std::vector<Contact> contacts;
    std::vector<double> init_pose(7); // x y z qx qy qz qw
    YAML_PARSE(config["dcm_planner"], contact_names, std::vector<std::string>);
    for(unsigned int i=0; i<contact_names.size();i++)
    {
      YAML_PARSE(config["dcm_planner"][contact_names[i]], init_pose, std::vector<double>);
      Contact c(contact_names[i]);
      c.state.pose.translation() << init_pose[0], init_pose[1], init_pose[2];
      Eigen::Quaterniond q;
      q.x() = init_pose[3];
      q.y() = init_pose[4];
      q.z() = init_pose[5];
      q.w() = init_pose[6];
      c.state.pose.linear() << q.toRotationMatrix();

      contacts.push_back(c);
    }

    _planner.generateSteps(contacts);

    std::cout << "\033[1;32m[planner_executor] \033[0m" << "\033[32mconfigs loaded! \033[0m" << std::endl;
}

void PlannerExecutor::init_interactive_marker()
{
    _server = std::make_shared<interactive_markers::InteractiveMarkerServer>("planner_executor");

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
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

void PlannerExecutor::publish_markers()
{
    visualization_msgs::MarkerArray ma_zmp, ma_cp, ma_footstep, ma_com;
    for (int i = 0; i < _footstep_seq.size(); i++)
    {
        // publish zmp
        visualization_msgs::Marker m_zmp;
        m_zmp.header.frame_id = "map";
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
        m_footstep.header.frame_id = "map";
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
        m_cp.header.frame_id = "map";
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
        m_com.header.frame_id = "map";
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
}

void PlannerExecutor::run()
{
    // g2o local refinement
    _g2o_optimizer.solve();
    _g2o_optimizer.getFootsteps(_footstep_seq);

    // dcm planner solve
    _planner.setFootsteps(_footstep_seq);
    _planner.solve();
    _planner.getSolution(_footstep_seq, _cp_trj, _com_trj);

    // publish markers on rviz
    publish_markers();
}

