#include <muvt_ros/joint_planner_executor.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <muvt_core/environment/joint/vertex_robot_pos.h>
#include <muvt_core/environment/joint/edge_joint_limits.h>
#include <muvt_core/environment/joint/edge_collision.h>

#include <thread>
#include <chrono>

using namespace Muvt;

JointPlannerExecutor::JointPlannerExecutor():
_nh(""),
_nhpr("~"),
_optimizer(),
_obs_counter(0)
{
    init_load_model();
    init_load_config();
    init_interactive_marker();

    update_vertices_and_edges();
}

void JointPlannerExecutor::run()
{
    _optimizer.solve();
    _rspub->publishTransforms(ros::Time::now(), "");
}

void JointPlannerExecutor::init_load_model()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();

    _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);
}

void JointPlannerExecutor::init_load_config()
{
  if(!_nhpr.hasParam("config"))
  {
    throw std::runtime_error(Utils::Color::kBoldRed + "[planner_executor]" + Utils::Color::kEndl + Utils::Color::kRed + "Mandatory private parameter 'config' missing" + Utils::Color::kEndl);
  }

  std::string optimizer_config_string;
  _nhpr.getParam("config", optimizer_config_string);

  _config = YAML::Load(optimizer_config_string);
  std::cout << Utils::Color::kBoldGreen << "[planner_executor]" << Utils::Color::kEndl << Utils::Color::kGreen << "configs loaded!" << Utils::Color::kEndl << std::endl;
}

void JointPlannerExecutor::init_interactive_marker()
{
  _server = std::make_shared<interactive_markers::InteractiveMarkerServer>("planner_executor");

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "obstacle" + std::to_string(_obs_counter);
  int_marker.description = "obstacle" + std::to_string(_obs_counter);
  _obs_counter++;

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
  Eigen::Matrix3d R;        std::cout << "creating ComputeLinkDistance..." << std::endl;

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
  _server->setCallback(int_marker.name, boost::bind(&JointPlannerExecutor::interactive_markers_feedback, this, _1));
  _server->applyChanges();
}

void JointPlannerExecutor::interactive_markers_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  auto c = feedback->marker_name.back();
  int index = c - '0';
  Eigen::Vector3d obs(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

  std::cout << Utils::Color::kBold << "[planner_executor]" << Utils::Color::kEndBold << " moving obstacle_" << c << "(" << feedback->marker_name << ") to " << obs.transpose() << std::endl;

  auto edges = _optimizer.getEdges();
  for(auto edge : edges)
  {
    if(HyperGraph::JointSpace::EdgeCollision* e = dynamic_cast<HyperGraph::JointSpace::EdgeCollision*>(edge))
    {
      e->updateObstacle(obs, index);
    }
  }
}

void JointPlannerExecutor::update_vertices_and_edges()
{
    /// TODO: remove hardcoded vertex generation. Maybe using a ros topic?
    Eigen::VectorXd q, q_init, q_fin;
    _model->getJointPosition(q_init);
    q_fin = q_init;
    q_fin(0) += 2.0;
    std::vector<Eigen::VectorXd> eigen_vertices;
    eigen_vertices.push_back(q_init);

    for (int i = 0; i < 20; i++)
    {
        q = q_init + (q_fin - q_init)/20 * i;
        eigen_vertices.push_back(q);
    }

    // create g2o vertices
    std::vector<OptimizableGraph::Vertex*> g2o_vertices;
    for (int i = 0; i < eigen_vertices.size(); i++)
    {
      HyperGraph::VertexRobotPos* vertex = new HyperGraph::VertexRobotPos();
      vertex->setDimension(_model->getJointNum());
      vertex->setId(i);
      vertex->setEstimate(eigen_vertices[i]);
      if (i == 0)
          vertex->setFixed(true);
      auto v = dynamic_cast<OptimizableGraph::Vertex*>(vertex);
      g2o_vertices.push_back(v);
    }
    _optimizer.setVertices(g2o_vertices);

    // Create Edges from config file
    std::vector<OptimizableGraph::Edge*> g2o_edges;
    if (!_config["constraints"])
    {
        std::cout << Utils::Color::kBoldYellow << "[planner_executor]" << Utils::Color::kEndl << Utils::Color::kYellow << " no constraints defined!" << Utils::Color::kEndl;
    }
    else
    {
        for (auto constraint : _config["constraints"])
        {
            std::string constraint_name = constraint.as<std::string>();
            for (int i = 0; i < g2o_vertices.size(); i++)
            {
                if (constraint_name == "joint_limits")
                {
                    HyperGraph::JointSpace::EdgeJointLimits* edge = new HyperGraph::JointSpace::EdgeJointLimits(_model);
                    edge->vertices()[0] = g2o_vertices[i];
                    auto e = dynamic_cast<OptimizableGraph::Edge*>(edge);
                    g2o_edges.push_back(e);
                }

                else if (constraint_name == "velocity_limits")
                {}

                else if (constraint_name == "collisions")
                {
                    HyperGraph::JointSpace::EdgeCollision* edge = new HyperGraph::JointSpace::EdgeCollision(_model, 100);
                    edge->vertices()[0] = g2o_vertices[i];
                    edge->addObstacle(Eigen::Vector3d(0.5, -1.2, 0.0), Eigen::Vector3d(0.5, 0.5, 0.5), 0);
                    auto e = dynamic_cast<OptimizableGraph::Edge*>(edge);
                    g2o_edges.push_back(e);
                }
                else
                {
                    throw std::runtime_error(Utils::Color::kBoldRed + "[planner_executor] " + Utils::Color::kEndl +
                                             Utils::Color::kRed + constraint_name + " is not a valid constraint!" + Utils::Color::kEndl);
                }
            }
        }
    }
    _optimizer.setEdges(g2o_edges);
    _optimizer.update();
}

