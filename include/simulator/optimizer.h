#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ros/ros.h>
#include <ros/service.h>
#include <string.h>
#include <chrono>
#include <thread>

#include <eigen_conversions/eigen_msg.h>

#include <g2o/config.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/core/optimization_algorithm_factory.h"
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <simulator/simulator.h>
#include <environment/edge_xyz.h>
#include <environment/robot_pos.h>
#include <environment/vertex_robot_pos.h>
#include <environment/edge_robot_pos.h>
#include <environment/edge_robot_vel.h>
#include <environment/edge_joint_limits.h>

#include <teb_test/SetObstacle.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>

namespace XBot { namespace HyperGraph {

class Optimizer {
public:
    Optimizer();
    
    void publish();
    
    void run();

private:
    void init_load_model();
    void init_load_simulator();
    void init_load_optimizer();
    void load_vertices();
    void load_edges();
    void add_edges(int index);
    void update_edges(int index);
    void clear_edges();
    void optimize();
    
    void publishCartesianReferences(int index);
    
    bool create_obstacle_service(teb_test::SetObstacle::Request& req, teb_test::SetObstacle::Response& res);
    bool optimization_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void interactive_markers_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    

    ros::NodeHandle _nhpr, _nh;
    ros::ServiceServer _create_obs_srv, _opt_srv;
    ros::Publisher _trj_pub, _obs_pub, _ref_pub;
    
    XBot::ModelInterface::Ptr _model, _sol_model;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
    
    std::vector<Eigen::Vector3d> _obstacles;
    std::vector<Eigen::Vector3d> _trajectory;

    Simulator::Ptr _simulator;

    g2o::SparseOptimizer _optimizer;

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
    
    int _index, _incr;
    
    Eigen::VectorXd _q_old_sol;
};
} }

#endif // OPTIMIZER_H
