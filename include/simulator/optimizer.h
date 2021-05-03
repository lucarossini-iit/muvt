#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ros/ros.h>
#include <ros/service.h>

#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <g2o/config.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/core/optimization_algorithm_factory.h"
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include <simulator/simulator.h>
#include <environment/edge_xyz.h>

#include <teb_test/SetObstacle.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>

namespace XBot { namespace HyperGraph {

class Optimizer {
public:
    Optimizer();
    
    void publish();
    
    void run();

private:
    void init_load_simulator();
    void init_load_optimizer();
    void load_vertices();
    void load_edges();
    
    bool create_obstacle_service(teb_test::SetObstacle::Request& req, teb_test::SetObstacle::Response& res);

//    void interactive_markers_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    

    ros::NodeHandle _nhpr, _nh;
    ros::ServiceServer _create_obs_srv;
    ros::Publisher _trj_pub, _obs_pub;
    
    std::vector<Eigen::Vector3d> _obstacles;
    std::vector<Eigen::Vector3d> _trajectory;

    Simulator::Ptr _simulator;

    g2o::SparseOptimizer _optimizer;

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
};
} }

#endif // OPTIMIZER_H
