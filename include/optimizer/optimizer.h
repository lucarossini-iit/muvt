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
#include <environment/edge_task.h>

#include <teb_test/SetObstacle.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>

namespace XBot { namespace HyperGraph {

class Optimizer {
public:
    Optimizer(ModelInterface::Ptr model);

private:
    ModelInterface::Ptr _model;
};

}}

#endif // OPTIMIZER_H
