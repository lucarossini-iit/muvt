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

#include <OpenSoT/utils/collision_utils.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>

#include <simulator/simulator.h>
#include <environment/edge_xyz.h>
#include <environment/robot_pos.h>
#include <environment/vertex_robot_pos.h>
#include <environment/edge_robot_pos.h>
#include <environment/edge_robot_vel.h>
#include <environment/edge_joint_limits.h>
#include <environment/edge_task.h>
#include <environment/edge_collision.h>
#include <environment/obstacle.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <teb_test/ObjectMessageString.h>
#include <teb_test/SetObstacle.h>
#include <std_srvs/Empty.h>

namespace XBot { namespace HyperGraph {

class Optimizer {
public:
    typedef std::vector<obstacle> obstacles;
    Optimizer();
    Optimizer(std::vector<Eigen::VectorXd> vertices);

    void setVertices(std::vector<Eigen::VectorXd> vertices);

    void run();

private:
    /* Macro for option parsing */
    #define YAML_PARSE_OPTION(yaml, name, type, default_value) \
    type name = default_value; \
    if(yaml[#name]) \
    { \
        type value = yaml[#name].as<type>(); \
        std::cout << "Found " #type " option '" #name "' with value = " << value << std::endl; \
        name = value; \
        } \
        else { \
        std::cout << "No option '" #name "' specified, using default" << std::endl; \
    } \
    /* End macro for option parsing */

    void init_load_model();
    void init_load_config();
    void init_optimizer();
    void init_vertices();
    void init_load_edges();

    void optimize();

    void object_callback(const teb_test::ObjectMessageStringConstPtr& msg);

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _obj_sub;
    ros::Publisher _sol_pub;

    ModelInterface::Ptr _model;

    YAML::Node _optimizer_config;

    g2o::SparseOptimizer _optimizer;
    std::vector<Eigen::VectorXd> _vertices;
    std::shared_ptr<ComputeLinksDistance> _cld;
    obstacles _obstacles;
    unsigned int _number_obs;
    int _iterations;
};

}}

#endif // OPTIMIZER_H