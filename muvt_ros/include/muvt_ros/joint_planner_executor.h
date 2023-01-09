#ifndef OPTIMIZER_JOINT_H
#define OPTIMIZER_JOINT_H

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// XBot and CartesI/O
#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

// Muvt
#include <muvt_core/optimizer/optimizer_joint.h>
#include <muvt_ros/utils.h>

// msgs and srvs
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>

namespace Muvt {

class JointPlannerExecutor {
public:
    JointPlannerExecutor();

    void run();

private:
    void init_load_model();
    void init_load_config();
    void init_interactive_marker();

    void interactive_markers_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void update_vertices_and_edges();

    ros::NodeHandle _nh, _nhpr;
    XBot::ModelInterface::Ptr _model;
    YAML::Node _config;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;

    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    HyperGraph::OptimizerJoint _optimizer;

    int _obs_counter;
};
}
#endif // OPTIMIZER_JOINT_H
