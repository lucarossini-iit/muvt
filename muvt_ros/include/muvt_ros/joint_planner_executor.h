#ifndef OPTIMIZER_JOINT_H
#define OPTIMIZER_JOINT_H

#include <ros/ros.h>

// XBot and CartesI/O
#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

// Muvt
#include <muvt_core/optimizer/optimizer_joint.h>

namespace Muvt {
class JointPlannerExecutor {
public:
    JointPlannerExecutor();

    void run();

private:
    void init_load_model();

    ros::NodeHandle _nh, _nhpr;
    XBot::ModelInterface::Ptr _model;

    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;

    HyperGraph::OptimizerJoint _optimizer;
};
}
#endif // OPTIMIZER_JOINT_H
