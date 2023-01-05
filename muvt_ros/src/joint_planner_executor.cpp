#include <muvt_ros/joint_planner_executor.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

using namespace Muvt;

JointPlannerExecutor::JointPlannerExecutor():
_nh(""),
_nhpr("~"),
_optimizer()
{
    init_load_model();
}

void JointPlannerExecutor::run()
{
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

