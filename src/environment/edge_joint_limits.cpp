#include <environment/edge_joint_limits.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeJointLimits::EdgeJointLimits(XBot::ModelInterface::Ptr model):
BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos>(),
_model(model)
{}

void EdgeJointLimits::resize()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    setDimension(v1->estimateDimension());
}

void EdgeJointLimits::computeError()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    Eigen::VectorXd min(v1->estimateDimension()), max(v1->estimateDimension());
    _model->getJointLimits(min, max);
   _error.setZero(v1->estimateDimension());

    double eps = 0.1;
    double S = 3;
    int n = 2;

    for (int i = 0; i < v1->estimateDimension(); i++)
    {
        double value = 1/exp(S*v1->estimate()(i) - min(i)) + exp(S*v1->estimate()(i) - max(i));
        _error(i) = value;
    }
}
