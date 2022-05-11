#include <muvt_core/environment/joint/edge_joint_limits.h>

using namespace Muvt::HyperGraph;
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
    double S = 6;
    int n = 0.0000000001;

    for (int i = 0; i < v1->estimateDimension(); i++)
    {
        double value = (1/exp(S*v1->estimate()(i) - min(i)) + exp(S*v1->estimate()(i) - max(i)))*n;
        _error(i) = value;
    }
}

Eigen::VectorXd EdgeJointLimits::getError() const
{
    return _error;
}
