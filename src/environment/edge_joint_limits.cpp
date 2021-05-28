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

    _error.conservativeResize(v1->estimateDimension());
    _measurement.conservativeResize(v1->estimateDimension());
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

//    for (int i = 0; i < v1->estimateDimension(); i++)
//    {
//        if (v1->estimate()(i) > max(i) - eps)
//        {
//            double value = pow((-v1->estimate()(i)-(-max(i)-eps))/S, n);
//            _error(i) = value;
//        }
//        else
//            _error(i) = 0;
//    }

//    for (int i = 0; i < v1->estimateDimension(); i++)
//    {
//        if (v1->estimate()(i) < min(i) + eps)
//        {
//           double value = pow((v1->estimate()(i)-(-min(i)-eps))/S, n);
//           _error(i) += value;
//        }
//        else
//            _error(i) += 0;
//    }
}
