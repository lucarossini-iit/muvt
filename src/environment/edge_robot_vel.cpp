#include <environment/edge_robot_vel.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeRobotVel::EdgeRobotVel(XBot::ModelInterface::Ptr model):
BaseBinaryEdge<-1, Eigen::VectorXd, VertexRobotPos, VertexRobotPos>(),
_model(model)
{
    _model->getVelocityLimits(_vel_max);
    _vel_max = _vel_max;
    _vel_min = -_vel_max;
}

void EdgeRobotVel::resize()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    if (v1 == nullptr)
        std::runtime_error("first set vertices, then resize!");

    setDimension(v1->estimateDimension());
}

void EdgeRobotVel::computeError()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);
    auto v2 = dynamic_cast<const VertexRobotPos*>(_vertices[1]);

    Eigen::VectorXd q1(v1->estimateDimension()), q2(v1->estimateDimension()), diff(v1->estimateDimension());
    q1 = v1->estimate();
    q2 = v2->estimate();
   _error.setZero(v1->estimateDimension());

    double eps = 0.005;
    double S = 3;
    double r = 0.2;
    int n = 2;
    diff = (q2/0.01 - q1/0.01);

    for (int i = 0; i < q1.size(); i++)
    {
        double value = 1/exp(S*diff(i) - _vel_min(i)) + exp(S*diff(i) - _vel_max(i));
        _error(i) = value;
    }
}

EdgeRobotUnaryVel::EdgeRobotUnaryVel(XBot::ModelInterface::Ptr model):
BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos>(),
_model(model)
{}

void EdgeRobotUnaryVel::resize()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    if (v1 == nullptr)
        std::runtime_error("first set vertices, then resize!");

    _error.conservativeResize(v1->estimateDimension());
    _measurement.conservativeResize(v1->estimateDimension());
}

void EdgeRobotUnaryVel::setRef(Eigen::VectorXd ref)
{
    _ref = ref;
}

void EdgeRobotUnaryVel::getRef(Eigen::VectorXd ref) const
{
    ref = _ref;
}

void EdgeRobotUnaryVel::computeError()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    Eigen::VectorXd q1(v1->estimateDimension()), diff(v1->estimateDimension());
    Eigen::VectorXd vel_max, vel_min;
    _model->getVelocityLimits(vel_max);
    vel_min = -vel_max;
    q1 = v1->estimate();
   _error.setZero(v1->estimateDimension());

    double eps = 0.005;
    double S = 0.05;
    double r = 0.2;
    int n = 2;
    diff = (_ref/0.01 - q1/0.01);

    for (int i = 0; i < q1.size(); i++)
    {
        if (diff(i) > vel_max(i) - eps)
        {
            double value = pow((-diff(i)-(-vel_max(i)-eps))/S, n);
            _error(i) = value;
        }
        else
            _error(i) = 0;
    }

    for (int i = 0; i < q1.size(); i++)
    {
        if (diff(i) < vel_min(i) + eps)
        {
           double value = pow((-diff(i)-(-vel_min(i)-eps))/S, n);
           _error(i) += value;
        }
        else
            _error(i) += 0;
    }
}

