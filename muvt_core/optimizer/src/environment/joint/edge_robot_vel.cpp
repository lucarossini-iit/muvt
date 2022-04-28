#include <environment/joint/edge_robot_vel.h>

using namespace Muvt::HyperGraph;
using namespace g2o;

EdgeRobotVel::EdgeRobotVel(Muvt::ModelInterface::Ptr model):
BaseBinaryEdge<-1, Eigen::VectorXd, VertexRobotPos, VertexRobotPos>(),
_model(model)
{
    _model->getVelocityLimits(_vel_max);
//    _vel_max /= 5;
    double vel_max_fb_tr = 0.5;
    double vel_max_fb_or = 0.1;
    _vel_max(0) = vel_max_fb_tr; _vel_max(1) = vel_max_fb_tr; _vel_max(2) = vel_max_fb_tr; _vel_max(3) = vel_max_fb_or; _vel_max(4) = vel_max_fb_or; _vel_max(5) = vel_max_fb_or;
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

    double S = 2, Sc = 10;
    diff = (q2/0.1 - q1/0.1);
    _vel = diff;

    for (int i = 0; i < q1.size(); i++)
    {
        double value = diff(i); // + 1/pow(exp(diff(i) - _vel_min(i)), Sc) + pow(exp(diff(i) - _vel_max(i)), Sc) ;
        _error(i) = value;
    }
}

Eigen::VectorXd EdgeRobotVel::getError() const
{
    return _error;
}

Eigen::VectorXd EdgeRobotVel::getVelocities() const
{
    return _vel;
}

EdgeRobotUnaryVel::EdgeRobotUnaryVel(Muvt::ModelInterface::Ptr model):
BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos>(),
_model(model)
{}

void EdgeRobotUnaryVel::resize()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    if (v1 == nullptr)
        std::runtime_error("first set vertices, then resize!");

    setDimension(v1->estimateDimension());
    _ref.resize(v1->estimateDimension());
}

void EdgeRobotUnaryVel::setRef(Eigen::VectorXd ref)
{
    _ref = ref;
}

void EdgeRobotUnaryVel::getRef(Eigen::VectorXd ref) const
{
    ref = _ref;
}

Eigen::VectorXd EdgeRobotUnaryVel::getError() const
{
    return _error;
}

void EdgeRobotUnaryVel::computeError()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    Eigen::VectorXd q1(v1->estimateDimension()), diff(v1->estimateDimension());
    Eigen::VectorXd vel_max, vel_min;
    _model->getVelocityLimits(vel_max);
    q1 = v1->estimate();
   _error.setZero(v1->estimateDimension());

    double eps = 0.005;
    double S = 0.05;
    double r = 0.2;
    int n = 2;
    diff = (_ref/0.05 - q1/0.05);

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


