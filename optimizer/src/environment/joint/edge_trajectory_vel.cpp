#include <environment/joint/edge_trajectory_vel.h>

using namespace XBot::HyperGraph;

EdgeTrajectoryVel::EdgeTrajectoryVel(XBot::ModelInterface::Ptr model):
UnaryEdge(model)
{}

void EdgeTrajectoryVel::setRef(const Eigen::VectorXd ref)
{
    _ref = ref;
}

void EdgeTrajectoryVel::resize()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    if (v1 == nullptr)
        std::runtime_error("first set vertices, then resize!");

    setDimension(v1->estimateDimension());
}

Eigen::VectorXd EdgeTrajectoryVel::getError() const
{
    return _error;
}

void EdgeTrajectoryVel::computeError()
{
    _error.setZero();
    const VertexRobotPos* v = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    auto diff = (v->estimate() - _ref)/0.01;
    double S = 2;

    for (int i = 0; i < diff.size(); i++)
    {
        double value = pow(diff(i), S);
        _error(i) = value;
    }
}

