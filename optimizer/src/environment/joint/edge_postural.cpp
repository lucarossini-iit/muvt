#include <environment/joint/edge_postural.h>

using namespace Muvt::HyperGraph;
using namespace g2o;

EdgePostural::EdgePostural(Muvt::ModelInterface::Ptr model):
UnaryEdge(model)
{}

void EdgePostural::setReference(Eigen::VectorXd ref)
{
    _ref = ref;
}

bool EdgePostural::resize()
{
    auto v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    if (v1 == nullptr)
        std::runtime_error("first set vertices, then resize!");

    setDimension(v1->estimateDimension());
    return true;
}

Eigen::VectorXd EdgePostural::getError() const
{
    return _error;
}

Eigen::VectorXd EdgePostural::getReference() const
{
    return _ref;
}

void EdgePostural::computeError()
{
    auto v = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    Eigen::VectorXd diff = _ref - v->estimate();

    _error = diff;
    _error.head(6) = Eigen::Vector6d::Zero();
}
