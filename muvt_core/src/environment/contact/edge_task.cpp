#include <muvt_core/environment/contact/edge_task.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

EdgeTask::EdgeTask():
BaseUnaryEdge<3, Eigen::Vector3d, VertexContact>()
{}

void EdgeTask::setReference(const Eigen::Vector3d reference)
{
    _reference = reference;
}

void EdgeTask::computeError()
{
    auto v = dynamic_cast<VertexContact*>(_vertices[0]);

    Eigen::Vector3d diff = v->estimate().state.pose.translation() - _reference;
    _error = diff.array().square();
}

