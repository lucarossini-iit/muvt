#include <muvt/environment/contact/edge_relative_pose.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

EdgeRelativePose::EdgeRelativePose():
BaseBinaryEdge<3, Eigen::Vector3d, VertexContact, VertexContact>()
{}

bool EdgeRelativePose::checkVertices()
{
    auto v1 = dynamic_cast<VertexContact*>(_vertices[0]);
    auto v2 = dynamic_cast<VertexContact*>(_vertices[1]);

    // the two contact must occur sequentially
    if (v2->estimate().state.time != v1->estimate().state.time - _step_time && v2->estimate().state.time != v1->estimate().state.time + _step_time)
    {
        std::cout << "step_time: " << _step_time << std::endl;
        std::cout << "vertex " << v1->id() << " and " << v2->id() << " are not adjacent contacts" << std::endl;
        return false;
    }

    // check if the two vertices refer to the same distal link
    if (v1->estimate().getDistalLink() == v2->estimate().getDistalLink())
    {
        std::cout << "step_time: " << _step_time << std::endl;
        std::cout << "vertex " << v1->id() << " and " << v2->id() << " refer to the same distal link" << std::endl;
        return false;
    }

    return true;
}

void EdgeRelativePose::computeError()
{
    auto v1 = dynamic_cast<VertexContact*>(_vertices[0]);
    auto v2 = dynamic_cast<VertexContact*>(_vertices[1]);

    auto diff = v2->estimate().state.pose.translation() - v1->estimate().state.pose.translation();

    double S = 2, Sc = 100;

    _error[0] = 1/exp(Sc*diff(0) - Sc*0.1) + exp(Sc*diff(0) - Sc*0.3);
    if (v1->estimate().getDistalLink() == "l_sole")
        _error[1] = 1/exp(Sc*diff(1) + Sc*0.3) + exp(Sc*diff(1) + Sc*0.15);
    else
        _error[1] = 1/exp(Sc*diff(1) - Sc*0.15) + exp(Sc*diff(1) - Sc*0.3);
    _error[2] = diff(2) * diff(2);
}
