#include <environment/contact/edge_relative_pose.h>

using namespace g2o;
using namespace XBot::HyperGraph;

EdgeRelativePose::EdgeRelativePose():
BaseBinaryEdge<3, Eigen::Vector3d, VertexContact, VertexContact>()
{}

bool EdgeRelativePose::checkVertices()
{
    auto v1 = dynamic_cast<VertexContact*>(_vertices[0]);
    auto v2 = dynamic_cast<VertexContact*>(_vertices[1]);

    // check if the two contact occur at the same time
    if (v1->estimate().state.time != v2->estimate().state.time)
    {
        std::cout << "vertex " << v1->id() << " and " << v2->id() << " occur at different times" << std::endl;
        return false;
    }

    // check if the two vertices refer to the same distal link
    if (v1->estimate().getDistalLink() == v2->estimate().getDistalLink())
    {
        std::cout << "vertex " << v1->id() << " and " << v2->id() << " refer to the same distal link" << std::endl;
        return false;
    }

    return true;
}

void EdgeRelativePose::computeError()
{
    auto v1 = dynamic_cast<VertexContact*>(_vertices[0]);
    auto v2 = dynamic_cast<VertexContact*>(_vertices[1]);

    _error = v1->estimate().state.pose.translation() - v2->estimate().state.pose.translation();
}
