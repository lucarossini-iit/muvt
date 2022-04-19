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

    auto diff = v1->estimate().state.pose.translation() - v2->estimate().state.pose.translation();
//    _error[0] = std::pow(std::sqrt(std::pow(diff(0), 2)) - _step_size, 2);
//    _error[1] = std::pow(std::sqrt(std::pow(diff(1), 2)) - 0.2, 2);
    _error[0] = diff(0) + _step_size;
    _error[1] = std::sqrt(diff(1)*diff(1)) - 0.2;
    _error[2] = 0.0;

    if (true)
    {
        std::cout << "v1 (id " << v1->id() << "): " << v1->estimate().state.pose.translation().transpose() << std::endl;
        std::cout << "v2 (id " << v2->id() << "): " << v2->estimate().state.pose.translation().transpose() << std::endl;
        std::cout << "error: " << _error.transpose() << std::endl;
    }
}
