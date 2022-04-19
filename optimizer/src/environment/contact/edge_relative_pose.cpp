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

    auto diff = v2->estimate().state.pose.translation() - v1->estimate().state.pose.translation();
//    _error[0] = std::pow(std::sqrt(std::pow(diff(0), 2)) - _step_size, 2);
//    _error[1] = std::pow(std::sqrt(std::pow(diff(1), 2)) - 0.2, 2);
//    _error[0] = diff(0) + _step_size;
//    _error[1] = std::sqrt(diff(1)*diff(1)) - 0.2;
//    _error[2] = 0.0;

    double S = 2, Sc = 100;
//    diff = (q2/0.1 - q1/0.1);
//    _vel = diff;

    _error[0] = 1/exp(Sc*diff(0) - Sc*0.1) + exp(Sc*diff(0) - Sc*0.3);
    if (v1->estimate().getDistalLink() == "l_sole")
        _error[1] = 1/exp(Sc*diff(1) + Sc*0.3) + exp(Sc*diff(1) + Sc*0.1);
    else
        _error[1] = 1/exp(Sc*diff(1) - Sc*0.1) + exp(Sc*diff(1) - Sc*0.3);
    _error[2] = diff(2) * diff(2);

//    if (_error[1] > 0.001)
//    {
//        std::cout << "v1 (id " << v1->id() << "  distal_link " << v1->estimate().getDistalLink() << "): " << v1->estimate().state.pose.translation().transpose() << std::endl;
//        std::cout << "v2 (id " << v2->id() << "  distal_link " << v2->estimate().getDistalLink() << "): " << v2->estimate().state.pose.translation().transpose() << std::endl;
//        std::cout << "diff: " << diff.transpose() << "  error: " << _error.transpose() << std::endl;
//    }
}
