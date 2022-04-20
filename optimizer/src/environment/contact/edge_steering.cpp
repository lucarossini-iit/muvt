#include <environment/contact/edge_steering.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeSteering::EdgeSteering():
BaseUnaryEdge<1, double, VertexContact>(),
_prev_contact(new VertexContact)
{}

void EdgeSteering::setPreviousContact(OptimizableGraph::Vertex* contact)
{
    _prev_contact = dynamic_cast<VertexContact*>(contact);
}

void EdgeSteering::computeError()
{
    auto v = dynamic_cast<VertexContact*>(_vertices[0]);

    Eigen::Vector3d dir = v->estimate().state.pose.translation() - _prev_contact->estimate().state.pose.translation();
    dir.normalize();

//    double angle = std::atan2(dir(1), dir(0));
    double angle = 0.7;

    _error[0] = std::pow(v->estimate().state.pose.linear().eulerAngles(0, 1, 2)[2] - angle, 2);

    if (_error[0] > 0.01)
    {
        std::cout << "vertex " << v->id() << ": " << v->estimate().state.pose.translation().transpose() << std::endl;
        std::cout << "dir: " << dir.transpose() << std::endl;
        std::cout << "angle: " << angle << std::endl;
        std::cout << "error: " << _error << std::endl;
    }
}

