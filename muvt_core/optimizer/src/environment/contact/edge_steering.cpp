#include <muvt/environment/contact/edge_steering.h>

using namespace Muvt::HyperGraph;
using namespace g2o;

EdgeSteering::EdgeSteering():
BaseUnaryEdge<3, Eigen::Vector3d, VertexContact>(),
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

    double angle = std::atan2(dir(1), dir(0));
//    double angle = 0.7;

//    _error[0] = std::pow(v->estimate().state.pose.linear().eulerAngles(0, 1, 2)[0], 2);
//    _error[1] = std::pow(v->estimate().state.pose.linear().eulerAngles(0, 1, 2)[1], 2);
    _error.setZero();
    _error[2] = std::pow(v->estimate().state.pose.linear().eulerAngles(0, 1, 2)[2] - angle, 2);
//    std::cout << "val: " << v->estimate().state.pose.linear().eulerAngles(0, 1, 2)[2] << std::endl;
//    std::cout << "angle: " << angle << std::endl;
//    std::cout << "error: " << _error.transpose() << std::endl;

}

