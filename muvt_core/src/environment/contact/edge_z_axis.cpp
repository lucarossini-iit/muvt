#include <muvt_core/environment/contact/edge_z_axis.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

EdgeZAxis::EdgeZAxis():
BaseUnaryEdge<1, double, VertexContact>()
{}

void EdgeZAxis::setReference(const double reference)
{
    _reference = reference;
}

void EdgeZAxis::computeError()
{
    auto v = dynamic_cast<VertexContact*>(_vertices[0]);

    double diff = v->estimate().state.pose.translation()(2) - _reference;
    _error(0) = std::pow(diff, 2);
}

