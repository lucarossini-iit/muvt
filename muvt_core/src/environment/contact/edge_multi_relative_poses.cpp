#include <muvt_core/environment/contact/edge_multi_relative_poses.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

EdgeMultiRelativePoses::EdgeMultiRelativePoses(unsigned int n_contacts):
BaseMultiEdge<-1, Eigen::VectorXd>(),
_n_contacts(n_contacts)
{
  _vertices.resize(_n_contacts + 1);
  //setDimension(3 * _n_contacts);

  _dimension = 3 * _n_contacts;
  _information.resize(_dimension, _dimension);
  _error.resize(_dimension, 1);
  _measurement.resize(_dimension, 1);
}

bool EdgeMultiRelativePoses::setLimits(const std::string distal_link, const Eigen::Vector3d lower, const Eigen::Vector3d upper)
{
    DistanceLimits limits;
    limits.lower_limits = lower;
    limits.upper_limits = upper;

    _limits[distal_link] = limits;
}

void EdgeMultiRelativePoses::computeError()
{
    double Sc = 100;

    VertexContact* current_vertex = dynamic_cast<VertexContact*>(_vertices[0]);
    for (int i = 0; i < _n_contacts; i++)
    {
        VertexContact* v = dynamic_cast<VertexContact*>(_vertices[i+1]);
        Eigen::Vector3d current_p = current_vertex->estimate().state.pose.translation();
        Eigen::Vector3d p = v->estimate().state.pose.translation();
        Eigen::Vector3d diff = current_p - p;

        Eigen::Vector3d lower_limits = _limits[v->estimate().getDistalLink()].lower_limits;
        Eigen::Vector3d upper_limits = _limits[v->estimate().getDistalLink()].upper_limits;

        // error along x direction
        if (current_p(0) > p(0))
            _error(i*3 + 0) = 1/exp(Sc*diff(0) - Sc*lower_limits(0)) + exp(Sc*diff(0) - Sc*upper_limits(0));
        else
            _error(i*3 + 0) = 1/exp(Sc*diff(0) + Sc*upper_limits(0)) + exp(Sc*diff(0) - Sc*lower_limits(0));

        if (current_p(1) > p(1))
            _error(i*3 + 1) = 1/exp(Sc*diff(1) - Sc*lower_limits(1)) + exp(Sc*diff(1) - Sc*upper_limits(1));
        else
            _error(i*3 + 1) = 1/exp(Sc*diff(1) + Sc*upper_limits(1)) + exp(Sc*diff(1) - Sc*lower_limits(1));

        if (current_p(2) > p(2))
            _error(i*3 + 2) = 1/exp(Sc*diff(2) - Sc*lower_limits(2)) + exp(Sc*diff(2) - Sc*upper_limits(2));
        else
            _error(i*3 + 2) = 1/exp(Sc*diff(2) + Sc*upper_limits(2)) + exp(Sc*diff(2) - Sc*lower_limits(2));
    }
}
