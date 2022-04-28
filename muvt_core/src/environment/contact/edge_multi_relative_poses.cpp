#include <muvt_core/environment/contact/edge_multi_relative_poses.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

EdgeMultiRelativePoses::EdgeMultiRelativePoses(unsigned int n_contacts):
BaseMultiEdge<-1, Eigen::VectorXd>(),
_n_contacts(n_contacts)
{
  _vertices.resize(_n_contacts);
  _error.resize(3 * (_n_contacts - 1));
  _l_limits.resize(3 * (_n_contacts - 1));
  _u_limits.resize(3 * (_n_contacts - 1));
}

bool EdgeMultiRelativePoses::setLimits(const Eigen::VectorXd l_limits, const Eigen::VectorXd u_limits)
{
    if (l_limits.size() != _l_limits.size())
    {
        throw std::runtime_error("Error while initializing EdgeMultiRelativePoses: lower limits vector size is wrong!");
        return false;
    }
    if (u_limits.size() != _u_limits.size())
    {
        throw std::runtime_error("Error while initializing EdgeMultiRelativePoses: upper limits vector size is wrong!");
        return false;
    }

    _l_limits = l_limits;
    _u_limits = u_limits;
    return true;
}

void EdgeMultiRelativePoses::computeError()
{
    for(unsigned int i = 0; i < _n_contacts - 1; i++)
    {
      auto v1 = dynamic_cast<VertexContact*>(_vertices[i]);
      auto v2 = dynamic_cast<VertexContact*>(_vertices[i+1]);
      std::cout << "ComputeError" << std::endl;
      auto error3d = compute3dError(v1->estimate().state.pose.translation(),v2->estimate().state.pose.translation());
      std::cout << "ComputeError" << std::endl;
      _error(i*3) = error3d(0);
      _error((i+1)*3) = error3d(1);
      _error((i+2)*3) = error3d(2);
      std::cout << "ComputeError" << std::endl;
    }

    std::cout << "ComputeError" << std::endl;
}
