#include <muvt/environment/contact/edge_multi_relative_poses.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

EdgeMultiRelativePoses::EdgeMultiRelativePoses(unsigned int n_edges):
BaseMultiEdge<12, std::vector<Eigen::Vector3d>>(),_n_edges(n_edges)
{
  _vertices.resize(_n_edges);
}

bool EdgeMultiRelativePoses::checkVertices()
{
    //auto v1 = dynamic_cast<VertexContact*>(_vertices[0]);
    //auto v2 = dynamic_cast<VertexContact*>(_vertices[1]);
    //
    //// the two contact must occur sequentially ( CRAWL )
    //if (v2->estimate().state.time != v1->estimate().state.time - _step_time && v2->estimate().state.time != v1->estimate().state.time + _step_time)
    //{
    //    std::cout << "step_time: " << _step_time << std::endl;
    //    std::cout << "vertex " << v1->id() << " and " << v2->id() << " are not adjacent contacts" << std::endl;
    //    return false;
    //}
    //
    //// check if the two vertices refer to the same distal link
    //if (v1->estimate().getDistalLink() == v2->estimate().getDistalLink())
    //{
    //    std::cout << "step_time: " << _step_time << std::endl;
    //    std::cout << "vertex " << v1->id() << " and " << v2->id() << " refer to the same distal link" << std::endl;
    //    return false;
    //}

    return true;
}

void EdgeMultiRelativePoses::computeError()
{
    for(unsigned int i = 0; i<_n_edges-1; i++)
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
