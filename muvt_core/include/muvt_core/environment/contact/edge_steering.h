#ifndef MUVT_CORE_EDGE_STEERING_H
#define MUVT_CORE_EDGE_STEERING_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/optimizable_graph.h>

#include <muvt_core/environment/contact/vertex_contact.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

class EdgeSteering : public BaseUnaryEdge<3, Eigen::Vector3d, VertexContact> {
public:
    EdgeSteering();

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        return os.good();
    }

    void setPreviousContact(OptimizableGraph::Vertex* contact);

    void computeError();

private:
    VertexContact* _prev_contact;
};
} }

#endif // MUVT_CORE_EDGE_STEERING_H
