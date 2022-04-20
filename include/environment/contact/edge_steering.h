#ifndef EDGE_STEERING_H
#define EDGE_STEERING_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/optimizable_graph.h>

#include <environment/contact/vertex_contact.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgeSteering : public BaseUnaryEdge<1, double, VertexContact> {
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

#endif // EDGE_STEERING_H
