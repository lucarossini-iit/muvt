#ifndef EDGE_TASK_H
#define EDGE_TASK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <g2o/core/base_unary_edge.h>

#include <muvt_core/environment/contact/vertex_contact.h>

namespace Muvt { namespace HyperGraph {

class EdgeTask : public BaseUnaryEdge<3, Eigen::Vector3d, VertexContact> {
public:
    EdgeTask();

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        return os.good();
    }

    void setReference(const Eigen::Vector3d reference);

    Eigen::Vector3d getReference() const { return _reference; }

    void computeError();

private:
    Eigen::Vector3d _reference;

};
} }

#endif // EDGE_TASK_H
