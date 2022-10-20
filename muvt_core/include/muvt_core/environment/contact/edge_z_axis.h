#ifndef EDGE_Z_AXIS_H
#define EDGE_Z_AXIS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <g2o/core/base_unary_edge.h>

#include <muvt_core/environment/contact/vertex_contact.h>

namespace Muvt { namespace HyperGraph {

class EdgeZAxis : public BaseUnaryEdge<1, double, VertexContact> {
public:
    EdgeZAxis();

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        return os.good();
    }

    void setReference(const double reference);

    double getReference() const { return _reference; }

    void computeError();

private:
    double _reference;

};
} }

#endif // EDGE_Z_AXIS_H
