#ifndef EDGE_RELATIVE_POSE_H
#define EDGE_RELATIVE_POSE_H

#include <g2o/core/base_binary_edge.h>

#include <environment/contact/vertex_contact.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

// At the moment, we consider the bipedal case. An extension to a multi-contact case can be done using
// a BaseMultiEdge

class EdgeRelativePose : public BaseBinaryEdge<3, Eigen::Vector3d, VertexContact, VertexContact> {
public:
    EdgeRelativePose();

    bool read(std::istream& is)
    {
        Eigen::VectorXd meas;
        internal::readVector(is, meas);
        setMeasurement(meas);

        return is.good() || is.eof();
    }

    bool write(std::ostream& os) const
    {
        Eigen::VectorXd p = measurement();
        os << p;

        return os.good();
    }

    bool checkVertices();

    void computeError();
};
} }

#endif // EDGE_RELATIVE_POSE_H
