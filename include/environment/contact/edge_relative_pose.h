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
        return is.good() || is.eof();
    }

    bool write(std::ostream& os) const
    {
        return os.good();
    }

    void setStepTime(const double step_time) { _step_time = step_time; }
    double getStepTime() const { return _step_time; }

    void setStepSize(const double step_size) { _step_size = step_size; }
    double getStepSize() const { return _step_size; }

    bool checkVertices();

    void computeError();

private:
    double _step_time;
    double _step_size;
};
} }

#endif // EDGE_RELATIVE_POSE_H
