#ifndef EDGE_MULTI_RELATIVE_POSES_H
#define EDGE_MULTI_RELATIVE_POSES_H

#include <g2o/core/base_multi_edge.h>

#include <muvt/environment/contact/vertex_contact.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

// At the moment, we consider the bipedal case. An extension to a multi-contact case can be done using
// a BaseMultiEdge

class EdgeMultiRelativePoses : public BaseMultiEdge<12, std::vector<Eigen::Vector3d>> {
public:
    EdgeMultiRelativePoses(unsigned int n_edges);

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

    virtual void computeError();
//    void linearizeOplus() { _jacobianOplusXi = Eigen::MatrixXd::Identity(3,3); _jacobianOplusXj = Eigen::MatrixXd::Identity(3,3); }

private:
    double _step_time;
    double _step_size;
    unsigned int _n_edges;
};
} }

#endif // EDGE_MULTI_RELATIVE_POSES_H
