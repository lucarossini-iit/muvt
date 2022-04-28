#ifndef MUVT_CORE_EDGE_MULTI_RELATIVE_POSES_H
#define MUVT_CORE_EDGE_MULTI_RELATIVE_POSES_H

#include <g2o/core/base_multi_edge.h>

#include <muvt_core/environment/contact/vertex_contact.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

struct DistanceLimits {
    Eigen::Vector3d lower_limits;
    Eigen::Vector3d upper_limits;
};


// At the moment, we consider the bipedal case. An extension to a multi-contact case can be done using
// a BaseMultiEdge

class EdgeMultiRelativePoses : public BaseMultiEdge<-1, Eigen::VectorXd> {
public:
    EdgeMultiRelativePoses(unsigned int n_contacts);

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

    bool setLimits(const std::string distal_link, const Eigen::Vector3d lower, const Eigen::Vector3d upper);

    virtual void computeError();
//    void linearizeOplus() { _jacobianOplusXi = Eigen::MatrixXd::Identity(3,3); _jacobianOplusXj = Eigen::MatrixXd::Identity(3,3); }

private:
    double _step_time;
    double _step_size;
    std::map<std::string, DistanceLimits> _limits;
    unsigned int _n_contacts;
};
} }

#endif // MUVT_CORE_EDGE_MULTI_RELATIVE_POSES_H
