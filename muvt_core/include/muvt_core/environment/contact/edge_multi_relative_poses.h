#ifndef MUVT_CORE_EDGE_MULTI_RELATIVE_POSES_H
#define MUVT_CORE_EDGE_MULTI_RELATIVE_POSES_H

#include <g2o/core/base_multi_edge.h>

#include <muvt_core/environment/contact/vertex_contact.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

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

    /**
    * @brief Set the position lower and upper limits for the relative distance of each contact.
    * The limits follows the contact sequence. For example, if we have 4 contacts and _vertices[0] is the
    * second one, the limits should follow the order contact 1-3-4
    * @param the lower and upper limit vectors
    * @return true if success
    **/
    bool setLimits(const Eigen::VectorXd l_limits, const Eigen::VectorXd u_limits);

    virtual void computeError();
//    void linearizeOplus() { _jacobianOplusXi = Eigen::MatrixXd::Identity(3,3); _jacobianOplusXj = Eigen::MatrixXd::Identity(3,3); }

private:
    double _step_time;
    double _step_size;
    Eigen::VectorXd _l_limits;
    Eigen::VectorXd _u_limits;
    unsigned int _n_contacts;
};
} }

#endif // MUVT_CORE_EDGE_MULTI_RELATIVE_POSES_H
