#ifndef MUVT_CORE_EDGE_TRAJECTORY_VEL_H
#define MUVT_CORE_EDGE_TRAJECTORY_VEL_H

#include <muvt_core/environment/joint/unary_edge.h>

namespace Muvt { namespace HyperGraph {

class EdgeTrajectoryVel : public UnaryEdge {
public:
    EdgeTrajectoryVel(XBot::ModelInterface::Ptr model);

    bool read(std::istream& is){ return true; }
    bool write(std::ostream &os) const { return true; }

    void resize();

    void setRef(const Eigen::VectorXd ref);

    void computeError();
    Eigen::VectorXd getError() const;

private:
    Eigen::VectorXd _ref;
};
} }

#endif // MUVT_CORE_EDGE_TRAJECTORY_VEL_H
