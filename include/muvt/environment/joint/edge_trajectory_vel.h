#ifndef EDGE_TRAJECTORY_VEL_H
#define EDGE_TRAJECTORY_VEL_H

#include <environment/joint/unary_edge.h>

namespace Muvt { namespace HyperGraph {

class EdgeTrajectoryVel : public UnaryEdge {
public:
    EdgeTrajectoryVel(Muvt::ModelInterface::Ptr model);

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

#endif // EDGE_TRAJECTORY_VEL_H
