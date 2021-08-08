#ifndef EDGE_TASK_H
#define EDGE_TASK_H

#include <g2o/core/base_unary_edge.h>

#include <environment/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgeTask : public BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos> {
public:
    EdgeTask();

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        return true;
    }

    void setReference(Eigen::VectorXd ref);

    void resize();

    Eigen::VectorXd getError() const;

    Eigen::VectorXd getReference() const;

    void computeError();

private:
    Eigen::VectorXd _ref;
};

} }
#endif // EDGE_TASK_H
