#ifndef EDGE_TASK_H
#define EDGE_TASK_H

#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>

#include <environment/unary_edge.h>
#include <environment/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgeTask : public UnaryEdge {
public:
    EdgeTask(XBot::ModelInterface::Ptr model);

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        return true;
    }

    void setReference(Eigen::VectorXd ref);
    void setEndEffectors(std::vector<std::string> ee);

    bool resize();

    Eigen::VectorXd getError() const;

    Eigen::VectorXd getReference() const;

    void computeError();

private:
    Eigen::VectorXd _ref;
    std::vector<Eigen::Affine3d> _T_ref;
    std::vector<std::string> _ee;
};

} }
#endif // EDGE_TASK_H
