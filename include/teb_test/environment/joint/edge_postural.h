#ifndef EDGE_POSTURAL_H
#define EDGE_POSTURAL_H

#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>

#include <environment/joint/unary_edge.h>
#include <environment/joint/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgePostural : public UnaryEdge {
public:
    EdgePostural(XBot::ModelInterface::Ptr model);

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        return true;
    }

    void setReference(Eigen::VectorXd ref);

    bool resize();

    Eigen::VectorXd getError() const;

    Eigen::VectorXd getReference() const;

    void computeError();

private:
    Eigen::VectorXd _ref;
};

} }

#endif // EDGE_POSTURAL_H
