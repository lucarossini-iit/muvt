#ifndef MUVT_CORE_EDGE_POSTURAL_H
#define MUVT_CORE_EDGE_POSTURAL_H

#include <g2o/core/base_unary_edge.h>

#include <MuvtInterface/ModelInterface.h>

#include <environment/joint/unary_edge.h>
#include <environment/joint/vertex_robot_pos.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

class EdgePostural : public UnaryEdge {
public:
    EdgePostural(Muvt::ModelInterface::Ptr model);

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

#endif // MUVT_CORE_EDGE_POSTURAL_H
