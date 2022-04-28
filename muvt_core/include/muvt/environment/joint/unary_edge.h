#ifndef UNARY_EDGE_H
#define UNARY_EDGE_H

#include <g2o/core/base_unary_edge.h>

#include <MuvtInterface/ModelInterface.h>

#include <environment/joint/vertex_robot_pos.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

class UnaryEdge : public BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    UnaryEdge(Muvt::ModelInterface::Ptr model);

    virtual bool read(std::istream& is) = 0;
    virtual bool write(std::ostream& os) const = 0;

    virtual void computeError() = 0;

protected:
    Muvt::ModelInterface::Ptr _model;
};

} }

#endif // UNARY_EDGE_H
