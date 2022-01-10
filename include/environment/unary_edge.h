#ifndef UNARY_EDGE_H
#define UNARY_EDGE_H

#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>

#include <environment/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class UnaryEdge : public BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    UnaryEdge(XBot::ModelInterface::Ptr model);

    virtual bool read(std::istream& is) = 0;
    virtual bool write(std::ostream& os) const = 0;

    virtual void computeError() = 0;

protected:
    XBot::ModelInterface::Ptr _model;
};

} }

#endif // UNARY_EDGE_H
