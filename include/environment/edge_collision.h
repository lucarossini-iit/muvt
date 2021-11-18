#ifndef EDGE_COLLISION_H
#define EDGE_COLLISION_H

#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/collision_utils.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>

#include <eigen_conversions/eigen_msg.h>

#include <environment/vertex_robot_pos.h>
#include <environment/obstacle.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgeCollision : public BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::vector<obstacle> obstacles;

    EdgeCollision(XBot::ModelInterface::Ptr& model,
                  std::shared_ptr<ComputeLinksDistance> dist,
                  int max_pair_link);

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        Eigen::VectorXd p = measurement();
        os << p;

        return os.good();
    }

    void setObstacles(obstacles obs);
    void resize(int size);
    void computeError();

    Eigen::VectorXd getError() const;

    unsigned int ID;

private:
    XBot::ModelInterface::Ptr _model;
    std::shared_ptr<ComputeLinksDistance> _cld;

}; } }

#endif // EDGE_COLLISION_H
