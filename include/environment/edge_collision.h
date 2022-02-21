#ifndef EDGE_COLLISION_H
#define EDGE_COLLISION_H

#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/collision_utils.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>

#include <eigen_conversions/eigen_msg.h>

#include <environment/unary_edge.h>
#include <environment/vertex_robot_pos.h>
#include <environment/obstacle.h>
#include <octomap_msgs/OctomapWithPose.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgeCollision : public UnaryEdge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::vector<obstacle> obstacles;

    EdgeCollision(XBot::ModelInterface::Ptr model,
                  std::shared_ptr<ComputeLinksDistance> dist,
                  int max_pair_link);

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        auto v = dynamic_cast<VertexRobotPos*>(_vertices[0]);
        os << "vertex: " << v->estimate().transpose() << std::endl;
        os << "error: " << _error.transpose() << std::endl;

        return os.good();
    }

    void setObstacles(const obstacles obs, const octomap_msgs::OctomapWithPosePtr octomap = nullptr);
    void resize(int size);
    void clear();
    void computeError();

    Eigen::VectorXd getError() const;

    unsigned int ID;

private:
    std::shared_ptr<ComputeLinksDistance> _cld;

}; } }

#endif // EDGE_COLLISION_H
